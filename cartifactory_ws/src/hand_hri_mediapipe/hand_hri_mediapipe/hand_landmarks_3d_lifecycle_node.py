#!/usr/bin/env python3
"""
Lifecycle ROS2 node that projects MediaPipe 2D hand landmarks into 3D space.

The node listens to tracked hand identifiers and dynamically creates one
synchronized processing pipeline per hand. Each hand pipeline synchronizes:
- /humans/hands/<id>/landmarks2d  (HandLandmarks2D)
- aligned depth image             (sensor_msgs/Image)
- camera intrinsics               (sensor_msgs/CameraInfo)

The node publishes:
- /humans/hands/<id>/landmarks3d_marker   (visualization_msgs/Marker, SPHERE_LIST)
- /humans/hands/<id>/skeleton3d_marker    (visualization_msgs/Marker, LINE_LIST)
- /humans/hands/<id>/landmarks3d          (HandLandmarks3D)
- Optional /humans/hands/<id>/landmarks3d_cloud (sensor_msgs/PointCloud2)

Lifecycle behavior:
- Unconfigured:
    Only parameters are declared.
- Inactive:
    The tracked-hands subscriber and cleanup timer are configured, but hand
    pipelines are not created and synchronized data is not processed.
- Active:
    Hand pipelines are created dynamically and 3D landmarks are published.
- Deactivate:
    Processing stops and all dynamic hand pipelines are destroyed.
- Cleanup:
    The tracked-hands subscriber, timer, bridge, and dynamic resources are released.

Author:
    CARTIF Technology Centre
"""

import logging
import math
import threading
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from hand_hri_mediapipe_msgs.msg import HandLandmarks2D, HandLandmarks3D
from hri_msgs.msg import IdsList
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rich.logging import RichHandler
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler(rich_tracebacks=True)],
    force=True,
)
LOG = logging.getLogger("hand_landmarks_3d_lifecycle")


HAND_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (0, 9), (9, 10), (10, 11), (11, 12),
    (0, 13), (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17),
]


def make_cloud_xyz(header: Header, points_xyz: np.ndarray) -> PointCloud2:
    """Create a PointCloud2 message from XYZ points.

    Parameters
    ----------
    header : Header
        Header used by the generated point cloud.
    points_xyz : np.ndarray
        Array of 3D points with shape (N, 3).

    Returns
    -------
    PointCloud2
        Point cloud containing XYZ float32 fields.
    """
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(points_xyz.shape[0])
    msg.is_bigendian = False
    msg.is_dense = True
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width

    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    if msg.width == 0:
        msg.data = b""
        return msg

    points_float32 = points_xyz.astype(np.float32, copy=False)
    msg.data = points_float32.tobytes()
    return msg


class Hand3DProjectorLifecycleNode(LifecycleNode):
    """Lifecycle node that projects normalized 2D hand landmarks into 3D.

    The node uses the depth image and camera intrinsics to deproject each 2D
    landmark into a 3D point in the camera frame. A dynamic hand pipeline is
    created for each active hand identifier received on the tracked-hands topic.

    Parameters
    ----------
    tracked_topic : str
        Topic containing currently tracked hand identifiers.
    depth_topic : str
        Aligned depth image topic.
    caminfo_topic : str
        CameraInfo topic associated with the aligned depth image.
    publish_pointcloud : bool
        Enables publishing a PointCloud2 message with valid 3D landmarks.
    norm_margin : float
        Extra normalized margin accepted around [0, 1] before rejecting a point.
    clamp_norm : bool
        Clamps normalized coordinates to [0, 1] when they are inside the margin.
    min_depth_m : float
        Minimum valid depth in meters.
    max_depth_m : float
        Maximum valid depth in meters.
    depth_hole_fill_radius : int
        Radius in pixels used to search for valid neighboring depth values.
    stale_hand_seconds : float
        Time after which a hand pipeline is removed if no tracked update arrives.
    sphere_diameter : float
        RViz sphere marker diameter in meters.
    line_width : float
        RViz skeleton line width in meters.
    points_alpha : float
        Alpha value used by the 3D landmark marker.
    lines_alpha : float
        Alpha value used by the 3D skeleton marker.
    """

    def __init__(self):
        """Declare parameters and initialize lifecycle-managed state."""
        super().__init__("hand_landmarks_3d_node")

        self.declare_parameter("tracked_topic", "/humans/hands/tracked")
        self.declare_parameter("depth_topic", "/camera/cameraD455/aligned_depth_to_color/image_raw")
        self.declare_parameter("caminfo_topic", "/camera/cameraD455/aligned_depth_to_color/camera_info")

        self.declare_parameter("publish_pointcloud", False)

        self.declare_parameter("norm_margin", 0.05)
        self.declare_parameter("clamp_norm", True)
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 2.0)
        self.declare_parameter("depth_hole_fill_radius", 2)

        self.declare_parameter("stale_hand_seconds", 1.0)

        self.declare_parameter("sphere_diameter", 0.012)
        self.declare_parameter("line_width", 0.004)
        self.declare_parameter("points_alpha", 0.9)
        self.declare_parameter("lines_alpha", 0.85)

        self.active = False
        self.configured = False

        self.tracked_topic = ""
        self.depth_topic = ""
        self.caminfo_topic = ""
        self.publish_pointcloud = False

        self.norm_margin = 0.05
        self.clamp_norm = True
        self.min_depth_m = 0.05
        self.max_depth_m = 2.0
        self.depth_hole_fill_radius = 2
        self.stale_hand_seconds = 1.0

        self.sphere_diameter = 0.012
        self.line_width = 0.004
        self.points_alpha = 0.9
        self.lines_alpha = 0.85

        self.bridge: Optional[CvBridge] = None
        self.sub_tracked = None
        self.cleanup_timer = None

        self._lock = threading.Lock()
        self._hands: Dict[str, dict] = {}

        LOG.info("Hand 3D projector lifecycle node created. Initial state: Unconfigured")

    def on_configure(self, state):
        """Configure the tracked subscriber, cleanup timer, and parameter values.

        This transition allocates the static resources required by the node and
        moves it to Inactive. Dynamic per-hand pipelines are created later only
        when the node is Active.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS if configuration succeeds, otherwise FAILURE.
        """
        _ = state

        try:
            self.tracked_topic = str(self.get_parameter("tracked_topic").value)
            self.depth_topic = str(self.get_parameter("depth_topic").value)
            self.caminfo_topic = str(self.get_parameter("caminfo_topic").value)
            self.publish_pointcloud = bool(self.get_parameter("publish_pointcloud").value)

            self.norm_margin = float(self.get_parameter("norm_margin").value)
            self.clamp_norm = bool(self.get_parameter("clamp_norm").value)
            self.min_depth_m = float(self.get_parameter("min_depth_m").value)
            self.max_depth_m = float(self.get_parameter("max_depth_m").value)
            self.depth_hole_fill_radius = int(self.get_parameter("depth_hole_fill_radius").value)
            self.stale_hand_seconds = float(self.get_parameter("stale_hand_seconds").value)

            self.sphere_diameter = float(self.get_parameter("sphere_diameter").value)
            self.line_width = float(self.get_parameter("line_width").value)
            self.points_alpha = float(self.get_parameter("points_alpha").value)
            self.lines_alpha = float(self.get_parameter("lines_alpha").value)

            self.bridge = CvBridge()
            self.sub_tracked = self.create_subscription(
                IdsList,
                self.tracked_topic,
                self.tracked_callback,
                10,
            )
            self.cleanup_timer = self.create_timer(0.25, self.cleanup_timer_callback)

            self.configured = True

            LOG.info("Hand 3D projector lifecycle node configured.")
            LOG.info(f"  tracked_topic: {self.tracked_topic}")
            LOG.info(f"  depth_topic:   {self.depth_topic}")
            LOG.info(f"  caminfo_topic: {self.caminfo_topic}")
            LOG.info(f"  publish_pointcloud: {self.publish_pointcloud}")

            return TransitionCallbackReturn.SUCCESS

        except Exception as exc:
            LOG.exception(f"Failed to configure Hand 3D projector lifecycle node: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        """Enable dynamic hand pipeline creation and 3D projection.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS when the node is ready to process data.
        """
        _ = state
        self.active = True
        LOG.info("Hand 3D projector lifecycle node activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Disable processing and destroy all dynamic hand pipelines.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS after dynamic pipelines are removed.
        """
        _ = state
        self.active = False
        self.destroy_all_hand_pipelines()
        LOG.info("Hand 3D projector lifecycle node deactivated. Dynamic hand pipelines destroyed.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Release all resources and return to Unconfigured.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS if cleanup succeeds, otherwise FAILURE.
        """
        _ = state
        self.active = False

        try:
            self.destroy_all_hand_pipelines()

            if self.cleanup_timer is not None:
                self.cleanup_timer.cancel()
                self.destroy_timer(self.cleanup_timer)
                self.cleanup_timer = None

            if self.sub_tracked is not None:
                self.destroy_subscription(self.sub_tracked)
                self.sub_tracked = None

            self.bridge = None
            self.configured = False

            LOG.info("Hand 3D projector lifecycle node cleaned up.")
            return TransitionCallbackReturn.SUCCESS

        except Exception as exc:
            LOG.exception(f"Failed to cleanup Hand 3D projector lifecycle node: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state):
        """Stop processing and release dynamic resources during shutdown.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS after best-effort shutdown handling.
        """
        _ = state
        self.active = False
        self.destroy_all_hand_pipelines()
        LOG.info("Hand 3D projector lifecycle node shutting down.")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        """Disable processing and release dynamic pipelines after an error.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS after moving to a safe inactive-like internal state.
        """
        _ = state
        self.active = False
        self.destroy_all_hand_pipelines()
        LOG.error("Hand 3D projector lifecycle node entered error state.")
        return TransitionCallbackReturn.SUCCESS

    def tracked_callback(self, msg: IdsList):
        """Create or update hand pipelines from the tracked hand list.

        Dynamic pipelines are only created while the node is Active.

        Parameters
        ----------
        msg : IdsList
            Message containing currently tracked hand identifiers.

        Returns
        -------
        None
        """
        if not self.active:
            return

        current_time_sec = self.get_clock().now().nanoseconds * 1e-9
        hand_ids = list(msg.ids)

        with self._lock:
            for hand_id in hand_ids:
                if hand_id not in self._hands:
                    self.create_hand_pipeline(hand_id)
                self._hands[hand_id]["last_seen_sec"] = current_time_sec

    def cleanup_timer_callback(self):
        """Remove dynamic hand pipelines that have not been updated recently.

        Returns
        -------
        None
        """
        if not self.active:
            return

        current_time_sec = self.get_clock().now().nanoseconds * 1e-9
        hands_to_remove = []

        with self._lock:
            for hand_id, hand_state in self._hands.items():
                last_seen_sec = float(hand_state.get("last_seen_sec", 0.0))
                if (current_time_sec - last_seen_sec) > self.stale_hand_seconds:
                    hands_to_remove.append(hand_id)

            for hand_id in hands_to_remove:
                self.destroy_hand_pipeline_unlocked(hand_id)

        if hands_to_remove:
            LOG.info(f"Removed stale hands: {hands_to_remove}")

    def create_hand_pipeline(self, hand_id: str):
        """Create synchronized subscribers and publishers for one tracked hand.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand.

        Returns
        -------
        None
        """
        landmarks2d_topic = f"/humans/hands/{hand_id}/landmarks2d"

        pub_points_marker = self.create_publisher(
            Marker,
            f"/humans/hands/{hand_id}/landmarks3d_marker",
            10,
        )
        pub_skeleton_marker = self.create_publisher(
            Marker,
            f"/humans/hands/{hand_id}/skeleton3d_marker",
            10,
        )
        pub_landmarks3d = self.create_publisher(
            HandLandmarks3D,
            f"/humans/hands/{hand_id}/landmarks3d",
            10,
        )

        pub_cloud = None
        if self.publish_pointcloud:
            pub_cloud = self.create_publisher(
                PointCloud2,
                f"/humans/hands/{hand_id}/landmarks3d_cloud",
                10,
            )

        sub_landmarks2d = Subscriber(self, HandLandmarks2D, landmarks2d_topic)
        sub_depth = Subscriber(self, Image, self.depth_topic)
        sub_camera_info = Subscriber(self, CameraInfo, self.caminfo_topic)

        synchronizer = ApproximateTimeSynchronizer(
            [sub_landmarks2d, sub_depth, sub_camera_info],
            queue_size=10,
            slop=0.12,
        )
        synchronizer.registerCallback(
            lambda landmarks2d_msg, depth_msg, camera_info_msg: self.synchronized_callback(
                hand_id,
                landmarks2d_msg,
                depth_msg,
                camera_info_msg,
            )
        )

        self._hands[hand_id] = {
            "sub_landmarks2d": sub_landmarks2d,
            "sub_depth": sub_depth,
            "sub_camera_info": sub_camera_info,
            "synchronizer": synchronizer,
            "pub_landmarks3d": pub_landmarks3d,
            "pub_points_marker": pub_points_marker,
            "pub_skeleton_marker": pub_skeleton_marker,
            "pub_cloud": pub_cloud,
            "last_seen_sec": self.get_clock().now().nanoseconds * 1e-9,
            "landmarks2d_topic": landmarks2d_topic,
        }

        LOG.info(f"[{hand_id}] lifecycle pipeline created. Sub: {landmarks2d_topic}")

    def destroy_hand_pipeline_unlocked(self, hand_id: str):
        """Destroy one dynamic hand pipeline.

        This method assumes that the caller already holds self._lock.

        Parameters
        ----------
        hand_id : str
            Identifier of the hand pipeline to destroy.

        Returns
        -------
        None
        """
        hand_state = self._hands.get(hand_id)
        if hand_state is None:
            return

        for key in ["sub_landmarks2d", "sub_depth", "sub_camera_info"]:
            try:
                subscriber = hand_state[key]
                if hasattr(subscriber, "sub") and subscriber.sub is not None:
                    self.destroy_subscription(subscriber.sub)
            except Exception:
                pass

        for key in ["pub_landmarks3d", "pub_points_marker", "pub_skeleton_marker", "pub_cloud"]:
            try:
                publisher = hand_state.get(key)
                if publisher is not None:
                    self.destroy_publisher(publisher)
            except Exception:
                pass

        del self._hands[hand_id]
        LOG.info(f"[{hand_id}] lifecycle pipeline destroyed.")

    def destroy_all_hand_pipelines(self):
        """Destroy all dynamic hand pipelines.

        Returns
        -------
        None
        """
        with self._lock:
            for hand_id in list(self._hands.keys()):
                self.destroy_hand_pipeline_unlocked(hand_id)
            self._hands.clear()

    def normalized_to_pixel(
        self,
        x_normalized: float,
        y_normalized: float,
        width: int,
        height: int,
    ) -> Optional[Tuple[int, int]]:
        """Convert normalized image coordinates into pixel coordinates.

        Parameters
        ----------
        x_normalized : float
            Normalized horizontal coordinate.
        y_normalized : float
            Normalized vertical coordinate.
        width : int
            Image width in pixels.
        height : int
            Image height in pixels.

        Returns
        -------
        Optional[Tuple[int, int]]
            Pixel coordinates (u, v), or None when the point is outside the
            accepted normalized range.
        """
        if not (
            -self.norm_margin <= x_normalized <= 1.0 + self.norm_margin
            and -self.norm_margin <= y_normalized <= 1.0 + self.norm_margin
        ):
            return None

        if self.clamp_norm:
            x_normalized = min(1.0, max(0.0, float(x_normalized)))
            y_normalized = min(1.0, max(0.0, float(y_normalized)))

        u = int(round(x_normalized * (width - 1)))
        v = int(round(y_normalized * (height - 1)))

        if u < 0 or u >= width or v < 0 or v >= height:
            return None

        return u, v

    def depth_at(self, depth_image: np.ndarray, u: int, v: int) -> float:
        """Read a valid depth value in meters at a given pixel.

        If the direct depth value is invalid, a local neighborhood can be used
        to find a robust median depth value.

        Parameters
        ----------
        depth_image : np.ndarray
            Depth image as a NumPy array.
        u : int
            Horizontal pixel coordinate.
        v : int
            Vertical pixel coordinate.

        Returns
        -------
        float
            Depth in meters, or NaN when no valid value is available.
        """
        height, width = depth_image.shape[:2]

        def read_raw(x: int, y: int) -> float:
            value = depth_image[y, x]
            if depth_image.dtype == np.uint16:
                return float(value) * 0.001
            return float(value)

        z = read_raw(u, v)
        if (not math.isfinite(z)) or z <= 0.0:
            radius = max(0, int(self.depth_hole_fill_radius))
            if radius <= 0:
                return float("nan")

            x0 = max(0, u - radius)
            x1 = min(width - 1, u + radius)
            y0 = max(0, v - radius)
            y1 = min(height - 1, v + radius)

            values = []
            for yy in range(y0, y1 + 1):
                for xx in range(x0, x1 + 1):
                    neighbor_depth = read_raw(xx, yy)
                    if math.isfinite(neighbor_depth) and neighbor_depth > 0.0:
                        values.append(neighbor_depth)

            if not values:
                return float("nan")

            z = float(np.median(np.array(values, dtype=np.float32)))

        if (not math.isfinite(z)) or z < self.min_depth_m or z > self.max_depth_m:
            return float("nan")

        return z

    def deproject_pixel(self, u: int, v: int, z: float, camera_info: CameraInfo) -> np.ndarray:
        """Deproject a depth pixel into a 3D camera-frame point.

        Parameters
        ----------
        u : int
            Horizontal pixel coordinate.
        v : int
            Vertical pixel coordinate.
        z : float
            Depth value in meters.
        camera_info : CameraInfo
            Camera intrinsic parameters.

        Returns
        -------
        np.ndarray
            3D point [x, y, z] in the camera frame.
        """
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])

        x = (float(u) - cx) / fx * z
        y = (float(v) - cy) / fy * z
        return np.array([x, y, z], dtype=np.float64)

    def synchronized_callback(
        self,
        hand_id: str,
        landmarks2d_msg: HandLandmarks2D,
        depth_msg: Image,
        camera_info_msg: CameraInfo,
    ):
        """Project synchronized 2D landmarks, depth, and camera info into 3D.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand.
        landmarks2d_msg : HandLandmarks2D
            Normalized 2D landmarks.
        depth_msg : Image
            Aligned depth image.
        camera_info_msg : CameraInfo
            Camera intrinsic parameters.

        Returns
        -------
        None
        """
        if not self.active:
            return

        if self.bridge is None:
            return

        with self._lock:
            if hand_id not in self._hands:
                return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=depth_msg.encoding)
        except Exception as exc:
            LOG.warning(f"[{hand_id}] depth image conversion failed: {exc}")
            return

        if depth_image is None or depth_image.ndim < 2:
            return

        height, width = depth_image.shape[:2]
        points3d: List[Optional[np.ndarray]] = [None] * 21

        for landmark_index in range(21):
            x_normalized = float(landmarks2d_msg.landmarks[landmark_index].x)
            y_normalized = float(landmarks2d_msg.landmarks[landmark_index].y)

            pixel = self.normalized_to_pixel(x_normalized, y_normalized, width, height)
            if pixel is None:
                continue

            u, v = pixel
            depth_m = self.depth_at(depth_image, u, v)
            if not math.isfinite(depth_m):
                continue

            points3d[landmark_index] = self.deproject_pixel(u, v, depth_m, camera_info_msg)

        header = Header()
        header.stamp = landmarks2d_msg.header.stamp
        header.frame_id = depth_msg.header.frame_id if depth_msg.header.frame_id else landmarks2d_msg.header.frame_id

        self.publish_markers(hand_id, header, points3d)
        self.publish_landmarks3d(hand_id, header, landmarks2d_msg, points3d)
        self.publish_cloud_if_enabled(hand_id, header, points3d)

    def publish_landmarks3d(
        self,
        hand_id: str,
        header: Header,
        landmarks2d_msg: HandLandmarks2D,
        points3d: List[Optional[np.ndarray]],
    ):
        """Publish projected 3D hand landmarks.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand.
        header : Header
            Output message header.
        landmarks2d_msg : HandLandmarks2D
            Source 2D landmarks message.
        points3d : List[Optional[np.ndarray]]
            Projected 3D landmarks. Invalid landmarks are represented as None.

        Returns
        -------
        None
        """
        landmarks3d_msg = HandLandmarks3D()
        landmarks3d_msg.header = header
        landmarks3d_msg.id = landmarks2d_msg.id
        landmarks3d_msg.handedness = landmarks2d_msg.handedness
        landmarks3d_msg.confidence = float(getattr(landmarks2d_msg, "confidence", 0.0))

        for landmark_index in range(21):
            point3d = points3d[landmark_index]
            if point3d is None:
                landmarks3d_msg.valid[landmark_index] = False
                landmarks3d_msg.landmarks[landmark_index].x = float("nan")
                landmarks3d_msg.landmarks[landmark_index].y = float("nan")
                landmarks3d_msg.landmarks[landmark_index].z = float("nan")
            else:
                landmarks3d_msg.valid[landmark_index] = True
                landmarks3d_msg.landmarks[landmark_index].x = float(point3d[0])
                landmarks3d_msg.landmarks[landmark_index].y = float(point3d[1])
                landmarks3d_msg.landmarks[landmark_index].z = float(point3d[2])

        with self._lock:
            publisher = self._hands.get(hand_id, {}).get("pub_landmarks3d")

        if publisher is not None:
            publisher.publish(landmarks3d_msg)

    def publish_cloud_if_enabled(
        self,
        hand_id: str,
        header: Header,
        points3d: List[Optional[np.ndarray]],
    ):
        """Publish an optional point cloud with valid 3D landmarks.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand.
        header : Header
            Output message header.
        points3d : List[Optional[np.ndarray]]
            Projected 3D landmarks.

        Returns
        -------
        None
        """
        with self._lock:
            publisher = self._hands.get(hand_id, {}).get("pub_cloud")

        if publisher is None:
            return

        valid_points = np.array([point for point in points3d if point is not None], dtype=np.float64)
        publisher.publish(make_cloud_xyz(header, valid_points))

    def publish_markers(
        self,
        hand_id: str,
        header: Header,
        points3d: List[Optional[np.ndarray]],
    ):
        """Publish RViz markers for valid 3D landmarks and skeleton edges.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand.
        header : Header
            Marker header.
        points3d : List[Optional[np.ndarray]]
            Projected 3D landmarks.

        Returns
        -------
        None
        """
        with self._lock:
            hand_state = self._hands.get(hand_id)
            if hand_state is None:
                return
            pub_points_marker = hand_state["pub_points_marker"]
            pub_skeleton_marker = hand_state["pub_skeleton_marker"]

        points_marker = Marker()
        points_marker.header = header
        points_marker.ns = f"{hand_id}_landmarks"
        points_marker.id = 0
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        points_marker.pose.orientation.w = 1.0
        points_marker.scale.x = self.sphere_diameter
        points_marker.scale.y = self.sphere_diameter
        points_marker.scale.z = self.sphere_diameter
        points_marker.color = ColorRGBA(r=0.0, g=0.2, b=1.0, a=float(self.points_alpha))
        points_marker.points = [
            Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            for point in points3d
            if point is not None
        ]
        pub_points_marker.publish(points_marker)

        skeleton_marker = Marker()
        skeleton_marker.header = header
        skeleton_marker.ns = f"{hand_id}_skeleton"
        skeleton_marker.id = 0
        skeleton_marker.type = Marker.LINE_LIST
        skeleton_marker.action = Marker.ADD
        skeleton_marker.pose.orientation.w = 1.0
        skeleton_marker.scale.x = self.line_width
        skeleton_marker.color = ColorRGBA(r=0.2, g=1.0, b=1.0, a=float(self.lines_alpha))

        line_points = []
        for start_index, end_index in HAND_EDGES:
            start_point = points3d[start_index]
            end_point = points3d[end_index]
            if start_point is None or end_point is None:
                continue
            line_points.append(Point(x=float(start_point[0]), y=float(start_point[1]), z=float(start_point[2])))
            line_points.append(Point(x=float(end_point[0]), y=float(end_point[1]), z=float(end_point[2])))

        skeleton_marker.points = line_points
        pub_skeleton_marker.publish(skeleton_marker)


def main(args=None):
    """Run the 3D hand projector lifecycle node."""
    rclpy.init(args=args)
    node = Hand3DProjectorLifecycleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
