#!/usr/bin/env python3
"""
Lifecycle ROS2 node for transforming hand landmarks and palm plane estimations
from the camera reference frame into the robot base reference frame.

The node uses the calibrated transform from robot TCP to camera and the current
TCP pose to compute the transform from camera coordinates to the robot base.

It subscribes to:
- /humans/hands/tracked
- /ur/tcp_pose
- /humans/hands/<id>/landmarks3d
- /humans/hands/<id>/palm_plane_cam

and publishes:
- /humans/hands/<id>/landmarks3d_base
- /humans/hands/<id>/palm_plane_base
- Optional RViz markers for transformed palm centroid and normal

Lifecycle behavior:
- Unconfigured:
    Only parameters are declared.
- Inactive:
    Static subscribers are configured. The TCP pose can be cached, but dynamic
    per-hand pipelines are not created and no transformed outputs are published.
- Active:
    Dynamic hand pipelines are created from tracked IDs and transformed outputs
    are published.
- Deactivate:
    Processing stops and all dynamic hand pipelines are destroyed.
- Cleanup:
    Static subscriptions, timers, dynamic resources, and internal state are
    released.

Author:
    CARTIF Technology Centre
"""

import logging
import time
from typing import Optional

import numpy as np
import rclpy

from geometry_msgs.msg import Point, PoseStamped, Vector3
from hand_hri_mediapipe_msgs.msg import HandLandmarks3D, PlaneStamped
from hri_msgs.msg import IdsList
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from scipy.spatial.transform import Rotation as R
from rich.logging import RichHandler
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler(rich_tracebacks=True)],
    force=True,
)
LOG = logging.getLogger("hand_cam_to_base_lifecycle")


def normalize(vector: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    """Normalize a vector while avoiding division by zero.

    Parameters
    ----------
    vector : np.ndarray
        Input vector.
    eps : float, optional
        Minimum norm accepted as non-degenerate, by default 1e-12.

    Returns
    -------
    np.ndarray
        Unit vector, or the input vector if its norm is too small.
    """
    vector = np.asarray(vector, dtype=float).reshape(3)
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > eps else vector


def clamp_angle_step(previous_normal: np.ndarray, new_normal: np.ndarray, max_deg: float) -> np.ndarray:
    """Limit the angular change between two normal vectors.

    Parameters
    ----------
    previous_normal : np.ndarray
        Previous filtered normal.
    new_normal : np.ndarray
        New raw normal.
    max_deg : float
        Maximum angular step in degrees.

    Returns
    -------
    np.ndarray
        Normal vector with limited angular change.
    """
    previous_normal = normalize(previous_normal)
    new_normal = normalize(new_normal)

    dot_value = float(np.clip(np.dot(previous_normal, new_normal), -1.0, 1.0))
    angle_rad = float(np.arccos(dot_value))
    max_rad = float(np.deg2rad(max_deg))

    if angle_rad <= max_rad or angle_rad < 1e-9:
        return new_normal

    interpolation = max_rad / angle_rad
    sin_angle = float(np.sin(angle_rad))
    if abs(sin_angle) < 1e-12:
        return new_normal

    weight_previous = float(np.sin((1.0 - interpolation) * angle_rad) / sin_angle)
    weight_new = float(np.sin(interpolation * angle_rad) / sin_angle)

    return normalize(weight_previous * previous_normal + weight_new * new_normal)


def make_centroid_marker(frame_id: str, stamp, position: np.ndarray, namespace: str, marker_id: int = 0) -> Marker:
    """Create an RViz sphere marker for a transformed palm centroid.

    Parameters
    ----------
    frame_id : str
        Reference frame for the marker.
    stamp
        ROS timestamp.
    position : np.ndarray
        3D centroid position.
    namespace : str
        Marker namespace.
    marker_id : int, optional
        Marker identifier, by default 0.

    Returns
    -------
    Marker
        RViz sphere marker.
    """
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = float(position[0])
    marker.pose.position.y = float(position[1])
    marker.pose.position.z = float(position[2])
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    marker.color = ColorRGBA(r=0.0, g=0.6, b=1.0, a=0.9)
    return marker


def make_normal_marker(frame_id: str, stamp, position: np.ndarray, normal: np.ndarray, namespace: str, marker_id: int = 0) -> Marker:
    """Create an RViz arrow marker for a transformed palm normal.

    Parameters
    ----------
    frame_id : str
        Reference frame for the marker.
    stamp
        ROS timestamp.
    position : np.ndarray
        Arrow start point.
    normal : np.ndarray
        Arrow direction.
    namespace : str
        Marker namespace.
    marker_id : int, optional
        Marker identifier, by default 0.

    Returns
    -------
    Marker
        RViz arrow marker.
    """
    start = np.asarray(position, dtype=float).reshape(3)
    end = start + 0.10 * normalize(normal)

    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.01
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color = ColorRGBA(r=1.0, g=0.75, b=0.0, a=0.95)
    marker.points = [
        Point(x=float(start[0]), y=float(start[1]), z=float(start[2])),
        Point(x=float(end[0]), y=float(end[1]), z=float(end[2])),
    ]
    return marker


class HandCamToBaseLifecycleNode(LifecycleNode):
    """Lifecycle node that transforms hand data from camera frame to base frame.

    Parameters
    ----------
    tracked_topic : str
        Topic containing currently tracked hand identifiers.
    tcp_pose_topic : str
        Topic containing the current TCP pose in the robot base frame.
    frame_out : str
        Output frame identifier.
    landmarks3d_suffix_in : str
        Per-hand input suffix for camera-frame 3D landmarks.
    landmarks3d_suffix_out : str
        Per-hand output suffix for base-frame 3D landmarks.
    plane_suffix_in : str
        Per-hand input suffix for camera-frame palm plane.
    plane_suffix_out : str
        Per-hand output suffix for base-frame palm plane.
    T_tcp_cam : list[float]
        Homogeneous transform from TCP frame to camera frame, row-major 4x4.
    filter_plane_normal : bool
        Enables exponential filtering of the transformed palm normal.
    filter_alpha : float
        EMA alpha used for normal filtering.
    limit_step_enable : bool
        Enables angular step limiting for the transformed normal.
    max_step_deg : float
        Maximum angular step in degrees.
    flip_guard_enable : bool
        Prevents normal sign flips between consecutive estimates.
    stale_hand_seconds : float
        Timeout used to remove stale dynamic hand pipelines.
    publish_markers : bool
        Enables RViz marker publication.
    """

    def __init__(self):
        """Declare parameters and initialize lifecycle-managed state."""
        super().__init__("hand_cam_to_base_node")

        self.declare_parameter("tracked_topic", "/humans/hands/tracked")
        self.declare_parameter("tcp_pose_topic", "/ur/tcp_pose")
        self.declare_parameter("frame_out", "base")
        self.declare_parameter("landmarks3d_suffix_in", "landmarks3d")
        self.declare_parameter("landmarks3d_suffix_out", "landmarks3d_base")
        self.declare_parameter("plane_suffix_in", "palm_plane_cam")
        self.declare_parameter("plane_suffix_out", "palm_plane_base")
        self.declare_parameter(
            "T_tcp_cam",
            [
                1.000, -0.005, -0.005, -0.046,
                0.005,  1.000,  0.001, -0.064,
                0.005, -0.001,  1.000,  0.124,
                0.000,  0.000,  0.000,  1.000,
            ],
        )
        self.declare_parameter("filter_plane_normal", True)
        self.declare_parameter("filter_alpha", 0.15)
        self.declare_parameter("limit_step_enable", True)
        self.declare_parameter("max_step_deg", 2.0)
        self.declare_parameter("flip_guard_enable", True)
        self.declare_parameter("stale_hand_seconds", 1.0)
        self.declare_parameter("publish_markers", True)

        self.active = False
        self.tracked_topic = ""
        self.tcp_pose_topic = ""
        self.frame_out = "base"
        self.landmarks3d_suffix_in = "landmarks3d"
        self.landmarks3d_suffix_out = "landmarks3d_base"
        self.plane_suffix_in = "palm_plane_cam"
        self.plane_suffix_out = "palm_plane_base"

        self.filter_plane_normal = True
        self.filter_alpha = 0.15
        self.limit_step_enable = True
        self.max_step_deg = 2.0
        self.flip_guard_enable = True
        self.stale_hand_seconds = 1.0
        self.publish_markers = True

        self.T_tcp_cam = np.eye(4, dtype=float)
        self.R_tcp_cam = np.eye(3, dtype=float)
        self.t_tcp_cam = np.zeros(3, dtype=float)

        self.latest_tcp_pose: Optional[PoseStamped] = None
        self.sub_tracked = None
        self.sub_tcp_pose = None
        self.cleanup_timer = None
        self._hands = {}

        LOG.info("Hand camera-to-base lifecycle node created. Initial state: Unconfigured")

    def on_configure(self, state):
        """Read parameters and create static lifecycle resources."""
        _ = state

        try:
            self.tracked_topic = str(self.get_parameter("tracked_topic").value)
            self.tcp_pose_topic = str(self.get_parameter("tcp_pose_topic").value)
            self.frame_out = str(self.get_parameter("frame_out").value)
            self.landmarks3d_suffix_in = str(self.get_parameter("landmarks3d_suffix_in").value)
            self.landmarks3d_suffix_out = str(self.get_parameter("landmarks3d_suffix_out").value)
            self.plane_suffix_in = str(self.get_parameter("plane_suffix_in").value)
            self.plane_suffix_out = str(self.get_parameter("plane_suffix_out").value)

            self.filter_plane_normal = bool(self.get_parameter("filter_plane_normal").value)
            self.filter_alpha = float(self.get_parameter("filter_alpha").value)
            self.limit_step_enable = bool(self.get_parameter("limit_step_enable").value)
            self.max_step_deg = float(self.get_parameter("max_step_deg").value)
            self.flip_guard_enable = bool(self.get_parameter("flip_guard_enable").value)
            self.stale_hand_seconds = float(self.get_parameter("stale_hand_seconds").value)
            self.publish_markers = bool(self.get_parameter("publish_markers").value)

            transform_values = self.get_parameter("T_tcp_cam").value
            self.T_tcp_cam = np.array(transform_values, dtype=float).reshape((4, 4))
            self.R_tcp_cam = self.T_tcp_cam[:3, :3]
            self.t_tcp_cam = self.T_tcp_cam[:3, 3]

            self.sub_tcp_pose = self.create_subscription(
                PoseStamped,
                self.tcp_pose_topic,
                self.tcp_pose_callback,
                10,
            )
            self.sub_tracked = self.create_subscription(
                IdsList,
                self.tracked_topic,
                self.tracked_callback,
                10,
            )
            self.cleanup_timer = self.create_timer(0.25, self.cleanup_timer_callback)

            LOG.info("Hand camera-to-base lifecycle node configured.")
            LOG.info(f"  tcp_pose_topic: {self.tcp_pose_topic}")
            LOG.info(f"  tracked_topic: {self.tracked_topic}")
            LOG.info(f"  output_frame: {self.frame_out}")

            return TransitionCallbackReturn.SUCCESS

        except Exception as exc:
            LOG.exception(f"Failed to configure hand camera-to-base lifecycle node: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        """Enable dynamic hand pipelines and output publication."""
        _ = state
        self.active = True
        LOG.info("Hand camera-to-base lifecycle node activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Disable processing and destroy all dynamic hand pipelines."""
        _ = state
        self.active = False
        self.destroy_all_hand_pipelines()
        LOG.info("Hand camera-to-base lifecycle node deactivated. Dynamic hand pipelines destroyed.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Release static and dynamic resources."""
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

            if self.sub_tcp_pose is not None:
                self.destroy_subscription(self.sub_tcp_pose)
                self.sub_tcp_pose = None

            self.latest_tcp_pose = None
            self._hands.clear()

            LOG.info("Hand camera-to-base lifecycle node cleaned up.")
            return TransitionCallbackReturn.SUCCESS

        except Exception as exc:
            LOG.exception(f"Failed to cleanup hand camera-to-base lifecycle node: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state):
        """Stop processing and release dynamic resources during shutdown."""
        _ = state
        self.active = False
        self.destroy_all_hand_pipelines()
        LOG.info("Hand camera-to-base lifecycle node shutting down.")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        """Move the node to a safe internal state after an error."""
        _ = state
        self.active = False
        self.destroy_all_hand_pipelines()
        LOG.error("Hand camera-to-base lifecycle node entered error state.")
        return TransitionCallbackReturn.SUCCESS

    def tcp_pose_callback(self, msg: PoseStamped):
        """Store the latest TCP pose.

        TCP pose caching is allowed while inactive so the node can immediately
        transform data after activation.

        Parameters
        ----------
        msg : PoseStamped
            Current TCP pose in the robot base frame.
        """
        self.latest_tcp_pose = msg

    def tracked_callback(self, msg: IdsList):
        """Create or update dynamic hand pipelines from tracked IDs."""
        if not self.active:
            return

        current_time = time.time()
        for hand_id in msg.ids:
            if hand_id not in self._hands:
                self.create_hand_pipeline(hand_id)
            self._hands[hand_id]["last_seen"] = current_time

    def create_hand_pipeline(self, hand_id: str):
        """Create subscriptions and publishers for one tracked hand."""
        landmarks_topic_in = f"/humans/hands/{hand_id}/{self.landmarks3d_suffix_in}"
        landmarks_topic_out = f"/humans/hands/{hand_id}/{self.landmarks3d_suffix_out}"
        plane_topic_in = f"/humans/hands/{hand_id}/{self.plane_suffix_in}"
        plane_topic_out = f"/humans/hands/{hand_id}/{self.plane_suffix_out}"

        sub_landmarks = self.create_subscription(
            HandLandmarks3D,
            landmarks_topic_in,
            lambda msg, current_hand_id=hand_id: self.landmarks_callback(current_hand_id, msg),
            10,
        )
        sub_plane = self.create_subscription(
            PlaneStamped,
            plane_topic_in,
            lambda msg, current_hand_id=hand_id: self.plane_callback(current_hand_id, msg),
            10,
        )

        pub_landmarks = self.create_publisher(HandLandmarks3D, landmarks_topic_out, 10)
        pub_plane = self.create_publisher(PlaneStamped, plane_topic_out, 10)

        pub_centroid = None
        pub_normal = None
        if self.publish_markers:
            pub_centroid = self.create_publisher(
                Marker,
                f"/humans/hands/{hand_id}/palm_plane_centroid_marker_base",
                10,
            )
            pub_normal = self.create_publisher(
                Marker,
                f"/humans/hands/{hand_id}/palm_plane_normal_marker_base",
                10,
            )

        self._hands[hand_id] = {
            "sub_landmarks": sub_landmarks,
            "sub_plane": sub_plane,
            "pub_landmarks": pub_landmarks,
            "pub_plane": pub_plane,
            "pub_centroid": pub_centroid,
            "pub_normal": pub_normal,
            "last_seen": time.time(),
            "filtered_base_normal": None,
        }

        LOG.info(f"[{hand_id}] camera-to-base pipeline created.")
        LOG.info(f"    {landmarks_topic_in} -> {landmarks_topic_out}")
        LOG.info(f"    {plane_topic_in} -> {plane_topic_out}")

    def destroy_hand_pipeline(self, hand_id: str):
        """Destroy dynamic resources associated with one hand."""
        hand_state = self._hands.get(hand_id)
        if hand_state is None:
            return

        for key in ["sub_landmarks", "sub_plane"]:
            subscription = hand_state.get(key)
            if subscription is not None:
                self.destroy_subscription(subscription)

        for key in ["pub_landmarks", "pub_plane", "pub_centroid", "pub_normal"]:
            publisher = hand_state.get(key)
            if publisher is not None:
                self.destroy_publisher(publisher)

        del self._hands[hand_id]
        LOG.info(f"[{hand_id}] camera-to-base pipeline destroyed.")

    def destroy_all_hand_pipelines(self):
        """Destroy all dynamic hand pipelines."""
        for hand_id in list(self._hands.keys()):
            self.destroy_hand_pipeline(hand_id)
        self._hands.clear()

    def cleanup_timer_callback(self):
        """Remove dynamic hand pipelines that have not been updated recently."""
        if not self.active:
            return

        current_time = time.time()
        stale_hands = []
        for hand_id, hand_state in self._hands.items():
            if current_time - hand_state["last_seen"] > self.stale_hand_seconds:
                stale_hands.append(hand_id)

        for hand_id in stale_hands:
            self.destroy_hand_pipeline(hand_id)

    def get_base_to_camera_transform(self):
        """Compute the transform from camera frame to base frame.

        Returns
        -------
        tuple[np.ndarray | None, np.ndarray | None]
            Rotation matrix and translation vector from camera to base, or
            (None, None) if no TCP pose has been received.
        """
        if self.latest_tcp_pose is None:
            return None, None

        tcp_pose = self.latest_tcp_pose
        orientation = tcp_pose.pose.orientation

        rotation_base_tcp = R.from_quat(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        ).as_matrix()
        translation_base_tcp = np.array(
            [
                tcp_pose.pose.position.x,
                tcp_pose.pose.position.y,
                tcp_pose.pose.position.z,
            ],
            dtype=float,
        )

        rotation_base_camera = rotation_base_tcp @ self.R_tcp_cam
        translation_base_camera = rotation_base_tcp @ self.t_tcp_cam + translation_base_tcp

        return rotation_base_camera, translation_base_camera

    def transform_point(self, point_camera: np.ndarray, rotation_base_camera: np.ndarray, translation_base_camera: np.ndarray):
        """Transform a point from camera coordinates to base coordinates."""
        return rotation_base_camera @ point_camera + translation_base_camera

    def update_filtered_normal(self, hand_id: str, raw_base_normal: np.ndarray) -> np.ndarray:
        """Update the filtered base-frame normal for one hand."""
        hand_state = self._hands.get(hand_id)
        if hand_state is None:
            return normalize(raw_base_normal)

        base_normal = normalize(raw_base_normal)

        if hand_state["filtered_base_normal"] is None:
            hand_state["filtered_base_normal"] = base_normal.copy()
            return hand_state["filtered_base_normal"]

        normal_to_use = base_normal.copy()

        if self.flip_guard_enable and float(np.dot(hand_state["filtered_base_normal"], normal_to_use)) < 0.0:
            normal_to_use = -normal_to_use

        if self.limit_step_enable and self.max_step_deg > 0.0:
            normal_to_use = clamp_angle_step(
                hand_state["filtered_base_normal"],
                normal_to_use,
                self.max_step_deg,
            )

        if self.filter_plane_normal:
            alpha = float(np.clip(self.filter_alpha, 0.0, 1.0))
            hand_state["filtered_base_normal"] = normalize(
                alpha * normal_to_use + (1.0 - alpha) * hand_state["filtered_base_normal"]
            )
        else:
            hand_state["filtered_base_normal"] = normal_to_use

        return hand_state["filtered_base_normal"]

    def landmarks_callback(self, hand_id: str, msg: HandLandmarks3D):
        """Transform 3D hand landmarks from camera frame to base frame."""
        if not self.active:
            return

        rotation_base_camera, translation_base_camera = self.get_base_to_camera_transform()
        if rotation_base_camera is None or translation_base_camera is None:
            return

        hand_state = self._hands.get(hand_id)
        if hand_state is None:
            return

        output = HandLandmarks3D()
        output.header = Header()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.frame_out
        output.id = msg.id
        output.handedness = msg.handedness
        output.confidence = msg.confidence

        for index in range(21):
            valid = bool(msg.valid[index])
            output.valid[index] = valid

            if not valid:
                output.landmarks[index].x = float("nan")
                output.landmarks[index].y = float("nan")
                output.landmarks[index].z = float("nan")
                continue

            point = msg.landmarks[index]
            point_camera = np.array([point.x, point.y, point.z], dtype=float)

            if not np.all(np.isfinite(point_camera)):
                output.valid[index] = False
                output.landmarks[index].x = float("nan")
                output.landmarks[index].y = float("nan")
                output.landmarks[index].z = float("nan")
                continue

            point_base = self.transform_point(point_camera, rotation_base_camera, translation_base_camera)
            output.landmarks[index].x = float(point_base[0])
            output.landmarks[index].y = float(point_base[1])
            output.landmarks[index].z = float(point_base[2])

        hand_state["pub_landmarks"].publish(output)

    def plane_callback(self, hand_id: str, msg: PlaneStamped):
        """Transform a palm plane from camera frame to base frame."""
        if not self.active:
            return

        rotation_base_camera, translation_base_camera = self.get_base_to_camera_transform()
        if rotation_base_camera is None or translation_base_camera is None:
            return

        hand_state = self._hands.get(hand_id)
        if hand_state is None:
            return

        normal_camera = np.array([msg.normal.x, msg.normal.y, msg.normal.z], dtype=float)
        centroid_camera = np.array([msg.centroid.x, msg.centroid.y, msg.centroid.z], dtype=float)

        if not np.all(np.isfinite(normal_camera)) or not np.all(np.isfinite(centroid_camera)):
            return

        raw_base_normal = normalize(rotation_base_camera @ normal_camera)
        base_normal = self.update_filtered_normal(hand_id, raw_base_normal)
        centroid_base = self.transform_point(centroid_camera, rotation_base_camera, translation_base_camera)
        d_base = -float(np.dot(base_normal, centroid_base))

        output = PlaneStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.frame_out
        output.normal = Vector3(x=float(base_normal[0]), y=float(base_normal[1]), z=float(base_normal[2]))
        output.d = float(d_base)
        output.centroid = Point(
            x=float(centroid_base[0]),
            y=float(centroid_base[1]),
            z=float(centroid_base[2]),
        )
        output.rmse_m = msg.rmse_m
        output.rmse_mm = msg.rmse_mm
        output.n_inliers = msg.n_inliers
        output.fit_method = msg.fit_method + "|cam_to_base"
        output.accepted = bool(msg.accepted)
        output.reject_reasons = list(msg.reject_reasons)

        hand_state["pub_plane"].publish(output)

        if self.publish_markers:
            self.publish_plane_markers(hand_id, hand_state, msg.header.stamp, centroid_base, base_normal)

    def publish_plane_markers(self, hand_id: str, hand_state: dict, stamp, centroid_base: np.ndarray, base_normal: np.ndarray):
        """Publish transformed centroid and normal markers."""
        if hand_state["pub_centroid"] is not None:
            hand_state["pub_centroid"].publish(
                make_centroid_marker(
                    frame_id=self.frame_out,
                    stamp=stamp,
                    position=centroid_base,
                    namespace=f"{hand_id}_palm_plane_centroid_base",
                )
            )

        if hand_state["pub_normal"] is not None:
            hand_state["pub_normal"].publish(
                make_normal_marker(
                    frame_id=self.frame_out,
                    stamp=stamp,
                    position=centroid_base,
                    normal=base_normal,
                    namespace=f"{hand_id}_palm_plane_normal_base",
                )
            )


def main(args=None):
    """Run the hand camera-to-base lifecycle node."""
    rclpy.init(args=args)
    node = HandCamToBaseLifecycleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
