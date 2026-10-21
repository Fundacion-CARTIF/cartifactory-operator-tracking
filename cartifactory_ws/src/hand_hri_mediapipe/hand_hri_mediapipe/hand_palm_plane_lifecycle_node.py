#!/usr/bin/env python3
"""
Lifecycle ROS2 node for estimating stable palm planes from 3D hand landmarks.

This node subscribes to tracked hand identifiers and dynamically creates one
processing pipeline per hand. For each active hand, it subscribes to:

- /humans/hands/<id>/landmarks3d

and publishes:

- /humans/hands/<id>/palm_plane_cam
- /humans/hands/<id>/palm_plane_centroid_marker
- /humans/hands/<id>/palm_plane_normal_marker
- /humans/hands/<id>/palm_plane_mesh_marker

Lifecycle behavior:
- Unconfigured:
    Only parameters are declared. No subscriptions, timers, or publishers exist.
- Inactive:
    The tracked-hands subscriber and cleanup timer are configured, but hand
    pipelines are not created and 3D landmarks are not processed.
- Active:
    Dynamic hand pipelines are created from tracked IDs and palm planes are
    estimated and published.
- Deactivate:
    Processing stops and all dynamic hand pipelines are destroyed.
- Cleanup:
    Static subscriptions, timers, dynamic publishers/subscribers, and internal
    state are released.

Author:
    CARTIF Technology Centre
"""

import logging
import time
from typing import Optional

import cv2
import numpy as np
import rclpy

from geometry_msgs.msg import Point, Vector3
from geometry_msgs.msg import Point as GeoPoint
from hand_hri_mediapipe_msgs.msg import HandLandmarks3D, PlaneStamped
from hri_msgs.msg import IdsList
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
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
LOG = logging.getLogger("hand_palm_plane_lifecycle")


def normalize(vector: np.ndarray, eps: float = 1e-12) -> Optional[np.ndarray]:
    """Normalize a vector while avoiding division by zero.

    Parameters
    ----------
    vector : np.ndarray
        Input vector.
    eps : float, optional
        Minimum norm accepted as non-degenerate, by default 1e-12.

    Returns
    -------
    Optional[np.ndarray]
        Unit vector, or None if the vector norm is too small.
    """
    norm = float(np.linalg.norm(vector))
    if norm < eps:
        return None
    return vector / norm


def clamp_angle_step(previous_normal: np.ndarray, new_normal: np.ndarray, max_deg: float) -> Optional[np.ndarray]:
    """Limit the angular change between two normal vectors.

    Parameters
    ----------
    previous_normal : np.ndarray
        Previous filtered normal vector.
    new_normal : np.ndarray
        New raw normal vector.
    max_deg : float
        Maximum allowed angular step in degrees.

    Returns
    -------
    Optional[np.ndarray]
        Normal vector with limited angular step, or None if normalization fails.
    """
    previous_normal = normalize(previous_normal)
    new_normal = normalize(new_normal)

    if previous_normal is None or new_normal is None:
        return new_normal

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


def fit_plane_svd(points_xyz: np.ndarray):
    """Fit a plane to 3D points using singular value decomposition.

    Parameters
    ----------
    points_xyz : np.ndarray
        Array of 3D points with shape (N, 3).

    Returns
    -------
    tuple[np.ndarray, float, np.ndarray, float, int]
        Plane normal, plane offset d, centroid, RMSE in meters, and number of
        inlier points used for the fit.

    Raises
    ------
    ValueError
        If fewer than three valid points are available or the normal is
        degenerate.
    """
    valid_points = points_xyz[np.all(np.isfinite(points_xyz), axis=1)]
    if valid_points.shape[0] < 3:
        raise ValueError("Not enough valid points to fit a plane.")

    centroid = valid_points.mean(axis=0)
    centered_points = valid_points - centroid

    _, _, vh = np.linalg.svd(centered_points, full_matrices=False)
    normal = normalize(vh[-1, :])
    if normal is None:
        raise ValueError("Degenerate plane normal.")

    d = -float(np.dot(normal, centroid))
    signed_distances = centered_points @ normal
    rmse_m = float(np.sqrt(np.mean(signed_distances ** 2)))

    return normal.astype(float), d, centroid.astype(float), rmse_m, int(valid_points.shape[0])


def make_centroid_marker(frame_id: str, stamp, position: np.ndarray, namespace: str, marker_id: int = 0) -> Marker:
    """Create an RViz sphere marker for the palm plane centroid.

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
    marker.id = int(marker_id)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = float(position[0])
    marker.pose.position.y = float(position[1])
    marker.pose.position.z = float(position[2])
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
    return marker


def make_normal_marker(
    frame_id: str,
    stamp,
    position: np.ndarray,
    normal: np.ndarray,
    namespace: str,
    marker_id: int = 0,
    length: float = 0.10,
    rgba=(0.0, 1.0, 1.0, 0.95),
) -> Marker:
    """Create an RViz arrow marker for the palm plane normal.

    Parameters
    ----------
    frame_id : str
        Reference frame for the marker.
    stamp
        ROS timestamp.
    position : np.ndarray
        Arrow start point.
    normal : np.ndarray
        Normal vector direction.
    namespace : str
        Marker namespace.
    marker_id : int, optional
        Marker identifier, by default 0.
    length : float, optional
        Arrow length in meters, by default 0.10.
    rgba : tuple, optional
        Marker color as RGBA, by default cyan.

    Returns
    -------
    Marker
        RViz arrow marker.
    """
    start = np.asarray(position, dtype=float).reshape(3)
    end = start + float(length) * np.asarray(normal, dtype=float).reshape(3)

    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = int(marker_id)
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.008
    marker.scale.y = 0.016
    marker.scale.z = 0.016
    marker.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
    marker.points = [
        GeoPoint(x=float(start[0]), y=float(start[1]), z=float(start[2])),
        GeoPoint(x=float(end[0]), y=float(end[1]), z=float(end[2])),
    ]
    return marker


def project_points_to_plane(points: np.ndarray, normal: np.ndarray, d: float) -> np.ndarray:
    """Project 3D points onto a plane.

    Parameters
    ----------
    points : np.ndarray
        3D points with shape (N, 3).
    normal : np.ndarray
        Unit plane normal.
    d : float
        Plane equation offset.

    Returns
    -------
    np.ndarray
        Projected 3D points.
    """
    distances = points @ normal + d
    return points - distances[:, None] * normal[None, :]


def plane_basis_from_normal(normal: np.ndarray):
    """Compute two orthogonal basis vectors lying on a plane.

    Parameters
    ----------
    normal : np.ndarray
        Plane normal.

    Returns
    -------
    tuple[np.ndarray, np.ndarray]
        Two unit vectors spanning the plane.

    Raises
    ------
    ValueError
        If a valid basis cannot be constructed.
    """
    axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(np.dot(axis, normal)) > 0.9:
        axis = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    basis_u = np.cross(axis, normal)
    basis_u = normalize(basis_u)
    if basis_u is None:
        raise ValueError("Could not build a valid plane basis.")

    basis_v = np.cross(normal, basis_u)
    return basis_u, basis_v


def to_plane_2d(points_plane: np.ndarray, basis_u: np.ndarray, basis_v: np.ndarray) -> np.ndarray:
    """Convert 3D points on a plane into 2D plane coordinates.

    Parameters
    ----------
    points_plane : np.ndarray
        3D points projected onto a plane.
    basis_u : np.ndarray
        First plane basis vector.
    basis_v : np.ndarray
        Second plane basis vector.

    Returns
    -------
    np.ndarray
        2D coordinates with shape (N, 2).
    """
    return np.stack([points_plane @ basis_u, points_plane @ basis_v], axis=1)


def convex_hull_indices_2d(points_2d: np.ndarray) -> np.ndarray:
    """Compute convex hull indices for 2D points.

    Parameters
    ----------
    points_2d : np.ndarray
        2D points with shape (N, 2).

    Returns
    -------
    np.ndarray
        Indices of hull vertices.

    Raises
    ------
    ValueError
        If the hull contains fewer than three points.
    """
    points_cv = points_2d.astype(np.float32).reshape(-1, 1, 2)
    hull = cv2.convexHull(points_cv, returnPoints=False).reshape(-1).astype(int)
    if hull.size < 3:
        raise ValueError("Convex hull contains fewer than three points.")
    return hull


def triangulate_convex_polygon(vertex_count: int) -> np.ndarray:
    """Triangulate a convex polygon using fan triangulation.

    Parameters
    ----------
    vertex_count : int
        Number of polygon vertices.

    Returns
    -------
    np.ndarray
        Triangle index array with shape (N - 2, 3).
    """
    triangles = []
    for index in range(1, vertex_count - 1):
        triangles.append([0, index, index + 1])
    return np.array(triangles, dtype=np.int32)


def make_mesh_marker(
    header: Header,
    vertices: np.ndarray,
    triangles: np.ndarray,
    rgba=(0.0, 1.0, 0.0, 0.35),
    marker_namespace: str = "hand_palm_plane_mesh",
    marker_id: int = 0,
) -> Marker:
    """Create an RViz TRIANGLE_LIST marker from vertices and triangle indices.

    Parameters
    ----------
    header : Header
        Marker header.
    vertices : np.ndarray
        3D vertices with shape (N, 3).
    triangles : np.ndarray
        Triangle indices with shape (M, 3).
    rgba : tuple, optional
        Mesh color, by default translucent green.
    marker_namespace : str, optional
        Marker namespace.
    marker_id : int, optional
        Marker identifier.

    Returns
    -------
    Marker
        RViz triangle list marker.
    """
    marker = Marker()
    marker.header = header
    marker.ns = marker_namespace
    marker.id = int(marker_id)
    marker.type = Marker.TRIANGLE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))

    marker_points = []
    for index_a, index_b, index_c in triangles.astype(np.int32):
        for point in (vertices[index_a], vertices[index_b], vertices[index_c]):
            marker_points.append(GeoPoint(x=float(point[0]), y=float(point[1]), z=float(point[2])))

    marker.points = marker_points
    return marker


class PalmPlaneFromHand3DLifecycleNode(LifecycleNode):
    """Lifecycle node for estimating stable palm planes from 3D hand landmarks.

    Parameters
    ----------
    tracked_topic : str
        Topic containing currently tracked hand identifiers.
    landmarks3d_suffix : str
        Per-hand suffix used to subscribe to 3D landmarks.
    plane_suffix : str
        Per-hand suffix used to publish palm plane estimates.
    plane_indices : list[int]
        Landmark indices used for plane fitting.
    min_inliers : int
        Minimum number of valid points required to accept a plane.
    rmse_accept_mm : float
        Maximum allowed plane fitting RMSE in millimeters.
    publish_last_good_if_rejected : bool
        Republish the last valid plane when the current estimate is rejected.
    plane_flip_guard_enable : bool
        Keep plane normal orientation consistent over time.
    plane_limit_step_enable : bool
        Limit angular change between consecutive normals.
    plane_max_step_deg : float
        Maximum normal angular step in degrees.
    plane_ema_enable : bool
        Enable exponential moving average filtering of the normal.
    plane_ema_alpha : float
        EMA alpha used for normal filtering.
    plane_initial_face_camera : bool
        Force the first normal estimate to face the camera frame.
    stale_hand_seconds : float
        Timeout used to remove stale dynamic hand pipelines.
    publish_markers : bool
        Enable RViz marker publication.
    normal_marker_length_m : float
        Length of the normal vector marker.
    publish_mesh_marker : bool
        Enable palm mesh marker publication.
    mesh_marker_ns : str
        Namespace used for mesh markers.
    mesh_marker_id : int
        Mesh marker identifier.
    mesh_rgba : list[float]
        Mesh marker color.
    mesh_use_all_valid_points : bool
        Use all valid landmarks for mesh generation instead of only plane points.
    """

    def __init__(self):
        """Declare parameters and initialize lifecycle-managed state."""
        super().__init__("hand_palm_plane_node")

        self.declare_parameter("tracked_topic", "/humans/hands/tracked")
        self.declare_parameter("landmarks3d_suffix", "landmarks3d")
        self.declare_parameter("plane_suffix", "palm_plane_cam")
        self.declare_parameter("plane_indices", [0, 5, 9, 13, 17])
        self.declare_parameter("min_inliers", 3)
        self.declare_parameter("rmse_accept_mm", 20.0)
        self.declare_parameter("publish_last_good_if_rejected", True)
        self.declare_parameter("plane_flip_guard_enable", True)
        self.declare_parameter("plane_limit_step_enable", True)
        self.declare_parameter("plane_max_step_deg", 5.0)
        self.declare_parameter("plane_ema_enable", True)
        self.declare_parameter("plane_ema_alpha", 0.2)
        self.declare_parameter("plane_initial_face_camera", True)
        self.declare_parameter("stale_hand_seconds", 1.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("normal_marker_length_m", 0.10)
        self.declare_parameter("publish_mesh_marker", True)
        self.declare_parameter("mesh_marker_ns", "hand_palm_plane_mesh")
        self.declare_parameter("mesh_marker_id", 0)
        self.declare_parameter("mesh_rgba", [0.0, 1.0, 0.0, 0.35])
        self.declare_parameter("mesh_use_all_valid_points", False)

        self.active = False
        self.tracked_topic = ""
        self.landmarks3d_suffix = ""
        self.plane_suffix = ""
        self.plane_indices = []

        self.min_inliers = 3
        self.rmse_accept_mm = 20.0
        self.publish_last_good_if_rejected = True
        self.plane_flip_guard_enable = True
        self.plane_limit_step_enable = True
        self.plane_max_step_deg = 5.0
        self.plane_ema_enable = True
        self.plane_ema_alpha = 0.2
        self.plane_initial_face_camera = True
        self.stale_hand_seconds = 1.0
        self.publish_markers = True
        self.normal_marker_length_m = 0.10
        self.publish_mesh_marker = True
        self.mesh_marker_ns = "hand_palm_plane_mesh"
        self.mesh_marker_id = 0
        self.mesh_rgba = [0.0, 1.0, 0.0, 0.35]
        self.mesh_use_all_valid_points = False

        self.sub_tracked = None
        self.cleanup_timer = None
        self._hands = {}

        LOG.info("Palm plane lifecycle node created. Initial state: Unconfigured")

    def on_configure(self, state):
        """Read parameters and create static lifecycle resources.

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
            self.landmarks3d_suffix = str(self.get_parameter("landmarks3d_suffix").value)
            self.plane_suffix = str(self.get_parameter("plane_suffix").value)
            self.plane_indices = [int(index) for index in self.get_parameter("plane_indices").value]

            self.min_inliers = int(self.get_parameter("min_inliers").value)
            self.rmse_accept_mm = float(self.get_parameter("rmse_accept_mm").value)
            self.publish_last_good_if_rejected = bool(self.get_parameter("publish_last_good_if_rejected").value)

            self.plane_flip_guard_enable = bool(self.get_parameter("plane_flip_guard_enable").value)
            self.plane_limit_step_enable = bool(self.get_parameter("plane_limit_step_enable").value)
            self.plane_max_step_deg = float(self.get_parameter("plane_max_step_deg").value)
            self.plane_ema_enable = bool(self.get_parameter("plane_ema_enable").value)
            self.plane_ema_alpha = float(self.get_parameter("plane_ema_alpha").value)
            self.plane_initial_face_camera = bool(self.get_parameter("plane_initial_face_camera").value)

            self.stale_hand_seconds = float(self.get_parameter("stale_hand_seconds").value)
            self.publish_markers = bool(self.get_parameter("publish_markers").value)
            self.normal_marker_length_m = float(self.get_parameter("normal_marker_length_m").value)

            self.publish_mesh_marker = bool(self.get_parameter("publish_mesh_marker").value)
            self.mesh_marker_ns = str(self.get_parameter("mesh_marker_ns").value)
            self.mesh_marker_id = int(self.get_parameter("mesh_marker_id").value)
            self.mesh_rgba = [float(value) for value in self.get_parameter("mesh_rgba").value]
            self.mesh_use_all_valid_points = bool(self.get_parameter("mesh_use_all_valid_points").value)

            self.sub_tracked = self.create_subscription(
                IdsList,
                self.tracked_topic,
                self.tracked_callback,
                10,
            )
            self.cleanup_timer = self.create_timer(0.25, self.cleanup_timer_callback)

            LOG.info("Palm plane lifecycle node configured.")
            LOG.info(f"  tracked_topic: {self.tracked_topic}")
            LOG.info(f"  plane_indices: {self.plane_indices}")
            LOG.info(f"  plane_suffix: {self.plane_suffix}")

            return TransitionCallbackReturn.SUCCESS

        except Exception as exc:
            LOG.exception(f"Failed to configure palm plane lifecycle node: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        """Enable dynamic hand pipelines and palm plane estimation."""
        _ = state
        self.active = True
        LOG.info("Palm plane lifecycle node activated.")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Disable plane estimation and destroy all dynamic hand pipelines."""
        _ = state
        self.active = False
        self.destroy_all_hand_pipelines()
        LOG.info("Palm plane lifecycle node deactivated. Dynamic hand pipelines destroyed.")
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

            self._hands.clear()

            LOG.info("Palm plane lifecycle node cleaned up.")
            return TransitionCallbackReturn.SUCCESS

        except Exception as exc:
            LOG.exception(f"Failed to cleanup palm plane lifecycle node: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state):
        """Stop processing and release dynamic resources during shutdown."""
        _ = state
        self.active = False
        self.destroy_all_hand_pipelines()
        LOG.info("Palm plane lifecycle node shutting down.")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        """Move the node to a safe internal state after an error."""
        _ = state
        self.active = False
        self.destroy_all_hand_pipelines()
        LOG.error("Palm plane lifecycle node entered error state.")
        return TransitionCallbackReturn.SUCCESS

    def tracked_callback(self, msg: IdsList):
        """Create or update dynamic hand pipelines from tracked IDs.

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

        current_time = time.time()
        for hand_id in msg.ids:
            if hand_id not in self._hands:
                self.create_hand_pipeline(hand_id)
            self._hands[hand_id]["last_seen"] = current_time

    def create_hand_pipeline(self, hand_id: str):
        """Create the subscriber and publishers for one tracked hand.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand.

        Returns
        -------
        None
        """
        landmarks_topic = f"/humans/hands/{hand_id}/{self.landmarks3d_suffix}"
        plane_topic = f"/humans/hands/{hand_id}/{self.plane_suffix}"

        sub_landmarks = self.create_subscription(
            HandLandmarks3D,
            landmarks_topic,
            lambda msg, current_hand_id=hand_id: self.hand_landmarks_callback(current_hand_id, msg),
            10,
        )

        pub_plane = self.create_publisher(PlaneStamped, plane_topic, 10)

        pub_centroid = None
        pub_normal = None
        pub_mesh = None

        if self.publish_markers:
            pub_centroid = self.create_publisher(
                Marker,
                f"/humans/hands/{hand_id}/palm_plane_centroid_marker",
                10,
            )
            pub_normal = self.create_publisher(
                Marker,
                f"/humans/hands/{hand_id}/palm_plane_normal_marker",
                10,
            )
            if self.publish_mesh_marker:
                pub_mesh = self.create_publisher(
                    Marker,
                    f"/humans/hands/{hand_id}/palm_plane_mesh_marker",
                    10,
                )

        self._hands[hand_id] = {
            "sub_landmarks": sub_landmarks,
            "pub_plane": pub_plane,
            "pub_centroid": pub_centroid,
            "pub_normal": pub_normal,
            "pub_mesh": pub_mesh,
            "last_seen": time.time(),
            "last_good_plane": None,
            "filtered_plane_normal": None,
        }

        LOG.info(f"[{hand_id}] palm plane pipeline created. Sub: {landmarks_topic}, Pub: {plane_topic}")

    def destroy_hand_pipeline(self, hand_id: str):
        """Destroy the dynamic resources associated with one hand.

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

        if hand_state.get("sub_landmarks") is not None:
            self.destroy_subscription(hand_state["sub_landmarks"])

        for key in ["pub_plane", "pub_centroid", "pub_normal", "pub_mesh"]:
            publisher = hand_state.get(key)
            if publisher is not None:
                self.destroy_publisher(publisher)

        del self._hands[hand_id]
        LOG.info(f"[{hand_id}] palm plane pipeline destroyed.")

    def destroy_all_hand_pipelines(self):
        """Destroy all dynamic hand pipelines.

        Returns
        -------
        None
        """
        for hand_id in list(self._hands.keys()):
            self.destroy_hand_pipeline(hand_id)
        self._hands.clear()

    def cleanup_timer_callback(self):
        """Remove hand pipelines that have not been updated recently."""
        if not self.active:
            return

        current_time = time.time()
        stale_hands = []
        for hand_id, hand_state in self._hands.items():
            if (current_time - hand_state["last_seen"]) > self.stale_hand_seconds:
                stale_hands.append(hand_id)

        for hand_id in stale_hands:
            self.destroy_hand_pipeline(hand_id)

    def is_point_valid(self, msg: HandLandmarks3D, index: int) -> bool:
        """Check if a landmark index contains a valid 3D point."""
        if index < 0 or index >= len(msg.landmarks):
            return False
        if hasattr(msg, "valid") and len(msg.valid) == len(msg.landmarks):
            return bool(msg.valid[index])
        return True

    def point_to_numpy(self, msg: HandLandmarks3D, index: int) -> np.ndarray:
        """Convert a landmark point into a NumPy array."""
        point = msg.landmarks[index]
        return np.array([float(point.x), float(point.y), float(point.z)], dtype=np.float64)

    def extract_plane_points(self, msg: HandLandmarks3D) -> np.ndarray:
        """Extract valid 3D landmarks used for plane fitting."""
        points = []
        for index in self.plane_indices:
            if not self.is_point_valid(msg, index):
                continue
            point = self.point_to_numpy(msg, index)
            if np.all(np.isfinite(point)):
                points.append(point)
        return np.array(points, dtype=np.float64)

    def extract_mesh_points(self, msg: HandLandmarks3D, plane_points: np.ndarray) -> np.ndarray:
        """Extract valid 3D landmarks used for palm mesh generation."""
        if not self.mesh_use_all_valid_points:
            return plane_points

        points = []
        for index in range(len(msg.landmarks)):
            if not self.is_point_valid(msg, index):
                continue
            point = self.point_to_numpy(msg, index)
            if np.all(np.isfinite(point)):
                points.append(point)
        return np.array(points, dtype=np.float64)

    def orient_plane_normal(self, hand_state: dict, raw_normal: np.ndarray) -> Optional[np.ndarray]:
        """Orient a raw plane normal consistently over time."""
        normal = normalize(raw_normal)
        if normal is None:
            return None

        filtered_normal = hand_state["filtered_plane_normal"]
        if filtered_normal is None:
            if self.plane_initial_face_camera and normal[2] > 0.0:
                normal = -normal
            return normal

        if self.plane_flip_guard_enable and float(np.dot(filtered_normal, normal)) < 0.0:
            normal = -normal

        return normal

    def filter_plane_normal(self, hand_state: dict, raw_normal: np.ndarray) -> Optional[np.ndarray]:
        """Filter a palm plane normal using optional step limiting and EMA."""
        normal = normalize(raw_normal)
        if normal is None:
            return None

        if hand_state["filtered_plane_normal"] is None:
            hand_state["filtered_plane_normal"] = normal.copy()
            return hand_state["filtered_plane_normal"]

        normal_to_use = normal.copy()

        if self.plane_limit_step_enable and self.plane_max_step_deg > 0.0:
            normal_to_use = clamp_angle_step(
                hand_state["filtered_plane_normal"],
                normal_to_use,
                self.plane_max_step_deg,
            )

        if normal_to_use is None:
            return None

        if self.plane_ema_enable:
            alpha = float(np.clip(self.plane_ema_alpha, 0.0, 1.0))
            hand_state["filtered_plane_normal"] = normalize(
                alpha * normal_to_use + (1.0 - alpha) * hand_state["filtered_plane_normal"]
            )
        else:
            hand_state["filtered_plane_normal"] = normal_to_use

        return hand_state["filtered_plane_normal"]

    def hand_landmarks_callback(self, hand_id: str, msg: HandLandmarks3D):
        """Estimate and publish the palm plane for one hand.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand.
        msg : HandLandmarks3D
            Input 3D hand landmarks.

        Returns
        -------
        None
        """
        if not self.active:
            return

        hand_state = self._hands.get(hand_id)
        if hand_state is None:
            return

        plane_points = self.extract_plane_points(msg)
        if plane_points.shape[0] < 3:
            return

        try:
            normal_svd, _, centroid, rmse_m, inlier_count = fit_plane_svd(plane_points)
        except Exception:
            return

        raw_normal = self.orient_plane_normal(hand_state, normal_svd)
        if raw_normal is None:
            return

        plane_normal = self.filter_plane_normal(hand_state, raw_normal)
        if plane_normal is None:
            return

        plane_d = -float(np.dot(plane_normal, centroid))
        rmse_mm = float(rmse_m * 1000.0)

        accepted = (
            inlier_count >= self.min_inliers
            and np.isfinite(rmse_mm)
            and rmse_mm <= self.rmse_accept_mm
        )

        reject_reasons = []
        if inlier_count < self.min_inliers:
            reject_reasons.append(f"inliers<{self.min_inliers} (got={inlier_count})")
        if (not np.isfinite(rmse_mm)) or rmse_mm > self.rmse_accept_mm:
            reject_reasons.append(f"rmse>{self.rmse_accept_mm:.1f}mm (got={rmse_mm:.2f})")

        plane_data = {
            "normal": plane_normal,
            "d": plane_d,
            "centroid": centroid,
            "rmse_m": rmse_m,
            "rmse_mm": rmse_mm,
            "inlier_count": inlier_count,
        }

        if accepted:
            hand_state["last_good_plane"] = plane_data
        elif self.publish_last_good_if_rejected and hand_state["last_good_plane"] is not None:
            plane_data = hand_state["last_good_plane"]

        self.publish_plane(hand_state, msg.header, plane_data, accepted, reject_reasons)

        if self.publish_markers:
            self.publish_plane_markers(hand_id, hand_state, msg.header, plane_data)

        if self.publish_markers and self.publish_mesh_marker:
            self.publish_mesh_marker_if_possible(hand_state, msg, plane_points, plane_data)

    def publish_plane(self, hand_state: dict, header: Header, plane_data: dict, accepted: bool, reject_reasons: list[str]):
        """Publish a PlaneStamped message."""
        msg = PlaneStamped()
        msg.header = header
        msg.normal = Vector3(
            x=float(plane_data["normal"][0]),
            y=float(plane_data["normal"][1]),
            z=float(plane_data["normal"][2]),
        )
        msg.d = float(plane_data["d"])
        msg.centroid = Point(
            x=float(plane_data["centroid"][0]),
            y=float(plane_data["centroid"][1]),
            z=float(plane_data["centroid"][2]),
        )
        msg.rmse_m = float(plane_data["rmse_m"])
        msg.rmse_mm = float(plane_data["rmse_mm"])
        msg.n_inliers = int(plane_data["inlier_count"])
        msg.fit_method = "svd_stable_plane"
        msg.accepted = bool(accepted)
        msg.reject_reasons = [str(reason) for reason in reject_reasons]

        hand_state["pub_plane"].publish(msg)

    def publish_plane_markers(self, hand_id: str, hand_state: dict, header: Header, plane_data: dict):
        """Publish centroid and normal markers for the estimated plane."""
        centroid = np.array(plane_data["centroid"], dtype=float)
        normal = np.array(plane_data["normal"], dtype=float)

        if hand_state["pub_centroid"] is not None:
            hand_state["pub_centroid"].publish(
                make_centroid_marker(
                    frame_id=header.frame_id,
                    stamp=header.stamp,
                    position=centroid,
                    namespace=f"{hand_id}_palm_plane_centroid",
                )
            )

        if hand_state["pub_normal"] is not None:
            hand_state["pub_normal"].publish(
                make_normal_marker(
                    frame_id=header.frame_id,
                    stamp=header.stamp,
                    position=centroid,
                    normal=normal,
                    namespace=f"{hand_id}_palm_plane_normal_stable",
                    length=self.normal_marker_length_m,
                    rgba=(0.0, 1.0, 1.0, 0.95),
                )
            )

    def publish_mesh_marker_if_possible(self, hand_state: dict, msg: HandLandmarks3D, plane_points: np.ndarray, plane_data: dict):
        """Build and publish a palm mesh marker if enough valid points are available."""
        if hand_state.get("pub_mesh") is None:
            return

        try:
            mesh_points = self.extract_mesh_points(msg, plane_points=plane_points)
            if mesh_points.shape[0] < 3:
                return

            normal = np.array(plane_data["normal"], dtype=np.float64)
            d = float(plane_data["d"])

            tolerance_m = 0.02
            distances = mesh_points @ normal + d
            keep_mask = np.isfinite(distances) & (np.abs(distances) <= tolerance_m)
            mesh_points = mesh_points[keep_mask]
            if mesh_points.shape[0] < 3:
                return

            projected_points = project_points_to_plane(mesh_points, normal, d)
            basis_u, basis_v = plane_basis_from_normal(normal)
            points_2d = to_plane_2d(projected_points, basis_u, basis_v)

            hull_indices = convex_hull_indices_2d(points_2d)
            hull_vertices = projected_points[hull_indices]
            triangles = triangulate_convex_polygon(hull_vertices.shape[0])

            marker_header = Header()
            marker_header.stamp = msg.header.stamp
            marker_header.frame_id = msg.header.frame_id

            hand_state["pub_mesh"].publish(
                make_mesh_marker(
                    header=marker_header,
                    vertices=hull_vertices,
                    triangles=triangles,
                    rgba=(
                        self.mesh_rgba[0],
                        self.mesh_rgba[1],
                        self.mesh_rgba[2],
                        self.mesh_rgba[3],
                    ),
                    marker_namespace=self.mesh_marker_ns,
                    marker_id=self.mesh_marker_id,
                )
            )
        except Exception as exc:
            LOG.warning(f"[mesh] failed: {exc}")


def main(args=None):
    """Run the palm plane lifecycle node."""
    rclpy.init(args=args)
    node = PalmPlaneFromHand3DLifecycleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
