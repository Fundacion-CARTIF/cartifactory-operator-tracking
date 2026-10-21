#!/usr/bin/env python3
"""ROS2 node for estimating stable palm planes from 3D hand landmarks.
This node subscribes to 3D hand landmark topics and computes a palm plane for each tracked hand using Singular Value Decomposition (SVD)-based plane fitting. The estimated plane is stabilized over time using configurable filtering techniques and optionally visualized in RViz.

The node dynamically creates and removes per-hand processing pipelines according to the currently tracked hands.

Author:
    CARTIF Technology Centre
"""
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from hri_msgs.msg import IdsList
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Vector3, Point
from geometry_msgs.msg import Point as GeoPoint

from hand_hri_mediapipe_msgs.msg import PlaneStamped
from hand_hri_mediapipe_msgs.msg import HandLandmarks3D

import logging
from rich.logging import RichHandler


logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler(rich_tracebacks=True)],
    force=True,
)
LOG = logging.getLogger("hand_palm_plane")


def normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray | None:
    """Function that converts a vector into a unit vector (a vector with length 1), while safely avoiding division by zero.

    Parameters
    ----------
    v : np.ndarray
        NumPy array we want to convert into a unit vector.
    eps : float, optional
        Small value to avoid division by zero, by default 1e-12

    Returns
    -------
    np.ndarray
        Unit vector in the same direction as v, or the original vector if its length is too small.
    """
    n = float(np.linalg.norm(v))
    if n < eps:
        return None
    return v / n


def clamp_angle_step(n_prev: np.ndarray, n_new: np.ndarray, max_deg: float):
    """Function that clamps the angle between two normal vectors to a maximum value. Clamping does not allow the direction to change more than a certain maximum angle in one update.

    Parameters
    ----------
    n_prev : np.ndarray
        Previous normal vector.
    n_new : np.ndarray
        New normal vector.
    max_deg : float
        Maximum angle in degrees.
    Returns
    -------
    np.ndarray
        Clamped normal vector.
    """
    n_prev_norm = normalize(n_prev)
    n_new_norm = normalize(n_new)

    if n_prev_norm is None or n_new_norm is None:
        return n_new_norm

    n_prev = n_prev_norm
    n_new = n_new_norm

    dot = float(np.clip(np.dot(n_prev, n_new), -1.0, 1.0))
    ang = float(np.arccos(dot))

    max_rad = float(np.deg2rad(max_deg))
    if ang <= max_rad or ang < 1e-9:
        return n_new

    t = max_rad / ang
    sin_ang = float(np.sin(ang))
    if abs(sin_ang) < 1e-12:
        return n_new

    a = float(np.sin((1.0 - t) * ang) / sin_ang)
    b = float(np.sin(t * ang) / sin_ang)
    return normalize(a * n_prev + b * n_new)


def fit_plane_svd(points_xyz: np.ndarray):
    """This function removes invalid points, computes the centroid of the valid points, and estimates the best-fit plane in a least-squares sense using SVD. The plane normal corresponds to the singular vector associated with the smallest singular value.

    The function also computes:
    - The plane equation coefficient `d`
    - The centroid of the fitted points
    - The root mean square error (RMSE) of the fit
    - The number of valid inlier points used

    Parameters
    ----------
    points_xyz : np.ndarray
        Array of 3D points of shape `(N, 3)`.

    Returns
    -------
    tuple[np.ndarray, float, np.ndarray, float, int]
        A tuple containing:

        - normal : np.ndarray
            Normalized plane normal vector of shape `(3,)`.

        - d : float
            Plane equation offset for the equation:
            `ax + by + cz + d = 0`.

        - centroid : np.ndarray
            Centroid of the valid input points.

        - rmse : float
            Root mean square fitting error in meters.

        - n_inliers : int
            Number of valid points used for plane fitting.

    Raises
    ------
    ValueError
        If fewer than three valid points are available or if the estimated
        plane normal is degenerate.
    """
    pts = points_xyz[np.all(np.isfinite(points_xyz), axis=1)]
    if pts.shape[0] < 3:
        raise ValueError("No hay suficientes puntos válidos para ajustar el plano.")

    centroid = pts.mean(axis=0)
    P = pts - centroid

    _, _, Vt = np.linalg.svd(P, full_matrices=False)
    normal = normalize(Vt[-1, :])
    if normal is None:
        raise ValueError("Normal degenerada.")

    d = -float(np.dot(normal, centroid))
    dist_signed = P @ normal
    rmse = float(np.sqrt(np.mean(dist_signed ** 2)))

    return normal.astype(float), float(d), centroid.astype(float), rmse, int(pts.shape[0])


def make_centroid_marker(frame_id: str, stamp, p: np.ndarray, ns: str, mid: int = 0) -> Marker:
    """This function creates a marker of type SPHERE to visualize the centroid of the palm plane in RViz.

    Parameters
    ----------
    frame_id : str
        The reference frame for the marker (e.g., "base").
    stamp : _type_
        The timestamp for the marker.
    p : np.ndarray
        The position of the marker.
    ns : str
        The namespace for the marker.
    marker_id : int, optional
        The ID for the marker, by default 0

    Returns
    -------
    Marker
        The created marker.
    """
    m = Marker()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = int(mid)
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.pose.position.x = float(p[0])
    m.pose.position.y = float(p[1])
    m.pose.position.z = float(p[2])
    m.scale.x = 0.02
    m.scale.y = 0.02
    m.scale.z = 0.02
    m.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
    return m


def make_normal_marker(
    frame_id: str,
    stamp,
    p: np.ndarray,
    n: np.ndarray,
    ns: str,
    mid: int = 0,
    length: float = 0.10,
    rgba=(0.0, 1.0, 1.0, 0.95),
) -> Marker:
    m = Marker()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = int(mid)
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.008
    m.scale.y = 0.016
    m.scale.z = 0.016
    m.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))
    """This function creates a marker to visualize the normal vector of the palm plane in RViz.

    Parameters
    ----------
    frame_id : str
        The reference frame for the marker (e.g., "base").
    stamp : _type_
        The timestamp for the marker.
    p : np.ndarray
        The position of the marker.
    n : np.ndarray
        The normal vector for the marker.
    ns : str
        The namespace for the marker.
    marker_id : int, optional
        The ID for the marker, by default 0

    Returns
    -------
    Marker
        The created normal marker.
    """


    p0 = np.asarray(p, dtype=float).reshape(3)
    p1 = p0 + float(length) * np.asarray(n, dtype=float).reshape(3)

    m.points = [
        GeoPoint(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])),
        GeoPoint(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])),
    ]
    return m


def project_points_to_plane(pts: np.ndarray, n: np.ndarray, d: float) -> np.ndarray:
    """Project 3D points onto a plane.

    Parameters
    ----------
    pts : np.ndarray
        Array of 3D points with shape (N, 3).

    n : np.ndarray
        Normalized plane normal vector with shape (3,).

    d : float
        Plane offset for the equation ax + by + cz + d = 0.

    Returns
    -------
    np.ndarray
        Projected 3D points on the plane, with shape (N, 3).
    """
    dist = pts @ n + d
    return pts - dist[:, None] * n[None, :]


def plane_basis_from_normal(n: np.ndarray):
    """Compute two orthogonal basis vectors lying on a plane.

    Parameters
    ----------
    n : np.ndarray
        Normalized plane normal vector.

    Returns
    -------
    tuple[np.ndarray, np.ndarray]
        Two orthogonal unit vectors that span the plane.

    Raises
    ------
    ValueError
        If a valid plane basis cannot be constructed.
    """
    a = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(np.dot(a, n)) > 0.9:
        a = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    u = np.cross(a, n)
    u = normalize(u)
    if u is None:
        raise ValueError("No se pudo construir base del plano.")

    v = np.cross(n, u)
    return u, v


def to_plane_2d(pts_plane: np.ndarray, u: np.ndarray, v: np.ndarray):
    """
    Convert 3D points on a plane into 2D plane coordinates.

    Parameters
    ----------
    pts_plane : np.ndarray
        Array of 3D points lying on the plane.

    u : np.ndarray
        First basis vector of the plane.

    v : np.ndarray
        Second basis vector of the plane.

    Returns
    -------
    np.ndarray
        Array of 2D coordinates with shape (N, 2).
    """
    return np.stack([pts_plane @ u, pts_plane @ v], axis=1)


def convex_hull_indices_2d(pts2: np.ndarray):
    """
    Compute the convex hull indices of 2D points.

    Parameters
    ----------
    pts2 : np.ndarray
        Array of 2D points with shape (N, 2).

    Returns
    -------
    np.ndarray
        Indices of the points forming the convex hull.

    Raises
    ------
    ValueError
        If the computed hull has fewer than three points.
    """
    pts_cv = pts2.astype(np.float32).reshape(-1, 1, 2)
    hull = cv2.convexHull(pts_cv, returnPoints=False)
    hull = hull.reshape(-1).astype(int)
    if hull.size < 3:
        raise ValueError("Hull con menos de 3 puntos.")
    return hull


def triangulate_convex_polygon(num_vertices: int):
    """
    Triangulate a convex polygon using a fan triangulation.

    Parameters
    ----------
    num_vertices : int
        Number of vertices in the convex polygon.

    Returns
    -------
    np.ndarray
        Triangle indices with shape (N - 2, 3).
    """
    tris = []
    for i in range(1, num_vertices - 1):
        tris.append([0, i, i + 1])
    return np.array(tris, dtype=np.int32)


def make_mesh_marker(
    header: Header,
    vertices: np.ndarray,
    triangles: np.ndarray,
    rgba=(0.0, 1.0, 0.0, 0.35),
    marker_ns="hand_palm_plane_mesh",
    marker_id=0,
    ) -> Marker:
    """This function generates a `TRIANGLE_LIST` visualization marker from a set of vertices and triangle indices. The resulting marker can be displayed in RViz to visualize a polygonal mesh, such as the estimated palm surface.

    Parameters
    ----------
    header : Header
        ROS message header containing timestamp and reference frame information.

    vertices : np.ndarray
        Array of 3D vertex coordinates with shape `(N, 3)`.

    triangles : np.ndarray
        Array of triangle indices with shape `(M, 3)`. Each row contains the indices of the three vertices defining one triangle.

    rgba : tuple[float, float, float, float], optional
        RGBA color used for the mesh marker, by default (0.0, 1.0, 0.0, 0.35).

    marker_ns : str, optional
        Namespace assigned to the visualization marker, by default "hand_palm_plane_mesh".

    marker_id : int, optional
        Unique marker identifier within the namespace, by default `0`.

    Returns
    -------
    Marker
        RViz `TRIANGLE_LIST` marker representing the mesh surface.
    """
    m = Marker()
    m.header = header
    m.ns = marker_ns
    m.id = int(marker_id)
    m.type = Marker.TRIANGLE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 1.0
    m.scale.y = 1.0
    m.scale.z = 1.0
    m.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))

    pts_list = []
    for a, b, c in triangles.astype(np.int32):
        for p in (vertices[a], vertices[b], vertices[c]):
            pts_list.append(GeoPoint(x=float(p[0]), y=float(p[1]), z=float(p[2])))

    m.points = pts_list
    return m


class PalmPlaneFromHand3DNode(Node):
    """ROS2 node for estimating stable palm planes from 3D hand landmarks.

    This node subscribes to dynamically tracked 3D hand landmark topics and computes a palm plane for each hand using Singular Value Decomposition (SVD)-based plane fitting. The estimated plane is stabilized over time using configurable filtering and orientation constraints.

    The node also publishes optional RViz visualization markers, including palm centroids, normal vectors, and triangulated palm mesh surfaces.
    """
    def __init__(self):
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

        self.tracked_topic = self.get_parameter("tracked_topic").value
        self.lm3d_suffix = self.get_parameter("landmarks3d_suffix").value
        self.plane_suffix = self.get_parameter("plane_suffix").value
        self.plane_indices = [int(x) for x in self.get_parameter("plane_indices").value]

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
        self.mesh_marker_ns = self.get_parameter("mesh_marker_ns").value
        self.mesh_marker_id = int(self.get_parameter("mesh_marker_id").value)
        self.mesh_rgba = [float(x) for x in self.get_parameter("mesh_rgba").value]
        self.mesh_use_all_valid_points = bool(self.get_parameter("mesh_use_all_valid_points").value)

        self.create_subscription(IdsList, self.tracked_topic, self._cb_tracked, 10)

        self._hands = {}
        self.create_timer(0.25, self._timer_cleanup)

        LOG.info("Palm plane node started.")
        LOG.info(f"tracked_topic: {self.tracked_topic}")
        LOG.info(f"indices plano: {self.plane_indices}")
        LOG.info("Publicando solo plano estable y normal estable.")

    def _cb_tracked(self, msg: IdsList):
        """This callback receives the list of active hand identifiers. For each new hand, it creates the required ROS subscribers and publishers. For existing hands, it updates the last-seen timestamp used to detect stale hands.

        Parameters
        ----------
        msg : IdsList
            Message containing the identifiers of the currently tracked hands.

        Returns
        -------
        None
        """
        now = time.time()
        for hand_id in msg.ids:
            if hand_id not in self._hands:
                self._add_hand(hand_id)
            self._hands[hand_id]["last_seen"] = now

    def _add_hand(self, hand_id: str):
        """This function creates the subscriber for the hand's 3D landmarks and the publishers used to publish the estimated palm plane and optional RViz visualization markers. It also initializes the internal state required for plane filtering and last-good-plane caching.

        Parameters
        ----------
        hand_id : str
            Identifier of the hand for which the processing pipeline is created.

        Returns
        -------
        None
        """
        topic_in = f"/humans/hands/{hand_id}/{self.lm3d_suffix}"
        topic_plane = f"/humans/hands/{hand_id}/{self.plane_suffix}"

        sub = self.create_subscription(
            HandLandmarks3D,
            topic_in,
            lambda m, hid=hand_id: self._cb_hand3d(hid, m),
            10,
        )

        pub_plane = self.create_publisher(PlaneStamped, topic_plane, 10)

        pub_centroid = None
        pub_plane_normal = None
        pub_mesh = None

        if self.publish_markers:
            pub_centroid = self.create_publisher(
                Marker,
                f"/humans/hands/{hand_id}/palm_plane_centroid_marker",
                10,
            )
            pub_plane_normal = self.create_publisher(
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
            "sub": sub,
            "pub_plane": pub_plane,
            "pub_centroid": pub_centroid,
            "pub_plane_normal": pub_plane_normal,
            "pub_mesh": pub_mesh,
            "last_seen": time.time(),
            "last_good_plane": None,
            "n_plane_filt": None,
        }

        LOG.info(f"[+] hand {hand_id}: sub={topic_in} pub={topic_plane}")

    def _timer_cleanup(self):
        """Remove stale hand entries from the internal tracking state. This timer callback checks the last-seen timestamp of each tracked hand and removes hands that have not been updated within the configured timeout.

        Returns
        -------
        None
            This function does not return a value. It updates the internal hand dictionary as a side effect.
        """
        now = time.time()
        dead = []
        for hid, st in self._hands.items():
            if (now - st["last_seen"]) > self.stale_hand_seconds:
                dead.append(hid)

        for hid in dead:
            LOG.info(f"[-] removing stale hand {hid}")
            self._hands.pop(hid, None)

    def _point_valid(self, msg: HandLandmarks3D, idx: int) -> bool:
        """
        Check whether a hand landmark index contains a valid point.

        Parameters
        ----------
        msg : HandLandmarks3D
            Message containing 3D hand landmarks and optional validity flags.

        idx : int
            Index of the landmark to validate.

        Returns
        -------
        bool
            True if the landmark index is within range and marked as valid, otherwise False.
        """
        if idx < 0 or idx >= len(msg.landmarks):
            return False
        if hasattr(msg, "valid") and len(msg.valid) == len(msg.landmarks):
            return bool(msg.valid[idx])
        return True

    def _point_np(self, msg: HandLandmarks3D, idx: int) -> np.ndarray:
        """
        Convert a 3D hand landmark point into a NumPy array.

        Parameters
        ----------
        msg : HandLandmarks3D
            Message containing 3D hand landmarks.

        idx : int
            Index of the landmark to convert.

        Returns
        -------
        np.ndarray
            NumPy array of shape `(3,)` containing the landmark coordinates.
        """
        p = msg.landmarks[idx]
        return np.array([float(p.x), float(p.y), float(p.z)], dtype=np.float64)

    def _extract_plane_points(self, msg: HandLandmarks3D) -> np.ndarray:
        """Extract valid 3D landmarks used for palm plane fitting.

        Parameters
        ----------
        msg : HandLandmarks3D
            Message containing 3D hand landmarks.

        Returns
        -------
        np.ndarray
            Array of valid palm plane points with shape (N, 3).
        """
        pts = []
        for idx in self.plane_indices:
            if not self._point_valid(msg, idx):
                continue
            p = self._point_np(msg, idx)
            if np.all(np.isfinite(p)):
                pts.append(p)
        return np.array(pts, dtype=np.float64)

    def _extract_mesh_points(self, msg: HandLandmarks3D, pts_plane: np.ndarray) -> np.ndarray:
        """
        Extract 3D landmark points used for palm mesh generation.

        Parameters
        ----------
        msg : HandLandmarks3D
            Message containing 3D hand landmarks.

        pts_plane : np.ndarray
            Points already selected for palm plane estimation.

        Returns
        -------
        np.ndarray
            Array of valid 3D points used to build the palm mesh.
        """
        if not self.mesh_use_all_valid_points:
            return pts_plane

        pts = []
        for i in range(len(msg.landmarks)):
            if not self._point_valid(msg, i):
                continue
            p = self._point_np(msg, i)
            if np.all(np.isfinite(p)):
                pts.append(p)

        return np.array(pts, dtype=np.float64)

    def _orient_svd_for_stable_plane(self, st: dict, n_svd: np.ndarray):
        """Orient an SVD-estimated plane normal consistently over time.

        Parameters
        ----------
        st : dict
            Internal per-hand state containing the filtered plane normal.

        n_svd : np.ndarray
            Raw plane normal estimated from SVD.

        Returns
        -------
        Optional[np.ndarray]
            Oriented unit normal vector, or `None` if the input normal is invalid.
        """
        n = normalize(n_svd)
        if n is None:
            return None

        if st["n_plane_filt"] is None:
            if self.plane_initial_face_camera and n[2] > 0.0:
                n = -n
            return n

        if self.plane_flip_guard_enable and float(np.dot(st["n_plane_filt"], n)) < 0.0:
            n = -n

        return n

    def _filter_plane_normal(self, st: dict, n_plane_raw: np.ndarray):
        """
        Filter and stabilize a palm plane normal vector.

        The function applies optional angular step limiting and exponential moving average filtering to reduce jitter in the estimated plane normal.

        Parameters
        ----------
        st : dict
            Internal per-hand state containing the previous filtered normal.

        n_plane_raw : np.ndarray
            Raw oriented plane normal vector.

        Returns
        -------
        Optional[np.ndarray]
            Filtered unit normal vector, or `None` if the input normal is invalid.
        """
        n = normalize(n_plane_raw)
        if n is None:
            return None

        if st["n_plane_filt"] is None:
            st["n_plane_filt"] = n.copy()
            return st["n_plane_filt"]

        n_use = n.copy()

        if self.plane_limit_step_enable and self.plane_max_step_deg > 0.0:
            n_use = clamp_angle_step(st["n_plane_filt"], n_use, self.plane_max_step_deg)

        if self.plane_ema_enable:
            a = float(np.clip(self.plane_ema_alpha, 0.0, 1.0))
            st["n_plane_filt"] = normalize(a * n_use + (1.0 - a) * st["n_plane_filt"])
        else:
            st["n_plane_filt"] = n_use

        return st["n_plane_filt"]

    def _cb_hand3d(self, hand_id: str, msg: HandLandmarks3D):
        """
        This callback extracts the configured palm landmarks, estimates a plane using SVD-based fitting, stabilizes the plane normal over time, validates the fit quality, and publishes the resulting plane message.

        If enabled, the function also publishes RViz visualization markers for:
        - The palm centroid
        - The stabilized normal vector
        - A triangulated palm mesh surface

        The estimated plane can optionally be rejected based on RMSE or inlier count thresholds. In that case, the last valid plane estimate may be
        republished if configured.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand associated with the incoming landmarks.

        msg : HandLandmarks3D
            Message containing the 3D hand landmarks and optional validity flags.

        Returns
        -------
        None
            This function does not return a value. It publishes plane estimates and visualization markers as a side effect.
        """
        st = self._hands.get(hand_id)
        if st is None:
            return

        pts = self._extract_plane_points(msg)
        if pts.shape[0] < 3:
            return

        try:
            n_svd, _, c, rmse_m, nin = fit_plane_svd(pts)
        except Exception:
            return

        n_plane_raw = self._orient_svd_for_stable_plane(st, n_svd)
        if n_plane_raw is None:
            return

        n_plane = self._filter_plane_normal(st, n_plane_raw)
        if n_plane is None:
            return

        d_plane = -float(np.dot(n_plane, c))
        rmse_mm = float(rmse_m * 1000.0)

        accepted = (
            nin >= self.min_inliers
            and np.isfinite(rmse_mm)
            and rmse_mm <= self.rmse_accept_mm
        )

        reject_reasons = []
        if nin < self.min_inliers:
            reject_reasons.append(f"inliers<{self.min_inliers} (got={nin})")
        if (not np.isfinite(rmse_mm)) or rmse_mm > self.rmse_accept_mm:
            reject_reasons.append(f"rmse>{self.rmse_accept_mm:.1f}mm (got={rmse_mm:.2f})")

        plane_to_pub = {
            "n": n_plane,
            "d": d_plane,
            "c": c,
            "rmse_m": rmse_m,
            "rmse_mm": rmse_mm,
            "nin": nin,
        }

        if accepted:
            st["last_good_plane"] = plane_to_pub
        elif self.publish_last_good_if_rejected and st["last_good_plane"] is not None:
            plane_to_pub = st["last_good_plane"]

        out = PlaneStamped()
        out.header = msg.header
        out.normal = Vector3(
            x=float(plane_to_pub["n"][0]),
            y=float(plane_to_pub["n"][1]),
            z=float(plane_to_pub["n"][2]),
        )
        out.d = float(plane_to_pub["d"])
        out.centroid = Point(
            x=float(plane_to_pub["c"][0]),
            y=float(plane_to_pub["c"][1]),
            z=float(plane_to_pub["c"][2]),
        )
        out.rmse_m = float(plane_to_pub["rmse_m"])
        out.rmse_mm = float(plane_to_pub["rmse_mm"])
        out.n_inliers = int(plane_to_pub["nin"])
        out.fit_method = "svd_stable_plane"
        out.accepted = bool(accepted)
        out.reject_reasons = [str(x) for x in reject_reasons]
        st["pub_plane"].publish(out)

        if self.publish_markers:
            if st["pub_centroid"] is not None:
                st["pub_centroid"].publish(
                    make_centroid_marker(
                        frame_id=msg.header.frame_id,
                        stamp=msg.header.stamp,
                        p=np.array(plane_to_pub["c"], dtype=float),
                        ns=f"{hand_id}_palm_plane_centroid",
                    )
                )

            if st["pub_plane_normal"] is not None:
                st["pub_plane_normal"].publish(
                    make_normal_marker(
                        frame_id=msg.header.frame_id,
                        stamp=msg.header.stamp,
                        p=np.array(plane_to_pub["c"], dtype=float),
                        n=np.array(plane_to_pub["n"], dtype=float),
                        ns=f"{hand_id}_palm_plane_normal_stable",
                        length=self.normal_marker_length_m,
                        rgba=(0.0, 1.0, 1.0, 0.95),
                    )
                )

        if self.publish_markers and self.publish_mesh_marker and st.get("pub_mesh") is not None:
            try:
                pts_mesh = self._extract_mesh_points(msg, pts_plane=pts)
                if pts_mesh.shape[0] < 3:
                    return

                n_mesh = np.array(plane_to_pub["n"], dtype=np.float64)
                d_mesh = float(plane_to_pub["d"])

                tol_m = 0.02
                dist = pts_mesh @ n_mesh + d_mesh
                keep = np.isfinite(dist) & (np.abs(dist) <= tol_m)
                pts_mesh = pts_mesh[keep]
                if pts_mesh.shape[0] < 3:
                    return

                pts_proj = project_points_to_plane(pts_mesh, n_mesh, d_mesh)
                u, v = plane_basis_from_normal(n_mesh)
                pts2 = to_plane_2d(pts_proj, u, v)

                hull_idx = convex_hull_indices_2d(pts2)
                hull3 = pts_proj[hull_idx]

                triangles = triangulate_convex_polygon(hull3.shape[0])

                mh = Header()
                mh.stamp = msg.header.stamp
                mh.frame_id = msg.header.frame_id

                rgba = self.mesh_rgba
                st["pub_mesh"].publish(
                    make_mesh_marker(
                        mh,
                        vertices=hull3,
                        triangles=triangles,
                        rgba=(rgba[0], rgba[1], rgba[2], rgba[3]),
                        marker_ns=self.mesh_marker_ns,
                        marker_id=self.mesh_marker_id,
                    )
                )
            except Exception as e:
                LOG.warning(f"[mesh] failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PalmPlaneFromHand3DNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()