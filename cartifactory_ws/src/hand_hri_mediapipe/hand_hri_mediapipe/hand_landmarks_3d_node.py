#!/usr/bin/env python3
"""
ROS2 node that tracks the hand_ids dinamically, amd for each of them synchronizes:
 - /humans/hands/<id>/landmarks2d   (HandLandmarks2D)
 - depth (aligned_depth_to_color)   (sensor_msgs/Image)
 - camera_info                      (sensor_msgs/CameraInfo)

Notes:
- The landmarks mapping is MediaPipe Hands standard (0..20).
- If a landmark falls out of the hand or has invalid depth, it is set as invalid (and not published)

Published topics:
 - /humans/hands/<id>/landmarks3d_marker  (Marker SPHERE_LIST)
 - /humans/hands/<id>/skeleton3d_marker   (Marker LINE_LIST)
 - (opcional) /humans/hands/<id>/landmarks3d_cloud (PointCloud2)

Author:
    CARTIF Technology Centre
"""

import math
import threading
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from message_filters import Subscriber, ApproximateTimeSynchronizer

from hri_msgs.msg import IdsList
from hand_hri_mediapipe_msgs.msg import HandLandmarks2D , HandLandmarks3D

import logging
from rich.logging import RichHandler


logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler(rich_tracebacks=True)],
    force=True,
)
LOG = logging.getLogger("hand_mediapipe")


# --- Standard connections, MediaPipe style (similar to mp_hands.HAND_CONNECTIONS) ---
HAND_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 4),          # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),          # index
    (0, 9), (9, 10), (10, 11), (11, 12),     # middle
    (0, 13), (13, 14), (14, 15), (15, 16),   # ring
    (0, 17), (17, 18), (18, 19), (19, 20),   # pinky
    (5, 9), (9, 13), (13, 17)                # palm arcs
]


def _make_cloud_xyz(header: Header, pts_xyz: np.ndarray) -> PointCloud2:
    """This function creates a PointCloud message using the XYZ points

    Parameters
    ----------
    header : Header
        Header of msg
    pts_xyz : np.ndarray
        Points to be used for point cloud creation

    Returns
    -------
    PointCloud2
        ROS2 PointCloud2 message
    """
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(pts_xyz.shape[0])
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

    pts32 = pts_xyz.astype(np.float32, copy=False)
    msg.data = pts32.tobytes()
    return msg


class Hand3DProjectorNode(Node):
    """ROS2 node that projects 2D hand landmarks into 3D space using synchronized depth images and camera intrinsic parameters.

    The node dynamically creates processing pipelines for tracked hands, converts normalized 2D landmark coordinates into 3D camera-frame coordinates, and publishes:
        - 3D hand landmarks
        - RViz visualization markers
        - Optional point clouds

    """
    def __init__(self):
        super().__init__("hand_landmarks_3d_projector")

        # ---------- Params ----------
        self.declare_parameter("tracked_topic", "/humans/hands/tracked")
        self.declare_parameter("depth_topic", "/camera/cameraD455/aligned_depth_to_color/image_raw")
        self.declare_parameter("caminfo_topic", "/camera/cameraD455/aligned_depth_to_color/camera_info")

        # Publicación
        self.declare_parameter("publish_pointcloud", False)

        # Normalización / validación
        self.declare_parameter("norm_margin", 0.05)          # admite un poco fuera antes de descartar
        self.declare_parameter("clamp_norm", True)           # clamp a [0,1] si está dentro del margen
        self.declare_parameter("min_depth_m", 0.05)
        self.declare_parameter("max_depth_m", 2.0)
        self.declare_parameter("depth_hole_fill_radius", 2)  # 0 desactiva; 2 => ventana 5x5

        # Hand parameters
        self.declare_parameter("stale_hand_seconds", 1.0)    # If no  X s, se elimina

        # Marker style
        self.declare_parameter("sphere_diameter", 0.012)     # metros
        self.declare_parameter("line_width", 0.004)          # metros
        self.declare_parameter("points_alpha", 0.9)
        self.declare_parameter("lines_alpha", 0.85)

        self.tracked_topic = self.get_parameter("tracked_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.caminfo_topic = self.get_parameter("caminfo_topic").value
        self.publish_pointcloud = bool(self.get_parameter("publish_pointcloud").value)

        self.norm_margin:float = float(self.get_parameter("norm_margin").value)
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

        # tracked subscriber
        self.create_subscription(IdsList, self.tracked_topic, self._cb_tracked, 10)

        # Estado: pipelines por mano
        self._lock = threading.Lock()
        self._hands: Dict[str, dict] = {}  # hand_id -> { 'ts', 'sub_lm', 'sub_d', 'sub_ci', pubs..., last_seen_sec }

        # Limpieza periódica de manos stale
        self.create_timer(0.25, self._cleanup_timer)

        LOG.info("Hand 3D projector node started.")
        LOG.info(f"  tracked: {self.tracked_topic}")
        LOG.info(f"  depth:   {self.depth_topic}")
        LOG.info(f"  caminfo: {self.caminfo_topic}")
        LOG.info(f"  publish_pointcloud={self.publish_pointcloud}")

    # -------------------- Dinámica: alta/baja de manos --------------------
    def _cb_tracked(self, msg: IdsList):
        """This callback receives the list of active hand identifiers and updates their last-seen timestamps. If a newly detected hand does not yet have an associated  processing pipeline, a new pipeline is created dynamically.

        Parameters
        ----------
        msg : IdsList
            Message containing the identifiers of the currently tracked hands.

        Returns
        -------
        None
        """
            
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        ids = list(msg.ids)

        with self._lock:
            # marcar last_seen
            for hid in ids:
                if hid not in self._hands:
                    self._create_hand_pipeline(hid)
                self._hands[hid]["last_seen_sec"] = now_sec

    def _cleanup_timer(self):
        """This timer callback periodically checks the last-seen timestamp of alltracked hands and removes the pipelines associated with hands that have not been updated within the configured timeout period.

        Returns
        -------
        None
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        to_remove = []

        with self._lock:
            for hid, h in self._hands.items():
                last_seen = float(h.get("last_seen_sec", 0.0))
                if (now_sec - last_seen) > self.stale_hand_seconds:
                    to_remove.append(hid)

            for hid in to_remove:
                self._destroy_hand_pipeline(hid)

        if to_remove:
            LOG.info(f"Removed stale hands: {to_remove}")

    def _create_hand_pipeline(self, hand_id: str):
        """This function creates the publishers, subscribers, and approximate time synchronizer required to process one tracked hand. The pipeline subscribes to the hand's 2D landmarks, synchronized depth image, and camera information, then registers a synchronized callback to generate 3D landmarks and visualization outputs.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand for which the processing pipeline is created.

        Returns
        -------
        None
        """
        lm_topic = f"/humans/hands/{hand_id}/landmarks2d"

        # Publishers
        pub_pts = self.create_publisher(Marker, f"/humans/hands/{hand_id}/landmarks3d_marker", 10)
        pub_skel = self.create_publisher(Marker, f"/humans/hands/{hand_id}/skeleton3d_marker", 10)
        pub_lm3d = self.create_publisher(HandLandmarks3D,f"/humans/hands/{hand_id}/landmarks3d",10)
        pub_cloud = None
        if self.publish_pointcloud:
            pub_cloud = self.create_publisher(PointCloud2, f"/humans/hands/{hand_id}/landmarks3d_cloud", 10)

        # Subscribers (se duplican depth/caminfo por mano: ok para 1-2 manos)
        sub_lm = Subscriber(self, HandLandmarks2D, lm_topic)
        sub_d = Subscriber(self, Image, self.depth_topic)
        sub_ci = Subscriber(self, CameraInfo, self.caminfo_topic)

        ts = ApproximateTimeSynchronizer([sub_lm, sub_d, sub_ci], queue_size=10, slop=0.12)
        ts.registerCallback(lambda lm, d, ci: self._cb_sync(hand_id, lm, d, ci))

        self._hands[hand_id] = {
            "sub_lm": sub_lm,
            "sub_d": sub_d,
            "sub_ci": sub_ci,
            "ts": ts,
            "pub_lm3d": pub_lm3d,
            "pub_pts": pub_pts,
            "pub_skel": pub_skel,
            "pub_cloud": pub_cloud,
            "last_seen_sec": self.get_clock().now().nanoseconds * 1e-9,
            "lm_topic": lm_topic,
        }

        LOG.info(f"[{hand_id}] pipeline created. Sub: {lm_topic}")

    def _destroy_hand_pipeline(self, hand_id: str):
        """This function removes the subscribers and internal state associated with the specified hand identifier. The corresponding publishers are not explicitly destroyed because they are automatically cleaned up when the ROS2 node is destroyed.

        Parameters
        ----------
        hand_id : str
            Identifier of the hand whose processing pipeline should be removed.

        Returns
        -------
        None
        """
        h = self._hands.get(hand_id)
        if h is None:
            return

        # message_filters.Subscriber tiene el subscription real en .sub (si existe).
        # Hacemos best-effort de destroy_subscription.
        for key in ["sub_lm", "sub_d", "sub_ci"]:
            try:
                sub = h[key]
                if hasattr(sub, "sub") and sub.sub is not None:
                    self.destroy_subscription(sub.sub)
            except Exception:
                pass

        # Publishers se destruyen con el nodo al final; no hace falta destruir explícitamente
        del self._hands[hand_id]
        LOG.info(f"[{hand_id}] pipeline destroyed.")

    # -------------------- Proyección 2D->3D --------------------
    def _norm_to_pixel(self, xn: float, yn: float, W: int, H: int) -> Optional[Tuple[int, int]]:
        """The function validates the normalized coordinates using the configured margin limits and optionally clamps them into the valid `[0, 1]` range before converting them into integer image pixel coordinates.

        Parameters
        ----------
        xn : float
            Normalized horizontal image coordinate.

        yn : float
            Normalized vertical image coordinate.

        W : int
            Image width in pixels.

        H : int
            Image height in pixels.

        Returns
        -------
        Optional[Tuple[int, int]]
            Tuple `(u, v)` containing the pixel coordinates if the normalized coordinates are valid. Returns `None` if the coordinates fall outside the accepted image region.
        """
        # validar margen
        if not (-self.norm_margin <= xn <= 1.0 + self.norm_margin and -self.norm_margin <= yn <= 1.0 + self.norm_margin):
            return None

        if self.clamp_norm:
            xn = min(1.0, max(0.0, float(xn)))
            yn = min(1.0, max(0.0, float(yn)))

        u = int(round(xn * (W - 1)))
        v = int(round(yn * (H - 1)))

        if u < 0 or u >= W or v < 0 or v >= H:
            return None
        return u, v

    def _depth_at(self, depth_img: np.ndarray, u: int, v: int) -> float:
        """This function extracts the depth value at pixel coordinates (u, v) and returns it in meters. It supports common depth image formats such as uint16, where depth is usually encoded in millimeters, and floating-point images, where depth is expected to already be in meters.

        If the depth value at the requested pixel is invalid, the function can optionally search a local neighborhood around the pixel and use the median of valid depth values as a robust hole-filling strategy.

        Parameters
        ----------
        depth_img : np.ndarray
            Depth image as a NumPy array.

        u : int
            Horizontal pixel coordinate.

        v : int
            Vertical pixel coordinate.

        Returns
        -------
        float
            Valid depth value in meters. Returns `NaN` if no valid depth value is found or if the depth is outside the configured valid range.
        """
        H, W = depth_img.shape[:2]

        def read_raw(x: int, y: int) -> float:
            val = depth_img[y, x]
            if depth_img.dtype == np.uint16:
                return float(val) * 0.001
            else:
                return float(val)

        z = read_raw(u, v)
        if (not math.isfinite(z)) or z <= 0.0:
            r = max(0, int(self.depth_hole_fill_radius))
            if r <= 0:
                return float("nan")

            x0 = max(0, u - r)
            x1 = min(W - 1, u + r)
            y0 = max(0, v - r)
            y1 = min(H - 1, v + r)

            vals = []
            for yy in range(y0, y1 + 1):
                for xx in range(x0, x1 + 1):
                    zz = read_raw(xx, yy)
                    if math.isfinite(zz) and zz > 0.0:
                        vals.append(zz)

            if not vals:
                return float("nan")

            # robust: mediana (evita outliers)
            z = float(np.median(np.array(vals, dtype=np.float32)))

        if (not math.isfinite(z)) or z < self.min_depth_m or z > self.max_depth_m:
            return float("nan")

        return z

    def _deproject(self, u: int, v: int, z: float, ci: CameraInfo) -> np.ndarray:
        """Deproject a 2D image pixel with depth into a 3D point in the camera frame.

        This function uses the pinhole camera model and the intrinsic camera parameters contained in the `CameraInfo` message to convert image coordinates (u, v) and depth z into Cartesian coordinates (x, y, z) in the camera reference frame.

        Parameters
        ----------
        u : int
            Horizontal pixel coordinate in the image.

        v : int
            Vertical pixel coordinate in the image.

        z : float
            Depth value in meters associated with the pixel.

        ci : CameraInfo
            Camera intrinsic parameters used for deprojection.

        Returns
        -------
        np.ndarray
            A NumPy array of shape `(3,)` containing the 3D point
            coordinates `[x, y, z]` in the camera frame.
        """
        #Pinhole: (u,v,z)->(x,y,z) en frame de la cámara.
        fx = float(ci.k[0])
        fy = float(ci.k[4])
        cx = float(ci.k[2])
        cy = float(ci.k[5])

        x = (float(u) - cx) / fx * z
        y = (float(v) - cy) / fy * z
        return np.array([x, y, z], dtype=np.float64)

    # -------------------- Callback sincronizado --------------------
    def _cb_sync(self, hand_id: str, lm_msg: HandLandmarks2D, depth_msg: Image, ci_msg: CameraInfo):
        """ Process synchronized hand landmarks, depth image, and camera information.
        This callback projects normalized 2D hand landmarks into 3D camera-frame coordinates using the aligned depth image and camera intrinsic parameters.
        It then publishes the resulting 3D landmarks, visualization markers, and optionally a point cloud containing the valid projected landmarks.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand associated with this synchronized data.

        lm_msg : HandLandmarks2D
            Message containing the normalized 2D hand landmarks, hand metadata, and timestamp.

        depth_msg : Image
            Aligned depth image used to obtain the depth value for each landmark.

        ci_msg : CameraInfo
            Camera intrinsic parameters used to deproject image pixels into 3D camera-frame coordinates.

        Returns
        -------
        None
        """
        # Convertir depth a numpy
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding=depth_msg.encoding)
        except Exception as e:
            LOG.warning(f"[{hand_id}] depth cv conversion failed: {e}")
            return

        if depth_img is None or depth_img.ndim < 2:
            return

        H, W = depth_img.shape[:2]

        # Proyectar los 21 landmarks
        pts3d: List[Optional[np.ndarray]] = [None] * 21

        # NOTA: lm_msg.landmarks es array [21] de NormalizedPointOfInterest2D
        for i in range(21):
            xn = float(lm_msg.landmarks[i].x)
            yn = float(lm_msg.landmarks[i].y)

            uv = self._norm_to_pixel(xn, yn, W, H)
            if uv is None:
                pts3d[i] = None
                continue

            u, v = uv
            z = self._depth_at(depth_img, u, v)
            if not math.isfinite(z):
                pts3d[i] = None
                continue

            pts3d[i] = self._deproject(u, v, z, ci_msg)

        # Header: usa el tiempo de adquisición del landmark (y frame de depth/cam)
        header = Header()
        header.stamp = lm_msg.header.stamp
        header.frame_id = depth_msg.header.frame_id if depth_msg.header.frame_id else lm_msg.header.frame_id

        # Publicar markers (puntos y esqueleto)
        self._publish_markers(hand_id, header, pts3d)
        
        lm3d_msg = HandLandmarks3D()
        lm3d_msg.header = header
        lm3d_msg.id = lm_msg.id               # viene en HandLandmarks2D
        lm3d_msg.handedness = lm_msg.handedness
        lm3d_msg.confidence = float(getattr(lm_msg, "confidence", 0.0))  # si existe

        # Rellenar landmarks + valid
        for i in range(21):
            p = pts3d[i]
            if p is None:
                lm3d_msg.valid[i] = False
                lm3d_msg.landmarks[i].x = float("nan")
                lm3d_msg.landmarks[i].y = float("nan")
                lm3d_msg.landmarks[i].z = float("nan")
            else:
                lm3d_msg.valid[i] = True
                lm3d_msg.landmarks[i].x = float(p[0])
                lm3d_msg.landmarks[i].y = float(p[1])
                lm3d_msg.landmarks[i].z = float(p[2])

        with self._lock:
            pub_lm3d = self._hands.get(hand_id, {}).get("pub_lm3d", None)
        if pub_lm3d is not None:
            pub_lm3d.publish(lm3d_msg)

        # Opcional: nube
        with self._lock:
            pub_cloud = self._hands.get(hand_id, {}).get("pub_cloud", None)
        if pub_cloud is not None:
            valid_pts = np.array([p for p in pts3d if p is not None], dtype=np.float64)
            pub_cloud.publish(_make_cloud_xyz(header, valid_pts))

    def _publish_markers(self, hand_id: str, header: Header, pts3d: List[Optional[np.ndarray]]):
        """Publish RViz markers for projected 3D hand landmarks and skeleton edges.
        This function creates and publishes two visualization markers: a sphere list for valid 3D landmark points and a line list for the hand skeleton connections. Invalid landmarks represented by None are skipped.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand whose markers are being published.

        header : Header
            ROS message header used to timestamp and frame the visualization markers.

        pts3d : List[Optional[np.ndarray]]
            List of 3D landmark points in the camera frame. Each valid point is expected to be a NumPy array of shape `(3,)`; invalid landmarks should be represented as `None`.

        Returns
        -------
        None
        """
        with self._lock:
            h = self._hands.get(hand_id)
            if h is None:
                return
            pub_pts = h["pub_pts"]
            pub_skel = h["pub_skel"]

        # ---- Marker puntos (SPHERE_LIST) ----
        m_pts = Marker()
        m_pts.header = header
        m_pts.ns = f"{hand_id}_landmarks"
        m_pts.id = 0
        m_pts.type = Marker.SPHERE_LIST
        m_pts.action = Marker.ADD
        m_pts.pose.orientation.w = 1.0
        m_pts.scale.x = self.sphere_diameter
        m_pts.scale.y = self.sphere_diameter
        m_pts.scale.z = self.sphere_diameter
        m_pts.color = ColorRGBA(r=0.0, g=0.2, b=1.0, a=float(self.points_alpha))

        pts_list = []
        for p in pts3d:
            if p is None:
                continue
            pts_list.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2])))
        m_pts.points = pts_list
        pub_pts.publish(m_pts)

        # ---- Marker esqueleto (LINE_LIST) ----
        m_ln = Marker()
        m_ln.header = header
        m_ln.ns = f"{hand_id}_skeleton"
        m_ln.id = 0
        m_ln.type = Marker.LINE_LIST
        m_ln.action = Marker.ADD
        m_ln.pose.orientation.w = 1.0
        m_ln.scale.x = self.line_width
        m_ln.color = ColorRGBA(r=0.2, g=1.0, b=1.0, a=float(self.lines_alpha))

        line_pts = []
        for a, b in HAND_EDGES:
            pa = pts3d[a]
            pb = pts3d[b]
            if pa is None or pb is None:
                continue
            line_pts.append(Point(x=float(pa[0]), y=float(pa[1]), z=float(pa[2])))
            line_pts.append(Point(x=float(pb[0]), y=float(pb[1]), z=float(pb[2])))
        m_ln.points = line_pts
        pub_skel.publish(m_ln)


def main(args=None):
    rclpy.init(args=args)
    node = Hand3DProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()