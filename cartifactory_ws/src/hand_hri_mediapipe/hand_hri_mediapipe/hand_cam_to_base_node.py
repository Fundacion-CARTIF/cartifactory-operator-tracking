#!/usr/bin/env python3
"""
ROS2 node for transforming hand landmarks and palm plane estimations from the camera reference frame into the robot base reference frame.

Using the calibrated transformation between the robot TCP and the camera, the node computes the transformation from camera coordinates to the robot base frame and republishes:

- Hand landmarks in the base frame
- Palm plane estimations in the base frame
- Optional RViz visualization markers for palm centroid and normal vector

Published topics:
- /humans/hands/<id>/landmarks3d_base
- /humans/hands/<id>/palm_plane_base
- Optional visualization markers

Author:
    CARTIF Technology Centre

"""

import time
import numpy as np
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Vector3, Point

from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

from scipy.spatial.transform import Rotation as R

from hri_msgs.msg import IdsList
from hand_hri_mediapipe_msgs.msg import PlaneStamped
from hand_hri_mediapipe_msgs.msg import HandLandmarks3D


def normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
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
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    return v / n if n > eps else v


def angle_between_normals_deg(n1: np.ndarray, n2: np.ndarray) -> float:
    """Function that calculates the angle between two normal vectors in degrees.

    Parameters
    ----------
    n1 : np.ndarray
        First normal vector.
    n2 : np.ndarray
        Second normal vector.
    Returns
    -------
    float
        Angle between the two normal vectors in degrees.
    """
    n1 = normalize(n1)
    n2 = normalize(n2)
    dot = float(np.clip(np.dot(n1, n2), -1.0, 1.0))
    return float(np.degrees(np.arccos(dot)))


def clamp_angle_step(n_prev: np.ndarray, n_new: np.ndarray, max_deg: float) -> np.ndarray:
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
    n_prev = normalize(n_prev)
    n_new = normalize(n_new)

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


def make_centroid_marker(frame_id: str, stamp, p: np.ndarray, ns: str, marker_id: int = 0) -> Marker:
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
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.pose.position.x = float(p[0])
    m.pose.position.y = float(p[1])
    m.pose.position.z = float(p[2])
    m.scale.x = 0.03
    m.scale.y = 0.03
    m.scale.z = 0.03
    m.color = ColorRGBA(r=0.0, g=0.6, b=1.0, a=0.9)
    return m


def make_normal_marker(frame_id: str, stamp, p: np.ndarray, n: np.ndarray, ns: str, marker_id: int = 0) -> Marker:
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

    m = Marker()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = marker_id
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.01
    m.scale.y = 0.02
    m.scale.z = 0.02
    m.color = ColorRGBA(r=1.0, g=0.75, b=0.0, a=0.95)

    p0 = np.asarray(p, dtype=float).reshape(3)
    p1 = p0 + 0.10 * normalize(n)

    m.points = [
        Point(x=float(p0[0]), y=float(p0[1]), z=float(p0[2])),
        Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])),
    ]
    return m

class HandCamToBaseNode(Node):
    """ROS2 node that transforms hand landmarks and palm plane estimations from the camera reference frame into the  robot base reference frame, using the calibrated transformation between the robot TCP and the camera."""
    def __init__(self):
        super().__init__("hand_cam_to_base_node")

        # ROS2 Parameters declaration
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
        # Get the parameter values
        self.tracked_topic = self.get_parameter("tracked_topic").value
        self.tcp_pose_topic = self.get_parameter("tcp_pose_topic").value
        self.frame_out: str = str(self.get_parameter("frame_out").value)
        self.landmarks3d_suffix_in = self.get_parameter("landmarks3d_suffix_in").value
        self.landmarks3d_suffix_out = self.get_parameter("landmarks3d_suffix_out").value
        self.plane_suffix_in = self.get_parameter("plane_suffix_in").value
        self.plane_suffix_out = self.get_parameter("plane_suffix_out").value
        self.filter_plane_normal: bool = bool(self.get_parameter("filter_plane_normal").value)
        self.filter_alpha: float = float(self.get_parameter("filter_alpha").value)
        self.limit_step_enable: bool = bool(self.get_parameter("limit_step_enable").value)
        self.max_step_deg: float = float(self.get_parameter("max_step_deg").value)
        self.flip_guard_enable: bool = bool(self.get_parameter("flip_guard_enable").value)
        self.stale_hand_seconds: float = float(self.get_parameter("stale_hand_seconds").value)
        self.publish_markers: bool = bool(self.get_parameter("publish_markers").value)

        T_list = self.get_parameter("T_tcp_cam").value
        self.T_tcp_cam = np.array(T_list, dtype=float).reshape((4, 4))
        self.R_tcp_cam = self.T_tcp_cam[:3, :3]
        self.t_tcp_cam = self.T_tcp_cam[:3, 3]

        self.latest_tcp_pose = None
        self._hands = {}

        #* -- ROS2 Interfaces
        self.create_subscription(PoseStamped, self.tcp_pose_topic, self.cb_tcp_pose, 10)
        self.create_subscription(IdsList, self.tracked_topic, self.cb_tracked, 10)
        self.create_timer(0.25, self.cleanup_timer)

        self.get_logger().info("Hand cam->base node started")
        self.get_logger().info(f"TCP pose topic: {self.tcp_pose_topic}")
        self.get_logger().info(f"Tracked topic: {self.tracked_topic}")
        self.get_logger().info(f"Output frame: {self.frame_out}")

    def cb_tcp_pose(self, msg: PoseStamped):
        self.latest_tcp_pose = msg

    def cb_tracked(self, msg: IdsList):
        now = time.time()

        for hand_id in msg.ids:
            if hand_id not in self._hands:
                self.add_hand(hand_id)

            self._hands[hand_id]["last_seen"] = now

    def add_hand(self, hand_id: str):
        """This function adds a new hand, creating its subscriptions/publishers.

        Parameters
        ----------
        hand_id : str
            ID of the hand to add. This name will be the one appearing in the topic namespace for the hand.
        """
        topic_lm_in = f"/humans/hands/{hand_id}/{self.landmarks3d_suffix_in}"
        topic_lm_out = f"/humans/hands/{hand_id}/{self.landmarks3d_suffix_out}"

        topic_plane_in = f"/humans/hands/{hand_id}/{self.plane_suffix_in}"
        topic_plane_out = f"/humans/hands/{hand_id}/{self.plane_suffix_out}"

        sub_lm = self.create_subscription(
            HandLandmarks3D,
            topic_lm_in,
            lambda msg, hid=hand_id: self.cb_landmarks3d(hid, msg),
            10,
        )

        sub_plane = self.create_subscription(
            PlaneStamped,
            topic_plane_in,
            lambda msg, hid=hand_id: self.cb_plane(hid, msg),
            10,
        )

        pub_lm = self.create_publisher(HandLandmarks3D, topic_lm_out, 10)
        pub_plane = self.create_publisher(PlaneStamped, topic_plane_out, 10)

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

        self._hands[hand_id] = { # Dictionary to store the interfaces/information of the hand.
            "sub_lm": sub_lm,
            "sub_plane": sub_plane,
            "pub_lm": pub_lm,
            "pub_plane": pub_plane,
            "pub_centroid": pub_centroid,
            "pub_normal": pub_normal,
            "last_seen": time.time(),
            "n_base_filt": None,
        }

        self.get_logger().info(f"[+] {hand_id}")
        self.get_logger().info(f"    {topic_lm_in} -> {topic_lm_out}")
        self.get_logger().info(f"    {topic_plane_in} -> {topic_plane_out}")

    def cleanup_timer(self):
        """This function is called periodically to remove stale hands that have not been seen for a certain amount of time This time is defined as "stale_hand_seconds" parameter from the class.
        """
        now = time.time()
        dead = []

        for hand_id, st in self._hands.items():
            if now - st["last_seen"] > self.stale_hand_seconds: # Checks if the hand has not been detected for more than the threshold
                dead.append(hand_id)

        for hand_id in dead:
            self.get_logger().info(f"[-] removing stale hand {hand_id}")
            self._hands.pop(hand_id, None)

    def get_T_base_cam(self):
        """Compute the transformation from the camera frame to the robot base frame using the current robot TCP pose received from ROS topics and the calibrated transformation between the TCP frame and the camera frame.

        Returns
        -------
        tuple[np.ndarray | None, np.ndarray | None]
            A tuple containing:

            - R_base_cam : np.ndarray
                Rotation matrix (3x3) from camera frame to base frame.

            - t_base_cam : np.ndarray
                Translation vector (3,) from camera frame to base frame.

            Returns `(None, None)` if no TCP pose has been received yet.
        """
        if self.latest_tcp_pose is None:
            return None, None

        tcp = self.latest_tcp_pose
        q = tcp.pose.orientation

        R_base_tcp = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        t_base_tcp = np.array(
            [
                tcp.pose.position.x,
                tcp.pose.position.y,
                tcp.pose.position.z,
            ],
            dtype=float,
        )

        R_base_cam = R_base_tcp @ self.R_tcp_cam
        t_base_cam = R_base_tcp @ self.t_tcp_cam + t_base_tcp

        return R_base_cam, t_base_cam

    def transform_point(self, p_cam: np.ndarray, R_base_cam: np.ndarray, t_base_cam: np.ndarray) -> np.ndarray:
        """This function transforms a point from camera coordinates to base coordinates using the provided rotation matrix and translation vector.

        Parameters
        ----------
        p_cam : np.ndarray
            Point in camera coordinates to be transformed.
        R_base_cam : np.ndarray
            Rotation matrix (3x3) from camera frame to base frame.
        t_base_cam : np.ndarray
            Translation vector (3,) from camera frame to base frame.

        Returns
        -------
        np.ndarray
            Point in base coordinates.
        """
        return R_base_cam @ p_cam + t_base_cam

    def update_filtered_normal(self, hand_id: str, n_base_raw: np.ndarray) -> np.ndarray:
        """Update the filtered normal vector for a given hand.

        Parameters
        ----------
        hand_id : str
            The ID of the hand we are updating.
        n_base_raw : np.ndarray
            The new normal vector in base coordinates that we want to filter and update.

        Returns
        -------
        np.ndarray
            The updated filtered normal vector in base coordinates.
        """
        st = self._hands.get(hand_id)
        if st is None:
            return normalize(n_base_raw)

        n_base = normalize(n_base_raw)

        if st["n_base_filt"] is None:
            st["n_base_filt"] = n_base.copy()
            return st["n_base_filt"]

        n_use = n_base.copy()

        if self.flip_guard_enable and float(np.dot(st["n_base_filt"], n_use)) < 0.0:
            n_use = -n_use

        if self.limit_step_enable and self.max_step_deg > 0.0:
            n_use = clamp_angle_step(st["n_base_filt"], n_use, self.max_step_deg)

        if self.filter_plane_normal:
            a = float(np.clip(self.filter_alpha, 0.0, 1.0))
            st["n_base_filt"] = normalize(a * n_use + (1.0 - a) * st["n_base_filt"])
        else:
            st["n_base_filt"] = n_use

        return st["n_base_filt"]

    def cb_landmarks3d(self, hand_id: str, msg: HandLandmarks3D):
        """Callback function processing 3D hand landmarks messages. It transforms the landmarks from camera coordinates to base coordinates and republishes them.

        Parameters
        ----------
        hand_id : str
            The ID of the hand we are processing.
        msg : HandLandmarks3D
            The 3D hand landmarks message.
        """
        R_base_cam, t_base_cam = self.get_T_base_cam() # Obtain rotation and translation from camera to base frame
        if R_base_cam is None:
            return

        st = self._hands.get(hand_id)
        if st is None:
            return

        out = HandLandmarks3D()
        out.header = Header()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_out

        out.id = msg.id
        out.handedness = msg.handedness
        out.confidence = msg.confidence

        for i in range(21): # Iterates over the 21 hand landmarks
            valid = bool(msg.valid[i])
            out.valid[i] = valid

            if not valid:
                out.landmarks[i].x = float("nan")
                out.landmarks[i].y = float("nan")
                out.landmarks[i].z = float("nan")
                continue

            p = msg.landmarks[i]
            p_cam = np.array([p.x, p.y, p.z], dtype=float)

            if not np.all(np.isfinite(p_cam)):
                out.valid[i] = False
                out.landmarks[i].x = float("nan")
                out.landmarks[i].y = float("nan")
                out.landmarks[i].z = float("nan")
                continue

            p_base = self.transform_point(p_cam=p_cam, 
                                          R_base_cam=R_base_cam, 
                                          t_base_cam=t_base_cam)


            out.landmarks[i].x = float(p_base[0])
            out.landmarks[i].y = float(p_base[1])
            out.landmarks[i].z = float(p_base[2])

        st["pub_lm"].publish(out)

    def cb_plane(self, hand_id: str, msg: PlaneStamped):
        """Callback function processing palm plane estimation messages. It transforms the plane parameters from camera coordinates to base coordinates, applies optional filtering to the normal vector, and republishes the transformed plane.

        Parameters
        ----------
        hand_id : str
            The ID of the hand we are processing.
        msg : PlaneStamped
            The stimation message in camera coordinates.
        """
        R_base_cam, t_base_cam = self.get_T_base_cam() # Obtain rotation and translation from camera to base frame
        if R_base_cam is None or t_base_cam is None:
            return

        st = self._hands.get(hand_id)
        if st is None:
            return

        n_cam = np.array(
            [msg.normal.x, msg.normal.y, msg.normal.z],
            dtype=float,
        )

        c_cam = np.array(
            [msg.centroid.x, msg.centroid.y, msg.centroid.z],
            dtype=float,
        )

        if not np.all(np.isfinite(n_cam)) or not np.all(np.isfinite(c_cam)):
            return

        n_base_raw = normalize(R_base_cam @ n_cam)
        n_base = self.update_filtered_normal(hand_id, n_base_raw)

        c_base = self.transform_point(c_cam, R_base_cam, t_base_cam)

        d_base = -float(np.dot(n_base, c_base))

        out = PlaneStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_out

        out.normal = Vector3(
            x=float(n_base[0]),
            y=float(n_base[1]),
            z=float(n_base[2]),
        )

        out.d = float(d_base)

        out.centroid = Point(
            x=float(c_base[0]),
            y=float(c_base[1]),
            z=float(c_base[2]),
        )

        out.rmse_m = msg.rmse_m
        out.rmse_mm = msg.rmse_mm
        out.n_inliers = msg.n_inliers
        out.fit_method = msg.fit_method + "|cam_to_base"
        out.accepted = bool(msg.accepted)
        out.reject_reasons = list(msg.reject_reasons)

        st["pub_plane"].publish(out)

        if self.publish_markers:
            if st["pub_centroid"] is not None:
                st["pub_centroid"].publish(
                    make_centroid_marker(
                        frame_id=self.frame_out,
                        stamp=msg.header.stamp,
                        p=c_base,
                        ns=f"{hand_id}_palm_plane_centroid_base",
                    )
                )

            if st["pub_normal"] is not None:
                st["pub_normal"].publish(
                    make_normal_marker(
                        frame_id=self.frame_out,
                        stamp=msg.header.stamp,
                        p=c_base,
                        n=n_base,
                        ns=f"{hand_id}_palm_plane_normal_base",
                    )
                )


def main(args=None):
    rclpy.init(args=args)
    node = HandCamToBaseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()