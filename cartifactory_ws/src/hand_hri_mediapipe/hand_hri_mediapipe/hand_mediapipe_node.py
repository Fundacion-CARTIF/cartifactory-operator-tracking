#!/usr/bin/env python3
"""
ROS2 node for real-time hand detection and 2D landmark extraction using MediaPipe Hands.

This node subscribes to RGB camera images, runs MediaPipe hand tracking inference, and publishes:
- Tracked hand identifiers
- Normalized 2D hand landmarks
- Normalized 2D hand bounding boxes (ROIs)
- Optional debug visualization images

The node dynamically creates publishers for each detected hand and supports left/right handedness classification with optional handedness swapping to compensate for mirrored camera streams.

Author:
    CARTIF Technology Centre
"""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from hri_msgs.msg import IdsList, NormalizedRegionOfInterest2D, NormalizedPointOfInterest2D
from hand_hri_mediapipe_msgs.msg import HandLandmarks2D

import mediapipe as mp
import cv2

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


class MediaPipeHandsNode(Node):
    """ROS2 node for real-time hand detection and 2D landmark extraction using MediaPipe Hands.
    This node subscribes to RGB camera images, performs MediaPipe hand tracking inference, and publishes normalized 2D hand landmarks, hand regions of interest (ROIs), tracked hand identifiers, and optional debug images. The node dynamically creates publishers for each detected hand and supports handedness correction for non-mirrored camera streams.
    """
    def __init__(self):
        super().__init__("hand_mediapipe_node")

        # ---------------- Params ----------------
        self.declare_parameter("image_topic", "/camera/cameraD455/color/image_raw")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/humans/hands/debug_image")
        self.declare_parameter("max_hands", 2)
        self.declare_parameter("model_complexity", 1)
        self.declare_parameter("min_detection_confidence", 0.6)
        self.declare_parameter("min_tracking_confidence", 0.6)
        self.declare_parameter("swap_handedness", True)

        self.image_topic = self.get_parameter("image_topic").value
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_topic = self.get_parameter("debug_image_topic").value

        max_hands = int(self.get_parameter("max_hands").value)
        model_complexity = int(self.get_parameter("model_complexity").value)
        min_det = float(self.get_parameter("min_detection_confidence").value)
        min_trk = float(self.get_parameter("min_tracking_confidence").value)
        self.swap_handedness = bool(self.get_parameter("swap_handedness").value)

        self.bridge = CvBridge()

        # ---------------- MediaPipe init (UNA SOLA VEZ) ----------------
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_draw_styles = mp.solutions.drawing_styles

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_hands,
            model_complexity=model_complexity,
            min_detection_confidence=min_det,
            min_tracking_confidence=min_trk,
        )

        # ---------------- Publishers ----------------
        self.pub_tracked = self.create_publisher(IdsList, "/humans/hands/tracked", 10)

        self.pub_debug = None
        if self.publish_debug_image:
            self.pub_debug = self.create_publisher(Image, self.debug_image_topic, 10)

        # Cache de publishers por hand_id (para no crear en cada frame)
        self._roi_pubs = {}
        self._lm_pubs = {}

        # ---------------- Subscriber ----------------
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        LOG.info("MediaPipe Hands HRI node started.")
        LOG.info(f"Sub: {self.image_topic}")
        LOG.info(f"Pub: /humans/hands/tracked")
        LOG.info(f"Debug image: {self.publish_debug_image} -> {self.debug_image_topic}")

    def _get_roi_pub(self, hand_id: str):
        """This function returns a cached publisher for the normalized 2D region of interest (ROI) topic corresponding to the specified hand identifier.
        If the publisher does not already exist, it is created dynamically and stored for future reuse.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand.

        Returns
        -------
        Publisher
            ROS2 publisher used to publish `NormalizedRegionOfInterest2D`
        messages for the specified hand.
        """
        topic = f"/humans/hands/{hand_id}/roi2d"
        if topic not in self._roi_pubs:
            self._roi_pubs[topic] = self.create_publisher(NormalizedRegionOfInterest2D, topic, 10)
        return self._roi_pubs[topic]

    def _get_lm_pub(self, hand_id: str):
        """This function returns a cached publisher for the normalized 2D landmark topic corresponding to the specified hand identifier. If the publisher does not already exist, it is created dynamically and stored for future reuse.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand.

        Returns
        -------
        Publisher
            ROS2 publisher used to publish `HandLandmarks2D` messages for the specified hand.
        """
        topic = f"/humans/hands/{hand_id}/landmarks2d"
        if topic not in self._lm_pubs:
            self._lm_pubs[topic] = self.create_publisher(HandLandmarks2D, topic, 10)
        return self._lm_pubs[topic]

    def image_callback(self, msg: Image):
        """This callback converts the incoming ROS image into an OpenCV image, runs MediaPipe Hands inference, and publishes the detected hand identifiers, normalized 2D landmarks, normalized regions of interest, and optional
        debug visualization image.

        The tracked hand list is published on every callback, even when no hands
        are detected.

        Parameters
        ----------
        msg : Image
            Input RGB image message received from the camera.

        Returns
        -------
        None
        """
        # Convertir a OpenCV
        frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # Inferencia
        results = self.hands.process(frame_rgb)

        # tracked
        ids_msg = IdsList()
        ids_msg.header = msg.header
        ids_msg.ids = []

        # Si hay manos, publicar ROI + landmarks y pintar imagen si procede
        if results.multi_hand_landmarks:
            for hand_landmarks, handedness in zip(
                results.multi_hand_landmarks,
                results.multi_handedness,
            ):
                raw_label = handedness.classification[0].label

                # MediaPipe asume imagen selfie/espejada.
                # Con RealSense normalmente conviene intercambiar Left/Right.
                if self.swap_handedness:
                    if raw_label == "Left":
                        label = "Right"
                    elif raw_label == "Right":
                        label = "Left"
                    else:
                        label = "Unknown"
                else:
                    label = raw_label

                if label == "Right":
                    hand_id = "hand_0"
                    handedness_value = HandLandmarks2D.HANDEDNESS_RIGHT
                elif label == "Left":
                    hand_id = "hand_1"
                    handedness_value = HandLandmarks2D.HANDEDNESS_LEFT
                else:
                    hand_id = "hand_unknown"
                    handedness_value = HandLandmarks2D.HANDEDNESS_UNKNOWN

                ids_msg.ids.append(hand_id)

                # ---------- ROI (normalized bbox) ----------
                xs = [lm.x for lm in hand_landmarks.landmark]
                ys = [lm.y for lm in hand_landmarks.landmark]

                xmin = max(0.0, min(xs))
                xmax = min(1.0, max(xs))
                ymin = max(0.0, min(ys))
                ymax = min(1.0, max(ys))

                roi_msg = NormalizedRegionOfInterest2D()
                roi_msg.header = msg.header
                roi_msg.xmin = float(xmin)
                roi_msg.ymin = float(ymin)
                roi_msg.xmax = float(xmax)
                roi_msg.ymax = float(ymax)

                self._get_roi_pub(hand_id).publish(roi_msg)

                # ---------- Landmarks ----------
                landmarks_msg = HandLandmarks2D()
                landmarks_msg.header = msg.header
                landmarks_msg.id = hand_id
                landmarks_msg.confidence = float(handedness.classification[0].score)

                # "Left" or "Right"
                landmarks_msg.handedness = handedness_value

                # array/list
                landmarks_msg.landmarks = []
                for lm in hand_landmarks.landmark:
                    poi = NormalizedPointOfInterest2D()
                    poi.x = float(lm.x)
                    poi.y = float(lm.y)
                    landmarks_msg.landmarks.append(poi)

                self._get_lm_pub(hand_id).publish(landmarks_msg)

                # ---------- Debug draw ----------
                if self.publish_debug_image and self.pub_debug is not None:
                    self.mp_draw.draw_landmarks(
                        frame_bgr,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_draw_styles.get_default_hand_landmarks_style(),
                        self.mp_draw_styles.get_default_hand_connections_style(),
                    )

        # Publicar tracked siempre (vacío si no hay manos)
        self.pub_tracked.publish(ids_msg)

        # Publicar imagen debug si está activado (aunque no haya manos, te sirve para ver el stream)
        if self.publish_debug_image and self.pub_debug is not None:
            out = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            out.header = msg.header
            self.pub_debug.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeHandsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()