#!/usr/bin/env python3
"""
Lifecycle ROS2 node for real-time hand detection and 2D landmark extraction using MediaPipe Hands.

This node subscribes to an RGB image topic, runs MediaPipe Hands inference, and publishes:
- Tracked hand identifiers on /humans/hands/tracked.
- Normalized 2D hand landmarks on /humans/hands/<id>/landmarks2d.
- Normalized 2D hand regions of interest on /humans/hands/<id>/roi2d.
- Optional debug visualization images.

Runtime hand selection:
- "both": publish both hand_0 and hand_1.
- "right": publish only hand_0.
- "left": publish only hand_1.

The selected mode can be changed while the node is active using either:
- ros2 param set /hand_mediapipe_node hand_selection left
- std_msgs/String commands on /hand_change

Lifecycle behavior:
- Unconfigured:
    Only parameters are declared. No MediaPipe model, publishers, or subscribers are created.
- Inactive:
    MediaPipe, publishers, and the image subscriber are configured, but images are ignored.
- Active:
    Incoming images are processed and hand information is published.
- Deactivate:
    Processing is stopped, but configured resources are kept.
- Cleanup:
    Publishers, subscribers, MediaPipe resources, and cached hand publishers are released.

Author:
    CARTIF Technology Centre
"""

import logging
from typing import Optional

import cv2
import mediapipe as mp
import rclpy

from cv_bridge import CvBridge
from hri_msgs.msg import (
    IdsList,
    NormalizedPointOfInterest2D,
    NormalizedRegionOfInterest2D,
)
from hand_hri_mediapipe_msgs.msg import HandLandmarks2D
from rcl_interfaces.msg import SetParametersResult
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rich.logging import RichHandler
from sensor_msgs.msg import Image
from std_msgs.msg import String


logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler(rich_tracebacks=True)],
    force=True,
)
LOG = logging.getLogger("hand_mediapipe_lifecycle")


class MediaPipeHandsLifecycleNode(LifecycleNode):
    """Lifecycle ROS2 node for MediaPipe-based 2D hand landmark extraction.

    The node performs MediaPipe inference only while Active. Runtime hand
    selection is implemented as an output filter, so MediaPipe can keep
    detecting up to max_hands while the node publishes only the selected
    hand subset.
    """

    VALID_HAND_SELECTIONS = {"left", "right", "both"}

    def __init__(self):
        """Declare parameters and initialize lifecycle-managed resources."""
        super().__init__("hand_mediapipe_node")

        self.declare_parameter("image_topic", "/camera/cameraD455/color/image_raw")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_topic", "/humans/hands/debug_image")
        self.declare_parameter("max_hands", 2)
        self.declare_parameter("model_complexity", 1)
        self.declare_parameter("min_detection_confidence", 0.6)
        self.declare_parameter("min_tracking_confidence", 0.6)
        self.declare_parameter("swap_handedness", True)
        self.declare_parameter("hand_selection", "both")
        self.declare_parameter("hand_command_topic", "/hand_change")

        self.active = False

        self.bridge: Optional[CvBridge] = None

        self.image_topic = ""
        self.debug_image_topic = ""
        self.hand_command_topic = ""
        self.publish_debug_image = False
        self.swap_handedness = True
        self.hand_selection = "both"

        self.mp_hands = None
        self.mp_draw = None
        self.mp_draw_styles = None
        self.hands = None

        self.sub_image = None
        self.sub_hand_command = None
        self.pub_tracked = None
        self.pub_debug = None

        self._roi_pubs = {}
        self._lm_pubs = {}

        self.add_on_set_parameters_callback(self.on_parameter_update)

        LOG.info("MediaPipe lifecycle node created. Initial state: Unconfigured")

    def on_configure(self, state):
        """Configure MediaPipe, publishers, and the input image subscription.

        This transition moves the node from Unconfigured to Inactive. Resources
        are allocated here, but image processing is not enabled until the node is
        activated.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS if all resources were created correctly, otherwise FAILURE.
        """
        _ = state

        try:
            self.image_topic = str(self.get_parameter("image_topic").value)
            self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
            self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
            self.hand_command_topic = str(self.get_parameter("hand_command_topic").value)

            requested_hand_selection = str(self.get_parameter("hand_selection").value).strip().lower()
            if requested_hand_selection not in self.VALID_HAND_SELECTIONS:
                LOG.warning(
                    f"Invalid initial hand_selection={requested_hand_selection!r}. Falling back to both."
                )
                requested_hand_selection = "both"
            self.hand_selection = requested_hand_selection

            max_hands = int(self.get_parameter("max_hands").value)
            model_complexity = int(self.get_parameter("model_complexity").value)
            min_detection_confidence = float(self.get_parameter("min_detection_confidence").value)
            min_tracking_confidence = float(self.get_parameter("min_tracking_confidence").value)
            self.swap_handedness = bool(self.get_parameter("swap_handedness").value)

            self.bridge = CvBridge()

            self.mp_hands = mp.solutions.hands
            self.mp_draw = mp.solutions.drawing_utils
            self.mp_draw_styles = mp.solutions.drawing_styles

            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=max_hands,
                model_complexity=model_complexity,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence,
            )

            self.pub_tracked = self.create_publisher(
                IdsList,
                "/humans/hands/tracked",
                10,
            )

            if self.publish_debug_image:
                self.pub_debug = self.create_publisher(
                    Image,
                    self.debug_image_topic,
                    10,
                )

            self.sub_image = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback,
                10,
            )

            self.sub_hand_command = self.create_subscription(
                String,
                self.hand_command_topic,
                self.hand_command_callback,
                10,
            )

            LOG.info("MediaPipe lifecycle node configured.")
            LOG.info(f"  image_topic: {self.image_topic}")
            LOG.info("  tracked_topic: /humans/hands/tracked")
            LOG.info(f"  debug_image: {self.publish_debug_image} -> {self.debug_image_topic}")
            LOG.info(f"  hand_selection: {self.hand_selection}")
            LOG.info(f"  hand_command_topic: {self.hand_command_topic}")

            return TransitionCallbackReturn.SUCCESS

        except Exception as exc:
            LOG.exception(f"Failed to configure MediaPipe lifecycle node: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        """Enable image processing and topic publication.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS when the node is ready to process images.
        """
        _ = state
        self.active = True
        LOG.info(f"MediaPipe lifecycle node activated. hand_selection={self.hand_selection}")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """Disable image processing without destroying configured resources.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS when processing has been stopped.
        """
        _ = state
        self.active = False
        LOG.info("MediaPipe lifecycle node deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """Release all configured resources and return to Unconfigured.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS if resources were released correctly, otherwise FAILURE.
        """
        _ = state
        self.active = False

        try:
            if self.sub_image is not None:
                self.destroy_subscription(self.sub_image)
                self.sub_image = None

            if self.sub_hand_command is not None:
                self.destroy_subscription(self.sub_hand_command)
                self.sub_hand_command = None

            if self.pub_tracked is not None:
                self.destroy_publisher(self.pub_tracked)
                self.pub_tracked = None

            if self.pub_debug is not None:
                self.destroy_publisher(self.pub_debug)
                self.pub_debug = None

            for publisher in self._roi_pubs.values():
                self.destroy_publisher(publisher)
            self._roi_pubs.clear()

            for publisher in self._lm_pubs.values():
                self.destroy_publisher(publisher)
            self._lm_pubs.clear()

            if self.hands is not None:
                self.hands.close()
                self.hands = None

            self.bridge = None
            self.mp_hands = None
            self.mp_draw = None
            self.mp_draw_styles = None

            LOG.info("MediaPipe lifecycle node cleaned up.")
            return TransitionCallbackReturn.SUCCESS

        except Exception as exc:
            LOG.exception(f"Failed to cleanup MediaPipe lifecycle node: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state):
        """Handle shutdown from any lifecycle state.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS after best-effort resource release.
        """
        _ = state
        self.active = False

        try:
            if self.hands is not None:
                self.hands.close()
                self.hands = None
        except Exception:
            pass

        LOG.info("MediaPipe lifecycle node shutting down.")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state):
        """Stop processing if the node enters the lifecycle error state.

        Parameters
        ----------
        state
            Current lifecycle state provided by rclpy.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS after disabling processing.
        """
        _ = state
        self.active = False
        LOG.error("MediaPipe lifecycle node entered error state.")
        return TransitionCallbackReturn.SUCCESS


    def on_parameter_update(self, parameters):
        """Validate and apply dynamic parameter updates.

        The `hand_selection` parameter can be changed while the node is Active.
        This does not recreate MediaPipe. It only changes which detected hands
        are published.

        Parameters
        ----------
        parameters : list
            Parameters requested for update.

        Returns
        -------
        SetParametersResult
            Successful result when all requested updates are valid.
        """
        for parameter in parameters:
            if parameter.name != "hand_selection":
                continue

            requested_selection = str(parameter.value).strip().lower()
            if requested_selection not in self.VALID_HAND_SELECTIONS:
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"Invalid hand_selection={requested_selection!r}. "
                        "Valid values are: left, right, both."
                    ),
                )

            previous_selection = self.hand_selection
            self.hand_selection = requested_selection
            LOG.info(f"hand_selection changed: {previous_selection} -> {self.hand_selection}")

        return SetParametersResult(successful=True)

    def hand_command_callback(self, msg: String):
        """Handle runtime hand-selection commands from the orchestrator topic.

        Parameters
        ----------
        msg : String
            Command containing "left", "right", or "both".

        Returns
        -------
        None
        """
        requested_selection = msg.data.strip().lower()

        if requested_selection not in self.VALID_HAND_SELECTIONS:
            LOG.warning(
                f"Ignoring invalid hand command {requested_selection!r}. "
                "Valid values are: left, right, both."
            )
            return

        previous_selection = self.hand_selection
        self.hand_selection = requested_selection

        # Keep the parameter value synchronized with the command topic.
        try:
            self.set_parameters([
                rclpy.parameter.Parameter(
                    "hand_selection",
                    rclpy.Parameter.Type.STRING,
                    requested_selection,
                )
            ])
        except Exception as exc:
            LOG.warning(f"Could not synchronize hand_selection parameter: {exc}")

        LOG.info(f"hand_selection command received: {previous_selection} -> {self.hand_selection}")

    def should_publish_hand(self, hand_id: str) -> bool:
        """Check whether a detected hand should be published.

        Parameters
        ----------
        hand_id : str
            Stable hand identifier. By convention, hand_0 is right and hand_1 is left.

        Returns
        -------
        bool
            True if the current hand_selection mode allows publishing this hand.
        """
        if self.hand_selection == "both":
            return hand_id in ("hand_0", "hand_1")

        if self.hand_selection == "right":
            return hand_id == "hand_0"

        if self.hand_selection == "left":
            return hand_id == "hand_1"

        return False

    def _get_roi_pub(self, hand_id: str):
        """Return a cached ROI publisher for a specific hand.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand.

        Returns
        -------
        Publisher
            Publisher for /humans/hands/<hand_id>/roi2d.
        """
        topic = f"/humans/hands/{hand_id}/roi2d"
        if topic not in self._roi_pubs:
            self._roi_pubs[topic] = self.create_publisher(
                NormalizedRegionOfInterest2D,
                topic,
                10,
            )
        return self._roi_pubs[topic]

    def _get_lm_pub(self, hand_id: str):
        """Return a cached 2D landmarks publisher for a specific hand.

        Parameters
        ----------
        hand_id : str
            Identifier of the tracked hand.

        Returns
        -------
        Publisher
            Publisher for /humans/hands/<hand_id>/landmarks2d.
        """
        topic = f"/humans/hands/{hand_id}/landmarks2d"
        if topic not in self._lm_pubs:
            self._lm_pubs[topic] = self.create_publisher(
                HandLandmarks2D,
                topic,
                10,
            )
        return self._lm_pubs[topic]

    def image_callback(self, msg: Image):
        """Process an RGB image and publish hand tracking outputs.

        The callback only runs inference while the node is Active. If the node is
        Inactive, the image is ignored.

        Parameters
        ----------
        msg : Image
            Input RGB image message.

        Returns
        -------
        None
        """
        if not self.active:
            return

        if self.bridge is None or self.hands is None:
            return

        frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        results = self.hands.process(frame_rgb)

        ids_msg = IdsList()
        ids_msg.header = msg.header
        ids_msg.ids = []

        if results.multi_hand_landmarks:
            for hand_landmarks, handedness in zip(
                results.multi_hand_landmarks,
                results.multi_handedness,
            ):
                hand_id, handedness_value = self._resolve_hand_identity(handedness)

                if not self.should_publish_hand(hand_id):
                    continue

                ids_msg.ids.append(hand_id)

                roi_msg = self._build_roi_msg(msg, hand_landmarks)
                self._get_roi_pub(hand_id).publish(roi_msg)

                landmarks_msg = self._build_landmarks_msg(
                    msg,
                    hand_id,
                    handedness_value,
                    hand_landmarks,
                    handedness,
                )
                self._get_lm_pub(hand_id).publish(landmarks_msg)

                if self.publish_debug_image and self.pub_debug is not None:
                    self.mp_draw.draw_landmarks(
                        frame_bgr,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                        self.mp_draw_styles.get_default_hand_landmarks_style(),
                        self.mp_draw_styles.get_default_hand_connections_style(),
                    )

        if self.pub_tracked is not None:
            self.pub_tracked.publish(ids_msg)

        if self.publish_debug_image and self.pub_debug is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            debug_msg.header = msg.header
            self.pub_debug.publish(debug_msg)

    def _resolve_hand_identity(self, handedness):
        """Map MediaPipe handedness output to a stable hand identifier.

        Parameters
        ----------
        handedness
            MediaPipe handedness classification result.

        Returns
        -------
        tuple[str, int]
            Stable hand identifier and HandLandmarks2D handedness enum value.
        """
        raw_label = handedness.classification[0].label

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
            return "hand_0", HandLandmarks2D.HANDEDNESS_RIGHT

        if label == "Left":
            return "hand_1", HandLandmarks2D.HANDEDNESS_LEFT

        return "hand_unknown", HandLandmarks2D.HANDEDNESS_UNKNOWN

    def _build_roi_msg(self, image_msg: Image, hand_landmarks) -> NormalizedRegionOfInterest2D:
        """Build a normalized ROI message from MediaPipe hand landmarks.

        Parameters
        ----------
        image_msg : Image
            Source image message used for the output header.
        hand_landmarks
            MediaPipe hand landmarks result.

        Returns
        -------
        NormalizedRegionOfInterest2D
            Normalized bounding box around the hand landmarks.
        """
        xs = [landmark.x for landmark in hand_landmarks.landmark]
        ys = [landmark.y for landmark in hand_landmarks.landmark]

        roi_msg = NormalizedRegionOfInterest2D()
        roi_msg.header = image_msg.header
        roi_msg.xmin = float(max(0.0, min(xs)))
        roi_msg.ymin = float(max(0.0, min(ys)))
        roi_msg.xmax = float(min(1.0, max(xs)))
        roi_msg.ymax = float(min(1.0, max(ys)))
        return roi_msg

    def _build_landmarks_msg(
        self,
        image_msg: Image,
        hand_id: str,
        handedness_value: int,
        hand_landmarks,
        handedness,
    ) -> HandLandmarks2D:
        """Build a HandLandmarks2D message from MediaPipe landmarks.

        Parameters
        ----------
        image_msg : Image
            Source image message used for the output header.
        hand_id : str
            Stable hand identifier.
        handedness_value : int
            HandLandmarks2D handedness enum value.
        hand_landmarks
            MediaPipe hand landmarks result.
        handedness
            MediaPipe handedness classification result.

        Returns
        -------
        HandLandmarks2D
            Output 2D landmarks message.
        """
        confidence = float(handedness.classification[0].score)

        landmarks_msg = HandLandmarks2D()
        landmarks_msg.header = image_msg.header
        landmarks_msg.id = hand_id
        landmarks_msg.confidence = confidence
        landmarks_msg.handedness = handedness_value
        landmarks_msg.landmarks = []

        for landmark in hand_landmarks.landmark:
            point = NormalizedPointOfInterest2D()
            point.x = float(landmark.x)
            point.y = float(landmark.y)
            point.c = confidence
            landmarks_msg.landmarks.append(point)

        return landmarks_msg


def main(args=None):
    """Run the MediaPipe hand lifecycle node."""
    rclpy.init(args=args)
    node = MediaPipeHandsLifecycleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
