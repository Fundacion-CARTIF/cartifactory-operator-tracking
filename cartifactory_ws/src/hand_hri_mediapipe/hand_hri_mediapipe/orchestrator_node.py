#!/usr/bin/env python3
"""
ROS2 orchestrator node for the hand HRI MediaPipe lifecycle pipeline.

This node centralizes:
- Lifecycle control of the hand HRI processing nodes.
- Runtime hand-selection commands.
- Lightweight monitoring of the pipeline state.

Managed lifecycle nodes:
- /hand_mediapipe_node
- /hand_landmarks_3d_node
- /hand_palm_plane_node
- /hand_cam_to_base_node

Action servers:
- /lifecycle_control
- /hand_change_action

Published monitoring topics:
- /hand_mediapipe_node/current_state
- /hand_landmarks_3d_node/current_state
- /hand_palm_plane_node/current_state
- /hand_cam_to_base_node/current_state
- /stats/hand_pipeline

Author:
    CARTIF Technology Centre
"""

from __future__ import annotations

import logging
from functools import partial
from typing import Dict, Optional, Tuple, cast

import rclpy
from geometry_msgs.msg import PoseStamped
from hand_hri_mediapipe_msgs.action import HandChange, LifecycleControl
from hand_hri_mediapipe_msgs.msg import PipelineStats
from hri_msgs.msg import IdsList
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
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
LOG = logging.getLogger("orchestrator")


class Orchestrator(Node):
    """Orchestrates lifecycle control, hand selection, and monitoring.

    Parameters
    ----------
    poll_period_s : float
        Period used to query lifecycle states and publish pipeline statistics.
    lf_action_topic : str
        Action topic used for lifecycle control.
    hand_action_topic : str
        Action topic used for hand-selection control.
    hand_topic : str
        Topic used to send hand-selection commands to the MediaPipe node.
    pipeline_stats_topic : str
        Topic used to publish aggregated pipeline monitoring information.
    image_topic : str
        RGB input image topic used to estimate input FPS and camera availability.
    debug_image_topic : str
        Debug image topic used to estimate processed FPS.
    tracked_topic : str
        Topic containing currently tracked hand identifiers.
    tcp_pose_topic : str
        Topic containing the current robot TCP pose.
    fps_ema_alpha : float
        Exponential moving average alpha for FPS estimation.
    camera_timeout_sec : float
        Timeout used to mark the camera as unavailable.
    tcp_timeout_sec : float
        Timeout used to mark TCP pose as unavailable.
    """

    FORWARD_ORDER = ["mediapipe", "landmarks", "palm", "cam2base"]
    REVERSE_ORDER = ["cam2base", "palm", "landmarks", "mediapipe"]

    VALID_LIFECYCLE_TARGETS = {"configure", "activate", "deactivate", "cleanup", "shutdown"}
    VALID_HAND_SELECTIONS = {"right", "left", "both"}

    def __init__(self):
        """Initialize parameters, interfaces, actions, and monitoring state."""
        super().__init__("orchestrator")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter("poll_period_s", 5.0)

        self.declare_parameter("lf_action_topic", "/lifecycle_control")
        self.declare_parameter("hand_action_topic", "/hand_change_action")
        self.declare_parameter("hand_topic", "/hand_change")

        self.declare_parameter("pipeline_stats_topic", "/stats/hand_pipeline")
        self.declare_parameter("image_topic", "/camera/cameraD455/color/image_raw")
        self.declare_parameter("debug_image_topic", "/humans/hands/debug_image")
        self.declare_parameter("tracked_topic", "/humans/hands/tracked")
        self.declare_parameter("tcp_pose_topic", "/ur/tcp_pose")

        self.declare_parameter("fps_ema_alpha", 0.2)
        self.declare_parameter("camera_timeout_sec", 5.0)
        self.declare_parameter("tcp_timeout_sec", 5.0)

        poll_period_s = float(cast(float, self.get_parameter("poll_period_s").value))

        lf_action_topic = str(cast(str, self.get_parameter("lf_action_topic").value))
        hand_action_topic = str(cast(str, self.get_parameter("hand_action_topic").value))
        hand_pub_topic = str(cast(str, self.get_parameter("hand_topic").value))

        self.pipeline_stats_topic = str(cast(str, self.get_parameter("pipeline_stats_topic").value))
        self.image_topic = str(cast(str, self.get_parameter("image_topic").value))
        self.debug_image_topic = str(cast(str, self.get_parameter("debug_image_topic").value))
        self.tracked_topic = str(cast(str, self.get_parameter("tracked_topic").value))
        self.tcp_pose_topic = str(cast(str, self.get_parameter("tcp_pose_topic").value))

        self.fps_ema_alpha = float(cast(float, self.get_parameter("fps_ema_alpha").value))
        self.camera_timeout_sec = float(cast(float, self.get_parameter("camera_timeout_sec").value))
        self.tcp_timeout_sec = float(cast(float, self.get_parameter("tcp_timeout_sec").value))

        # ------------------------------------------------------------------
        # Callback groups
        # ------------------------------------------------------------------
        self._async_group = ReentrantCallbackGroup()
        self._action_group = ReentrantCallbackGroup()

        self.busy_with_lifecycle_change = False
        self.busy_with_hand_change = False

        # ------------------------------------------------------------------
        # Lifecycle nodes managed by this orchestrator
        # ------------------------------------------------------------------
        self.topic_names: Dict[str, str] = {
            "mediapipe": "/hand_mediapipe_node",
            "landmarks": "/hand_landmarks_3d_node",
            "palm": "/hand_palm_plane_node",
            "cam2base": "/hand_cam_to_base_node",
        }

        self.last_statuses: Dict[str, str] = {
            key: "unknown" for key in self.topic_names.keys()
        }

        self._cmd_to_transition_id: Dict[str, int] = {
            "configure": Transition.TRANSITION_CONFIGURE,
            "cleanup": Transition.TRANSITION_CLEANUP,
            "clean up": Transition.TRANSITION_CLEANUP,
            "activate": Transition.TRANSITION_ACTIVATE,
            "deactivate": Transition.TRANSITION_DEACTIVATE,
            "shutdown_active": Transition.TRANSITION_ACTIVE_SHUTDOWN,
            "shutdown_inactive": Transition.TRANSITION_INACTIVE_SHUTDOWN,
            "shutdown_unconfigured": Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,
        }

        # ------------------------------------------------------------------
        # Monitoring internal state
        # ------------------------------------------------------------------
        self._last_input_t: Optional[float] = None
        self._last_processed_t: Optional[float] = None
        self._last_camera_msg_time: Optional[float] = None
        self._last_tcp_pose_time: Optional[float] = None

        self._fps_input_ema = 0.0
        self._fps_processed_ema = 0.0

        self._tracked_hand_ids = []
        self._hand_selection = "both"

        self._last_stats_fingerprint = None

        self._action_goals_received = 0
        self._action_goals_accepted = 0
        self._action_goals_rejected = 0
        self._action_goals_canceled = 0
        self._action_goals_succeeded = 0
        self._action_goals_failed = 0

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self.state_publishers = {
            key: self.create_publisher(String, f"{node_name}/current_state", 10)
            for key, node_name in self.topic_names.items()
        }

        self.hand_pub = self.create_publisher(String, hand_pub_topic, 10)

        self.pub_pipeline_stats = self.create_publisher(
            PipelineStats,
            self.pipeline_stats_topic,
            10,
        )

        # ------------------------------------------------------------------
        # Lifecycle service clients
        # ------------------------------------------------------------------
        self.get_states = {
            key: self.create_client(
                GetState,
                f"{node_name}/get_state",
                callback_group=self._async_group,
            )
            for key, node_name in self.topic_names.items()
        }

        self.set_states = {
            key: self.create_client(
                ChangeState,
                f"{node_name}/change_state",
                callback_group=self._async_group,
            )
            for key, node_name in self.topic_names.items()
        }

        # ------------------------------------------------------------------
        # Monitoring subscribers
        # ------------------------------------------------------------------
        self.sub_input_image = self.create_subscription(
            Image,
            self.image_topic,
            self.on_input_image,
            qos_profile_sensor_data,
        )

        self.sub_debug_image = self.create_subscription(
            Image,
            self.debug_image_topic,
            self.on_debug_image,
            qos_profile_sensor_data,
        )

        self.sub_tracked_hands = self.create_subscription(
            IdsList,
            self.tracked_topic,
            self.on_tracked_hands,
            10,
        )

        self.sub_tcp_pose = self.create_subscription(
            PoseStamped,
            self.tcp_pose_topic,
            self.on_tcp_pose,
            10,
        )

        # ------------------------------------------------------------------
        # Timer
        # ------------------------------------------------------------------
        self._timer = self.create_timer(
            poll_period_s,
            self.poll_and_publish_state,
            callback_group=self._async_group,
        )

        # ------------------------------------------------------------------
        # Action servers
        # ------------------------------------------------------------------
        self.lifecycle_control_action = ActionServer(
            node=self,
            action_type=LifecycleControl,
            action_name=lf_action_topic,
            execute_callback=self.lifecycle_execute_callback,
            goal_callback=self.lifecycle_goal_callback,
            cancel_callback=self.lifecycle_cancel_callback,
            callback_group=self._action_group,
        )

        self.hand_action = ActionServer(
            node=self,
            action_type=HandChange,
            action_name=hand_action_topic,
            execute_callback=self.hand_execute_callback,
            goal_callback=self.hand_goal_callback,
            cancel_callback=self.hand_cancel_callback,
            callback_group=self._action_group,
        )

        LOG.info("Orchestrator initialized.")
        LOG.info(f"  lifecycle_action_topic: {lf_action_topic}")
        LOG.info(f"  hand_action_topic:      {hand_action_topic}")
        LOG.info(f"  hand_command_topic:     {hand_pub_topic}")
        LOG.info(f"  pipeline_stats_topic:   {self.pipeline_stats_topic}")
        LOG.info(f"  managed_nodes:          {self.topic_names}")

    # ======================================================================
    # Action callbacks: lifecycle control
    # ======================================================================

    def lifecycle_goal_callback(self, goal_request: LifecycleControl.Goal) -> GoalResponse:
        """Validate lifecycle action goals before execution."""
        self._action_goals_received += 1

        target_state = goal_request.target_state.strip().lower()

        if target_state not in self.VALID_LIFECYCLE_TARGETS:
            self._action_goals_rejected += 1
            self.get_logger().warn(
                f"Lifecycle request rejected. Unknown target_state='{target_state}'. "
                f"Valid options: {sorted(self.VALID_LIFECYCLE_TARGETS)}"
            )
            return GoalResponse.REJECT

        if self.busy_with_lifecycle_change:
            self._action_goals_rejected += 1
            self.get_logger().warn("Lifecycle request rejected. Orchestrator is busy.")
            return GoalResponse.REJECT

        self.busy_with_lifecycle_change = True
        self._action_goals_accepted += 1
        LOG.info(f"Lifecycle request accepted: {target_state}")
        return GoalResponse.ACCEPT

    async def lifecycle_execute_callback(self, goal_handle) -> LifecycleControl.Result:
        """Execute a lifecycle transition request for the full pipeline.

        The current implementation keeps the user's simple transition model:
        the requested transition is sent to each lifecycle node in the selected
        order. For robust repeated calls, the caller should avoid requesting
        invalid transitions such as "configure" when nodes are already inactive.
        """
        feedback_msg = LifecycleControl.Feedback()
        result_msg = LifecycleControl.Result()

        target_state = goal_handle.request.target_state.strip().lower()
        execution_order = self.get_execution_order(target_state)

        success_by_node = {}

        try:
            for node_key in execution_order:
                feedback_msg.feedback = f"Changing node '{node_key}' to '{target_state}'"
                goal_handle.publish_feedback(feedback_msg)

                future = self.change_state_async(target_state, node_key)

                if future is None:
                    success_by_node[node_key] = False
                    continue

                response = await future
                success_by_node[node_key] = bool(response.success)

                feedback_msg.feedback = (
                    f"Node '{node_key}' service response: success={response.success}"
                )
                goal_handle.publish_feedback(feedback_msg)

            success = all(success_by_node.values())

            if success:
                goal_handle.succeed()
                self._action_goals_succeeded += 1
                result_msg.success = True
                result_msg.result_msg = f"Nodes changed successfully to {target_state}"
                LOG.info(result_msg.result_msg)
            else:
                goal_handle.abort()
                self._action_goals_failed += 1
                result_msg.success = False
                result_msg.result_msg = f"Nodes could not be changed to {target_state}"
                self.get_logger().warn(result_msg.result_msg)

        except Exception as exc:  # noqa: BLE001
            goal_handle.abort()
            self._action_goals_failed += 1
            result_msg.success = False
            result_msg.result_msg = f"Exception while changing nodes state: {type(exc).__name__}: {exc}"
            self.get_logger().error(result_msg.result_msg)

        finally:
            self.busy_with_lifecycle_change = False
            LOG.info(
                f"Returning lifecycle result: success={result_msg.success}, "
                f"result_msg='{result_msg.result_msg}'"
            )
            self.publish_pipeline_stats()

        return result_msg

    def lifecycle_cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept lifecycle action cancellation requests."""
        _ = goal_handle
        self._action_goals_canceled += 1
        LOG.info("Received lifecycle cancel request.")
        return CancelResponse.ACCEPT

    def get_execution_order(self, target_state: str):
        """Return the node execution order for a target lifecycle transition."""
        if target_state in {"deactivate", "cleanup", "shutdown"}:
            return list(self.REVERSE_ORDER)
        return list(self.FORWARD_ORDER)

    # ======================================================================
    # Action callbacks: hand selection
    # ======================================================================

    def hand_goal_callback(self, goal_request: HandChange.Goal) -> GoalResponse:
        """Validate hand-selection action goals before execution."""
        self._action_goals_received += 1

        hand = goal_request.hand.strip().lower()

        if hand not in self.VALID_HAND_SELECTIONS:
            self._action_goals_rejected += 1
            self.get_logger().warn(
                f"Hand change request rejected. Unknown hand='{hand}'. "
                f"Valid options: {sorted(self.VALID_HAND_SELECTIONS)}"
            )
            return GoalResponse.REJECT

        if self.busy_with_hand_change:
            self._action_goals_rejected += 1
            self.get_logger().warn("Hand change request rejected. Orchestrator is busy.")
            return GoalResponse.REJECT

        self.busy_with_hand_change = True
        self._action_goals_accepted += 1
        LOG.info(f"Hand change request accepted: {hand}")
        return GoalResponse.ACCEPT

    def hand_execute_callback(self, goal_handle) -> HandChange.Result:
        """Publish a hand-selection command for the MediaPipe lifecycle node."""
        feedback_msg = HandChange.Feedback()
        result_msg = HandChange.Result()

        hand = goal_handle.request.hand.strip().lower()

        try:
            feedback_msg.feedback = f"Publishing hand-selection command: {hand}"
            goal_handle.publish_feedback(feedback_msg)

            msg = String()
            msg.data = hand
            self.hand_pub.publish(msg)

            self._hand_selection = hand

            goal_handle.succeed()
            self._action_goals_succeeded += 1
            result_msg.success = True
            result_msg.result_msg = f"Hand changed to {hand}"
            LOG.info(result_msg.result_msg)

        except Exception as exc:  # noqa: BLE001
            goal_handle.abort()
            self._action_goals_failed += 1
            result_msg.success = False
            result_msg.result_msg = f"Exception while changing hand: {type(exc).__name__}: {exc}"
            self.get_logger().error(result_msg.result_msg)

        finally:
            self.busy_with_hand_change = False
            LOG.info(
                f"Returning hand result: success={result_msg.success}, "
                f"result_msg='{result_msg.result_msg}'"
            )
            self.publish_pipeline_stats()

        return result_msg

    def hand_cancel_callback(self, goal_handle) -> CancelResponse:
        """Accept hand-selection action cancellation requests."""
        _ = goal_handle
        self._action_goals_canceled += 1
        LOG.info("Received hand change cancel request.")
        return CancelResponse.ACCEPT

    # ======================================================================
    # Lifecycle state polling
    # ======================================================================

    def poll_and_publish_state(self) -> None:
        """Query lifecycle states and publish aggregated monitoring stats."""
        for key, topic_name in self.topic_names.items():
            if not self.get_states[key].service_is_ready():
                self.get_logger().warn(
                    f"get_state service for {topic_name} not ready.",
                    throttle_duration_sec=5.0,
                )
                continue

            request = GetState.Request()
            future = self.get_states[key].call_async(request)
            future.add_done_callback(partial(self.on_get_state, node_key=key))

        self.publish_pipeline_stats()

    def on_get_state(self, future, node_key: str) -> None:
        """Handle a completed lifecycle GetState response."""
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            LOG.error(f"GetState failed for {self.topic_names[node_key]}: {exc}")
            return

        state_id = int(response.current_state.id)
        state_label = str(response.current_state.label)
        state_text = f"{state_label} ({state_id})"

        msg = String()
        msg.data = state_text

        if state_text != self.last_statuses[node_key]:
            LOG.info(f"{self.topic_names[node_key]} state: {self.last_statuses[node_key]} -> {state_text}")
            self.last_statuses[node_key] = state_text

        self.state_publishers[node_key].publish(msg)

    def change_state_async(self, state: str, node_key: str):
        """Send a lifecycle ChangeState request asynchronously."""
        command = " ".join(state.strip().lower().split())

        if command == "shutdown":
            command = self.resolve_shutdown_command(node_key)

        transition_id = self._cmd_to_transition_id.get(command)
        LOG.info(f"{self.topic_names[node_key]} command: {command} (id={transition_id})")

        if transition_id is None:
            known = ", ".join(sorted(set(self._cmd_to_transition_id.keys())))
            self.get_logger().warn(f"Unknown command '{state}'. Known: {known}")
            return None

        request = ChangeState.Request()
        request.transition.id = int(transition_id)
        return self.set_states[node_key].call_async(request)

    def resolve_shutdown_command(self, node_key: str) -> str:
        """Choose the shutdown transition according to the last known state."""
        last_state = self.last_statuses.get(node_key, "").lower()

        if last_state.startswith("active"):
            return "shutdown_active"

        if last_state.startswith("inactive"):
            return "shutdown_inactive"

        return "shutdown_unconfigured"

    # ======================================================================
    # Monitoring callbacks
    # ======================================================================

    def on_input_image(self, msg: Image):
        """Update input FPS and camera availability from RGB images."""
        _ = msg
        now_sec = self.time_now_sec()
        self._fps_input_ema, self._last_input_t = self.update_fps_ema(
            now_sec,
            self._last_input_t,
            self._fps_input_ema,
        )
        self._last_camera_msg_time = now_sec

    def on_debug_image(self, msg: Image):
        """Update processed FPS from MediaPipe debug images."""
        _ = msg
        now_sec = self.time_now_sec()
        self._fps_processed_ema, self._last_processed_t = self.update_fps_ema(
            now_sec,
            self._last_processed_t,
            self._fps_processed_ema,
        )

    def on_tracked_hands(self, msg: IdsList):
        """Store the last tracked hand identifiers."""
        self._tracked_hand_ids = list(msg.ids)

    def on_tcp_pose(self, msg: PoseStamped):
        """Update TCP pose availability timestamp."""
        _ = msg
        self._last_tcp_pose_time = self.time_now_sec()

    def update_fps_ema(
        self,
        now_sec: float,
        last_timestamp: Optional[float],
        current_ema: float,
    ) -> Tuple[float, float]:
        """Update an exponential moving average FPS estimation."""
        if last_timestamp is None:
            return current_ema, now_sec

        delta_t = max(now_sec - last_timestamp, 1e-6)
        instant_fps = 1.0 / delta_t
        alpha = min(max(float(self.fps_ema_alpha), 0.0), 1.0)
        new_ema = (1.0 - alpha) * current_ema + alpha * instant_fps
        return new_ema, now_sec

    # ======================================================================
    # Pipeline stats publication
    # ======================================================================

    def publish_pipeline_stats(self):
        """Build and publish the aggregated pipeline monitoring message."""
        stats_msg = self.build_pipeline_stats_msg()
        fingerprint = self.pipeline_stats_fingerprint(stats_msg)

        if self._last_stats_fingerprint is None or fingerprint != self._last_stats_fingerprint:
            self.pub_pipeline_stats.publish(stats_msg)
            self._last_stats_fingerprint = fingerprint

    def build_pipeline_stats_msg(self) -> PipelineStats:
        """Create a PipelineStats message from the latest monitored values."""
        now_sec = self.time_now_sec()

        msg = PipelineStats()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_name = "hand_hri_orchestrator"

        msg.fps_input = float(self._fps_input_ema)
        msg.fps_processed = float(self._fps_processed_ema)
        msg.avg_latency_ms = 0.0

        msg.camera_available = (
            self._last_camera_msg_time is not None
            and (now_sec - self._last_camera_msg_time) <= self.camera_timeout_sec
        )

        msg.tcp_pose_available = (
            self._last_tcp_pose_time is not None
            and (now_sec - self._last_tcp_pose_time) <= self.tcp_timeout_sec
        )

        msg.action_goals_received = int(self._action_goals_received)
        msg.action_goals_accepted = int(self._action_goals_accepted)
        msg.action_goals_rejected = int(self._action_goals_rejected)
        msg.action_goals_canceled = int(self._action_goals_canceled)
        msg.action_goals_succeeded = int(self._action_goals_succeeded)
        msg.action_goals_failed = int(self._action_goals_failed)

        msg.mediapipe_state = self.last_statuses.get("mediapipe", "unknown")
        msg.landmarks_3d_state = self.last_statuses.get("landmarks", "unknown")
        msg.palm_plane_state = self.last_statuses.get("palm", "unknown")
        msg.cam_to_base_state = self.last_statuses.get("cam2base", "unknown")

        msg.tracked_hand_ids = list(self._tracked_hand_ids)
        msg.tracked_hands_count = len(self._tracked_hand_ids)

        msg.hand_selection = self._hand_selection

        return msg

    def pipeline_stats_fingerprint(self, msg: PipelineStats):
        """Return a compact representation used to avoid repeated publications."""
        return (
            int(round(float(msg.fps_input))),
            int(round(float(msg.fps_processed))),
            int(round(float(msg.avg_latency_ms))),
            bool(msg.camera_available),
            bool(msg.tcp_pose_available),
            int(msg.action_goals_received),
            int(msg.action_goals_accepted),
            int(msg.action_goals_rejected),
            int(msg.action_goals_canceled),
            int(msg.action_goals_succeeded),
            int(msg.action_goals_failed),
            str(msg.mediapipe_state),
            str(msg.landmarks_3d_state),
            str(msg.palm_plane_state),
            str(msg.cam_to_base_state),
            tuple(msg.tracked_hand_ids),
            int(msg.tracked_hands_count),
            str(msg.hand_selection),
        )

    def time_now_sec(self) -> float:
        """Return the current ROS time in seconds."""
        now_msg = self.get_clock().now().to_msg()
        return float(now_msg.sec) + float(now_msg.nanosec) / 1e9

    # ======================================================================
    # Startup service waiting
    # ======================================================================

    def wait_for_services(self, timeout_step_sec: float = 1.0) -> None:
        """Wait until all lifecycle GetState and ChangeState services exist."""
        for key, topic_name in self.topic_names.items():
            while not self.get_states[key].wait_for_service(timeout_sec=timeout_step_sec):
                self.get_logger().warn(
                    f"Waiting for {topic_name}/get_state ...",
                    throttle_duration_sec=5.0,
                )

            while not self.set_states[key].wait_for_service(timeout_sec=timeout_step_sec):
                self.get_logger().warn(
                    f"Waiting for {topic_name}/change_state ...",
                    throttle_duration_sec=5.0,
                )

        LOG.info("All lifecycle services are available.")


def main():
    """Run the orchestrator node."""
    rclpy.init()

    node = Orchestrator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    node.wait_for_services()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
