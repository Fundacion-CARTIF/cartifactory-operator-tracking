#!/usr/bin/env python3
"""
ROS2 node for orchestrating all the nodes inside this package
This node comunicates through a ROS2 action for requesting the initialization or stopping of the hand HRI nodes

Author:
    CARTIF Technology Centre
"""
from typing import Dict, Optional, cast
from functools import partial
import logging
from rich.logging import RichHandler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


from std_msgs.msg import String
from sensor_msgs.msg import Image

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

from hand_hri_mediapipe_msgs.action import LifecycleControl, HandChange

logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler(rich_tracebacks=True)],
    force=True,
)
LOG = logging.getLogger("orchestator")


class Orchestrator(Node):
    """

    """

    def __init__(self):
        super().__init__("orchestrator")

        # ---- Parameters ----
        # self.declare_parameter("hand_cam_2_base_node", "/camera/inperso")
        # self._target_node = str(self.get_parameter("target_node").value).rstrip("/")
        self.declare_parameter("poll_period_s", 5.0)
        poll_period_s: float = cast(float, self.get_parameter("poll_period_s").value)

        self.declare_parameter("lf_action_topic", "/lifecycle_control")
        lf_action_topic: str = cast(str, self.get_parameter("lf_action_topic").value)

        self.declare_parameter("hand_action_topic", "/hand_change_action")
        hand_action_topic: str = cast(str, self.get_parameter("hand_action_topic").value)

        self.declare_parameter("hand_topic", "/hand_change")
        hand_pub_topic: str = cast(str, self.get_parameter("hand_topic").value)

        # Callback groups
        self._async_group = ReentrantCallbackGroup()
        self._async_actions = ReentrantCallbackGroup()
        self._state_group = MutuallyExclusiveCallbackGroup()

        self.busy_with_change = False   # Flag to define if we are in the middle of a process
        # Store last published status to avoid duplicates in logging
        self.last_statuses = {
                "cam2base":"None",
                "landmarks":"None",
                "mediapipe":"None",
                "palm":"None",
                    }
        self.change_state_success = {
                "cam2base":False,
                "landmarks":False,
                "mediapipe":False,
                "palm":False,
                    }

        # ---- Command mapping ----
        # You can add aliases freely (e.g., "clean up" -> cleanup)
        self._cmd_to_transition_id: Dict[str, int] = {
            "configure": Transition.TRANSITION_CONFIGURE,          # 1
            "cleanup": Transition.TRANSITION_CLEANUP,              # 2
            "clean up": Transition.TRANSITION_CLEANUP,             # alias
            "activate": Transition.TRANSITION_ACTIVATE,            # 3
            "deactivate": Transition.TRANSITION_DEACTIVATE,        # 4
            "shutdown": Transition.TRANSITION_ACTIVE_SHUTDOWN,     # 5 (see note below)
            # You can expose more explicit shutdowns if you want:
            "shutdown_active": Transition.TRANSITION_ACTIVE_SHUTDOWN,
            "shutdown_inactive": Transition.TRANSITION_INACTIVE_SHUTDOWN,
            "shutdown_unconfigured": Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,
        }

        hand_cam_to_base_topic = "/hand_cam_to_base_node"
        hand_landmarks_3d_topic = "/hand_landmarks_3d_node"
        hand_mediapipe_topic = "/hand_mediapipe_node"
        hand_palm_plane_topic = "/hand_palm_plane_node"

        self.topic_names = {
                    "cam2base":     "/hand_cam_to_base_node",
                    "landmarks":    "/hand_landmarks_3d_node",
                    "mediapipe":    "/hand_mediapipe_node",
                    "palm":         "/hand_palm_plane_node",
                        }

        #* === ROS2 INTERFACES =================
        # ---- State publishers ----
        self.state_publishers = {
                    "cam2base":     self.create_publisher(String, f"{hand_cam_to_base_topic}/current_state", 10),
                    "landmarks":    self.create_publisher(String, f"{hand_landmarks_3d_topic}/current_state", 10),
                    "mediapipe":    self.create_publisher(String, f"{hand_mediapipe_topic}/current_state", 10),
                    "palm":         self.create_publisher(String, f"{hand_palm_plane_topic}/current_state", 10),
                        }

        # ---- Hand change publishers ----
        self.hand_pub = self.create_publisher(String, f"{hand_pub_topic}", 10) # Publisher to connect to the mediapipe node
        # ---- Service clients ----
        self.get_states = {
                    "cam2base":     self.create_client(GetState, f"{hand_cam_to_base_topic}/get_state", callback_group=self._async_group)   ,
                    "landmarks":    self.create_client(GetState, f"{hand_landmarks_3d_topic}/get_state", callback_group=self._async_group),
                    "mediapipe":    self.create_client(GetState, f"{hand_mediapipe_topic}/get_state", callback_group=self._async_group),
                    "palm":         self.create_client(GetState, f"{hand_palm_plane_topic}/get_state", callback_group=self._async_group),
                        }

        self.set_states = {
                    "cam2base":     self.create_client(ChangeState, f"{hand_cam_to_base_topic}/change_state", callback_group=self._async_group)  ,
                    "landmarks":    self.create_client(ChangeState, f"{hand_landmarks_3d_topic}/change_state", callback_group=self._async_group) ,
                    "mediapipe":    self.create_client(ChangeState, f"{hand_mediapipe_topic}/change_state", callback_group=self._async_group) ,
                    "palm":         self.create_client(ChangeState, f"{hand_palm_plane_topic}/change_state", callback_group=self._async_group) ,
                        }

        # ---- Timer ----
        self._timer = self.create_timer(poll_period_s, self._poll_and_publish_state, callback_group=self._async_group)

        # ---- Action ----
        self.lifecycle_control_action = ActionServer(
            node=self,
            action_type=LifecycleControl,
            action_name= lf_action_topic, # Comming from arguments
            execute_callback=self.lf_control_execute_callback,
            goal_callback=self.lf_control_goal_callback,
            cancel_callback=self.lf_control_cancel_callback,
            callback_group=self._async_actions)

        self.hand_action = ActionServer(
            node=self,
            action_type=HandChange,
            action_name= hand_action_topic, # Comming from arguments
            execute_callback=self.hand_execute_callback,
            goal_callback=self.hand_goal_callback,
            cancel_callback=self.hand_cancel_callback,
            callback_group=self._async_actions)
# === Action Change State==========================================================================================================
    async def lf_control_execute_callback(self, goal_handle) -> LifecycleControl.Result:
        """Execute the goal, publishing feedback periodically.

        Parameters
        ----------
        goal_handle : rclpy.action.server.ServerGoalHandle
            Goal handle for the active goal.

        Returns
        -------
        example_interfaces.action.ScanProcess.Result
            Final result containing the output.
        """
        feedback_msg = LifecycleControl.Feedback()      # Define Feedback object message
        result_msg = LifecycleControl.Result()          # Define Result object message
        LOG.info("Execute Callback")
        target_state = goal_handle.request.target_state
        # order_of_execution = ["cam2base", "landmarks"] # Using the node keys
        order_of_execution = ["mediapipe", "landmarks", "palm", "cam2base"] # Using the node keys
        success_by_node = {}
        try:
            for node in order_of_execution:
                feedback_msg.feedback = (f"Changing node '{node}' to '{target_state}'")
                goal_handle.publish_feedback(feedback_msg)

                srv_future = self._change_state(state=target_state, node_key=node)

                if srv_future is None:
                    success_by_node[node] = False
                    continue

                response = await srv_future

                # rclpy.spin_until_future_complete(self,srv_future,timeout_sec=5.0,)

                # if not srv_future.done():
                #     self.get_logger().warn(f"Timeout changing node '{node}' to '{target_state}'")
                #     success_by_node[node] = False
                #     continue

                # response = srv_future.result()
                success_by_node[node] = bool(response.success)
                feedback_msg.feedback = (f"Node '{node}' service response: success={response.success}")
                goal_handle.publish_feedback(feedback_msg)

            success = all(success_by_node.values())

            if success: 
                goal_handle.succeed()                           # We MUST return information to the server use one or more of: goal_handle.succeed() / goal_handle.abort() / goal_handle.canceled()
                result_msg.success = True
                result_msg.result_msg = f"Nodes changed succesfully to {target_state}"
                LOG.info(f"Nodes changed succesfully to {target_state}")
            else:
                goal_handle.abort()   
                result_msg.success = False
                result_msg.result_msg = f"Nodes could not be changed to {target_state}"
                self.get_logger().warn(f"Nodes could not be changed to {target_state}")
        
        except Exception as e:
            goal_handle.abort()
            result_msg.success = False
            result_msg.result_msg = (f"Exception while changing nodes state: {str(e)}")
        finally:
            self.busy_with_change = False
            LOG.info(f"Returning result: success={result_msg.success}, "
                f"result_msg='{result_msg.result_msg}'")
        return result_msg 


    def lf_control_goal_callback(self, goal_request: LifecycleControl.Goal) -> GoalResponse:
        """Evaluates the action request before passing it to the execution.

        Parameters
        ----------
        goal_request : LifecycleControl.Goal
            Request part of the LifeCycle action, contains:
                - target_state (string): target state we want to change the system to. It will be evaluated to check if its one of the permitted options. Options are: ["activate", "deactivate"]

        Returns
        -------
        GoalResponse
            _description_
        """
        list_of_options = ["activate", "deactivate", "configure", "cleanup", "shutdown"]

        if goal_request.target_state.lower() in list_of_options and not self.busy_with_change:
            LOG.info("Action request accepted")
            self.busy_with_change = True
            return GoalResponse.ACCEPT
        elif goal_request.target_state.lower() not in list_of_options and not self.busy_with_change:
            self.get_logger().warn(f"Request not in options. Available options: {list_of_options}")
            return GoalResponse.REJECT
        elif self.busy_with_change:
            self.get_logger().warn("Busy with another request. Please try again later")
            return GoalResponse.REJECT
        else:
            self.get_logger().warn("Unkown error")
            return GoalResponse.REJECT

    def lf_control_cancel_callback(self, goal_handle) -> CancelResponse:
        LOG.info("Received cancel request")
        return CancelResponse.ACCEPT

# === Action Change Hand==========================================================================================================
    def hand_execute_callback(self, goal_handle) -> HandChange.Result:
        """Execute the goal, publishing feedback periodically.

        Parameters
        ----------
        goal_handle : rclpy.action.server.ServerGoalHandle
            Goal handle for the active goal.

        Returns
        -------
        example_interfaces.action.ScanProcess.Result
            Final result containing the output.
        """
        feedback_msg = HandChange.Feedback()      # Define Feedback object message
        result_msg = HandChange.Result()          # Define Result object message
        LOG.info("Execute Hand Callback")
        hand = goal_handle.request.hand
        try:
            msg = String()
            msg.data = str(hand)
            self.hand_pub.publish(msg)
            goal_handle.succeed()                           # We MUST return information to the server use one or more of: goal_handle.succeed() / goal_handle.abort() / goal_handle.canceled()
            result_msg.success = True
            result_msg.result_msg = f"Hand changed to {hand}"
            LOG.info(f"Hand changed to {hand}")
        except Exception as e:
            goal_handle.abort()
            result_msg.success = False
            result_msg.result_msg = (f"Exception while changing hand: {str(e)}")
        finally:
            LOG.info(f"Returning result: success={result_msg.success}, "
                f"result_msg='{result_msg.result_msg}'")
        return result_msg 


    def hand_goal_callback(self, goal_request: HandChange.Goal) -> GoalResponse:
        """Evaluates the action request before passing it to the execution.

        Parameters
        ----------
        goal_request : LifecycleControl.Goal
            Request part of the LifeCycle action, contains:
                - target_state (string): target state we want to change the system to. It will be evaluated to check if its one of the permitted options. Options are: ["activate", "deactivate"]

        Returns
        -------
        GoalResponse
            _description_
        """
        list_of_options = ["right", "left", "both"]

        if goal_request.hand.lower() in list_of_options:
            LOG.info("Action request for changing hands accepted")
            return GoalResponse.ACCEPT
        elif goal_request.hand.lower() not in list_of_options:
            self.get_logger().warn(f"Request for changing hands not in options. Available options: {list_of_options}")
            return GoalResponse.REJECT
        else:
            self.get_logger().warn("Unkown error")
            return GoalResponse.REJECT

    def hand_cancel_callback(self, goal_handle) -> CancelResponse:
        LOG.info("Received cancel request")
        return CancelResponse.ACCEPT

# === Get state of all services ==========================================================================================================
    def _poll_and_publish_state(self) -> None:
        """Timer callback: query GetState and publish a string representation. Avoid spamming calls if service is not ready.
        """
        for key, topic_name in self.topic_names.items():
            if not self.get_states[key].service_is_ready():
                self.get_logger().warn(f"get_state service for {topic_name} not ready.", throttle_duration_sec=5.0) # Throttled warning (once every ~5 seconds)            
            else:
                req = GetState.Request()
                # LOG.info("Calling get_state service.")
                future = self.get_states[key].call_async(req)
                future.add_done_callback(partial(self._on_get_state, node_key=key))

    def _on_get_state(self, future, node_key: str) -> None:
        """
        Handle completion of GetState call, publish state.

        Parameters
        ----------
        future
            Future returned by the service call.
        """
        # LOG.info("Retrieving call from get_state service...")
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            LOG.error(f"GetState failed: {exc}")
            return

        state_id = int(resp.current_state.id)
        state_label = str(resp.current_state.label)
        out = String()
        out.data = f"{state_label} ({state_id})"
        # LOG.info(f"Status from service: {out.data}")
        if out.data != self.last_statuses[node_key]:
            LOG.info(f"{self.topic_names[node_key]} state: {self.last_statuses[node_key]} -> {out.data}")
            self.last_statuses[node_key] = out.data
            self.state_publishers[node_key].publish(out)

# === Setting new states in lifecycle nodes =====================================================================================================
    def _change_state(self, state: str, node_key:str):
        """
        Handle incoming lifecycle command messages.

        Expected commands: 'configure', 'activate', 'deactivate', 'cleanup', 'shutdown', ...

        Parameters
        ----------
        msg
            std_msgs/String containing the command.
        """
        # LOG.info(f"Got new transition request {state} for {self.topic_names[node_key]}.")
        # Normalize command strings to simplify parsing.
        cmd = " ".join(state.strip().lower().split())

        # Convert a normalized command to a Transition ID.
        transition_id = self._cmd_to_transition_id.get(cmd)
        LOG.info(f"{self.topic_names[node_key]} Command: {cmd} (ID{transition_id})")

        if transition_id is None:                                               # If the requested state command is unknown
            known = ", ".join(sorted(set(self._cmd_to_transition_id.keys())))
            self.get_logger().warn(f"Unknown cmd '{state}'. Known: {known}")
            return

        # Call ChangeState asynchronously (don't block the subscription callback).
        req = ChangeState.Request()
        req.transition.id = int(transition_id)
        req.transition.id = int(transition_id)
        return self.set_states[node_key].call_async(req)

        # future = self.set_states[node_key].call_async(req)
        # future.add_done_callback(lambda fut: self._on_change_state_done(node_key , cmd, transition_id, fut))

    def _on_change_state_done(self, node_key:str, cmd: str, transition_id: int, future) -> None:
        """
        Handle completion of ChangeState call.

        Parameters
        ----------
        cmd
            Command string requested by the user.
        transition_id
            Transition ID used.
        future
            Future returned by the service call.
        """
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001
            LOG.error(f"ChangeState {self.topic_names[node_key]} '{cmd}' (id={transition_id}) failed: {exc}")
            return
        if resp.success:
            LOG.info(f"ChangeState {self.topic_names[node_key]} OK: '{cmd}' (id={transition_id})")
            self.change_state_success[node_key] = True
        else:
            self.get_logger().warn(f"ChangeState {self.topic_names[node_key]} rejected/failed: '{cmd}' (id={transition_id}).\nTransition may be invalid from current state.")
            self.change_state_success[node_key] = False

# == Services monitoring and warnings ========================
    def wait_for_service(self, timeout_step_sec: float = 1.0) -> None:
            """
            Wait until lifecycle services are available to start the node.

            Parameters
            ----------
            timeout_step_sec
                Time step (seconds) for each wait iteration.
            """
            for key, topic_name in self.topic_names.items():
                if not self.get_states[key].service_is_ready():
                    self.get_logger().warn(f"get_state service for {topic_name} not ready.", throttle_duration_sec=5.0) # Throttled warning (once every ~5 seconds)            
                else:
                    req = GetState.Request()
                    # LOG.info("Calling get_state service.")
                    future = self.get_states[key].call_async(req)
                    future.add_done_callback(partial(self._on_get_state, node_key=key))
                while not self.get_states[key].wait_for_service(timeout_sec=timeout_step_sec):
                    self.get_logger().warn(f"Waiting for {topic_name}/get_state ...", throttle_duration_sec=5.0,)
                while not self.set_states[key].wait_for_service(timeout_sec=timeout_step_sec):
                    self.get_logger().warn( f"Waiting for {topic_name}/change_state ...", throttle_duration_sec=5.0,)

def main():
    rclpy.init()
    node = Orchestrator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    node.wait_for_service()

    try:
        executor.spin()
        # rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()