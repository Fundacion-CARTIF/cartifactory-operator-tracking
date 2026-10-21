import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

# Replace with your real package
from hand_hri_mediapipe_msgs.action import LifecycleControl


class LifecycleControlClient(Node):

    def __init__(self):
        super().__init__("lifecycle_control_client")

        self._action_client = ActionClient(
            self,
            LifecycleControl,
            "lifecycle_control",
        )

    def send_goal(self, target_state: str):
        """
        Send a LifecycleControl action goal.

        Parameters
        ----------
        target_state : str
            Lifecycle transition to request.
        """
        goal_msg = LifecycleControl.Goal()
        goal_msg.target_state = target_state

        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        self.get_logger().info(
            f"Sending goal with target_state='{target_state}'"
        )

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        """
        Handle goal response.

        Parameters
        ----------
        future : Future
            Future containing the goal handle.
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.result_callback
        )

    def feedback_callback(self, feedback_msg):
        """
        Receive feedback from the action server.

        Parameters
        ----------
        feedback_msg : LifecycleControl.FeedbackMessage
            Feedback message.
        """
        feedback = feedback_msg.feedback.feedback

        self.get_logger().info(
            f"Feedback: {feedback}"
        )

    def result_callback(self, future):
        response = future.result()

        status = response.status
        result = response.result

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal succeeded")

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn("Goal aborted")

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Goal canceled")

        else:
            self.get_logger().warn(f"Goal finished with status: {status}")

        self.get_logger().info(f"Result success: {result.success}")
        self.get_logger().info(f"Result message: {result.result_msg}")

        rclpy.shutdown()


def main(args=None):
    """
    Main function.
    """
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print(
            "\nUsage:\n"
            "ros2 run <your_package> lifecycle_control_client <target_state>\n"
        )

        print("Examples:")
        print("  ros2 run my_package lifecycle_control_client activate")
        print("  ros2 run my_package lifecycle_control_client deactivate")
        print("  ros2 run my_package lifecycle_control_client configure")
        print("  ros2 run my_package lifecycle_control_client cleanup")
        print("  ros2 run my_package lifecycle_control_client shutdown\n")

        return

    target_state = sys.argv[1]

    node = LifecycleControlClient()

    node.send_goal(target_state)

    rclpy.spin(node)


if __name__ == "__main__":
    main()