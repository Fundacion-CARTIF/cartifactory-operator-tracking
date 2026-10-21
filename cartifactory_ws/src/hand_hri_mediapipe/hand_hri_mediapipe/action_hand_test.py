#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

# Replace with your real interface package
from hand_hri_mediapipe_msgs.action import HandChange


class HandChangeClient(Node):

    def __init__(self):
        super().__init__("hand_change_client")

        self._action_client = ActionClient(
            self,
            HandChange,
            "/hand_change_action",
        )

    def send_goal(self, hand: str):
        goal_msg = HandChange.Goal()
        goal_msg.hand = hand

        self.get_logger().info("Waiting for hand change action server...")
        self._action_client.wait_for_server()

        self.get_logger().info(f"Sending hand change request: '{hand}'")

        future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.feedback
        self.get_logger().info(f"Feedback: {feedback}")

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

        self.get_logger().info(f"Result success: {result.success}")
        self.get_logger().info(f"Result message: {result.result_msg}")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print(
            "\nUsage:\n"
            "ros2 run <your_package> hand_change_client <hand>\n\n"
            "Examples:\n"
            "  ros2 run my_package hand_change_client right\n"
            "  ros2 run my_package hand_change_client left\n"
            "  ros2 run my_package hand_change_client both\n"
        )
        return

    hand = sys.argv[1].lower()

    node = HandChangeClient()
    node.send_goal(hand)

    rclpy.spin(node)


if __name__ == "__main__":
    main()