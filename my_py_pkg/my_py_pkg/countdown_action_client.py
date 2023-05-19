#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import Countdown


class CountdownActionClientNode(Node):

    def __init__(self):
        super().__init__("countdown_action_client")

        # Create an action client for the Countdown action
        self._countdown_action_client_ = ActionClient(
            self, Countdown, "countdown")

        # Log a message to indicate that the action client has started
        self.get_logger().info("Countdown Action Client has started.")

        # Send a goal with starting number 3
        self.send_goal_countdown(3)
        #self.send_goal_countdown(21)

    def send_goal_countdown(self, starting_num):
        # Create a goal message and set the starting number
        goal_msg = Countdown.Goal()
        goal_msg.starting_num = starting_num

        # Wait for the action server to become available
        while not self._countdown_action_client_.wait_for_server(timeout_sec=2):
            self.get_logger().warn("Waiting for Countdown Action Server...")

        # Send the goal asynchronously and register a feedback callback
        self._send_goal_future_ = self._countdown_action_client_.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback_countdown)
        self._send_goal_future_.add_done_callback(
            self.goal_response_callback_countdown)

    def feedback_callback_countdown(self, feedback_msg):
        # Process the feedback received from the action server
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received Feedback: {feedback.current_num}")

    def goal_response_callback_countdown(self, future):
        # Process the response from the action server after sending the goal
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        # Get the result of the action and register a callback
        self._get_result_future_ = goal_handle.get_result_async()
        self._get_result_future_.add_done_callback(
            self.get_result_callback_countdown)

    def get_result_callback_countdown(self, future):
        # Process the result of the action
        try:
            result = future.result().result
            self.get_logger().info(
                f"Result: Finished Countdown: {result.finished_countdown}")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of the CountdownActionClientNode
    node = CountdownActionClientNode()

    # Run the node until it's shutdown
    rclpy.spin(node)

    # Shutdown the ROS client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
