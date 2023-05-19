#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from my_robot_interfaces.action import Countdown


class CountdownActionServerNode(Node):

    def __init__(self):
        super().__init__("countdown_action_server")

        # Create an action server for the Countdown action
        self.countdown_action_server = ActionServer(
            self, Countdown, "countdown", self.callback_countdown)

        # Log a message to indicate that the action server has started
        self.get_logger().info("Countdown Action Server has started.")

    def callback_countdown(self, goal_handle):
        self.get_logger().info("Starting countdown")

        # Create a feedback message
        feedback_msg = Countdown.Feedback()

        # Assign the feedback as the starting number of the goal request
        feedback_msg.current_num = goal_handle.request.starting_num

        while feedback_msg.current_num > 0:
            self.get_logger().info(f"Feedback: {feedback_msg.current_num}")

            # Publish the feedback message to the client
            goal_handle.publish_feedback(feedback_msg)

            feedback_msg.current_num -= 1
            time.sleep(0.5)

        # Indicate that the goal was completed successfully
        goal_handle.succeed()

        # Create a result message
        result = Countdown.Result()
        result.finished_countdown = True
        return result


def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of the CountdownActionServerNode
    node = CountdownActionServerNode()

    # Run the node until it's shutdown
    rclpy.spin(node)

    # Shutdown the ROS client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
