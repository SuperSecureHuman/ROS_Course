#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from my_robot_interfaces.action import Countdown


class CountdownActionServerNode(Node):

    def __init__(self):
        super().__init__("countdown_action_server")
        self.countdown_action_server = ActionServer(
            self, Countdown, "countdown", self.callback_countdown)
        # self.timer_ = self.create_timer(1, self.publish_hw_status)
        self.get_logger().info("Countdown Action Server has started.")

    def callback_countdown(self, goal_handle):
        #self.get_logger().info(f"@action_cb: goal_handle: {type(goal_handle)}")
        self.get_logger().info("Starting countdown")
        feedback_msg = Countdown.Feedback()

        # Assign the feedback as the starting number of goal request
        feedback_msg.current_num = goal_handle.request.starting_num

        while feedback_msg.current_num > 0:
            self.get_logger().info(f"Feedback : {feedback_msg.current_num}")
            goal_handle.publish_feedback(feedback_msg)
            feedback_msg.current_num -= 1
            time.sleep(0.5)

        # To indicate the goal was completed successfully
        self.get_logger().info(f"@action_cb: goal_handle: {type(goal_handle)}")
        goal_handle.succeed()
        result = Countdown.Result()
        result.finished_countdown = True
        return result


def main(args=None):
    rclpy.init(args=args)
    # Code goes here
    node = CountdownActionServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
