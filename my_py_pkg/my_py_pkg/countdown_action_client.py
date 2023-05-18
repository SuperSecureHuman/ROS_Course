#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from my_robot_interfaces.action import Countdown


class CountdownActionClientNode(Node):

    def __init__(self):
        super().__init__("countdown_action_client")
        self._countdown_action_client_ = ActionClient(
            self, Countdown, "countdown")
        self.get_logger().info("Countdown Action Client has started.")

        self.send_goal_countdown(3)
        #self.send_goal_countdown(21)

    def send_goal_countdown(self, starting_num):
        goal_msg = Countdown.Goal()
        goal_msg.starting_num = starting_num

        while not self._countdown_action_client_.wait_for_server(timeout_sec=2):
            self.get_logger().warn("Waiting for Countdown Action Server...")

        self._send_goal_future_ = self._countdown_action_client_.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback_countdown)
        self._send_goal_future_.add_done_callback(self.goal_response_callback_countdown)

    def feedback_callback_countdown(self, feedback_msg):
        self.get_logger().info(f"@fbk_cb: feedback_msg: {type(feedback_msg)}")
        feedback = feedback_msg.feedback
        self.get_logger().info(f"@fbk_cb: feedback: {type(feedback)}")
        self.get_logger().info(f"Recieved Feedback: {feedback.current_num}")

    def goal_response_callback_countdown(self, future):
        self.get_logger().info(f"@goal_resp_cb: future: {type(future)}")
        goal_handle = future.result()
        self.get_logger().info(f"@goal_resp_cb: goal_handle: {type(goal_handle)}")

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future_ = goal_handle.get_result_async()
        self._get_result_future_.add_done_callback(self.get_result_callback_countdown)

    def get_result_callback_countdown(self, future):
        self.get_logger().info(f"@goal_resp_cb: future: {type(future)}")
        try:
            result = future.result().result
            self.get_logger().info(f"@get_res_cb: result: {type(result)}")
            self.get_logger().info(
                f"Result: Finished Countdown: {result.finished_countdown}")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
        


def main(args=None):
    rclpy.init(args=args)
    # Code goes here
    node = CountdownActionClientNode()
    #node.send_goal_countdown(10)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
