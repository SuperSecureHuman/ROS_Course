#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStatusArray
from my_robot_interfaces.srv import SetLed
# from std_msgs.msg import String


class LedPanelNode(Node):

    def __init__(self):
        super().__init__("led_panel")
        self.led_states_ = [0, 0, 0]
        self.publisher_ = self.create_publisher(
            LedStatusArray, "led_status", 10)
        self.status_server_ = self.create_service(
            SetLed, "set_led", self.callback_set_led)
        self.timer_ = self.create_timer(2, self.publish_led_status)
        self.get_logger().info("Led Panel is Ready!")

    def publish_led_status(self):
        msg = LedStatusArray()
        msg.led_status = self.led_states_
        self.publisher_.publish(msg)

    def callback_set_led(self, request, response):
        led_number = request.led_number
        state = request.state

        if led_number >= 4 or led_number < 0 or state not in [0, 1]:
            response.success = False
            return response

        self.led_states_[led_number - 1] = state
        response.success = True
        self.publish_led_status()
        return response


def main(args=None):
    rclpy.init(args=args)
    # Code goes here
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
