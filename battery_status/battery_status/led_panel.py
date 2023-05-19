#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStatusArray
from my_robot_interfaces.srv import SetLed

class LedPanelNode(Node):

    def __init__(self):
        super().__init__("led_panel")

        # Initialize the LED states as all off (0)
        self.led_states_ = [0, 0, 0]

        # Create a publisher to publish the LED status
        self.publisher_ = self.create_publisher(LedStatusArray, "led_status", 10)

        # Create a service to handle LED state requests
        self.status_server_ = self.create_service(SetLed, "set_led", self.callback_set_led)

        # Create a timer that calls the publish_led_status function every 2 seconds
        self.timer_ = self.create_timer(2, self.publish_led_status)

        self.get_logger().info("Led Panel is Ready!")

    def publish_led_status(self):
        # Create and publish a message with the current LED states
        msg = LedStatusArray()
        msg.led_status = self.led_states_
        self.publisher_.publish(msg)

    def callback_set_led(self, request, response):
        # Callback function to handle the LED state requests

        led_number = request.led_number
        state = request.state

        # Check if the LED number and state are valid
        if led_number >= 4 or led_number < 0 or state not in [0, 1]:
            response.success = False
            return response

        # Update the LED state and publish the updated status
        self.led_states_[led_number - 1] = state
        response.success = True
        self.publish_led_status()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
