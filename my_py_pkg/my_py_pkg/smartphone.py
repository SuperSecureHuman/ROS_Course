#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class SmartPhoneNode(Node):
    def __init__(self):
        super().__init__("smartphone")
    
        # Create a subscription to the "robot_news" topic and set the callback function
        self.subscriber = self.create_subscription(String, "robot_news", self.callback_robot_news, 10)
        
        # Log a message to indicate that the smartphone is waiting for news
        self.get_logger().info("Smartphone waiting for news")
    
    def callback_robot_news(self, msg):
        # Callback function that is triggered whenever a new message is received on the "robot_news" topic
        # Log the received news message
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the SmartPhoneNode
    node = SmartPhoneNode()
    
    # Run the node until it's shutdown
    rclpy.spin(node)
    
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == "__main__":
    main()
