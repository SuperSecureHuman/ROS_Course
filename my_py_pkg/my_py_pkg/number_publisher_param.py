#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32

class NumberPublisherNode(Node):

    def __init__(self):
        super().__init__("number_publisher")

        # Declare and retrieve parameters
        self.declare_parameter("number_to_publish", 3)
        self.declare_parameter("publish_frequency", 2.0)
        self.number_ = self.get_parameter("number_to_publish").value
        self.pub_freq_ = self.get_parameter("publish_frequency").value

        # Create a publisher for the "number" topic
        self.number_publisher_ = self.create_publisher(Int32, "number", 10)

        # Create a timer that triggers the publish_number method at a specified frequency
        self.timer_ = self.create_timer(1.0 / self.pub_freq_, self.publish_number)

        # Log a message to indicate that the node has started
        self.get_logger().info("Number Generation has started.")

    def publish_number(self):
        # Create a new Int32 message and set its value to the current number
        msg = Int32()
        msg.data = self.number_

        # Publish the number message
        self.number_publisher_.publish(msg)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of the NumberPublisherNode
    node = NumberPublisherNode()

    # Run the node until it's shutdown
    rclpy.spin(node)

    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == "__main__":
    main()
