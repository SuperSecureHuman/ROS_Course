#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32

class NumberGenNode(Node):

    def __init__(self):
        super().__init__("number_publisher")
        
        # Initialize the number variable
        self.number_ = 3
        
        # Create a publisher for the "number" topic
        self.publisher_ = self.create_publisher(Int32, "number", 10)
        
        # Create a timer that triggers the publish_number method every 1 second
        self.timer_ = self.create_timer(1.0, self.publish_number)
        
        # Log a message to indicate that the node has started
        self.get_logger().info("Number Generation has started.")

    def publish_number(self):
        # Create a new Int32 message and set its value to the current number
        msg = Int32()
        msg.data = self.number_
        
        # Publish the number message
        self.publisher_.publish(msg)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    
    # Create an instance of the NumberGenNode
    node = NumberGenNode()
    
    # Run the node until it's shutdown
    rclpy.spin(node)
    
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == "__main__":
    main()
