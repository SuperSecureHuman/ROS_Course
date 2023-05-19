#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32

class NumberCounterNode(Node):

    def __init__(self):
        super().__init__("number_counter")
        
        # Initialize the counter variable
        self.counter_ = 0
        
        # Create a subscription to the "number" topic and set the callback function
        self.number_subscriber_ = self.create_subscription(
            Int32, "number", self.callback_number_counter, 10)
        
        # Create a publisher for the "number_counter" topic
        self.counter_publisher_ = self.create_publisher(Int32, "number_counter", 10)
        
        # Log a message to indicate that the node has started
        self.get_logger().info("Number Counter has started!")

    def callback_number_counter(self, msg):
        # Update the counter by adding the received number
        self.counter_ += msg.data
        
        # Log the current value of the counter
        self.get_logger().info(f"{self.counter_}")
        
        # Create a new Int32 message and set its value to the counter
        counter_msg = Int32()
        counter_msg.data = self.counter_
        
        # Publish the counter message
        self.counter_publisher_.publish(counter_msg)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    
    # Create an instance of the NumberCounterNode
    node = NumberCounterNode()
    
    # Run the node until it's shutdown
    rclpy.spin(node)
    
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == "__main__":
    main()
