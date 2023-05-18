#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32
#from std_msgs.msg import String

class NumberCounterNode(Node):

    def __init__(self):
        super().__init__("number_counter")
        self.counter_ = 0
        self.number_subscriber_ = self.create_subscription(
            Int32, "number", self.callback_number_counter, 10)
        self.counter_publisher_ = self.create_publisher(Int32, "number_counter",10)
        # Create a subscription, names the topic,callback function which reieves msg from topic, queue size is 10
        self.get_logger().info("Number Counter has started!")

    def callback_number_counter(self, msg):
        self.counter_ += msg.data
        self.get_logger().info(f"{self.counter_}")
        counter_msg = Int32()
        counter_msg.data = self.counter_
        self.counter_publisher_.publish(counter_msg)

    

def main(args=None):
    rclpy.init(args=args)
    #Code goes here
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
