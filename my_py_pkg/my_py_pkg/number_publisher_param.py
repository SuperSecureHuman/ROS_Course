#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int32
# from std_msgs.msg import String


class NumberPublisherNode(Node):

    def __init__(self):
        super().__init__("number_publisher")

        # self.declare_parameter("test")
        self.declare_parameter("number_to_publish", 3)
        self.declare_parameter("publish_frequency", 2.0)

        self.number_ = self.get_parameter("number_to_publish").value
        self.pub_freq_ = self.get_parameter("publish_frequency").value

        self.number_publisher_ = self.create_publisher(Int32, "number", 10)
        self.timer_ = self.create_timer(
            1.0/self.pub_freq_, self.publish_number)
        self.get_logger().info("Number Generation has started.")

    def publish_number(self):
        msg = Int32()
        msg.data = self.number_
        self.number_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    # Code goes here
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
