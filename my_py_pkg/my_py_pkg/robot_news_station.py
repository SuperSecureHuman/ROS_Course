#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStation(Node):
    def __init__(self):
        super().__init__("my_robot_new_station")  # Node Name here
    
        self.robotName = "roboNews"
        self.publisher = self.create_publisher(String, "robot_news", 10)
        self.timer = self.create_timer(0.5, self.publish_news)
    
    def publish_news(self):
        msg = String()
        msg.data = f"Helloo, this is {self.robotName} from news station"
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStation()
    rclpy.spin(node)
    rclpy.shutdown

if __name__ == "__main__":
    main()
