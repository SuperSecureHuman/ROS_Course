#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStation(Node):
    def __init__(self):
        super().__init__("my_robot_new_station")  # Set the node name
    
        self.robotName = "roboNews"
        
        # Create a publisher for the "robot_news" topic
        self.publisher = self.create_publisher(String, "robot_news", 10)
        
        # Create a timer that triggers the publish_news method every 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_news)
    
    def publish_news(self):
        # Create a new String message and set its data to a formatted news message
        msg = String()
        msg.data = f"Hello, this is {self.robotName} from the news station"
        
        # Publish the news message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the RobotNewsStation node
    node = RobotNewsStation()
    
    # Run the node until it's shutdown
    rclpy.spin(node)
    
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == "__main__":
    main()
