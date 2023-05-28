#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import InverseTwo
import numpy as np

class Inverse_Client_2(Node):

    def __init__(self):
        super().__init__("inv_kin_2_client")

        self.call_ik_server(1.0, 0.0)
        self.call_ik_server(0.5, 0.5)



    def call_ik_server(self, x1, y1):

        client = self.create_client(InverseTwo, "inv_kin_2")

        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Inverse Kinematics 2...")
        
        request = InverseTwo.Request()
        request.x1 = x1
        request.y1 = y1

        future = client.call_async(request)

        future.add_done_callback(
            lambda future: self.callback_ik_server(future, x1, y1)
        )

    
    def callback_ik_server(self, future, x1, y1):

        try:
            response = future.result()
            self.get_logger().info(
                f"Angles for x = {x1} and y = {y1} are t1 = {response.t1} and t2 = {response.t2}"
            )

        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of the AddTwoIntsClientNode
    node = Inverse_Client_2()

    # Run the node until it's shutdown
    rclpy.spin(node)

    # Shutdown the ROS client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
