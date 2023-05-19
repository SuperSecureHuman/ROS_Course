#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_int_server")

        # Create a service named "add_two_ints" with the AddTwoInts service type
        # and set the callback function to handle incoming requests
        self.server_ = self.create_service(
            AddTwoInts, "add_two_ints", self.callback_add_two_ints)

        # Log a message to indicate that the server has started
        self.get_logger().info("Add two int server has started ")

    def callback_add_two_ints(self, request, response):
        # Calculate the sum of the two integers from the request
        response.sum = request.a + request.b

        # Log a message to indicate the result of the addition
        self.get_logger().info(
            f"The sum of {request.a} and {request.b} is {response.sum}")

        # Return the response object
        return response


def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of the AddTwoIntsServerNode
    node = AddTwoIntsServerNode()

    # Run the node until it's shutdown
    rclpy.spin(node)

    # Shutdown the ROS client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
