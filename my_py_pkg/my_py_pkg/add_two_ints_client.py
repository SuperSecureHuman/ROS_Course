#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from example_interfaces.srv import AddTwoInts


class AddTwoIntsClientNode(Node):

    def __init__(self):
        super().__init__("add_two_ints_client")

        # Call the add_two_ints service with inputs 6 and 7
        self.call_add_two_ints(6, 7)

        # Call the add_two_ints service with inputs 9 and 10
        self.call_add_two_ints(9, 10)

    def call_add_two_ints(self, a, b):
        # Create a client to communicate with the add_two_ints service
        client_ = self.create_client(AddTwoInts, "add_two_ints")

        # Wait for the service to become available
        while not client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for Server Add_two_ints ...")

        # Create a request object and set the input values
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Send the request to the service asynchronously
        future_ = client_.call_async(request)

        # Register a callback function to be called when the response is received
        future_.add_done_callback(
            partial(self.callback_call_add_two_ints, a=a, b=b))

    def callback_call_add_two_ints(self, future_, a, b):
        # Try to get the result from the future
        try:
            response = future_.result()
            self.get_logger().info(
                f"{a} + {b} = {response.sum}")
        except Exception as e:
            # If an exception occurs, log an error message
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of the AddTwoIntsClientNode
    node = AddTwoIntsClientNode()

    # Run the node until it's shutdown
    rclpy.spin(node)

    # Shutdown the ROS client library
    rclpy.shutdown()


if __name__ == "__main__":
    main()
