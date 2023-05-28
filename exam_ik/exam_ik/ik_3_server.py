#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import InverseThree
import numpy as np


# Function to return 3 joint angle, when end effector x,y is given

def compute_angles(x,y):

    L1 = 0.5
    L2 = 0.3
    L3 = 0.2

    ct3 = (x**2 + y**2 - L1**2 - L2**2 - L3**2)/(2*L2*L3)
    st3 = np.sqrt(1 - ct3**2)

    t3 = np.arctan2(st3, ct3)

    ct2 = ((x**2 + y**2 - L1**2 - L2**2)/(2*L1*L2)) - ((L3/L2)*np.cos(t3))
    st2 = np.sqrt(1 - ct2**2)

    t2 = np.arctan2(st2, ct2)

    t1 = np.arctan2(y, x) - np.arctan2(L2*np.sin(t2) + L3*np.sin(t2 + t3), L1 + L2*np.cos(t2) + L3*np.cos(t2 + t3))

    return t1, t2, t3


def find_rechable(x1, y1):

    L1 = 0.5
    L2 = 0.3
    L3 = 0.2

    x = x1
    y = y1

    d = np.sqrt(x**2 + y**2)

    if d > L1 + L2 + L3:
        return 0
    
    elif d < np.abs(L1 - L2 - L3):
        return 1
    else:
        return 2




class Inverse_Server_3(Node):
    def __init__(self):
        super().__init__("inv_kin_3_server")

    
        self.server_ = self.create_service(
            InverseThree, "inv_kin_3", self.callback_inv_kin_3)
    
        self.get_logger().info("Inverse Kinematics 3 Server has started")
    
    def callback_inv_kin_3(self, request, response):

        t1, t2, t3 = compute_angles(request.x1, request.y1)

        t1 = np.rad2deg(t1)
        t2 = np.rad2deg(t2)
        t3 = np.rad2deg(t3)

        response.t1 = t1
        response.t2 = t2
        response.t3 = t3

        self.get_logger().info(
            f"Theta 1 is {response.t1} and Theta 2 is {response.t2} and Theta 3 is {response.t3}")

        rechable_info = find_rechable(request.x1, request.y1)

        # Log the rechable info if its rechable, dextrous or singularity

        if rechable_info == 0:
            self.get_logger().info("The given x,y is not rechable")

        elif rechable_info == 1:
            self.get_logger().info("The given x,y is rechable")

        else:
            self.get_logger().info("The given x,y is rechable")

        return response
    
def main(args=None):

    rclpy.init(args=args)

    node = Inverse_Server_3()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
