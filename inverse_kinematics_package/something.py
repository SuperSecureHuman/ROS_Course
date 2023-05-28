#!/usr/bin/env python3

#ros2 topic pub /end_effector_position example_interfaces/msg/Float32 "{data: [x, y]}"


import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32

class EndEffectorToJointAnglesNode(Node):
    def __init__(self):
        super().__init__('end_effector_to_joint_angles')
        self.subscription = self.create_subscription(
            Float32,
            'end_effector_position',
            self.end_effector_position_callback,
            10
        )

    def end_effector_position_callback(self, msg):
        end_effector_position = msg.data

        # Perform inverse kinematics calculations here to convert
        # end effector position to joint angles
        # Example calculations:
        x = end_effector_position[0]
        y = end_effector_position[1]

        # Perform inverse kinematics calculations
        joint_angles_str = f"Joint angles: Joint1={x}, Joint2={y}"
        self.get_logger().info(joint_angles_str)

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorToJointAnglesNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()




import math

def calculate_inverse_kinematics(x, y):
    # Robot parameters
    link1_length = 1.0
    link2_length = 1.0

    # Calculate joint2 angle
    c2 = (x**2 + y**2 - link1_length**2 - link2_length**2) / (2 * link1_length * link2_length)
    s2 = math.sqrt(1 - c2**2)
    joint2 = math.atan2(s2, c2)

    # Calculate joint1 angle
    k1 = link1_length + link2_length * c2
    k2 = link2_length * s2
    joint1 = math.atan2(y, x) - math.atan2(k2, k1)

    return joint1, joint2

# Example usage
x = 1.5
y = 0.5
joint1, joint2 = calculate_inverse_kinematics(x, y)
print(f"Joint angles: Joint1={joint1}, Joint2={joint2}")


import math

def calculate_inverse_kinematics(x, y, z):
    # Robot parameters
    arm1_length = 1.0
    arm2_length = 1.0
    arm3_length = 1.0

    # Calculate joint angles
    joint1 = math.atan2(y, x)
    joint3 = math.acos((x**2 + y**2 + z**2 - arm1_length**2 - arm2_length**2 - arm3_length**2) / (2 * arm2_length * arm3_length))
    joint2 = math.atan2(z, math.sqrt(x**2 + y**2)) - math.atan2(arm3_length * math.sin(joint3), arm1_length + arm2_length * math.cos(joint3))

    return joint1, joint2, joint3

# Example usage
x = 1.5
y = 0.5
z = 0.8
joint1, joint2, joint3 = calculate_inverse_kinematics(x, y, z)
print(f"Joint angles: Joint1={joint1}, Joint2={joint2}, Joint3={joint3}")



import math
import numpy as np

def calculate_inverse_kinematics(x, y, dh_table):
    num_joints = len(dh_table)

    # Initialize transformation matrices
    T = [np.identity(4) for _ in range(num_joints)]

    # Forward pass to compute the transformation matrices
    for i in range(num_joints):
        a, alpha, d, theta = dh_table[i]

        # Compute the transformation matrix
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        cos_alpha = math.cos(alpha)
        sin_alpha = math.sin(alpha)

        T[i][0, 0] = cos_theta
        T[i][0, 1] = -sin_theta * cos_alpha
        T[i][0, 2] = sin_theta * sin_alpha
        T[i][0, 3] = a * cos_theta

        T[i][1, 0] = sin_theta
        T[i][1, 1] = cos_theta * cos_alpha
        T[i][1, 2] = -cos_theta * sin_alpha
        T[i][1, 3] = a * sin_theta

        T[i][2, 1] = sin_alpha
        T[i][2, 2] = cos_alpha
        T[i][2, 3] = d

    # Compute the desired end effector position as a homogeneous transformation matrix
    desired_position = np.array([[1, 0, 0, x],
                                 [0, 1, 0, y],
                                 [0, 0, 1, 0],
                                 [0, 0, 0, 1]])

    # Initialize joint angles
    joint_angles = [0.0] * num_joints

    # Inverse pass to compute the joint angles
    for i in range(num_joints - 1, -1, -1):
        T_current = np.linalg.inv(T[i])
        T_desired = np.linalg.inv(desired_position)

        # Compute the joint angle
        joint_angles[i] = math.atan2(T_current[1, 0], T_current[0, 0])

        # Update the desired position for the next joint
        desired_position = np.matmul(T_current, T_desired)

    return joint_angles

# Example usage
x = 1.5
y = 0.5

# Define the DH table [a, alpha, d, theta]
dh_table = [
    [1.0, 0, 0, 0],
    [1.0, 0, 0, 0],
]

joint_angles = calculate_inverse_kinematics(x, y, dh_table)
print(f"Joint angles: {joint_angles}")


