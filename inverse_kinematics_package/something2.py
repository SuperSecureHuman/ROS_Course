import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32


class EndEffectorToJointAnglesNode(Node):
    def __init__(self):
        super().__init__('end_effector_to_joint_angles')
        self.declare_parameter('end_effector_x', 0.0)
        self.declare_parameter('end_effector_y', 0.0)
        self.get_logger().info('Node initialized')

    def update_joint_angles(self):
        end_effector_x = self.get_parameter('end_effector_x').value
        end_effector_y = self.get_parameter('end_effector_y').value

        # Perform inverse kinematics calculations
        joint_angles_str = f"Joint angles: Joint1={end_effector_x}, Joint2={end_effector_y}"
        self.get_logger().info(joint_angles_str)


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorToJointAnglesNode()

    # Load parameters from the launch file
    end_effector_x = float(node.get_parameter('end_effector_x').value)
    end_effector_y = float(node.get_parameter('end_effector_y').value)

    # Update joint angles with the loaded parameters
    node.update_joint_angles()

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
