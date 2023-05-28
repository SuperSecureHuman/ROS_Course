import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='inverse_kinematics_package',
            executable='end_effector_to_joint_angles.py',
            name='end_effector_to_joint_angles',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
