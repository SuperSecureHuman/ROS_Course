from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Start the client of inverse kinematics 2
    ik_2_client_node = Node(
        package="exam_ik",
        executable="ik_2_client",
        name="ik_2_client"
    )

    # Start the server of inverse kinematics 2
    ik_2_server_node = Node(
        package="exam_ik",
        executable="ik_2_server",
        name="ik_2_server"
    )

    ld.add_action(ik_2_client_node)
    ld.add_action(ik_2_server_node)

    return ld