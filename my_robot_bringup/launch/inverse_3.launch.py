from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Start the client of inverse kinematics 3
    ik_3_client_node = Node(
        package="exam_ik",
        executable="ik_3_client",
        name="ik_3_client"
    )

    # Start the server of inverse kinematics 3
    ik_3_server_node = Node(
        package="exam_ik",
        executable="ik_3_server",
        name="ik_3_server"
    )

    ld.add_action(ik_3_client_node)
    ld.add_action(ik_3_server_node)

    return ld