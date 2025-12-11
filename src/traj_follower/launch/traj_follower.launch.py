from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="traj_follower",
                executable="traj_follower_node",
                name="traj_follower",
                output="screen",
            )
        ]
    )
