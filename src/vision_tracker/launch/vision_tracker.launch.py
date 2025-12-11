from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision_tracker",
                executable="vision_tracker_node",
                name="vision_tracker",
                output="screen",
            )
        ]
    )
