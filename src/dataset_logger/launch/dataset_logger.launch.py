from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="dataset_logger",
                executable="dataset_logger_node",
                name="dataset_logger",
                output="screen",
            )
        ]
    )
