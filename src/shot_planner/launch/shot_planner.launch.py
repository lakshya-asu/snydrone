from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="shot_planner",
                executable="shot_planner_node",
                name="shot_planner",
                output="screen",
            )
        ]
    )
