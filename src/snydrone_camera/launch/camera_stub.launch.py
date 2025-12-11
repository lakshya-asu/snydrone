from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("snydrone_camera"),
        "config",
        "camera_stub.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="snydrone_camera",
                executable="camera_stub_node",
                name="camera_stub",
                output="screen",
                parameters=[config],
            )
        ]
    )
