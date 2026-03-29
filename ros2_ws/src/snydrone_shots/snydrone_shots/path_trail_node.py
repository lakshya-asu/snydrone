#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PathTrailNode(Node):
    def __init__(self):
        super().__init__("snydrone_path_trail")

        self.sub = self.create_subscription(
            PoseStamped, "/snydrone/setpoint/pose", self.on_pose, 10
        )
        self.pub = self.create_publisher(Path, "/snydrone/setpoint/path", 10)

        self.path = Path()
        self.max_poses = 500

        self.get_logger().info("Publishing /snydrone/setpoint/path")

    def on_pose(self, msg: PoseStamped):
        self.path.header = msg.header
        self.path.poses.append(msg)

        if len(self.path.poses) > self.max_poses:
            self.path.poses.pop(0)

        self.pub.publish(self.path)


def main():
    rclpy.init()
    node = PathTrailNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
