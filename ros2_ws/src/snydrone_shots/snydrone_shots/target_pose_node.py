#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__("snydrone_target_pose_publisher")
        self.pub = self.create_publisher(PoseStamped, "/snydrone/target/pose", 10)

        # fixed target location for now (meters)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 1.5

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz
        self.get_logger().info("Publishing /snydrone/target/pose at 20 Hz")

    def tick(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"  # we will align frames later

        msg.pose.position.x = self.target_x
        msg.pose.position.y = self.target_y
        msg.pose.position.z = self.target_z

        # identity quaternion
        msg.pose.orientation.w = 1.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TargetPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
