#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


def yaw_to_quaternion(yaw: float):
    # roll = pitch = 0
    # quaternion for yaw around Z
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


class OrbitShotNode(Node):
    def __init__(self):
        super().__init__("snydrone_orbit_shot")

        # Subscribe to target pose
        self.target_sub = self.create_subscription(
            PoseStamped, "/snydrone/target/pose", self.on_target_pose, 10
        )

        # Publish desired drone pose setpoint
        self.sp_pub = self.create_publisher(PoseStamped, "/snydrone/setpoint/pose", 10)

        # Orbit parameters (cinematic knobs)
        self.radius_m = 3.0
        self.height_m = 2.0
        self.angular_speed_rps = 0.25  # radians/sec  (~ 25 sec per full circle)
        self.publish_hz = 20.0

        self.target_pose = None
        self.t0 = self.get_clock().now()

        self.timer = self.create_timer(1.0 / self.publish_hz, self.tick)

        self.get_logger().info("OrbitShotNode running")
        self.get_logger().info(f"Publishing /snydrone/setpoint/pose at {self.publish_hz} Hz")
        self.get_logger().info(f"Orbit radius={self.radius_m}m height={self.height_m}m omega={self.angular_speed_rps}rad/s")

    def on_target_pose(self, msg: PoseStamped):
        self.target_pose = msg

    def tick(self):
        if self.target_pose is None:
            return

        # Time since start
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        # Target position
        tx = self.target_pose.pose.position.x
        ty = self.target_pose.pose.position.y
        tz = self.target_pose.pose.position.z

        # Orbit angle
        theta = self.angular_speed_rps * t

        # Drone position on a circle around target
        x = tx + self.radius_m * math.cos(theta)
        y = ty + self.radius_m * math.sin(theta)
        z = tz + self.height_m

        # Yaw so the drone looks at the target
        yaw = math.atan2(ty - y, tx - x)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = self.target_pose.header.frame_id  # "world"

        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        sp.pose.orientation.x = qx
        sp.pose.orientation.y = qy
        sp.pose.orientation.z = qz
        sp.pose.orientation.w = qw

        self.sp_pub.publish(sp)


def main():
    rclpy.init()
    node = OrbitShotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
