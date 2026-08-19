#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from snydrone_shots.orbit_geometry import orbit_setpoint, yaw_to_quaternion


class OrbitShotNode(Node):
    def __init__(self):
        super().__init__("snydrone_orbit_shot")

        # Subscribe to target pose
        self.target_sub = self.create_subscription(
            PoseStamped, "/snydrone/target/pose", self.on_target_pose, 10
        )

        # Publish desired drone pose setpoint
        self.sp_pub = self.create_publisher(PoseStamped, "/snydrone/setpoint/pose", 10)

        # Orbit parameters (cinematic knobs). Speed is linear metres per
        # second along the orbit path (ratified 2026-08-17). 0.75 m/s on
        # a 3 m radius matches the 0.25 rad/s this node used to fly.
        self.radius_m = 3.0
        self.height_m = 2.0
        self.speed_mps = 0.75
        self.publish_hz = 20.0

        self.target_pose = None
        self.t0 = self.get_clock().now()

        self.timer = self.create_timer(1.0 / self.publish_hz, self.tick)

        self.get_logger().info("OrbitShotNode running")
        self.get_logger().info(f"Publishing /snydrone/setpoint/pose at {self.publish_hz} Hz")
        self.get_logger().info(f"Orbit radius={self.radius_m}m height={self.height_m}m speed={self.speed_mps}m/s")

    def on_target_pose(self, msg: PoseStamped):
        self.target_pose = msg

    def tick(self):
        if self.target_pose is None:
            return

        # Time since start
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        target = (
            self.target_pose.pose.position.x,
            self.target_pose.pose.position.y,
            self.target_pose.pose.position.z,
        )

        # The geometry lives in the tested pure module, not here. This
        # node only feeds it the clock and the target and ships the
        # result. clockwise=False keeps the direction this node always
        # flew (positive theta, counterclockwise from above).
        spec = {
            "radius": self.radius_m,
            "height": self.height_m,
            "speed": self.speed_mps,
            "clockwise": False,
            "look_at": "target",
        }
        x, y, z, yaw = orbit_setpoint(target, t, spec)
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
