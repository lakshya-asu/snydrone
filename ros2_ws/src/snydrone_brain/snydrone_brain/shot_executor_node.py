#!/usr/bin/env python3

import json
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def quaternion_from_yaw(yaw: float):
    """
    Returns quaternion (x,y,z,w) for yaw-only rotation.
    No numpy, no tf_transformations.
    """
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class ShotExecutorNode(Node):
    def __init__(self):
        super().__init__("snydrone_shot_executor")

        self.current_spec = None
        self.target_pose = None
        self.shot_start_time = None
        self.last_setpoint = None

        self.sub_spec = self.create_subscription(
            String,
            "/snydrone/shot/spec",
            self.on_spec,
            10,
        )

        self.sub_target = self.create_subscription(
            PoseStamped,
            "/snydrone/target/pose",
            self.on_target_pose,
            10,
        )

        self.pub_setpoint = self.create_publisher(
            PoseStamped,
            "/snydrone/setpoint/pose",
            10,
        )

        self.dt = 1.0 / 20.0
        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info("ShotExecutor ready")
        self.get_logger().info("Listening: /snydrone/shot/spec, /snydrone/target/pose")
        self.get_logger().info("Publishing: /snydrone/setpoint/pose @ 20 Hz")

    def on_spec(self, msg: String):
        try:
            spec = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse spec JSON: {e}")
            return

        shot = spec.get("shot", "orbit")
        radius = float(spec.get("radius", 3.0))
        height = float(spec.get("height", 3.5))
        speed = float(spec.get("speed", 0.6))
        duration_s = float(spec.get("duration_s", 10.0))
        clockwise = bool(spec.get("clockwise", True))
        look_at = spec.get("look_at", "target")
        yaw_offset_deg = float(spec.get("yaw_offset_deg", 0.0))

        radius = clamp(radius, 0.5, 50.0)
        height = clamp(height, 0.2, 50.0)
        speed = clamp(speed, 0.05, 10.0)
        duration_s = clamp(duration_s, 0.5, 600.0)

        self.current_spec = {
            "shot": shot,
            "radius": radius,
            "height": height,
            "speed": speed,
            "duration_s": duration_s,
            "clockwise": clockwise,
            "look_at": look_at,
            "yaw_offset_deg": yaw_offset_deg,
        }

        self.shot_start_time = self.get_clock().now()
        self.get_logger().info(f"New spec accepted: {self.current_spec}")

    def on_target_pose(self, msg: PoseStamped):
        self.target_pose = msg

    def on_timer(self):
        if self.current_spec is None:
            return
        if self.target_pose is None:
            return
        if self.shot_start_time is None:
            return

        now = self.get_clock().now()
        t = (now - self.shot_start_time).nanoseconds * 1e-9

        duration = self.current_spec["duration_s"]
        if t > duration:
            if self.last_setpoint is not None:
                self.pub_setpoint.publish(self.last_setpoint)
            return

        shot = self.current_spec["shot"]

        if shot == "orbit":
            sp = self.compute_orbit_setpoint(t)
        else:
            sp = self.hold_target_setpoint()

        self.last_setpoint = sp
        self.pub_setpoint.publish(sp)

    def hold_target_setpoint(self) -> PoseStamped:
        target = self.target_pose.pose.position
        height = self.current_spec["height"]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_pose.header.frame_id or "world"

        msg.pose.position.x = target.x
        msg.pose.position.y = target.y
        msg.pose.position.z = height

        qx, qy, qz, qw = quaternion_from_yaw(0.0)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        return msg

    def compute_orbit_setpoint(self, t: float) -> PoseStamped:
        target = self.target_pose.pose.position

        r = self.current_spec["radius"]
        h = self.current_spec["height"]
        v = self.current_spec["speed"]
        clockwise = self.current_spec["clockwise"]
        look_at = self.current_spec["look_at"]

        omega = v / max(r, 1e-3)
        if clockwise:
            omega = -omega

        theta = omega * t

        x = target.x + r * math.cos(theta)
        y = target.y + r * math.sin(theta)
        z = h

        if look_at == "target":
            yaw = math.atan2(target.y - y, target.x - x)
            yaw += math.radians(self.current_spec["yaw_offset_deg"])
        else:
            yaw = theta + (math.pi / 2.0)

        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_pose.header.frame_id or "world"

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        return msg


def main():
    rclpy.init()
    node = ShotExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
