#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from snydrone_brain.shot_spec import ShotSpecError, parse_shot_spec
from snydrone_shots.orbit_geometry import orbit_setpoint, yaw_to_quaternion


class ShotExecutorNode(Node):
    """Turns an accepted shot spec plus a target pose into pose setpoints.

    All spec validation and clamping lives in shot_spec.parse_shot_spec,
    and all orbit geometry lives in orbit_geometry.orbit_setpoint. Both
    are pure modules with their own unit tests; this node only owns the
    ROS plumbing around them. shot_spec.LIMITS is the single source of
    truth for numeric ranges. This node holds no limit constants of its
    own and never re-clamps.
    """

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
        # The planner already validates before publishing, but this topic
        # is open to anyone with a terminal, so the executor runs the same
        # validation again. Same code path, same LIMITS, so the two can
        # never disagree about what is flyable.
        try:
            spec = parse_shot_spec(msg.data)
        except ShotSpecError as e:
            self.get_logger().error(f"Spec rejected, not flying: {e}")
            return

        clamped = spec.pop("clamped", [])
        if clamped:
            self.get_logger().warn(
                f"clamped to shot_spec.LIMITS: {', '.join(clamped)}")

        self.current_spec = spec
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

    def _pose_msg(self, x, y, z, yaw) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_pose.header.frame_id or "world"

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        return msg

    def hold_target_setpoint(self) -> PoseStamped:
        target = self.target_pose.pose.position
        # Height is relative to the target, matching orbit_geometry.
        z = target.z + self.current_spec["height"]
        return self._pose_msg(target.x, target.y, z, 0.0)

    def compute_orbit_setpoint(self, t: float) -> PoseStamped:
        target = self.target_pose.pose.position
        x, y, z, yaw = orbit_setpoint(
            (target.x, target.y, target.z), t, self.current_spec)
        return self._pose_msg(x, y, z, yaw)


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
