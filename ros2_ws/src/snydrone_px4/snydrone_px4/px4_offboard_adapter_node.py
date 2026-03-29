#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleControlMode,
)


def quat_to_yaw(qx, qy, qz, qw) -> float:
    """Quaternion -> yaw (rad). Standard ROS ENU yaw."""
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def enu_to_ned_position(x_enu, y_enu, z_enu):
    """
    ENU -> NED
    ENU: x=East, y=North, z=Up
    NED: x=North, y=East, z=Down
    """
    x_ned = y_enu
    y_ned = x_enu
    z_ned = -z_enu
    return x_ned, y_ned, z_ned


class Px4OffboardAdapter(Node):
    def __init__(self):
        super().__init__("snydrone_px4_offboard_adapter")

        # ---------------------------
        # Params
        # ---------------------------
        self.declare_parameter("setpoint_topic", "/snydrone/setpoint/pose")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("pose_frame", "ENU")  # ENU or NED (what your PoseStamped means)
        self.declare_parameter("auto_arm_and_offboard", True)
        self.declare_parameter("warmup_cycles", 25)  # ~1.25s at 20 Hz

        self.setpoint_topic = self.get_parameter("setpoint_topic").get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.pose_frame = self.get_parameter("pose_frame").get_parameter_value().string_value.upper()
        self.auto_arm_and_offboard = bool(self.get_parameter("auto_arm_and_offboard").value)
        self.warmup_cycles = int(self.get_parameter("warmup_cycles").value)

        # ---------------------------
        # QoS (PX4 uXRCE topics are usually BEST_EFFORT)
        # ---------------------------
        qos_px4_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        qos_px4_in = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---------------------------
        # PX4 publishers
        # ---------------------------
        self.pub_offboard = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_px4_in)
        self.pub_traj = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_px4_in)
        self.pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_px4_in)

        # ---------------------------
        # PX4 subscribers (optional but useful for debug)
        # ---------------------------
        self.sub_vehicle_status = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v1", self.vehicle_status_cb, qos_px4_out
        )
        self.sub_vehicle_ctrl_mode = self.create_subscription(
            VehicleControlMode, "/fmu/out/vehicle_control_mode", self.vehicle_ctrl_mode_cb, qos_px4_out
        )

        # ---------------------------
        # Setpoint input from your shot planner/orbit node
        # ---------------------------
        self.sub_setpoint_pose = self.create_subscription(
            PoseStamped, self.setpoint_topic, self.setpoint_pose_cb, qos_pose
        )

        # ---------------------------
        # State
        # ---------------------------
        self.last_pose: PoseStamped | None = None
        self.vehicle_status: VehicleStatus | None = None
        self.vehicle_control_mode: VehicleControlMode | None = None

        self.sent_offboard = False
        self.sent_arm = False
        self.tick_count = 0

        # timer
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info("PX4 Offboard Adapter running")
        self.get_logger().info(f"Listening: {self.setpoint_topic}")
        self.get_logger().info("Publishing: /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint, /fmu/in/vehicle_command")
        self.get_logger().info(f"pose_frame={self.pose_frame}  rate_hz={self.rate_hz}  warmup_cycles={self.warmup_cycles}")

    # ---------------------------
    # Callbacks
    # ---------------------------
    def setpoint_pose_cb(self, msg: PoseStamped):
        self.last_pose = msg

    def vehicle_status_cb(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def vehicle_ctrl_mode_cb(self, msg: VehicleControlMode):
        self.vehicle_control_mode = msg

    # ---------------------------
    # PX4 Message Helpers
    # ---------------------------
    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.now_us()

        # We are controlling POSITION (TrajectorySetpoint position)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.pub_offboard.publish(msg)

    def publish_trajectory_setpoint_from_pose(self, pose_msg: PoseStamped):
        """
        Convert PoseStamped -> PX4 TrajectorySetpoint (NED).
        This node assumes your PoseStamped position is either ENU or NED depending on param.
        """
        p = pose_msg.pose.position
        q = pose_msg.pose.orientation

        x, y, z = float(p.x), float(p.y), float(p.z)

        if self.pose_frame == "ENU":
            x_ned, y_ned, z_ned = enu_to_ned_position(x, y, z)
        elif self.pose_frame == "NED":
            x_ned, y_ned, z_ned = x, y, z
        else:
            self.get_logger().warn(f"Unknown pose_frame='{self.pose_frame}', defaulting to ENU->NED")
            x_ned, y_ned, z_ned = enu_to_ned_position(x, y, z)

        yaw_enu = quat_to_yaw(q.x, q.y, q.z, q.w)

        # Approx conversion for yaw ENU -> yaw NED:
        # ENU yaw: 0 = East, +CCW (towards North)
        # NED yaw: 0 = North, +CW (towards East)
        # This mapping is: yaw_ned = (pi/2 - yaw_enu)
        yaw_ned = (math.pi / 2.0) - yaw_enu
        # wrap to [-pi, pi]
        yaw_ned = math.atan2(math.sin(yaw_ned), math.cos(yaw_ned))

        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()

        # ✅ New px4_msgs uses arrays:
        # float32[3] position, velocity, acceleration, jerk
        msg.position = [x_ned, y_ned, z_ned]

        nan = float("nan")
        msg.velocity = [nan, nan, nan]
        msg.acceleration = [nan, nan, nan]
        msg.jerk = [nan, nan, nan]

        msg.yaw = float(yaw_ned)
        msg.yawspeed = 0.0

        self.pub_traj.publish(msg)

    def send_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.command = int(command)

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.pub_cmd.publish(msg)

    def arm(self):
        # MAV_CMD_COMPONENT_ARM_DISARM = 400
        self.send_vehicle_command(400, param1=1.0)
        self.get_logger().info("Sent ARM command")

    def disarm(self):
        self.send_vehicle_command(400, param1=0.0)
        self.get_logger().info("Sent DISARM command")

    def set_offboard_mode(self):
        # MAV_CMD_DO_SET_MODE = 176
        # PX4 offboard typically uses param1 = 1 (custom), param2 = PX4_MAIN_MODE_OFFBOARD (6)
        # But many setups accept param2=6.
        self.send_vehicle_command(176, param1=1.0, param2=6.0)
        self.get_logger().info("Sent OFFBOARD mode command")

    # ---------------------------
    # Main Timer
    # ---------------------------
    def timer_cb(self):
        self.tick_count += 1

        # Always publish OffboardControlMode at >2Hz (we do 20Hz)
        self.publish_offboard_control_mode()

        # Publish trajectory setpoint if we have a pose
        if self.last_pose is not None:
            self.publish_trajectory_setpoint_from_pose(self.last_pose)

        # Auto switch + arm (only after warmup setpoints)
        if not self.auto_arm_and_offboard:
            return

        if self.tick_count < self.warmup_cycles:
            return

        # Send offboard once
        if not self.sent_offboard:
            self.set_offboard_mode()
            self.sent_offboard = True

        # Send arm once
        if not self.sent_arm:
            self.arm()
            self.sent_arm = True


def main(args=None):
    rclpy.init(args=args)
    node = Px4OffboardAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
