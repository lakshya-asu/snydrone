"""Frame conversion functions for ENU and NED coordinate systems."""

import math


def quat_to_yaw(qx, qy, qz, qw) -> float:
    """Quaternion -> yaw (rad). Standard ROS ENU yaw."""
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw) -> tuple:
    """Pure Z-axis rotation quaternion from yaw (rad). Returns (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


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


def enu_to_ned_yaw(yaw_enu) -> float:
    """Convert ENU yaw to NED yaw, wrapped into (-pi, pi]."""
    yaw_ned = (math.pi / 2.0) - yaw_enu
    yaw_ned = math.atan2(math.sin(yaw_ned), math.cos(yaw_ned))
    return yaw_ned
