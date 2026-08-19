import math


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)


def orbit_setpoint(target, t, spec):
    tx, ty, tz = target
    radius = spec["radius"]
    height = spec["height"]
    speed = spec["speed"]
    clockwise = spec["clockwise"]
    look_at = spec["look_at"]

    if radius <= 0.0:
        raise ValueError("orbit radius must be positive, got %r" % radius)

    # speed is linear meters per second along the orbit path (ratified
    # 2026-08-17), so the angular rate depends on the radius: the same
    # speed takes proportionally longer around a wider orbit.
    theta = (speed / radius) * t
    if clockwise:
        theta = -theta

    x = tx + radius * math.cos(theta)
    y = ty + radius * math.sin(theta)
    z = tz + height

    if look_at == "target":
        yaw = math.atan2(ty - y, tx - x)
    else:
        yaw = 0.0

    return (x, y, z, yaw)
