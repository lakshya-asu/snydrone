"""Sample a validated shot spec into a timed trajectory."""

from snydrone_shots.orbit_geometry import orbit_setpoint


def sample_trajectory(target, spec, hz):
    """Sample a shot spec into a sequence of timed poses.

    Args:
        target: 3-tuple (tx, ty, tz) of the orbit center.
        spec: validated shot spec dict with keys radius, height, speed,
              duration_s, clockwise, look_at.
        hz: sample rate in Hz. Must be positive.

    Returns:
        List of (t, x, y, z, yaw) 5-tuples, all floats.
    """
    if hz <= 0:
        raise ValueError("hz must be positive")

    duration = spec["duration_s"]

    if duration == 0:
        x, y, z, yaw = orbit_setpoint(target, 0.0, spec)
        return [(0.0, x, y, z, yaw)]

    n = int(duration * hz)
    if n < 1:
        n = 1

    result = []
    for i in range(n + 1):
        t = duration if i == n else i / hz
        x, y, z, yaw = orbit_setpoint(target, t, spec)
        result.append((t, x, y, z, yaw))

    return result
