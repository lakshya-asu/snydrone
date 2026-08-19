import math


DEFAULT_LIMITS = {
    "max_speed_mps": 8.0,
    "max_accel_mps2": 6.0,
    "max_yaw_rate_rps": 2.0,
    "min_altitude_m": 0.5,
    "max_altitude_m": 120.0,
    "geofence_radius_m": 100.0,
    "keep_out_centre": (0.0, 0.0),
    "keep_out_radius_m": 0.0,
}


def check_trajectory(traj, limits=None):
    if len(traj) < 2:
        raise ValueError("trajectory must have at least 2 samples")

    for i in range(1, len(traj)):
        if traj[i][0] <= traj[i - 1][0]:
            raise ValueError("timestamps must be strictly increasing")

    if limits is None:
        limits = {}

    merged = dict(DEFAULT_LIMITS)
    merged.update(limits)

    violations = []
    n = len(traj)

    # Precompute segment speeds
    seg_speeds = []
    for i in range(n - 1):
        dt = traj[i + 1][0] - traj[i][0]
        dist = math.sqrt(
            (traj[i + 1][1] - traj[i][1]) ** 2
            + (traj[i + 1][2] - traj[i][2]) ** 2
            + (traj[i + 1][3] - traj[i][3]) ** 2
        )
        seg_speeds.append(dist / dt)

    # Speed check (per segment)
    for i in range(n - 1):
        if _over(seg_speeds[i], merged["max_speed_mps"]):
            violations.append({
                "kind": "speed",
                "index": i,
                "value": seg_speeds[i],
                "limit": merged["max_speed_mps"],
            })

    # Acceleration check (between consecutive segments, reported at middle sample)
    for i in range(n - 2):
        dt = (traj[i + 2][0] - traj[i][0]) / 2.0
        acc = abs(seg_speeds[i + 1] - seg_speeds[i]) / dt
        if _over(acc, merged["max_accel_mps2"]):
            violations.append({
                "kind": "acceleration",
                "index": i + 1,
                "value": acc,
                "limit": merged["max_accel_mps2"],
            })

    # Yaw rate check (per segment)
    for i in range(n - 1):
        dt = traj[i + 1][0] - traj[i][0]
        dyaw = traj[i + 1][4] - traj[i][4]
        wrapped = math.atan2(math.sin(dyaw), math.cos(dyaw))
        rate = abs(wrapped) / dt
        if _over(rate, merged["max_yaw_rate_rps"]):
            violations.append({
                "kind": "yaw_rate",
                "index": i,
                "value": rate,
                "limit": merged["max_yaw_rate_rps"],
            })

    # Altitude check (per sample)
    for i in range(n):
        z = traj[i][3]
        if _below(z, merged["min_altitude_m"]):
            violations.append({
                "kind": "altitude",
                "index": i,
                "value": z,
                "limit": merged["min_altitude_m"],
            })
        elif _over(z, merged["max_altitude_m"]):
            violations.append({
                "kind": "altitude",
                "index": i,
                "value": z,
                "limit": merged["max_altitude_m"],
            })

    # Geofence check (per sample)
    for i in range(n):
        r = math.hypot(traj[i][1], traj[i][2])
        if _over(r, merged["geofence_radius_m"]):
            violations.append({
                "kind": "geofence",
                "index": i,
                "value": r,
                "limit": merged["geofence_radius_m"],
            })

    # Keep-out check (per segment, swept path)
    if merged["keep_out_radius_m"] > 0:
        cx, cy = merged["keep_out_centre"]
        for i in range(n - 1):
            x1, y1 = traj[i][1], traj[i][2]
            x2, y2 = traj[i + 1][1], traj[i + 1][2]
            dist = _point_segment_distance(cx, cy, x1, y1, x2, y2)
            if _below(dist, merged["keep_out_radius_m"]):
                violations.append({
                    "kind": "keep_out",
                    "index": i,
                    "value": dist,
                    "limit": merged["keep_out_radius_m"],
                })

    violations.sort(key=lambda v: v["index"])

    return {"ok": len(violations) == 0, "violations": violations}


def _over(value, limit):
    return value > limit + max(abs(limit), 1.0) * 1e-9


def _below(value, limit):
    return value < limit - max(abs(limit), 1.0) * 1e-9


def _point_segment_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    len_sq = dx * dx + dy * dy
    if len_sq == 0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / len_sq
    t = max(0.0, min(1.0, t))
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    return math.hypot(px - proj_x, py - proj_y)
