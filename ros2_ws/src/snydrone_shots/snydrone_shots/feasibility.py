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

    # A non-finite sample would slide through every limit comparison below
    # (NaN compares false against everything), so "ok" on such a trajectory
    # would be the most dangerous possible answer. Refuse to check it.
    for sample in traj:
        for value in sample:
            if not math.isfinite(value):
                raise ValueError("trajectory contains non-finite values")

    for i in range(1, len(traj)):
        if traj[i][0] <= traj[i - 1][0]:
            raise ValueError("timestamps must be strictly increasing")

    if limits is None:
        limits = {}

    merged = dict(DEFAULT_LIMITS)
    merged.update(limits)

    violations = []
    n = len(traj)

    # Precompute segment velocity vectors and their speeds
    seg_vels = []
    seg_speeds = []
    for i in range(n - 1):
        dt = traj[i + 1][0] - traj[i][0]
        vx = (traj[i + 1][1] - traj[i][1]) / dt
        vy = (traj[i + 1][2] - traj[i][2]) / dt
        vz = (traj[i + 1][3] - traj[i][3]) / dt
        seg_vels.append((vx, vy, vz))
        seg_speeds.append(math.sqrt(vx * vx + vy * vy + vz * vz))

    # Speed check (per segment)
    for i in range(n - 1):
        if _over(seg_speeds[i], merged["max_speed_mps"]):
            violations.append({
                "kind": "speed",
                "index": i,
                "value": seg_speeds[i],
                "limit": merged["max_speed_mps"],
            })

    # Acceleration check (between consecutive segments, reported at middle
    # sample). tang[i] is the tangential component at sample i + 1; kept so
    # the vector-norm check below can combine it with the lateral one.
    tang = [0.0] * (n - 2)
    for i in range(n - 2):
        dt = (traj[i + 2][0] - traj[i][0]) / 2.0
        acc = abs(seg_speeds[i + 1] - seg_speeds[i]) / dt
        tang[i] = acc
        if _over(acc, merged["max_accel_mps2"]):
            violations.append({
                "kind": "acceleration",
                "index": i + 1,
                "value": acc,
                "limit": merged["max_accel_mps2"],
            })

    # Centripetal (lateral) acceleration check. The check above measures
    # only the rate of change of speed MAGNITUDE, so a constant-speed turn
    # reads as zero even when the turn itself demands more lateral thrust
    # than the airframe has. Here the direction change of the velocity
    # vector between consecutive segments gives the lateral component:
    # a_lat = speed * dphi / dt, the finite-difference form of curvature
    # times speed squared. Checked against the same acceleration envelope,
    # since max_accel_mps2 bounds what the airframe can produce in any
    # direction. Segments with (near) zero speed have no direction and are
    # skipped; the tangential check already covers stop-and-go motion.
    #
    # Chord correction: sampled chords under-read a curved path. On an arc
    # the chord speed is the true speed times sinc(dphi/2), so the raw
    # finite difference reads low by exactly that factor and a coarse gate
    # rate under-reads tight turns (radius 1 m at 2.5 m/s is truly
    # 6.25 m/s^2 but reads 5.85 raw at 2 Hz and would pass). Multiplying
    # by (dphi/2)/sin(dphi/2) recovers the true value exactly on constant
    # arcs, tends to 1 on straight motion, and is always >= 1, so the
    # corrected gate is never less strict than the raw one.
    lats = [0.0] * (n - 2)
    for i in range(n - 2):
        v1 = seg_vels[i]
        v2 = seg_vels[i + 1]
        s1 = seg_speeds[i]
        s2 = seg_speeds[i + 1]
        if s1 < 1e-9 or s2 < 1e-9:
            continue
        dot = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / (s1 * s2)
        dphi = math.acos(max(-1.0, min(1.0, dot)))
        dt = (traj[i + 2][0] - traj[i][0]) / 2.0
        lat = 0.5 * (s1 + s2) * dphi / dt
        if dphi > 1e-9:
            lat *= (dphi / 2.0) / math.sin(dphi / 2.0)
        lats[i] = lat
        if _over(lat, merged["max_accel_mps2"]):
            violations.append({
                "kind": "centripetal",
                "index": i + 1,
                "value": lat,
                "limit": merged["max_accel_mps2"],
            })

    # Vector-norm acceleration check. The tangential and lateral components
    # are perpendicular, so the airframe must produce their vector sum. A
    # trajectory whose components are each under the envelope can still
    # demand more total acceleration than the envelope allows; that
    # combination is caught here as its own kind, "accel_norm", against the
    # same max_accel_mps2 (no new physics constant). Reported only where
    # neither component already tripped at that sample, so one over-limit
    # manoeuvre is never double-counted.
    for i in range(n - 2):
        norm = math.hypot(tang[i], lats[i])
        if not _over(norm, merged["max_accel_mps2"]):
            continue
        if _over(tang[i], merged["max_accel_mps2"]):
            continue
        if _over(lats[i], merged["max_accel_mps2"]):
            continue
        violations.append({
            "kind": "accel_norm",
            "index": i + 1,
            "value": norm,
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


# Units per violation kind, for human-readable refusals.
_UNITS = {
    "speed": "m/s",
    "acceleration": "m/s^2",
    "centripetal": "m/s^2",
    "accel_norm": "m/s^2",
    "yaw_rate": "rad/s",
    "altitude": "m",
    "geofence": "m",
    "keep_out": "m",
}

_KIND_NOUN = {
    "speed": "speed",
    "acceleration": "tangential acceleration",
    "centripetal": "centripetal acceleration",
    "accel_norm": "total acceleration",
    "yaw_rate": "yaw rate",
    "altitude": "altitude",
    "geofence": "distance from origin",
    "keep_out": "clearance to the keep-out zone",
}


def describe_violation(v):
    """One violation as a sentence naming the limit and the margin.

    Every violation dict carries kind, value, and limit; this renders it
    as e.g. "centripetal acceleration 9.00 m/s^2 exceeds the 6.00 m/s^2
    limit by 3.00 m/s^2". For the two below-a-floor kinds (altitude under
    the floor, keep-out clearance) the wording flips to "below the
    minimum by". The margin is always stated, so a refusal built from
    these says which limit failed and by how much.
    """
    kind = v["kind"]
    unit = _UNITS.get(kind, "")
    noun = _KIND_NOUN.get(kind, kind)
    value = v["value"]
    limit = v["limit"]
    if value > limit:
        return "%s %.2f %s exceeds the %.2f %s limit by %.2f %s" % (
            noun, value, unit, limit, unit, value - limit, unit)
    return "%s %.2f %s is below the %.2f %s minimum by %.2f %s" % (
        noun, value, unit, limit, unit, limit - value, unit)


def worst_by_kind(violations):
    """The single worst violation of each kind, keyed by kind.

    Worst means the largest margin past the limit, whichever side the
    limit is on. Lets a refusal summarise hundreds of per-sample
    violations as one line per failed limit.
    """
    worst = {}
    for v in violations:
        margin = abs(v["value"] - v["limit"])
        kind = v["kind"]
        if kind not in worst or margin > abs(worst[kind]["value"] - worst[kind]["limit"]):
            worst[kind] = v
    return worst


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
