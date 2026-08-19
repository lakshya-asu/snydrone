import math

DEFAULT_CAMERA = {
    "hfov_rad": math.radians(80.0),
    "vfov_rad": math.radians(60.0),
    "min_range_m": 0.5,
    "max_range_m": 40.0,
}

_TOL = 1e-9


def check_coverage(traj, targets, camera=None, normals=None):
    if not traj:
        raise ValueError("traj is empty")
    if not targets:
        raise ValueError("targets is empty")
    if normals is not None and len(normals) != len(targets):
        raise ValueError("normals has different length from targets")

    cam = dict(DEFAULT_CAMERA)
    if camera is not None:
        cam.update(camera)

    hfov_half = cam["hfov_rad"] / 2.0
    vfov_half = cam["vfov_rad"] / 2.0
    min_range = cam["min_range_m"]
    max_range = cam["max_range_m"]

    n_targets = len(targets)
    observed_set = set()

    for (t, x, y, z, yaw) in traj:
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for i in range(n_targets):
            if i in observed_set:
                continue

            tx, ty, tz = targets[i]
            dx = tx - x
            dy = ty - y
            dz = tz - z

            # Range check
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < min_range - _TOL or dist > max_range + _TOL:
                continue

            # Vertical field of view and horizontal field of view
            horiz_dist = math.hypot(dx, dy)
            if horiz_dist < _TOL:
                # Target is directly above or below the camera.
                elevation = math.copysign(math.pi / 2.0, dz) if dz != 0 else 0.0
                if abs(elevation) > vfov_half + _TOL:
                    continue
                # Horizontal bearing is undefined; elevation already
                # constrained, so horizontal is treated as satisfied.
            else:
                elevation = math.atan2(dz, horiz_dist)
                if abs(elevation) > vfov_half + _TOL:
                    continue

                # Horizontal field of view: angle between camera facing
                # (cos_yaw, sin_yaw) and direction to target in xy plane.
                dot = cos_yaw * dx + sin_yaw * dy
                cos_theta = dot / horiz_dist
                cos_theta = max(-1.0, min(1.0, cos_theta))
                theta = math.acos(cos_theta)
                if theta > hfov_half + _TOL:
                    continue

            # Surface normal check (only when normals are supplied)
            if normals is not None:
                nx, ny, nz = normals[i]
                if nx * dx + ny * dy + nz * dz >= 0.0:
                    continue

            observed_set.add(i)

    observed = sorted(observed_set)
    missed = sorted(set(range(n_targets)) - observed_set)
    fraction = len(observed) / n_targets

    return {"fraction": fraction, "observed": observed, "missed": missed}
