"""Resolution-aware coverage: ground sample distance and related utilities."""

import math

_TOL = 1e-9


def ground_sample_distance(range_m, camera):
    """Return the ground sample distance in metres per pixel.

    gsd = 2 * range_m * tan(hfov_rad / 2) / image_width_px
    """
    if range_m < 0:
        raise ValueError("range_m must be non-negative")
    hfov = camera.get("hfov_rad")
    if hfov is None or hfov <= 0:
        raise ValueError("hfov_rad must be a positive number")
    width = camera.get("image_width_px")
    if width is None or width <= 0:
        raise ValueError("image_width_px must be a positive number")
    if range_m == 0:
        return 0.0
    return 2.0 * range_m * math.tan(hfov / 2.0) / width


def max_range_for_gsd(required_gsd_m, camera):
    """Return the maximum range at which the camera still meets the required GSD."""
    if required_gsd_m <= 0:
        raise ValueError("required_gsd_m must be positive")
    gsd_at_one_metre = ground_sample_distance(1.0, camera)
    return required_gsd_m / gsd_at_one_metre


def _normalize_angle(angle):
    """Wrap an angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def check_resolution(traj, targets, camera, required_gsd_m):
    """Check which targets are observed at the required resolution.

    Returns a dict with keys: fraction, observed, missed, best_gsd.
    """
    if not traj:
        raise ValueError("traj must not be empty")
    if not targets:
        raise ValueError("targets must not be empty")
    if required_gsd_m <= 0:
        raise ValueError("required_gsd_m must be positive")

    hfov = camera["hfov_rad"]
    vfov = camera["vfov_rad"]
    width = camera["image_width_px"]
    min_range = camera.get("min_range_m")
    max_range = camera.get("max_range_m")

    if min_range is None:
        min_range = 0.0
    if max_range is None:
        max_range = float("inf")

    hfov_half = hfov / 2.0
    vfov_half = vfov / 2.0

    best_gsd = [None] * len(targets)

    for sample in traj:
        cx = sample[1]
        cy = sample[2]
        cz = sample[3]
        yaw = sample[4]

        for i, target in enumerate(targets):
            tx = target[0]
            ty = target[1]
            tz = target[2]

            dx = tx - cx
            dy = ty - cy
            dz = tz - cz

            range_m = math.sqrt(dx * dx + dy * dy + dz * dz)

            # Range check (permissive tolerance)
            if range_m < min_range - _TOL or range_m > max_range + _TOL:
                continue

            hdist = math.sqrt(dx * dx + dy * dy)

            # Elevation
            if hdist < _TOL:
                # Degenerate: target directly above or below camera
                if dz > 0:
                    elevation = math.pi / 2.0
                elif dz < 0:
                    elevation = -math.pi / 2.0
                else:
                    elevation = 0.0
            else:
                elevation = math.atan2(dz, hdist)

            # Elevation check (permissive tolerance)
            if abs(elevation) > vfov_half + _TOL:
                continue

            # Bearing check (permissive tolerance)
            if hdist >= _TOL:
                target_bearing = math.atan2(dy, dx)
                angular_diff = _normalize_angle(target_bearing - yaw)
                if abs(angular_diff) > hfov_half + _TOL:
                    continue
            # If hdist < _TOL and we passed the elevation check, the target
            # is directly above or below and we accept it (bearing undefined).

            # Visible: compute GSD and track best
            gsd = 2.0 * range_m * math.tan(hfov / 2.0) / width
            if best_gsd[i] is None or gsd < best_gsd[i]:
                best_gsd[i] = gsd

    observed = []
    missed = []
    for i in range(len(targets)):
        if best_gsd[i] is not None and best_gsd[i] <= required_gsd_m + _TOL:
            observed.append(i)
        else:
            missed.append(i)

    fraction = len(observed) / len(targets)

    return {
        "fraction": fraction,
        "observed": observed,
        "missed": missed,
        "best_gsd": best_gsd,
    }
