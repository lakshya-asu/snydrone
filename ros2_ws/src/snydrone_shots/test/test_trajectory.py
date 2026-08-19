"""Spec for trajectory sampling.

Turns a validated shot spec into a concrete sequence of timed poses.
This is what the browser demo renders and what the executor will step
through, so it is worth pinning down exactly: an off-by-one in the
sample count or a drifting timestep is the kind of thing that looks
fine on a plot and desynchronises in flight.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from snydrone_shots.trajectory import sample_trajectory  # noqa: E402

TARGET = (2.0, -1.0, 0.5)


def spec(**over):
    base = {"shot": "orbit", "radius": 3.0, "height": 2.0, "speed": 0.5,
            "duration_s": 4.0, "clockwise": True, "look_at": "target"}
    base.update(over)
    return base


# ------------------------------------------------------------- structure

def test_returns_inclusive_sample_count():
    # 4 seconds at 10 Hz is 41 samples, both endpoints included.
    traj = sample_trajectory(TARGET, spec(duration_s=4.0), hz=10.0)
    assert len(traj) == 41


def test_each_sample_has_time_and_pose():
    s = sample_trajectory(TARGET, spec(), hz=5.0)[0]
    assert len(s) == 5
    t, x, y, z, yaw = s
    assert all(isinstance(v, float) for v in (t, x, y, z, yaw))


def test_first_sample_is_at_time_zero_and_last_at_duration():
    traj = sample_trajectory(TARGET, spec(duration_s=3.0), hz=8.0)
    assert traj[0][0] == pytest.approx(0.0)
    assert traj[-1][0] == pytest.approx(3.0)


def test_timesteps_are_evenly_spaced_without_drift():
    traj = sample_trajectory(TARGET, spec(duration_s=7.0), hz=20.0)
    dts = [b[0] - a[0] for a, b in zip(traj, traj[1:])]
    for dt in dts:
        assert dt == pytest.approx(1.0 / 20.0, abs=1e-9)


# -------------------------------------------------------------- geometry

def test_every_sample_sits_on_the_orbit_circle():
    traj = sample_trajectory(TARGET, spec(radius=4.0), hz=15.0)
    for _, x, y, _, _ in traj:
        assert math.hypot(x - TARGET[0], y - TARGET[1]) == pytest.approx(4.0)


def test_height_is_constant_and_relative_to_target():
    traj = sample_trajectory(TARGET, spec(height=6.0), hz=6.0)
    for _, _, _, z, _ in traj:
        assert z == pytest.approx(TARGET[2] + 6.0)


def test_matches_the_geometry_module_pointwise():
    # The sampler must not reimplement the maths, only step it.
    from snydrone_shots.orbit_geometry import orbit_setpoint
    sp = spec()
    for t, x, y, z, yaw in sample_trajectory(TARGET, sp, hz=7.0):
        ex, ey, ez, eyaw = orbit_setpoint(TARGET, t, sp)
        assert (x, y, z) == pytest.approx((ex, ey, ez))
        assert math.cos(yaw - eyaw) == pytest.approx(1.0, abs=1e-9)


def test_direction_is_honoured():
    cw = sample_trajectory(TARGET, spec(clockwise=True), hz=10.0)
    ccw = sample_trajectory(TARGET, spec(clockwise=False), hz=10.0)
    assert cw[3][2] < TARGET[1] < ccw[3][2]


# ------------------------------------------------------------ validation

@pytest.mark.parametrize("hz", [0.0, -5.0])
def test_non_positive_rate_is_rejected(hz):
    with pytest.raises(ValueError):
        sample_trajectory(TARGET, spec(), hz=hz)


def test_zero_duration_still_yields_the_start_pose():
    traj = sample_trajectory(TARGET, spec(duration_s=0.0), hz=10.0)
    assert len(traj) == 1
    assert traj[0][0] == pytest.approx(0.0)


def test_rate_that_does_not_divide_duration_still_ends_exactly_on_it():
    # 1.0 s at 3 Hz: the last step is short rather than overshooting.
    traj = sample_trajectory(TARGET, spec(duration_s=1.0), hz=3.0)
    assert traj[-1][0] == pytest.approx(1.0)
    assert all(a[0] < b[0] for a, b in zip(traj, traj[1:]))
