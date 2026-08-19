"""Spec for orbit trajectory geometry.

Pure math, no ROS, no clock, no publisher. The geometry that decides
where the aircraft goes is the part most worth testing and is currently
the part least reachable by a test, because it lives inside a timer
callback that needs a running node to invoke.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from snydrone_shots.orbit_geometry import (  # noqa: E402
    orbit_setpoint,
    yaw_to_quaternion,
)

TARGET = (10.0, -4.0, 2.0)


def spec(**over):
    base = {"shot": "orbit", "radius": 3.0, "height": 3.5, "speed": 0.6,
            "duration_s": 10.0, "clockwise": True, "look_at": "target"}
    base.update(over)
    return base


# ------------------------------------------------------------- start pose

def test_starts_on_the_positive_x_side_of_the_target():
    x, y, z, _ = orbit_setpoint(TARGET, 0.0, spec())
    assert x == pytest.approx(TARGET[0] + 3.0)
    assert y == pytest.approx(TARGET[1])


def test_height_is_relative_to_the_target_and_constant():
    for t in (0.0, 1.7, 9.9):
        _, _, z, _ = orbit_setpoint(TARGET, t, spec(height=5.0))
        assert z == pytest.approx(TARGET[2] + 5.0)


# ---------------------------------------------------------------- radius

def test_radius_is_held_for_the_whole_orbit():
    for t in [i * 0.37 for i in range(40)]:
        x, y, _, _ = orbit_setpoint(TARGET, t, spec(radius=4.25))
        d = math.hypot(x - TARGET[0], y - TARGET[1])
        assert d == pytest.approx(4.25, abs=1e-9)


# --------------------------------------------------------------- direction

def test_clockwise_and_counterclockwise_go_opposite_ways():
    # Viewed from above with x right and y up, clockwise means y goes
    # negative first from the +x start point.
    _, y_cw, _, _ = orbit_setpoint(TARGET, 0.1, spec(clockwise=True))
    _, y_ccw, _, _ = orbit_setpoint(TARGET, 0.1, spec(clockwise=False))
    assert y_cw < TARGET[1] < y_ccw


def test_the_two_directions_are_mirror_images():
    for t in (0.5, 2.0, 4.5):
        cw = orbit_setpoint(TARGET, t, spec(clockwise=True))
        ccw = orbit_setpoint(TARGET, t, spec(clockwise=False))
        assert cw[0] == pytest.approx(ccw[0])
        assert cw[1] - TARGET[1] == pytest.approx(-(ccw[1] - TARGET[1]))


# ------------------------------------------------------------------ speed
#
# Speed is LINEAR meters per second along the orbit path. Ratified by
# Lakshya 2026-08-17 ("snydrone speed would be m/s of course"). The
# original implementation used it as rad/s, which made the same spec
# fly a 20 m orbit at 6.7 times the ground speed of a 3 m orbit.

def test_speed_is_meters_per_second_along_the_path():
    # In dt seconds at v m/s the aircraft covers v*dt meters of arc.
    v, dt, r = 1.5, 0.01, 4.0
    a = orbit_setpoint(TARGET, 0.0, spec(speed=v, radius=r))
    b = orbit_setpoint(TARGET, dt, spec(speed=v, radius=r))
    step = math.hypot(b[0] - a[0], b[1] - a[1])
    assert step == pytest.approx(v * dt, rel=1e-3)


def test_the_period_scales_with_the_radius():
    # One full lap is 2*pi*r/v seconds. A wider orbit at the same speed
    # must take proportionally longer, which is exactly what the rad/s
    # reading got wrong.
    v = 2.0
    for r in (2.0, 5.0):
        period = 2 * math.pi * r / v
        start = orbit_setpoint(TARGET, 0.0, spec(speed=v, radius=r))
        later = orbit_setpoint(TARGET, period, spec(speed=v, radius=r))
        assert later[0] == pytest.approx(start[0])
        assert later[1] == pytest.approx(start[1])
        halfway = orbit_setpoint(TARGET, period / 2, spec(speed=v, radius=r))
        assert halfway[0] == pytest.approx(TARGET[0] - r)


def test_a_nonpositive_radius_is_rejected():
    # shot_spec clamps radius to at least 1.0, so this only fires on a
    # spec that bypassed validation. Dividing by it silently would spin
    # the aircraft infinitely fast instead.
    with pytest.raises(ValueError):
        orbit_setpoint(TARGET, 1.0, spec(radius=0.0))


def test_zero_speed_holds_position():
    a = orbit_setpoint(TARGET, 0.0, spec(speed=0.0))
    b = orbit_setpoint(TARGET, 30.0, spec(speed=0.0))
    assert a[:3] == pytest.approx(b[:3])


# -------------------------------------------------------------------- yaw

def test_yaw_points_at_the_target():
    for t in [i * 0.61 for i in range(20)]:
        x, y, _, yaw = orbit_setpoint(TARGET, t, spec())
        expected = math.atan2(TARGET[1] - y, TARGET[0] - x)
        # Compare as a direction so that pi and -pi agree.
        assert math.cos(yaw - expected) == pytest.approx(1.0, abs=1e-9)


def test_look_at_none_holds_yaw_at_zero():
    for t in (0.0, 3.3):
        _, _, _, yaw = orbit_setpoint(TARGET, t, spec(look_at="none"))
        assert yaw == pytest.approx(0.0)


# ----------------------------------------------------------- quaternion

def test_quaternion_is_a_pure_yaw_rotation():
    qx, qy, qz, qw = yaw_to_quaternion(1.1)
    assert qx == 0.0 and qy == 0.0
    assert qz == pytest.approx(math.sin(0.55))
    assert qw == pytest.approx(math.cos(0.55))


def test_quaternion_is_normalised():
    for yaw in (-3.0, -0.2, 0.0, 0.9, 3.1):
        q = yaw_to_quaternion(yaw)
        assert math.sqrt(sum(c * c for c in q)) == pytest.approx(1.0)


# ------------------------------------------------------------- robustness

def test_negative_time_does_not_raise():
    orbit_setpoint(TARGET, -2.0, spec())


def test_target_at_origin_is_handled():
    x, y, z, _ = orbit_setpoint((0.0, 0.0, 0.0), 1.0, spec())
    assert math.hypot(x, y) == pytest.approx(3.0)
