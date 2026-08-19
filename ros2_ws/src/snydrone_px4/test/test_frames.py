"""Spec for ROS ENU to PX4 NED frame conversion.

Frame conversions are where drone software goes wrong quietly. A sign
error does not raise, it flies east instead of north, and the first
symptom is in the air. These are four-line pure functions and there is
no reason for them to be untested.

Convention, stated once so the assertions below are readable:
  ENU: x = East,  y = North, z = Up     yaw 0 = East,  positive toward North
  NED: x = North, y = East,  z = Down   yaw 0 = North, positive toward East
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from snydrone_px4.frames import (  # noqa: E402
    enu_to_ned_position,
    enu_to_ned_yaw,
    quat_to_yaw,
    yaw_to_quat,
)


# ------------------------------------------------------------- position

def test_east_becomes_east():
    # 1 m East in ENU is (1, 0, 0); in NED east is the y axis.
    assert enu_to_ned_position(1.0, 0.0, 0.0) == pytest.approx((0.0, 1.0, 0.0))


def test_north_becomes_north():
    assert enu_to_ned_position(0.0, 1.0, 0.0) == pytest.approx((1.0, 0.0, 0.0))


def test_up_becomes_negative_down():
    # The sign error that flies a drone into the ground.
    assert enu_to_ned_position(0.0, 0.0, 1.0) == pytest.approx((0.0, 0.0, -1.0))


def test_conversion_is_its_own_inverse():
    # Swapping x with y and negating z twice returns the original, which
    # is a cheap invariant that catches most transcription errors.
    v = (3.5, -2.25, 7.0)
    assert enu_to_ned_position(*enu_to_ned_position(*v)) == pytest.approx(v)


def test_horizontal_distance_is_preserved():
    for v in [(1.0, 2.0, 3.0), (-4.0, 0.5, -1.0), (0.0, 0.0, 0.0)]:
        n = enu_to_ned_position(*v)
        assert math.hypot(n[0], n[1]) == pytest.approx(math.hypot(v[0], v[1]))
        assert abs(n[2]) == pytest.approx(abs(v[2]))


# ------------------------------------------------------------------ yaw

def test_enu_east_is_ned_ninety_degrees():
    # Facing East: ENU yaw 0, NED yaw +pi/2.
    assert enu_to_ned_yaw(0.0) == pytest.approx(math.pi / 2)


def test_enu_north_is_ned_zero():
    assert enu_to_ned_yaw(math.pi / 2) == pytest.approx(0.0)


def test_enu_west_is_ned_negative_ninety():
    assert enu_to_ned_yaw(math.pi) == pytest.approx(-math.pi / 2)


def test_yaw_result_is_always_wrapped():
    # Unwrapped output is the bug this guards: PX4 rejects or misreads
    # a yaw outside (-pi, pi].
    for yaw in [i * 0.37 for i in range(-40, 40)]:
        out = enu_to_ned_yaw(yaw)
        assert -math.pi - 1e-9 <= out <= math.pi + 1e-9


def test_yaw_conversion_is_its_own_inverse():
    for yaw in (-2.0, -0.4, 0.0, 1.3, 3.0):
        back = enu_to_ned_yaw(enu_to_ned_yaw(yaw))
        assert math.cos(back - yaw) == pytest.approx(1.0, abs=1e-9)


# ----------------------------------------------------------- quaternion

def test_quat_to_yaw_inverts_yaw_to_quat():
    for yaw in (-3.0, -1.0, 0.0, 0.75, 2.9):
        q = yaw_to_quat(yaw)
        assert math.cos(quat_to_yaw(*q) - yaw) == pytest.approx(1.0, abs=1e-9)


def test_yaw_to_quat_is_a_pure_yaw_rotation():
    qx, qy, qz, qw = yaw_to_quat(0.8)
    assert qx == 0.0 and qy == 0.0
    assert qz == pytest.approx(math.sin(0.4))
    assert qw == pytest.approx(math.cos(0.4))


def test_identity_quaternion_is_zero_yaw():
    assert quat_to_yaw(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)


def test_quat_to_yaw_ignores_roll_and_pitch_magnitude():
    # Yaw extraction must stay stable when the quaternion is not a pure
    # yaw rotation, which is what actually arrives from a simulator.
    y = quat_to_yaw(0.0, 0.0, math.sin(0.5), math.cos(0.5))
    assert y == pytest.approx(1.0)
