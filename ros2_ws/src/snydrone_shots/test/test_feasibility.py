"""Spec for trajectory feasibility checking.

The published systems in this space validate waypoints, not paths. PEACE
(2026) checks per-waypoint altitude and geofence and nothing else, which
passes a trajectory that is inside the fence at every sample and leaves
it between two of them, and passes one that asks the aircraft to
accelerate harder than it physically can.

This checks the path: speed, acceleration, yaw rate, an altitude band,
and a geofence that is tested along each segment rather than only at its
endpoints.

Everything here is units-agnostic on purpose. It consumes a sampled
trajectory of positions against time, so it is unaffected by the
unresolved question of whether a shot spec's "speed" is angular or
linear.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from snydrone_shots.feasibility import (  # noqa: E402
    DEFAULT_LIMITS,
    check_trajectory,
    describe_violation,
    worst_by_kind,
)


def straight(n=11, dt=0.1, vx=1.0, z=5.0):
    """A benign constant-velocity line, comfortably inside every limit."""
    return [(i * dt, i * dt * vx, 0.0, z, 0.0) for i in range(n)]


def limits(**over):
    out = dict(DEFAULT_LIMITS)
    out.update(over)
    return out


# ------------------------------------------------------------- structure

def test_a_benign_trajectory_passes_cleanly():
    r = check_trajectory(straight(), limits())
    assert r["ok"] is True
    assert r["violations"] == []


def test_result_reports_ok_and_violations():
    r = check_trajectory(straight(), limits())
    assert set(r) >= {"ok", "violations"}
    assert isinstance(r["violations"], list)


def test_ok_is_false_exactly_when_there_are_violations():
    bad = check_trajectory(straight(vx=99.0), limits())
    assert bad["ok"] is False
    assert len(bad["violations"]) > 0


@pytest.mark.parametrize("traj", [[], [(0.0, 0.0, 0.0, 5.0, 0.0)]])
def test_too_short_to_check_is_an_error_not_a_pass(traj):
    # A one-sample trajectory has no segments, so every check vacuously
    # passes. Reporting ok would be the most dangerous possible answer.
    with pytest.raises(ValueError):
        check_trajectory(traj, limits())


def test_non_increasing_time_is_rejected():
    t = straight()
    t[3] = (t[2][0], ) + t[3][1:]
    with pytest.raises(ValueError):
        check_trajectory(t, limits())


# ---------------------------------------------------------------- speed

def test_excess_speed_is_caught_and_located():
    r = check_trajectory(straight(vx=20.0), limits(max_speed_mps=5.0))
    v = [x for x in r["violations"] if x["kind"] == "speed"]
    assert v, "no speed violation reported"
    assert v[0]["index"] == 0
    assert v[0]["limit"] == 5.0
    assert v[0]["value"] == pytest.approx(20.0)


def test_speed_exactly_at_the_limit_passes():
    r = check_trajectory(straight(vx=5.0), limits(max_speed_mps=5.0))
    assert [x for x in r["violations"] if x["kind"] == "speed"] == []


# --------------------------------------------------------- acceleration

def test_a_step_change_in_velocity_is_caught_as_acceleration():
    # Stationary, then abruptly moving: infinite-ish acceleration.
    traj = [(0.0, 0.0, 0.0, 5.0, 0.0),
            (0.1, 0.0, 0.0, 5.0, 0.0),
            (0.2, 2.0, 0.0, 5.0, 0.0),
            (0.3, 4.0, 0.0, 5.0, 0.0)]
    r = check_trajectory(traj, limits(max_speed_mps=100.0,
                                      max_accel_mps2=5.0))
    assert [x for x in r["violations"] if x["kind"] == "acceleration"]


def test_constant_velocity_has_no_acceleration_violation():
    r = check_trajectory(straight(vx=3.0), limits(max_accel_mps2=0.5))
    assert [x for x in r["violations"] if x["kind"] == "acceleration"] == []


# ------------------------------------- centripetal (lateral) acceleration

def circle(radius, speed, hz=2.0, revs=1.0, z=5.0):
    """A constant-speed circular arc: zero tangential acceleration."""
    period = 2.0 * math.pi * radius / speed
    n = max(2, int(period * revs * hz))
    out = []
    for i in range(n + 1):
        t = i / hz
        theta = (speed / radius) * t
        out.append((t, radius * math.cos(theta),
                    radius * math.sin(theta), z, 0.0))
    return out


def test_a_constant_speed_tight_turn_is_caught_as_centripetal():
    # THE gap the tangential check cannot see: speed magnitude is constant
    # so |d speed|/dt is zero, but a 1 m radius at 3 m/s demands 9 m/s^2
    # of lateral thrust against a 6 m/s^2 envelope.
    r = check_trajectory(circle(radius=1.0, speed=3.0),
                         limits(max_accel_mps2=6.0))
    assert [x for x in r["violations"] if x["kind"] == "acceleration"] == [], \
        "constant speed must not read as tangential acceleration"
    assert [x for x in r["violations"] if x["kind"] == "centripetal"], (
        "constant-speed tight turn passed: centripetal load is unchecked")


def test_a_gentle_wide_turn_has_no_centripetal_violation():
    # 10 m radius at 2 m/s is 0.4 m/s^2 of lateral load, nowhere near
    # the envelope.
    r = check_trajectory(circle(radius=10.0, speed=2.0),
                         limits(max_accel_mps2=6.0))
    assert [x for x in r["violations"] if x["kind"] == "centripetal"] == []


def test_centripetal_violation_reports_the_acceleration_envelope():
    r = check_trajectory(circle(radius=1.0, speed=3.0),
                         limits(max_accel_mps2=6.0))
    v = [x for x in r["violations"] if x["kind"] == "centripetal"]
    assert v
    assert v[0]["limit"] == 6.0
    assert v[0]["value"] > 6.0
    assert isinstance(v[0]["index"], int)


def test_chord_correction_recovers_the_true_lateral_load():
    # Raw finite differencing under-reads a curve: a 1 m radius at 2.5 m/s
    # is truly 6.25 m/s^2 of lateral load, but the 2 Hz chords read 5.85
    # raw, under the 6.0 envelope, and the turn would pass. The sinc
    # correction recovers the true value on an arc, so this must refuse.
    r = check_trajectory(circle(radius=1.0, speed=2.5, hz=2.0),
                         limits(max_accel_mps2=6.0))
    v = [x for x in r["violations"] if x["kind"] == "centripetal"]
    assert v, "sampling under-read let a truly infeasible turn pass"
    assert v[0]["value"] == pytest.approx(6.25, abs=1e-6)


def test_corrected_lateral_load_exactly_at_the_envelope_passes():
    # 1.5 m radius at 3 m/s is exactly 6.0 m/s^2. At-limit passes, and the
    # correction must not push the reading spuriously over.
    r = check_trajectory(circle(radius=1.5, speed=3.0, hz=2.0),
                         limits(max_accel_mps2=6.0))
    assert [x for x in r["violations"] if x["kind"] == "centripetal"] == []


def test_correction_is_neutral_on_straight_motion():
    # No direction change, no correction: a straight line stays clean.
    r = check_trajectory(straight(vx=3.0), limits(max_accel_mps2=6.0))
    kinds = {x["kind"] for x in r["violations"]}
    assert "centripetal" not in kinds
    assert "accel_norm" not in kinds


# --------------------------------------------- vector-norm acceleration

def accelerating_circle(radius, v0, a_tan, hz=10.0, duration=1.0, z=5.0):
    """A circular path with speed ramping at a_tan: both components live."""
    out = []
    n = int(duration * hz)
    for i in range(n + 1):
        t = i / hz
        # arc length s = v0 t + a t^2 / 2, angle = s / radius
        theta = (v0 * t + 0.5 * a_tan * t * t) / radius
        out.append((t, radius * math.cos(theta),
                    radius * math.sin(theta), z, 0.0))
    return out


def test_components_each_under_but_norm_over_is_caught():
    # Tangential 4.5 m/s^2 and lateral 4.4 to 5.4 m/s^2 are each under the
    # 6.0 envelope, but their vector sum runs 6.3 to 7.0, more total thrust
    # than the airframe has. Only the norm check can see this. The ramp is
    # kept short so the lateral component never crosses 6.0 on its own.
    traj = accelerating_circle(radius=4.0, v0=4.2, a_tan=4.5,
                               hz=50.0, duration=0.1)
    r = check_trajectory(traj, limits(max_accel_mps2=6.0,
                                      max_speed_mps=100.0))
    kinds = {x["kind"] for x in r["violations"]}
    assert "acceleration" not in kinds, "tangential alone must be under"
    assert "centripetal" not in kinds, "lateral alone must be under"
    assert "accel_norm" in kinds, (
        "components under, vector sum over: the norm check missed it")


def test_norm_is_not_double_reported_when_a_component_already_tripped():
    # A pure tight turn: centripetal is over on its own, so the norm (which
    # is then the same number) must not be reported again.
    r = check_trajectory(circle(radius=1.0, speed=3.0),
                         limits(max_accel_mps2=6.0))
    kinds = [x["kind"] for x in r["violations"]]
    assert "centripetal" in kinds
    assert "accel_norm" not in kinds


# ------------------------------------------------------ non-finite input

@pytest.mark.parametrize("bad", [float("nan"), float("inf"), float("-inf")])
def test_non_finite_samples_are_an_error_not_a_pass(bad):
    # NaN compares false against every limit, so a NaN trajectory would
    # sail through every check and report ok. That must be an error.
    traj = [(0.0, 0.0, 0.0, 5.0, 0.0), (1.0, bad, 0.0, 5.0, 0.0)]
    with pytest.raises(ValueError):
        check_trajectory(traj, limits())


def test_hover_segments_have_no_direction_and_are_skipped():
    # A stationary aircraft has no velocity direction; the centripetal
    # check must not divide by zero or invent a turn.
    traj = [(0.0, 0.0, 0.0, 5.0, 0.0),
            (1.0, 0.0, 0.0, 5.0, 0.0),
            (2.0, 0.0, 0.0, 5.0, 0.0)]
    r = check_trajectory(traj, limits())
    assert [x for x in r["violations"] if x["kind"] == "centripetal"] == []


# -------------------------------------------------------------- yaw rate

def test_yaw_rate_is_caught():
    traj = [(0.0, 0.0, 0.0, 5.0, 0.0),
            (0.1, 0.0, 0.0, 5.0, 3.0)]
    r = check_trajectory(traj, limits(max_yaw_rate_rps=1.0))
    assert [x for x in r["violations"] if x["kind"] == "yaw_rate"]


def test_yaw_rate_uses_the_short_way_around():
    # -3.1 to 3.1 is 0.08 rad the short way, not 6.2. Measuring it the
    # long way would flag a slow, legal turn through the wrap point.
    traj = [(0.0, 0.0, 0.0, 5.0, -3.1),
            (1.0, 0.0, 0.0, 5.0, 3.1)]
    r = check_trajectory(traj, limits(max_yaw_rate_rps=1.0))
    assert [x for x in r["violations"] if x["kind"] == "yaw_rate"] == []


# -------------------------------------------------------------- altitude

def test_below_the_floor_is_caught():
    r = check_trajectory(straight(z=0.2), limits(min_altitude_m=1.0))
    assert [x for x in r["violations"] if x["kind"] == "altitude"]


def test_above_the_ceiling_is_caught():
    r = check_trajectory(straight(z=200.0), limits(max_altitude_m=120.0))
    assert [x for x in r["violations"] if x["kind"] == "altitude"]


# -------------------------------------------------------------- geofence

def test_outside_the_fence_at_a_sample_is_caught():
    r = check_trajectory(straight(vx=10.0), limits(geofence_radius_m=5.0,
                                                   max_speed_mps=100.0))
    assert [x for x in r["violations"] if x["kind"] == "geofence"]


def test_a_segment_fully_inside_the_fence_is_not_flagged():
    traj = [(0.0, -2.0, 3.0, 5.0, 0.0), (4.0, 2.0, 3.0, 5.0, 0.0)]
    r = check_trajectory(traj, limits(geofence_radius_m=25.0,
                                      max_speed_mps=100.0))
    assert [x for x in r["violations"] if x["kind"] == "geofence"] == []


# -------------------------------------------------------- keep-out zone

def test_a_segment_cutting_through_the_keep_out_zone_is_caught():
    # THE point of this module, and the case a per-waypoint validator
    # cannot see. Both endpoints are about 20 m from the subject, well
    # outside a 10 m keep-out cylinder, but the straight line between
    # them passes within 5 m of it. Checking only the samples passes
    # this trajectory and the aircraft flies through the subject.
    traj = [(0.0, -20.0, 5.0, 5.0, 0.0),
            (4.0, 20.0, 5.0, 5.0, 0.0)]
    lim = limits(keep_out_centre=(0.0, 0.0), keep_out_radius_m=10.0,
                 max_speed_mps=100.0, max_accel_mps2=1e9)
    for _, x, y, _, _ in traj:
        assert math.hypot(x, y) > 10.0, "setup wrong: samples must be clear"
    r = check_trajectory(traj, lim)
    assert [x for x in r["violations"] if x["kind"] == "keep_out"], (
        "swept path entered the keep-out zone between samples "
        "and was not caught")


def test_a_segment_that_stays_clear_of_the_keep_out_zone_passes():
    traj = [(0.0, -20.0, 30.0, 5.0, 0.0),
            (4.0, 20.0, 30.0, 5.0, 0.0)]
    r = check_trajectory(traj, limits(keep_out_centre=(0.0, 0.0),
                                      keep_out_radius_m=10.0,
                                      max_speed_mps=100.0))
    assert [x for x in r["violations"] if x["kind"] == "keep_out"] == []


def test_keep_out_is_skipped_when_no_zone_is_configured():
    r = check_trajectory(straight(), limits(keep_out_radius_m=0.0))
    assert [x for x in r["violations"] if x["kind"] == "keep_out"] == []


# ------------------------------------------------------------- reporting

def test_every_violation_carries_kind_index_value_and_limit():
    r = check_trajectory(straight(vx=50.0, z=0.1),
                         limits(max_speed_mps=1.0, min_altitude_m=2.0))
    assert len(r["violations"]) >= 2
    for v in r["violations"]:
        assert set(v) >= {"kind", "index", "value", "limit"}
        assert isinstance(v["index"], int)


def test_violations_are_ordered_by_index():
    r = check_trajectory(straight(n=20, vx=50.0), limits(max_speed_mps=1.0))
    idx = [v["index"] for v in r["violations"]]
    assert idx == sorted(idx)


def test_describe_violation_names_the_limit_and_the_margin():
    # A refusal must say which limit failed and by how much, in units.
    r = check_trajectory(circle(radius=1.0, speed=3.0),
                         limits(max_accel_mps2=6.0))
    v = [x for x in r["violations"] if x["kind"] == "centripetal"][0]
    text = describe_violation(v)
    assert "centripetal" in text
    assert "6.00" in text, "the limit value is not named"
    assert "m/s^2" in text, "the unit is not named"
    assert "by 3.00" in text, "the margin over the limit is not named"


def test_describe_violation_flips_wording_for_below_a_floor():
    r = check_trajectory(straight(z=0.2), limits(min_altitude_m=1.0))
    v = [x for x in r["violations"] if x["kind"] == "altitude"][0]
    text = describe_violation(v)
    assert "below" in text
    assert "by 0.80" in text


def test_worst_by_kind_keeps_the_largest_margin_per_kind():
    # Speeds ramp 10, 20, 30: the worst speed violation is the fastest one.
    traj = [(0.0, 0.0, 0.0, 5.0, 0.0),
            (1.0, 10.0, 0.0, 5.0, 0.0),
            (2.0, 30.0, 0.0, 5.0, 0.0),
            (3.0, 60.0, 0.0, 5.0, 0.0)]
    r = check_trajectory(traj, limits(max_speed_mps=1.0,
                                      max_accel_mps2=1e9))
    worst = worst_by_kind(r["violations"])
    assert worst["speed"]["value"] == pytest.approx(30.0)
