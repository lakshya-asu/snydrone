"""Spec for coverage verification.

The gap this fills, in the words of the 2026 inspection survey: nothing
on the market verifies that a capture actually met its specification.
Vendors will draw you a coverage mesh to eyeball during the flight and
warn that a strict geofence "may result in gaps in coverage". That is a
warning, not a check.

Given a flown or planned trajectory and the surface you meant to see,
this answers the only question that matters afterwards: which parts did
the camera actually observe, and which did it miss.

Deliberately geometric and units-agnostic. It consumes positions, yaws
and target points, so it is unaffected by the open question of what a
shot spec's "speed" means.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from snydrone_shots.coverage import (  # noqa: E402
    DEFAULT_CAMERA,
    check_coverage,
)


def cam(**over):
    out = dict(DEFAULT_CAMERA)
    out.update(over)
    return out


# One sample, at the origin, looking along +x.
AT_ORIGIN = [(0.0, 0.0, 0.0, 0.0, 0.0)]


# ------------------------------------------------------------- structure

def test_reports_fraction_observed_and_missed():
    r = check_coverage(AT_ORIGIN, [(5.0, 0.0, 0.0)], cam())
    assert set(r) >= {"fraction", "observed", "missed"}
    assert r["fraction"] == pytest.approx(1.0)
    assert r["observed"] == [0]
    assert r["missed"] == []


def test_no_targets_is_an_error_not_full_coverage():
    # Returning 1.0 for an empty target set would report perfect
    # coverage of nothing, which is the most misleading possible answer.
    with pytest.raises(ValueError):
        check_coverage(AT_ORIGIN, [], cam())


def test_empty_trajectory_is_an_error():
    with pytest.raises(ValueError):
        check_coverage([], [(5.0, 0.0, 0.0)], cam())


def test_observed_and_missed_partition_the_targets():
    targets = [(5.0, 0.0, 0.0), (-5.0, 0.0, 0.0), (0.0, 5.0, 0.0)]
    r = check_coverage(AT_ORIGIN, targets, cam())
    assert sorted(r["observed"] + r["missed"]) == [0, 1, 2]
    assert set(r["observed"]).isdisjoint(r["missed"])


# ------------------------------------------------------------ visibility

def test_a_target_straight_ahead_is_observed():
    r = check_coverage(AT_ORIGIN, [(5.0, 0.0, 0.0)], cam())
    assert r["observed"] == [0]


def test_a_target_behind_the_camera_is_not_observed():
    r = check_coverage(AT_ORIGIN, [(-5.0, 0.0, 0.0)], cam())
    assert r["missed"] == [0]


def test_yaw_aims_the_camera():
    # Same target, camera rotated to face it.
    target = [(0.0, 5.0, 0.0)]
    assert check_coverage(AT_ORIGIN, target, cam())["missed"] == [0]
    facing_y = [(0.0, 0.0, 0.0, 0.0, math.pi / 2)]
    assert check_coverage(facing_y, target, cam())["observed"] == [0]


def test_horizontal_field_of_view_is_enforced():
    # 60 degree total hfov: 25 degrees off axis is in, 40 is out.
    c = cam(hfov_rad=math.radians(60.0))
    inside = [(5.0 * math.cos(math.radians(25)),
               5.0 * math.sin(math.radians(25)), 0.0)]
    outside = [(5.0 * math.cos(math.radians(40)),
                5.0 * math.sin(math.radians(40)), 0.0)]
    assert check_coverage(AT_ORIGIN, inside, c)["observed"] == [0]
    assert check_coverage(AT_ORIGIN, outside, c)["missed"] == [0]


def test_vertical_field_of_view_is_enforced():
    c = cam(vfov_rad=math.radians(40.0), max_range_m=100.0)
    # 5 m ahead, 0.5 m up: about 5.7 degrees, inside a +/-20 degree cone.
    assert check_coverage(AT_ORIGIN, [(5.0, 0.0, 0.5)], c)["observed"] == [0]
    # 5 m ahead, 5 m up: 45 degrees, outside it.
    assert check_coverage(AT_ORIGIN, [(5.0, 0.0, 5.0)], c)["missed"] == [0]


# ---------------------------------------------------------------- ranges

def test_a_target_beyond_max_range_is_not_observed():
    r = check_coverage(AT_ORIGIN, [(50.0, 0.0, 0.0)], cam(max_range_m=20.0))
    assert r["missed"] == [0]


def test_a_target_closer_than_min_range_is_not_observed():
    # Too close to focus is as unobserved as too far to resolve.
    r = check_coverage(AT_ORIGIN, [(0.2, 0.0, 0.0)], cam(min_range_m=1.0))
    assert r["missed"] == [0]


# --------------------------------------------------------- surface normals

def test_a_surface_facing_away_is_not_observed():
    # Target 5 m ahead whose normal points further away (+x). The camera
    # is looking at its back face, so it cannot see it.
    targets = [(5.0, 0.0, 0.0)]
    normals = [(1.0, 0.0, 0.0)]
    r = check_coverage(AT_ORIGIN, targets, cam(), normals=normals)
    assert r["missed"] == [0]


def test_a_surface_facing_the_camera_is_observed():
    r = check_coverage(AT_ORIGIN, [(5.0, 0.0, 0.0)], cam(),
                       normals=[(-1.0, 0.0, 0.0)])
    assert r["observed"] == [0]


def test_normals_are_optional():
    assert check_coverage(AT_ORIGIN, [(5.0, 0.0, 0.0)], cam())["fraction"] \
        == pytest.approx(1.0)


def test_mismatched_normals_length_is_an_error():
    with pytest.raises(ValueError):
        check_coverage(AT_ORIGIN, [(5.0, 0.0, 0.0)], cam(),
                       normals=[(1.0, 0.0, 0.0), (1.0, 0.0, 0.0)])


# ----------------------------------------------- accumulation over a path

def test_coverage_accumulates_across_the_trajectory():
    # Neither sample sees both targets; together they see both.
    traj = [(0.0, 0.0, 0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0, 0.0, math.pi / 2)]
    targets = [(5.0, 0.0, 0.0), (0.0, 5.0, 0.0)]
    assert check_coverage([traj[0]], targets, cam())["fraction"] == \
        pytest.approx(0.5)
    assert check_coverage(traj, targets, cam())["fraction"] == \
        pytest.approx(1.0)


def test_fraction_matches_the_observed_count():
    targets = [(5.0, 0.0, 0.0), (-5.0, 0.0, 0.0),
               (0.0, 5.0, 0.0), (0.0, -5.0, 0.0)]
    r = check_coverage(AT_ORIGIN, targets, cam())
    assert r["fraction"] == pytest.approx(len(r["observed"]) / len(targets))


def test_observed_and_missed_are_sorted_and_unique():
    traj = [(float(i), 0.0, 0.0, 0.0, 0.0) for i in range(5)]
    targets = [(5.0, 0.0, 0.0), (6.0, 0.0, 0.0), (-5.0, 0.0, 0.0)]
    r = check_coverage(traj, targets, cam())
    assert r["observed"] == sorted(set(r["observed"]))
    assert r["missed"] == sorted(set(r["missed"]))
