"""Spec for resolution-aware coverage.

Coverage answers "was it in frame". That is not the question an
inspection is judged on. A girder photographed from 40 m is in frame
and useless; the defect you are looking for is smaller than a pixel.
The metric practitioners actually specify is ground sample distance,
the real-world size of one pixel, and the whole point of standoff
control is holding it.

So this pairs with coverage.py: same visibility rules, plus a
resolution floor. A target counts only if it was both seen and seen
finely enough.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from snydrone_shots.resolution import (  # noqa: E402
    check_resolution,
    ground_sample_distance,
    max_range_for_gsd,
)

CAM = {
    "hfov_rad": math.radians(60.0),
    "vfov_rad": math.radians(45.0),
    "image_width_px": 1920,
    "min_range_m": 0.5,
    "max_range_m": 60.0,
}


# ------------------------------------------------------------------ gsd

def test_gsd_is_metres_per_pixel_across_the_frame():
    # Frame width at range r is 2 r tan(hfov/2); divide by pixel count.
    r = 10.0
    want = 2 * r * math.tan(math.radians(30.0)) / 1920
    assert ground_sample_distance(r, CAM) == pytest.approx(want)


def test_gsd_scales_linearly_with_range():
    a = ground_sample_distance(5.0, CAM)
    b = ground_sample_distance(10.0, CAM)
    assert b == pytest.approx(2 * a)


def test_a_wider_lens_gives_a_coarser_pixel():
    narrow = ground_sample_distance(10.0, {**CAM, "hfov_rad": math.radians(30.0)})
    wide = ground_sample_distance(10.0, {**CAM, "hfov_rad": math.radians(90.0)})
    assert wide > narrow


def test_more_pixels_give_a_finer_one():
    low = ground_sample_distance(10.0, {**CAM, "image_width_px": 640})
    high = ground_sample_distance(10.0, {**CAM, "image_width_px": 4096})
    assert high < low


def test_zero_range_is_zero_gsd():
    assert ground_sample_distance(0.0, CAM) == pytest.approx(0.0)


@pytest.mark.parametrize("bad", [-1.0, -0.001])
def test_negative_range_is_an_error(bad):
    with pytest.raises(ValueError):
        ground_sample_distance(bad, CAM)


@pytest.mark.parametrize("cam", [
    {**CAM, "image_width_px": 0},
    {**CAM, "hfov_rad": 0.0},
])
def test_a_degenerate_camera_is_an_error(cam):
    with pytest.raises(ValueError):
        ground_sample_distance(10.0, cam)


# -------------------------------------------------------- inverse form

def test_max_range_inverts_gsd():
    # The standoff planning question: how close must I be to resolve this?
    target_gsd = 0.002
    r = max_range_for_gsd(target_gsd, CAM)
    assert ground_sample_distance(r, CAM) == pytest.approx(target_gsd)


def test_max_range_rejects_a_non_positive_requirement():
    for bad in (0.0, -0.01):
        with pytest.raises(ValueError):
            max_range_for_gsd(bad, CAM)


# ------------------------------------------------- resolution coverage

def near(x):
    """One sample at the origin looking along +x, target x metres out."""
    return [(0.0, 0.0, 0.0, 0.0, 0.0)], [(x, 0.0, 0.0)]


def test_a_close_target_meets_a_fine_requirement():
    traj, targets = near(5.0)
    r = check_resolution(traj, targets, CAM, required_gsd_m=0.01)
    assert r["observed"] == [0]
    assert r["fraction"] == pytest.approx(1.0)


def test_the_same_target_further_away_fails_the_same_requirement():
    traj, targets = near(50.0)
    r = check_resolution(traj, targets, CAM, required_gsd_m=0.01)
    assert r["missed"] == [0]


def test_the_best_gsd_achieved_is_reported_per_target():
    traj = [(0.0, 0.0, 0.0, 0.0, 0.0), (1.0, 15.0, 0.0, 0.0, 0.0)]
    targets = [(20.0, 0.0, 0.0)]
    r = check_resolution(traj, targets, CAM, required_gsd_m=1.0)
    # Closest approach is from the second sample, 5 m away.
    assert r["best_gsd"][0] == pytest.approx(
        ground_sample_distance(5.0, CAM))


def test_best_gsd_is_none_for_a_target_never_seen():
    traj = [(0.0, 0.0, 0.0, 0.0, 0.0)]
    targets = [(-10.0, 0.0, 0.0)]          # behind the camera
    r = check_resolution(traj, targets, CAM, required_gsd_m=1.0)
    assert r["missed"] == [0]
    assert r["best_gsd"][0] is None


def test_visibility_still_applies():
    # Close enough for the resolution floor, but out of frame.
    traj = [(0.0, 0.0, 0.0, 0.0, 0.0)]
    targets = [(0.0, 3.0, 0.0)]            # 90 degrees off a 60 degree fov
    r = check_resolution(traj, targets, CAM, required_gsd_m=1.0)
    assert r["missed"] == [0]


def test_beyond_max_range_fails_even_at_a_loose_requirement():
    traj, targets = near(200.0)
    r = check_resolution(traj, targets, CAM, required_gsd_m=10.0)
    assert r["missed"] == [0]


def test_observed_and_missed_partition_the_targets():
    traj = [(0.0, 0.0, 0.0, 0.0, 0.0)]
    targets = [(5.0, 0.0, 0.0), (50.0, 0.0, 0.0), (-5.0, 0.0, 0.0)]
    r = check_resolution(traj, targets, CAM, required_gsd_m=0.01)
    assert sorted(r["observed"] + r["missed"]) == [0, 1, 2]
    assert r["observed"] == [0]


def test_empty_inputs_are_errors():
    with pytest.raises(ValueError):
        check_resolution([], [(1.0, 0.0, 0.0)], CAM, required_gsd_m=0.1)
    with pytest.raises(ValueError):
        check_resolution([(0.0, 0.0, 0.0, 0.0, 0.0)], [], CAM,
                         required_gsd_m=0.1)
