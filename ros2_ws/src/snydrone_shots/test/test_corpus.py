"""Spec for the labelled corpus and its evaluation harness.

This is the honesty check. The corpus claims a ground-truth label for
every shot request; these tests prove the labels are true against the
real gates, not asserted. If a NOMINAL case fails feasibility, or an
INFEASIBLE_GEOMETRIC case is actually flyable, or a swept-path case turns
out to have a waypoint inside the keep-out after all, a test here fails
and the mislabel is surfaced rather than shipped.

The load-bearing property is the last one: every swept-path case must be
individually-waypoint-clear yet segment-blocked, because that is the
exact class a per-waypoint validator misses and the swept-path check
catches. If that property does not hold, the differentiating number the
corpus exists to produce is a fiction.

Pure logic, like the modules it exercises: no ROS, no network, no
simulator.
"""

import json
import math
import os
import sys

import pytest

_HERE = os.path.dirname(__file__)
_SHOTS_ROOT = os.path.join(_HERE, "..")
_BRAIN_ROOT = os.path.join(_HERE, "..", "..", "snydrone_brain")
for _p in (_SHOTS_ROOT, _BRAIN_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from snydrone_brain.shot_spec import (  # noqa: E402
    LIMITS,
    ShotSpecError,
    parse_shot_spec,
)
from snydrone_shots.feasibility import (  # noqa: E402
    DEFAULT_LIMITS,
    check_trajectory,
)
from snydrone_shots.trajectory import sample_trajectory  # noqa: E402
from snydrone_shots.corpus import prompts  # noqa: E402
from snydrone_shots.corpus.prompts import (  # noqa: E402
    AMBIGUOUS,
    CORPUS,
    CORPUS_GATE_HZ,
    INFEASIBLE_DYNAMIC,
    INFEASIBLE_GEOMETRIC,
    LABELS,
    NOMINAL,
    OUT_OF_LIMITS,
)
from snydrone_shots.corpus.evaluate import (  # noqa: E402
    DYNAMIC_KINDS,
    GEOMETRIC_KINDS,
    evaluate,
    naive_waypoint_check,
)


def _limits_for(env):
    merged = dict(DEFAULT_LIMITS)
    merged.update(env or {})
    return merged


def _parse(entry):
    return parse_shot_spec(json.dumps(entry["spec"]))


def _sample_and_check(entry, spec):
    traj = sample_trajectory(entry["target"], spec, CORPUS_GATE_HZ)
    feas = check_trajectory(traj, _limits_for(entry["env"]))
    return traj, feas


def _ids(label):
    return [c["id"] for c in prompts.by_label(label)]


# ------------------------------------------------------------- structure

def test_corpus_is_a_sensible_size():
    # The brief asks for roughly 80 to 120 cases.
    assert 80 <= len(CORPUS) <= 120


def test_every_case_has_the_required_shape():
    for c in CORPUS:
        assert set(c) >= {"id", "label", "spec", "target", "env",
                          "swept", "rationale"}
        assert isinstance(c["spec"], dict)
        assert len(c["target"]) == 3
        assert isinstance(c["env"], dict)
        assert c["rationale"], "case %s has no rationale" % c["id"]


def test_case_ids_are_unique():
    ids = [c["id"] for c in CORPUS]
    assert len(ids) == len(set(ids))


def test_no_duplicate_prompts():
    seen = set()
    for c in CORPUS:
        key = (json.dumps(c["spec"], sort_keys=True), c["target"],
               json.dumps(c["env"], sort_keys=True))
        assert key not in seen, "duplicate prompt at %s" % c["id"]
        seen.add(key)


def test_labels_are_exhaustive_and_mutually_exclusive():
    # Every case carries exactly one label, drawn from the closed set, and
    # every label in the set is actually used.
    used = set()
    for c in CORPUS:
        assert c["label"] in LABELS
        used.add(c["label"])
    assert used == set(LABELS)


# --------------------------------------------------------------- NOMINAL

def test_every_nominal_passes_both_gates():
    for c in prompts.by_label(NOMINAL):
        spec = _parse(c)
        assert spec["clamped"] == [], (
            "NOMINAL %s was clamped at parse: %s" % (c["id"], spec["clamped"]))
        _traj, feas = _sample_and_check(c, spec)
        assert feas["ok"], (
            "NOMINAL %s failed feasibility: %s; the case or the envelope is "
            "wrong" % (c["id"], feas["violations"]))


# --------------------------------------------------------- OUT_OF_LIMITS

def test_every_out_of_limits_clamps_at_parse():
    for c in prompts.by_label(OUT_OF_LIMITS):
        spec = _parse(c)
        assert spec["clamped"], (
            "OUT_OF_LIMITS %s did not clamp; it is inside LIMITS" % c["id"])
        # Every clamped field is genuinely a LIMITS field.
        for field in spec["clamped"]:
            assert field in LIMITS


# ----------------------------------------------------- INFEASIBLE_DYNAMIC

def test_every_dynamic_case_passes_limits_then_fails_on_dynamics_only():
    for c in prompts.by_label(INFEASIBLE_DYNAMIC):
        spec = _parse(c)
        assert spec["clamped"] == [], (
            "INFEASIBLE_DYNAMIC %s clamped at parse, so its defect is "
            "out-of-limits, not dynamics" % c["id"])
        _traj, feas = _sample_and_check(c, spec)
        assert not feas["ok"], (
            "INFEASIBLE_DYNAMIC %s passed feasibility" % c["id"])
        kinds = {v["kind"] for v in feas["violations"]}
        assert kinds, c["id"]
        assert kinds <= DYNAMIC_KINDS, (
            "INFEASIBLE_DYNAMIC %s also trips a geometric bound: %s"
            % (c["id"], sorted(kinds)))


# --------------------------------------------------- INFEASIBLE_GEOMETRIC

def test_every_geometric_case_passes_limits_then_fails_on_geometry_only():
    for c in prompts.by_label(INFEASIBLE_GEOMETRIC):
        spec = _parse(c)
        assert spec["clamped"] == [], (
            "INFEASIBLE_GEOMETRIC %s clamped at parse" % c["id"])
        _traj, feas = _sample_and_check(c, spec)
        assert not feas["ok"], (
            "INFEASIBLE_GEOMETRIC %s passed feasibility" % c["id"])
        kinds = {v["kind"] for v in feas["violations"]}
        assert kinds, c["id"]
        assert kinds <= GEOMETRIC_KINDS, (
            "INFEASIBLE_GEOMETRIC %s also trips a dynamic bound: %s"
            % (c["id"], sorted(kinds)))


# ------------------------------------------------------------- AMBIGUOUS

def test_every_ambiguous_case_is_refused_at_parse():
    for c in prompts.by_label(AMBIGUOUS):
        with pytest.raises(ShotSpecError):
            _parse(c)


# ----------------------------------------------------------- swept-path

def test_there_are_at_least_ten_swept_path_cases():
    assert len(prompts.swept_cases()) >= 10


def test_every_swept_case_is_labelled_geometric():
    for c in prompts.swept_cases():
        assert c["label"] == INFEASIBLE_GEOMETRIC


def test_every_swept_case_is_waypoint_clear_but_segment_blocked():
    # The single most important property in the corpus. Every waypoint must
    # be individually outside the keep-out (a per-waypoint checker sees
    # nothing) while a segment between two waypoints enters it (the
    # swept-path check catches it).
    for c in prompts.swept_cases():
        spec = _parse(c)
        traj, feas = _sample_and_check(c, spec)
        limits = _limits_for(c["env"])
        rho = limits["keep_out_radius_m"]
        cx, cy = limits["keep_out_centre"]
        assert rho > 0.0, "swept case %s has no keep-out" % c["id"]

        # Individually clear: no waypoint is inside the keep-out, so the
        # per-waypoint validator finds nothing at all.
        assert naive_waypoint_check(traj, limits) == [], (
            "swept case %s has a waypoint the per-waypoint check can see; it "
            "is not a genuine swept-path case" % c["id"])
        for (_t, x, y, _z, _yaw) in traj:
            assert math.hypot(x - cx, y - cy) >= rho - 1e-9, c["id"]

        # Segment blocked: the swept-path check reports a keep-out crossing.
        kinds = {v["kind"] for v in feas["violations"]}
        assert "keep_out" in kinds, (
            "swept case %s was not caught by the swept-path check" % c["id"])


# ------------------------------------------------------------- harness

def test_evaluate_confusion_matrix_is_diagonal():
    # The honesty check at the harness level: the system agrees with every
    # ground-truth label, so the confusion matrix has zero off-diagonal
    # mass and there are no disagreements.
    report = evaluate()
    conf = report["confusion"]
    for gt in LABELS:
        for pred in LABELS:
            if gt != pred:
                assert conf[gt][pred] == 0, (
                    "system mislabels %s as %s" % (gt, pred))
    assert report["overall_accuracy"] == 1.0
    assert report["disagreements"] == []


def test_confusion_matrix_row_sums_match_label_counts():
    report = evaluate()
    for label in LABELS:
        row_sum = sum(report["confusion"][label][p] for p in LABELS)
        assert row_sum == len(prompts.by_label(label))


def test_swept_path_delta_is_the_whole_gap():
    # Every swept-path case is caught by the swept-path check and missed by
    # the per-waypoint check, so the delta is the full detection rate.
    report = evaluate()
    sp = report["swept_path"]
    assert sp["total"] == len(prompts.swept_cases())
    assert sp["caught_by_swept_path_check"] == sp["total"]
    assert sp["caught_by_per_waypoint_check"] == 0
    assert sp["swept_path_detection_rate"] == 1.0
    assert sp["per_waypoint_detection_rate"] == 0.0
    assert sp["delta"] == 1.0


def test_report_is_json_serialisable_and_deterministic():
    a = evaluate()
    b = evaluate()
    dumped = json.dumps(a, sort_keys=True)
    assert json.dumps(b, sort_keys=True) == dumped
    # Round-trips without loss.
    assert json.loads(dumped)["overall_accuracy"] == 1.0
