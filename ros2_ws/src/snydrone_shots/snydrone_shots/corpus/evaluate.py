"""Evaluation harness: run the corpus through the real flight pipeline.

Every corpus case is pushed through exactly the path a spec takes in
flight: shot_spec.parse_shot_spec, then trajectory.sample_trajectory,
then feasibility.check_trajectory. The system's decision is classified
into one of four outcomes and mapped to a predicted label, which is
scored against the ground-truth label as a confusion matrix.

The headline the corpus exists to produce is the swept-path number: of
the keep-out cases whose waypoints are each individually clear, how many
the swept-path segment check catches, versus how many a per-waypoint-only
checker (the PEACE baseline: altitude band plus geofence plus keep-out,
tested at waypoints, no segments and no dynamics) catches. That gap is
the differentiating result.

Runnable as `python -m snydrone_shots.corpus.evaluate` and importable:
evaluate() returns a plain-dict, JSON-serialisable report, and calling
the module runs evaluate() and prints it. No network, no ROS, no
simulator; the harness is pure logic over the corpus.
"""

import json
import math
import os
import sys


def _ensure_paths():
    """Put the sibling package roots on sys.path.

    shot_spec lives in the snydrone_brain package and the rest in
    snydrone_shots. Inserting both roots lets the harness import them
    whether it is run as a module, imported by a test, or executed from
    an arbitrary working directory.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    shots_root = os.path.dirname(os.path.dirname(here))
    src = os.path.dirname(shots_root)
    brain_root = os.path.join(src, "snydrone_brain")
    for path in (shots_root, brain_root):
        if path not in sys.path:
            sys.path.insert(0, path)


_ensure_paths()

from snydrone_brain.shot_spec import ShotSpecError, parse_shot_spec  # noqa: E402
from snydrone_shots.feasibility import (  # noqa: E402
    DEFAULT_LIMITS,
    check_trajectory,
)
from snydrone_shots.trajectory import sample_trajectory  # noqa: E402
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

# feasibility.py sorts its violations into these two families. The dynamic
# family is the flight envelope (how the aircraft may move); the geometric
# family is where it may be (altitude band, geofence, keep-out).
DYNAMIC_KINDS = frozenset({"speed", "acceleration", "centripetal",
                           "yaw_rate"})
GEOMETRIC_KINDS = frozenset({"altitude", "geofence", "keep_out"})

# System decisions, one per case.
ACCEPTED = "ACCEPTED"
CLAMPED_AT_PARSE = "CLAMPED_AT_PARSE"
REJECTED_AT_PARSE = "REJECTED_AT_PARSE"
REJECTED_AT_FEASIBILITY = "REJECTED_AT_FEASIBILITY"

# Which ground-truth label each decision is the system's way of catching.
DECISION_LABEL = {
    ACCEPTED: NOMINAL,
    CLAMPED_AT_PARSE: OUT_OF_LIMITS,
    REJECTED_AT_PARSE: AMBIGUOUS,
    # REJECTED_AT_FEASIBILITY is split by violation family below.
}


def _over(value, limit):
    """Match feasibility.py: strictly over, with a relative tolerance."""
    return value > limit + max(abs(limit), 1.0) * 1e-9


def _below(value, limit):
    """Match feasibility.py: strictly under, with a relative tolerance."""
    return value < limit - max(abs(limit), 1.0) * 1e-9


def _limits_for(env):
    merged = dict(DEFAULT_LIMITS)
    merged.update(env or {})
    return merged


def _pipeline(entry):
    """Push one case through the real pipeline.

    Returns (result, traj, feasibility_result). result is a dict with the
    decision, the predicted label, and any violation kinds or clamped
    fields. traj and feasibility_result are None when the case never
    reached that stage (a parse rejection or a clamp stops early).
    """
    raw = json.dumps(entry["spec"])
    try:
        # Strict mode: a corpus case that omits a required field is
        # underspecified and must score as AMBIGUOUS, not be silently
        # completed from DEFAULTS. The flight-side callers (executor and
        # planner) stay on the lenient default, where filling omissions
        # from DEFAULTS is the documented planner contract.
        spec = parse_shot_spec(raw, require_complete=True)
    except ShotSpecError as exc:
        return (
            {
                "decision": REJECTED_AT_PARSE,
                "predicted": AMBIGUOUS,
                "kinds": [],
                "clamped": [],
                "detail": str(exc),
            },
            None,
            None,
        )

    clamped = spec.pop("clamped", [])
    if clamped:
        return (
            {
                "decision": CLAMPED_AT_PARSE,
                "predicted": OUT_OF_LIMITS,
                "kinds": [],
                "clamped": list(clamped),
                "detail": "clamped: " + ", ".join(clamped),
            },
            None,
            None,
        )

    traj = sample_trajectory(entry["target"], spec, CORPUS_GATE_HZ)
    feas = check_trajectory(traj, _limits_for(entry["env"]))

    if feas["ok"]:
        return (
            {
                "decision": ACCEPTED,
                "predicted": NOMINAL,
                "kinds": [],
                "clamped": [],
                "detail": "%d waypoints inside the envelope" % len(traj),
            },
            traj,
            feas,
        )

    kinds = sorted({v["kind"] for v in feas["violations"]})
    has_geometric = any(k in GEOMETRIC_KINDS for k in kinds)
    has_dynamic = any(k in DYNAMIC_KINDS for k in kinds)
    # The geometric bounds are the hard safety ones, so a case that trips
    # both families is predicted geometric. mixed is surfaced so such a
    # case never hides.
    predicted = INFEASIBLE_GEOMETRIC if has_geometric else INFEASIBLE_DYNAMIC
    return (
        {
            "decision": REJECTED_AT_FEASIBILITY,
            "predicted": predicted,
            "kinds": kinds,
            "clamped": [],
            "mixed": has_geometric and has_dynamic,
            "detail": "violations: " + ", ".join(kinds),
        },
        traj,
        feas,
    )


def classify(entry):
    """Classify one case: return just the decision/prediction dict."""
    result, _traj, _feas = _pipeline(entry)
    return result


def naive_waypoint_check(traj, limits):
    """A per-waypoint-only validator, standing in for PEACE.

    Tests the altitude band, the geofence, and the keep-out cylinder at
    each waypoint, and nothing else: no segment (swept-path) reasoning and
    no dynamics. Returns the sorted set of violation kinds it can see. On a
    trajectory whose waypoints are each individually clear, it returns an
    empty list even when the swept path is not clear.
    """
    kinds = set()
    ko_radius = limits.get("keep_out_radius_m", 0.0)
    ko_cx, ko_cy = limits.get("keep_out_centre", (0.0, 0.0))
    for (_t, x, y, z, _yaw) in traj:
        if _below(z, limits["min_altitude_m"]) or _over(z, limits["max_altitude_m"]):
            kinds.add("altitude")
        if _over(math.hypot(x, y), limits["geofence_radius_m"]):
            kinds.add("geofence")
        if ko_radius > 0.0:
            if _below(math.hypot(x - ko_cx, y - ko_cy), ko_radius):
                kinds.add("keep_out")
    return sorted(kinds)


def evaluate(corpus=None):
    """Run the whole corpus and return a JSON-serialisable report."""
    if corpus is None:
        corpus = CORPUS

    confusion = {gt: {pred: 0 for pred in LABELS} for gt in LABELS}
    per_label_total = {label: 0 for label in LABELS}
    per_label_correct = {label: 0 for label in LABELS}
    disagreements = []
    mixed_cases = []

    swept_total = 0
    swept_caught_swept = 0
    swept_caught_naive = 0
    swept_intrusions = []

    for entry in corpus:
        result, traj, feas = _pipeline(entry)
        gt = entry["label"]
        pred = result["predicted"]

        confusion[gt][pred] += 1
        per_label_total[gt] += 1
        if gt == pred:
            per_label_correct[gt] += 1
        else:
            disagreements.append(
                {
                    "id": entry["id"],
                    "ground_truth": gt,
                    "predicted": pred,
                    "decision": result["decision"],
                    "detail": result.get("detail", ""),
                    "rationale": entry["rationale"],
                }
            )
        if result.get("mixed"):
            mixed_cases.append({"id": entry["id"], "kinds": result["kinds"]})

        if entry["swept"]:
            swept_total += 1
            limits = _limits_for(entry["env"])
            swept_kinds = {v["kind"] for v in feas["violations"]} if feas else set()
            if "keep_out" in swept_kinds:
                swept_caught_swept += 1
            if naive_waypoint_check(traj, limits):
                swept_caught_naive += 1
            # Illustrative depth: how far inside the keep-out the swept path
            # reaches while every waypoint stays outside it.
            if feas is not None:
                depths = [
                    limits["keep_out_radius_m"] - v["value"]
                    for v in feas["violations"]
                    if v["kind"] == "keep_out"
                ]
                if depths:
                    swept_intrusions.append(max(depths))

    total = len(corpus)
    correct = sum(per_label_correct.values())

    per_label_rate = {
        label: (per_label_correct[label] / per_label_total[label]
                if per_label_total[label] else None)
        for label in LABELS
    }

    swept_rate = (swept_caught_swept / swept_total) if swept_total else None
    naive_rate = (swept_caught_naive / swept_total) if swept_total else None
    delta = (swept_rate - naive_rate) if swept_total else None

    return {
        "corpus_size": total,
        "gate_hz": CORPUS_GATE_HZ,
        "labels": list(LABELS),
        "confusion": confusion,
        "overall_accuracy": correct / total if total else None,
        "correct": correct,
        "per_label_total": per_label_total,
        "per_label_correct": per_label_correct,
        "per_label_accuracy": per_label_rate,
        "disagreements": disagreements,
        "mixed_family_cases": mixed_cases,
        "swept_path": {
            "total": swept_total,
            "caught_by_swept_path_check": swept_caught_swept,
            "caught_by_per_waypoint_check": swept_caught_naive,
            "swept_path_detection_rate": swept_rate,
            "per_waypoint_detection_rate": naive_rate,
            "delta": delta,
            "max_intrusion_m": max(swept_intrusions) if swept_intrusions else None,
            "mean_intrusion_m": (sum(swept_intrusions) / len(swept_intrusions)
                                 if swept_intrusions else None),
        },
    }


def _pct(value):
    return "n/a" if value is None else "%.1f%%" % (100.0 * value)


def format_report(report):
    """Render the report dict as a fixed-width text block."""
    lines = []
    lines.append("SNYdrone shot-corpus evaluation")
    lines.append("=" * 68)
    lines.append(
        "corpus: %d cases, sampled at %.1f Hz into waypoint plans"
        % (report["corpus_size"], report["gate_hz"])
    )
    lines.append("")

    labels = report["labels"]
    short = {
        NOMINAL: "NOMINAL",
        OUT_OF_LIMITS: "OUT_LIM",
        INFEASIBLE_DYNAMIC: "INF_DYN",
        INFEASIBLE_GEOMETRIC: "INF_GEO",
        AMBIGUOUS: "AMBIG",
    }
    header = "%-18s" % "ground truth \\ pred"
    for label in labels:
        header += "%9s" % short[label]
    header += "%8s" % "total"
    lines.append("confusion matrix (rows = ground truth, cols = system)")
    lines.append("-" * len(header))
    lines.append(header)
    for gt in labels:
        row = "%-18s" % short[gt]
        for pred in labels:
            row += "%9d" % report["confusion"][gt][pred]
        row += "%8d" % report["per_label_total"][gt]
        lines.append(row)
    lines.append("-" * len(header))
    lines.append("")

    lines.append("headline metrics")
    lines.append("-" * 68)
    lines.append(
        "overall accuracy          %s  (%d / %d cases)"
        % (_pct(report["overall_accuracy"]), report["correct"],
           report["corpus_size"])
    )
    lines.append("per-label accuracy:")
    for label in labels:
        lines.append(
            "  %-22s %s  (%d / %d)"
            % (label, _pct(report["per_label_accuracy"][label]),
               report["per_label_correct"][label],
               report["per_label_total"][label])
        )
    lines.append("")

    sp = report["swept_path"]
    lines.append("swept-path result (the differentiating number)")
    lines.append("-" * 68)
    lines.append("swept-path keep-out cases          %d" % sp["total"])
    lines.append(
        "caught by swept-path check         %d  (%s)"
        % (sp["caught_by_swept_path_check"], _pct(sp["swept_path_detection_rate"]))
    )
    lines.append(
        "caught by per-waypoint check       %d  (%s)"
        % (sp["caught_by_per_waypoint_check"], _pct(sp["per_waypoint_detection_rate"]))
    )
    lines.append("swept vs per-waypoint delta        %s" % _pct(sp["delta"]))
    if sp["max_intrusion_m"] is not None:
        lines.append(
            "keep-out intrusion depth           max %.3f m, mean %.3f m"
            % (sp["max_intrusion_m"], sp["mean_intrusion_m"])
        )
    lines.append("")

    if report["mixed_family_cases"]:
        lines.append("cases tripping both dynamic and geometric bounds:")
        for case in report["mixed_family_cases"]:
            lines.append("  %s  %s" % (case["id"], ", ".join(case["kinds"])))
        lines.append("")

    lines.append("disagreements (system decision vs ground-truth label)")
    lines.append("-" * 68)
    if not report["disagreements"]:
        lines.append("none: the system agrees with every ground-truth label.")
    else:
        for dis in report["disagreements"]:
            lines.append(
                "  %-8s truth=%s predicted=%s (%s)"
                % (dis["id"], dis["ground_truth"], dis["predicted"],
                   dis["detail"])
            )
    return "\n".join(lines)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    report = evaluate()
    if "--json" in argv:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(format_report(report))
    # Non-zero exit if the system ever disagrees with the corpus, so the
    # harness is usable as a CI gate.
    return 0 if not report["disagreements"] else 1


if __name__ == "__main__":
    sys.exit(main())
