"""Spec for the shot-spec validation layer.

This test is the specification. It runs without ROS, without a network,
and without an API key, which is the whole point: the parsing and
validation of a model-authored flight command is pure logic and should
never have needed a drone stack to exercise.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from snydrone_brain.shot_spec import (  # noqa: E402
    DEFAULTS,
    LIMITS,
    ShotSpecError,
    parse_shot_spec,
)


# --------------------------------------------------------------- unwrapping

@pytest.mark.parametrize("raw", [
    '{"shot": "orbit"}',
    '```json\n{"shot": "orbit"}\n```',
    '```\n{"shot": "orbit"}\n```',
    '   {"shot": "orbit"}   ',
    'Sure, here you go:\n```json\n{"shot": "orbit"}\n```\n',
])
def test_unwraps_whatever_the_model_wrapped_it_in(raw):
    assert parse_shot_spec(raw)["shot"] == "orbit"


@pytest.mark.parametrize("raw", ["", "   ", "not json at all", "[1, 2, 3]",
                                 '"a bare string"', "null", "{"])
def test_rejects_unparseable_or_non_object(raw):
    with pytest.raises(ShotSpecError):
        parse_shot_spec(raw)


# ----------------------------------------------------------------- defaults

def test_every_missing_field_gets_its_default():
    spec = parse_shot_spec("{}")
    for key, value in DEFAULTS.items():
        assert spec[key] == value, f"{key} did not default correctly"


def test_supplied_fields_are_not_overwritten_by_defaults():
    spec = parse_shot_spec('{"radius": 4.5, "clockwise": false}')
    assert spec["radius"] == 4.5
    assert spec["clockwise"] is False
    assert spec["height"] == DEFAULTS["height"]


# ------------------------------------------------- strict (complete) mode

def _complete_raw():
    import json
    return json.dumps(dict(DEFAULTS))


def test_strict_mode_accepts_a_complete_spec():
    spec = parse_shot_spec(_complete_raw(), require_complete=True)
    for key, value in DEFAULTS.items():
        assert spec[key] == value


def test_strict_mode_refuses_an_omitted_field_by_name():
    with pytest.raises(ShotSpecError) as exc_info:
        parse_shot_spec('{"shot": "orbit", "radius": 5.0, "height": 4.0, '
                        '"speed": 1.0, "duration_s": 10.0, '
                        '"clockwise": true}', require_complete=True)
    assert "look_at" in str(exc_info.value)


def test_strict_mode_names_every_omitted_field():
    with pytest.raises(ShotSpecError) as exc_info:
        parse_shot_spec('{"shot": "orbit"}', require_complete=True)
    message = str(exc_info.value)
    for field in DEFAULTS:
        if field == "shot":
            continue
        assert field in message, f"missing field {field} not reported"


def test_strict_mode_refuses_the_empty_object():
    with pytest.raises(ShotSpecError):
        parse_shot_spec("{}", require_complete=True)


def test_lenient_mode_stays_the_default():
    # Existing callers (executor, planner) rely on omissions filling from
    # DEFAULTS; that contract must not change under them.
    spec = parse_shot_spec("{}")
    assert spec["shot"] == DEFAULTS["shot"]
    assert spec["clamped"] == []


# --------------------------------------------------------------- coercion

def test_numeric_strings_are_coerced():
    spec = parse_shot_spec('{"radius": "4.5", "duration_s": "12"}')
    assert spec["radius"] == 4.5
    assert isinstance(spec["radius"], float)
    assert spec["duration_s"] == 12.0


def test_non_numeric_value_in_a_numeric_field_is_an_error():
    with pytest.raises(ShotSpecError):
        parse_shot_spec('{"radius": "wide"}')


@pytest.mark.parametrize("raw", [
    '{"radius": "nan"}',
    '{"radius": "inf"}',
    '{"radius": "-inf"}',
    '{"radius": NaN}',        # Python's JSON decoder accepts the literal
    '{"height": Infinity}',
    '{"speed": -Infinity}',
])
def test_non_finite_numbers_are_refused_not_flown(raw):
    # float("nan") parses, NaN compares false against every clamp bound
    # and every feasibility limit, and before this guard a NaN radius was
    # ACCEPTED end to end and produced NaN setpoints. Non-finite input is
    # not a number the aircraft can fly; refuse it at parse.
    with pytest.raises(ShotSpecError):
        parse_shot_spec(raw)


def test_booleans_accept_the_usual_string_spellings():
    assert parse_shot_spec('{"clockwise": "false"}')["clockwise"] is False
    assert parse_shot_spec('{"clockwise": "True"}')["clockwise"] is True
    assert parse_shot_spec('{"clockwise": 0}')["clockwise"] is False


# -------------------------------------------------------------- enumerations

def test_unknown_shot_is_rejected_rather_than_defaulted():
    # Defaulting an unrecognised shot to "orbit" would fly a manoeuvre the
    # operator never asked for, which is worse than refusing to fly.
    with pytest.raises(ShotSpecError):
        parse_shot_spec('{"shot": "barrel_roll"}')


def test_shot_names_are_case_insensitive_and_normalised():
    assert parse_shot_spec('{"shot": "ORBIT"}')["shot"] == "orbit"
    assert parse_shot_spec('{"shot": " Dolly_In "}')["shot"] == "dolly_in"


def test_unknown_look_at_is_rejected():
    with pytest.raises(ShotSpecError):
        parse_shot_spec('{"look_at": "the horizon"}')


# ----------------------------------------------------------------- clamping

def test_out_of_range_values_are_clamped_to_the_limit():
    spec = parse_shot_spec('{"radius": 500.0}')
    assert spec["radius"] == LIMITS["radius"][1]


def test_negative_radius_is_clamped_up_not_flown():
    spec = parse_shot_spec('{"radius": -5.0}')
    assert spec["radius"] == LIMITS["radius"][0]


def test_clamping_is_reported_so_the_caller_can_log_it():
    spec = parse_shot_spec('{"radius": 500.0, "height": -2.0}')
    assert set(spec["clamped"]) == {"radius", "height"}


def test_nothing_clamped_reports_an_empty_list():
    assert parse_shot_spec('{"radius": 4.0}')["clamped"] == []


def test_every_limited_field_is_actually_enforced():
    for field, (lo, hi) in LIMITS.items():
        below = parse_shot_spec('{"%s": %r}' % (field, lo - 1000))
        above = parse_shot_spec('{"%s": %r}' % (field, hi + 1000))
        assert below[field] == lo, f"{field} not clamped at its floor"
        assert above[field] == hi, f"{field} not clamped at its ceiling"


# ------------------------------------------------------------------- output

def test_unknown_keys_are_dropped_not_forwarded():
    # The executor reads this dict directly; an unexpected key from a
    # hallucinating model must not reach it.
    spec = parse_shot_spec('{"shot": "orbit", "afterburner": true}')
    assert "afterburner" not in spec


def test_result_contains_exactly_the_known_fields_plus_clamped():
    spec = parse_shot_spec("{}")
    assert set(spec) == set(DEFAULTS) | {"clamped"}
