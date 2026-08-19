"""Shot specification parsing and validation for SNYdrone."""

import json

__all__ = ["DEFAULTS", "LIMITS", "ShotSpecError", "parse_shot_spec"]


class ShotSpecError(ValueError):
    """Raised when raw LLM output cannot be turned into a safe shot spec."""


# Units: radius and height in metres, duration_s in seconds. speed is
# LINEAR metres per second along the flight path for every shot type
# (ratified 2026-08-17); the orbit geometry derives its angular rate
# from speed and radius. It is never radians per second.
DEFAULTS = {
    "shot": "orbit",
    "radius": 3.0,
    "height": 3.5,
    "speed": 0.6,
    "duration_s": 10.0,
    "clockwise": True,
    "look_at": "target",
}

LIMITS = {
    "radius": (1.0, 20.0),
    "height": (0.5, 30.0),
    "speed": (0.0, 3.0),
    "duration_s": (0.5, 300.0),
}

_NUMERIC_FIELDS = ("radius", "height", "speed", "duration_s")
_VALID_SHOTS = ("orbit", "dolly_in", "dolly_out", "follow", "pan")
_VALID_LOOK_AT = ("target", "none")


def _extract_json_object(raw):
    """Locate and parse the first JSON object embedded in raw text."""
    start = raw.find("{")
    if start == -1:
        raise ShotSpecError("No JSON object found in input")

    decoder = json.JSONDecoder()
    try:
        obj, _end = decoder.raw_decode(raw[start:])
    except (json.JSONDecodeError, ValueError) as exc:
        raise ShotSpecError("Failed to parse JSON object: %s" % exc)

    if not isinstance(obj, dict):
        raise ShotSpecError("Parsed JSON value is not an object")

    return obj


def _coerce_numeric(value, field):
    """Coerce a value to float for a numeric field, raising ShotSpecError on failure."""
    if isinstance(value, bool):
        raise ShotSpecError(
            "Field '%s': expected a number, got a boolean" % field
        )
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            raise ShotSpecError(
                "Field '%s': cannot parse %r as a number" % (field, value)
            )
    raise ShotSpecError(
        "Field '%s': expected a number, got %s" % (field, type(value).__name__)
    )


def _coerce_bool(value, field):
    """Coerce a value to bool for the clockwise field."""
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and not isinstance(value, bool):
        if value in (0, 1):
            return bool(value)
        raise ShotSpecError(
            "Field '%s': expected 0 or 1, got %d" % (field, value)
        )
    if isinstance(value, str):
        lower = value.strip().lower()
        if lower == "true":
            return True
        if lower == "false":
            return False
    raise ShotSpecError(
        "Field '%s': cannot interpret %r as a boolean" % (field, value)
    )


def parse_shot_spec(raw: str) -> dict:
    """Parse and validate raw LLM output into a safe shot specification.

    Returns a dict with exactly the seven DEFAULTS keys plus a ``clamped``
    key (a list of field names that were clamped into their LIMITS range).
    Raises ShotSpecError for any input that cannot be turned into a safe
    specification.
    """
    obj = _extract_json_object(raw)

    # Start from defaults; only overwrite with values actually present in obj.
    spec = dict(DEFAULTS)

    for key in DEFAULTS:
        if key not in obj:
            continue
        value = obj[key]
        if key in _NUMERIC_FIELDS:
            spec[key] = _coerce_numeric(value, key)
        elif key == "clockwise":
            spec[key] = _coerce_bool(value, key)
        elif key in ("shot", "look_at"):
            if not isinstance(value, str):
                raise ShotSpecError(
                    "Field '%s': expected a string, got %s"
                    % (key, type(value).__name__)
                )
            spec[key] = value.strip().lower()

    # Validate enumerated fields.
    if spec["shot"] not in _VALID_SHOTS:
        raise ShotSpecError("Invalid shot: %r" % spec["shot"])
    if spec["look_at"] not in _VALID_LOOK_AT:
        raise ShotSpecError("Invalid look_at: %r" % spec["look_at"])

    # Clamp numeric fields using LIMITS.
    clamped = []
    for field, (low, high) in LIMITS.items():
        val = spec[field]
        if val < low:
            spec[field] = low
            clamped.append(field)
        elif val > high:
            spec[field] = high
            clamped.append(field)

    spec["clamped"] = clamped
    return spec
