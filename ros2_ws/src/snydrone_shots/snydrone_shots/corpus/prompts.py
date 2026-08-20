"""A labelled corpus of structured shot requests.

Each entry is a shot spec of the kind the planner emits (a dict of raw
fields, not natural language), tagged with a ground-truth label in a
small closed set. The labels partition the space by which gate should
reject the request, or by whether it is admissible at all:

    NOMINAL              valid and feasible: passes parse without a clamp
                         and passes the whole feasibility envelope.
    OUT_OF_LIMITS        a well-formed value outside shot_spec.LIMITS. The
                         parser understands it and clamps it into range.
    INFEASIBLE_DYNAMIC   passes limits, then violates the dynamic envelope
                         in feasibility.py (speed, acceleration, yaw rate).
    INFEASIBLE_GEOMETRIC passes limits, then violates the geometric
                         envelope (altitude band, geofence, keep-out),
                         including SWEPT-PATH cases where every waypoint is
                         individually clear but a segment between two of
                         them crosses the keep-out cylinder.
    AMBIGUOUS            underspecified: a value the parser cannot resolve
                         to a definite meaning (a non-canonical shot name,
                         a non-numeric magnitude, an uninterpretable
                         boolean), so the request is not concrete enough
                         to execute and is refused at parse.

No physics numbers are invented here. Every threshold is read from
shot_spec.LIMITS (the clamp layer) and feasibility.DEFAULT_LIMITS (the
flight envelope), and each case is derived from one of those bounds.

The dynamic cases are tight, fast orbits and trip one of two envelope
quantities. With subject tracking (look_at="target") the yaw rate
speed / radius exceeds the vehicle limit. With a fixed heading
(look_at="none") there is no yaw at all, and the violated quantity is
the centripetal acceleration speed^2 / radius, the lateral load a
constant-speed turn demands, measured by the checker from the direction
change of the sampled velocity. The tangential speed of an orbit equals
the spec speed, which the clamp caps at 3.0 m/s, well under the 8.0 m/s
envelope, and a constant-speed orbit has no tangential acceleration, so
neither of those can ever be the violated quantity.

One sampling honesty note: the gate measures centripetal load from the
2.0 Hz waypoint plan, and finite differencing under-reads the true
continuous value on very tight turns (radius 1 m at 2.5 m/s is truly
6.25 m/s^2 but reads 5.55 at this rate and passes). Corpus labels state
what the gate decides at CORPUS_GATE_HZ, so boundary cases are chosen
where the sampled reading and the true value agree on which side of the
envelope they fall.

The swept-path cases command the orbit radius to equal the required
standoff to the subject. Every waypoint then sits exactly on the standoff
boundary, which a per-waypoint distance check passes, but the straight
motion between waypoints cuts the corner and enters the standoff cylinder.
This is the class the swept-path check catches and a per-waypoint check
provably cannot.
"""

# Sampling rate the harness renders each orbit into a waypoint plan at.
# This is the granularity a per-waypoint validator receives, and the
# granularity the swept-path check reasons over. Kept an exact divisor of
# every case duration so the final segment is never short (a short final
# segment would drop the chord speed and read as a spurious deceleration).
CORPUS_GATE_HZ = 2.0

NOMINAL = "NOMINAL"
OUT_OF_LIMITS = "OUT_OF_LIMITS"
INFEASIBLE_DYNAMIC = "INFEASIBLE_DYNAMIC"
INFEASIBLE_GEOMETRIC = "INFEASIBLE_GEOMETRIC"
AMBIGUOUS = "AMBIGUOUS"

# Exhaustive and mutually exclusive.
LABELS = (
    NOMINAL,
    OUT_OF_LIMITS,
    INFEASIBLE_DYNAMIC,
    INFEASIBLE_GEOMETRIC,
    AMBIGUOUS,
)


def orbit(radius=3.0, height=3.5, speed=0.6, duration_s=10.0,
          clockwise=True, look_at="target"):
    """Build an orbit spec dict as the planner would emit it."""
    return {
        "shot": "orbit",
        "radius": radius,
        "height": height,
        "speed": speed,
        "duration_s": duration_s,
        "clockwise": clockwise,
        "look_at": look_at,
    }


def case(cid, label, spec, target=(0.0, 0.0, 1.0), env=None,
         swept=False, rationale=""):
    """Assemble one labelled corpus entry.

    target is the subject pose (the orbit centre). env is a set of
    feasibility limit overrides describing the scene, most often a
    keep-out cylinder. swept marks a keep-out case whose waypoints are all
    individually clear. rationale is the one-line justification tying the
    case to a named bound.
    """
    return {
        "id": cid,
        "label": label,
        "spec": spec,
        "target": tuple(float(v) for v in target),
        "env": dict(env or {}),
        "swept": swept,
        "rationale": rationale,
    }


def _keepout(target, radius_m):
    """A keep-out cylinder centred on the subject at the given radius."""
    return {
        "keep_out_centre": (float(target[0]), float(target[1])),
        "keep_out_radius_m": float(radius_m),
    }


def _offset_keepout(centre_xy, radius_m):
    """A keep-out cylinder centred away from the subject."""
    return {
        "keep_out_centre": (float(centre_xy[0]), float(centre_xy[1])),
        "keep_out_radius_m": float(radius_m),
    }


# ---------------------------------------------------------------------------
# NOMINAL: inside both limit sets and inside the whole flight envelope.
# radius in [1, 20], height in [0.5, 30], speed in [0, 3], all with the
# subject-tracking yaw rate (speed / radius) under the 2.0 rad/s envelope.
# ---------------------------------------------------------------------------
_NOMINAL = [
    case("nom-01", NOMINAL, orbit(radius=3.0, height=3.5, speed=0.6),
         rationale="the DEFAULTS orbit, comfortably inside every bound"),
    case("nom-02", NOMINAL, orbit(radius=5.0, height=4.0, speed=1.0),
         rationale="mid-range radius and speed, yaw rate 0.20 rad/s"),
    case("nom-03", NOMINAL, orbit(radius=8.0, height=5.0, speed=1.5),
         rationale="wide slow orbit, yaw rate 0.19 rad/s"),
    case("nom-04", NOMINAL, orbit(radius=4.0, height=3.0, speed=0.8),
         rationale="tight-ish but slow, well within dynamics"),
    case("nom-05", NOMINAL, orbit(radius=10.0, height=6.0, speed=1.9),
         rationale="near-max speed on a wide radius stays feasible"),
    case("nom-06", NOMINAL, orbit(radius=2.0, height=2.0, speed=0.5),
         rationale="small orbit near the radius floor, slow"),
    case("nom-07", NOMINAL, orbit(radius=6.0, height=4.5, speed=1.2),
         rationale="typical cinematic orbit"),
    case("nom-08", NOMINAL, orbit(radius=15.0, height=10.0, speed=2.0),
         rationale="large orbit, yaw rate 0.13 rad/s"),
    case("nom-09", NOMINAL, orbit(radius=1.0, height=1.0, speed=0.4),
         rationale="radius exactly at the 1.0 m floor, slow enough"),
    case("nom-10", NOMINAL, orbit(radius=20.0, height=30.0, speed=3.0),
         rationale="radius, height, speed all exactly at their ceilings"),
    case("nom-11", NOMINAL, orbit(radius=7.0, height=0.5, speed=1.0),
         rationale="height at the 0.5 m floor, subject on the ground"),
    case("nom-12", NOMINAL, orbit(radius=5.0, height=4.0, speed=1.0,
                                  clockwise=False),
         rationale="counter-clockwise, direction must not affect the gates"),
    case("nom-13", NOMINAL, orbit(radius=9.0, height=6.0, speed=1.4,
                                  look_at="none"),
         rationale="fixed heading, no subject tracking, still feasible"),
    case("nom-14", NOMINAL, orbit(radius=12.0, height=8.0, speed=2.2),
         rationale="fast on a wide radius, yaw rate 0.18 rad/s"),
    case("nom-15", NOMINAL, orbit(radius=3.0, height=3.5, speed=0.6,
                                  duration_s=0.5),
         rationale="duration at the 0.5 s floor"),
    case("nom-16", NOMINAL, orbit(radius=4.0, height=3.0, speed=0.9,
                                  duration_s=60.0),
         rationale="a long minute-long orbit, still inside the envelope"),
    case("nom-17", NOMINAL, orbit(radius=18.0, height=25.0, speed=2.8),
         rationale="near the top of every range at once"),
    case("nom-18", NOMINAL, orbit(radius=2.5, height=2.5, speed=0.7),
         rationale="small subject orbit"),
    case("nom-19", NOMINAL, orbit(radius=6.0, height=12.0, speed=1.1),
         target=(20.0, -15.0, 2.0),
         rationale="subject offset from origin but orbit inside the geofence"),
    case("nom-20", NOMINAL, orbit(radius=5.0, height=4.0, speed=1.0),
         target=(0.0, 0.0, 1.0), env=_keepout((0.0, 0.0), 2.0),
         rationale="a 2 m keep-out well inside a 5 m orbit, never touched"),
    case("nom-21", NOMINAL, orbit(radius=8.0, height=5.0, speed=1.5),
         target=(30.0, 40.0, 3.0),
         rationale="subject 50 m out, orbit reaches 58 m, inside 100 m fence"),
    case("nom-22", NOMINAL, orbit(radius=11.0, height=7.0, speed=2.0,
                                  clockwise=False, look_at="none"),
         rationale="fast wide fixed-heading orbit, all bounds satisfied"),
    case("nom-23", NOMINAL, orbit(radius=2.0, height=4.0, speed=3.0,
                                  duration_s=6.0, look_at="none"),
         rationale="fixed-heading control for the centripetal check: "
                   "speed^2/radius is 4.5 m/s^2, under the 6.0 envelope"),
    case("nom-24", NOMINAL, orbit(radius=1.5, height=4.0, speed=3.0,
                                  duration_s=6.0, look_at="none"),
         rationale="centripetal boundary: speed^2/radius exactly 6.0 m/s^2, "
                   "at the envelope and at-limit passes"),
]

# ---------------------------------------------------------------------------
# OUT_OF_LIMITS: a well-formed number outside shot_spec.LIMITS. The parser
# clamps it into range and reports the clamp; the request as stated is
# refused and the safe clamped value is what would fly.
# ---------------------------------------------------------------------------
_OUT_OF_LIMITS = [
    case("ool-01", OUT_OF_LIMITS, orbit(radius=50.0),
         rationale="radius 50 m over the 20 m ceiling, clamps to 20"),
    case("ool-02", OUT_OF_LIMITS, orbit(radius=0.3),
         rationale="radius 0.3 m under the 1.0 m floor, clamps to 1.0"),
    case("ool-03", OUT_OF_LIMITS, orbit(radius=100.0, speed=1.0),
         rationale="radius 100 m far over ceiling, clamps to 20"),
    case("ool-04", OUT_OF_LIMITS, orbit(height=45.0),
         rationale="height 45 m over the 30 m ceiling, clamps to 30"),
    case("ool-05", OUT_OF_LIMITS, orbit(height=0.1),
         rationale="height 0.1 m under the 0.5 m floor, clamps to 0.5"),
    case("ool-06", OUT_OF_LIMITS, orbit(speed=10.0, radius=8.0),
         rationale="speed 10 m/s over the 3.0 m/s ceiling, clamps to 3.0"),
    case("ool-07", OUT_OF_LIMITS, orbit(speed=-2.0),
         rationale="negative speed under the 0.0 floor, clamps to 0.0"),
    case("ool-08", OUT_OF_LIMITS, orbit(duration_s=500.0),
         rationale="duration 500 s over the 300 s ceiling, clamps to 300"),
    case("ool-09", OUT_OF_LIMITS, orbit(duration_s=0.1),
         rationale="duration 0.1 s under the 0.5 s floor, clamps to 0.5"),
    case("ool-10", OUT_OF_LIMITS, orbit(radius=21.0),
         rationale="radius 1 m over the ceiling, clamps to 20"),
    case("ool-11", OUT_OF_LIMITS, orbit(radius=0.99),
         rationale="radius just under the 1.0 m floor, clamps to 1.0"),
    case("ool-12", OUT_OF_LIMITS, orbit(speed=3.0001, radius=10.0),
         rationale="speed a hair over the ceiling, clamps to 3.0"),
    case("ool-13", OUT_OF_LIMITS, orbit(radius=40.0, height=60.0),
         rationale="radius and height both over ceiling, both clamp"),
    case("ool-14", OUT_OF_LIMITS, orbit(radius=200.0, speed=25.0,
                                        height=90.0),
         rationale="an 'ignore your limits' spec, three fields clamp"),
    case("ool-15", OUT_OF_LIMITS, orbit(height=1000.0, radius=5.0),
         rationale="height 1 km, clamps to 30"),
    case("ool-16", OUT_OF_LIMITS, orbit(radius=-5.0),
         rationale="negative radius under the floor, clamps to 1.0"),
    case("ool-17", OUT_OF_LIMITS, orbit(speed=5.0, radius=0.5),
         rationale="over-speed and under-radius, both clamp into range"),
    case("ool-18", OUT_OF_LIMITS, orbit(duration_s=3600.0, radius=12.0),
         rationale="an hour-long duration, clamps to 300 s"),
]

# ---------------------------------------------------------------------------
# INFEASIBLE_DYNAMIC: inside shot_spec.LIMITS (no clamp), then a dynamic
# envelope quantity is exceeded. For look_at="target" cases it is the
# subject-tracking yaw rate speed / radius over the 2.0 rad/s envelope.
# For look_at="none" cases there is no yaw motion at all and the violated
# quantity is the centripetal acceleration speed^2 / radius over the
# 6.0 m/s^2 envelope. radius stays >= 1.0 and speed <= 3.0 so nothing
# clamps; radius is small and the subject sits at the origin so no
# geometric bound is touched.
# ---------------------------------------------------------------------------
_DYNAMIC = [
    case("dyn-01", INFEASIBLE_DYNAMIC, orbit(radius=1.0, speed=2.5,
                                             height=4.0, duration_s=6.0),
         rationale="yaw rate 2.5 rad/s over the 2.0 rad/s envelope"),
    case("dyn-02", INFEASIBLE_DYNAMIC, orbit(radius=1.0, speed=2.1,
                                             height=4.0, duration_s=6.0),
         rationale="yaw rate 2.1 rad/s, just over the envelope"),
    case("dyn-03", INFEASIBLE_DYNAMIC, orbit(radius=1.0, speed=3.0,
                                             height=4.0, duration_s=6.0),
         rationale="tightest fastest legal orbit, yaw rate 3.0 rad/s"),
    case("dyn-04", INFEASIBLE_DYNAMIC, orbit(radius=1.2, speed=2.9,
                                             height=5.0, duration_s=6.0),
         rationale="yaw rate 2.42 rad/s over the envelope"),
    case("dyn-05", INFEASIBLE_DYNAMIC, orbit(radius=1.4, speed=3.0,
                                             height=5.0, duration_s=8.0),
         rationale="yaw rate 2.14 rad/s over the envelope"),
    case("dyn-06", INFEASIBLE_DYNAMIC, orbit(radius=1.1, speed=2.6,
                                             height=3.0, duration_s=6.0),
         rationale="yaw rate 2.36 rad/s over the envelope"),
    case("dyn-07", INFEASIBLE_DYNAMIC, orbit(radius=1.3, speed=2.9,
                                             height=4.0, duration_s=6.0),
         rationale="yaw rate 2.23 rad/s over the envelope"),
    case("dyn-08", INFEASIBLE_DYNAMIC, orbit(radius=1.45, speed=3.0,
                                             height=6.0, duration_s=8.0),
         rationale="yaw rate 2.07 rad/s, near the boundary but over"),
    case("dyn-09", INFEASIBLE_DYNAMIC, orbit(radius=1.0, speed=2.05,
                                             height=2.5, duration_s=6.0),
         rationale="yaw rate 2.05 rad/s, the smallest over-limit margin"),
    case("dyn-10", INFEASIBLE_DYNAMIC, orbit(radius=1.2, speed=2.8,
                                             height=8.0, duration_s=10.0),
         rationale="yaw rate 2.33 rad/s over the envelope"),
    case("dyn-11", INFEASIBLE_DYNAMIC, orbit(radius=1.0, speed=2.5,
                                             height=4.0, duration_s=6.0,
                                             clockwise=False),
         rationale="counter-clockwise tight orbit, yaw rate still 2.5 rad/s"),
    case("dyn-12", INFEASIBLE_DYNAMIC, orbit(radius=1.35, speed=2.95,
                                             height=5.0, duration_s=8.0),
         rationale="yaw rate 2.19 rad/s over the envelope"),
    case("dyn-13", INFEASIBLE_DYNAMIC, orbit(radius=1.25, speed=2.7,
                                             height=3.5, duration_s=6.0),
         rationale="yaw rate 2.16 rad/s over the envelope"),
    case("dyn-14", INFEASIBLE_DYNAMIC, orbit(radius=1.15, speed=2.4,
                                             height=4.0, duration_s=6.0),
         rationale="yaw rate 2.09 rad/s over the envelope"),
    case("dyn-15", INFEASIBLE_DYNAMIC, orbit(radius=1.0, speed=3.0,
                                             height=4.0, duration_s=6.0,
                                             look_at="none"),
         rationale="the verifier-gap case: fixed heading, no yaw motion, "
                   "centripetal 9.0 m/s^2 over the 6.0 envelope"),
    case("dyn-16", INFEASIBLE_DYNAMIC, orbit(radius=1.4, speed=3.0,
                                             height=4.0, duration_s=6.0,
                                             look_at="none"),
         rationale="centripetal boundary: 6.43 m/s^2 true, 6.13 sampled, "
                   "over the envelope on both readings"),
    case("dyn-17", INFEASIBLE_DYNAMIC, orbit(radius=1.2, speed=2.8,
                                             height=4.0, duration_s=6.0,
                                             look_at="none"),
         rationale="fixed heading, centripetal 6.53 m/s^2 over the envelope"),
]

# ---------------------------------------------------------------------------
# INFEASIBLE_GEOMETRIC, non-swept: the violation is visible at the
# waypoints themselves, so a per-waypoint checker would also catch these.
# They exercise the geofence, the keep-out at waypoints, and the altitude
# band. Kept free of any dynamic violation (yaw rate under 2.0 rad/s).
# ---------------------------------------------------------------------------
_GEOMETRIC_PLAIN = [
    case("geo-01", INFEASIBLE_GEOMETRIC, orbit(radius=5.0, height=4.0,
                                               speed=1.0),
         target=(98.0, 0.0, 1.0),
         rationale="subject 98 m out, orbit reaches 103 m past the 100 m fence"),
    case("geo-02", INFEASIBLE_GEOMETRIC, orbit(radius=10.0, height=6.0,
                                               speed=1.5),
         target=(95.0, 0.0, 1.0),
         rationale="orbit swings to 105 m from origin, over the geofence"),
    case("geo-03", INFEASIBLE_GEOMETRIC, orbit(radius=8.0, height=5.0,
                                               speed=1.4, duration_s=40.0),
         target=(-70.0, 70.0, 2.0),
         rationale="subject 99 m out diagonally, a full revolution reaches "
                   "107 m past the fence"),
    case("geo-04", INFEASIBLE_GEOMETRIC, orbit(radius=6.0, height=25.0,
                                               speed=1.0),
         target=(0.0, 0.0, 100.0),
         rationale="subject on a 100 m tower, orbit at 125 m past the ceiling"),
    case("geo-05", INFEASIBLE_GEOMETRIC, orbit(radius=5.0, height=30.0,
                                               speed=1.0),
         target=(0.0, 0.0, 95.0),
         rationale="orbit at 125 m altitude over the 120 m ceiling"),
    case("geo-06", INFEASIBLE_GEOMETRIC, orbit(radius=4.0, height=0.5,
                                               speed=0.8),
         target=(0.0, 0.0, -1.0),
         rationale="subject 1 m below datum, orbit at -0.5 m under the floor"),
    case("geo-07", INFEASIBLE_GEOMETRIC, orbit(radius=5.0, height=4.0,
                                               speed=1.0),
         target=(0.0, 0.0, 1.0), env=_offset_keepout((5.0, 0.0), 3.0),
         rationale="a 3 m keep-out on the ring swallows the waypoints near it"),
    case("geo-08", INFEASIBLE_GEOMETRIC, orbit(radius=6.0, height=4.0,
                                               speed=1.2),
         target=(0.0, 0.0, 1.0), env=_offset_keepout((6.0, 0.0), 4.0),
         rationale="large keep-out overlapping the orbit ring at waypoints"),
    case("geo-09", INFEASIBLE_GEOMETRIC, orbit(radius=8.0, height=5.0,
                                               speed=1.5),
         target=(90.0, 40.0, 2.0),
         rationale="subject 98.5 m out, wide orbit clears the fence"),
    case("geo-10", INFEASIBLE_GEOMETRIC, orbit(radius=7.0, height=28.0,
                                               speed=1.2),
         target=(0.0, 0.0, 96.0),
         rationale="orbit at 124 m altitude, over the ceiling"),
]

# ---------------------------------------------------------------------------
# INFEASIBLE_GEOMETRIC, SWEPT-PATH: the orbit radius is commanded equal to
# the required standoff (the keep-out radius). Every waypoint sits exactly
# on the standoff boundary, so a per-waypoint distance check passes, but
# the straight segment between waypoints cuts inside the cylinder. This is
# the class the swept-path segment check catches and a per-waypoint check
# provably cannot. At least ten, as required.
# ---------------------------------------------------------------------------
def _swept(cid, radius, speed, height, target, duration_s=8.0,
           clockwise=True):
    """A swept-path case: keep-out radius equals the commanded orbit radius."""
    tgt = tuple(float(v) for v in target)
    return case(
        cid, INFEASIBLE_GEOMETRIC,
        orbit(radius=radius, height=height, speed=speed,
              duration_s=duration_s, clockwise=clockwise),
        target=tgt, env=_keepout(tgt, radius), swept=True,
        rationale=("orbit radius %.1f m equals the standoff; waypoints sit on "
                   "the boundary, the chord between them cuts inside"
                   % radius),
    )


_SWEPT = [
    _swept("swp-01", 5.0, 1.8, 4.0, (0.0, 0.0, 1.0)),
    _swept("swp-02", 4.0, 1.6, 3.0, (0.0, 0.0, 1.0)),
    _swept("swp-03", 6.0, 1.5, 4.5, (10.0, 10.0, 2.0)),
    _swept("swp-04", 8.0, 1.6, 5.0, (0.0, 0.0, 1.0)),
    _swept("swp-05", 3.0, 1.4, 3.0, (0.0, 0.0, 1.0)),
    _swept("swp-06", 7.0, 1.7, 4.0, (-20.0, 5.0, 2.0)),
    _swept("swp-07", 5.0, 1.2, 4.0, (0.0, 0.0, 1.0)),
    _swept("swp-08", 4.0, 1.0, 3.0, (0.0, 0.0, 1.0)),
    _swept("swp-09", 6.0, 1.8, 4.5, (0.0, 0.0, 1.0)),
    _swept("swp-10", 10.0, 1.9, 6.0, (0.0, 0.0, 1.0)),
    _swept("swp-11", 2.0, 1.2, 2.0, (0.0, 0.0, 1.0)),
    _swept("swp-12", 3.5, 1.5, 3.0, (15.0, -15.0, 2.0), clockwise=False),
]

# ---------------------------------------------------------------------------
# AMBIGUOUS: a value the parser cannot resolve to a definite meaning. The
# planner failed to produce a concrete executable field, so the request is
# refused at parse rather than guessed at.
# ---------------------------------------------------------------------------
_AMBIGUOUS = [
    case("amb-01", AMBIGUOUS,
         {"shot": "circle", "radius": 5.0, "height": 4.0, "speed": 1.0},
         rationale="'circle' is not a canonical shot name, cannot resolve"),
    case("amb-02", AMBIGUOUS,
         {"shot": "spiral", "radius": 5.0, "height": 4.0},
         rationale="'spiral' is not in the shot vocabulary"),
    case("amb-03", AMBIGUOUS,
         {"shot": "flyby", "radius": 6.0, "speed": 1.0},
         rationale="'flyby' is not an implemented shot type"),
    case("amb-04", AMBIGUOUS,
         {"shot": "cinematic", "radius": 5.0},
         rationale="'cinematic' names a mood, not a shot"),
    case("amb-05", AMBIGUOUS,
         {"shot": "", "radius": 5.0, "height": 4.0},
         rationale="empty shot string, nothing to execute"),
    case("amb-06", AMBIGUOUS,
         {"shot": "orbit", "radius": "close", "height": 4.0},
         rationale="'close' radius is a relative word, not a magnitude"),
    case("amb-07", AMBIGUOUS,
         {"shot": "orbit", "radius": "wide", "speed": 1.0},
         rationale="'wide' radius cannot be parsed to metres"),
    case("amb-08", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "speed": "fast"},
         rationale="'fast' speed is unit-free and unresolvable"),
    case("amb-09", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "speed": "slow"},
         rationale="'slow' speed cannot be parsed to a number"),
    case("amb-10", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "height": "high"},
         rationale="'high' height is a relative word, not metres"),
    case("amb-11", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "clockwise": "maybe"},
         rationale="'maybe' is neither true nor false"),
    case("amb-12", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "clockwise": "clockwise"},
         rationale="'clockwise' is not a boolean the coercer accepts"),
    case("amb-13", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "look_at": "subject"},
         rationale="'subject' is not a valid look_at (target or none)"),
    case("amb-14", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "look_at": "car"},
         rationale="'car' names an object, not a valid look_at mode"),
    case("amb-15", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "duration_s": "long"},
         rationale="'long' duration cannot be parsed to seconds"),
    case("amb-16", AMBIGUOUS,
         {"radius": 5.0, "height": 4.0, "speed": "medium", "shot": "orbit"},
         rationale="'medium' speed is unresolvable"),
]

CORPUS = (
    _NOMINAL
    + _OUT_OF_LIMITS
    + _DYNAMIC
    + _GEOMETRIC_PLAIN
    + _SWEPT
    + _AMBIGUOUS
)


def by_label(label):
    """Return every case carrying the given ground-truth label."""
    return [c for c in CORPUS if c["label"] == label]


def swept_cases():
    """Return the swept-path keep-out cases."""
    return [c for c in CORPUS if c["swept"]]
