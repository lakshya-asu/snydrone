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
                         boolean), or a required field omitted outright,
                         so the request is not concrete enough to execute
                         and is refused at strict parse
                         (require_complete=True).

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

Sampling honesty, resolved: raw finite differencing under-reads the true
continuous centripetal value on tight turns (radius 1 m at 2.5 m/s is
truly 6.25 m/s^2 but the raw 2 Hz chords read 5.85). The checker now
applies the chord correction (dphi/2)/sin(dphi/2), which recovers the
true value exactly on a constant arc, so the gate reads the physics, not
the sampling artifact, and boundary cases can sit at the true envelope.

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
         swept=False, mixed=False, rationale=""):
    """Assemble one labelled corpus entry.

    target is the subject pose (the orbit centre). env is a set of
    feasibility limit overrides describing the scene, most often a
    keep-out cylinder. swept marks a keep-out case whose waypoints are all
    individually clear. mixed marks a case that deliberately trips both
    the dynamic and the geometric family at once (labelled geometric,
    because that is the harness tie-break). rationale is the one-line
    justification tying the case to a named bound.
    """
    return {
        "id": cid,
        "label": label,
        "spec": spec,
        "target": tuple(float(v) for v in target),
        "env": dict(env or {}),
        "swept": swept,
        "mixed": mixed,
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
    # Boundary lane (2026-08-20 growth): cases that sit exactly at, or a
    # hair inside, an envelope bound, so an off-by-strictness regression
    # in any gate flips one of these first.
    case("nom-25", NOMINAL, orbit(radius=1.5, height=4.0, speed=3.0,
                                  duration_s=6.0),
         rationale="double at-limit: yaw rate exactly 2.0 rad/s AND "
                   "centripetal exactly 6.0 m/s^2, both pass at-limit"),
    case("nom-26", NOMINAL, orbit(radius=1.55, height=4.0, speed=3.0,
                                  duration_s=6.0, look_at="none"),
         rationale="centripetal 5.81 m/s^2, just inside the 6.0 envelope"),
    case("nom-27", NOMINAL, orbit(radius=1.6, height=4.0, speed=3.0,
                                  duration_s=6.0),
         rationale="yaw rate 1.875 rad/s and centripetal 5.63 m/s^2, both "
                   "just inside their envelopes"),
    case("nom-28", NOMINAL, orbit(radius=6.0, height=4.0, speed=1.0),
         target=(94.0, 0.0, 1.0),
         rationale="geofence boundary: the orbit reaches exactly 100 m "
                   "from origin, at the fence and at-limit passes"),
    case("nom-29", NOMINAL, orbit(radius=5.0, height=30.0, speed=1.0),
         target=(0.0, 0.0, 90.0),
         rationale="altitude boundary: exactly 120 m, at the ceiling"),
    case("nom-30", NOMINAL, orbit(radius=5.0, height=0.5, speed=1.0),
         target=(0.0, 0.0, 0.0),
         rationale="altitude boundary: exactly 0.5 m, at the floor"),
    case("nom-31", NOMINAL, orbit(radius=5.0, height=4.0, speed=1.8,
                                  duration_s=8.0),
         target=(0.0, 0.0, 1.0), env=_keepout((0.0, 0.0), 4.97),
         rationale="swept-path boundary, clear side: at 2 Hz the chord "
                   "sags to 4.9798 m from the centre, 1 cm outside the "
                   "4.97 m keep-out"),
    case("nom-32", NOMINAL, orbit(radius=5.0, height=4.0, speed=1.0),
         target=(0.0, 0.0, 1.0), env=_offset_keepout((30.0, 0.0), 5.0),
         rationale="an offset keep-out 25 m clear of the whole orbit"),
    case("nom-33", NOMINAL, orbit(radius=1.6, height=4.0, speed=3.0,
                                  duration_s=6.0, clockwise=False),
         rationale="counter-clockwise near-boundary orbit, yaw 1.875 rad/s"),
    case("nom-34", NOMINAL, orbit(radius=2.0, height=4.0, speed=3.0,
                                  duration_s=6.0),
         rationale="max legal speed on a 2 m radius: yaw 1.5 rad/s and "
                   "centripetal 4.5 m/s^2, both inside"),
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
    # Boundary lane (2026-08-20 growth): a hair outside the clamp range,
    # so the clamp comparison's strictness is pinned from the outside too.
    case("ool-19", OUT_OF_LIMITS, orbit(radius=20.0000001),
         rationale="radius 1e-7 over the 20 m ceiling still clamps"),
    case("ool-20", OUT_OF_LIMITS, orbit(duration_s=300.001),
         rationale="duration 1 ms over the 300 s ceiling still clamps"),
    case("ool-21", OUT_OF_LIMITS, orbit(speed=3.01, height=0.49),
         rationale="speed and height each a hair outside, both clamp"),
    case("ool-22", OUT_OF_LIMITS, orbit(radius=1e6),
         rationale="an absurd kilometre-scale radius clamps to 20 m"),
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
         rationale="centripetal boundary: 6.43 m/s^2 true, and with the "
                   "chord correction the gate reads exactly that, over"),
    case("dyn-17", INFEASIBLE_DYNAMIC, orbit(radius=1.2, speed=2.8,
                                             height=4.0, duration_s=6.0,
                                             look_at="none"),
         rationale="fixed heading, centripetal 6.53 m/s^2 over the envelope"),
    # Boundary lane (2026-08-20 growth). dyn-18 and dyn-19 are only
    # catchable because of the chord correction: their RAW 2 Hz readings
    # (5.93 and 5.85) sit under the envelope, their true loads (6.21 and
    # 6.25) sit over it.
    case("dyn-18", INFEASIBLE_DYNAMIC, orbit(radius=1.45, speed=3.0,
                                             height=4.0, duration_s=6.0,
                                             look_at="none"),
         rationale="centripetal 6.21 m/s^2 true; raw 2 Hz chords read 5.93 "
                   "and would pass, the chord correction refuses"),
    case("dyn-19", INFEASIBLE_DYNAMIC, orbit(radius=1.0, speed=2.5,
                                             height=4.0, duration_s=6.0,
                                             look_at="none"),
         rationale="centripetal 6.25 m/s^2 true; raw 2 Hz chords read 5.85 "
                   "and would pass, the chord correction refuses"),
    case("dyn-20", INFEASIBLE_DYNAMIC, orbit(radius=1.48, speed=3.0,
                                             height=4.0, duration_s=6.0),
         rationale="just past the double boundary: yaw 2.03 rad/s and "
                   "centripetal 6.08 m/s^2, both barely over"),
    case("dyn-21", INFEASIBLE_DYNAMIC, orbit(radius=1.3, speed=3.0,
                                             height=4.0, duration_s=6.0,
                                             look_at="none"),
         rationale="fixed heading, centripetal 6.92 m/s^2 over the envelope"),
    case("dyn-22", INFEASIBLE_DYNAMIC, orbit(radius=1.0, speed=2.2,
                                             height=4.0, duration_s=6.0),
         rationale="yaw rate 2.2 rad/s over while centripetal 4.84 stays "
                   "under: the yaw gate alone must catch it"),
    case("dyn-23", INFEASIBLE_DYNAMIC, orbit(radius=1.1, speed=2.7,
                                             height=4.0, duration_s=6.0,
                                             look_at="none"),
         rationale="fixed heading, centripetal 6.63 m/s^2 over the envelope"),
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
    _swept("swp-13", 9.0, 1.7, 5.0, (0.0, 0.0, 1.0)),
    _swept("swp-14", 4.5, 1.3, 3.5, (-10.0, 20.0, 2.0)),
]

# ---------------------------------------------------------------------------
# Swept-path BOUNDARY (2026-08-20 growth): the keep-out radius is set just
# above the chord's closest approach R*cos(dphi/2) but below the waypoint
# distance R, so the intrusion is millimetres. Every waypoint is still
# individually clear; only the segment check can see the crossing. Paired
# with nom-31, the same orbit against a keep-out 1 cm smaller, which is
# clear. These pin the swept-path check at its decision boundary.
# ---------------------------------------------------------------------------
def _swept_boundary(cid, radius, speed, height, keep_out_radius,
                    target=(0.0, 0.0, 1.0)):
    tgt = tuple(float(v) for v in target)
    return case(
        cid, INFEASIBLE_GEOMETRIC,
        orbit(radius=radius, height=height, speed=speed, duration_s=8.0),
        target=tgt, env=_keepout(tgt, keep_out_radius), swept=True,
        rationale=("boundary swept case: waypoints at %.2f m, chord sags "
                   "millimetres inside the %.3f m keep-out"
                   % (radius, keep_out_radius)),
    )


_SWEPT_BOUNDARY = [
    # R=5, v=1.8 at 2 Hz: chord closest approach 4.9798 m; 4.99 catches.
    _swept_boundary("swb-01", 5.0, 1.8, 4.0, 4.99),
    # R=8, v=1.6: dphi=0.1, closest approach 7.9900 m; 7.995 catches.
    _swept_boundary("swb-02", 8.0, 1.6, 5.0, 7.995),
    # R=3, v=1.4: dphi=0.2333, closest approach 2.9796 m; 2.99 catches.
    _swept_boundary("swb-03", 3.0, 1.4, 3.0, 2.99),
]

# ---------------------------------------------------------------------------
# MIXED (2026-08-20 growth): cases that trip a dynamic AND a geometric
# bound at once. Labelled INFEASIBLE_GEOMETRIC because the geometric
# bounds are the hard safety ones and that is the harness tie-break; the
# harness also surfaces every such case in mixed_family_cases so the
# tie-break is exercised, not hidden. Until these, that code path had no
# coverage.
# ---------------------------------------------------------------------------
_MIXED = [
    case("mix-01", INFEASIBLE_GEOMETRIC,
         orbit(radius=1.2, height=4.0, speed=2.8, duration_s=6.0),
         target=(99.5, 0.0, 1.0), mixed=True,
         rationale="tight fast orbit past the fence: yaw 2.33 rad/s over "
                   "AND the orbit reaches 100.7 m from origin"),
    case("mix-02", INFEASIBLE_GEOMETRIC,
         orbit(radius=1.0, height=4.0, speed=2.5, duration_s=6.0),
         target=(0.0, 0.0, 118.0), mixed=True,
         rationale="tight orbit over the ceiling: yaw 2.5 rad/s over AND "
                   "altitude 122 m over the 120 m ceiling"),
    case("mix-03", INFEASIBLE_GEOMETRIC,
         orbit(radius=1.3, height=4.0, speed=2.9, duration_s=6.0),
         target=(0.0, 0.0, 1.0), env=_offset_keepout((1.3, 0.0), 0.8),
         mixed=True,
         rationale="tight orbit through a keep-out: yaw 2.23 rad/s over "
                   "AND waypoints inside the 0.8 m cylinder on the ring"),
    case("mix-04", INFEASIBLE_GEOMETRIC,
         orbit(radius=1.4, height=4.0, speed=3.0, duration_s=6.0,
               look_at="none"),
         target=(99.0, 0.0, 1.0), mixed=True,
         rationale="fixed heading: centripetal 6.43 m/s^2 over AND the "
                   "orbit reaches 100.4 m from origin"),
    case("mix-05", INFEASIBLE_GEOMETRIC,
         orbit(radius=1.0, height=0.5, speed=2.5, duration_s=6.0),
         target=(0.0, 0.0, -0.3), mixed=True,
         rationale="tight orbit below the floor: yaw 2.5 rad/s over AND "
                   "altitude 0.2 m under the 0.5 m floor"),
]

# ---------------------------------------------------------------------------
# AMBIGUOUS: underspecified in one of two ways. Either a field carries a
# value the parser cannot resolve to a definite meaning (a non-canonical
# shot name, a relative word instead of a magnitude, an uninterpretable
# boolean), or a required field is omitted outright. Both are refused at
# parse under strict mode (require_complete=True), which is how the
# evaluation harness runs the corpus; the lenient default would silently
# fill omissions from DEFAULTS and make them undetectable. The bad-value
# cases are complete seven-field specs so the unresolvable value, not an
# incidental omission, is what trips.
# ---------------------------------------------------------------------------
def _bad(field, value):
    """A complete, otherwise-valid orbit spec with one unresolvable field."""
    spec = orbit(radius=5.0, height=4.0, speed=1.0, duration_s=10.0)
    spec[field] = value
    return spec


_AMBIGUOUS = [
    case("amb-01", AMBIGUOUS, _bad("shot", "circle"),
         rationale="'circle' is not a canonical shot name, cannot resolve"),
    case("amb-02", AMBIGUOUS, _bad("shot", "spiral"),
         rationale="'spiral' is not in the shot vocabulary"),
    case("amb-03", AMBIGUOUS, _bad("shot", "flyby"),
         rationale="'flyby' is not an implemented shot type"),
    case("amb-04", AMBIGUOUS, _bad("shot", "cinematic"),
         rationale="'cinematic' names a mood, not a shot"),
    case("amb-05", AMBIGUOUS, _bad("shot", ""),
         rationale="empty shot string, nothing to execute"),
    case("amb-06", AMBIGUOUS, _bad("radius", "close"),
         rationale="'close' radius is a relative word, not a magnitude"),
    case("amb-07", AMBIGUOUS, _bad("radius", "wide"),
         rationale="'wide' radius cannot be parsed to metres"),
    case("amb-08", AMBIGUOUS, _bad("speed", "fast"),
         rationale="'fast' speed is unit-free and unresolvable"),
    case("amb-09", AMBIGUOUS, _bad("speed", "slow"),
         rationale="'slow' speed cannot be parsed to a number"),
    case("amb-10", AMBIGUOUS, _bad("height", "high"),
         rationale="'high' height is a relative word, not metres"),
    case("amb-11", AMBIGUOUS, _bad("clockwise", "maybe"),
         rationale="'maybe' is neither true nor false"),
    case("amb-12", AMBIGUOUS, _bad("clockwise", "clockwise"),
         rationale="'clockwise' is not a boolean the coercer accepts"),
    case("amb-13", AMBIGUOUS, _bad("look_at", "subject"),
         rationale="'subject' is not a valid look_at (target or none)"),
    case("amb-14", AMBIGUOUS, _bad("look_at", "car"),
         rationale="'car' names an object, not a valid look_at mode"),
    case("amb-15", AMBIGUOUS, _bad("duration_s", "long"),
         rationale="'long' duration cannot be parsed to seconds"),
    case("amb-16", AMBIGUOUS, _bad("speed", "medium"),
         rationale="'medium' speed is unresolvable"),
    # Underspecification by omission: fields are simply missing. Only the
    # strict parse path can see these; the lenient default would fill them
    # from DEFAULTS and fly a shot the user never fully specified.
    case("amb-17", AMBIGUOUS, {"shot": "orbit"},
         rationale="only the shot type given, all six parameters omitted"),
    case("amb-18", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "height": 4.0},
         rationale="speed, duration, direction, and look_at all omitted"),
    case("amb-19", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "height": 4.0, "speed": 1.0,
          "duration_s": 10.0, "clockwise": True},
         rationale="a single omission: look_at is missing"),
    case("amb-20", AMBIGUOUS, {"shot": "orbit", "duration_s": 12.0},
         rationale="a duration with no geometry: radius and height omitted"),
    case("amb-21", AMBIGUOUS, {"radius": 6.0, "speed": 1.0},
         rationale="no shot type at all, nothing says what to fly"),
    # Omission lane growth (2026-08-20): every single-field omission is
    # now covered, plus the empty object.
    case("amb-22", AMBIGUOUS, {"shot": "orbit", "look_at": "target"},
         rationale="shot and framing given, all five numbers omitted"),
    case("amb-23", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "height": 4.0, "speed": 1.0,
          "duration_s": 10.0, "look_at": "target"},
         rationale="a single omission: clockwise is missing"),
    case("amb-24", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "height": 4.0, "speed": 1.0,
          "clockwise": True, "look_at": "target"},
         rationale="a single omission: duration_s is missing"),
    case("amb-25", AMBIGUOUS,
         {"shot": "orbit", "radius": 5.0, "speed": 1.0,
          "duration_s": 10.0, "clockwise": True, "look_at": "target"},
         rationale="a single omission: height is missing"),
    case("amb-26", AMBIGUOUS,
         {"shot": "orbit", "height": 4.0, "speed": 1.0,
          "duration_s": 10.0, "clockwise": True, "look_at": "target"},
         rationale="a single omission: radius is missing"),
    case("amb-27", AMBIGUOUS, {},
         rationale="the empty object: everything omitted at once"),
    # Non-finite and wrong-type lane (2026-08-20): values float() or the
    # JSON decoder happily produce but no aircraft can fly. The "nan"
    # cases found a real hole: before the parser's finite check, a NaN
    # radius passed every clamp and every feasibility limit (NaN compares
    # false) and was ACCEPTED end to end.
    case("amb-28", AMBIGUOUS, _bad("radius", "nan"),
         rationale="the string 'nan' parses to NaN, which sails through "
                   "every comparison; must be refused, not flown"),
    case("amb-29", AMBIGUOUS, _bad("speed", "inf"),
         rationale="the string 'inf' parses to infinity, not a speed"),
    case("amb-30", AMBIGUOUS, _bad("radius", float("nan")),
         rationale="a literal NaN via JSON's NaN extension, refused"),
    case("amb-31", AMBIGUOUS, _bad("height", float("inf")),
         rationale="a literal Infinity height, refused"),
    case("amb-32", AMBIGUOUS, _bad("speed", float("-inf")),
         rationale="a literal -Infinity speed, refused"),
    case("amb-33", AMBIGUOUS, _bad("radius", "4 m"),
         rationale="a number with a unit suffix does not parse; the "
                   "planner emits bare numbers"),
    case("amb-34", AMBIGUOUS, _bad("speed", "1,5"),
         rationale="a European decimal comma does not parse to a number"),
    case("amb-35", AMBIGUOUS, _bad("radius", True),
         rationale="a boolean is not a radius even though bool is an int"),
    case("amb-36", AMBIGUOUS, _bad("duration_s", [10]),
         rationale="a list is not a duration"),
    case("amb-37", AMBIGUOUS, _bad("look_at", 3),
         rationale="look_at must be a string, not a number"),
    case("amb-38", AMBIGUOUS, _bad("shot", 42),
         rationale="shot must be a string, not a number"),
    case("amb-39", AMBIGUOUS, _bad("clockwise", 2),
         rationale="only 0 and 1 coerce to a boolean; 2 is undecidable"),
    case("amb-40", AMBIGUOUS, _bad("speed", ""),
         rationale="an empty string is not a number"),
    case("amb-41", AMBIGUOUS, _bad("radius", None),
         rationale="JSON null carries no magnitude to fly"),
]

CORPUS = (
    _NOMINAL
    + _OUT_OF_LIMITS
    + _DYNAMIC
    + _GEOMETRIC_PLAIN
    + _SWEPT
    + _SWEPT_BOUNDARY
    + _MIXED
    + _AMBIGUOUS
)


def by_label(label):
    """Return every case carrying the given ground-truth label."""
    return [c for c in CORPUS if c["label"] == label]


def swept_cases():
    """Return the swept-path keep-out cases."""
    return [c for c in CORPUS if c["swept"]]


def mixed_cases():
    """Return the cases that trip both violation families at once."""
    return [c for c in CORPUS if c["mixed"]]
