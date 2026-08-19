// Spec for the browser-side shot library.
//
// This is a deliberate port of the Python modules, not a new design.
// The demo has to compute exactly what the aircraft would fly, because
// a demo that draws a different path from the one the drone takes is
// worse than no demo at all. Where this file and the Python disagree,
// the Python wins and this is the bug.
//
// Reference: snydrone_brain/shot_spec.py, snydrone_shots/orbit_geometry.py,
// snydrone_shots/trajectory.py
//
// Run: node --test docs/demo/

import test from "node:test";
import assert from "node:assert/strict";

import {
  DEFAULTS,
  LIMITS,
  ShotSpecError,
  parseShotSpec,
  orbitSetpoint,
  sampleTrajectory,
} from "./shotlib.mjs";

const TARGET = [2.0, -1.0, 0.5];
const close = (a, b, eps = 1e-9) => Math.abs(a - b) <= eps;
const spec = (over = {}) => ({
  shot: "orbit", radius: 3.0, height: 2.0, speed: 0.5,
  duration_s: 4.0, clockwise: true, look_at: "target", ...over,
});

// ------------------------------------------------------------ parsing

test("unwraps fences and surrounding prose", () => {
  for (const raw of [
    '{"shot": "orbit"}',
    '```json\n{"shot": "orbit"}\n```',
    '```\n{"shot": "orbit"}\n```',
    'Sure, here you go:\n```json\n{"shot": "orbit"}\n```\n',
    '   {"shot":"orbit"}   ',
  ]) {
    assert.equal(parseShotSpec(raw).shot, "orbit");
  }
});

test("rejects unparseable or non-object input", () => {
  for (const raw of ["", "   ", "not json", "[1,2,3]", '"str"', "null", "{"]) {
    assert.throws(() => parseShotSpec(raw), ShotSpecError);
  }
});

test("fills every missing field from DEFAULTS", () => {
  const s = parseShotSpec("{}");
  for (const [k, v] of Object.entries(DEFAULTS)) assert.equal(s[k], v);
});

test("does not overwrite supplied fields", () => {
  const s = parseShotSpec('{"radius": 4.5, "clockwise": false}');
  assert.equal(s.radius, 4.5);
  assert.equal(s.clockwise, false);
  assert.equal(s.height, DEFAULTS.height);
});

test("coerces numeric strings and rejects non-numeric ones", () => {
  assert.equal(parseShotSpec('{"radius": "4.5"}').radius, 4.5);
  assert.throws(() => parseShotSpec('{"radius": "wide"}'), ShotSpecError);
});

test("coerces the usual boolean spellings", () => {
  assert.equal(parseShotSpec('{"clockwise": "false"}').clockwise, false);
  assert.equal(parseShotSpec('{"clockwise": "True"}').clockwise, true);
  assert.equal(parseShotSpec('{"clockwise": 0}').clockwise, false);
});

test("normalises enums and rejects unknown ones", () => {
  assert.equal(parseShotSpec('{"shot": "ORBIT"}').shot, "orbit");
  assert.equal(parseShotSpec('{"shot": " Dolly_In "}').shot, "dolly_in");
  // Flying an unrequested manoeuvre is worse than refusing to fly.
  assert.throws(() => parseShotSpec('{"shot": "barrel_roll"}'), ShotSpecError);
  assert.throws(() => parseShotSpec('{"look_at": "horizon"}'), ShotSpecError);
});

test("clamps to limits and reports what was clamped", () => {
  assert.equal(parseShotSpec('{"radius": 500}').radius, LIMITS.radius[1]);
  assert.equal(parseShotSpec('{"radius": -5}').radius, LIMITS.radius[0]);
  const s = parseShotSpec('{"radius": 500, "height": -2}');
  assert.deepEqual([...s.clamped].sort(), ["height", "radius"]);
  assert.deepEqual(parseShotSpec('{"radius": 4}').clamped, []);
});

test("every limited field is enforced at both ends", () => {
  for (const [field, [lo, hi]] of Object.entries(LIMITS)) {
    assert.equal(parseShotSpec(`{"${field}": ${lo - 1000}}`)[field], lo);
    assert.equal(parseShotSpec(`{"${field}": ${hi + 1000}}`)[field], hi);
  }
});

test("drops unknown keys and returns exactly the known shape", () => {
  const s = parseShotSpec('{"shot":"orbit","afterburner":true}');
  assert.equal("afterburner" in s, false);
  assert.deepEqual(
    Object.keys(s).sort(),
    [...Object.keys(DEFAULTS), "clamped"].sort(),
  );
});

// ----------------------------------------------------------- geometry

test("orbit starts on the positive x side at the given height", () => {
  const [x, y, z] = orbitSetpoint(TARGET, 0, spec());
  assert.ok(close(x, TARGET[0] + 3.0));
  assert.ok(close(y, TARGET[1]));
  assert.ok(close(z, TARGET[2] + 2.0));
});

test("orbit holds its radius", () => {
  for (let i = 0; i < 40; i++) {
    const t = i * 0.37;
    const [x, y] = orbitSetpoint(TARGET, t, spec({ radius: 4.25 }));
    assert.ok(close(Math.hypot(x - TARGET[0], y - TARGET[1]), 4.25));
  }
});

test("clockwise and counter-clockwise mirror each other", () => {
  const cw = orbitSetpoint(TARGET, 0.1, spec({ clockwise: true }));
  const ccw = orbitSetpoint(TARGET, 0.1, spec({ clockwise: false }));
  assert.ok(cw[1] < TARGET[1] && TARGET[1] < ccw[1]);
  assert.ok(close(cw[1] - TARGET[1], -(ccw[1] - TARGET[1])));
});

test("yaw points at the target, and look_at none holds zero", () => {
  for (let i = 0; i < 20; i++) {
    const t = i * 0.61;
    const [x, y, , yaw] = orbitSetpoint(TARGET, t, spec());
    const want = Math.atan2(TARGET[1] - y, TARGET[0] - x);
    assert.ok(close(Math.cos(yaw - want), 1.0));
  }
  assert.ok(close(orbitSetpoint(TARGET, 3.3, spec({ look_at: "none" }))[3], 0));
});

// Speed is LINEAR metres per second along the orbit path, ratified
// 2026-08-17 and matching orbit_geometry.py. The rad/s reading these
// tests replace made a 20 m orbit fly 6.7 times faster over the
// ground than a 3 m orbit on the same spec.

test("speed is metres per second along the path", () => {
  const v = 1.5, dt = 0.01, r = 4.0;
  const a = orbitSetpoint(TARGET, 0, spec({ speed: v, radius: r }));
  const b = orbitSetpoint(TARGET, dt, spec({ speed: v, radius: r }));
  const step = Math.hypot(b[0] - a[0], b[1] - a[1]);
  assert.ok(close(step, v * dt, v * dt * 1e-3));
});

test("the orbit period scales with the radius", () => {
  const v = 2.0;
  for (const r of [2.0, 5.0]) {
    const period = (2 * Math.PI * r) / v;
    const start = orbitSetpoint(TARGET, 0, spec({ speed: v, radius: r }));
    const later = orbitSetpoint(TARGET, period, spec({ speed: v, radius: r }));
    assert.ok(close(later[0], start[0], 1e-6) && close(later[1], start[1], 1e-6));
    const half = orbitSetpoint(TARGET, period / 2, spec({ speed: v, radius: r }));
    assert.ok(close(half[0], TARGET[0] - r, 1e-6));
  }
});

test("a nonpositive radius is rejected instead of dividing by it", () => {
  assert.throws(() => orbitSetpoint(TARGET, 1.0, spec({ radius: 0 })), RangeError);
});

test("zero speed holds position", () => {
  const a = orbitSetpoint(TARGET, 0, spec({ speed: 0 }));
  const b = orbitSetpoint(TARGET, 30, spec({ speed: 0 }));
  for (let i = 0; i < 3; i++) assert.ok(close(a[i], b[i]));
});

// --------------------------------------------------------- trajectory

test("sampling is inclusive of both endpoints", () => {
  const traj = sampleTrajectory(TARGET, spec({ duration_s: 4.0 }), 10);
  assert.equal(traj.length, 41);
  assert.ok(close(traj[0][0], 0));
  assert.ok(close(traj[traj.length - 1][0], 4.0));
});

test("timesteps do not drift", () => {
  const traj = sampleTrajectory(TARGET, spec({ duration_s: 7.0 }), 20);
  for (let i = 1; i < traj.length; i++) {
    assert.ok(close(traj[i][0] - traj[i - 1][0], 0.05));
  }
});

test("a rate that does not divide the duration still ends exactly on it", () => {
  const traj = sampleTrajectory(TARGET, spec({ duration_s: 1.0 }), 3);
  assert.ok(close(traj[traj.length - 1][0], 1.0));
  for (let i = 1; i < traj.length; i++) {
    assert.ok(traj[i][0] > traj[i - 1][0]);
  }
});

test("sampler agrees with the geometry pointwise", () => {
  const sp = spec();
  for (const [t, x, y, z, yaw] of sampleTrajectory(TARGET, sp, 7)) {
    const [ex, ey, ez, eyaw] = orbitSetpoint(TARGET, t, sp);
    assert.ok(close(x, ex) && close(y, ey) && close(z, ez));
    assert.ok(close(Math.cos(yaw - eyaw), 1.0));
  }
});

test("zero duration yields one sample, bad rate throws", () => {
  assert.equal(sampleTrajectory(TARGET, spec({ duration_s: 0 }), 10).length, 1);
  for (const hz of [0, -5]) {
    assert.throws(() => sampleTrajectory(TARGET, spec(), hz), RangeError);
  }
});
