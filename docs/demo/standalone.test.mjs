// Guards the single-file build (build_standalone.py).
//
// standalone.html carries a copy of shotlib.mjs inlined into a script
// tag. A copy can rot, and the way it rots is silent: a bad string
// replace truncates the library, the page still loads, and every preset
// quietly reports the wrong verdict. So this checks two things.
//
//   1. Structure. The inlined region is exactly shotlib.mjs with the
//      export keywords removed, and nothing else.
//   2. Behaviour. The inlined code is loaded and exercised on its own,
//      not the module it was copied from, so a mangled copy fails here
//      rather than in front of whoever opened the file.

import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

const HERE = dirname(fileURLToPath(import.meta.url));
const BANNER =
  "// --- inlined from shotlib.mjs by build_standalone.py, do not edit here ---";

const html = readFileSync(join(HERE, "standalone.html"), "utf8");
const lib = readFileSync(join(HERE, "shotlib.mjs"), "utf8");

const STRIPPED_LIB = lib
  .replace(/export class /g, "class ")
  .replace(/export const /g, "const ")
  .replace(/export function /g, "function ")
  .trimEnd();

/** Just the library, not the page's own UI code that follows it. */
function inlinedLibrary() {
  assert.notEqual(
    html.indexOf(BANNER),
    -1,
    "banner missing: standalone.html was hand-edited",
  );
  const at = html.indexOf(STRIPPED_LIB);
  assert.notEqual(
    at,
    -1,
    "inlined library differs from shotlib.mjs; run build_standalone.py",
  );
  return html.slice(at, at + STRIPPED_LIB.length);
}

test("the inlined copy is shotlib.mjs with the exports stripped", () => {
  assert.equal(inlinedLibrary(), STRIPPED_LIB);
});

test("no export or import survives the inlining", () => {
  const script = html.slice(html.indexOf(BANNER), html.indexOf("</script>"));
  assert.doesNotMatch(script, /^\s*export\s/m);
  assert.doesNotMatch(script, /^\s*import\s/m);
});

test("the page is not a module script", () => {
  // A module script is blocked wherever the document has an opaque
  // origin, which is exactly the case this build exists to serve.
  assert.doesNotMatch(html, /<script[^>]*type=["']module["']/);
});

test("the page fetches nothing at runtime", () => {
  assert.doesNotMatch(html, /<script[^>]+src=/);
  assert.doesNotMatch(html, /<link[^>]+stylesheet/);
  assert.doesNotMatch(html, /https?:\/\/(?!127\.0\.0\.1)/);
});

// -------------------------------------------------------- behaviour

// Load the inlined text itself, so a truncated or mangled copy fails
// here. Exports are appended only to make it importable; they add no
// behaviour of their own.
const inlined = await import(
  "data:text/javascript," +
    encodeURIComponent(
      inlinedLibrary() +
        "\nexport { parseShotSpec, sampleTrajectory, ShotSpecError, LIMITS };",
    )
);

test("the inlined copy accepts a valid shot", () => {
  const spec = inlined.parseShotSpec(
    '{"shot":"orbit","radius":6,"height":3,"speed":0.4,' +
      '"duration_s":16,"clockwise":true,"look_at":"target"}',
  );
  assert.equal(spec.shot, "orbit");
  assert.equal(spec.radius, 6);
  assert.equal(spec.clamped.length, 0);
});

test("the inlined copy still finds JSON wrapped in prose", () => {
  const spec = inlined.parseShotSpec(
    'Sure! Here you go:\n```json\n{"shot":"orbit","radius":4.5,' +
      '"height":2,"speed":0.6,"duration_s":12,"clockwise":false,' +
      '"look_at":"target"}\n```\nLet me know if you want it tighter.',
  );
  assert.equal(spec.radius, 4.5);
});

test("the inlined copy still clamps out-of-range values", () => {
  const spec = inlined.parseShotSpec(
    '{"shot":"orbit","radius":400,"height":3,"speed":0.4,' +
      '"duration_s":16,"clockwise":true,"look_at":"target"}',
  );
  assert.equal(spec.radius, inlined.LIMITS.radius[1]);
  assert.ok(spec.clamped.includes("radius"));
});

test("the inlined copy still rejects an invented shot type", () => {
  assert.throws(
    () => inlined.parseShotSpec('{"shot":"barrel_roll","radius":6}'),
    inlined.ShotSpecError,
  );
});

test("the inlined copy still rejects prose with no JSON in it", () => {
  assert.throws(
    () => inlined.parseShotSpec("I would fly around it a couple of times."),
    inlined.ShotSpecError,
  );
});

test("the inlined copy still samples a trajectory", () => {
  const spec = inlined.parseShotSpec(
    '{"shot":"orbit","radius":6,"height":3,"speed":0.4,' +
      '"duration_s":16,"clockwise":true,"look_at":"target"}',
  );
  // Samples are [t, x, y, z, yaw], 16 s at 10 Hz inclusive of both ends.
  const traj = inlined.sampleTrajectory([0, 0, 0], spec, 10);
  assert.equal(traj.length, 161);
  for (const [, x, y, z] of traj) {
    assert.ok(Math.abs(Math.hypot(x, y) - 6) < 1e-9, "off the orbit");
    assert.equal(z, 3);
  }
});
