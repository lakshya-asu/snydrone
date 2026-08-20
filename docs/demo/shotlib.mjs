export class ShotSpecError extends Error {
  constructor(message) {
    super(message);
    this.name = 'ShotSpecError';
  }
}

export const DEFAULTS = {
  shot: 'orbit',
  radius: 3.0,
  height: 3.5,
  speed: 0.6,
  duration_s: 10.0,
  clockwise: true,
  look_at: 'target',
};

export const LIMITS = {
  radius: [1.0, 20.0],
  height: [0.5, 30.0],
  speed: [0.0, 3.0],
  duration_s: [0.5, 300.0],
};

const VALID_SHOTS = ['orbit', 'dolly_in', 'dolly_out', 'follow', 'pan'];
const VALID_LOOK_AT = ['target', 'none'];
const NUMERIC_FIELDS = ['radius', 'height', 'speed', 'duration_s'];

function extractJsonObject(raw) {
  const start = raw.indexOf('{');
  if (start === -1) return null;

  let depth = 0;
  let inString = false;
  let escaped = false;

  for (let i = start; i < raw.length; i++) {
    const ch = raw[i];
    if (escaped) {
      escaped = false;
      continue;
    }
    if (ch === '\\') {
      escaped = true;
      continue;
    }
    if (ch === '"') {
      inString = !inString;
      continue;
    }
    if (inString) continue;
    if (ch === '{') {
      depth++;
    } else if (ch === '}') {
      depth--;
      if (depth === 0) {
        return raw.substring(start, i + 1);
      }
    }
  }
  return null;
}

function coerceNumber(value, field) {
  if (typeof value === 'number') {
    if (!Number.isFinite(value)) {
      throw new ShotSpecError(field + ' must be a finite number');
    }
    return value;
  }
  if (typeof value === 'string') {
    if (value.trim() === '') {
      throw new ShotSpecError(field + ' must be a number, got empty string');
    }
    const n = Number(value);
    if (!Number.isFinite(n)) {
      throw new ShotSpecError(field + ' must be a number, got "' + value + '"');
    }
    return n;
  }
  throw new ShotSpecError(field + ' must be a number');
}

function coerceClockwise(value) {
  if (value === true || value === 1 || value === 'true' || value === 'True') return true;
  if (value === false || value === 0 || value === 'false' || value === 'False') return false;
  throw new ShotSpecError('clockwise must be a boolean');
}

export function parseShotSpec(raw) {
  if (typeof raw !== 'string') {
    throw new ShotSpecError('Input must be a string');
  }

  const jsonStr = extractJsonObject(raw);
  if (jsonStr === null) {
    throw new ShotSpecError('No JSON object found in input');
  }

  let obj;
  try {
    obj = JSON.parse(jsonStr);
  } catch (e) {
    throw new ShotSpecError('Invalid JSON: ' + e.message);
  }

  if (obj === null || typeof obj !== 'object' || Array.isArray(obj)) {
    throw new ShotSpecError('Spec must be a JSON object');
  }

  const result = {};

  // shot
  if (obj.shot === undefined) {
    result.shot = DEFAULTS.shot;
  } else {
    if (typeof obj.shot !== 'string') {
      throw new ShotSpecError('shot must be a string');
    }
    result.shot = obj.shot.trim().toLowerCase();
    if (!VALID_SHOTS.includes(result.shot)) {
      throw new ShotSpecError('Invalid shot: ' + result.shot);
    }
  }

  // Numeric fields
  for (const field of NUMERIC_FIELDS) {
    if (obj[field] === undefined) {
      result[field] = DEFAULTS[field];
    } else {
      result[field] = coerceNumber(obj[field], field);
    }
  }

  // clockwise
  if (obj.clockwise === undefined) {
    result.clockwise = DEFAULTS.clockwise;
  } else {
    result.clockwise = coerceClockwise(obj.clockwise);
  }

  // look_at
  if (obj.look_at === undefined) {
    result.look_at = DEFAULTS.look_at;
  } else {
    if (typeof obj.look_at !== 'string') {
      throw new ShotSpecError('look_at must be a string');
    }
    result.look_at = obj.look_at.trim().toLowerCase();
    if (!VALID_LOOK_AT.includes(result.look_at)) {
      throw new ShotSpecError('Invalid look_at: ' + result.look_at);
    }
  }

  // Clamp numeric fields
  const clamped = [];
  for (const field of NUMERIC_FIELDS) {
    const [min, max] = LIMITS[field];
    if (result[field] < min) {
      result[field] = min;
      clamped.push(field);
    } else if (result[field] > max) {
      result[field] = max;
      clamped.push(field);
    }
  }

  result.clamped = clamped;

  return result;
}

export function orbitSetpoint(target, t, spec) {
  const tx = target[0];
  const ty = target[1];
  const tz = target[2];

  if (spec.radius <= 0) {
    throw new RangeError('orbit radius must be positive, got ' + spec.radius);
  }

  // speed is linear metres per second along the orbit path (ratified
  // 2026-08-17), so the angular rate depends on the radius, exactly as
  // in the flight code's orbit_geometry.py.
  let theta = (spec.speed / spec.radius) * t;
  if (spec.clockwise) {
    theta = -theta;
  }

  const x = tx + spec.radius * Math.cos(theta);
  const y = ty + spec.radius * Math.sin(theta);
  const z = tz + spec.height;

  let yaw = 0;
  if (spec.look_at === 'target') {
    yaw = Math.atan2(ty - y, tx - x);
  }

  return [x, y, z, yaw];
}

// Direct port of snydrone_shots/feasibility.py. The Python is the
// authority; where the two disagree, the Python wins and this is the bug.
export const FLIGHT_LIMITS = {
  max_speed_mps: 8.0,
  max_accel_mps2: 6.0,
  max_yaw_rate_rps: 2.0,
  min_altitude_m: 0.5,
  max_altitude_m: 120.0,
  geofence_radius_m: 100.0,
  keep_out_centre: [0.0, 0.0],
  keep_out_radius_m: 0.0,
};

function over(value, limit) {
  return value > limit + Math.max(Math.abs(limit), 1.0) * 1e-9;
}

function below(value, limit) {
  return value < limit - Math.max(Math.abs(limit), 1.0) * 1e-9;
}

function pointSegmentDistance(px, py, x1, y1, x2, y2) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) return Math.hypot(px - x1, py - y1);
  let t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
  t = Math.max(0.0, Math.min(1.0, t));
  return Math.hypot(px - (x1 + t * dx), py - (y1 + t * dy));
}

export function checkTrajectory(traj, limits) {
  if (traj.length < 2) {
    throw new RangeError('trajectory must have at least 2 samples');
  }
  for (const sample of traj) {
    for (const value of sample) {
      if (!Number.isFinite(value)) {
        throw new RangeError('trajectory contains non-finite values');
      }
    }
  }
  for (let i = 1; i < traj.length; i++) {
    if (traj[i][0] <= traj[i - 1][0]) {
      throw new RangeError('timestamps must be strictly increasing');
    }
  }

  const merged = { ...FLIGHT_LIMITS, ...(limits || {}) };
  const violations = [];
  const n = traj.length;

  const segVels = [];
  const segSpeeds = [];
  for (let i = 0; i < n - 1; i++) {
    const dt = traj[i + 1][0] - traj[i][0];
    const vx = (traj[i + 1][1] - traj[i][1]) / dt;
    const vy = (traj[i + 1][2] - traj[i][2]) / dt;
    const vz = (traj[i + 1][3] - traj[i][3]) / dt;
    segVels.push([vx, vy, vz]);
    segSpeeds.push(Math.hypot(vx, vy, vz));
  }

  for (let i = 0; i < n - 1; i++) {
    if (over(segSpeeds[i], merged.max_speed_mps)) {
      violations.push({ kind: 'speed', index: i, value: segSpeeds[i],
                        limit: merged.max_speed_mps });
    }
  }

  // Tangential acceleration, reported at the middle sample.
  const tang = new Array(Math.max(0, n - 2)).fill(0.0);
  for (let i = 0; i < n - 2; i++) {
    const dt = (traj[i + 2][0] - traj[i][0]) / 2.0;
    const acc = Math.abs(segSpeeds[i + 1] - segSpeeds[i]) / dt;
    tang[i] = acc;
    if (over(acc, merged.max_accel_mps2)) {
      violations.push({ kind: 'acceleration', index: i + 1, value: acc,
                        limit: merged.max_accel_mps2 });
    }
  }

  // Centripetal (lateral) acceleration with the chord correction
  // (dphi/2)/sin(dphi/2): exact on constant arcs, 1 on straight motion.
  const lats = new Array(Math.max(0, n - 2)).fill(0.0);
  for (let i = 0; i < n - 2; i++) {
    const v1 = segVels[i];
    const v2 = segVels[i + 1];
    const s1 = segSpeeds[i];
    const s2 = segSpeeds[i + 1];
    if (s1 < 1e-9 || s2 < 1e-9) continue;
    const dot = (v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]) / (s1 * s2);
    const dphi = Math.acos(Math.max(-1.0, Math.min(1.0, dot)));
    const dt = (traj[i + 2][0] - traj[i][0]) / 2.0;
    let lat = 0.5 * (s1 + s2) * dphi / dt;
    if (dphi > 1e-9) lat *= (dphi / 2.0) / Math.sin(dphi / 2.0);
    lats[i] = lat;
    if (over(lat, merged.max_accel_mps2)) {
      violations.push({ kind: 'centripetal', index: i + 1, value: lat,
                        limit: merged.max_accel_mps2 });
    }
  }

  // Vector norm of the two perpendicular components, reported only where
  // neither component tripped on its own.
  for (let i = 0; i < n - 2; i++) {
    const norm = Math.hypot(tang[i], lats[i]);
    if (!over(norm, merged.max_accel_mps2)) continue;
    if (over(tang[i], merged.max_accel_mps2)) continue;
    if (over(lats[i], merged.max_accel_mps2)) continue;
    violations.push({ kind: 'accel_norm', index: i + 1, value: norm,
                      limit: merged.max_accel_mps2 });
  }

  for (let i = 0; i < n - 1; i++) {
    const dt = traj[i + 1][0] - traj[i][0];
    const dyaw = traj[i + 1][4] - traj[i][4];
    const wrapped = Math.atan2(Math.sin(dyaw), Math.cos(dyaw));
    const rate = Math.abs(wrapped) / dt;
    if (over(rate, merged.max_yaw_rate_rps)) {
      violations.push({ kind: 'yaw_rate', index: i, value: rate,
                        limit: merged.max_yaw_rate_rps });
    }
  }

  for (let i = 0; i < n; i++) {
    const z = traj[i][3];
    if (below(z, merged.min_altitude_m)) {
      violations.push({ kind: 'altitude', index: i, value: z,
                        limit: merged.min_altitude_m });
    } else if (over(z, merged.max_altitude_m)) {
      violations.push({ kind: 'altitude', index: i, value: z,
                        limit: merged.max_altitude_m });
    }
  }

  for (let i = 0; i < n; i++) {
    const r = Math.hypot(traj[i][1], traj[i][2]);
    if (over(r, merged.geofence_radius_m)) {
      violations.push({ kind: 'geofence', index: i, value: r,
                        limit: merged.geofence_radius_m });
    }
  }

  if (merged.keep_out_radius_m > 0) {
    const [cx, cy] = merged.keep_out_centre;
    for (let i = 0; i < n - 1; i++) {
      const dist = pointSegmentDistance(
        cx, cy, traj[i][1], traj[i][2], traj[i + 1][1], traj[i + 1][2]);
      if (below(dist, merged.keep_out_radius_m)) {
        violations.push({ kind: 'keep_out', index: i, value: dist,
                          limit: merged.keep_out_radius_m });
      }
    }
  }

  violations.sort((a, b) => a.index - b.index);
  return { ok: violations.length === 0, violations };
}

const VIOLATION_UNITS = {
  speed: 'm/s', acceleration: 'm/s^2', centripetal: 'm/s^2',
  accel_norm: 'm/s^2', yaw_rate: 'rad/s', altitude: 'm',
  geofence: 'm', keep_out: 'm',
};

const VIOLATION_NOUNS = {
  speed: 'speed',
  acceleration: 'tangential acceleration',
  centripetal: 'centripetal acceleration',
  accel_norm: 'total acceleration',
  yaw_rate: 'yaw rate',
  altitude: 'altitude',
  geofence: 'distance from origin',
  keep_out: 'clearance to the keep-out zone',
};

export function describeViolation(v) {
  const unit = VIOLATION_UNITS[v.kind] || '';
  const noun = VIOLATION_NOUNS[v.kind] || v.kind;
  const f = (x) => x.toFixed(2);
  if (v.value > v.limit) {
    return `${noun} ${f(v.value)} ${unit} exceeds the ${f(v.limit)} ${unit} ` +
           `limit by ${f(v.value - v.limit)} ${unit}`;
  }
  return `${noun} ${f(v.value)} ${unit} is below the ${f(v.limit)} ${unit} ` +
         `minimum by ${f(v.limit - v.value)} ${unit}`;
}

export function worstByKind(violations) {
  const worst = {};
  for (const v of violations) {
    const margin = Math.abs(v.value - v.limit);
    const cur = worst[v.kind];
    if (!cur || margin > Math.abs(cur.value - cur.limit)) worst[v.kind] = v;
  }
  return worst;
}

export function sampleTrajectory(target, spec, hz) {
  if (hz <= 0) {
    throw new RangeError('hz must be positive');
  }

  const duration = spec.duration_s;
  const samples = [];

  if (duration === 0) {
    const pose = orbitSetpoint(target, 0, spec);
    samples.push([0, pose[0], pose[1], pose[2], pose[3]]);
    return samples;
  }

  const n = Math.floor(duration * hz);

  for (let i = 0; i <= n; i++) {
    const t = i / hz;
    const pose = orbitSetpoint(target, t, spec);
    samples.push([t, pose[0], pose[1], pose[2], pose[3]]);
  }

  if (n / hz < duration) {
    const pose = orbitSetpoint(target, duration, spec);
    samples.push([duration, pose[0], pose[1], pose[2], pose[3]]);
  }

  return samples;
}
