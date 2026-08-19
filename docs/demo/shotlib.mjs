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

  let theta = spec.speed * t;
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
