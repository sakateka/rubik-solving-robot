/* Exact cube permutation model (canonical frame).
 * Integer math: positions in {-1,0,1}³, orientation = images of unit axes.
 * 90° turns only — no float drift, comparisons are exact. */
'use strict';

const AXIS_OF_FACE = {
  R: [1, 0, 0], L: [-1, 0, 0],
  U: [0, 1, 0], D: [0, -1, 0],
  F: [0, 0, 1], B: [0, 0, -1],
};

function rotVec(v, axis, quarters) {
  let out = [...v];
  for (let i = 0; i < ((quarters % 4) + 4) % 4; i++) {
    const [x, y, z] = out;
    if (axis[0] > 0) out = [x, -z, y];
    else if (axis[0] < 0) out = [x, z, -y];
    else if (axis[1] > 0) out = [z, y, -x];
    else if (axis[1] < 0) out = [-z, y, x];
    else if (axis[2] > 0) out = [-y, x, z];
    else out = [y, -x, z];
  }
  return out;
}

function freshCube() {
  const state = [];
  for (let x = -1; x <= 1; x++)
    for (let y = -1; y <= 1; y++)
      for (let z = -1; z <= 1; z++) {
        if (!x && !y && !z) continue;
        state.push({
          p: [x, y, z],
          m: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        });
      }
  return state;
}

function applyMove(state, face, turns) {
  const axis = AXIS_OF_FACE[face];
  for (const c of state) {
    if (c.p[0] * axis[0] + c.p[1] * axis[1] + c.p[2] * axis[2] > 0) {
      c.p = rotVec(c.p, axis, turns);
      for (let col = 0; col < 3; col++) c.m[col] = rotVec(c.m[col], axis, turns);
    }
  }
}

/** Parses "R U R' F2" into [{face, turns}] with cw=-1, ccw=+1, half=+2. */
function parseMoves(text) {
  const moves = [];
  for (const token of text.split(/[\s,]+/).filter(Boolean)) {
    const m = /^([URFDLB])(2|'|'2|2')?$/.exec(token);
    if (!m) throw new Error(`bad move token: ${token}`);
    moves.push({ face: m[1], turns: m[2]?.includes('2') ? 2 : m[2] ? 1 : -1 });
  }
  return moves;
}

/** Compares live scene cubies against the expected model.
 * Returns a mismatch description ('ok' when equal). */
function diffWithScene(expected, actual) {
  const key = p => p.join(',');
  const expByPos = new Map(expected.map(c => [key(c.p), c]));
  let mismatches = 0;
  const details = [];
  if (actual.length !== expected.length) return `${actual.length} cubies in scene`;
  for (const act of actual) {
    const exp = expByPos.get(key(act.p));
    if (!exp) { mismatches++; details.push(`no model cubie at ${act.p}`); continue; }
    for (let col = 0; col < 3; col++)
      for (let row = 0; row < 3; row++)
        if (Math.abs(exp.m[col][row] - act.m[col][row]) > 0.05) {
          mismatches++;
          if (details.length < 4)
            details.push(`orientation at ${act.p} col${col}: ` +
              `model[${exp.m[col]}] scene[${act.m[col].map(Math.round)}]`);
        }
  }
  return mismatches === 0 ? 'ok' : `${mismatches} mismatches: ${details.join('; ')}`;
}

module.exports = { freshCube, applyMove, parseMoves, diffWithScene };
