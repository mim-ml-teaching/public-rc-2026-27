// Helpers for the magnetometer-calibration widgets: local Earth field per city, rotations, simulated attitudes,
// and an ellipsoid fit that returns the correction in the form ArduPilot stores it (offsets o, symmetric M, radius r).
//
// Frames: NED (north, east, down) for the Earth; body frame FRD (forward, right, down); rotations are 3×3 arrays of
// rows; R maps body → NED. Field values in milligauss (mG); 1 µT = 10 mG.
import { matmul, matvec, transpose, symEig, solve, rng } from './linalg.js';

// From ArduPilot's own table (libraries/AP_Declination/tables.cpp, IGRF, 10° grid, bilinear interpolation).
// F = total intensity (mG), I = inclination (deg, + = down), D = declination (deg, + = east).
export const CITIES = {
  Tallinn: { F: 523, I: 73.2, D: 9.6 },
  'Toruń': { F: 504, I: 68.4, D: 6.3 },
  Warsaw: { F: 504, I: 68.1, D: 6.8 },
  Opole: { F: 498, I: 66.8, D: 5.7 },
  Kyiv: { F: 509, I: 67.5, D: 8.3 },
};
export const deg = Math.PI / 180;

// Earth field in NED (mG); with magnetic = true the declination is ignored (x axis = magnetic north).
export function fieldNED(c, magnetic = false) {
  const H = c.F * Math.cos(c.I * deg), Z = c.F * Math.sin(c.I * deg), d = magnetic ? 0 : c.D * deg;
  return [H * Math.cos(d), H * Math.sin(d), Z];
}

export const Rx = a => { const c = Math.cos(a), s = Math.sin(a); return [[1, 0, 0], [0, c, -s], [0, s, c]]; };
export const Ry = a => { const c = Math.cos(a), s = Math.sin(a); return [[c, 0, s], [0, 1, 0], [-s, 0, c]]; };
export const Rz = a => { const c = Math.cos(a), s = Math.sin(a); return [[c, -s, 0], [s, c, 0], [0, 0, 1]]; };
// body → NED from roll, pitch, yaw (ZYX order, as ArduPilot's from_euler)
export const euler = (roll, pitch, yaw) => matmul(Rz(yaw), matmul(Ry(pitch), Rx(roll)));
export { matmul, matvec, transpose, rng };
export const add = (a, b) => a.map((v, i) => v + b[i]);
export const sub = (a, b) => a.map((v, i) => v - b[i]);
export const norm = a => Math.hypot(...a);

// Uniformly distributed random rotation (Shoemake's method).
function randomRotation(r) {
  const u1 = r(), u2 = 2 * Math.PI * r(), u3 = 2 * Math.PI * r();
  const a = Math.sqrt(1 - u1), b = Math.sqrt(u1);
  const [w, x, y, z] = [a * Math.sin(u2), a * Math.cos(u2), b * Math.sin(u3), b * Math.cos(u3)];
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
  ];
}

// n attitudes (body → NED). motion: "all" (every orientation), "tilts" (any yaw, roll/pitch within ±20°),
// "yaw" (flat spin on a table: roll = pitch = 0).
export function attitudes(n, motion = 'all', seed = 3) {
  const r = rng(seed), out = [];
  for (let k = 0; k < n; k++) {
    if (motion === 'all') out.push(randomRotation(r));
    else if (motion === 'tilts') out.push(euler((r() * 2 - 1) * 20 * deg, (r() * 2 - 1) * 20 * deg, r() * 2 * Math.PI));
    else out.push(euler(0, 0, (k + r() * 0.5) / n * 2 * Math.PI));
  }
  return out;
}

// Symmetric 3×3 square root / inverse square root via the eigen-decomposition.
function symFun(A, f) {
  const { vals, vecs } = symEig(A);
  const R = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  vals.forEach((l, k) => { const v = vecs[k], fl = f(l); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) R[i][j] += fl * v[i] * v[j]; });
  return R;
}
export const det3 = M => M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
  + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

// Algebraic least-squares ellipsoid fit:  pᵀAp + 2gᵀp = 1  (9 unknowns), points scaled by 1/500 for conditioning.
// Returns the correction  b̂ = M (m + o)  with M symmetric, det M = 1 (ArduPilot's DIA/ODI are normalised the same
// way up to the radius), |b̂| ≈ r for every sample; fitness = RMS of (r − |b̂|) in mG, as in CompassCalibrator.
export function fitEllipsoid(pts) {
  const s = 500, P = pts.map(p => p.map(v => v / s));
  const N = [...Array(9)].map(() => new Array(9).fill(0)), rhs = new Array(9).fill(0);
  for (const [x, y, z] of P) {
    const d = [x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z];
    for (let i = 0; i < 9; i++) { rhs[i] += d[i]; for (let j = 0; j < 9; j++) N[i][j] += d[i] * d[j]; }
  }
  const ev = symEig(N).vals, cond = ev[8] / Math.max(ev[0], 1e-300);
  const v = solve(N, rhs);
  if (!v) return { ok: false, cond, why: 'singular system' };
  const A = [[v[0], v[3], v[4]], [v[3], v[1], v[5]], [v[4], v[5], v[2]]], g = [v[6], v[7], v[8]];
  const c0 = solve(A, g.map(x => -x));                     // centre: A c0 = −g
  if (!c0) return { ok: false, cond, why: 'not an ellipsoid' };
  const k = 1 + matvec(A, c0).reduce((acc, a, i) => acc + a * c0[i], 0);
  const A1 = A.map(row => row.map(a => a / k));
  if (symEig(A1).vals[0] <= 0) return { ok: false, cond, why: 'not an ellipsoid' };
  const Mu = symFun(A1, l => Math.sqrt(l));                 // |Mu (p − c0)| = 1 in scaled units
  const rn = Math.cbrt(1 / det3(Mu));
  const M = Mu.map(row => row.map(a => a * rn));            // det M = 1
  const o = c0.map(c => -c * s), r = rn * s;
  let ss = 0;
  for (const p of pts) ss += (r - norm(matvec(M, add(p, o)))) ** 2;
  return { ok: true, o, M, r, fitness: Math.sqrt(ss / pts.length), cond, center: c0.map(c => c * s) };
}

// ArduPilot's acceptance test (CompassCalibrator::fit_acceptable, defaults COMPASS_OFFS_MAX = 1800, CAL_FIT = 16).
export function apAccept(fit, tol = 16) {
  if (!fit.ok) return { pass: false, why: fit.why };
  const M = fit.M, why = [];
  if (!(fit.r > 150 && fit.r < 950)) why.push('radius outside 150–950 mG');
  if (fit.o.some(v => Math.abs(v) >= 1800)) why.push('offset ≥ 1800');
  if ([M[0][0], M[1][1], M[2][2]].some(d => !(d > 0.2 && d < 5))) why.push('diagonal outside 0.2–5');
  if ([M[0][1], M[0][2], M[1][2]].some(d => Math.abs(d) >= 1)) why.push('off-diagonal ≥ 1');
  if (!(fit.fitness <= tol)) why.push(`fitness > ${tol} mG`);
  return { pass: why.length === 0, why: why.join(', ') };
}
