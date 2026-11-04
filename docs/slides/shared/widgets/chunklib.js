// The maths behind the action-chunk illustration (no DOM, so it can be tested in node). Demonstrations are full paths of a
// gripper from the start to the box, around an obstacle on the left or on the right (T positions each). A chunk is K = 50
// consecutive positions, a vector in R^100. Given an observation o (the gripper position), the chunk distribution p(A | o) is
// made of the demonstrations' chunks that start near o (kernel width SIG_O). A^0 ~ N(0, I) is carried to a chunk by Euler steps of the exact marginal flow of the conditional
// OT paths A^τ = (1 − τ) A^0 + τ A^1 (the flow-matching target of the lecture), a weighted version of slide 23.
import { gauss } from './flowlib.js';

export const K = 50, D = 2 * K, SIG_O = 0.3;
export const NCH = 4, T = (K - 1) * NCH + 1;   // four chunks that share their end points: T = 197 positions
export const START = [0, -2.4], GOAL = [0, 2.25];
export const OBSTACLES = [{ x: 0, y: -0.85, hw: 0.45, hh: 0.4 }, { x: 0, y: 0.85, hw: 0.45, hh: 0.4 }];

// a smooth path through the waypoints (Catmull-Rom, uniform parameter): T positions
function spline(pts) {
  const n = pts.length, P = [pts[0], ...pts, pts[n - 1]], out = new Float64Array(2 * T);
  for (let i = 0; i < T; i++) {
    const u = i / (T - 1) * (n - 1), seg = Math.min(n - 2, Math.floor(u)), f = u - seg, [p0, p1, p2, p3] = [P[seg], P[seg + 1], P[seg + 2], P[seg + 3]];
    for (let c = 0; c < 2; c++) out[2 * i + c] = 0.5 * (2 * p1[c] + (-p0[c] + p2[c]) * f + (2 * p0[c] - 5 * p1[c] + 4 * p2[c] - p3[c]) * f * f + (-p0[c] + 3 * p1[c] - 3 * p2[c] + p3[c]) * f * f * f);
  }
  return out;
}
// demonstrations: around the first obstacle on side s1 and around the second on side s2 (four strategies, `per` of each)
export function makeDemos(r, per = 40) {
  const demos = [];
  for (let m = 0; m < 4 * per; m++) {
    const s1 = m & 1 ? 1 : -1, s2 = m & 2 ? 1 : -1, a1 = 1.0 + 0.5 * r(), a2 = 1.0 + 0.5 * r(), b = 0.9 + 0.4 * r();
    const pts = [[0.1 * gauss(r), START[1] + 0.05 * gauss(r)], [s1 * a1, OBSTACLES[0].y + 0.08 * gauss(r)], [s1 * (0.95 + 0.1 * r()), -0.35], [0.5 * (s1 + s2) * b, 0.06 * gauss(r)], [s2 * (0.95 + 0.1 * r()), 0.35], [s2 * a2, OBSTACLES[1].y + 0.08 * gauss(r)], [0.1 * gauss(r), GOAL[1] + 0.06 * gauss(r)]];
    const P = spline(pts); for (let i = 0; i < 2 * T; i++) P[i] += 0.012 * gauss(r);
    demos.push({ s1, s2, P });
  }
  return demos;
}

// p(A | o) at stage `stage` of the task (read off the observation: how far the gripper has come): the stage-th segment of every
// demonstration, bent to start exactly at o (the shift fades out along the chunk) and weighted by how close the demonstration's
// segment starts to o. The last segment ends at the box.
export function targetFor(o, demos, stage) {
  const X1 = [], lw = [], b = (K - 1) * stage;
  for (const { P } of demos) {
    const v = new Float64Array(D), ox = o[0] - P[2 * b], oy = o[1] - P[2 * b + 1];
    for (let k = 0; k < K; k++) { const f = 1 - k / (K - 1); v[2 * k] = P[2 * (b + k)] + ox * f; v[2 * k + 1] = P[2 * (b + k) + 1] + oy * f; }
    X1.push(v); lw.push(-(ox * ox + oy * oy) / (2 * SIG_O * SIG_O));
  }
  return { X1, lw };
}

// the marginal velocity at (t, x): conditional velocities (x1 − x)/(1 − t), weighted by prior(o) × posterior(x1 | x_t = x)
export function velocity(target, t, x) {
  const { X1, lw } = target, n = X1.length, s2 = 2 * (1 - t) * (1 - t), l = new Float64Array(n); let mx = -Infinity;
  for (let i = 0; i < n; i++) { let d2 = 0; const xi = X1[i]; for (let j = 0; j < D; j++) { const e = x[j] - t * xi[j]; d2 += e * e; } l[i] = lw[i] - d2 / s2; if (l[i] > mx) mx = l[i]; }
  let Z = 0; for (let i = 0; i < n; i++) { l[i] = Math.exp(l[i] - mx); Z += l[i]; }
  const u = new Float64Array(D);
  for (let i = 0; i < n; i++) { const w = l[i] / Z, xi = X1[i]; for (let j = 0; j < D; j++) u[j] += w * (xi[j] - x[j]); }
  for (let j = 0; j < D; j++) u[j] /= (1 - t);
  return u;
}
export function flow(target, x0, N) {
  const path = [Float64Array.from(x0)]; let x = path[0];
  for (let k = 0; k < N; k++) { const u = velocity(target, k / N, x), y = new Float64Array(D); for (let j = 0; j < D; j++) y[j] = x[j] + u[j] / N; path.push(y); x = y; }
  return path;
}

// one episode: inference after inference, each from the gripper position the previous chunk ended at. want = [s1, s2]: the
// sides around the first and the second obstacle (decided in the first and in the third chunk); the whole episode is
// re-sampled until it comes out that way
export function episode(r, demos, N, want = [0, 0]) {
  let infs = [], ok = false;
  for (let tries = 0; tries < 80 && !ok; tries++) {
    let o = START.slice(); infs = [];
    for (let j = 0; j < NCH; j++) {
      const target = targetFor(o, demos, j), path = flow(target, Float64Array.from({ length: D }, () => gauss(r)), N), F = path[N];
      infs.push({ path, o: o.slice() }); o = [F[2 * (K - 1)], F[2 * (K - 1) + 1]];
    }
    const e1 = Math.sign(infs[0].path[N][2 * (K - 1)]), e3 = Math.sign(infs[2].path[N][2 * (K - 1)]);
    ok = (!want[0] || e1 === want[0]) && (!want[1] || e3 === want[1]);
  }
  return { infs, ok };
}
