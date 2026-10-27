// Incremental structure from motion, computed for real on a small synthetic scene (three.js).
// The scene: a house (4 × 3 m footprint, 2.5 m walls, a gable roof to 3.7 m, a door and windows) on the ground,
// about 125 feature points on its faces and on the ground, and 9 cameras on a ring (radius ≈ 9 m, height ≈ 2.5 m)
// looking at it, numbered as if someone walked around the house. Intrinsics f = 400 px, image 480 × 360.
// Observations are exact projections plus Gaussian noise (σ = 0.7 px) of the points a camera can see (inside the
// image, in front of the camera, on a face turned towards it, not hidden by the house); no outliers.
//
// Frames: world = three.js axes (y up, metres); camera frames as in OpenCV (x right, y down, z forward), a camera
// is (w, t) with x_c = R(w) X + t, R = rodrigues(w), centre c = −Rᵀ t.
//
// The pipeline (every step is a real computation, done once at mount and stored, so stepping back and forth is free):
//   step 0  initial pair: camera 1 defines the frame; camera 2's relative pose stands in for the one from E
//           (the true pose perturbed by 1° / 2°, the baseline length is the gauge); the points both see are
//           triangulated (linear DLT).
//   step 1  bundle adjustment of the pair.
//   then for each camera k = 3 … 9:  register k by PnP (Gauss-Newton on the 2D–3D reprojection error, started from
//           a perturbed pose: in a real pipeline the start comes from P3P + RANSAC, lecture 2), triangulate the
//           new points (seen by ≥ 2 registered cameras), bundle adjustment: local (cameras k−2 … k and their points,
//           the other cameras fixed) or, for k = 3, 6, 9, global (everything).
// Bundle adjustment: Levenberg–Marquardt with numeric Jacobians, normal equations reduced to the cameras by the Schur
// complement (the point blocks are 3 × 3), dense Cholesky on the reduced system. Gauge: camera 1 fixed and one
// translation component of camera 2 fixed (the scale): 7 constraints. The whole pipeline takes < 0.1 s.
// Readout errors are RMS over all observations of the registered cameras (noise σ = 0.7 px per coordinate, so the
// floor is about 1 px, less once the unknowns absorb part of the noise).
//
// Display: the 3D view (registered cameras as frustums, the camera being registered in green with rays to the points
// it uses, points: established fg, new in this step accent; the true house and the not-yet-registered cameras as a
// faint wireframe: "truth"), the image of the current camera (observed features: dots, reprojections of the current
// estimate: red crosses, residuals drawn ×10) and a readout with the counts and the mean reprojection error.
// After a bundle-adjustment step the cameras and points glide from the old to the refined positions.
//
// config: {
//   width: 640, height: 400    3D view (px)
//   imageWidth: 300            the image panel (the 480 × 360 image scaled down)
//   step: null                 start at this step (0 … 22; disables autoplay); print default: the last step
//   autoplay: true             advance automatically while the slide is shown (never in print); speed 1×/2×/4×
//   truth: true                show the true house and camera positions (faint)
//   seed: 3
//   noise: 0.7                 pixel noise σ of the observations (px); larger: visibly scattered points, larger residuals
//   perturb: 1                 scale of the initial-pose perturbations (pair: 1° / 2°, PnP start: 3° / 0.3 m, times this)
//   residualScale: 10          magnification of the residuals drawn in the image
//   glide: 0.6                 seconds of the move after a bundle-adjustment step
//   pnpPoints: 0               register each camera on only this many of the points it sees (0: all of them): a cruder
//                              pose, as from a minimal P3P sample, so the adjustments have more to correct
//   baEvery: 1                 bundle adjustment after every n-th registered camera (1: after each, local, global every
//                              third; n > 1: global, after cameras n, 2n, …, and after the last one)
// }
import { C, h, fmt, isPrint } from './util.js';
import { THREE, makeStage, label, viewButtons } from './three-util.js';
import { rng, rodrigues, logR, symEig } from './linalg.js';

const IW = 480, IH = 360, F = 400, OX = IW / 2, OY = IH / 2, SIGMA = 0.7;
const NCAM = 9, RING = 9, CAMH = 2.5, S = 0.9;        // S: distance of the drawn image rectangle from the centre
const HW = 2, HD = 1.5, HH = 2.5, RY = 3.7;           // house: half-width (x), half-depth (z), wall height, ridge height
const deg = Math.PI / 180;

// ---------- small vector helpers ----------
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const nrm = a => { const n = Math.hypot(...a) || 1; return scl(a, 1 / n); };
const mv = (R, x) => [dot(R[0], x), dot(R[1], x), dot(R[2], x)];                 // R x
const mt = (R, x) => [0, 1, 2].map(i => R[0][i] * x[0] + R[1][i] * x[1] + R[2][i] * x[2]); // Rᵀ x
const lerp3 = (a, b, m) => [0, 1, 2].map(i => a[i] + m * (b[i] - a[i]));

// ---------- cameras ----------
function lookCam(c, target) {
  const f = nrm(sub(target, c)), r = nrm(cross(f, [0, 1, 0])), d = cross(f, r);
  const R = [r, d, f];                                   // rows: world → camera
  return { w: logR(R), t: scl(mv(R, c), -1) };
}
const centreOf = cam => scl(mt(rodrigues(cam.w), cam.t), -1);
const camFrom = (w, c) => ({ w: w.slice(), t: scl(mv(rodrigues(w), c), -1) });
function projectR(R, t, X) { const x = add(mv(R, X), t); return [F * x[0] / x[2] + OX, F * x[1] / x[2] + OY, x[2]]; }
const residual = (cam, X, uv) => { const p = projectR(rodrigues(cam.w), cam.t, X); return [p[0] - uv[0], p[1] - uv[1]]; };
const randAxis = r => nrm([r.normal(), r.normal(), r.normal()]);
const perturbR = (w, angle, r) => logR(mulR(rodrigues(scl(randAxis(r), angle)), rodrigues(w)));
const mulR = (A, B) => A.map(row => [0, 1, 2].map(j => row[0] * B[0][j] + row[1] * B[1][j] + row[2] * B[2][j]));

// ---------- the scene ----------
function makeScene(seed) {
  const r = rng(seed);
  const pts = [];
  const P = (p, ...ns) => pts.push({ p, ns });
  const NF = [0, 0, 1], NB = [0, 0, -1], NL = [-1, 0, 0], NR = [1, 0, 0], NU = [0, 1, 0];
  const NRF = nrm([0, HD, RY - HH]), NRB = nrm([0, HD, -(RY - HH)]);   // roof slopes
  // wall corners, ridge ends
  for (const sx of [-1, 1]) for (const sz of [-1, 1]) {
    P([sx * HW, 0, sz * HD], sx > 0 ? NR : NL, sz > 0 ? NF : NB);
    P([sx * HW, HH, sz * HD], sx > 0 ? NR : NL, sz > 0 ? NF : NB, sz > 0 ? NRF : NRB);
  }
  for (const sx of [-1, 1]) P([sx * HW, RY, 0], sx > 0 ? NR : NL, NRF, NRB);
  // door (front) and windows (front, back, sides)
  for (const x of [-0.4, 0.4]) for (const y of [0, 2]) P([x, y, HD], NF);
  for (const x0 of [-1.7, 0.9]) for (const dx of [0, 0.8]) for (const y of [1, 1.8]) { P([x0 + dx, y, HD], NF); P([x0 + dx, y, -HD], NB); }
  for (const z of [-0.4, 0.4]) for (const y of [1, 1.8]) { P([HW, y, z], NR); P([-HW, y, z], NL); }
  // texture points on the walls, the gables and the roof
  for (let i = 0; i < 8; i++) { P([-HW + 2 * HW * r(), 0.2 + (HH - 0.4) * r(), HD], NF); P([-HW + 2 * HW * r(), 0.2 + (HH - 0.4) * r(), -HD], NB); }
  for (let i = 0; i < 7; i++) { P([HW, 0.2 + (HH - 0.4) * r(), -HD + 2 * HD * r()], NR); P([-HW, 0.2 + (HH - 0.4) * r(), -HD + 2 * HD * r()], NL); }
  for (let i = 0; i < 3; i++) for (const sx of [-1, 1]) { const u = r(); const z = (2 * r() - 1) * HD * (1 - u); P([sx * HW, HH + (RY - HH) * u * 0.9, z], sx > 0 ? NR : NL); }
  for (let i = 0; i < 10; i++) for (const sz of [-1, 1]) { const u = 0.1 + 0.8 * r(); P([-HW + 2 * HW * r(), HH + (RY - HH) * u, sz * HD * (1 - u)], sz > 0 ? NRF : NRB); }
  // ground: an annulus around the house
  for (let i = 0; i < 30; i++) { const a = 2 * Math.PI * r(), d = 2.8 + 3.6 * r(); P([d * Math.cos(a), 0, d * Math.sin(a)], NU); }
  // the cameras on the ring, walking around the house
  const cams = [];
  for (let k = 0; k < NCAM; k++) {
    const a = -Math.PI / 2 + 2 * Math.PI * k / NCAM + 0.25 * (r() - 0.5);
    const d = RING + (r() - 0.5), c = [d * Math.cos(a), CAMH + 0.6 * (r() - 0.5), d * Math.sin(a)];
    cams.push(lookCam(c, [0.3 * (r() - 0.5), 1.4 + 0.3 * (r() - 0.5), 0.3 * (r() - 0.5)]));
  }
  return { pts, cams };
}

// segment a → b passes through the box [lo, hi] (slab test)
function hitsBox(a, b, lo, hi) {
  let t0 = 0, t1 = 1;
  for (let i = 0; i < 3; i++) {
    const d = b[i] - a[i];
    if (Math.abs(d) < 1e-12) { if (a[i] < lo[i] || a[i] > hi[i]) return false; continue; }
    let ta = (lo[i] - a[i]) / d, tb = (hi[i] - a[i]) / d;
    if (ta > tb) [ta, tb] = [tb, ta];
    t0 = Math.max(t0, ta); t1 = Math.min(t1, tb);
    if (t0 > t1) return false;
  }
  return true;
}
const BOXLO = [-HW + 0.05, 0.05, -HD + 0.05], BOXHI = [HW - 0.05, HH - 0.05, HD - 0.05];

function observe(scene, seed, sigma = SIGMA) {
  const r = rng(seed * 31 + 7);
  return scene.cams.map(cam => {
    const c = centreOf(cam), R = rodrigues(cam.w), m = new Map();
    scene.pts.forEach((pt, i) => {
      if (!pt.ns.some(n => dot(sub(c, pt.p), n) > 0.05)) return;
      if (hitsBox(c, pt.p, BOXLO, BOXHI)) return;
      const p = projectR(R, cam.t, pt.p);
      if (p[2] < 0.3 || p[0] < 2 || p[0] > IW - 2 || p[1] < 2 || p[1] > IH - 2) return;
      m.set(i, [p[0] + sigma * r.normal(), p[1] + sigma * r.normal()]);
    });
    return m;
  });
}

// ---------- geometry solvers ----------
// linear triangulation (DLT) from the observations of point i in the given cameras
function triangulate(views) {
  const A = [];
  for (const { cam, uv } of views) {
    const R = rodrigues(cam.w), t = cam.t, x = (uv[0] - OX) / F, y = (uv[1] - OY) / F;
    A.push([x * R[2][0] - R[0][0], x * R[2][1] - R[0][1], x * R[2][2] - R[0][2], x * t[2] - t[0]]);
    A.push([y * R[2][0] - R[1][0], y * R[2][1] - R[1][1], y * R[2][2] - R[1][2], y * t[2] - t[1]]);
  }
  const AtA = [0, 1, 2, 3].map(i => [0, 1, 2, 3].map(j => A.reduce((s, row) => s + row[i] * row[j], 0)));
  const { vecs } = symEig(AtA), v = vecs[0];
  if (Math.abs(v[3]) < 1e-9) return null;
  const X = [v[0] / v[3], v[1] / v[3], v[2] / v[3]];
  for (const { cam } of views) if (add(mv(rodrigues(cam.w), X), cam.t)[2] < 0.1) return null;
  return X;
}

function cholSolve(A, b, n) {
  const L = Float64Array.from(A);
  for (let j = 0; j < n; j++) {
    let d = L[j * n + j];
    for (let k = 0; k < j; k++) d -= L[j * n + k] ** 2;
    if (d <= 1e-14) return null;
    d = Math.sqrt(d); L[j * n + j] = d;
    for (let i = j + 1; i < n; i++) {
      let s = L[i * n + j];
      for (let k = 0; k < j; k++) s -= L[i * n + k] * L[j * n + k];
      L[i * n + j] = s / d;
    }
  }
  const y = new Float64Array(n), x = new Float64Array(n);
  for (let i = 0; i < n; i++) { let s = b[i]; for (let k = 0; k < i; k++) s -= L[i * n + k] * y[k]; y[i] = s / L[i * n + i]; }
  for (let i = n - 1; i >= 0; i--) { let s = y[i]; for (let k = i + 1; k < n; k++) s -= L[k * n + i] * x[k]; x[i] = s / L[i * n + i]; }
  return x;
}
function inv3(M) {
  const [a, b, c, d, e, f, g, hh, i] = M;
  const det = a * (e * i - f * hh) - b * (d * i - f * g) + c * (d * hh - e * g);
  if (Math.abs(det) < 1e-18) return null;
  return [e * i - f * hh, c * hh - b * i, b * f - c * e, f * g - d * i, a * i - c * g, c * d - a * f, d * hh - e * g, b * g - a * hh, a * e - b * d].map(v => v / det);
}

// residual and its numeric Jacobian blocks: 2 × 6 (camera: w, t) and 2 × 3 (point)
function jac(cam, X, uv) {
  const r0 = residual(cam, X, uv), Jc = [new Array(6), new Array(6)], Jp = [new Array(3), new Array(3)], e = 1e-6;
  for (let p = 0; p < 6; p++) {
    const q = { w: cam.w.slice(), t: cam.t.slice() };
    (p < 3 ? q.w : q.t)[p % 3] += e;
    const r = residual(q, X, uv);
    Jc[0][p] = (r[0] - r0[0]) / e; Jc[1][p] = (r[1] - r0[1]) / e;
  }
  for (let p = 0; p < 3; p++) {
    const Y = X.slice(); Y[p] += e;
    const r = residual(cam, Y, uv);
    Jp[0][p] = (r[0] - r0[0]) / e; Jp[1][p] = (r[1] - r0[1]) / e;
  }
  return { r: r0, Jc, Jp };
}

// Levenberg–Marquardt on the reprojection error. cams, pts: Maps (mutated on accept). free: [{ k, params }] cameras whose
// listed parameters (0 … 5: w, t) are unknown; fixed: cameras that only constrain the points; ptIds: the points
// (unknown if pointsFree). Returns { before, after } (cost = Σ‖r‖²) and the number of observations.
function optimise({ cams, pts, obs, free, fixed, ptIds, pointsFree, iters = 10 }) {
  const camOff = new Map(); let nc = 0;
  for (const f of free) { camOff.set(f.k, nc); nc += f.params.length; }
  const pIdx = new Map(ptIds.map((i, j) => [i, j])), np = ptIds.length;
  const camList = [...free.map(f => f.k), ...fixed];
  const observations = [];
  for (const k of camList) for (const i of ptIds) if (obs[k].has(i)) observations.push([k, i, obs[k].get(i)]);
  const freeOf = new Map(free.map(f => [f.k, f.params]));
  const cost = (cm, pm) => observations.reduce((s, [k, i, uv]) => { const r = residual(cm.get(k), pm.get(i), uv); return s + r[0] * r[0] + r[1] * r[1]; }, 0);
  const copyC = m => new Map([...m].map(([k, c]) => [k, { w: c.w.slice(), t: c.t.slice() }]));
  const copyP = m => new Map([...m].map(([i, X]) => [i, X.slice()]));
  let lambda = 1e-3, c0 = cost(cams, pts);
  const before = c0;
  for (let it = 0; it < iters; it++) {
    const Hc = new Float64Array(nc * nc), bc = new Float64Array(nc);
    const V = new Float64Array(9 * np), bp = new Float64Array(3 * np), W = new Float64Array(nc * 3 * np);
    for (const [k, i, uv] of observations) {
      const { r, Jc, Jp } = jac(cams.get(k), pts.get(i), uv);
      const fp = freeOf.get(k), oc = camOff.get(k), j = pIdx.get(i);
      if (fp) for (let a = 0; a < fp.length; a++) {
        const pa = fp[a];
        bc[oc + a] -= Jc[0][pa] * r[0] + Jc[1][pa] * r[1];
        for (let b = 0; b < fp.length; b++) { const pb = fp[b]; Hc[(oc + a) * nc + oc + b] += Jc[0][pa] * Jc[0][pb] + Jc[1][pa] * Jc[1][pb]; }
        if (pointsFree) for (let q = 0; q < 3; q++) W[(oc + a) * 3 * np + 3 * j + q] += Jc[0][pa] * Jp[0][q] + Jc[1][pa] * Jp[1][q];
      }
      if (pointsFree) for (let a = 0; a < 3; a++) {
        bp[3 * j + a] -= Jp[0][a] * r[0] + Jp[1][a] * r[1];
        for (let b = 0; b < 3; b++) V[9 * j + 3 * a + b] += Jp[0][a] * Jp[0][b] + Jp[1][a] * Jp[1][b];
      }
    }
    let accepted = false;
    for (let tries = 0; tries < 6 && !accepted; tries++) {
      // damped blocks, Schur complement S = Hc − W V⁻¹ Wᵀ, g = bc − W V⁻¹ bp
      const Sm = Float64Array.from(Hc), g = Float64Array.from(bc);
      for (let a = 0; a < nc; a++) Sm[a * nc + a] *= 1 + lambda;
      const Vinv = [];
      if (pointsFree) for (let j = 0; j < np; j++) {
        const Vd = Array.from(V.subarray(9 * j, 9 * j + 9));
        for (let a = 0; a < 3; a++) Vd[3 * a + a] = Vd[3 * a + a] * (1 + lambda) + 1e-9;
        const Vi = inv3(Vd); Vinv.push(Vi);
        if (!Vi) continue;
        // WV = W_j V⁻¹ (nc × 3)
        const WV = new Float64Array(nc * 3);
        for (let a = 0; a < nc; a++) for (let q = 0; q < 3; q++) { let s = 0; for (let m = 0; m < 3; m++) s += W[a * 3 * np + 3 * j + m] * Vi[3 * m + q]; WV[3 * a + q] = s; }
        for (let a = 0; a < nc; a++) {
          let s = 0; for (let q = 0; q < 3; q++) s += WV[3 * a + q] * bp[3 * j + q];
          g[a] -= s;
          for (let b = 0; b < nc; b++) { let v = 0; for (let q = 0; q < 3; q++) v += WV[3 * a + q] * W[b * 3 * np + 3 * j + q]; Sm[a * nc + b] -= v; }
        }
      }
      const dc = nc ? cholSolve(Sm, g, nc) : new Float64Array(0);
      if (nc && !dc) { lambda *= 10; continue; }
      const cm = copyC(cams), pm = copyP(pts);
      for (const f of free) { const c = cm.get(f.k), oc = camOff.get(f.k); f.params.forEach((p, a) => { (p < 3 ? c.w : c.t)[p % 3] += dc[oc + a]; }); }
      if (pointsFree) for (let j = 0; j < np; j++) {
        const Vi = Vinv[j]; if (!Vi) continue;
        const rhs = [0, 1, 2].map(q => { let s = bp[3 * j + q]; for (let a = 0; a < nc; a++) s -= W[a * 3 * np + 3 * j + q] * dc[a]; return s; });
        const X = pm.get(ptIds[j]);
        for (let a = 0; a < 3; a++) X[a] += Vi[3 * a] * rhs[0] + Vi[3 * a + 1] * rhs[1] + Vi[3 * a + 2] * rhs[2];
      }
      const c1 = cost(cm, pm);
      if (c1 < c0) {
        for (const [k, c] of cm) { const d = cams.get(k); d.w = c.w; d.t = c.t; }
        for (const [i, X] of pm) pts.set(i, X);
        const rel = (c0 - c1) / c0;
        c0 = c1; lambda = Math.max(lambda / 3, 1e-9); accepted = true;
        if (rel < 1e-7) it = iters;
      } else lambda *= 5;
    }
    if (!accepted) break;
  }
  return { before, after: c0, n: observations.length };
}

// ---------- the pipeline, all steps precomputed ----------
function runPipeline(scene, obs, seed, perturb = 1, pnpPoints = 0, baEvery = 1) {
  const r = rng(seed * 17 + 5);
  const T = scene.cams;
  const cams = new Map(), pts = new Map();
  const snaps = [];
  // gauge: camera 1 fixed (6), camera 2's t_z fixed (the scale; t_z ≈ its distance to the house, the largest component)
  const gauge = k => (k === 0 ? null : k === 1 ? { k, params: [0, 1, 2, 3, 4] } : { k, params: [0, 1, 2, 3, 4, 5] });
  const meanErr = () => {   // RMS of the reprojection error over all current observations
    let s = 0, n = 0;
    for (const [k, cam] of cams) for (const [i, uv] of obs[k]) if (pts.has(i)) { const e = residual(cam, pts.get(i), uv); s += e[0] * e[0] + e[1] * e[1]; n++; }
    return n ? Math.sqrt(s / n) : 0;
  };
  const nObs = () => { let n = 0; for (const [k] of cams) for (const [i] of obs[k]) if (pts.has(i)) n++; return n; };
  const snap = (name, kind, cur, extra = {}) => snaps.push({
    name, kind, cur,
    cams: new Map([...cams].map(([k, c]) => [k, { w: c.w.slice(), t: c.t.slice(), c: centreOf(c) }])),
    pts: new Map([...pts].map(([i, X]) => [i, X.slice()])),
    err: meanErr(), nObs: nObs(), newPts: new Set(), rays: [], ...extra,
  });
  const triangulateNew = () => {
    const added = [];
    scene.pts.forEach((_, i) => {
      if (pts.has(i)) return;
      const views = [...cams].filter(([k]) => obs[k].has(i)).map(([k, cam]) => ({ cam, uv: obs[k].get(i) }));
      if (views.length < 2) return;
      const X = triangulate(views);
      if (X) { pts.set(i, X); added.push(i); }
    });
    return added;
  };
  // step 0: the initial pair
  cams.set(0, { w: T[0].w.slice(), t: T[0].t.slice() });
  {
    const c0 = centreOf(T[0]), c1 = centreOf(T[1]), b = sub(c1, c0), L = Math.hypot(...b);
    const dir = mv(rodrigues(scl(randAxis(r), 2 * deg * perturb)), scl(b, 1 / L));
    cams.set(1, camFrom(perturbR(T[1].w, 1 * deg * perturb, r), add(c0, scl(dir, L))));
  }
  let added = triangulateNew();
  snap('initial pair: cameras 1 and 2 (E → R, t), triangulate', 'pair', 1, { newPts: new Set(added) });
  let res = optimise({ cams, pts, obs, free: [gauge(1)], fixed: [0], ptIds: [...pts.keys()], pointsFree: true });
  snap('bundle adjustment: the pair', 'ba', 1, { before: Math.sqrt(res.before / res.n), after: Math.sqrt(res.after / res.n) });
  for (let k = 2; k < NCAM; k++) {
    // register camera k: PnP against the existing points it observes
    let known = [...obs[k].keys()].filter(i => pts.has(i));
    if (pnpPoints > 0 && pnpPoints < known.length) {   // a random handful of them
      for (let i = known.length - 1; i > 0; i--) { const j = Math.floor(r() * (i + 1)); [known[i], known[j]] = [known[j], known[i]]; }
      known = known.slice(0, pnpPoints);
    }
    const c = centreOf(T[k]);
    cams.set(k, camFrom(perturbR(T[k].w, 3 * deg * perturb, r), add(c, scl(randAxis(r), 0.3 * perturb))));
    optimise({ cams, pts, obs, free: [{ k, params: [0, 1, 2, 3, 4, 5] }], fixed: [], ptIds: known, pointsFree: false });
    snap(`register camera ${k + 1}: PnP on ${known.length} known points`, 'pnp', k, { rays: known });
    added = triangulateNew();
    snap(`triangulate ${added.length} new points (seen by ≥ 2 cameras)`, 'tri', k, { newPts: new Set(added) });
    if (baEvery > 1 && (k + 1) % baEvery !== 0 && k < NCAM - 1) continue;
    const global = baEvery > 1 || (k + 1) % 3 === 0;
    const freeKs = global ? [...cams.keys()] : [k - 2, k - 1, k].filter(j => cams.has(j));
    const free = freeKs.map(gauge).filter(Boolean), fixed = [...cams.keys()].filter(j => !free.some(f => f.k === j));
    const ptIds = [...pts.keys()].filter(i => freeKs.some(j => obs[j].has(i)));
    res = optimise({ cams, pts, obs, free, fixed, ptIds, pointsFree: true, iters: global ? 20 : 10 });
    snap(global ? `global bundle adjustment: all ${cams.size} cameras, ${pts.size} points` : `local bundle adjustment: cameras ${freeKs[0] + 1} to ${k + 1}, ${ptIds.length} points`,
      'ba', k, { before: Math.sqrt(res.before / res.n), after: Math.sqrt(res.after / res.n), baCams: new Set(freeKs) });
  }
  return snaps;
}

// ---------- three.js drawing helpers ----------
function disposeGroup(g) {
  g.traverse(o => { if (o.geometry) o.geometry.dispose(); if (o.material) { if (o.material.map) o.material.map.dispose(); o.material.dispose(); } });
  g.clear();
}
function segments(pairs, color, opacity = 1) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(pairs.flat(2), 3));
  return new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color, transparent: opacity < 1, opacity }));
}
function frustumSegments(cam) {
  const R = rodrigues(cam.w), c = cam.c || centreOf(cam);
  const corner = (u, v) => add(c, mt(R, [(u - OX) / F * S, (v - OY) / F * S, S]));
  const cs = [corner(0, 0), corner(IW, 0), corner(IW, IH), corner(0, IH)];
  return [...cs.map(p => [c, p]), [cs[0], cs[1]], [cs[1], cs[2]], [cs[2], cs[3]], [cs[3], cs[0]]];
}
function houseSegments() {
  const seg = [];
  const box = (lo, hi) => {
    const v = (i, j, k) => [i ? hi[0] : lo[0], j ? hi[1] : lo[1], k ? hi[2] : lo[2]];
    for (const [a, b] of [[0, 1], [1, 3], [3, 2], [2, 0], [4, 5], [5, 7], [7, 6], [6, 4], [0, 4], [1, 5], [2, 6], [3, 7]]) seg.push([v(a & 1, (a >> 1) & 1, a >> 2), v(b & 1, (b >> 1) & 1, b >> 2)]);
  };
  box([-HW, 0, -HD], [HW, HH, HD]);
  seg.push([[-HW, RY, 0], [HW, RY, 0]]);
  for (const sx of [-1, 1]) for (const sz of [-1, 1]) seg.push([[sx * HW, HH, sz * HD], [sx * HW, RY, 0]]);
  const rect = (pts) => { for (let i = 0; i < 4; i++) seg.push([pts[i], pts[(i + 1) % 4]]); };
  rect([[-0.4, 0, HD], [0.4, 0, HD], [0.4, 2, HD], [-0.4, 2, HD]]);
  for (const x0 of [-1.7, 0.9]) for (const z of [HD, -HD]) rect([[x0, 1, z], [x0 + 0.8, 1, z], [x0 + 0.8, 1.8, z], [x0, 1.8, z]]);
  for (const x of [HW, -HW]) rect([[x, 1, -0.4], [x, 1, 0.4], [x, 1.8, 0.4], [x, 1.8, -0.4]]);
  return seg;
}
function dotTexture() {
  const c = document.createElement('canvas'); c.width = c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = '#fff'; g.beginPath(); g.arc(32, 32, 28, 0, 2 * Math.PI); g.fill();
  const t = new THREE.CanvasTexture(c); return t;
}

export async function mount(el, cfg) {
  const print = isPrint();
  const W = cfg.width || 660, H = cfg.height || 400, IWD = cfg.imageWidth || 380;
  const seed = cfg.seed ?? 3, RS = cfg.residualScale ?? 10, GLIDE = cfg.glide ?? 0.6;
  const scene = makeScene(seed), obs = observe(scene, seed, cfg.noise ?? SIGMA);
  const snaps = runPipeline(scene, obs, seed, cfg.perturb ?? 1, cfg.pnpPoints ?? 0, cfg.baEvery ?? 1);
  const LAST = snaps.length - 1;
  const state = {
    step: Math.max(0, Math.min(LAST, cfg.step ?? (print ? LAST : 0))),
    truth: cfg.truth ?? true, playing: false, speed: 1, wait: 0, morph: 1, morphing: false,
  };

  // ---- 3D stage
  const canvas = h('canvas', { width: W, height: H });
  const view3d = { label: '3D view', position: [8, 10, 16.5], target: [0, 0.6, 0], fov: 40 };
  const viewTop = { label: 'top view', position: [0, 27, 0.01], target: [0, 0, 0], fov: 40 };
  const stage = makeStage(canvas, { position: view3d.position, target: view3d.target, fov: view3d.fov });
  const { scene: sc, render } = stage;
  const grid = new THREE.GridHelper(26, 26, 0x3a3f4b, 0x2a2e36); grid.position.y = -0.01; sc.add(grid);
  const truthG = new THREE.Group(); sc.add(truthG);
  truthG.add(segments(houseSegments(), 0x5a606c, 0.9));
  scene.cams.forEach(cam => truthG.add(segments(frustumSegments(cam), 0x4a505c, 0.7)));
  const dyn = new THREE.Group(); sc.add(dyn);
  const dotTex = dotTexture();

  // the displayed estimate: snapshot `step`, blended from the previous one after a bundle adjustment
  function displayed() {
    const s = snaps[state.step], m = state.morph, prev = snaps[state.step - 1];
    if (s.kind !== 'ba' || m >= 1 || !prev) return s;
    const e = m * m * (3 - 2 * m);
    const cams = new Map([...s.cams].map(([k, c]) => {
      const p = prev.cams.get(k); if (!p) return [k, c];
      const cc = lerp3(p.c, c.c, e), w = lerp3(p.w, c.w, e);
      return [k, { ...camFrom(w, cc), c: cc }];
    }));
    const pts = new Map([...s.pts].map(([i, X]) => [i, prev.pts.has(i) ? lerp3(prev.pts.get(i), X, e) : X]));
    return { ...s, cams, pts };
  }

  function build() {
    const d = displayed(), s = snaps[state.step];
    disposeGroup(dyn);
    truthG.visible = state.truth;
    // points
    const ids = [...d.pts.keys()];
    if (ids.length) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(ids.flatMap(i => d.pts.get(i)), 3));
      const col = new THREE.Color(), cols = [];
      ids.forEach(i => { col.set(s.newPts.has(i) ? C.accent : C.fg); cols.push(col.r, col.g, col.b); });
      geo.setAttribute('color', new THREE.Float32BufferAttribute(cols, 3));
      dyn.add(new THREE.Points(geo, new THREE.PointsMaterial({ size: 0.32, vertexColors: true, map: dotTex, alphaTest: 0.5, transparent: true, sizeAttenuation: true })));
    }
    // cameras
    for (const [k, cam] of d.cams) {
      const current = k === s.cur && (s.kind === 'pnp' || s.kind === 'pair');
      const inBA = s.kind === 'ba' && (!s.baCams || s.baCams.has(k));
      const color = current ? 0x5fd38d : inBA ? 0xf2b134 : 0x5ab0ff;
      dyn.add(segments(frustumSegments(cam), color));
      const l = label(String(k + 1), { color: '#' + color.toString(16).padStart(6, '0'), size: 0.9, font: 'bold 64px Inter, Arial, sans-serif' });
      l.position.set(cam.c[0], cam.c[1] + 0.55, cam.c[2]); dyn.add(l);
    }
    // rays from the camera being registered to the points it uses
    if (s.kind === 'pnp' && s.rays.length) {
      const cam = d.cams.get(s.cur);
      dyn.add(segments(s.rays.filter(i => d.pts.has(i)).map(i => [cam.c, d.pts.get(i)]), 0x5fd38d, 0.35));
    }
    render();
    drawImage(d, s);
    drawReadout(d, s);
  }

  // ---- the image of the current camera
  const SC = IWD / IW;
  const img = h('canvas', { width: IW * 2, height: IH * 2, style: `width:${IWD}px; height:${Math.round(IH * SC)}px; border:1px solid #3a3f4b; border-radius:4px; display:block` });
  function drawImage(d, s) {
    const g = img.getContext('2d'), k = s.cur, K = 2;
    g.fillStyle = '#0d0f13'; g.fillRect(0, 0, img.width, img.height);
    g.strokeStyle = '#2a2e36'; g.lineWidth = 1.5;
    for (let u = 80; u < IW; u += 80) { g.beginPath(); g.moveTo(u * K, 0); g.lineTo(u * K, img.height); g.stroke(); }
    for (let v = 80; v < IH; v += 80) { g.beginPath(); g.moveTo(0, v * K); g.lineTo(img.width, v * K); g.stroke(); }
    const cam = d.cams.get(k);
    let n = 0, m = 0;
    for (const [i, uv] of obs[k]) {
      const has = d.pts.has(i);
      g.fillStyle = has ? C.fg : '#5a606c';
      g.beginPath(); g.arc(uv[0] * K, uv[1] * K, has ? 5 : 4, 0, 2 * Math.PI); g.fill();
      m++;
      if (!has || !cam) continue;
      const p = projectR(rodrigues(cam.w), cam.t, d.pts.get(i));
      const ex = (p[0] - uv[0]) * RS, ey = (p[1] - uv[1]) * RS;
      const x = uv[0] + ex, y = uv[1] + ey;
      g.strokeStyle = C.red; g.lineWidth = 2.5;
      g.beginPath(); g.moveTo(uv[0] * K, uv[1] * K); g.lineTo(x * K, y * K); g.stroke();
      g.beginPath(); g.moveTo(x * K - 7, y * K - 7); g.lineTo(x * K + 7, y * K + 7); g.moveTo(x * K - 7, y * K + 7); g.lineTo(x * K + 7, y * K - 7); g.stroke();
      n++;
    }
    g.font = '30px Inter, Arial, sans-serif'; g.fillStyle = C.fg;
    g.fillText(`image ${k + 1}`, 14, 38);
    g.font = '24px Inter, Arial, sans-serif'; g.fillStyle = C.dim;
    g.fillText(`${m} features, ${n} with a 3D point`, 14, img.height - 44);
    g.fillStyle = C.red; g.fillText(`× reprojection, residual drawn ×${RS}`, 14, img.height - 14);
  }

  // ---- readout
  const readout = h('div', { class: 'readout', style: `width:${IWD}px; line-height:1.35; font-size:0.8em` });
  function drawReadout(d, s) {
    const nc = d.cams.size, np = d.pts.size;
    let err;
    if (s.kind === 'ba') err = `RMS reprojection error ${fmt(s.before, 2)} px → <b style="color:${C.green}">${fmt(s.after, 2)} px</b>`;
    else err = `RMS reprojection error <b>${fmt(s.err, 2)} px</b>`;
    readout.innerHTML =
      `<div style="color:${C.fg}">${state.step + 1}/${snaps.length} <b style="color:${C.accent}">${s.name}</b></div>` +
      `<div>cameras <b>${nc}</b>/${NCAM} · points <b>${np}</b> · observations <b>${s.nObs}</b></div>` +
      `<div>unknowns 6·${nc} + 3·${np} = <b>${6 * nc + 3 * np}</b> (7 fixed: gauge), equations 2·${s.nObs} = <b>${2 * s.nObs}</b></div>` +
      `<div>${err}</div>`;
  }

  // ---- stepping and animation
  function goto(n, animate) {
    n = Math.max(0, Math.min(LAST, n));
    state.step = n;
    const s = snaps[n];
    if (animate && !print && s.kind === 'ba') { state.morph = 0; state.morphing = true; kick(); } else state.morph = 1;
    syncControls(); build();
  }
  let raf = null, last = null;
  const shown = () => canvas.closest('section')?.classList.contains('present') !== false;
  function tick(now) {
    raf = null;
    if (!canvas.isConnected) return;
    const dt = last == null ? 0 : Math.min(0.1, (now - last) / 1000);
    last = now;
    if (shown()) {
      let changed = false;
      if (state.morphing) { state.morph = Math.min(1, state.morph + dt / GLIDE); if (state.morph >= 1) state.morphing = false; changed = true; }
      if (state.playing && !state.morphing) {
        state.wait -= dt * state.speed;
        if (state.wait <= 0) {
          if (state.step >= LAST) { state.playing = false; syncControls(); }
          else { state.wait = 1.2; goto(state.step + 1, true); changed = false; }
        }
      }
      if (changed) build();
    }
    if (state.playing || state.morphing) raf = requestAnimationFrame(tick);
  }
  const kick = () => { if (!raf && !print) { last = null; raf = requestAnimationFrame(tick); } };
  function play() { if (state.step >= LAST) goto(0, false); state.playing = true; state.wait = 1.2; syncControls(); kick(); }
  function pause() { state.playing = false; syncControls(); }

  // ---- controls
  const playBtn = h('button', { style: 'width:5.6em', onclick: () => (state.playing ? pause() : play()) }, '▶ play');
  const nextBtn = h('button', { onclick: () => { pause(); goto(state.step + 1, true); } }, 'next step');
  const restartBtn = h('button', { onclick: () => { pause(); goto(0, false); } }, '↺ restart');
  const scrub = h('input', { type: 'range', min: 0, max: LAST, step: 1, value: state.step, style: 'width:10em' });
  scrub.addEventListener('input', () => { pause(); goto(parseInt(scrub.value, 10), false); });
  const speeds = [1, 2, 4].map(v => h('button', { onclick: () => { state.speed = v; syncControls(); } }, `${v}×`));
  const truthBtn = h('button', { onclick: () => { state.truth = !state.truth; syncControls(); build(); } }, 'truth');
  function syncControls() {
    playBtn.textContent = state.playing ? '⏸ pause' : '▶ play';
    scrub.value = state.step;
    speeds.forEach((b, i) => b.classList.toggle('active', [1, 2, 4][i] === state.speed));
    truthBtn.classList.toggle('active', state.truth);
  }

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${W}px; flex:none` }, canvas,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, playBtn, nextBtn, restartBtn, h('label', { style: 'margin-left:0.3em' }, 'speed'), ...speeds),
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, scrub, ...viewButtons(stage, [view3d, viewTop]), truthBtn)),
    h('div', { style: `display:flex; flex-direction:column; gap:0.4em; width:${IWD}px; flex:none` }, img, readout)));
  syncControls();
  build();
  if (print) {
    // The deck replaces the canvas by an image sized from its clientWidth a few frames after mount. reveal builds the
    // print layout asynchronously after initialize (the viewport carries "loading-scroll-mode" until its "pdf-ready"),
    // and a widget mounted meanwhile is frozen while its slide is being moved: clientWidth 0, an invisible image.
    // So the first widget of a deck waits here for the print layout.
    stage.renderer.render(stage.scene, stage.camera);
    const t0 = performance.now(), vp = document.querySelector('.reveal-viewport');
    while (vp?.classList.contains('loading-scroll-mode') && performance.now() - t0 < 20000) await new Promise(r => requestAnimationFrame(r));
  } else if (cfg.step == null && (cfg.autoplay ?? true)) play();
}
