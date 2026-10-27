// Bundle adjustment sandbox in 2D. Cameras with a one-dimensional image (pose x, y, θ; focal length f = 300 px,
// principal point o = 200, image width 400 px, FOV ≈ 67°) stand on an arc and look at a cluster of points. Every
// point in front of a camera and inside its image gives one observation u = f·x_c/z_c + o (+ Gaussian noise;
// x_c to the right, z_c forward along the heading). Unknowns: all camera poses and all points except the gauge:
// camera 0 is fixed (the frame) and the x of camera 1 (the scale): 3J + 2N − 4 numbers, the 4 being the similarity
// freedom of the plane (rotate, translate, scale) that the images cannot see.
// The estimate starts from the truth perturbed (cameras σ 0.3 m, 5°; points σ 0.4 m; times `perturb`) and is
// refined by Gauss-Newton: residual r = predicted − observed pixel, analytic Jacobian, normal equations
// JᵀJ Δ = −Jᵀr solved by dense Cholesky, one step per click; optional Levenberg-Marquardt damping
// (JᵀJ + λ diag JᵀJ, accepted step: λ/3, rejected: λ×4). Optional outliers (10% of the observations shifted by
// 20 … 60 px, as wrong matches that survived RANSAC would) and a Huber loss (iteratively reweighted least squares,
// k = 2.5·max(1, σ) px). Without the gauge fix (toggle) JᵀJ is singular: plain Gauss-Newton stops, LM converges
// but the whole model floats away from the truth.
// Panels (config `panels`): "world" (true cameras and points faint, estimates in colour, rays), "images" (one
// 1D image per camera with the observed and the predicted pixel of every point: the residuals), "structure" (the
// sparsity pattern of J, rows = observations grouped by camera, columns = camera then point parameters, and of
// H = JᵀJ with its blocks U cameras, V points, W coupling; the gauge columns grey), "plot" (log10 cost per iteration).
// config: {
//   width: 520, height: 320     world panel (px); the images strip below it has the same width
//   rightWidth: 400             width of the right column (structure, plot)
//   panels: ["world","images","structure","plot"]
//   cams: 5, points: 14         J cameras (3 … 8), N points (6 … 24)
//   noise: 0.5                  pixel noise σ (slider 0 … 4)
//   seed: 1
//   perturb: 0.7                scale of the initial perturbation (1: cameras σ 0.3 m, 5°; points σ 0.4 m)
//   solved: false               start from the converged state (print: true)
//   lm: false                   Levenberg-Marquardt damping
//   gauge: true                 fix camera 0 and the scale
//   outliers: false, robust: false
//   simple: false               only step / solve / reset / perturb, and a short readout (no LM, gauge, outlier, Huber)
// }
// Toggling outliers / Huber keeps the current estimate (bundle adjustment always starts from a good initial
// estimate; the question is whether the outliers pull it away): e.g. seed 5, perturb 0.3: plain least squares ends
// 2.5 m from the truth, Huber 16 cm, no outliers 12 cm. Planar bearing-only reconstruction is weakly conditioned:
// even without outliers 0.5 px of noise leaves 2 … 30 cm of error (bending along the arc), 1 px twice that.
// createProblem(opts) is exported for tests (no DOM).
import { C, h, fmt, isPrint } from './util.js';
import { rng } from './linalg.js';

const F = 300, O = 200, IW = 400, HF = Math.atan(O / F);
const deg = Math.PI / 180;
const MAX_ITER = 60;
const rgba = (hex, a) => `rgba(${parseInt(hex.slice(1, 3), 16)},${parseInt(hex.slice(3, 5), 16)},${parseInt(hex.slice(5, 7), 16)},${a})`;

// ---- the world: cameras on a 110° arc of radius 4.5 m looking at points spread over 6 × 5 m around the origin ----
function makeTruth(J, N, seed, R = 4.5, SX = 6, SY = 5) {
  const r = rng(seed * 1009 + 17);
  const cams = [], spread = 110 * deg;
  for (let j = 0; j < J; j++) {
    const a = -Math.PI / 2 + spread * (J > 1 ? j / (J - 1) - 0.5 : 0);
    const x = R * Math.cos(a), y = R * Math.sin(a);
    cams.push([x, y, Math.atan2(-y, -x) + (r() - 0.5) * 14 * deg]);
  }
  // every point must be seen by at least two cameras (else its depth is free and JᵀJ singular): resample otherwise
  const seen = p => cams.filter(c => { const [u, , zc] = project(c, p); return zc > 0.5 && u >= 0 && u <= IW; }).length;
  const pts = [];
  for (let i = 0; i < N; i++) {
    let p;
    for (let t = 0; t < 100; t++) { p = [(r() - 0.5) * SX, (r() - 0.5) * SY]; if (seen(p) >= 2) break; }
    pts.push(p);
  }
  return { cams, pts };
}

export function project(cam, p) {
  const dx = p[0] - cam[0], dy = p[1] - cam[1], c = Math.cos(cam[2]), s = Math.sin(cam[2]);
  const zc = dx * c + dy * s, xc = dx * s - dy * c;
  return [F * xc / zc + O, xc, zc];
}
// ∂u/∂(x, y, θ) of the camera and ∂u/∂(px, py) of the point
export function jac(cam, p) {
  const dx = p[0] - cam[0], dy = p[1] - cam[1], c = Math.cos(cam[2]), s = Math.sin(cam[2]);
  const zc = dx * c + dy * s, xc = dx * s - dy * c, z2 = zc * zc;
  const dpx = F * (s * zc - xc * c) / z2, dpy = F * (-c * zc - xc * s) / z2;
  return [-dpx, -dpy, F * (z2 + xc * xc) / z2, dpx, dpy];
}

// visible (camera, point) pairs with their exact pixel, a unit normal draw and an outlier offset
function makeObs(truth, seed) {
  const r = rng(seed * 7919 + 3), obs = [];
  truth.cams.forEach((cam, j) => truth.pts.forEach((p, i) => {
    const [u, , zc] = project(cam, p);
    const n = r.normal(), out = r() < 0.10 ? (r() < 0.5 ? -1 : 1) * (20 + 40 * r()) : 0;
    if (zc > 0.5 && u >= 0 && u <= IW) obs.push({ j, i, u0: u, n, out });
  }));
  return obs;
}

// Cholesky solve of the normal equations; null when the matrix is (numerically) singular
function cholSolve(A, b, n) {
  const L = Float64Array.from(A);
  let mx = 0;
  for (let i = 0; i < n; i++) mx = Math.max(mx, L[i * n + i]);
  for (let j = 0; j < n; j++) {
    let d = L[j * n + j];
    for (let k = 0; k < j; k++) d -= L[j * n + k] ** 2;
    if (d <= 1e-10 * mx) return null;
    d = Math.sqrt(d); L[j * n + j] = d;
    for (let i = j + 1; i < n; i++) {
      let s = L[i * n + j];
      for (let k = 0; k < j; k++) s -= L[i * n + k] * L[j * n + k];
      L[i * n + j] = s / d;
    }
  }
  const y = new Float64Array(n);
  for (let i = 0; i < n; i++) { let s = b[i]; for (let k = 0; k < i; k++) s -= L[i * n + k] * y[k]; y[i] = s / L[i * n + i]; }
  for (let i = n - 1; i >= 0; i--) { let s = y[i]; for (let k = i + 1; k < n; k++) s -= L[k * n + i] * y[k]; y[i] = s / L[i * n + i]; }
  return y;
}

// The problem and its solver, without any DOM: state (settings, iteration, history), truth, observations, the
// parameter vector q = [x_0, y_0, θ_0, …, x_J−1, y_J−1, θ_J−1, px_0, py_0, …] and one Gauss-Newton / LM step.
export function createProblem(opts = {}) {
  const state = {
    J: opts.cams ?? 5, N: opts.points ?? 14, noise: opts.noise ?? 0.5, seed: opts.seed ?? 1, seedP: 0, perturb: opts.perturb ?? 0.7,
    lm: !!opts.lm, gauge: opts.gauge ?? true, outliers: !!opts.outliers, robust: !!opts.robust,
    lambda: 1e-3, iter: 0, hist: [], converged: false, singular: false, rejected: 0,
  };
  const P = { state };
  const huberK = () => 2.5 * Math.max(1, state.noise);
  P.measured = o => o.u0 + state.noise * o.n + (state.outliers ? o.out : 0);
  P.setup = function (keepTruth) {
    if (!keepTruth) { P.truth = makeTruth(state.J, state.N, state.seed, opts.radius, opts.spreadX, opts.spreadY); P.obs = makeObs(P.truth, state.seed); }
    const n = 3 * state.J + 2 * state.N;
    const fixed = state.gauge ? new Set([0, 1, 2, 3]) : new Set();
    P.free = []; P.freeIdx = new Array(n).fill(-1);
    for (let c = 0; c < n; c++) if (!fixed.has(c)) { P.freeIdx[c] = P.free.length; P.free.push(c); }
    // the perturbed start: everything but the gauge-fixed numbers
    const r = rng(state.seed * 131 + state.seedP * 17 + 5);
    P.q0 = [...P.truth.cams.flat(), ...P.truth.pts.flat()];
    for (let c = 0; c < n; c++) {
      const v = r.normal() * state.perturb;
      if (fixed.has(c)) continue;
      if (c < 3 * state.J) P.q0[c] += (c % 3 === 2 ? 5 * deg : 0.3) * v; else P.q0[c] += 0.4 * v;
    }
    P.reset();
  };
  P.reset = function () { P.q = P.q0.slice(); P.restart(); };
  // keep the current estimate, start counting iterations afresh (after a change of the loss or the data)
  P.restart = function () {
    state.iter = 0; state.converged = false; state.singular = false;
    state.lambda = 1e-3; state.rejected = 0; state.hist = [P.cost(P.q)];
  };
  P.cam = (q, j) => [q[3 * j], q[3 * j + 1], q[3 * j + 2]];
  P.pt = (q, i) => [q[3 * state.J + 2 * i], q[3 * state.J + 2 * i + 1]];
  P.residuals = q => P.obs.map(o => project(P.cam(q, o.j), P.pt(q, o.i))[0] - P.measured(o));
  const rho = r => { if (!state.robust) return r * r; const k = huberK(); return Math.abs(r) <= k ? r * r : 2 * k * Math.abs(r) - k * k; };
  const weight = r => { if (!state.robust) return 1; const k = huberK(); return Math.abs(r) <= k ? 1 : k / Math.abs(r); };
  P.cost = q => P.residuals(q).reduce((s, r) => s + rho(r), 0);
  P.rms = q => { const r = P.residuals(q); return Math.sqrt(r.reduce((s, v) => s + v * v, 0) / Math.max(1, r.length)); };
  // RMS error of the estimated camera positions and points against the truth (metres)
  P.errTruth = q => {
    let s = 0, n = 0;
    for (let j = 0; j < state.J; j++) { s += (q[3 * j] - P.truth.cams[j][0]) ** 2 + (q[3 * j + 1] - P.truth.cams[j][1]) ** 2; n++; }
    for (let i = 0; i < state.N; i++) { const p = P.pt(q, i); s += (p[0] - P.truth.pts[i][0]) ** 2 + (p[1] - P.truth.pts[i][1]) ** 2; n++; }
    return Math.sqrt(s / n);
  };

  // one Gauss-Newton (or LM) step; false when nothing could be done
  P.step = function () {
    if (state.converged || state.singular) return false;
    const q = P.q, r = P.residuals(q), n = P.free.length, J = state.J;
    const rows = P.obs.map(o => { const d = jac(P.cam(q, o.j), P.pt(q, o.i)); const cj = 3 * o.j, ci = 3 * J + 2 * o.i; return [[cj, d[0]], [cj + 1, d[1]], [cj + 2, d[2]], [ci, d[3]], [ci + 1, d[4]]]; });
    const Hm = new Float64Array(n * n), g = new Float64Array(n);
    rows.forEach((row, k) => {
      const w = weight(r[k]);
      for (const [ca, va] of row) {
        const fa = P.freeIdx[ca]; if (fa < 0) continue;
        g[fa] += w * va * r[k];
        for (const [cb, vb] of row) { const fb = P.freeIdx[cb]; if (fb >= 0) Hm[fa * n + fb] += w * va * vb; }
      }
    });
    const c0 = state.hist[state.hist.length - 1];
    for (let attempt = 0; attempt < 12; attempt++) {
      const A = Float64Array.from(Hm);
      if (state.lm) for (let i = 0; i < n; i++) A[i * n + i] *= 1 + state.lambda;
      const d = cholSolve(A, g, n);
      if (!d) { if (!state.lm) { state.singular = true; return false; } state.lambda *= 4; continue; }
      const qn = q.slice();
      P.free.forEach((c, a) => { qn[c] -= d[a]; });
      const c1 = P.cost(qn);
      if (state.lm && c1 > c0) { state.lambda *= 4; state.rejected++; continue; }
      let mx = 0; for (const v of d) mx = Math.max(mx, Math.abs(v));
      P.q = qn; state.iter++; state.hist.push(c1);
      if (state.lm) state.lambda = Math.max(1e-9, state.lambda / 3);
      if (Math.abs(c0 - c1) < 1e-9 * c0 + 1e-12 || mx < 1e-9 || state.iter >= MAX_ITER) state.converged = true;
      return true;
    }
    state.converged = true; // LM could not reduce the cost any further
    return false;
  };
  P.solveAll = function () { while (!state.converged && !state.singular && state.iter < MAX_ITER) if (!P.step()) break; };
  P.setup(false);
  return P;
}

export function mount(el, cfg) {
  const print = isPrint();
  const W = cfg.width || 520, H = cfg.height || 320, RW = cfg.rightWidth || 400;
  const panels = cfg.panels || ['world', 'images', 'structure', 'plot'];
  const has = p => panels.includes(p);
  const prob = createProblem(cfg), state = prob.state;
  let solving = false;

  // ---- canvases ----
  const imgH = () => 14 * state.J + 24;
  const world = h('canvas', { width: W, height: H });
  const images = h('canvas', { width: W, height: imgH() });
  const SH = has('plot') ? H - 112 - 8 : H, PH = has('plot') ? 112 : 0;
  const structure = h('canvas', { width: RW, height: SH });
  const plot = h('canvas', { width: RW, height: PH });
  const readout = h('div', { class: 'readout', style: 'font-size:0.72em; line-height:1.3' });

  // world → canvas
  let X, Y, sc;
  function fitWorld() {
    const { cams, pts } = prob.truth;
    const xs = [...cams.map(c => c[0]), ...pts.map(p => p[0])], ys = [...cams.map(c => c[1]), ...pts.map(p => p[1])];
    const m = 1.1, x0 = Math.min(...xs) - m, x1 = Math.max(...xs) + m, y0 = Math.min(...ys) - m, y1 = Math.max(...ys) + m;
    sc = Math.min(W / (x1 - x0), H / (y1 - y0));
    const ox = (W - (x1 - x0) * sc) / 2, oy = (H - (y1 - y0) * sc) / 2;
    X = x => ox + (x - x0) * sc; Y = y => H - oy - (y - y0) * sc;
  }
  function wedge(g, cam, L) {
    const [x, y, th] = cam;
    g.beginPath(); g.moveTo(X(x), Y(y));
    g.lineTo(X(x + L * Math.cos(th + HF)), Y(y + L * Math.sin(th + HF)));
    g.lineTo(X(x + L * Math.cos(th - HF)), Y(y + L * Math.sin(th - HF)));
    g.closePath();
  }
  function drawWorld() {
    const g = world.getContext('2d'), q = prob.q, { cams, pts } = prob.truth;
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    g.font = '12px Inter, Arial, sans-serif';
    // scale bar
    g.strokeStyle = C.dim; g.lineWidth = 2; g.beginPath(); g.moveTo(12, H - 12); g.lineTo(12 + sc, H - 12); g.stroke();
    g.fillStyle = C.dim; g.textAlign = 'left'; g.fillText('1 m', 16 + sc, H - 8);
    // rays from the estimated cameras through the estimated points
    g.strokeStyle = rgba(C.dim, 0.22); g.lineWidth = 1;
    for (const o of prob.obs) { const c = prob.cam(q, o.j), p = prob.pt(q, o.i); g.beginPath(); g.moveTo(X(c[0]), Y(c[1])); g.lineTo(X(p[0]), Y(p[1])); g.stroke(); }
    // truth: faint
    g.setLineDash([4, 3]); g.strokeStyle = rgba(C.fg, 0.45); g.lineWidth = 1.2;
    cams.forEach(c => { wedge(g, c, 1.0); g.stroke(); });
    g.setLineDash([]);
    g.strokeStyle = rgba(C.fg, 0.5); g.lineWidth = 1.2;
    pts.forEach(p => { const x = X(p[0]), y = Y(p[1]); g.beginPath(); g.moveTo(x - 4, y - 4); g.lineTo(x + 4, y + 4); g.moveTo(x - 4, y + 4); g.lineTo(x + 4, y - 4); g.stroke(); });
    // estimates
    for (let j = 0; j < state.J; j++) {
      const c = prob.cam(q, j);
      wedge(g, c, 1.0); g.fillStyle = rgba(C.blue, 0.32); g.fill(); g.strokeStyle = C.blue; g.lineWidth = 1.6; g.stroke();
      g.fillStyle = C.blue; g.textAlign = 'center'; g.font = 'bold 12px Inter, Arial, sans-serif';
      const bx = X(c[0] - 0.45 * Math.cos(c[2])), by = Y(c[1] - 0.45 * Math.sin(c[2]));
      g.fillText(j, bx, by + 4);
      if (state.gauge && j < 2) {
        g.font = '11px Inter, Arial, sans-serif'; g.fillStyle = C.accent;
        const t = j === 0 ? 'fixed: the frame' : 'fixed: the scale', tw = g.measureText(t).width;
        g.fillText(t, Math.min(W - tw / 2 - 4, Math.max(tw / 2 + 4, bx)), Math.min(H - 20, by + 18));
      }
    }
    for (let i = 0; i < state.N; i++) { const p = prob.pt(q, i); g.fillStyle = C.red; g.beginPath(); g.arc(X(p[0]), Y(p[1]), 3.6, 0, 7); g.fill(); }
    g.font = '12px Inter, Arial, sans-serif'; g.textAlign = 'left';
    g.fillStyle = C.dim; g.fillText('╌ × truth', 12, 16);
    g.fillStyle = C.blue; g.fillText('▲ cameras', 90, 16);
    g.fillStyle = C.red; g.fillText('● points (estimates)', 170, 16);
    g.strokeStyle = C.line; g.lineWidth = 1; g.strokeRect(0.5, 0.5, W - 1, H - 1);
  }

  function drawImages() {
    const Hh = imgH();
    if (images.height !== Hh) images.height = Hh;
    const g = images.getContext('2d');
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, Hh);
    const x0 = 52, x1 = W - 10, px = u => x0 + Math.min(IW, Math.max(0, u)) / IW * (x1 - x0);
    g.font = '11px Inter, Arial, sans-serif'; g.textAlign = 'left';
    g.fillStyle = C.dim; g.fillText('1D images: observed pixel (tick up) vs predicted (tick down)', x0, 11);
    const r = prob.residuals(prob.q);
    for (let j = 0; j < state.J; j++) {
      const y = 24 + 14 * j;
      g.fillStyle = C.dim; g.fillText('cam ' + j, 8, y + 4);
      g.strokeStyle = C.line; g.lineWidth = 1; g.beginPath(); g.moveTo(x0, y); g.lineTo(x1, y); g.stroke();
      prob.obs.forEach((o, k) => {
        if (o.j !== j) return;
        const uo = prob.measured(o), up = uo + r[k];
        g.strokeStyle = rgba(C.red, 0.8); g.lineWidth = 2; g.beginPath(); g.moveTo(px(uo), y); g.lineTo(px(up), y); g.stroke();
        g.strokeStyle = C.fg; g.lineWidth = 1.2; g.beginPath(); g.moveTo(px(uo), y - 5); g.lineTo(px(uo), y); g.stroke();
        g.strokeStyle = C.red; g.beginPath(); g.moveTo(px(up), y); g.lineTo(px(up), y + 5); g.stroke();
      });
    }
    g.strokeStyle = C.line; g.strokeRect(0.5, 0.5, W - 1, Hh - 1);
  }

  function drawStructure() {
    const g = structure.getContext('2d'), Wc = structure.width, Hc = structure.height;
    g.fillStyle = C.bg2; g.fillRect(0, 0, Wc, Hc);
    const J = state.J, N = state.N, Pn = 3 * J + 2 * N, M = prob.obs.length, obs = prob.obs;
    const pad = 10, top = 22, bottom = 30;
    const availH = Hc - top - bottom, availW = Wc - 3 * pad;
    const cH = Math.min(availH, availW * 0.56) / Pn;
    const cJ = Math.max(0.4, Math.min(availH / M, (availW - cH * Pn - 14) / Pn));
    const gaugeCol = c => state.gauge && c < 4;
    const colColor = c => (gaugeCol(c) ? rgba(C.dim, 0.55) : c < 3 * J ? C.blue : C.red);
    // --- J pattern
    const jx = pad, jy = top, jw = cJ * Pn, jh = cJ * M;
    g.font = '12px Inter, Arial, sans-serif'; g.textAlign = 'left'; g.fillStyle = C.fg;
    g.fillText(`J: ${M} × ${Pn}`, jx, 14);
    g.fillStyle = C.bg3; g.fillRect(jx, jy, jw, jh);
    if (state.gauge) { g.fillStyle = rgba(C.dim, 0.18); g.fillRect(jx, jy, 4 * cJ, jh); }
    obs.forEach((o, k) => {
      const y = jy + k * cJ, hh = Math.max(1, cJ);
      for (let a = 0; a < 3; a++) { g.fillStyle = colColor(3 * o.j + a); g.fillRect(jx + (3 * o.j + a) * cJ, y, Math.max(1, cJ), hh); }
      for (let a = 0; a < 2; a++) { g.fillStyle = colColor(3 * J + 2 * o.i + a); g.fillRect(jx + (3 * J + 2 * o.i + a) * cJ, y, Math.max(1, cJ), hh); }
    });
    g.strokeStyle = C.line; g.lineWidth = 1; g.strokeRect(jx + 0.5, jy + 0.5, jw, jh);
    // --- H pattern
    const hx = jx + jw + 14 + pad, hy = top, hs = cH * Pn;
    g.fillStyle = C.fg; g.fillText(`H = JᵀJ: ${Pn} × ${Pn}`, hx, 14);
    g.fillStyle = C.bg3; g.fillRect(hx, hy, hs, hs);
    if (state.gauge) { g.fillStyle = rgba(C.dim, 0.18); g.fillRect(hx, hy, 4 * cH, hs); g.fillRect(hx, hy, hs, 4 * cH); }
    const cell = (r, c, color) => { g.fillStyle = gaugeCol(r) || gaugeCol(c) ? rgba(C.dim, 0.55) : color; g.fillRect(hx + c * cH, hy + r * cH, Math.max(1, cH), Math.max(1, cH)); };
    for (let j = 0; j < J; j++) for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) cell(3 * j + a, 3 * j + b, C.blue);
    for (let i = 0; i < N; i++) for (let a = 0; a < 2; a++) for (let b = 0; b < 2; b++) cell(3 * J + 2 * i + a, 3 * J + 2 * i + b, C.red);
    for (const o of obs) for (let a = 0; a < 3; a++) for (let b = 0; b < 2; b++) { cell(3 * o.j + a, 3 * J + 2 * o.i + b, C.purple); cell(3 * J + 2 * o.i + b, 3 * o.j + a, C.purple); }
    g.strokeStyle = C.line; g.strokeRect(hx + 0.5, hy + 0.5, hs, hs);
    g.strokeStyle = rgba(C.fg, 0.5); g.setLineDash([3, 3]);
    g.beginPath(); g.moveTo(hx + 3 * J * cH, hy); g.lineTo(hx + 3 * J * cH, hy + hs); g.moveTo(hx, hy + 3 * J * cH); g.lineTo(hx + hs, hy + 3 * J * cH); g.stroke();
    g.setLineDash([]);
    // block letters
    g.font = 'bold 15px Inter, Arial, sans-serif'; g.textAlign = 'center';
    const put = (t, x, y) => { g.fillStyle = C.bg; g.fillText(t, x + 1, y + 1); g.fillStyle = C.fg; g.fillText(t, x, y); };
    if (3 * J * cH > 18) put('U', hx + 1.5 * J * cH, hy + 1.5 * J * cH + 5);
    if (2 * N * cH > 18) put('V', hx + (3 * J + N) * cH, hy + (3 * J + N) * cH + 5);
    if (2 * N * cH > 18 && 3 * J * cH > 18) { put('W', hx + (3 * J + N) * cH, hy + 1.5 * J * cH + 5); put('Wᵀ', hx + 1.5 * J * cH, hy + (3 * J + N) * cH + 5); }
    // column groups (under H; under J only when it is wide enough)
    g.font = '11px Inter, Arial, sans-serif';
    const groups = (x0, c, short) => {
      g.fillStyle = C.blue; g.fillText(short ? 'cams' : `cameras 3J = ${3 * J}`, x0 + 1.5 * J * c, Hc - 16);
      g.fillStyle = C.red; g.fillText(short ? 'points' : `points 2N = ${2 * N}`, x0 + (3 * J + N) * c, Hc - 16);
    };
    if (jw > 240) groups(jx, cJ, false); else if (jw > 70) groups(jx, cJ, true);
    groups(hx, cH, hs < 240);
    g.textAlign = 'left'; g.fillStyle = C.dim;
    g.fillText(state.gauge ? 'grey: the 4 gauge columns (fixed, not unknowns)' : 'no gauge fix: H has 4 null directions', jx, Hc - 4);
    g.strokeStyle = C.line; g.strokeRect(0.5, 0.5, Wc - 1, Hc - 1);
  }

  const sup = v => String(v).replace(/-/g, '⁻').replace(/\d/g, d => '⁰¹²³⁴⁵⁶⁷⁸⁹'[d]);
  function drawPlot() {
    if (!has('plot')) return;
    const g = plot.getContext('2d'), Wc = plot.width, Hc = plot.height;
    g.fillStyle = C.bg2; g.fillRect(0, 0, Wc, Hc);
    const l = 40, r = 10, t = 20, b = 20;
    const hist = state.hist.map(c => Math.log10(Math.max(c, 1e-6)));
    const xmax = Math.max(8, hist.length - 1);
    let ymin = Math.min(...hist), ymax = Math.max(...hist);
    if (ymax - ymin < 1) { ymin -= 0.5; ymax += 0.5; }
    ymin = Math.floor(ymin); ymax = Math.ceil(ymax);
    const px = i => l + (Wc - l - r) * i / xmax, py = v => Hc - b - (Hc - b - t) * (v - ymin) / (ymax - ymin);
    g.strokeStyle = C.line; g.lineWidth = 1; g.beginPath(); g.moveTo(l, t); g.lineTo(l, Hc - b); g.lineTo(Wc - r, Hc - b); g.stroke();
    g.fillStyle = C.dim; g.font = '11px Inter, Arial, sans-serif'; g.textAlign = 'right';
    for (let v = ymin; v <= ymax; v += Math.max(1, Math.ceil((ymax - ymin) / 4))) g.fillText('10' + sup(v), l - 4, py(v) + 4);
    g.textAlign = 'center';
    for (let i = 0; i <= xmax; i += xmax > 16 ? 4 : 2) g.fillText(i, px(i), Hc - 6);
    g.textAlign = 'left'; g.fillStyle = C.fg; g.fillText('cost Σρ(r) (px²) per iteration', l, 12);
    g.strokeStyle = C.accent; g.lineWidth = 2; g.beginPath();
    hist.forEach((v, i) => (i ? g.lineTo(px(i), py(v)) : g.moveTo(px(i), py(v)))); g.stroke();
    hist.forEach((v, i) => { g.fillStyle = C.accent; g.beginPath(); g.arc(px(i), py(v), 2.5, 0, 7); g.fill(); });
    g.textAlign = 'right'; g.fillStyle = C.accent; g.fillText(`RMS ${fmt(prob.rms(prob.q), 2)} px`, Wc - r, 12);
    g.strokeStyle = C.line; g.strokeRect(0.5, 0.5, Wc - 1, Hc - 1);
  }

  function drawReadout() {
    const c = state.hist[state.hist.length - 1], M = prob.obs.length, rmsNow = prob.rms(prob.q);
    const status = state.singular ? `<span style="color:${C.red}">JᵀJ singular${state.gauge ? ' (degenerate)' : ': 4 free directions, no gauge'}</span>`
      : state.converged ? `<span style="color:${C.green}">converged, ${state.iter} steps</span>` : `iteration ${state.iter}`;
    if (cfg.simple) {
      readout.innerHTML =
        `<div>J = <b>${state.J}</b> cameras, N = <b>${state.N}</b> points, M = <b>${M}</b> observations</div>` +
        `<div>RMS reprojection error <b>${fmt(rmsNow, rmsNow >= 10 ? 1 : 2)} px</b></div><div>${status}</div>`;
      return;
    }
    readout.innerHTML =
      `<div>J = <b>${state.J}</b> cameras, N = <b>${state.N}</b> points, M = <b>${M}</b> obs.</div>` +
      `<div>unknowns 3J+2N${state.gauge ? '−4' : ''} = <b>${prob.free.length}</b> · vs truth ${fmt(100 * prob.errTruth(prob.q), 0)} cm</div>` +
      `<div>cost ${c >= 1e4 ? c.toExponential(1) : fmt(c, 1)} px², RMS <b>${fmt(rmsNow, rmsNow >= 10 ? 1 : 2)} px</b></div>` +
      `<div>${status} · ${state.lm ? `LM λ = ${state.lambda.toExponential(1)}` : 'Gauss-Newton'}${state.robust ? ', Huber' : ''}${state.outliers ? ', outliers' : ''}</div>`;
  }
  function draw() {
    if (has('world')) drawWorld();
    if (has('images')) drawImages();
    if (has('structure')) drawStructure();
    drawPlot();
    drawReadout();
    syncControls();
  }

  // ---- animated solve ----
  const shown = () => el.closest('section')?.classList.contains('present') !== false;
  function solveAnim() {
    if (solving) { solving = false; syncControls(); return; }
    if (state.converged || state.singular) return;
    solving = true; syncControls();
    const tick = () => {
      if (!solving || !el.isConnected) { solving = false; return; }
      if (shown()) { prob.step(); draw(); }
      if (state.converged || state.singular) { solving = false; syncControls(); return; }
      setTimeout(tick, 450);
    };
    setTimeout(tick, 60);
  }

  // ---- controls ----
  const stepBtn = h('button', { onclick: () => { solving = false; prob.step(); draw(); } }, '1 step');
  const solveBtn = h('button', { style: 'width:5.4em', onclick: solveAnim }, '▶ solve');
  const resetBtn = h('button', { onclick: () => { solving = false; prob.reset(); draw(); } }, 'reset');
  const perturbBtn = h('button', { onclick: () => { solving = false; state.seedP++; prob.setup(true); draw(); } }, 'perturb again');
  const toggle = (key, text, after) => h('button', { onclick: () => { solving = false; state[key] = !state[key]; after(); draw(); } }, text);
  const lmBtn = toggle('lm', 'LM damping', () => { state.lambda = 1e-3; });
  const gaugeBtn = toggle('gauge', 'gauge fix', () => prob.setup(true));
  const outBtn = toggle('outliers', 'outliers', () => prob.restart());
  const robustBtn = toggle('robust', 'Huber loss', () => prob.restart());
  const noiseVal = h('span', { class: 'readout', style: 'display:inline-block; width:4.4em; text-align:right; white-space:nowrap' });
  const noiseS = h('input', { type: 'range', min: 0, max: 4, step: 0.25, value: state.noise, style: 'width:7em' });
  noiseS.addEventListener('input', () => { solving = false; state.noise = parseFloat(noiseS.value); prob.reset(); draw(); });
  const num = (key, min, max, after) => {
    const inp = h('input', { type: 'number', min, max, step: 1, value: state[key] });
    inp.addEventListener('change', () => { solving = false; state[key] = Math.max(min, Math.min(max, Math.round(+inp.value) || min)); inp.value = state[key]; after(); });
    return inp;
  };
  const camsIn = num('J', 3, 8, () => { prob.setup(false); fitWorld(); draw(); });
  const ptsIn = num('N', 6, 24, () => { prob.setup(false); fitWorld(); draw(); });
  function syncControls() {
    solveBtn.textContent = solving ? '⏸ pause' : '▶ solve';
    lmBtn.classList.toggle('active', state.lm); gaugeBtn.classList.toggle('active', state.gauge);
    outBtn.classList.toggle('active', state.outliers); robustBtn.classList.toggle('active', state.robust);
    noiseVal.textContent = fmt(state.noise, 2) + ' px';
    stepBtn.disabled = state.converged || state.singular;
  }

  // ---- layout ----
  const left = h('div', { style: `display:flex; flex-direction:column; gap:8px; width:${W}px; flex:none` });
  if (has('world')) left.append(world);
  if (has('images')) left.append(images);
  const right = h('div', { style: `display:flex; flex-direction:column; gap:8px; width:${RW}px; flex:none` });
  if (has('structure')) right.append(structure);
  if (has('plot')) right.append(plot);
  const row = h('div', { style: 'display:flex; gap:10px; align-items:flex-start' }, left);
  const readoutInColumn = right.children.length > 0 && has('world') && has('images');
  if (right.children.length) { if (readoutInColumn) right.append(readout); row.append(right); }
  if (!readoutInColumn) { readout.style.display = 'flex'; readout.style.flexWrap = 'wrap'; readout.style.columnGap = '1.2em'; }
  el.classList.add('widget');
  el.append(row);
  if (!readoutInColumn) el.append(h('div', { style: 'margin-top:6px' }, readout));
  el.append(
    h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, stepBtn, solveBtn, resetBtn, perturbBtn, ...(cfg.simple ? [] : [lmBtn, gaugeBtn, outBtn, robustBtn])),
    h('div', { class: 'wctl interactive-only', style: 'gap:0.4em' },
      h('label', {}, 'noise σ'), noiseS, noiseVal,
      h('label', { style: 'margin-left:0.6em' }, 'cameras'), camsIn, h('label', { style: 'margin-left:0.6em' }, 'points'), ptsIn));

  fitWorld();
  if (cfg.solved ?? print) prob.solveAll();
  draw();
}
