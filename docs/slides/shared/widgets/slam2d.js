// Online landmark SLAM in 2D, the setting of lab 10: a robot drives once around a loop (and a bit further, revisiting
// the start) in a field of point landmarks. Two kinds of measurement, both noisy: odometry (Δx, Δy, Δθ per step, in
// the robot frame) and a range-bearing sensor (distance and angle to every landmark within its range and field of
// view). Data association is known: the sensor reports which landmark it sees.
//
// Estimates, both computed for real at every step:
//   dead reckoning: the odometry chained, nothing else (drifts without bound);
//   graph SLAM: all poses so far (x, y, θ) and all landmarks seen so far (x, y) are the unknowns of one least-squares
//   problem: pose 0 is fixed (the anchor), one factor per odometry step, one factor per observation (range, bearing),
//   weights 1/σ². Each new pose starts from the previous estimate + odometry, a new landmark from its first
//   observation, then Gauss–Newton (dense normal equations, Cholesky) refines everything. The pose block of the
//   normal matrix is banded (only consecutive poses share a factor) and the landmark block is block-diagonal, so the
//   factorisation is cheap: ordering the poses first and the landmarks last, no fill-in appears in the pose band.
//   A loop closure is nothing special for the optimiser: a landmark last seen more than 50 steps ago is observed
//   again, which adds a factor between a new pose and an old part of the map; a few more iterations and the map snaps.
//   95% ellipses: marginal covariances from the final normal matrix (columns of its inverse, by two triangular solves).
// Timing (M1, 145 poses, 14 landmarks, ~470 unknowns): < 1 ms per Gauss–Newton iteration, 3 per step.
//
// config: {
//   width: 620, height: 400     map canvas (px)
//   sigmaTheta: 0.6             odometry heading noise per step (degrees), slider 0 … 2
//   sigmaT: 0.03                odometry translation noise per step and axis (m), slider 0 … 0.1
//   sigmaRange: 0.15            range noise (m), slider 0 … 0.5
//   sigmaBearing: 2             bearing noise (degrees), slider 0 … 6
//   range: 6, fov: 70           sensor range (m) and half field of view (degrees)
//   landmarks: 14               number of landmarks (fixed layout, independent of the seed)
//   extra: 25                   steps driven past the start after the loop (120 steps)
//   t: null                     initial step (disables autoplay; for screenshots); print default: the last step
//   autoplay: true              start driving when the slide is shown (never in print)
//   speed: 1                    playback speed (1 = 10 steps per second)
//   graph: false                show the factor graph (nodes and factors) over the map
//   drOnly: false               dead reckoning only: no SLAM estimate (to motivate it)
//   seed: 6                     noise draws ("new run" increments it)
//   runs: 0                     faint extra dead-reckoning runs with other seeds
//   simple: false               only the robot, its path, the landmarks and what the sensor sees: no estimates, no
//                               plot, no sliders (to show what a landmark and an observation are)
// }
// Animation runs only while the slide is shown (section.present) and stops when the widget is unmounted.
import { C, h, fmt, isPrint } from './util.js';
import { rng } from './linalg.js';

const deg = Math.PI / 180;
const A = 8, B = 4.5, RC = 2.5;        // loop: half-width, half-height, corner radius (m)
const N = 120;                         // steps around the loop
const LOOP_GAP = 50;                   // a landmark unseen for this many steps, seen again: loop closure
const BW = 5;                          // band half-width of the pose block (3 unknowns per pose, consecutive poses coupled)
const X_RANGE = 12.25, Y_RANGE = 8.25; // world box shown (m)

const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));

// ---- the loop: arc length → pose (x, y, θ), y up, counter-clockwise, start at the bottom middle heading +x ----
function makePath() {
  const segs = [];
  const line = (x0, y0, th, L) => segs.push({ L, f: s => [x0 + s * Math.cos(th), y0 + s * Math.sin(th), th] });
  const arc = (cx, cy, a0, th0) => segs.push({ L: RC * Math.PI / 2, f: s => { const a = a0 + s / RC; return [cx + RC * Math.cos(a), cy + RC * Math.sin(a), th0 + s / RC]; } });
  const q = Math.PI / 2;
  line(0, -B, 0, A - RC);
  arc(A - RC, -B + RC, -q, 0);
  line(A, -B + RC, q, 2 * (B - RC));
  arc(A - RC, B - RC, 0, q);
  line(A - RC, B, 2 * q, 2 * (A - RC));
  arc(-A + RC, B - RC, q, 2 * q);
  line(-A, B - RC, 3 * q, 2 * (B - RC));
  arc(-A + RC, -B + RC, 2 * q, 3 * q);
  line(-A + RC, -B, 4 * q, A - RC);
  const total = segs.reduce((s, g) => s + g.L, 0);
  const at = s0 => {
    let s = ((s0 % total) + total) % total;
    for (const g of segs) { if (s <= g.L + 1e-9) { const p = g.f(s); return [p[0], p[1], wrap(p[2])]; } s -= g.L; }
    const g = segs[segs.length - 1]; return g.f(g.L);
  };
  return { at, total };
}
const PATH = makePath();

// relative motion of pose j seen from pose i, and its inverse
function relative(pi, pj) {
  const c = Math.cos(pi[2]), s = Math.sin(pi[2]), dx = pj[0] - pi[0], dy = pj[1] - pi[1];
  return [c * dx + s * dy, -s * dx + c * dy, wrap(pj[2] - pi[2])];
}
function compose(p, d) {
  const c = Math.cos(p[2]), s = Math.sin(p[2]);
  return [p[0] + c * d[0] - s * d[1], p[1] + s * d[0] + c * d[1], wrap(p[2] + d[2])];
}

// ---- landmarks: a fixed layout for a given count, 1.3 … 5 m from the path, 2 m apart ----
export function makeLandmarks(K) {
  const r = rng(3), samples = [];
  for (let i = 0; i < 400; i++) samples.push(PATH.at(PATH.total * i / 400));
  const dPath = (x, y) => Math.min(...samples.map(p => Math.hypot(p[0] - x, p[1] - y)));
  const lms = [];
  let guard = 0;
  while (lms.length < K && guard++ < 5000) {
    const x = -11.5 + 23 * r(), y = -7.5 + 15 * r();
    const d = dPath(x, y);
    if (d < 1.3 || d > 5) continue;
    if (lms.some(l => Math.hypot(l[0] - x, l[1] - y) < 2)) continue;
    lms.push([x, y]);
  }
  return lms;
}

// ---- simulation: truth, noisy odometry, noisy observations (noise scaled from a fixed sequence of normals) ----
export function simulate(cfg, lms) {
  const T = N + cfg.extra;
  const poses = [];
  for (let k = 0; k <= T; k++) poses.push(PATH.at(PATH.total * k / N));
  const r = rng(cfg.seed * 7919 + 13);
  const odo = [];
  for (let k = 0; k < T; k++) {
    const d = relative(poses[k], poses[k + 1]);
    odo.push([d[0] + cfg.sigmaT * r.normal(), d[1] + cfg.sigmaT * r.normal(), d[2] + cfg.sigmaTheta * deg * r.normal()]);
  }
  const r2 = rng(cfg.seed * 104729 + 7);
  const obs = [];   // obs[k]: [{ id, r, b }] at pose k
  for (let k = 0; k <= T; k++) {
    const p = poses[k], list = [];
    lms.forEach((l, id) => {
      const dx = l[0] - p[0], dy = l[1] - p[1], d = Math.hypot(dx, dy), b = wrap(Math.atan2(dy, dx) - p[2]);
      if (d <= cfg.range && Math.abs(b) <= cfg.fov * deg)
        list.push({ id, r: d + cfg.sigmaRange * r2.normal(), b: wrap(b + cfg.sigmaBearing * deg * r2.normal()) });
    });
    obs.push(list);
  }
  const dr = [poses[0].slice()];
  odo.forEach(m => dr.push(compose(dr[dr.length - 1], m)));
  return { T, poses, odo, obs, dr };
}

// ---- structured dense Cholesky: pose block (first np unknowns) banded with half-width BW, landmark block last ----
let BUF = { n: 0, H: null, L: null, b: null, y: null, x: null };
function buffers(n) {
  if (BUF.n < n) BUF = { n, H: new Float64Array(n * n), L: new Float64Array(n * n), b: new Float64Array(n), y: new Float64Array(n), x: new Float64Array(n) };
  return BUF;
}
function cholesky(H, L, n, np) {
  L.fill(0, 0, n * n);
  for (let j = 0; j < n; j++) {
    const kmin = j < np ? Math.max(0, j - BW) : 0;
    let d = H[j * n + j];
    for (let k = kmin; k < j; k++) d -= L[j * n + k] * L[j * n + k];
    if (d <= 1e-12) return false;
    d = Math.sqrt(d); L[j * n + j] = d;
    const iEnd = j < np ? Math.min(np, j + BW + 1) : n;
    for (let i = j + 1; i < iEnd; i++) {
      let s = H[i * n + j];
      for (let k = kmin; k < j; k++) s -= L[i * n + k] * L[j * n + k];
      L[i * n + j] = s / d;
    }
    if (j < np) for (let i = np; i < n; i++) {
      let s = H[i * n + j];
      for (let k = kmin; k < j; k++) s -= L[i * n + k] * L[j * n + k];
      L[i * n + j] = s / d;
    }
  }
  return true;
}
// solve L Lᵀ x = b with the same structure
function cholSolve(L, n, np, b, y, x) {
  for (let i = 0; i < n; i++) {
    let s = b[i];
    const kmin = i < np ? Math.max(0, i - BW) : 0;
    for (let k = kmin; k < i; k++) s -= L[i * n + k] * y[k];
    y[i] = s / L[i * n + i];
  }
  for (let i = n - 1; i >= 0; i--) {
    let s = y[i];
    if (i < np) {
      const kEnd = Math.min(np, i + BW + 1);
      for (let k = i + 1; k < kEnd; k++) s -= L[k * n + i] * x[k];
      for (let k = np; k < n; k++) s -= L[k * n + i] * x[k];
    } else for (let k = i + 1; k < n; k++) s -= L[k * n + i] * x[k];
    x[i] = s / L[i * n + i];
  }
}

// ---- graph SLAM up to step t: Gauss–Newton on poses 1 … t and the landmarks seen so far ----
// est: { poses: [[x, y, θ]…] (t + 1), lm: [[x, y]…] (in order of first sight), lmId: [id…], idx: Map id → index }
function optimise(est, sim, w, iters) {
  const t = est.poses.length - 1, nl = est.lm.length, np = 3 * t, n = np + 2 * nl;
  const { H, L, b, y, x } = buffers(n);
  const pI = k => 3 * (k - 1), lI = j => np + 2 * j;
  const add = (ia, ib, Ja, Jb, e, wv, ra, rb) => {   // Jᵀ W J blocks and Jᵀ W e for a factor with two variables
    const vars = [[ia, Ja, ra], [ib, Jb, rb]].filter(v => v[0] >= 0);
    for (const [oa, Ja2, ca] of vars) {
      for (let r = 0; r < ca; r++) {
        let g = 0;
        for (let m = 0; m < e.length; m++) g += Ja2[m][r] * wv[m] * e[m];
        b[oa + r] -= g;
        for (const [ob, Jb2, cb] of vars) for (let c = 0; c < cb; c++) {
          let v = 0;
          for (let m = 0; m < e.length; m++) v += Ja2[m][r] * wv[m] * Jb2[m][c];
          H[(oa + r) * n + ob + c] += v;
        }
      }
    }
  };
  const build = () => {
    H.fill(0, 0, n * n); b.fill(0, 0, n);
    for (let k = 0; k < t; k++) {              // odometry k → k + 1
      const pi = est.poses[k], pj = est.poses[k + 1], z = sim.odo[k], c = Math.cos(pi[2]), s = Math.sin(pi[2]);
      const dx = pj[0] - pi[0], dy = pj[1] - pi[1];
      const e = [c * dx + s * dy - z[0], -s * dx + c * dy - z[1], wrap(pj[2] - pi[2] - z[2])];
      const Ji = [[-c, -s, -s * dx + c * dy], [s, -c, -c * dx - s * dy], [0, 0, -1]];
      const Jj = [[c, s, 0], [-s, c, 0], [0, 0, 1]];
      add(k > 0 ? pI(k) : -1, pI(k + 1), Ji, Jj, e, w.odo, 3, 3);
    }
    for (let k = 0; k <= t; k++) for (const o of sim.obs[k]) {   // observations at pose k
      const j = est.idx.get(o.id); if (j === undefined) continue;
      const p = est.poses[k], l = est.lm[j], dx = l[0] - p[0], dy = l[1] - p[1], q = dx * dx + dy * dy, r = Math.sqrt(q);
      const e = [r - o.r, wrap(Math.atan2(dy, dx) - p[2] - o.b)];
      const Jp = [[-dx / r, -dy / r, 0], [dy / q, -dx / q, -1]];
      const Jl = [[dx / r, dy / r], [-dy / q, dx / q]];
      add(k > 0 ? pI(k) : -1, lI(j), Jp, Jl, e, w.obs, 3, 2);
    }
  };
  if (n === 0) return;
  for (let it = 0; it < iters; it++) {
    build();
    let lambda = 1e-6, ok = false;
    for (let tries = 0; tries < 6 && !ok; tries++) {
      for (let j = 0; j < n; j++) H[j * n + j] *= 1 + lambda;
      ok = cholesky(H, L, n, np);
      lambda *= 100;
    }
    if (!ok) break;
    cholSolve(L, n, np, b, y, x);
    let mx = 0;
    for (let k = 1; k <= t; k++) { const o = pI(k); est.poses[k][0] += x[o]; est.poses[k][1] += x[o + 1]; est.poses[k][2] = wrap(est.poses[k][2] + x[o + 2]); mx = Math.max(mx, Math.abs(x[o]), Math.abs(x[o + 1])); }
    for (let j = 0; j < nl; j++) { const o = lI(j); est.lm[j][0] += x[o]; est.lm[j][1] += x[o + 1]; mx = Math.max(mx, Math.abs(x[o]), Math.abs(x[o + 1])); }
    if (mx < 1e-5) break;
  }
  // marginal covariances: columns of H⁻¹ for the current pose and every landmark
  build();
  for (let j = 0; j < n; j++) H[j * n + j] *= 1 + 1e-9;
  if (!cholesky(H, L, n, np)) return;
  const col = c => { b.fill(0, 0, n); b[c] = 1; cholSolve(L, n, np, b, y, x); return x; };
  const cov2 = o => { const c0 = col(o), a = c0[o], bb = c0[o + 1]; const c1 = col(o + 1); return [a, bb, c1[o + 1]]; };
  est.poseCov = t > 0 ? cov2(pI(t)) : [0, 0, 0];
  est.lmCov = est.lm.map((_, j) => cov2(lI(j)));
}

// run the online estimate step by step, caching a snapshot per step
export function makeEstimator(sim, cfg) {
  const w = {
    odo: [1 / Math.max(cfg.sigmaT, 1e-3) ** 2, 1 / Math.max(cfg.sigmaT, 1e-3) ** 2, 1 / Math.max(cfg.sigmaTheta * deg, 1e-4) ** 2],
    obs: [1 / Math.max(cfg.sigmaRange, 1e-3) ** 2, 1 / Math.max(cfg.sigmaBearing * deg, 1e-4) ** 2],
  };
  const snaps = [];
  const est = { poses: [], lm: [], lmId: [], idx: new Map(), poseCov: [0, 0, 0], lmCov: [] };
  const lastSeen = new Map();
  const upTo = t => {
    while (snaps.length <= t) {
      const k = snaps.length;
      est.poses.push(k === 0 ? sim.poses[0].slice() : compose(est.poses[k - 1], sim.odo[k - 1]));
      const closure = [];
      for (const o of sim.obs[k]) {
        if (!est.idx.has(o.id)) {
          const p = est.poses[k], a = p[2] + o.b;
          est.idx.set(o.id, est.lm.length); est.lmId.push(o.id);
          est.lm.push([p[0] + o.r * Math.cos(a), p[1] + o.r * Math.sin(a)]);
        } else if (k - lastSeen.get(o.id) > LOOP_GAP) closure.push(o.id);
        lastSeen.set(o.id, k);
      }
      optimise(est, sim, w, closure.length ? 10 : 3);
      snaps.push({
        poses: est.poses.map(p => p.slice()), lm: est.lm.map(l => l.slice()), lmId: est.lmId.slice(),
        poseCov: est.poseCov.slice(), lmCov: est.lmCov.map(c => c.slice()), closure,
      });
    }
    return snaps[t];
  };
  return { upTo, length: () => snaps.length };
}

export function mount(el, cfg) {
  const W = cfg.width || 620, H = cfg.height || 400;
  const print = isPrint();
  const K = cfg.landmarks ?? 14;
  const state = {
    sigmaTheta: cfg.sigmaTheta ?? 0.6, sigmaT: cfg.sigmaT ?? 0.03, sigmaRange: cfg.sigmaRange ?? 0.15, sigmaBearing: cfg.sigmaBearing ?? 2,
    range: cfg.range ?? 6, fov: cfg.fov ?? 70, extra: cfg.extra ?? 25, seed: cfg.seed ?? 6, speed: cfg.speed ?? 1,
    graph: !!cfg.graph, drOnly: !!cfg.drOnly, runs: cfg.runs ?? 0, simple: !!cfg.simple,
    t: 0, playing: false,
  };
  const TOTAL = () => N + state.extra;
  state.t = cfg.t ?? (print ? TOTAL() : 0);
  const STEPS_PER_S = 10;
  const PW = cfg.panelWidth || 300, PH = 150;
  const lms = makeLandmarks(K);
  const canvas = h('canvas', { width: W, height: H });
  const g = canvas.getContext('2d');
  const plot = h('canvas', { width: PW, height: PH });
  const pg = plot.getContext('2d');
  const readout = h('div', { style: 'line-height:1.35; height:8.2em; overflow:hidden' });

  const sc = Math.min(W / (2 * X_RANGE), H / (2 * Y_RANGE));
  const X = x => W / 2 + x * sc, Y = y => H / 2 - y * sc;

  let sim, estimator, faint;
  function compute() {
    sim = simulate(state, lms);
    estimator = makeEstimator(sim, state);
    faint = [];
    for (let k = 1; k <= state.runs; k++) faint.push(simulate({ ...state, seed: state.seed * 101 + k }, lms).dr);
  }
  const snap = () => estimator.upTo(Math.round(state.t));

  // ---- drawing helpers ----
  function path(ctx, pts, color, width, alpha = 1, dash = []) {
    if (pts.length < 2) return;
    ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.lineJoin = 'round';
    ctx.beginPath(); pts.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])))); ctx.stroke();
    ctx.restore();
  }
  function robot(p, color, r = 8, outline = false) {
    const x = X(p[0]), y = Y(p[1]), c = Math.cos(p[2]), s = -Math.sin(p[2]);
    g.beginPath();
    g.moveTo(x + r * 1.4 * c, y + r * 1.4 * s);
    g.lineTo(x - r * c - r * 0.8 * s, y - r * s + r * 0.8 * c);
    g.lineTo(x - r * c + r * 0.8 * s, y - r * s - r * 0.8 * c);
    g.closePath();
    if (outline) { g.strokeStyle = color; g.lineWidth = 2.2; g.stroke(); } else { g.fillStyle = color; g.fill(); }
  }
  // 95% ellipse of a 2×2 covariance [sxx, sxy, syy] around (x, y)
  function ellipse(x, y, [a, b, c], color = C.accent) {
    const tr = (a + c) / 2, d = Math.sqrt(Math.max(0, ((a - c) / 2) ** 2 + b * b));
    const l1 = tr + d, l2 = Math.max(0, tr - d), ang = 0.5 * Math.atan2(2 * b, a - c), k = Math.sqrt(5.991);
    const r1 = Math.max(2, k * Math.sqrt(l1) * sc), r2 = Math.max(2, k * Math.sqrt(l2) * sc);
    g.save(); g.translate(X(x), Y(y)); g.rotate(-ang);
    g.beginPath(); g.ellipse(0, 0, r1, r2, 0, 0, 2 * Math.PI);
    g.globalAlpha = 0.14; g.fillStyle = color; g.fill();
    g.globalAlpha = 0.9; g.strokeStyle = color; g.lineWidth = 1.3; g.setLineDash([4, 3]); g.stroke();
    g.restore();
  }
  const cross = (x, y, r, color, w = 1.5) => {
    g.strokeStyle = color; g.lineWidth = w; g.beginPath();
    g.moveTo(X(x) - r, Y(y) - r); g.lineTo(X(x) + r, Y(y) + r); g.moveTo(X(x) - r, Y(y) + r); g.lineTo(X(x) + r, Y(y) - r); g.stroke();
  };

  function draw() {
    const t = Math.round(state.t), T = TOTAL(), s = state.drOnly || state.simple ? null : snap();
    const truePose = sim.poses[t], drPose = sim.dr[t];
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    g.font = '13px Inter, Arial, sans-serif';

    // sensor: field of view wedge and the rays of the current observations (from the true robot: the sensor is real)
    {
      const R = state.range * sc, a0 = -truePose[2] - state.fov * deg, a1 = -truePose[2] + state.fov * deg;
      g.save(); g.globalAlpha = 0.1; g.fillStyle = C.blue;
      g.beginPath(); g.moveTo(X(truePose[0]), Y(truePose[1])); g.arc(X(truePose[0]), Y(truePose[1]), R, a0, a1); g.closePath(); g.fill();
      g.restore();
      g.strokeStyle = C.blue; g.lineWidth = 1.2; g.globalAlpha = 0.8;
      for (const o of sim.obs[t]) { const l = lms[o.id]; g.beginPath(); g.moveTo(X(truePose[0]), Y(truePose[1])); g.lineTo(X(l[0]), Y(l[1])); g.stroke(); }
      g.globalAlpha = 1;
    }
    // true landmarks
    lms.forEach(l => cross(l[0], l[1], 4, C.dim));
    if (state.simple) {   // the robot (blue), its path and what it sees; landmarks green, like l_k in the formulas
      lms.forEach(l => cross(l[0], l[1], 4, C.green));
      for (const o of sim.obs[t]) { const l = lms[o.id]; g.fillStyle = C.green; g.fillRect(X(l[0]) - 5, Y(l[1]) - 5, 10, 10); }
      path(g, sim.poses.slice(0, t + 1), C.fg, 2, 0.8, [6, 5]);
      robot(truePose, C.blue, 9);
      g.fillStyle = C.fg; g.textAlign = 'center'; g.font = '13px Inter, Arial, sans-serif';
      g.fillText('start', X(sim.poses[0][0]), Y(sim.poses[0][1]) + 26);
      g.textAlign = 'left'; g.fillStyle = C.blue; g.fillText('▲ robot', 12, 20); g.fillStyle = C.green;
      g.fillText('× landmarks', 72, 20); g.fillStyle = C.blue; g.fillText('◢ sensor field of view   ─ observations', 160, 20);
      g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H - 2);
      return;
    }
    // paths
    faint.forEach(dr => path(g, dr.slice(0, t + 1), C.accent, 1.2, 0.25));
    path(g, sim.poses.slice(0, t + 1), C.fg, 2, 0.8, [6, 5]);
    path(g, sim.dr.slice(0, t + 1), C.accent, 2.5);
    const closing = !state.drOnly && closureIds(t);
    if (s) {
      // factor graph overlay: nodes and factors
      if (state.graph) {
        g.strokeStyle = C.blue; g.lineWidth = 0.8; g.globalAlpha = 0.55;
        for (let k = 0; k <= t; k += 3) for (const o of sim.obs[k]) {
          const j = s.lmId.indexOf(o.id); if (j < 0) continue;
          g.beginPath(); g.moveTo(X(s.poses[k][0]), Y(s.poses[k][1])); g.lineTo(X(s.lm[j][0]), Y(s.lm[j][1])); g.stroke();
        }
        g.globalAlpha = 1;
      }
      path(g, s.poses, C.green, state.graph ? 1.5 : 3);
      if (state.graph) {
        g.fillStyle = C.fg;
        s.poses.forEach((p, k) => { g.beginPath(); g.arc(X(p[0]), Y(p[1]), k % 4 ? 2 : 3.5, 0, 7); g.fill(); });
        const p0 = s.poses[0]; g.fillStyle = C.accent;   // the prior (anchor) at pose 0
        g.beginPath(); g.moveTo(X(p0[0]) - 7, Y(p0[1]) + 12); g.lineTo(X(p0[0]) + 7, Y(p0[1]) + 12); g.lineTo(X(p0[0]), Y(p0[1]) + 2); g.closePath(); g.fill();
      }
      // landmarks: ellipses, estimates (dots or graph squares)
      s.lm.forEach((l, j) => {
        ellipse(l[0], l[1], s.lmCov[j]);
        g.fillStyle = C.green;
        if (state.graph) g.fillRect(X(l[0]) - 4, Y(l[1]) - 4, 8, 8);
        else { g.beginPath(); g.arc(X(l[0]), Y(l[1]), 4, 0, 7); g.fill(); }
        if (closing && closing.includes(s.lmId[j])) { g.strokeStyle = C.green; g.lineWidth = 2.5; g.beginPath(); g.arc(X(l[0]), Y(l[1]), 11, 0, 7); g.stroke(); }
      });
      const sp = s.poses[t];
      ellipse(sp[0], sp[1], s.poseCov, C.green);
    }
    // robots: true (outline), dead reckoning (accent), SLAM (green)
    robot(truePose, C.fg, 8, true);
    robot(drPose, C.accent);
    if (s) robot(s.poses[t], C.green);
    // labels
    g.fillStyle = C.fg; g.textAlign = 'center'; g.font = '13px Inter, Arial, sans-serif';
    g.fillText('start', X(sim.poses[0][0]), Y(sim.poses[0][1]) + 26);
    if (t >= T - 2) { g.fillStyle = C.accent; g.fillText('dead reckoning thinks it is here', X(drPose[0]), Y(drPose[1]) - 16); }
    if (closing) {
      const sp = s.poses[t];
      g.fillStyle = C.green; g.font = 'bold 14px Inter, Arial, sans-serif';
      g.fillText('loop closure: a landmark seen again', X(sp[0]), Y(sp[1]) - 18);
    }
    // scale bar
    g.strokeStyle = C.dim; g.lineWidth = 2; g.font = '13px Inter, Arial, sans-serif';
    g.beginPath(); g.moveTo(14, H - 14); g.lineTo(14 + 5 * sc, H - 14); g.moveTo(14, H - 19); g.lineTo(14, H - 9); g.moveTo(14 + 5 * sc, H - 19); g.lineTo(14 + 5 * sc, H - 9); g.stroke();
    g.fillStyle = C.dim; g.textAlign = 'left'; g.fillText('5 m', 20 + 5 * sc, H - 9);
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H - 2);
    drawPlot(s);
    drawReadout(s);
  }
  // the landmarks closing a loop around step t (shown for 10 steps after the event)
  function closureIds(t) {
    const ids = [];
    for (let k = Math.max(0, t - 9); k <= t; k++) {
      if (k >= estimator.length() && k !== t) continue;
      const sk = estimator.upTo(k);
      sk.closure.forEach(id => { if (!ids.includes(id)) ids.push(id); });
    }
    return ids.length ? ids : null;
  }

  const posErr = (p, k) => Math.hypot(p[0] - sim.poses[k][0], p[1] - sim.poses[k][1]);
  function drawPlot(s) {
    const t = Math.round(state.t), T = TOTAL();
    pg.fillStyle = C.bg2; pg.fillRect(0, 0, PW, PH);
    const drErr = sim.dr.map(posErr);
    const slamErr = s ? [] : null;
    if (s) for (let k = 0; k <= t; k++) slamErr.push(posErr(estimator.upTo(k).poses[k], k));
    const ymax = Math.max(0.5, ...drErr) * 1.08;
    const l = 38, r = 10, tp = 10, b = 34;
    const px = k => l + (PW - l - r) * k / T, py = e => PH - b - (PH - b - tp) * e / ymax;
    pg.strokeStyle = C.line; pg.lineWidth = 1;
    pg.beginPath(); pg.moveTo(l, tp); pg.lineTo(l, PH - b); pg.lineTo(PW - r, PH - b); pg.stroke();
    pg.fillStyle = C.dim; pg.font = '13px Inter, Arial, sans-serif';
    pg.textAlign = 'center'; pg.fillText('step', (l + PW - r) / 2, PH - 4);
    [0, 50, 100].forEach(k => pg.fillText(k, px(k), PH - b + 15));
    pg.textAlign = 'right';
    const stepY = ymax > 4 ? 2 : ymax > 2 ? 1 : ymax > 1 ? 0.5 : 0.25;
    for (let e = 0; e <= ymax; e += stepY) pg.fillText(fmt(e, 2), l - 4, py(e) + 4);
    pg.save(); pg.translate(12, (tp + PH - b) / 2); pg.rotate(-Math.PI / 2); pg.textAlign = 'center'; pg.fillText('position error (m)', 0, 0); pg.restore();
    const curve = (es, color, w) => {
      pg.strokeStyle = color; pg.lineWidth = w;
      pg.beginPath(); es.forEach((e, k) => (k ? pg.lineTo(px(k), py(e)) : pg.moveTo(px(k), py(e)))); pg.stroke();
    };
    curve(drErr.slice(0, t + 1), C.accent, 2);
    if (slamErr) curve(slamErr, C.green, 2);
    if (t < T) { pg.strokeStyle = C.fg; pg.lineWidth = 1; pg.setLineDash([3, 3]); pg.beginPath(); pg.moveTo(px(t), tp); pg.lineTo(px(t), PH - b); pg.stroke(); pg.setLineDash([]); }
  }

  function drawReadout(s) {
    const t = Math.round(state.t), T = TOTAL(), tp = sim.poses[t], dr = sim.dr[t];
    const dist = PATH.total * t / N;
    const eDr = posErr(dr, t), hDr = Math.abs(wrap(dr[2] - tp[2])) / deg;
    let html = `<div>step <b>${t}</b> / ${T} · <b>${fmt(dist, 0)} m</b> driven · sees ${sim.obs[t].length}</div>` +
      `<div>dead reckoning <span style="color:${C.accent}"><b>${fmt(eDr, 2)} m</b></span> off, ${fmt(hDr, 1)}°</div>`;
    if (s) {
      const sp = s.poses[t], e = posErr(sp, t), hd = Math.abs(wrap(sp[2] - tp[2])) / deg;
      const lmErr = s.lm.map((l, j) => Math.hypot(l[0] - lms[s.lmId[j]][0], l[1] - lms[s.lmId[j]][1]));
      const mean = lmErr.length ? lmErr.reduce((a, v) => a + v, 0) / lmErr.length : 0;
      html += `<div>SLAM <span style="color:${C.green}"><b>${fmt(e, 2)} m</b></span> off, ${fmt(hd, 1)}°</div>` +
        `<div>map: <b>${s.lm.length}</b> / ${K} landmarks, ${fmt(mean, 2)} m off</div>` +
        `<div>${3 * t + 2 * s.lm.length} unknowns, ${t + countObs(t)} factors</div>`;
      const c = closureIds(t);
      if (c) html += `<div style="color:${C.green}">loop closure: landmark${c.length > 1 ? 's' : ''} ${c.join(', ')} seen again</div>`;
      else if (t >= T) html += `<div class="dim">past the start: the loop is closed</div>`;
      else html += `<div class="dim">data association is known (ids)</div>`;
    } else html += `<div class="dim">odometry only: errors add up, nothing removes them</div>`;
    readout.innerHTML = html;
  }
  const countObs = t => { let c = 0; for (let k = 0; k <= t; k++) c += sim.obs[k].length; return c; };

  // ---- animation: only while the slide is shown; stops when the widget is unmounted ----
  let raf = null, last = null;
  const shown = () => canvas.closest('section')?.classList.contains('present') !== false;
  function tick(now) {
    raf = null;
    if (!canvas.isConnected) return;
    const dt = last == null ? 0 : Math.min(0.1, (now - last) / 1000);
    last = now;
    if (shown()) {
      if (state.playing) {
        state.t = Math.min(TOTAL(), state.t + dt * STEPS_PER_S * state.speed);
        if (state.t >= TOTAL()) state.playing = false;
        syncControls();
      }
      draw();
    }
    if (state.playing) raf = requestAnimationFrame(tick);
  }
  const kick = () => { if (!raf && !print) { last = null; raf = requestAnimationFrame(tick); } };
  function play() { if (state.t >= TOTAL()) state.t = 0; state.playing = true; syncControls(); kick(); }
  function pause() { state.playing = false; syncControls(); }

  // ---- controls ----
  const slider = (min, max, step, val, fmtv, set, width = '5em') => {
    const out = h('span', { class: 'readout', style: 'display:inline-block; width:3.2em; text-align:right' }, fmtv(val));
    const inp = h('input', { type: 'range', min, max, step, value: val, style: `width:${width}` });
    inp.addEventListener('input', () => { const v = parseFloat(inp.value); out.textContent = fmtv(v); set(v); compute(); draw(); });
    return [inp, out];
  };
  const [thS, thV] = slider(0, 2, 0.1, state.sigmaTheta, v => `${fmt(v, 1)}°`, v => { state.sigmaTheta = v; });
  const [tS, tV] = slider(0, 0.1, 0.005, state.sigmaT, v => `${fmt(v * 100, 1)} cm`, v => { state.sigmaT = v; });
  const [rS, rV] = slider(0, 0.5, 0.025, state.sigmaRange, v => `${fmt(v * 100, 0)} cm`, v => { state.sigmaRange = v; });
  const [bS, bV] = slider(0, 6, 0.25, state.sigmaBearing, v => `${fmt(v, 2)}°`, v => { state.sigmaBearing = v; });
  const [rgS, rgV] = slider(2, 12, 0.5, state.range, v => `${fmt(v, 1)} m`, v => { state.range = v; });
  const playBtn = h('button', { style: 'width:5.6em', onclick: () => (state.playing ? pause() : play()) }, '▶ play');
  const scrub = h('input', { type: 'range', min: 0, max: TOTAL(), step: 1, value: state.t, style: 'width:12em' });
  scrub.addEventListener('input', () => { pause(); state.t = parseFloat(scrub.value); syncControls(); draw(); });
  const speeds = [0.5, 1, 2, 4].map(v => h('button', { onclick: () => { state.speed = v; syncControls(); } }, `${v}×`));
  const graphBtn = h('button', { onclick: () => { state.graph = !state.graph; syncControls(); draw(); } }, 'factor graph');
  const drBtn = h('button', { onclick: () => { state.drOnly = !state.drOnly; syncControls(); draw(); } }, 'dead reckoning only');
  const newBtn = h('button', { onclick: () => { state.seed++; compute(); draw(); } }, 'new run');
  function syncControls() {
    playBtn.textContent = state.playing ? '⏸ pause' : '▶ play';
    scrub.value = Math.round(state.t);
    speeds.forEach((b, i) => b.classList.toggle('active', [0.5, 1, 2, 4][i] === state.speed));
    graphBtn.classList.toggle('active', state.graph);
    drBtn.classList.toggle('active', state.drOnly);
  }

  const legend = h('div', { style: 'line-height:1.35' },
    h('div', {}, h('span', { style: `color:${C.fg}` }, '╍ '), 'true path, ', h('span', { style: `color:${C.fg}` }, '△'), ' robot, ', h('span', { style: `color:${C.dim}` }, '× '), 'true landmarks'),
    h('div', {}, h('span', { style: `color:${C.accent}` }, '━ ▲ '), 'dead reckoning (odometry only)'),
    h('div', {}, h('span', { style: `color:${C.green}` }, '━ ▲ ● '), 'SLAM: poses, map, ', h('span', { style: `color:${C.accent}` }, '◌ '), '95% regions'),
    h('div', {}, h('span', { style: `color:${C.blue}` }, '◢ '), 'sensor: range and field of view'),
    h('div', { class: 'dim' }, 'graph: ● poses ■ landmarks ━ odometry ╱ observation ▲ prior'));

  el.classList.add('widget');
  if (state.simple) {
    el.append(h('div', { style: `width:${W}px` }, canvas), h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, playBtn, scrub));
    compute(); syncControls(); draw();
    if (!print && cfg.t == null && (cfg.autoplay ?? true)) play();
    return;
  }
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${W}px; flex:none` }, canvas),
    h('div', { style: `display:flex; flex-direction:column; gap:0.4em; width:${PW}px; flex:none; font-size:0.8em` }, readout, plot, legend)),
    h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' },
      playBtn, scrub, h('label', { style: 'margin-left:0.3em' }, 'speed'), ...speeds,
      h('span', { style: 'margin-left:0.6em' }), newBtn, graphBtn, drBtn),
    h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' },
      h('label', {}, 'odometry σθ'), thS, thV, h('label', { style: 'margin-left:0.3em' }, 'σt'), tS, tV,
      h('label', { style: 'margin-left:0.6em' }, 'sensor σr'), rS, rV, h('label', { style: 'margin-left:0.3em' }, 'σb'), bS, bV,
      h('label', { style: 'margin-left:0.3em' }, 'range'), rgS, rgV));

  compute();
  syncControls();
  draw();
  if (!print && cfg.t == null && (cfg.autoplay ?? true)) play();
}
