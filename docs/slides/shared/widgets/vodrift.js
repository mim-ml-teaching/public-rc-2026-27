// Why visual odometry is not enough: drift accumulates, and what a loop closure does about it.
// Top view of a robot driving once around a city block (a rounded rectangle, 160 steps of ≈ 0.7 m) and back to
// the start. Visual odometry estimates only the relative motion between consecutive frames (Δx, Δy, Δθ in the robot
// frame); chaining these noisy steps gives the estimated path. Each step gets Gaussian noise: σθ on the heading
// change, σt on each translation component. "monocular" adds scale drift: monocular VO knows each translation only
// up to scale, so the scale factor of the steps does a slow random walk (σ of log-scale 0.01 per step).
// A small heading error rotates everything that follows, so the position error grows faster than the distance.
// Faint paths: the same estimate with other noise draws. Inset: position error vs distance travelled.
//
// Animation ("drive"): the robot moves step by step; the true path (dashed) and the VO estimate are drawn up to the
// current step, with both current poses, the error segment between them and a 95% uncertainty ellipse around the
// estimate. The ellipse is the first-order covariance propagated through the chain from the per-step noise
// (state x, y, θ and, for monocular, the log-scale; linearised along the true path); it stretches across the
// direction of travel because heading errors move the robot sideways (with scale drift also along it). The plot fills in live, with the expected
// error band (2σ of the position, from the same covariance) and a cursor.
//
// Loop closure (the finale): at the start place the robot recognises it has been here and measures its pose
// relative to pose 0 (σ 5 cm, 0.5°). We then solve the 2D pose graph by least squares: poses (x, y, θ), one edge per
// odometry step (weights 1/σ² from the noise above) plus the one loop edge, pose 0 fixed, Gauss–Newton (dense
// normal equations, Cholesky) started from the odometry chain. The drifted path morphs into the solution (linear
// interpolation of x, y for display). The graph uses rigid (SE(2)) edges, so with scale drift the loop closes but
// the scale errors are only spread out, not removed. Pressing "loop closure" before the end jumps to the end.
//
// config: {
//   width: 620, height: 400   map canvas (px)
//   sigmaTheta: 0.4           heading noise per step (degrees), slider 0 … 1
//   sigmaT: 0.02              translation noise per step and axis (m), slider 0 … 0.1
//   mono: false               monocular scale drift
//   loop: false               start with the loop-closure solution shown (implies t = 160)
//   runs: 6                   faint extra estimates with other seeds
//   seed: 6
//   autoplay: true            start driving when the slide is shown (never in print)
//   speed: 1                  playback speed (1 = 160 steps in 12 s); buttons 0.5×, 1×, 2×, 4×
//   autoLoop: false           at the end, pause 1 s, then run the loop closure automatically
//   t: null                   initial step 0 … 160 (disables autoplay; for screenshots)
// }
// Animation runs only while the slide is shown (section.present) and stops when the widget is unmounted.
// Print: the final state (t = 160), loop off unless configured, no ellipse animation.
import { C, h, fmt, isPrint } from './util.js';
import { rng } from './linalg.js';

const deg = Math.PI / 180;
const A = 20, B = 11, RC = 6;           // half-width, half-height, corner radius (m)
const N = 160;                          // odometry steps
const SCALE_SIGMA = 0.01;               // monocular: σ of log-scale per step
const LOOP_SIGMA = [0.05, 0.5 * deg];   // loop-closure measurement noise (position, heading)

// ---- ground truth: arc length → pose (x, y, θ), y up, counter-clockwise, start at the bottom middle heading +x ----
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
  const at = s => {
    for (const g of segs) { if (s <= g.L + 1e-9) return g.f(s); s -= g.L; }
    const g = segs[segs.length - 1]; return g.f(g.L);
  };
  const poses = [];
  for (let k = 0; k <= N; k++) poses.push(at(total * k / N));
  return { poses, total };
}
const GT = makePath();

const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));

// relative motion of pose j seen from pose i
function relative(pi, pj) {
  const c = Math.cos(pi[2]), s = Math.sin(pi[2]), dx = pj[0] - pi[0], dy = pj[1] - pi[1];
  return [c * dx + s * dy, -s * dx + c * dy, wrap(pj[2] - pi[2])];
}
function compose(p, d) {
  const c = Math.cos(p[2]), s = Math.sin(p[2]);
  return [p[0] + c * d[0] - s * d[1], p[1] + s * d[0] + c * d[1], p[2] + d[2]];
}

const TRUE_STEPS = GT.poses.slice(1).map((p, k) => relative(GT.poses[k], p));

// noisy odometry measurements and the chained estimate
function simulate(seed, sTh, sT, mono) {
  const r = rng(seed * 7919 + 13);
  let logS = 0;
  const meas = TRUE_STEPS.map(([dx, dy, dth]) => {
    if (mono) logS += SCALE_SIGMA * r.normal();
    const sc = Math.exp(logS);
    return [sc * dx + sT * r.normal(), sc * dy + sT * r.normal(), dth + sTh * r.normal()];
  });
  const est = [GT.poses[0].slice()];
  meas.forEach(m => est.push(compose(est[est.length - 1], m)));
  return { meas, est };
}

// ---- 2D pose graph: odometry edges + one loop edge (N → 0), pose 0 fixed, Gauss–Newton ----
function poseGraph(init, meas, sTh, sT) {
  const x = init.map(p => p.slice());
  const n = 3 * N;                       // unknowns: poses 1 … N
  const wT = 1 / Math.max(sT, 1e-3) ** 2, wR = 1 / Math.max(sTh, 0.01 * deg) ** 2;
  const edges = meas.map((z, k) => ({ i: k, j: k + 1, z, w: [wT, wT, wR] }));
  // loop: the start place recognised; measured pose of 0 relative to N (truth: no motion, heading after one turn)
  edges.push({ i: N, j: 0, z: [0, 0, 0], w: [1 / LOOP_SIGMA[0] ** 2, 1 / LOOP_SIGMA[0] ** 2, 1 / LOOP_SIGMA[1] ** 2] });
  for (let it = 0; it < 10; it++) {
    const Hm = new Float64Array(n * n), b = new Float64Array(n);
    for (const { i, j, z, w } of edges) {
      const pi = x[i], pj = x[j], c = Math.cos(pi[2]), s = Math.sin(pi[2]);
      const dx = pj[0] - pi[0], dy = pj[1] - pi[1];
      const e = [c * dx + s * dy - z[0], -s * dx + c * dy - z[1], wrap(pj[2] - pi[2] - z[2])];
      // Jacobians of e w.r.t. pose i and pose j (3×3, row-major)
      const Ji = [-c, -s, -s * dx + c * dy, s, -c, -c * dx - s * dy, 0, 0, -1];
      const Jj = [c, s, 0, -s, c, 0, 0, 0, 1];
      const blocks = [[i, Ji], [j, Jj]].filter(([k]) => k > 0).map(([k, J]) => [3 * (k - 1), J]);
      for (const [oa, Ja] of blocks) {
        for (let r = 0; r < 3; r++) {
          let g = 0;
          for (let m = 0; m < 3; m++) g += Ja[3 * m + r] * w[m] * e[m];
          b[oa + r] -= g;
        }
        for (const [ob, Jb] of blocks)
          for (let r = 0; r < 3; r++) for (let cc = 0; cc < 3; cc++) {
            let v = 0;
            for (let m = 0; m < 3; m++) v += Ja[3 * m + r] * w[m] * Jb[3 * m + cc];
            Hm[(oa + r) * n + ob + cc] += v;
          }
      }
    }
    const dxs = cholSolve(Hm, b, n);
    if (!dxs) break;
    let mx = 0;
    for (let k = 1; k <= N; k++) for (let r = 0; r < 3; r++) { x[k][r] += dxs[3 * (k - 1) + r]; mx = Math.max(mx, Math.abs(dxs[3 * (k - 1) + r])); }
    if (mx < 1e-6) break;
  }
  return x;
}
function cholSolve(A, b, n) {
  const L = A; // in place, lower triangle
  for (let j = 0; j < n; j++) {
    let d = L[j * n + j];
    for (let k = 0; k < j; k++) d -= L[j * n + k] ** 2;
    if (d <= 1e-12) return null;
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

const posErr = est => est.map((p, k) => Math.hypot(p[0] - GT.poses[k][0], p[1] - GT.poses[k][1]));

// first-order covariance of the chained estimate at every step, linearised along the true path.
// state (x, y, θ, log-scale); returns the 2×2 position covariance per step as [sxx, sxy, syy].
function covariances(sTh, sT, mono) {
  let P = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; // 4×4 row-major
  const out = [[0, 0, 0]];
  const mm = (a, b) => { const r = new Array(16).fill(0); for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) for (let k = 0; k < 4; k++) r[4 * i + j] += a[4 * i + k] * b[4 * k + j]; return r; };
  const tr = a => { const r = new Array(16); for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) r[4 * i + j] = a[4 * j + i]; return r; };
  TRUE_STEPS.forEach(([dx, dy], k) => {
    const th = GT.poses[k][2], c = Math.cos(th), s = Math.sin(th);
    const wx = c * dx - s * dy, wy = s * dx + c * dy;      // step in world coordinates
    // new pose = pose + R(θ) · e^{ls} d: Jacobian w.r.t. (x, y, θ, ls)
    const F = [1, 0, -wy, mono ? wx : 0, 0, 1, wx, mono ? wy : 0, 0, 0, 1, 0, 0, 0, 0, 1];
    P = mm(mm(F, P), tr(F));
    // step noise: translation (rotated by R), heading, scale random walk
    P[0] += sT * sT; P[5] += sT * sT; P[10] += sTh * sTh;
    if (mono) P[15] += SCALE_SIGMA * SCALE_SIGMA;
    out.push([P[0], P[1], P[5]]);
  });
  return out;
}

export function mount(el, cfg) {
  const W = cfg.width || 620, H = cfg.height || 400;
  const print = isPrint();
  const state = {
    sTh: cfg.sigmaTheta ?? 0.4, sT: cfg.sigmaT ?? 0.02, mono: !!cfg.mono, loop: !!cfg.loop,
    runs: cfg.runs ?? 6, seed: cfg.seed ?? 6, speed: cfg.speed ?? 1, autoLoop: !!cfg.autoLoop,
    t: cfg.loop ? N : (cfg.t ?? (print ? N : 0)), playing: false, morph: cfg.loop ? 1 : 0, morphing: false, endWait: 0,
  };
  const STEPS_PER_S = N / 12;
  const PW = 280, PH = 170;
  const canvas = h('canvas', { width: W, height: H });
  const g = canvas.getContext('2d');
  const plot = h('canvas', { width: PW, height: PH });
  const pg = plot.getContext('2d');
  const readout = h('div', { style: 'line-height:1.4; font-size:0.92em; height:5.6em; overflow:hidden' });

  // world (m) → canvas px; the block plus a margin for drift
  const span = Math.max((2 * A + 22) / W, (2 * B + 18) / H), sc = 1 / span;
  const X = x => W / 2 + x * sc, Y = y => H / 2 - y * sc;

  let main, faint, corrected, covs;
  function compute() {
    const sTh = state.sTh * deg;
    main = simulate(state.seed, sTh, state.sT, state.mono);
    faint = [];
    for (let k = 1; k <= state.runs; k++) faint.push(simulate(state.seed * 101 + k, sTh, state.sT, state.mono).est);
    corrected = poseGraph(main.est, main.meas, sTh, state.sT);
    covs = covariances(sTh, state.sT, state.mono);
  }

  // poses 0 … floor(t) plus the interpolated pose at t
  function upTo(pts, t) {
    const k = Math.floor(t), out = pts.slice(0, k + 1);
    if (k < N && t > k) { const a = pts[k], b = pts[k + 1], f = t - k; out.push([a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * wrap(b[2] - a[2])]); }
    return out;
  }
  const at = (pts, t) => upTo(pts, t).pop();
  const morphed = () => main.est.map((p, k) => { const q = corrected[k], m = smooth(state.morph); return [p[0] + m * (q[0] - p[0]), p[1] + m * (q[1] - p[1]), p[2] + m * wrap(q[2] - p[2])]; });
  const smooth = m => m * m * (3 - 2 * m);

  function path(ctx, pts, color, width, alpha = 1, dash = []) {
    if (pts.length < 2) return;
    ctx.save(); ctx.globalAlpha = alpha; ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash);
    ctx.lineJoin = 'round';
    ctx.beginPath(); pts.forEach((p, i) => (i ? ctx.lineTo(X(p[0]), Y(p[1])) : ctx.moveTo(X(p[0]), Y(p[1])))); ctx.stroke();
    ctx.restore();
  }
  function robot(p, color, r = 9, outline = false) {
    const x = X(p[0]), y = Y(p[1]), c = Math.cos(p[2]), s = -Math.sin(p[2]);
    g.beginPath();
    g.moveTo(x + r * 1.4 * c, y + r * 1.4 * s);
    g.lineTo(x - r * c - r * 0.8 * s, y - r * s + r * 0.8 * c);
    g.lineTo(x - r * c + r * 0.8 * s, y - r * s - r * 0.8 * c);
    g.closePath();
    if (outline) { g.strokeStyle = color; g.lineWidth = 2.5; g.stroke(); } else { g.fillStyle = color; g.fill(); }
  }
  // 95% ellipse of a 2×2 covariance [sxx, sxy, syy] around p
  function ellipse(p, [a, b, c]) {
    const tr = (a + c) / 2, d = Math.sqrt(Math.max(0, ((a - c) / 2) ** 2 + b * b));
    const l1 = tr + d, l2 = Math.max(0, tr - d), ang = 0.5 * Math.atan2(2 * b, a - c), k = Math.sqrt(5.991);
    const r1 = k * Math.sqrt(l1) * sc, r2 = k * Math.sqrt(l2) * sc;
    if (r1 < 0.5) return;
    g.save(); g.translate(X(p[0]), Y(p[1])); g.rotate(-ang);
    g.beginPath(); g.ellipse(0, 0, r1, Math.max(r2, 0.5), 0, 0, 2 * Math.PI);
    g.globalAlpha = 0.14; g.fillStyle = C.accent; g.fill();
    g.globalAlpha = 0.9; g.strokeStyle = C.accent; g.lineWidth = 1.5; g.setLineDash([5, 4]); g.stroke();
    g.restore();
  }

  function draw() {
    const t = state.t, atEnd = t >= N - 1e-9, m = state.morph;
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    // the block (buildings) inside the loop
    g.fillStyle = C.bg3; g.strokeStyle = C.line; g.lineWidth = 1.5;
    const ia = A - 3.5, ib = B - 3.5;
    g.beginPath(); g.roundRect(X(-ia), Y(ib), 2 * ia * sc, 2 * ib * sc, RC * 0.5 * sc); g.fill(); g.stroke();
    g.fillStyle = C.dim; g.font = '15px Inter, Arial, sans-serif'; g.textAlign = 'center';
    g.fillText('city block', X(0), Y(0) + 5);
    // scale bar
    g.strokeStyle = C.dim; g.lineWidth = 2;
    g.beginPath(); g.moveTo(16, H - 16); g.lineTo(16 + 10 * sc, H - 16); g.moveTo(16, H - 21); g.lineTo(16, H - 11); g.moveTo(16 + 10 * sc, H - 21); g.lineTo(16 + 10 * sc, H - 11); g.stroke();
    g.textAlign = 'left'; g.fillText('10 m', 22 + 10 * sc, H - 11);

    faint.forEach(est => path(g, upTo(est, t), C.accent, 1.5, m > 0 ? 0.12 : 0.28));
    path(g, upTo(GT.poses, t), C.fg, 2.5, 0.85, [7, 5]);
    path(g, upTo(main.est, t), C.accent, 3, 1 - 0.55 * m);
    if (m > 0) path(g, morphed(), C.green, 3.5, Math.min(1, 2 * m));

    const s0 = GT.poses[0], tp = at(GT.poses, t), ep = at(main.est, t);
    g.fillStyle = C.fg; g.beginPath(); g.arc(X(s0[0]), Y(s0[1]), 6, 0, 7); g.fill();
    g.font = '15px Inter, Arial, sans-serif'; g.textAlign = 'center';
    g.fillText(atEnd ? 'start = end' : 'start', X(s0[0]), Y(s0[1]) + 24);
    if (m === 0) {
      // uncertainty ellipse (interpolated covariance), error segment, both robots
      const k = Math.min(N - 1, Math.floor(t)), f = t - k, ca = covs[k], cb = covs[k + 1];
      ellipse(ep, ca.map((v, i) => v + f * (cb[i] - v)));
      g.strokeStyle = C.red; g.lineWidth = 2.5; g.setLineDash([4, 3]);
      g.beginPath(); g.moveTo(X(tp[0]), Y(tp[1])); g.lineTo(X(ep[0]), Y(ep[1])); g.stroke(); g.setLineDash([]);
      robot(tp, C.fg, 8, true);
      robot(ep, C.accent);
      if (atEnd) {
        g.fillStyle = C.accent; g.font = '15px Inter, Arial, sans-serif';
        g.textAlign = 'center';
        g.fillText('VO thinks it is here', X(ep[0]), Y(ep[1]) - 44);
      }
    } else {
      const cp = morphed()[N];
      g.strokeStyle = C.red; g.lineWidth = 2.5; g.setLineDash([4, 3]);
      g.beginPath(); g.moveTo(X(s0[0]), Y(s0[1])); g.lineTo(X(cp[0]), Y(cp[1])); g.stroke(); g.setLineDash([]);
      robot(cp, C.green);
      g.fillStyle = C.green; g.textAlign = 'center';
      g.fillText('loop closure: "I have been here"', X(s0[0]), Y(s0[1]) - 18);
    }
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H - 2);
    drawPlot();
    drawReadout();
  }

  function drawPlot() {
    const t = state.t, m = state.morph;
    pg.fillStyle = C.bg2; pg.fillRect(0, 0, PW, PH);
    const errs = posErr(main.est), ferrs = faint.map(posErr), cerr = posErr(corrected);
    const band = covs.map(([a, , c]) => 2 * Math.sqrt(a + c));
    const ymax = Math.max(1, ...errs, ...ferrs.flat(), ...band) * 1.08;
    const l = 38, r = 10, tp = 10, b = 34;
    const px = k => l + (PW - l - r) * k / N, py = e => PH - b - (PH - b - tp) * e / ymax;
    pg.strokeStyle = C.line; pg.lineWidth = 1;
    pg.beginPath(); pg.moveTo(l, tp); pg.lineTo(l, PH - b); pg.lineTo(PW - r, PH - b); pg.stroke();
    pg.fillStyle = C.dim; pg.font = '14px Inter, Arial, sans-serif';
    pg.textAlign = 'center'; pg.fillText('distance travelled (m)', (l + PW - r) / 2, PH - 4);
    [0, 50, 100].forEach(d => { const k = d / GT.total * N; pg.fillText(d, px(k), PH - b + 16); });
    pg.textAlign = 'right';
    const stepY = ymax > 8 ? 4 : ymax > 4 ? 2 : 1;
    for (let e = 0; e <= ymax; e += stepY) pg.fillText(e, l - 5, py(e) + 5);
    pg.save(); pg.translate(13, (tp + PH - b) / 2); pg.rotate(-Math.PI / 2); pg.textAlign = 'center'; pg.fillText('error (m)', 0, 0); pg.restore();
    const kEnd = Math.floor(t);
    const curve = (es, color, w, alpha = 1, upto = kEnd) => {
      pg.save(); pg.globalAlpha = alpha; pg.strokeStyle = color; pg.lineWidth = w;
      pg.beginPath(); es.slice(0, upto + 1).forEach((e, k) => (k ? pg.lineTo(px(k), py(e)) : pg.moveTo(px(k), py(e)))); pg.stroke(); pg.restore();
    };
    // expected error band (2σ of the position), drawn up to the cursor
    pg.save(); pg.globalAlpha = 0.16; pg.fillStyle = C.accent;
    pg.beginPath(); pg.moveTo(px(0), py(0));
    band.slice(0, kEnd + 1).forEach((e, k) => pg.lineTo(px(k), py(e)));
    pg.lineTo(px(kEnd), py(0)); pg.closePath(); pg.fill(); pg.restore();
    ferrs.forEach(es => curve(es, C.accent, 1.2, m > 0 ? 0.15 : 0.3));
    curve(errs, C.accent, 2.2, 1 - 0.5 * m);
    if (m > 0) curve(cerr, C.green, 2.5, Math.min(1, 2 * m), N);
    if (t < N) { pg.strokeStyle = C.fg; pg.lineWidth = 1; pg.setLineDash([3, 3]); pg.beginPath(); pg.moveTo(px(t), tp); pg.lineTo(px(t), PH - b); pg.stroke(); pg.setLineDash([]); }
  }

  function drawReadout() {
    const t = state.t, k = Math.round(t), ep = at(main.est, t), gt = at(GT.poses, t);
    const e = Math.hypot(ep[0] - gt[0], ep[1] - gt[1]), eh = Math.abs(wrap(ep[2] - gt[2])) / deg, dist = GT.total * t / N;
    let s =
      `<div>step <b>${k}</b> / ${N}, driven <b>${fmt(dist, 0)} m</b></div>` +
      `<div>VO error: <span style="color:${C.accent}"><b>${fmt(e, 1)} m</b></span>, heading ${fmt(eh, 1)}°</div>`;
    if (state.morph > 0) {
      const ce = posErr(corrected), mx = Math.max(...ce), mean = ce.reduce((a, v) => a + v, 0) / ce.length;
      s += `<div>after loop closure: <span style="color:${C.green}"><b>≤ ${fmt(mx, 1)} m</b></span> everywhere (mean ${fmt(mean, 1)} m)</div>`;
    } else if (t >= N) s += `<div class="dim">back at the start, but VO does not know it</div>`;
    else s += `<div class="dim">${fmt(dist > 0 ? 100 * e / dist : 0, 1)}% of the distance driven</div>`;
    readout.innerHTML = s;
  }

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
        state.t = Math.min(N, state.t + dt * STEPS_PER_S * state.speed);
        if (state.t >= N) {
          state.playing = false;
          if (state.autoLoop && !state.loop) state.endWait = 1;
        }
        syncControls();
      } else if (state.endWait > 0) {
        state.endWait -= dt;
        if (state.endWait <= 0) { state.endWait = 0; startLoop(); }
      }
      if (state.morphing) {
        state.morph = Math.min(1, state.morph + dt / 1.6);
        if (state.morph >= 1) state.morphing = false;
      }
      draw();
    }
    if (state.playing || state.morphing || state.endWait > 0) raf = requestAnimationFrame(tick);
  }
  const kick = () => { if (!raf && !print) { last = null; raf = requestAnimationFrame(tick); } };

  function play() {
    if (state.t >= N) { state.t = 0; resetLoop(); }
    state.playing = true; syncControls(); kick();
  }
  function pause() { state.playing = false; state.endWait = 0; syncControls(); }
  function resetLoop() { state.loop = false; state.morph = 0; state.morphing = false; state.endWait = 0; }
  function startLoop() {
    state.playing = false; state.t = N; state.loop = true; state.morphing = true; state.morph = 0;
    syncControls(); kick();
  }

  // ---- controls ----
  const slider = (min, max, step, val, fmtv, set) => {
    const out = h('span', { class: 'readout', style: 'display:inline-block; width:4.2em; text-align:right' }, fmtv(val));
    const inp = h('input', { type: 'range', min, max, step, value: val, style: 'width:7em' });
    inp.addEventListener('input', () => { const v = parseFloat(inp.value); out.textContent = fmtv(v); set(v); compute(); draw(); });
    return [inp, out];
  };
  const [thS, thV] = slider(0, 1, 0.05, state.sTh, v => `${fmt(v, 2)}°`, v => { state.sTh = v; });
  const [tS, tV] = slider(0, 0.1, 0.005, state.sT, v => `${fmt(v * 100, 1)} cm`, v => { state.sT = v; });
  const playBtn = h('button', { style: 'width:5.6em', onclick: () => (state.playing ? pause() : play()) }, '▶ play');
  const scrub = h('input', { type: 'range', min: 0, max: N, step: 1, value: state.t, style: 'width:12em' });
  scrub.addEventListener('input', () => { pause(); resetLoop(); state.t = parseFloat(scrub.value); syncControls(); draw(); });
  const speeds = [0.5, 1, 2, 4].map(v => h('button', { onclick: () => { state.speed = v; syncControls(); } }, `${v}×`));
  const monoBtn = h('button', { onclick: () => { state.mono = !state.mono; compute(); syncControls(); draw(); } }, 'monocular: scale drift');
  const loopBtn = h('button', { onclick: () => { if (state.loop) { resetLoop(); syncControls(); draw(); } else startLoop(); } }, 'loop closure');
  const newBtn = h('button', { onclick: () => { state.seed++; compute(); draw(); } }, 'new run');
  function syncControls() {
    playBtn.textContent = state.playing ? '⏸ pause' : '▶ play';
    scrub.value = state.t;
    speeds.forEach((b, i) => b.classList.toggle('active', [0.5, 1, 2, 4][i] === state.speed));
    monoBtn.classList.toggle('active', state.mono);
    loopBtn.classList.toggle('active', state.loop);
  }

  const legend = h('div', { style: 'line-height:1.35; font-size:0.92em' },
    h('div', {}, h('span', { style: `color:${C.fg}` }, '╍ '), 'true path, ', h('span', { style: `color:${C.fg}` }, '△'), ' true pose'),
    h('div', {}, h('span', { style: `color:${C.accent}` }, '━ '), 'visual odometry, ', h('span', { style: `color:${C.accent}` }, '▲')),
    h('div', { class: 'dim', style: 'padding-left:1.2em' }, 'faint: other noise draws'),
    h('div', {}, h('span', { style: `color:${C.accent}` }, '◌ '), '95% uncertainty region'),
    h('div', {}, h('span', { style: `color:${C.green}` }, '━ '), 'after loop closure'));

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${W}px` }, canvas,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' },
        playBtn, scrub, h('label', { style: 'margin-left:0.3em' }, 'speed'), ...speeds),
      h('div', { class: 'wctl interactive-only', style: 'gap:0.4em' },
        h('label', {}, 'σθ / step'), thS, thV, h('label', { style: 'margin-left:0.5em' }, 'σt / step'), tS, tV),
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' },
        newBtn, monoBtn, loopBtn)),
    h('div', { style: `display:flex; flex-direction:column; gap:0.5em; width:${PW}px; flex:none` },
      readout, plot, legend)));

  compute();
  syncControls();
  draw();
  if (!print && cfg.t == null && !cfg.loop && (cfg.autoplay ?? true)) play();
}
