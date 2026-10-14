// PnP as minimisation of the reprojection error, on a square fiducial marker (ArUco-like) with 4 known corners.
// The true pose is hidden; detected corners = exact projections + noise. The estimate (rotation vector w,
// translation t; marker → camera) is refined by Levenberg–Marquardt, one animated step at a time:
//   min_{w,t}  Σ_i ‖ π(K (R(w) X_i + t)) − x_i ‖²
// Two starting guesses tilt the marker to opposite sides. For a small / far marker both converge, to mirrored
// poses with almost the same error: the well-known pose flip of planar markers.
// Marker frame (as in OpenCV ArUco): x right, y up, z out of the marker towards the camera.
// config: { width (display px of the 640×480 image, default 560), dist (m, default 0.5), noise (px, default 0.5),
//           start: "A" | "B", solved: true, seed }
import { C, h, fmt, isPrint } from './util.js';
import { matmul, matvec, transpose, solve, rodrigues, logR, rng } from './linalg.js';

const IW = 640, IH = 480, F = 800, OX = 320, OY = 240, S = 0.1; // marker side 10 cm
const deg = Math.PI / 180;
const CORNERS = [[-S / 2, S / 2, 0], [S / 2, S / 2, 0], [S / 2, -S / 2, 0], [-S / 2, -S / 2, 0]];
// 6×6 ArUco-like bit pattern (1 = white) inside a black border
const BITS = ['000000', '011010', '010110', '001100', '011100', '000000'];
const Rx180 = [[1, 0, 0], [0, -1, 0], [0, 0, -1]];
const Ry = a => [[Math.cos(a), 0, Math.sin(a)], [0, 1, 0], [-Math.sin(a), 0, Math.cos(a)]];
const Rx = a => [[1, 0, 0], [0, Math.cos(a), -Math.sin(a)], [0, Math.sin(a), Math.cos(a)]];

const project = (R, t, X) => { const p = matvec(R, X).map((v, i) => v + t[i]); return [F * p[0] / p[2] + OX, F * p[1] / p[2] + OY]; };

export function mount(el, cfg) {
  const DW = cfg.width || 560;
  const state = { dist: cfg.dist ?? 0.5, noise: cfg.noise ?? 0.5, seed: cfg.seed ?? 3, est: null, log: [], start: cfg.start || 'A' };
  const canvas = h('canvas', { width: IW, height: IH, style: `width:${DW}px; height:${DW * IH / IW}px` });
  const g = canvas.getContext('2d');
  const top = h('canvas', { width: 300, height: 190 });
  const gt = top.getContext('2d');
  const readout = h('div', { style: 'font-size:0.95em; line-height:1.45' });
  let timer = null;

  // true pose: tilted 35° about the vertical axis and 12° about the horizontal one, slightly off-centre
  const truth = () => ({ R: matmul(matmul(Ry(35 * deg), Rx(12 * deg)), Rx180), t: [0.12 * state.dist, -0.05 * state.dist, state.dist] });
  const observed = () => {
    const r = rng(state.seed), T = truth();
    return CORNERS.map(X => project(T.R, T.t, X).map(v => v + state.noise * r.normal()));
  };
  const initial = which => {
    const T = truth();
    const R = matmul(which === 'A' ? Ry(10 * deg) : Ry(-40 * deg), Rx180);
    return { w: logR(R), t: [T.t[0] * 0.8, T.t[1] * 0.8, T.t[2] * 1.3] };
  };
  const residual = (q, x) => {
    const R = rodrigues(q.slice(0, 3)), t = q.slice(3);
    return CORNERS.flatMap((X, i) => { const p = project(R, t, X); return [p[0] - x[i][0], p[1] - x[i][1]]; });
  };
  const rms = (q, x) => { const r = residual(q, x); return Math.sqrt(r.reduce((s, v) => s + v * v, 0) / CORNERS.length); };

  // one Levenberg–Marquardt step with a numerical Jacobian
  function lmStep(est, x) {
    const q = [...est.w, ...est.t], r = residual(q, x);
    const J = r.map(() => new Array(6).fill(0));
    for (let k = 0; k < 6; k++) {
      const e = k < 3 ? 1e-6 : 1e-7 * Math.max(1, Math.abs(q[k]));
      const q2 = q.slice(); q2[k] += e;
      residual(q2, x).forEach((v, i) => { J[i][k] = (v - r[i]) / e; });
    }
    const JtJ = matmul(transpose(J), J), Jtr = matvec(transpose(J), r);
    const err0 = r.reduce((s, v) => s + v * v, 0);
    for (let mu = est.mu ?? 1e-3; mu < 1e8; mu *= 10) {
      const A = JtJ.map((row, i) => row.map((v, j) => v + (i === j ? mu * (JtJ[i][i] + 1e-9) : 0)));
      const d = solve(A, Jtr.map(v => -v));
      if (!d) continue;
      const q2 = q.map((v, i) => v + d[i]);
      const r2 = residual(q2, x);
      if (q2[5] > 0 && r2.reduce((s, v) => s + v * v, 0) < err0) return { w: q2.slice(0, 3), t: q2.slice(3), mu: mu / 10 };
    }
    return { ...est, done: true };
  }

  function drawMarker(R, t, outline, fillBits) {
    if (fillBits) {
      const cell = S / 8;
      for (let i = 0; i < 8; i++) for (let j = 0; j < 8; j++) {
        const white = i > 0 && i < 7 && j > 0 && j < 7 && BITS[j - 1][i - 1] === '1';
        const x0 = -S / 2 + i * cell, y0 = S / 2 - j * cell;
        const Q = [[x0, y0, 0], [x0 + cell, y0, 0], [x0 + cell, y0 - cell, 0], [x0, y0 - cell, 0]].map(X => project(R, t, X));
        g.fillStyle = white ? '#ffffff' : '#000000'; // a printed marker: pure black and white in both themes
        g.beginPath(); Q.forEach(([u, v], m) => (m ? g.lineTo(u, v) : g.moveTo(u, v))); g.closePath(); g.fill();
        g.strokeStyle = g.fillStyle; g.lineWidth = 0.6; g.stroke();
      }
    }
    if (outline) {
      const Q = CORNERS.map(X => project(R, t, X));
      g.strokeStyle = outline; g.lineWidth = 2.5; g.setLineDash([8, 6]);
      g.beginPath(); Q.forEach(([u, v], m) => (m ? g.lineTo(u, v) : g.moveTo(u, v))); g.closePath(); g.stroke(); g.setLineDash([]);
      const o = project(R, t, [0, 0, 0]);
      [[[S * 0.7, 0, 0], C.red], [[0, S * 0.7, 0], C.green], [[0, 0, S * 0.7], C.blue]].forEach(([d, col]) => {
        const p = project(R, t, d);
        g.strokeStyle = col; g.lineWidth = 4; g.beginPath(); g.moveTo(...o); g.lineTo(...p); g.stroke();
      });
    }
  }

  // top view (camera x horizontal, z up the canvas): the marker's x-edge for truth and estimate
  function drawTop(T, est) {
    const W = top.width, H = top.height;
    gt.fillStyle = C.bg2; gt.fillRect(0, 0, W, H);
    const sc = (H - 50) / (state.dist * 1.25), toC = (x, z) => [W / 2 + x * sc * 1.6, H - 22 - z * sc];
    const seg = (R, t, col, dash) => {
      const a = matvec(R, [-S / 2, 0, 0]).map((v, i) => v + t[i]), b = matvec(R, [S / 2, 0, 0]).map((v, i) => v + t[i]);
      const zoom = Math.max(1, 0.25 * state.dist / S); // draw the marker larger than life when far away
      const m = a.map((v, i) => (v + b[i]) / 2), A = a.map((v, i) => m[i] + (v - m[i]) * zoom), B = b.map((v, i) => m[i] + (v - m[i]) * zoom);
      gt.strokeStyle = col; gt.lineWidth = 4; gt.setLineDash(dash ? [6, 5] : []);
      gt.beginPath(); gt.moveTo(...toC(A[0], A[2])); gt.lineTo(...toC(B[0], B[2])); gt.stroke(); gt.setLineDash([]);
    };
    // camera and its field of view
    const c0 = toC(0, 0);
    gt.strokeStyle = C.dim; gt.lineWidth = 1;
    gt.beginPath(); gt.moveTo(...toC(-0.4 * state.dist * 1.25, state.dist * 1.25)); gt.lineTo(...c0); gt.lineTo(...toC(0.4 * state.dist * 1.25, state.dist * 1.25)); gt.stroke();
    gt.fillStyle = C.fg; gt.beginPath(); gt.arc(...c0, 5, 0, 7); gt.fill();
    seg(T.R, T.t, C.dim, true);
    if (est) seg(rodrigues(est.w), est.t, C.blue, false);
    gt.fillStyle = C.dim; gt.font = '19px Inter, Arial';
    gt.fillText('top view', 8, H - 52); gt.fillStyle = C.blue; gt.fillText('estimate', 8, H - 30); gt.fillStyle = C.dim; gt.fillText('true (dashed)', 8, H - 8);
    gt.fillText('camera', c0[0] + 8, c0[1] + 4);
  }

  function draw() {
    const T = truth(), x = observed();
    g.fillStyle = '#0d0f13'; g.fillRect(0, 0, IW, IH);
    drawMarker(T.R, T.t, null, true);
    x.forEach(([u, v]) => { g.strokeStyle = C.accent; g.lineWidth = 2.5; g.beginPath(); g.arc(u, v, 7, 0, 7); g.stroke(); });
    const est = state.est;
    if (est) drawMarker(rodrigues(est.w), est.t, C.blue, false);
    g.font = '26px Inter, Arial';
    g.fillStyle = C.accent; g.fillText('○ detected corners', 14, 34);
    g.fillStyle = C.blue; g.fillText('-- marker at the estimated pose', 14, 66);
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, IW - 2, IH - 2);
    drawTop(T, est);
    const e = est ? rms([...est.w, ...est.t], x) : NaN;
    const tilt = R => Math.round(Math.atan2(-R[0][2], -R[2][2]) / deg); // yaw of the marker normal about the camera's y axis
    readout.innerHTML =
      `<div>start: guess <b>${state.start}</b> · step ${state.log.length}</div>` +
      `<div>reprojection error: <b>${Number.isFinite(e) ? fmt(e, 2) + ' px' : '—'}</b></div>` +
      (est ? `<div>tilt: estimate <span style="color:${C.blue}">${tilt(rodrigues(est.w))}°</span>, true ${tilt(T.R)}°</div>` +
        `<div>distance: ${fmt(est.t[2], 2)} m (true ${fmt(T.t[2], 2)} m)</div>` : '') +
      `<div class="dim" style="margin-top:0.3em; font-family:var(--mono); font-size:0.85em; height:3.6em; overflow:hidden">${state.log.slice(-6).map(v => fmt(v, 1)).join(' → ')}</div>`;
  }

  const reset = which => { clearInterval(timer); state.start = which; state.est = initial(which); state.log = [rms([...state.est.w, ...state.est.t], observed())]; draw(); };
  const runToEnd = () => { for (let k = 0; k < 40 && !state.est.done; k++) iterate(); };
  function iterate() {
    const x = observed();
    state.est = lmStep(state.est, x);
    if (!state.est.done) state.log.push(rms([...state.est.w, ...state.est.t], x));
  }
  const solveAnimated = () => {
    clearInterval(timer);
    if (!state.est) reset(state.start);
    timer = setInterval(() => { iterate(); draw(); if (state.est.done) clearInterval(timer); }, 280);
  };

  const slider = (key, min, max, step, name, unit) => {
    const s = h('input', { type: 'range', min, max, step, value: state[key] });
    const val = h('span', { class: 'readout', style: 'display:inline-block; width:3.6em' }, `${fmt(state[key], 2)} ${unit}`);
    s.addEventListener('input', () => { state[key] = +s.value; val.textContent = `${fmt(state[key], 2)} ${unit}`; reset(state.start); });
    return h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', { style: 'width:4.2em' }, name), s, val);
  };

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${DW}px; flex:none` }, canvas,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em 0.8em' },
        h('div', { style: 'display:flex; gap:0.3em; flex-wrap:wrap' },
          h('button', { onclick: () => reset('A') }, 'start: guess A'), h('button', { onclick: () => reset('B') }, 'start: guess B'),
          h('button', { onclick: solveAnimated }, '▶ minimise'), h('button', { onclick: () => { state.seed++; reset(state.start); } }, 'new noise')),
        slider('dist', 0.25, 3, 0.05, 'distance', 'm'), slider('noise', 0, 3, 0.1, 'noise σ', 'px'))),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.4em; width:15em; flex:none' }, top, readout)));
  reset(state.start);
  if (cfg.solved ?? isPrint()) { runToEnd(); draw(); }
}
