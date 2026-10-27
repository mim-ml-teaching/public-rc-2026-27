// Why many views: triangulating one point from N bearing measurements (top view, 2D).
// N cameras stand on the line y = 0, spread over a baseline B, and each measures the direction (bearing) to the point
// P at depth Z with Gaussian noise σ. The point is estimated by nonlinear least squares on the bearings (Gauss-Newton,
// 2 unknowns); the scatter of many Monte-Carlo estimates and the 95% ellipse of the first-order covariance
// σ² (JᵀJ)⁻¹ show how the uncertainty shrinks with more views (∝ 1/√N) and, much faster, with a wider baseline
// (depth ∝ Z²/B, lecture 2), and why a narrow bundle of rays leaves the depth uncertain.
// The rays through P are the "bundle" of bundle adjustment: adjusting cameras and points until all bundles agree.
// config: {
//   width: 460, height: 340   canvas (px)
//   n: 4                      number of cameras, slider 2 … 12
//   sigma: 1.0                bearing noise (degrees), slider 0.2 … 3
//   baseline: 3               spread of the cameras (m), slider 0.5 … 10
//   depth: 6                  depth of P (m), slider 2 … 12
//   trials: 250               Monte-Carlo estimates drawn as a cloud
//   seed: 3
// }
import { C, h, fmt, isPrint } from './util.js';
import { rng } from './linalg.js';

const deg = Math.PI / 180;

export function mount(el, cfg) {
  const W = cfg.width || 460, H = cfg.height || 340;
  const state = { n: cfg.n ?? 4, sigma: cfg.sigma ?? 1.0, baseline: cfg.baseline ?? 3, depth: cfg.depth ?? 6, trials: cfg.trials ?? 250, seed: cfg.seed ?? 3 };
  const canvas = h('canvas', { width: W, height: H });
  const g = canvas.getContext('2d');
  const readout = h('div', { class: 'readout', style: 'line-height:1.45; font-size:0.92em' });

  const cams = () => Array.from({ length: state.n }, (_, j) => [state.n === 1 ? 0 : -state.baseline / 2 + state.baseline * j / (state.n - 1), 0]);
  const bearing = (c, P) => Math.atan2(P[0] - c[0], P[1] - c[1]);  // angle from the forward (+y) axis

  // Gauss-Newton on the bearings from a starting point
  function estimate(cs, meas, P0) {
    let P = P0.slice();
    for (let it = 0; it < 6; it++) {
      let a = 0, b = 0, d = 0, g0 = 0, g1 = 0;
      cs.forEach((c, j) => {
        const dx = P[0] - c[0], dy = P[1] - c[1], d2 = dx * dx + dy * dy;
        const r = Math.atan2(Math.sin(bearing(c, P) - meas[j]), Math.cos(bearing(c, P) - meas[j]));
        const jx = dy / d2, jy = -dx / d2;
        a += jx * jx; b += jx * jy; d += jy * jy; g0 += jx * r; g1 += jy * r;
      });
      const det = a * d - b * b;
      if (Math.abs(det) < 1e-18) break;
      const sx = -(d * g0 - b * g1) / det, sy = -(-b * g0 + a * g1) / det;
      P = [P[0] + sx, P[1] + sy];
      if (Math.hypot(sx, sy) < 1e-7) break;
    }
    return P;
  }
  // first-order covariance at the truth: σ² (JᵀJ)⁻¹, as [sxx, sxy, syy]
  function covariance(cs, P, s) {
    let a = 0, b = 0, d = 0;
    cs.forEach(c => { const dx = P[0] - c[0], dy = P[1] - c[1], d2 = dx * dx + dy * dy; const jx = dy / d2, jy = -dx / d2; a += jx * jx; b += jx * jy; d += jy * jy; });
    const det = a * d - b * b;
    if (Math.abs(det) < 1e-18) return [1e9, 0, 1e9];
    return [s * s * d / det, -s * s * b / det, s * s * a / det];
  }

  let cloud = [], cov = [0, 0, 0];
  function compute() {
    const cs = cams(), P = [0, state.depth], s = state.sigma * deg, r = rng(state.seed * 131 + 7);
    cloud = [];
    for (let t = 0; t < state.trials; t++) {
      const meas = cs.map(c => bearing(c, P) + s * r.normal());
      const e = estimate(cs, meas, P);
      if (Number.isFinite(e[0]) && Number.isFinite(e[1]) && Math.hypot(e[0], e[1] - state.depth) < 40) cloud.push(e);
    }
    cov = covariance(cs, P, s);
  }

  // world → canvas: x ∈ [−6.5, 6.5], y ∈ [−0.8, 13.5]
  const X0 = -6.5, X1 = 6.5, Y0 = -0.8, Y1 = 13.5;
  const sc = Math.min(W / (X1 - X0), H / (Y1 - Y0));
  const X = x => W / 2 + x * sc, Y = y => H - 8 - (y - Y0) * sc;

  function draw() {
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    const cs = cams(), P = [0, state.depth];
    // rays (the bundle)
    g.strokeStyle = C.blue; g.lineWidth = 1; g.globalAlpha = 0.55;
    cs.forEach(c => { const dx = P[0] - c[0], dy = P[1] - c[1], L = 2.2; g.beginPath(); g.moveTo(X(c[0]), Y(c[1])); g.lineTo(X(c[0] + dx * L), Y(c[1] + dy * L)); g.stroke(); });
    g.globalAlpha = 1;
    // noise cone of the first and the last camera (±2σ)
    g.fillStyle = C.blue; g.globalAlpha = 0.10;
    [cs[0], cs[cs.length - 1]].forEach(c => {
      const b = bearing(c, P), L = 20;
      g.beginPath(); g.moveTo(X(c[0]), Y(c[1]));
      g.lineTo(X(c[0] + L * Math.sin(b - 2 * state.sigma * deg)), Y(c[1] + L * Math.cos(b - 2 * state.sigma * deg)));
      g.lineTo(X(c[0] + L * Math.sin(b + 2 * state.sigma * deg)), Y(c[1] + L * Math.cos(b + 2 * state.sigma * deg)));
      g.closePath(); g.fill();
    });
    g.globalAlpha = 1;
    // cameras
    cs.forEach(c => {
      const x = X(c[0]), y = Y(c[1]), b = bearing(c, P), r = 9;
      g.save(); g.translate(x, y); g.rotate(b);
      g.fillStyle = C.blue; g.beginPath(); g.moveTo(-r * 0.7, 0); g.lineTo(r * 0.7, 0); g.lineTo(r * 0.45, -r * 1.2); g.lineTo(-r * 0.45, -r * 1.2); g.closePath(); g.fill();
      g.restore();
    });
    // the cloud of estimates
    g.fillStyle = C.accent; g.globalAlpha = 0.45;
    cloud.forEach(e => { g.beginPath(); g.arc(X(e[0]), Y(e[1]), 2, 0, 7); g.fill(); });
    g.globalAlpha = 1;
    // 95% ellipse
    const [a, b, c] = cov, tr = (a + c) / 2, dd = Math.sqrt(Math.max(0, ((a - c) / 2) ** 2 + b * b));
    const l1 = tr + dd, l2 = Math.max(0, tr - dd), ang = 0.5 * Math.atan2(2 * b, a - c), k = Math.sqrt(5.991);
    const r1 = k * Math.sqrt(l1) * sc, r2 = k * Math.sqrt(l2) * sc;
    if (r1 < W) {
      g.save(); g.translate(X(P[0]), Y(P[1])); g.rotate(-ang);
      g.beginPath(); g.ellipse(0, 0, Math.max(r1, 0.5), Math.max(r2, 0.5), 0, 0, 2 * Math.PI);
      g.strokeStyle = C.accent; g.lineWidth = 1.8; g.setLineDash([5, 4]); g.stroke(); g.setLineDash([]);
      g.restore();
    }
    // the point
    g.fillStyle = C.red; g.beginPath(); g.arc(X(P[0]), Y(P[1]), 5, 0, 7); g.fill();
    g.fillStyle = C.fg; g.font = '14px Inter, Arial, sans-serif'; g.textAlign = 'left';
    g.fillText('P', X(P[0]) + 10, Y(P[1]) - 8);
    g.fillStyle = C.dim; g.fillText(`baseline ${fmt(state.baseline, 1)} m`, X(-state.baseline / 2), Y(0) + 24);
    // scale bar
    g.strokeStyle = C.dim; g.lineWidth = 2; g.beginPath(); g.moveTo(14, 18); g.lineTo(14 + sc, 18); g.stroke();
    g.fillText('1 m', 18 + sc, 23);
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H - 2);
    // readout: std devs along / across the viewing direction (y = depth, x = lateral)
    const sdep = Math.sqrt(Math.max(cov[2], 0)), slat = Math.sqrt(Math.max(cov[0], 0));
    readout.innerHTML = `<div>${state.n} cameras, σ = ${fmt(state.sigma, 1)}°, Z = ${fmt(state.depth, 1)} m</div>` +
      `<div>uncertainty of P (1σ): <span style="color:${C.accent}">depth <b>${fmt(100 * sdep, 1)} cm</b>, sideways <b>${fmt(100 * slat, 1)} cm</b></span></div>` +
      `<div class="dim">${cloud.length} estimates from noisy bearings (dots), 95% ellipse (dashed)</div>`;
  }

  const slider = (label, key, min, max, step, fmtv) => {
    const out = h('span', { class: 'readout', style: 'text-align:right; white-space:nowrap' }, fmtv(state[key]));
    const inp = h('input', { type: 'range', min, max, step, value: state[key], style: 'width:100%; min-width:8em' });
    inp.addEventListener('input', () => { state[key] = parseFloat(inp.value); out.textContent = fmtv(state[key]); compute(); draw(); });
    return [h('label', {}, label), inp, out];
  };
  el.classList.add('widget');
  el.append(h('div', { style: `width:${W}px` }, canvas,
    h('div', { class: 'wctl interactive-only', style: 'display:grid; grid-template-columns:auto 1fr auto; gap:0.15em 0.6em; align-items:center' },
      ...slider('cameras', 'n', 2, 12, 1, v => `${v}`),
      ...slider('baseline', 'baseline', 0.5, 10, 0.5, v => `${fmt(v, 1)} m`),
      ...slider('noise σ', 'sigma', 0.2, 3, 0.1, v => `${fmt(v, 1)}°`),
      ...slider('depth Z', 'depth', 2, 12, 0.5, v => `${fmt(v, 1)} m`)),
    readout));
  compute();
  draw();
  if (isPrint()) draw();
}
