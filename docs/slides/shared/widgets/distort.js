// Lens distortion (Brown–Conrady, as in OpenCV) applied to a grid in normalised image coordinates.
//   r² = x² + y²
//   x_d = x (1 + k1 r² + k2 r⁴ + k3 r⁶) + 2 p1 x y + p2 (r² + 2x²)
//   y_d = y (1 + k1 r² + k2 r⁴ + k3 r⁶) + p1 (r² + 2y²) + 2 p2 x y
// Grid lines that start outside the field of view are drawn red: if the polynomial is not monotonic
// they can land inside the image ("points outside the view reappear").
// config: { preset: "barrel" | "pincushion" | "tangential" | "foldover" | "none", showOutside: false }
import { C, h, fmt } from './util.js';

const PRESETS = {
  none: { k1: 0, k2: 0, k3: 0, p1: 0, p2: 0 },
  barrel: { k1: -0.25, k2: 0.03, k3: 0, p1: 0, p2: 0 },
  pincushion: { k1: 0.25, k2: 0.05, k3: 0, p1: 0, p2: 0 },
  tangential: { k1: 0, k2: 0, k3: 0, p1: 0.04, p2: 0.025 },
  foldover: { k1: -0.35, k2: 0, k3: 0, p1: 0, p2: 0 },
};

export function mount(el, cfg) {
  const W = cfg.width || 520, H = cfg.height || 390, F = W / 1.8; // px per normalised unit
  const xMax = W / 2 / F, yMax = H / 2 / F;
  const state = { ...PRESETS[cfg.preset || 'barrel'], outside: cfg.showOutside ?? cfg.preset === 'foldover' };
  const canvas = h('canvas', { width: W, height: H });
  const plot = h('canvas', { width: 240, height: 180 });
  const g = canvas.getContext('2d'), gp = plot.getContext('2d');
  const readout = h('div', { class: 'readout' });

  const distort = (x, y) => {
    const { k1, k2, k3, p1, p2 } = state, r2 = x * x + y * y;
    const rad = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
    return [x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x), y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y];
  };
  const R_OUT = 1.45; // outside points are drawn only up to this radius
  const toC = ([x, y]) => [W / 2 + x * F, H / 2 + y * F];
  const inside = (x, y) => Math.abs(x) <= xMax && Math.abs(y) <= yMax;

  function drawLine(pts) {
    // pts: ideal points along a straight line; colour per segment by whether the ideal point is in view
    for (let i = 1; i < pts.length; i++) {
      const [a, b] = [pts[i - 1], pts[i]];
      const inView = inside(...a) && inside(...b);
      if (!inView && (!state.outside || Math.hypot(...a) > R_OUT || Math.hypot(...b) > R_OUT)) continue;
      const [u0, v0] = toC(distort(...a)), [u1, v1] = toC(distort(...b));
      g.strokeStyle = inView ? C.accent : 'rgba(255,107,107,0.75)'; g.lineWidth = inView ? 2 : 1.2;
      g.beginPath(); g.moveTo(u0, v0); g.lineTo(u1, v1); g.stroke();
    }
  }

  function draw() {
    g.fillStyle = '#0d0f13'; g.fillRect(0, 0, W, H);
    // ideal grid (faint)
    const step = 0.15, ext = 1.5;
    g.strokeStyle = '#2d313a'; g.lineWidth = 1;
    for (let x = -ext; x <= ext + 1e-9; x += step) { const [u] = toC([x, 0]); g.beginPath(); g.moveTo(u, 0); g.lineTo(u, H); g.stroke(); }
    for (let y = -ext; y <= ext + 1e-9; y += step) { const [, v] = toC([0, y]); g.beginPath(); g.moveTo(0, v); g.lineTo(W, v); g.stroke(); }
    // distorted grid
    const N = 240;
    for (let x = -ext; x <= ext + 1e-9; x += step) drawLine(Array.from({ length: N + 1 }, (_, i) => [x, -ext + 2 * ext * i / N]));
    for (let y = -ext; y <= ext + 1e-9; y += step) drawLine(Array.from({ length: N + 1 }, (_, i) => [-ext + 2 * ext * i / N, y]));
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H - 2);

    // r_d(r) along the diagonal direction (radial part only)
    const PW = plot.width, PH = plot.height, rMax = 2, pad = 26;
    const px = r => pad + (r / rMax) * (PW - pad - 8), py = r => PH - pad - (r / rMax) * (PH - pad - 8);
    gp.fillStyle = C.bg2; gp.fillRect(0, 0, PW, PH);
    gp.strokeStyle = C.dim; gp.lineWidth = 1;
    gp.beginPath(); gp.moveTo(pad, 6); gp.lineTo(pad, PH - pad); gp.lineTo(PW - 6, PH - pad); gp.stroke();
    gp.setLineDash([4, 4]); gp.beginPath(); gp.moveTo(px(0), py(0)); gp.lineTo(px(rMax), py(rMax)); gp.stroke(); gp.setLineDash([]);
    const rc = Math.hypot(xMax, yMax);
    gp.strokeStyle = C.purple; gp.beginPath(); gp.moveTo(px(rc), 6); gp.lineTo(px(rc), PH - pad); gp.stroke();
    gp.strokeStyle = C.accent; gp.lineWidth = 2; gp.beginPath();
    for (let i = 0; i <= 200; i++) {
      const r = rMax * i / 200, r2 = r * r, rd = r * (1 + state.k1 * r2 + state.k2 * r2 * r2 + state.k3 * r2 * r2 * r2);
      const y = Math.max(-0.2, Math.min(rMax * 1.05, rd));
      i ? gp.lineTo(px(r), py(y)) : gp.moveTo(px(r), py(y));
    }
    gp.stroke();
    gp.fillStyle = C.dim; gp.font = '13px Inter, Arial';
    gp.fillText('r', PW - 16, PH - pad + 16); gp.fillText('r_d', 2, 14);
    gp.fillStyle = C.purple; gp.fillText('image corner', px(rc) + 4, PH - pad - 6);

    readout.textContent = ['k1', 'k2', 'k3', 'p1', 'p2'].map(k => `${k}=${fmt(state[k], 3)}`).join('  ');
  }

  const sliders = {};
  const mk = (key, min, max) => {
    const s = h('input', { type: 'range', min, max, step: (max - min) / 400, value: state[key] });
    s.addEventListener('input', () => { state[key] = parseFloat(s.value); draw(); });
    sliders[key] = s;
    return h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', { style: 'width:1.8em' }, key), s);
  };
  const presetBtns = Object.keys(PRESETS).map(n => h('button', {
    onclick: () => {
      Object.assign(state, PRESETS[n]);
      if (n === 'foldover') state.outside = true;
      Object.entries(sliders).forEach(([k, s]) => { s.value = state[k]; });
      outsideBtn.classList.toggle('active', state.outside); draw();
    },
  }, n === 'foldover' ? 'fold-over' : n));
  const outsideBtn = h('button', { onclick: () => { state.outside = !state.outside; outsideBtn.classList.toggle('active', state.outside); draw(); } }, 'show points outside the view');
  outsideBtn.classList.toggle('active', state.outside);

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    canvas,
    h('div', { style: 'display:flex; flex-direction:column; align-items:flex-start; gap:0.4em; width:15em; flex:none' },
      plot, readout,
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.25em' },
        h('div', { style: 'display:flex; flex-wrap:wrap; gap:0.3em' }, presetBtns),
        mk('k1', -0.6, 0.6), mk('k2', -0.3, 0.3), mk('k3', -0.1, 0.1), mk('p1', -0.08, 0.08), mk('p2', -0.08, 0.08),
        outsideBtn))));
  draw();
}
