// Heading error left after an imperfect calibration, for a level vehicle turning through 360°.
// Body field at true (magnetic) heading ψ:  b = (H cos ψ, −H sin ψ, Z). The reading is distorted by
//   a leftover hard-iron offset (fixed in the body, 30° right of the nose), a leftover soft-iron error (x gain 1 + s,
//   y gain 1 − s), a small mounting tilt δ of the sensor about the roll axis (mixes Z into y), a declination error.
// Measured heading ψ̂ = atan2(−m_y, m_x) (+ declination error); error = ψ̂ − ψ.
// Left: the horizontal reading in the body frame as the vehicle turns (ideal: circle of radius H, dashed).
// Right: the heading error against ψ. The vertical field Z = H tan I is large in the north, so a tilt hurts there.
//
// config: { width (760), height (330), city ("Warsaw"), hard (mG), soft (fraction), tilt (deg), decl (deg),
//           controls: ["city", "hard", "soft", "tilt", "decl"] }
import { C, h, fmt, arrow, isPrint } from './util.js';
import { CITIES, deg } from './magutil.js';

const wrap = a => ((a + 180) % 360 + 360) % 360 - 180;

export function mount(el, cfg) {
  const W = cfg.width || 760, H = cfg.height || 330;
  const ctl = cfg.controls || ['city', 'hard', 'soft', 'tilt', 'decl'];
  const state = { city: cfg.city || 'Warsaw', hard: cfg.hard ?? 0, soft: cfg.soft ?? 0, tilt: cfg.tilt ?? 0, decl: cfg.decl ?? 0, psi: isPrint() ? 60 : 0 };
  const canvas = h('canvas', { width: W, height: H });
  const g = canvas.getContext('2d');
  const readout = h('div', { style: 'line-height:1.45' });

  function measure(psi) {
    const c = CITIES[state.city], Hh = c.F * Math.cos(c.I * deg), Z = c.F * Math.sin(c.I * deg), p = psi * deg;
    let m = [Hh * Math.cos(p), -Hh * Math.sin(p), Z];
    m = [m[0] + state.hard * Math.cos(30 * deg), m[1] + state.hard * Math.sin(30 * deg), m[2]];
    m = [m[0] * (1 + state.soft), m[1] * (1 - state.soft), m[2]];
    const d = state.tilt * deg;                                  // sensor rolled by δ: m_sensor = Rx(δ)ᵀ m
    m = [m[0], Math.cos(d) * m[1] + Math.sin(d) * m[2], -Math.sin(d) * m[1] + Math.cos(d) * m[2]];
    const est = Math.atan2(-m[1], m[0]) / deg + state.decl;
    return { m, Hh, Z, err: wrap(est - psi) };
  }

  function draw() {
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    const { Hh, Z } = measure(0);
    // ---- left: horizontal plane, body frame (x forward = up, y right = right) ----
    const cx = 175, cy = H / 2 + 12, s = 0.55;                       // px per mG
    const P = (mx, my) => [cx + my * s, cy - mx * s];
    g.strokeStyle = C.line; g.lineWidth = 1;
    g.beginPath(); g.moveTo(cx - 135, cy); g.lineTo(cx + 135, cy); g.moveTo(cx, cy - 130); g.lineTo(cx, cy + 135); g.stroke();
    g.fillStyle = C.dim; g.font = '15px Inter, Arial';
    g.fillText('x (nose)', cx + 6, cy - 118); g.fillText('y (right)', cx + 70, cy + 20);
    g.setLineDash([6, 5]); g.strokeStyle = C.dim; g.lineWidth = 1.5;
    g.beginPath(); g.arc(cx, cy, Hh * s, 0, 7); g.stroke(); g.setLineDash([]);
    g.strokeStyle = C.accent; g.lineWidth = 2.5; g.beginPath();
    for (let a = 0; a <= 360; a += 3) { const { m } = measure(a); const [u, v] = P(m[0], m[1]); a ? g.lineTo(u, v) : g.moveTo(u, v); }
    g.stroke();
    const cur = measure(state.psi), pr = state.psi * deg;
    const [iu, iv] = P(Hh * Math.cos(pr), -Hh * Math.sin(pr)), [mu, mv] = P(cur.m[0], cur.m[1]);
    arrow(g, cx, cy, iu, iv, C.dim, 2); arrow(g, cx, cy, mu, mv, C.accent, 3);
    g.fillStyle = C.fg; g.font = '600 15px Inter, Arial'; g.fillText('horizontal reading, body frame', 12, 22);

    // ---- right: heading error vs heading ----
    const x0 = 370, x1 = W - 16, y0 = 40, y1 = H - 40;
    let mx = 2;
    for (let a = 0; a < 360; a += 2) mx = Math.max(mx, Math.abs(measure(a).err));
    const ymax = Math.ceil(mx * 1.15);
    const X = a => x0 + (x1 - x0) * a / 360, Y = e => (y0 + y1) / 2 - (y1 - y0) / 2 * e / ymax;
    g.strokeStyle = C.line; g.lineWidth = 1; g.fillStyle = C.dim; g.font = '14px Inter, Arial';
    for (const a of [0, 90, 180, 270, 360]) { g.beginPath(); g.moveTo(X(a), y0); g.lineTo(X(a), y1); g.stroke(); g.fillText(`${a}°`, X(a) - 12, y1 + 18); }
    for (const e of [-ymax, 0, ymax]) { g.beginPath(); g.moveTo(x0, Y(e)); g.lineTo(x1, Y(e)); g.stroke(); g.fillText(`${e > 0 ? '+' : ''}${e}°`, x0 - 34, Y(e) + 5); }
    g.strokeStyle = C.red; g.lineWidth = 2.5; g.beginPath();
    for (let a = 0; a <= 360; a += 2) { const e = measure(a).err; a ? g.lineTo(X(a), Y(e)) : g.moveTo(X(a), Y(e)); }
    g.stroke();
    g.fillStyle = C.fg; g.beginPath(); g.arc(X(state.psi), Y(cur.err), 6, 0, 7); g.fill();
    g.fillStyle = C.fg; g.font = '600 15px Inter, Arial'; g.fillText('heading error vs true heading ψ', x0, 22);
    g.fillStyle = C.dim; g.font = '14px Inter, Arial'; g.fillText('ψ', x1 - 10, y1 + 34);

    const c = CITIES[state.city], tanI = Math.tan(c.I * deg);
    readout.innerHTML =
      `<div class="dim">${state.city}: $H = ${fmt(Hh, 0)}$, $Z = ${fmt(Z, 0)}$ mG, $\\tan I = ${fmt(tanI, 2)}$</div>` +
      `<div>max |error| <b>${fmt(mx, 1)}°</b></div>` +
      `<div class="dim" style="font-size:0.95em">hard iron: $\\le \\arcsin(e/H) = ${fmt(Math.asin(Math.min(1, state.hard / Hh)) / deg, 1)}°$<br>` +
      `tilt: $\\approx \\delta \\tan I = ${fmt(state.tilt * tanI, 1)}°$</div>`;
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
  }

  const slider = (key, lab, min, max, step, unit, k = 1) => {
    const val = h('span', { class: 'readout', style: 'display:inline-block; width:4em' }, `${fmt(state[key] * k, 1)}${unit}`);
    const sl = h('input', { type: 'range', min, max, step, value: state[key], style: 'width:8em' });
    sl.addEventListener('input', () => { state[key] = +sl.value; val.textContent = `${fmt(state[key] * k, 1)}${unit}`; draw(); });
    return h('div', { style: 'display:flex; align-items:center; gap:0.35em' }, h('label', { style: 'width:5.2em' }, lab), sl, val);
  };
  const rows = [];
  if (ctl.includes('city')) {
    const btns = Object.keys(CITIES).map(name => {
      const b = h('button', { onclick: () => { state.city = name; btns.forEach(x => x.classList.toggle('active', x === b)); draw(); } }, name);
      b.classList.toggle('active', name === state.city);
      return b;
    });
    rows.push(h('div', { style: 'display:flex; gap:0.25em; flex-wrap:wrap' }, btns));
  }
  if (ctl.includes('hard')) rows.push(slider('hard', 'hard iron', 0, 60, 1, ' mG'));
  if (ctl.includes('soft')) rows.push(slider('soft', 'soft iron', 0, 0.2, 0.005, ' %', 100));
  if (ctl.includes('tilt')) rows.push(slider('tilt', 'tilt δ', 0, 5, 0.1, '°'));
  if (ctl.includes('decl')) rows.push(slider('decl', 'decl. error', 0, 10, 0.1, '°'));

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    canvas,
    h('div', { style: 'display:flex; flex-direction:column; gap:0.45em; width:15em; flex:none' },
      readout, h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.3em' }, rows))));
  draw();
  if (isPrint()) return;
  const t0 = performance.now();
  const loop = now => {
    if (!canvas.isConnected) return;
    state.psi = ((now - t0) / 40) % 360; draw();
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}
