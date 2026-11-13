// Motor mixing for a quadcopter in the ArduPilot QUAD/X layout, top view, nose up on the screen.
// Body frame FRD: x forward, y right, z down. Motors: 1 front-right CCW, 2 rear-left CCW, 3 front-left CW,
// 4 rear-right CW (spin seen from above). Mixing (unit factors; ArduPilot uses ±0.707 for roll/pitch and rescales):
//   m1 = T − R + P + Y,  m2 = T + R − P + Y,  m3 = T + R + P − Y,  m4 = T − R − P − Y
// R > 0 rolls right (right side down), P > 0 pitches the nose up, Y > 0 yaws the nose right (clockwise from above).
// Outputs are clipped to [0, 1]; the "achieved" T, R, P, Y are recomputed from the clipped outputs, so saturation
// shows up as requested ≠ achieved.
// config: { width: 440, height: 400, T: 0.5, R: 0, P: 0, Y: 0, stack: false (true: table and sliders below the drawing) }
import { C, h, fmt } from './util.js';
import { S } from './plotsvg.js';

export const MOTORS = [
  { n: 1, x: 1, y: 1, spin: 'CCW', r: -1, p: 1, yaw: 1 },
  { n: 2, x: -1, y: -1, spin: 'CCW', r: 1, p: -1, yaw: 1 },
  { n: 3, x: 1, y: -1, spin: 'CW', r: 1, p: 1, yaw: -1 },
  { n: 4, x: -1, y: 1, spin: 'CW', r: -1, p: -1, yaw: -1 },
];

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 440, H = cfg.height || 400;
  const st = { T: cfg.T ?? 0.5, R: cfg.R ?? 0, P: cfg.P ?? 0, Y: cfg.Y ?? 0 };
  const svg = S('svg', { width: W, height: H, viewBox: `0 0 ${W} ${H}`, style: 'display:block; flex:none; font-family:Inter, Arial, sans-serif' });
  const table = h('div', { class: 'readout', style: 'line-height:1.45; width:15em' });
  const rows = [];
  const slider = (key, label, min, max, color) => {
    const v = h('span', { class: 'readout', style: 'display:inline-block; width:3.2em' });
    const s = h('input', { type: 'range', min, max, step: 0.05, value: st[key], style: 'width:8em', oninput: e => { st[key] = +e.target.value; draw(); } });
    rows.push(h('div', {}, h('label', { style: `display:inline-block; width:4.6em; color:${color}` }, label), s, v));
    return v;
  };
  const vals = { T: slider('T', 'thrust', 0, 1, C.fg), R: slider('R', 'roll', -0.5, 0.5, C.red), P: slider('P', 'pitch', -0.5, 0.5, C.green), Y: slider('Y', 'yaw', -0.5, 0.5, C.blue) };
  const ctl = h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.25em' }, rows);
  if (cfg.stack) el.append(svg, h('div', { style: 'display:flex; gap:1.2em; align-items:flex-start; font-size:0.85em' }, table, ctl));
  else el.append(h('div', { style: 'display:flex; gap:1.2em; align-items:flex-start' }, svg, h('div', {}, table, ctl)));

  function draw() {
    svg.replaceChildren();
    const cx = W / 2, cy = H / 2 + 6, d = Math.min(W, H) * 0.26, R0 = Math.min(W, H) * 0.13;
    const P = m => [cx + m.y * d, cy - m.x * d];          // y right → screen right, x forward → screen up
    // arms and body
    for (const m of MOTORS) { const [x, y] = P(m); svg.append(S('line', { x1: cx, y1: cy, x2: x, y2: y, stroke: C.dim, 'stroke-width': 8, 'stroke-linecap': 'round' })); }
    svg.append(S('rect', { x: cx - 26, y: cy - 34, width: 52, height: 68, rx: 10, fill: C.bg3, stroke: C.dim, 'stroke-width': 2 }));
    svg.append(S('path', { d: `M${cx},${cy - 28} L${cx - 10},${cy - 10} L${cx + 10},${cy - 10} Z`, fill: C.accent }));
    svg.append(S('text', { x: cx, y: 18, fill: C.accent, 'font-size': 16, 'text-anchor': 'middle', 'font-weight': 600 }, 'front (x)'));
    const out = MOTORS.map(m => {
      const raw = st.T + m.r * st.R + m.p * st.P + m.yaw * st.Y;
      return { raw, v: Math.max(0, Math.min(1, raw)) };
    });
    MOTORS.forEach((m, i) => {
      const [x, y] = P(m), o = out[i], sat = o.raw > 1.0001 || o.raw < -0.0001;
      const col = m.spin === 'CCW' ? C.purple : C.green;
      svg.append(S('circle', { cx: x, cy: y, r: R0, fill: col, 'fill-opacity': 0.08 + 0.42 * o.v, stroke: sat ? C.red : col, 'stroke-width': sat ? 5 : 2 }));
      // spin arrow: an arc with an arrow head, CW or CCW as seen from above
      const r = R0 * 0.72, ccw = m.spin === 'CCW', a0 = -0.6, a1 = a0 + (ccw ? -1 : 1) * 4.2;
      const pt = a => [x + r * Math.cos(a), y + r * Math.sin(a)];
      const [sx, sy] = pt(a0), [ex, ey] = pt(a1);
      svg.append(S('path', { d: `M${sx},${sy} A${r},${r} 0 1 ${ccw ? 0 : 1} ${ex},${ey}`, fill: 'none', stroke: C.fg, 'stroke-width': 2 }));
      const ta = a1 + (ccw ? -1 : 1) * Math.PI / 2, hx = ex + 9 * Math.cos(ta), hy = ey + 9 * Math.sin(ta);
      svg.append(S('path', { d: `M${hx},${hy} L${ex + 6 * Math.cos(ta + 2.3)},${ey + 6 * Math.sin(ta + 2.3)} L${ex + 6 * Math.cos(ta - 2.3)},${ey + 6 * Math.sin(ta - 2.3)} Z`, fill: C.fg }));
      svg.append(S('text', { x, y: y + 7, fill: C.fg, 'font-size': 22, 'font-weight': 700, 'text-anchor': 'middle' }, String(m.n)));
      const ly = m.x > 0 ? y - R0 - 10 : y + R0 + 22;
      svg.append(S('text', { x, y: ly, fill: sat ? C.red : C.dim, 'font-size': 15, 'text-anchor': 'middle' }, `${m.spin} · ${fmt(o.v, 2)}${sat ? ' (clipped)' : ''}`));
    });
    // achieved commands from the clipped outputs (the mixing matrix is orthogonal up to a factor 4)
    const ach = { T: 0, R: 0, P: 0, Y: 0 };
    MOTORS.forEach((m, i) => { ach.T += out[i].v / 4; ach.R += m.r * out[i].v / 4; ach.P += m.p * out[i].v / 4; ach.Y += m.yaw * out[i].v / 4; });
    const line = (k, label, color) => {
      const bad = Math.abs(ach[k] - st[k]) > 0.005;
      return `<span style="color:${color}">${label}</span> ${fmt(st[k], 2).padStart(5, ' ')} → <b style="color:${bad ? C.red : C.fg}">${fmt(ach[k], 2)}</b>`;
    };
    table.innerHTML = `requested → achieved<br>${line('T', 'T', C.fg)}<br>${line('R', 'R', C.red)}<br>${line('P', 'P', C.green)}<br>${line('Y', 'Y', C.blue)}`;
    for (const k of ['T', 'R', 'P', 'Y']) vals[k].textContent = fmt(st[k], 2);
  }
  draw();
}
