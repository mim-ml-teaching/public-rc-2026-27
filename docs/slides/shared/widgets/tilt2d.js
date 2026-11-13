// Tilt to translate: a multicopter can push only along its own "up" axis. Side view of a quadcopter tilted by θ
// while holding its altitude: the thrust T must have a vertical component equal to the weight (T cos θ = m g), so
// T = m g / cos θ, and the horizontal component T sin θ = m g tan θ accelerates it sideways.
// config: { width: 560, height: 360, tilt: 20 }
import { C, h, fmt } from './util.js';
import { S } from './plotsvg.js';

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 560, H = cfg.height || 360, st = { tilt: cfg.tilt ?? 20 };
  const svg = S('svg', { width: W, height: H, viewBox: `0 0 ${W} ${H}`, style: 'display:block; font-family:Inter, Arial, sans-serif' });
  const readout = h('div', { class: 'readout', style: 'line-height:1.5; min-height:4.6em' });
  const val = h('span', { class: 'readout', style: 'display:inline-block; width:3.5em' });
  const sl = h('input', { type: 'range', min: 0, max: 50, step: 1, value: st.tilt, oninput: e => { st.tilt = +e.target.value; draw(); } });
  el.append(svg, readout, h('div', { class: 'wctl interactive-only' }, h('label', {}, 'tilt θ '), sl, val));

  const arrowS = (x0, y0, x1, y1, color, width = 4, label, lx = 0, ly = 0) => {
    const a = Math.atan2(y1 - y0, x1 - x0), L = Math.hypot(x1 - x0, y1 - y0);
    if (L < 2) return;
    const hd = Math.min(14, L * 0.45);
    svg.append(S('line', { x1: x0, y1: y0, x2: x1 - hd * 0.7 * Math.cos(a), y2: y1 - hd * 0.7 * Math.sin(a), stroke: color, 'stroke-width': width }));
    svg.append(S('path', { d: `M${x1},${y1} L${x1 - hd * Math.cos(a - 0.4)},${y1 - hd * Math.sin(a - 0.4)} L${x1 - hd * Math.cos(a + 0.4)},${y1 - hd * Math.sin(a + 0.4)} Z`, fill: color }));
    if (label) svg.append(S('text', { x: x1 + lx, y: y1 + ly, fill: color, 'font-size': 17, 'font-weight': 600 }, label));
  };

  function draw() {
    svg.replaceChildren();
    const th = st.tilt * Math.PI / 180, cx = W * 0.4, cy = H * 0.52, k = 135;   // k px per m g
    svg.append(S('line', { x1: 10, x2: W - 10, y1: H - 18, y2: H - 18, stroke: C.line, 'stroke-width': 2 }));
    // body: a bar with two rotors, rotated by θ (nose to the right goes down: moves right)
    const g = S('g', { transform: `rotate(${st.tilt} ${cx} ${cy})` });
    g.append(S('rect', { x: cx - 110, y: cy - 7, width: 220, height: 14, rx: 5, fill: C.bg3, stroke: C.dim, 'stroke-width': 2 }));
    g.append(S('rect', { x: cx - 24, y: cy - 16, width: 48, height: 26, rx: 4, fill: C.bg3, stroke: C.dim, 'stroke-width': 2 }));
    for (const dx of [-110, 110]) {
      g.append(S('rect', { x: cx + dx - 5, y: cy - 22, width: 10, height: 16, fill: C.dim }));
      g.append(S('ellipse', { cx: cx + dx, cy: cy - 24, rx: 55, ry: 6, fill: 'none', stroke: C.dim, 'stroke-width': 2 }));
    }
    svg.append(g);
    // forces (units of m g)
    const T = 1 / Math.cos(th);
    const tx = cx + Math.sin(th) * T * k, ty = cy - Math.cos(th) * T * k;
    arrowS(cx, cy + 2, cx, cy + 2 + k, C.blue, 4, 'm g', 8, 0);
    svg.append(S('line', { x1: cx, y1: cy, x2: cx, y2: ty, stroke: C.green, 'stroke-width': 2, 'stroke-dasharray': '6 5' }));
    svg.append(S('line', { x1: cx, y1: ty, x2: tx, y2: ty, stroke: C.red, 'stroke-width': 2, 'stroke-dasharray': '6 5' }));
    arrowS(cx, cy, tx, ty, C.accent, 5, 'T', 12, 14);
    if (st.tilt > 3) arrowS(cx + 150, cy + 90, cx + 150 + Math.tan(th) * k * 0.8, cy + 90, C.red, 4, 'a', 8, 6);
    svg.append(S('text', { x: cx - 10, y: ty + 22, fill: C.green, 'font-size': 17, 'text-anchor': 'end' }, 'T cos θ = m g'));
    if (st.tilt > 3) svg.append(S('text', { x: tx - 4, y: ty - 10, fill: C.red, 'font-size': 17, 'text-anchor': 'end' }, 'T sin θ'));
    const a = 9.81 * Math.tan(th);
    val.textContent = `${st.tilt}°`;
    readout.innerHTML = `thrust T = m g / cos θ = <b style="color:${C.accent}">${fmt(T, 2)} m g</b><br>` +
      `sideways acceleration a = g tan θ = <b style="color:${C.red}">${fmt(a, 2)} m/s²</b><br>` +
      `after 3 s: ${fmt(a * 3, 1)} m/s (${fmt(a * 3 * 3.6, 0)} km/h)`;
  }
  draw();
}
