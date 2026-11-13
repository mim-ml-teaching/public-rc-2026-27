// What an accelerometer on a multicopter reads. It measures specific force f = a − g (all non-gravitational force
// per unit mass), in the body frame. Two vehicles, both tilted by θ:
//   left:  standing still on a slope. The ground pushes up with m g, so f = g "up" in the world; in the body frame
//          it is tilted by θ: the tilt is visible.
//   right: flying, tilted by θ and holding altitude. The only non-gravitational force is the thrust, along the body
//          "up" axis, so f points straight up the body axis whatever θ is: the accelerometer says "level".
// (Rotor drag adds a small sideways component in real flight; ignored here, it is what some estimators exploit.)
// config: { width: 980, height: 360, tilt: 25 }
import { C, h, fmt } from './util.js';
import { S } from './plotsvg.js';

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 980, H = cfg.height || 360, st = { tilt: cfg.tilt ?? 25 };
  const svg = S('svg', { width: W, height: H, viewBox: `0 0 ${W} ${H}`, style: 'display:block; font-family:Inter, Arial, sans-serif' });
  const val = h('span', { class: 'readout', style: 'display:inline-block; width:3.5em' });
  const sl = h('input', { type: 'range', min: 0, max: 45, step: 1, value: st.tilt, oninput: e => { st.tilt = +e.target.value; draw(); } });
  el.append(svg, h('div', { class: 'wctl interactive-only' }, h('label', {}, 'tilt θ '), sl, val));

  const arrowS = (x0, y0, x1, y1, color, width = 4, label, lx = 8, ly = 0, anchor = 'start') => {
    const a = Math.atan2(y1 - y0, x1 - x0), L = Math.hypot(x1 - x0, y1 - y0);
    if (L < 3) return;
    const hd = Math.min(14, L * 0.45);
    svg.append(S('line', { x1: x0, y1: y0, x2: x1 - hd * 0.7 * Math.cos(a), y2: y1 - hd * 0.7 * Math.sin(a), stroke: color, 'stroke-width': width }));
    svg.append(S('path', { d: `M${x1},${y1} L${x1 - hd * Math.cos(a - 0.4)},${y1 - hd * Math.sin(a - 0.4)} L${x1 - hd * Math.cos(a + 0.4)},${y1 - hd * Math.sin(a + 0.4)} Z`, fill: color }));
    if (label) svg.append(S('text', { x: x1 + lx, y: y1 + ly, fill: color, 'font-size': 16, 'font-weight': 600, 'text-anchor': anchor }, label));
  };
  const body = (cx, cy, deg) => {
    const g = S('g', { transform: `rotate(${deg} ${cx} ${cy})` });
    g.append(S('rect', { x: cx - 70, y: cy - 6, width: 140, height: 12, rx: 5, fill: C.bg3, stroke: C.dim, 'stroke-width': 2 }));
    g.append(S('rect', { x: cx - 16, y: cy - 13, width: 32, height: 20, rx: 4, fill: C.bg3, stroke: C.dim, 'stroke-width': 2 }));
    for (const dx of [-70, 70]) {
      g.append(S('rect', { x: cx + dx - 4, y: cy - 20, width: 8, height: 14, fill: C.dim }));
      g.append(S('ellipse', { cx: cx + dx, cy: cy - 22, rx: 36, ry: 4.5, fill: 'none', stroke: C.dim, 'stroke-width': 2 }));
    }
    g.append(S('line', { x1: cx, y1: cy, x2: cx, y2: cy - 150, stroke: C.dim, 'stroke-width': 1.5, 'stroke-dasharray': '4 5' }));
    g.append(S('text', { x: cx - 6, y: cy - 140, fill: C.dim, 'font-size': 14, 'text-anchor': 'end' }, 'body up'));
    svg.append(g);
  };

  function draw() {
    svg.replaceChildren();
    const th = st.tilt * Math.PI / 180, k = 100, pw = W / 2;
    val.textContent = `${st.tilt}°`;
    svg.append(S('line', { x1: pw, x2: pw, y1: 10, y2: H - 10, stroke: C.line }));
    // left: on a slope
    {
      const cx = pw * 0.48, cy = H * 0.6;
      svg.append(S('text', { x: pw / 2, y: 24, fill: C.accent, 'font-size': 19, 'font-weight': 600, 'text-anchor': 'middle' }, 'standing on a slope'));
      const base = cy + 30, L = 220;
      // slope line through (cx, base) rising to the left (vehicle nose to the right points down the slope)
      svg.append(S('line', { x1: cx - L * Math.cos(th), y1: base - L * Math.sin(th), x2: cx + L * Math.cos(th), y2: base + L * Math.sin(th), stroke: C.dim, 'stroke-width': 3 }));
      body(cx, base - 20 / Math.cos(th), st.tilt);
      const bx = cx, by = base - 20 / Math.cos(th);
      arrowS(bx, by, bx, by + 0.7 * k, C.blue, 3, 'gravity', 8, 0);
      arrowS(bx, by, bx, by - k, C.purple, 5, 'f (reads g, up)', 10, 4);
      svg.append(S('text', { x: pw / 2, y: H - 14, fill: C.green, 'font-size': 17, 'text-anchor': 'middle' },
        `tilt from accelerometer: ${fmt(st.tilt, 0)}°  ✓`));
    }
    // right: flying, tilted, holding altitude
    {
      const cx = pw + pw * 0.45, cy = H * 0.6;
      svg.append(S('text', { x: pw + pw / 2, y: 24, fill: C.accent, 'font-size': 19, 'font-weight': 600, 'text-anchor': 'middle' }, 'flying, tilted, holding altitude'));
      body(cx, cy, st.tilt);
      const T = 1 / Math.cos(th);
      arrowS(cx, cy, cx, cy + 0.7 * k, C.blue, 3, 'gravity', 8, 0);
      arrowS(cx, cy, cx + Math.sin(th) * T * k * 0.8, cy - Math.cos(th) * T * k * 0.8, C.purple, 5, 'f = T / m', 10, 4);
      if (st.tilt > 2) arrowS(cx + 120, cy + 70, cx + 120 + Math.tan(th) * 0.7 * k, cy + 70, C.red, 4, `a = ${fmt(9.81 * Math.tan(th), 1)} m/s²`, -10, 24, 'end');
      svg.append(S('text', { x: pw + pw / 2, y: H - 14, fill: C.red, 'font-size': 17, 'text-anchor': 'middle' },
        `tilt from accelerometer: 0°  (true: ${fmt(st.tilt, 0)}°)`));
    }
  }
  draw();
}
