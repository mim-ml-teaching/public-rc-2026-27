// Reaction torque, animated (top view). Left: two propellers spinning the same way (CW): the frame turns the other
// way (CCW), speeding up until the air drag on the frame balances the torque. Right: one CW and one CCW propeller:
// the torques cancel and the frame stays still; "speed up the CCW rotor" makes its (CW) reaction torque win and
// the frame yaws slowly CW. Toy model: frame rate Ω → Ω* = −k Σ s_i n_i² (s = +1 for CW) with a 1.2 s lag;
// propellers drawn at a slow, readable speed. Screen y points down, so a positive SVG rotation is CW seen from above.
// config: { width: 560, height: 330 }
import { C, h, isPrint } from './util.js';
import { S } from './plotsvg.js';

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 560, H = cfg.height || 330;
  const svg = S('svg', { viewBox: `0 0 ${W} ${H}`, style: 'width:100%; display:block' });
  const panels = [
    { cx: W * 0.25, title: 'same direction', spin: [1, 1], boost: [1, 1] },
    { cx: W * 0.75, title: 'opposite directions', spin: [1, -1], boost: [1, 1] },
  ];
  const st = { boost: false, t: 0 };
  for (const p of panels) { p.psi = 0; p.om = 0; p.phi = [0, 0.7]; }
  const btn = h('button', { onclick: () => { st.boost = !st.boost; btn.classList.toggle('active', st.boost); btn.textContent = st.boost ? 'equal speeds again' : 'speed up the CCW rotor'; } }, 'speed up the CCW rotor');
  const reset = h('button', { onclick: () => panels.forEach(p => { p.psi = 0; p.om = 0; }) }, 'reset');
  el.append(svg, h('div', { class: 'wctl interactive-only', style: 'justify-content:flex-end' }, btn, reset));

  const arc = (r, a0, a1, color, w = 3) => {          // arc with an arrow head at a1 (radians, SVG angles)
    const P = a => [r * Math.cos(a), r * Math.sin(a)], [x0, y0] = P(a0), [x1, y1] = P(a1), large = Math.abs(a1 - a0) > Math.PI ? 1 : 0;
    const g = S('g');
    g.append(S('path', { d: `M${x0},${y0} A${r},${r} 0 ${large} ${a1 > a0 ? 1 : 0} ${x1},${y1}`, fill: 'none', stroke: color, 'stroke-width': w }));
    const dir = a1 > a0 ? 1 : -1, tx = -Math.sin(a1) * dir, ty = Math.cos(a1) * dir, nx = Math.cos(a1), ny = Math.sin(a1);
    g.append(S('path', { d: `M${x1 + tx * 9},${y1 + ty * 9} L${x1 - tx * 2 + nx * 6},${y1 - ty * 2 + ny * 6} L${x1 - tx * 2 - nx * 6},${y1 - ty * 2 - ny * 6}Z`, fill: color }));
    return g;
  };

  function step(dt) {
    st.t += dt;
    panels.forEach((p, k) => {
      const n = p.spin.map((s, i) => (k === 1 && st.boost && s < 0 ? 1.15 : 1));
      p.phi = p.phi.map((a, i) => a + p.spin[i] * n[i] * 7 * dt);
      const target = -0.9 * p.spin.reduce((acc, s, i) => acc + s * n[i] * n[i], 0) / 2;   // rad/s
      p.om += (target - p.om) * dt / 1.2;
      p.psi += p.om * dt;
    });
  }
  function draw() {
    svg.replaceChildren();
    svg.append(S('line', { x1: W / 2, x2: W / 2, y1: 10, y2: H - 10, stroke: C.line }));
    panels.forEach((p, k) => {
      const cy = 145, arm = 78, deg = p.psi * 180 / Math.PI;
      svg.append(S('text', { x: p.cx, y: 24, fill: C.fg, 'font-size': 17, 'font-weight': 600, 'text-anchor': 'middle' }, p.title));
      const g = S('g', { transform: `translate(${p.cx},${cy}) rotate(${deg.toFixed(2)})` });
      g.append(S('line', { x1: -arm, x2: arm, y1: 0, y2: 0, stroke: C.dim, 'stroke-width': 9, 'stroke-linecap': 'round' }));
      g.append(S('rect', { x: -16, y: -12, width: 32, height: 24, rx: 5, fill: C.bg3, stroke: C.fg }));
      [-1, 1].forEach((sx, i) => {
        const col = p.spin[i] > 0 ? C.green : C.purple, rg = S('g', { transform: `translate(${sx * arm},0)` });
        rg.append(S('circle', { r: 40, fill: col, 'fill-opacity': 0.1, stroke: col, 'stroke-width': 1.5 }));
        const blades = S('g', { transform: `rotate(${(p.phi[i] * 180 / Math.PI).toFixed(1)})` });
        blades.append(S('rect', { x: -38, y: -4, width: 76, height: 8, rx: 4, fill: col, 'fill-opacity': 0.85 }));
        rg.append(blades, S('circle', { r: 6, fill: C.fg }));
        const a0 = -2.2, a1 = p.spin[i] > 0 ? a0 + 1.6 : a0 - 1.6;
        rg.append(arc(48, a0, a1, col, 2.5));
        g.append(rg);
        svg.append(S('text', { x: p.cx + sx * arm * Math.cos(p.psi), y: cy + sx * arm * Math.sin(p.psi) + 62, fill: col, 'font-size': 15, 'text-anchor': 'middle' },
          (p.spin[i] > 0 ? 'CW' : 'CCW') + (k === 1 && st.boost && p.spin[i] < 0 ? ' (faster)' : '')));
      });
      svg.append(g);
      const spinning = Math.abs(p.om) > 0.03;
      if (spinning) {
        const fr = S('g', { transform: `translate(${p.cx},${cy})` });
        fr.append(arc(arm + 52, p.om > 0 ? 0.6 : 2.5, p.om > 0 ? 2.5 : 0.6, C.red, 4));
        svg.append(fr);
      }
      const msg = !spinning ? ['frame torques cancel:', 'no yaw', C.green]
        : k === 0 ? ['frame torques add up:', 'the frame spins the other way (yaw)', C.red]
        : ['the faster CCW rotor wins:', 'the frame yaws CW, slowly', C.red];
      svg.append(S('text', { x: p.cx, y: H - 34, fill: msg[2], 'font-size': 15, 'text-anchor': 'middle' }, msg[0]));
      svg.append(S('text', { x: p.cx, y: H - 14, fill: msg[2], 'font-size': 15, 'text-anchor': 'middle' }, msg[1]));
    });
  }

  if (isPrint()) { for (let i = 0; i < 90; i++) step(0.02); draw(); return; }
  let last = performance.now();
  const loop = now => {
    if (!el.isConnected) return;
    const sec = el.closest('section');
    if (!sec || sec.classList.contains('present')) { step(Math.min(0.05, (now - last) / 1000)); draw(); }
    last = now;
    requestAnimationFrame(loop);
  };
  draw();
  requestAnimationFrame(loop);
}
