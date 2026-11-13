// X frame vs + frame: which motors change for "tilt forward" and "tilt right", and how fast each frame tilts.
// Same motors (each may change its thrust by at most ±Δmax around hover), same arm length l, same time-optimal tilt
// to 20° (full change, then full reverse to stop; X 0.29 s, + 0.34 s in the model). Torque about the tilt axis: X frame 4 · (l/√2) · Δ = 2√2 l Δ (all four motors, lever l/√2);
// + frame 2 · l · Δ (two motors, lever l; the other two do not change). So the X frame has √2 more authority.
// Top view: motor circles coloured by the change (red slower, green faster); side view: the tilt over time.
// config: { width: 1180, height: 400, frames: ["X", "plus"] }   (frames: ["X"] shows the X frame alone, centred)
import { C, h, fmt, isPrint } from './util.js';
import { S } from './plotsvg.js';

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 1180, H = cfg.height || 400, s2 = Math.SQRT1_2;
  const svg = S('svg', { viewBox: `0 0 ${W} ${H}`, style: 'width:100%; display:block' });
  // motor positions in body coordinates (x forward, y right), arm length 1
  const FR = { X: [[s2, s2], [s2, -s2], [-s2, s2], [-s2, -s2]], plus: [[1, 0], [0, 1], [-1, 0], [0, -1]] };
  const st = { cmd: 'forward', t: 0, u: { X: 0, plus: 0 }, th: { X: 0, plus: 0 }, w: { X: 0, plus: 0 }, d: { X: [0, 0, 0, 0], plus: [0, 0, 0, 0] }, reach: {} };
  const TARGET = 20 * Math.PI / 180, DMAX = 0.15, K = 40;          // K: torque → angular acceleration scale
  const restart = cmd => { st.cmd = cmd; st.t = 0; st.reach = {}; for (const f of ['X', 'plus']) { st.th[f] = 0; st.w[f] = 0; } btns.forEach(b => b.classList.toggle('active', b.dataset.cmd === cmd)); };
  const btns = [['forward', 'tilt forward (pitch)'], ['right', 'tilt right (roll)']].map(([c, t]) => { const b = h('button', { onclick: () => restart(c) }, t); b.dataset.cmd = c; return b; });
  el.append(svg, h('div', { class: 'wctl interactive-only' }, ...btns, h('button', { onclick: () => restart(st.cmd) }, 'replay')));
  btns[0].classList.add('active');

  function step(dt) {
    st.t += dt;
    for (const f of ['X', 'plus']) {
      // axis: forward tilt = nose down = rotation about y, uses motor x-coordinates; right tilt about x uses y-coordinates
      const lev = FR[f].map(([x, y]) => (st.cmd === 'forward' ? x : y));           // positive lever: motor must slow down
      // time-optimal (bang-bang) tilt, in closed form (a feedback version chattered at the switching point and made
      // the motor colours blink): full change for t1 = √(θ*/a), full reverse for another t1, then hold. a = maximum
      // angular acceleration of this frame = K Δmax Σ|lever|, so the frame with more torque arrives sooner (2 t1 ∝ 1/√a).
      const a = K * DMAX * lev.reduce((acc, l) => acc + Math.abs(l), 0), t1 = Math.sqrt(TARGET / a), t = st.t;
      const u = t < t1 ? 1 : t < 2 * t1 ? -1 : 0;
      st.th[f] = t < t1 ? a * t * t / 2 : t < 2 * t1 ? TARGET - a * (2 * t1 - t) ** 2 / 2 : TARGET;
      st.u[f] = u;
      st.d[f] = lev.map(l => (Math.abs(l) < 1e-6 ? 0 : -Math.sign(l) * DMAX * u));
      if (!st.reach[f] && t >= 2 * t1) st.reach[f] = 2 * t1;
    }
  }
  function draw() {
    svg.replaceChildren();
    const frames = (cfg.frames || ['X', 'plus']).map(f => [f, f === 'X' ? 'X frame' : '+ frame']);
    if (frames.length > 1) svg.append(S('line', { x1: W / 2, x2: W / 2, y1: 10, y2: H - 10, stroke: C.line }));
    frames.forEach(([f, title], k) => {
      const cx = frames.length > 1 ? W * (k ? 0.75 : 0.25) : W / 2, ty = 150, R = 95;
      svg.append(S('text', { x: cx, y: 24, fill: C.fg, 'font-size': 19, 'font-weight': 600, 'text-anchor': 'middle' }, title));
      // top view: x forward = up on the screen, y right = right
      const P = ([x, y]) => [cx - 120 + y * R, ty - x * R];
      const c0 = P([0, 0]);
      FR[f].forEach(m => { const [px, py] = P(m); svg.append(S('line', { x1: c0[0], y1: c0[1], x2: px, y2: py, stroke: C.dim, 'stroke-width': 8, 'stroke-linecap': 'round' })); });
      svg.append(S('rect', { x: c0[0] - 15, y: c0[1] - 20, width: 30, height: 40, rx: 6, fill: C.bg3, stroke: C.fg }));
      svg.append(S('path', { d: `M${c0[0]},${c0[1] - 20} l-8,13 l16,0z`, fill: C.accent }));
      FR[f].forEach((m, i) => {
        const [px, py] = P(m), d = st.d[f][i], col = Math.abs(d) < 0.01 ? C.dim : d < 0 ? C.red : C.green;
        svg.append(S('circle', { cx: px, cy: py, r: 30, fill: col, 'fill-opacity': 0.15 + 2.5 * Math.abs(d), stroke: col, 'stroke-width': 2 }));
        svg.append(S('text', { x: px, y: py + 6, fill: col, 'font-size': 16, 'font-weight': 700, 'text-anchor': 'middle' }, Math.abs(d) < 0.01 ? '=' : d < 0 ? 'slower' : 'faster'));
      });
      // side view of the tilt (looking along the tilt axis)
      const sx = cx + 150, sy = 175, th = st.th[f], arm = 80;
      svg.append(S('text', { x: sx, y: 60, fill: C.dim, 'font-size': 14, 'text-anchor': 'middle' }, st.cmd === 'forward' ? 'side view' : 'view from behind'));
      const g = S('g', { transform: `translate(${sx},${sy}) rotate(${(th * 180 / Math.PI).toFixed(2)})` });
      g.append(S('line', { x1: -arm, x2: arm, y1: 0, y2: 0, stroke: C.dim, 'stroke-width': 7, 'stroke-linecap': 'round' }));
      g.append(S('rect', { x: -14, y: -9, width: 28, height: 18, rx: 4, fill: C.bg3, stroke: C.fg }));
      for (const e of [-1, 1]) g.append(S('rect', { x: e * arm - 22, y: -14, width: 44, height: 5, rx: 2, fill: C.blue, 'fill-opacity': 0.7 }));
      g.append(S('text', { x: arm + 8, y: 18, fill: C.accent, 'font-size': 13 }, st.cmd === 'forward' ? 'front' : 'right'));
      g.append(S('line', { x1: 0, x2: 0, y1: -12, y2: -70, stroke: C.accent, 'stroke-width': 3 }));
      svg.append(g);
      svg.append(S('line', { x1: sx, x2: sx, y1: sy - 12, y2: sy - 75, stroke: C.line, 'stroke-dasharray': '4 4' }));
      const u = st.u[f], ph = u > 0.05 ? ['start the rotation', C.red] : u < -0.05 ? ['stop the rotation (briefly reversed)', C.green] : ['hold: all motors equal again', C.dim];
      svg.append(S('text', { x: sx, y: sy + 108, fill: ph[1], 'font-size': 14, 'text-anchor': 'middle' }, ph[0]));
      svg.append(S('text', { x: sx, y: sy + 60, fill: C.fg, 'font-size': 16, 'text-anchor': 'middle' }, `tilt ${fmt(th * 180 / Math.PI, 0)}°`));
      const r = st.reach[f];
      svg.append(S('text', { x: sx, y: sy + 84, fill: C.accent, 'font-size': 15, 'text-anchor': 'middle' }, r ? `20° after ${fmt(r, 2)} s` : ' '));
      // what changes
      svg.append(S('text', { x: cx, y: H - 52, fill: C.fg, 'font-size': 16, 'text-anchor': 'middle' },
        frames.length === 1 ? 'tilt forward: the two front motors slower, the two rear motors faster'
          : f === 'X' ? 'all 4 motors help, lever l/√2 each: torque 2√2 · lΔ' : 'only 2 motors help, lever l: torque 2 · lΔ'));
      svg.append(S('text', { x: cx, y: H - 28, fill: C.dim, 'font-size': 15, 'text-anchor': 'middle' },
        frames.length === 1 ? 'the thrust tilts forward with the frame: the drone accelerates forward'
          : f === 'X' ? 'the camera looks forward between two propellers' : 'the front propeller sits in the camera view'));
    });
  }
  if (isPrint()) { for (let i = 0; i < 9; i++) step(0.01); draw(); return; }
  let last = performance.now();
  const loop = now => {
    if (!el.isConnected) return;
    const sec = el.closest('section');
    if (!sec || sec.classList.contains('present')) {
      const dt = Math.min(0.05, (now - last) / 1000) * 0.2;           // slow motion (5×): the real tilt takes a quarter of a second
      step(dt); if (st.t > 1.0) restart(st.cmd); draw();
    }
    last = now;
    requestAnimationFrame(loop);
  };
  draw();
  requestAnimationFrame(loop);
}
