// Angle mode vs rate mode: the same stick input, the attitude over time drawn as a strip of side-view drones.
// Angle mode: the stick sets the pitch ANGLE, θ_cmd = 30° · stick, followed by a 2nd-order response (ω_n 12 rad/s,
// ζ 0.8). Rate mode: the stick sets the pitch RATE, q_cmd = 360°/s · stick, followed with a 50 ms lag; the angle is
// its integral (unbounded: a held stick gives a flip). Scenarios: half stick for 0.4 s (angle mode 15° and back,
// rate mode 72° and it stays), full stick for 1 s (angle mode stops at 30°, rate mode makes a full flip).
// Side view, nose to the right; pitching forward (nose down) is drawn as a clockwise rotation.
// Scenario "flip" (rows: ["rate"]): climb first, full pitch stick for 1 s, throttle cut while upside down.
// config: { width: 1180, height: 400, scenario: "push" | "hold" | "flip", rows: ["ang", "rate"] }
import { C, h, fmt, isPrint } from './util.js';
import { S } from './plotsvg.js';

const T_END = 2.6, DT = 0.002;
const SCEN = { push: { a: 0.3, b: 0.7, u: 0.5, label: 'half stick for 0.4 s, release' }, hold: { a: 0.3, b: 1.3, u: 1, label: 'full stick for 1 s' },
  flip: { a: 0.45, b: 1.45, u: 1, label: 'a flip', thr: [[0, 0.45, 0.9], [0.45, 0.7, 0.5], [0.7, 1.2, 0.1], [1.2, 1.45, 0.5], [1.45, 2.6, 0.75]] } };

function simulate(sc) {
  const out = []; let th = 0, w = 0, ra = 0, rq = 0;
  for (let t = 0; t <= T_END + 1e-9; t += DT) {
    const u = t >= sc.a && t < sc.b ? sc.u : 0;
    const cmd = 30 * u * Math.PI / 180, wn = 12, z = 0.8;
    w += (wn * wn * (cmd - th) - 2 * z * wn * w) * DT; th += w * DT;            // angle mode
    rq += ((360 * u * Math.PI / 180) - rq) * DT / 0.05; ra += rq * DT;             // rate mode
    out.push({ t, u, ang: th, rate: ra });
  }
  return out;
}

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 1180, H = cfg.height || 400;
  const svg = S('svg', { viewBox: `0 0 ${W} ${H}`, style: 'width:100%; display:block; font-family:Inter, Arial, sans-serif' });
  const st = { scen: cfg.scenario || 'push', t: 0 };
  let data = simulate(SCEN[st.scen]);
  const scenNames = cfg.scenario === 'flip' ? ['flip'] : ['push', 'hold'];
  const btns = Object.entries(SCEN).filter(([k]) => scenNames.includes(k)).map(([k, v]) => { const b = h('button', { onclick: () => { st.scen = k; st.t = 0; data = simulate(v); btns.forEach(x => x.classList.toggle('active', x === b)); } }, v.label); b.classList.toggle('active', k === st.scen); return b; });
  el.append(svg, h('div', { class: 'wctl interactive-only' }, ...btns, h('button', { onclick: () => { st.t = 0; } }, 'replay')));

  const L = 190, R = W - 30, X = t => L + (R - L) * t / T_END;
  const drone = (x, y, ang, op, col) => {
    const g = S('g', { transform: `translate(${x},${y}) rotate(${(ang * 180 / Math.PI).toFixed(1)}) scale(0.9)`, opacity: op });
    g.append(S('line', { x1: -26, x2: 26, y1: 0, y2: 0, stroke: C.dim, 'stroke-width': 4, 'stroke-linecap': 'round' }));
    g.append(S('rect', { x: -7, y: -5, width: 14, height: 10, rx: 2, fill: C.bg3, stroke: col, 'stroke-width': 1.5 }));
    for (const e of [-1, 1]) g.append(S('line', { x1: e * 26 - 10, x2: e * 26 + 10, y1: -6, y2: -6, stroke: C.blue, 'stroke-width': 3 }));
    g.append(S('path', { d: 'M26,4 l8,0 l-4,6z', fill: C.accent }));          // nose marker (front, right)
    g.append(S('line', { x1: 0, x2: 0, y1: -7, y2: -24, stroke: col, 'stroke-width': 2 }));   // thrust direction
    return g;
  };
  function draw() {
    svg.replaceChildren();
    const allRows = [{ key: 'ang', name: 'angle mode', sub: 'stick = angle', col: C.green }, { key: 'rate', name: 'rate (acro) mode', sub: 'stick = rotation rate', col: C.purple }];
    const rows = allRows.filter(r => (cfg.rows || ['ang', 'rate']).includes(r.key)).map((r, i) => ({ ...r, y: 165 + 135 * i + (cfg.rows && cfg.rows.length === 1 ? 70 : 0) }));
    // stick row
    const sy = 55, sh = 36;
    svg.append(S('text', { x: 10, y: sy - 6, fill: C.accent, 'font-size': 16, 'font-weight': 600 }, 'pitch stick'));
    svg.append(S('line', { x1: L, x2: R, y1: sy + sh, y2: sy + sh, stroke: C.line }));
    const sc = SCEN[st.scen];
    const top = sy + sh * (1 - sc.u);
    svg.append(S('path', { d: `M${L},${sy + sh} L${X(sc.a)},${sy + sh} L${X(sc.a)},${top} L${X(sc.b)},${top} L${X(sc.b)},${sy + sh} L${R},${sy + sh}`, fill: 'none', stroke: C.accent, 'stroke-width': 3 }));
    if (sc.thr) {                                                    // throttle trace (flip scenario)
      const ty = 125, th = 36;
      svg.append(S('text', { x: 10, y: ty - 6, fill: C.blue, 'font-size': 16, 'font-weight': 600 }, 'throttle'));
      svg.append(S('line', { x1: L, x2: R, y1: ty + th, y2: ty + th, stroke: C.line }));
      svg.append(S('path', { d: sc.thr.map(([a, b, v], i) => `${i ? 'L' : 'M'}${X(a)},${ty + th * (1 - v)} L${X(b)},${ty + th * (1 - v)}`).join(' '), fill: 'none', stroke: C.blue, 'stroke-width': 3 }));
      [['climb', 0.22], ['cut: upside down', 0.95], ['catch', 1.7]].forEach(([t, x]) => svg.append(S('text', { x: X(x), y: ty - 6, fill: C.blue, 'font-size': 13, 'text-anchor': 'middle' }, t)));
    }
    svg.append(S('text', { x: (X(sc.a) + X(sc.b)) / 2, y: top - 6, fill: C.accent, 'font-size': 13, 'text-anchor': 'middle' }, 'forward'));
    for (const r of rows) {
      svg.append(S('text', { x: 10, y: r.y - 4, fill: r.col, 'font-size': 16, 'font-weight': 600 }, r.name));
      svg.append(S('text', { x: 10, y: r.y + 16, fill: C.dim, 'font-size': 13 }, r.sub));
      svg.append(S('line', { x1: L, x2: R, y1: r.y + 42, y2: r.y + 42, stroke: C.line, 'stroke-dasharray': '3 5' }));
      const step = Math.round(0.2 / DT);
      for (let i = 0; i < data.length; i += step) {
        const d = data[i]; if (d.t > st.t + 1e-9) break;
        svg.append(drone(X(d.t), r.y, d[r.key], 0.95, r.col));
      }
      const last = data[Math.min(data.length - 1, Math.round(st.t / DT))];
      const deg = last[r.key] * 180 / Math.PI;
      svg.append(S('text', { x: R, y: r.y + 62, fill: r.col, 'font-size': 14, 'text-anchor': 'end' }, `pitch now ${fmt(deg, 0)}°`));
    }
    // time axis and playhead
    svg.append(S('line', { x1: L, x2: R, y1: H - 26, y2: H - 26, stroke: C.dim }));
    for (let t = 0; t <= T_END + 1e-9; t += 0.5) {
      svg.append(S('line', { x1: X(t), x2: X(t), y1: H - 26, y2: H - 20, stroke: C.dim }));
      svg.append(S('text', { x: X(t), y: H - 6, fill: C.dim, 'font-size': 13, 'text-anchor': 'middle' }, `${t} s`));
    }
    if (st.t < T_END) svg.append(S('line', { x1: X(st.t), x2: X(st.t), y1: 30, y2: H - 26, stroke: C.accent, 'stroke-opacity': 0.5 }));
  }
  if (isPrint()) { st.scen = cfg.scenario === 'flip' ? 'flip' : 'hold'; data = simulate(SCEN[st.scen]); st.t = T_END; draw(); return; }
  let last = performance.now();
  const loop = now => {
    if (!el.isConnected) return;
    const sec = el.closest('section');
    if (!sec || sec.classList.contains('present')) {
      st.t += Math.min(0.05, (now - last) / 1000) * 0.5;          // half speed
      if (st.t > T_END + 1.5) st.t = 0;
      draw();
    }
    last = now;
    requestAnimationFrame(loop);
  };
  draw();
  requestAnimationFrame(loop);
}
