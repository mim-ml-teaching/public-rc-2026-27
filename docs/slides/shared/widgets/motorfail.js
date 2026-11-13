// What happens when one motor fails? Top view of a quad (ArduPilot QUAD/X numbering) or a hexacopter (motors
// numbered clockwise from the front right, at 30° + k·60°), with a spin pattern. Click a motor (or use the buttons)
// to fail it. Everything is computed from the static force/torque balance, FRD body frame, unit arm length:
//   thrust Σ u_i,  roll τx = −Σ y_i u_i,  pitch τy = Σ x_i u_i,  yaw τz = Σ s_i k u_i  (s = +1 CCW, −1 CW; k = 0.1)
// with each motor output u_i ∈ [0, 1] and a hover thrust of n/2 (thrust-to-weight 2:1).
//  1. hover: is there u with hover thrust and zero torques?  2. margin: at that thrust (other torques zero), the
// largest torque in each of the six directions, as a fraction of the healthy vehicle's value. A zero bar means the
// controller cannot push back in that direction. LPs are solved exactly by enumerating basic solutions (n ≤ 6).
// config: { width: 460, height: 400, layout: "quad" | "hexPNPNPN" | "hexPPNNPN", fail: 1 (0 = none), layouts: [...] }
import { C, h, fmt } from './util.js';
import { S } from './plotsvg.js';
import { MOTORS as QUAD } from './quadmix.js';

const deg = Math.PI / 180;
const hex = pattern => [...pattern].map((c, i) => {
  const a = (30 + 60 * i) * deg;
  return { n: i + 1, x: Math.cos(a), y: Math.sin(a), spin: c === 'P' ? 'CW' : 'CCW' };
});
const LAYOUTS = {
  quad: { name: 'quad X', motors: QUAD.map(m => ({ n: m.n, x: m.x / Math.SQRT2, y: m.y / Math.SQRT2, spin: m.spin })) },
  hexPNPNPN: { name: 'hex PNPNPN', motors: hex('PNPNPN') },
  hexPPNNPN: { name: 'hex PPNNPN', motors: hex('PPNNPN') },
};
const K = 0.1;
const rowsOf = ms => [ms.map(() => 1), ms.map(m => -m.y), ms.map(m => m.x), ms.map(m => (m.spin === 'CCW' ? 1 : -1) * K)];

// all basic feasible solutions of A u = b, 0 ≤ u ≤ 1 (each variable free, at 0 or at 1; ≤ m free ones)
function vertices(A, b) {
  const m = A.length, n = A[0].length, out = [];
  const total = 3 ** n;
  for (let code = 0; code < total; code++) {
    const kind = [], free = [];
    let c = code;
    for (let i = 0; i < n; i++) { kind.push(c % 3); c = Math.floor(c / 3); if (kind[i] === 2) free.push(i); }
    if (free.length > m) continue;
    const u = kind.map(k => (k === 1 ? 1 : 0));
    const r = b.map((bi, j) => bi - A[j].reduce((s, a, i) => s + (kind[i] === 2 ? 0 : a * u[i]), 0));
    if (free.length) {
      // normal equations (AᵀA) x = Aᵀ r on the free columns
      const f = free.length, M = Array.from({ length: f }, () => new Array(f + 1).fill(0));
      for (let p = 0; p < f; p++) {
        for (let q = 0; q < f; q++) M[p][q] = A.reduce((s, row) => s + row[free[p]] * row[free[q]], 0);
        M[p][f] = A.reduce((s, row, j) => s + row[free[p]] * r[j], 0);
      }
      let ok = true;
      for (let p = 0; p < f && ok; p++) {
        let piv = p;
        for (let q = p + 1; q < f; q++) if (Math.abs(M[q][p]) > Math.abs(M[piv][p])) piv = q;
        if (Math.abs(M[piv][p]) < 1e-9) { ok = false; break; }
        [M[p], M[piv]] = [M[piv], M[p]];
        for (let q = 0; q < f; q++) if (q !== p) { const t = M[q][p] / M[p][p]; for (let z = p; z <= f; z++) M[q][z] -= t * M[p][z]; }
      }
      if (!ok) continue;
      free.forEach((i, p) => { u[i] = M[p][f] / M[p][p]; });
      if (free.some(i => u[i] < -1e-9 || u[i] > 1 + 1e-9)) continue;
    }
    const res = A.map((row, j) => row.reduce((s, a, i) => s + a * u[i], 0) - b[j]);
    if (res.every(v => Math.abs(v) < 1e-7)) out.push(u.map(v => Math.min(1, Math.max(0, v))));
  }
  return out;
}

export function analyse(ms, failed) {
  const alive = ms.filter(m => m.n !== failed), A = rowsOf(alive), hov = ms.length / 2;
  const V = vertices(A, [hov, 0, 0, 0]);
  const hover = V.length ? alive.map((_, i) => V.reduce((s, v) => s + v[i], 0) / V.length) : null;
  // largest torque in each direction at hover thrust, other two torques zero
  const avail = [];
  for (const ax of [1, 2, 3]) for (const sg of [1, -1]) {
    const others = [1, 2, 3].filter(a => a !== ax);
    const W = vertices([A[0], A[others[0]], A[others[1]]], [hov, 0, 0]);
    avail.push(W.length && hover ? Math.max(0, ...W.map(u => sg * A[ax].reduce((s, a, i) => s + a * u[i], 0))) : null);
  }
  return { alive, hover, avail };
}

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 460, H = cfg.height || 400;
  const st = { layout: cfg.layout || 'quad', fail: cfg.fail ?? 1 };
  const svg = S('svg', { width: W, height: H, viewBox: `0 0 ${W} ${H}`, style: 'display:block; flex:none; font-family:Inter, Arial, sans-serif' });
  const bars = S('svg', { width: 360, height: 220, viewBox: '0 0 360 220', style: 'display:block; font-family:Inter, Arial, sans-serif' });
  const verdict = h('div', { style: 'width:360px; min-height:4.2em; font-size:0.8em; line-height:1.35; margin-top:0.3em' });
  const lay = h('div', { class: 'wctl interactive-only', style: 'flex-wrap:wrap; width:360px' });
  const layouts = cfg.layouts || Object.keys(LAYOUTS);
  const lb = layouts.map(k => h('button', { onclick: () => { st.layout = k; if (st.fail > LAYOUTS[k].motors.length) st.fail = 1; draw(); } }, LAYOUTS[k].name));
  lay.append(...lb, h('button', { onclick: () => { st.fail = 0; draw(); } }, 'all working'));
  el.append(h('div', { style: 'display:flex; gap:1em; align-items:flex-start' }, svg,
    h('div', { style: 'width:360px; flex:none' }, h('div', { class: 'small', style: 'margin-bottom:0.2em' }, 'torque still available at hover (vs healthy)'), bars, verdict, lay)));

  const healthy = {};
  function draw() {
    const L = LAYOUTS[st.layout], ms = L.motors;
    lb.forEach((b, i) => b.classList.toggle('active', layouts[i] === st.layout));
    if (!healthy[st.layout]) healthy[st.layout] = analyse(ms, 0).avail;
    const res = analyse(ms, st.fail), ref = healthy[st.layout];
    // ---- top view
    svg.replaceChildren();
    const cx = W / 2, cy = H / 2 + 6, d = Math.min(W, H) * (ms.length > 4 ? 0.3 : 0.34), R0 = Math.min(W, H) * (ms.length > 4 ? 0.1 : 0.12);
    const P = m => [cx + m.y * d, cy - m.x * d];
    svg.append(S('text', { x: cx, y: 18, fill: C.accent, 'font-size': 16, 'text-anchor': 'middle', 'font-weight': 600 }, `front (x) · ${L.name}`));
    for (const m of ms) { const [x, y] = P(m); svg.append(S('line', { x1: cx, y1: cy, x2: x, y2: y, stroke: C.dim, 'stroke-width': 7, 'stroke-linecap': 'round' })); }
    svg.append(S('rect', { x: cx - 22, y: cy - 28, width: 44, height: 56, rx: 9, fill: C.bg3, stroke: C.dim, 'stroke-width': 2 }));
    svg.append(S('path', { d: `M${cx},${cy - 22} L${cx - 8},${cy - 8} L${cx + 8},${cy - 8} Z`, fill: C.accent }));
    ms.forEach(m => {
      const [x, y] = P(m), dead = m.n === st.fail, col = m.spin === 'CCW' ? C.purple : C.green;
      const ai = res.alive.indexOf(m), u = dead ? 0 : res.hover ? res.hover[ai] : null;
      const g = S('g', { style: 'cursor:pointer' });
      g.addEventListener('click', () => { st.fail = st.fail === m.n ? 0 : m.n; draw(); });
      g.append(S('circle', { cx: x, cy: y, r: R0, fill: C.bg }));
      g.append(S('circle', { cx: x, cy: y, r: R0, fill: dead ? C.bg2 : col, 'fill-opacity': dead ? 1 : 0.08 + 0.42 * (u ?? 0.5), stroke: dead ? C.red : col, 'stroke-width': 2, 'stroke-dasharray': dead ? '5 4' : 'none' }));
      if (dead) g.append(S('path', { d: `M${x - R0 * 0.6},${y - R0 * 0.6} L${x + R0 * 0.6},${y + R0 * 0.6} M${x + R0 * 0.6},${y - R0 * 0.6} L${x - R0 * 0.6},${y + R0 * 0.6}`, stroke: C.red, 'stroke-width': 4 }));
      g.append(S('text', { x, y: y + 7, fill: C.fg, 'font-size': 20, 'font-weight': 700, 'text-anchor': 'middle' }, String(m.n)));
      const below = y > cy + 1;
      const ly = below ? y + R0 + 18 : y - R0 - 8;
      g.append(S('text', { x, y: ly, fill: dead ? C.red : C.dim, 'font-size': 14, 'text-anchor': 'middle' }, dead ? 'failed' : `${m.spin}${u != null ? ' · ' + fmt(u, 2) : ''}`));
      svg.append(g);
    });
    svg.append(S('text', { x: cx, y: H - 4, fill: C.dim, 'font-size': 13, 'text-anchor': 'middle' }, 'click a motor to fail it · numbers: one hover solution'));
    // ---- bars
    bars.replaceChildren();
    const names = ['roll right', 'roll left', 'nose up', 'nose down', 'yaw right', 'yaw left'], cols = [C.red, C.red, C.green, C.green, C.blue, C.blue];
    names.forEach((nm, i) => {
      const y = 8 + i * 35, v = res.avail[i], frac = v == null ? 0 : v / ref[i];
      bars.append(S('text', { x: 92, y: y + 17, fill: cols[i], 'font-size': 15, 'text-anchor': 'end' }, nm));
      bars.append(S('rect', { x: 100, y, width: 200, height: 24, fill: C.bg2, stroke: C.line }));
      bars.append(S('rect', { x: 100, y, width: 200 * Math.min(1, frac), height: 24, fill: cols[i], 'fill-opacity': 0.6 }));
      bars.append(S('text', { x: 308, y: y + 17, fill: frac < 0.005 ? C.red : C.fg, 'font-size': 15, 'font-weight': frac < 0.005 ? 700 : 400 }, v == null ? 'none' : `${Math.round(100 * frac)}%`));
    });
    // ---- verdict
    const lost = res.avail.some(v => v == null || v < 1e-6);
    let msg;
    if (!st.fail) msg = `<span style="color:${C.green}">All motors work: every torque direction is available.</span>`;
    else if (!res.hover) msg = `<b style="color:${C.red}">No hover without spinning.</b> ` + (st.layout === 'quad'
      ? 'Zero roll and pitch torque forces the opposite motor to zero too; the remaining pair spins the same way, so the yaw torque cannot be cancelled. Keep flying by letting it spin (Mueller and D’Andrea, 2014).'
      : 'No combination of the remaining motors gives zero torque at hover thrust.');
    else if (lost) msg = `<b style="color:${C.red}">Hover is possible, but with no margin:</b> a motor is already at its limit and some torque direction is gone (0%), so the attitude cannot be fully controlled.`;
    else msg = `<b style="color:${C.green}">Still controllable:</b> hover with some torque left in every direction.`;
    verdict.innerHTML = msg;
  }
  draw();
}
