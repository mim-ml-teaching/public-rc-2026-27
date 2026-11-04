// A double pendulum, simulated (RK4) and drawn live. One trajectory is computed once per page and every
// instance shows the same moment of it, so the pendulum looks continuous when the slides that refer to it follow
// each other. The labels are per slide:
//   show: ['labels']    the positions r1(t), r2(t) of the masses
//         ['coords']    the disks numbered 1 and 2, projected onto the x and y axes, with the four numbers (x1, y1, x2, y2)
//         ['vcoords']   the same for the velocities: the four numbers (ẋ1, ẏ1, ẋ2, ẏ2)
//         ['velocity']  the velocity vectors of the masses
//         ['lengths']   the rod lengths L1, L2 (the constraints)
//         ['angles']    the angles θ1, θ2 (the generalised coordinates)
//   stretch: false      true: the rods stretch with the tension in them (exaggerated, up to +45%): a constraint that
//                       changes in time; the angles of the shared trajectory are kept, only the lengths change
// config: { width: 320, height: 300, show: ['labels'], trace: true, stretch: false, print_t: 1.6 }
import { C, h, isPrint } from './util.js';

const G = 9.81, L1 = 1, L2 = 1, M1 = 1, M2 = 1, DT = 0.002, T_TOTAL = 40, STRETCH = 0.5, STRETCH_MAX = 0.9;
let TRAJ = null, TMEAN = 1;  // [[th1, th2, w1, w2], ...] every DT; the mean rod tension along it
// the tension in the rods (centripetal + the weight below), used to stretch them
const tension = q => [Math.max(0, (M1 + M2) * (L1 * q[2] * q[2] + G * Math.cos(q[0]))), Math.max(0, M2 * (L2 * q[3] * q[3] + G * Math.cos(q[1])))];
function deriv([a, b, wa, wb]) {
  const d = a - b, s = Math.sin(d), c = Math.cos(d), den = 2 * M1 + M2 - M2 * Math.cos(2 * d);
  const dwa = (-G * (2 * M1 + M2) * Math.sin(a) - M2 * G * Math.sin(a - 2 * b) - 2 * s * M2 * (wb * wb * L2 + wa * wa * L1 * c)) / (L1 * den);
  const dwb = (2 * s * (wa * wa * L1 * (M1 + M2) + G * (M1 + M2) * Math.cos(a) + wb * wb * L2 * M2 * c)) / (L2 * den);
  return [wa, wb, dwa, dwb];
}
function trajectory() {
  if (TRAJ) return TRAJ;
  let q = [Math.PI * 0.55, Math.PI * 0.75, 0, 0];  // a lively start
  TRAJ = [q];
  const add = (u, v, k) => u.map((x, i) => x + k * v[i]);
  for (let i = 0; i < T_TOTAL / DT; i++) {
    const k1 = deriv(q), k2 = deriv(add(q, k1, DT / 2)), k3 = deriv(add(q, k2, DT / 2)), k4 = deriv(add(q, k3, DT));
    q = q.map((x, j) => x + DT / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]));
    TRAJ.push(q);
  }
  const ts = TRAJ.filter((_, i) => i % 10 === 0).flatMap(q => tension(q)); TMEAN = ts.reduce((a, b) => a + b, 0) / ts.length;
  return TRAJ;
}
const T0 = performance.now();

export function mount(el, cfg) {
  const W = cfg.width || 320, H = cfg.height || 300, show = new Set(cfg.show || ['labels']), trace = cfg.trace ?? true, stretch = !!cfg.stretch;
  const traj = trajectory();
  const canvas = h('canvas', { width: W, height: H }), g = canvas.getContext('2d');
  const px = W / 2, py = H / 2, S = (Math.min(W, H) / 2 - 26) / ((L1 + L2) * (stretch ? 1 + STRETCH_MAX : 1));  // pixels per metre: the full reach fits, over the top too
  const lens = q => { if (!stretch) return [L1, L2]; const [t1, t2] = tension(q); const e = t => Math.min(STRETCH_MAX, STRETCH * t / TMEAN); return [L1 * (1 + e(t1)), L2 * (1 + e(t2))]; };
  const pos = q => { const [l1, l2] = lens(q); const x1 = px + S * l1 * Math.sin(q[0]), y1 = py + S * l1 * Math.cos(q[0]); return [x1, y1, x1 + S * l2 * Math.sin(q[1]), y1 + S * l2 * Math.cos(q[1])]; };
  const label = (txt, x, y, color, dx = 12) => {   // beside (x, y), on the side with room, inside the canvas
    g.fillStyle = color; g.font = '14px Inter, Arial, sans-serif';
    const w = g.measureText(txt).width, right = x + dx + w < W - 6;
    g.textAlign = right ? 'left' : 'right'; g.fillText(txt, right ? x + dx : x - dx, Math.max(16, Math.min(H - 8, y + 5)));
    g.textAlign = 'left';
  };
  function draw(t) {
    const k = Math.min(traj.length - 1, Math.round((t % T_TOTAL) / DT)), q = traj[k];
    const [x1, y1, x2, y2] = pos(q);
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    if (trace) {  // the path of the second mass over the last 1.5 s
      g.lineWidth = 1.5; g.beginPath();
      for (let j = Math.max(0, k - 750); j <= k; j += 5) { const p = pos(traj[j]); j === Math.max(0, k - 750) ? g.moveTo(p[2], p[3]) : g.lineTo(p[2], p[3]); }
      g.strokeStyle = 'rgba(90,176,255,0.45)'; g.stroke();
    }
    // the pivot
    g.fillStyle = C.dim; g.beginPath(); g.arc(px, py, 5, 0, 7); g.fill();
    if (show.has('angles')) {
      g.setLineDash([4, 3]); g.strokeStyle = C.dim; g.lineWidth = 1;
      g.beginPath(); g.moveTo(px, py); g.lineTo(px, py + 50); g.moveTo(x1, y1); g.lineTo(x1, y1 + 50); g.stroke(); g.setLineDash([]);
      const arc = (cx, cy, th, color, label) => { const a = Math.atan2(Math.sin(th), Math.cos(th)); g.strokeStyle = color; g.lineWidth = 2; g.beginPath(); g.arc(cx, cy, 28, Math.PI / 2, Math.PI / 2 - a, a > 0); g.stroke(); g.fillStyle = color; g.font = '15px Inter, Arial, sans-serif'; g.fillText(label, cx + 34 * Math.sin(a / 2) + 4, cy + 34 * Math.cos(a / 2) + 5); };
      arc(px, py, q[0], C.red, 'θ₁'); arc(x1, y1, q[1], C.green, 'θ₂');
    }
    // rods and masses
    g.strokeStyle = C.fg; g.lineWidth = 2.5; g.beginPath(); g.moveTo(px, py); g.lineTo(x1, y1); g.lineTo(x2, y2); g.stroke();
    for (const [x, y] of [[x1, y1], [x2, y2]]) { g.fillStyle = C.bg2; g.strokeStyle = C.fg; g.lineWidth = 2.5; g.beginPath(); g.arc(x, y, 8, 0, 7); g.fill(); g.stroke(); }
    g.font = '13px Inter, Arial, sans-serif'; g.textAlign = 'left';
    if (show.has('lengths')) {   // the two lengths in the bottom-left table, like the coordinates
      const [l1, l2] = lens(q), tag = stretch ? '(t)' : '';
      const bx = 14, by = H - 54;
      g.fillStyle = 'rgba(30,33,40,0.92)'; g.fillRect(bx, by, 120, 42); g.strokeStyle = C.line; g.strokeRect(bx, by, 120, 42);
      g.font = '13px "JetBrains Mono", Menlo, monospace'; g.fillStyle = C.accent; g.textAlign = 'left';
      [[`L₁${tag}`, l1], [`L₂${tag}`, l2]].forEach(([n, v], i) => { g.fillText(n + ' =', bx + 8, by + 18 + i * 18); g.textAlign = 'right'; g.fillText(v.toFixed(2), bx + 112, by + 18 + i * 18); g.textAlign = 'left'; });
    }
    if (show.has('velocity')) {
      const v = (th, w, L) => [S * L * w * Math.cos(th), -S * L * w * Math.sin(th)];
      const v1 = v(q[0], q[2], L1), v2 = v(q[1], q[3], L2).map((c, i) => c + v1[i]);
      const arrow = (x, y, vx, vy, label) => { const s = 0.18; const ex = x + vx * s, ey = y + vy * s; const L = Math.hypot(ex - x, ey - y); if (L < 6) return; g.strokeStyle = C.green; g.fillStyle = C.green; g.lineWidth = 2.5; g.beginPath(); g.moveTo(x, y); g.lineTo(ex, ey); g.stroke(); const ux = (ex - x) / L, uy = (ey - y) / L; g.beginPath(); g.moveTo(ex + ux * 7, ey + uy * 7); g.lineTo(ex - uy * 5, ey + ux * 5); g.lineTo(ex + uy * 5, ey - ux * 5); g.closePath(); g.fill(); g.fillText(label, ex + ux * 10 + 4, ey + uy * 10 + 5); };
      arrow(x1, y1, v1[0], v1[1], 'ṙ₁'); arrow(x2, y2, v2[0], v2[1], 'ṙ₂');
    }
    if (show.has('coords') || show.has('vcoords')) {
      // the axes through the pivot and the projections of the disks onto them
      g.strokeStyle = C.dim; g.lineWidth = 1; g.beginPath(); g.moveTo(14, py); g.lineTo(W - 14, py); g.moveTo(px, 14); g.lineTo(px, H - 14); g.stroke();
      g.fillStyle = C.dim; g.font = '12px Inter, Arial, sans-serif'; g.textAlign = 'left'; g.fillText('x', W - 24, py - 6); g.fillText('y', px + 6, 24);
      g.setLineDash([3, 4]); g.strokeStyle = 'rgba(154,161,174,0.6)';
      for (const [x, y] of [[x1, y1], [x2, y2]]) { g.beginPath(); g.moveTo(x, y); g.lineTo(x, py); g.moveTo(x, y); g.lineTo(px, y); g.stroke(); }
      g.setLineDash([]);
      g.fillStyle = C.fg; g.font = 'bold 12px Inter, Arial, sans-serif'; g.textAlign = 'center';
      g.fillText('1', x1, y1 + 4); g.fillText('2', x2, y2 + 4); g.textAlign = 'left';
      // the four numbers, in a corner away from the disks
      const w2 = (x, y) => [(x - px) / S, -(y - py) / S];   // canvas → metres, y up
      const [X1, Y1] = w2(x1, y1), [X2, Y2] = w2(x2, y2);
      let rows;
      if (show.has('vcoords')) {
        const v = (th, w, L) => [L * w * Math.cos(th), L * w * Math.sin(th)];   // ẋ, ẏ (y up) of the rod end relative to its start
        const [l1, l2] = lens(q), v1 = v(q[0], q[2], l1), v2 = v(q[1], q[3], l2);
        rows = [['ẋ₁', v1[0]], ['ẏ₁', v1[1]], ['ẋ₂', v1[0] + v2[0]], ['ẏ₂', v1[1] + v2[1]]];
      } else rows = [['x₁', X1], ['y₁', Y1], ['x₂', X2], ['y₂', Y2]];
      const bx = 14, by = H - 90;   // always the bottom-left corner: it never jumps
      g.fillStyle = 'rgba(30,33,40,0.92)'; g.fillRect(bx, by, 104, 78); g.strokeStyle = C.line; g.strokeRect(bx, by, 104, 78);
      g.font = '13px "JetBrains Mono", Menlo, monospace';
      rows.forEach(([n, v], i) => { g.fillStyle = C.fg; g.fillText(n + ' =', bx + 8, by + 18 + i * 18); g.textAlign = 'right'; g.fillText((v >= 0 ? ' ' : '') + v.toFixed(2), bx + 96, by + 18 + i * 18); g.textAlign = 'left'; });
    }
    if (show.has('labels')) {
      label('r₁(t)', x1, y1, C.fg); label('r₂(t)', x2, y2, C.fg);
    }
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H - 2);
  }
  el.classList.add('widget'); el.style.width = W + 'px';
  el.append(canvas);
  if (isPrint()) { draw(cfg.print_t ?? 1.6); return; }
  let raf = 0;
  const shown = () => el.closest('section')?.classList.contains('present') !== false;
  const tick = () => { if (!el.isConnected) return; if (shown()) draw((performance.now() - T0) / 1000); raf = requestAnimationFrame(tick); };
  raf = requestAnimationFrame(tick);
}
