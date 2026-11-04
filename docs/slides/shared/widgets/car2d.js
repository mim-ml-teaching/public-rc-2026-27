// A car seen from the top, driving a figure of eight. The velocity (green) always points where the car points
// (the heading, red): the car cannot move sideways. The path is a lemniscate; the heading is the direction of
// the velocity along it, so the constraint holds by construction.
// config: { width: 320, height: 300, print_s: 1.0 }
import { C, h, isPrint } from './util.js';

export function mount(el, cfg) {
  const W = cfg.width || 320, H = cfg.height || 300;
  const canvas = h('canvas', { width: W, height: H }), g = canvas.getContext('2d');
  const A = 1, B = 0.62, S = (Math.min(W / (2 * A), H / (2 * B)) - 30), cx = W / 2, cy = H / 2;
  const P = s => [A * Math.sin(s), B * Math.sin(2 * s)];
  const V = s => [A * Math.cos(s), 2 * B * Math.cos(2 * s)];
  const X = x => cx + S * x, Y = y => cy - S * y;
  function draw(s) {
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    // axes and the path
    g.strokeStyle = C.line; g.lineWidth = 1; g.beginPath(); g.moveTo(14, cy); g.lineTo(W - 14, cy); g.moveTo(cx, 14); g.lineTo(cx, H - 14); g.stroke();
    g.fillStyle = C.dim; g.font = '12px Inter, Arial, sans-serif'; g.fillText('x', W - 22, cy - 6); g.fillText('y', cx + 6, 22);
    g.strokeStyle = 'rgba(90,176,255,0.35)'; g.lineWidth = 1.5; g.setLineDash([4, 4]); g.beginPath();
    for (let i = 0; i <= 200; i++) { const [x, y] = P(i / 200 * 2 * Math.PI); i ? g.lineTo(X(x), Y(y)) : g.moveTo(X(x), Y(y)); }
    g.stroke(); g.setLineDash([]);
    const [x, y] = P(s), [vx, vy] = V(s), phi = Math.atan2(vy, vx), speed = Math.hypot(vx, vy);
    const px = X(x), py = Y(y), c = Math.cos(phi), sn = Math.sin(phi);
    const R = (u, v) => [px + u * c + v * sn, py - (u * sn - v * c)];   // car frame (u forward, v left) → canvas
    // the sideways direction, crossed out: the car cannot go there
    g.strokeStyle = C.dim; g.lineWidth = 1.2; g.setLineDash([3, 3]);
    for (const side of [1, -1]) { const [ax, ay] = R(0, side * 16), [bx, by] = R(0, side * 42); g.beginPath(); g.moveTo(ax, ay); g.lineTo(bx, by); g.stroke(); }
    g.setLineDash([]);
    for (const side of [1, -1]) { const [mx, my] = R(0, side * 50); g.beginPath(); g.moveTo(mx - 4, my - 4); g.lineTo(mx + 4, my + 4); g.moveTo(mx - 4, my + 4); g.lineTo(mx + 4, my - 4); g.stroke(); }
    // the heading angle, measured from the x direction (the dashed reference line, drawn after the car)
    g.strokeStyle = C.red; g.lineWidth = 2; g.beginPath(); g.arc(px, py, 30, 0, -phi, phi > 0); g.stroke();
    g.fillStyle = C.red; g.font = '14px Inter, Arial, sans-serif'; g.fillText('φ', px + 34 * Math.cos(phi / 2) + 2, py - 34 * Math.sin(phi / 2) + 5);
    // the car: body and four wheels
    const poly = pts => { g.beginPath(); pts.forEach(([u, v], i) => { const [qx, qy] = R(u, v); i ? g.lineTo(qx, qy) : g.moveTo(qx, qy); }); g.closePath(); };
    g.fillStyle = C.fg; for (const [u, v] of [[-13, -13], [13, -13], [-13, 13], [13, 13]]) { poly([[u - 5, v - 3], [u + 5, v - 3], [u + 5, v + 3], [u - 5, v + 3]]); g.fill(); }
    poly([[-20, -11], [20, -11], [20, 11], [-20, 11]]); g.fillStyle = C.bg2; g.fill(); g.strokeStyle = C.blue; g.lineWidth = 2.5; g.stroke();
    poly([[8, -8], [16, -8], [16, 8], [8, 8]]); g.fillStyle = C.blue; g.fill();   // the windscreen: the front
    g.strokeStyle = C.red; g.lineWidth = 1.5; g.setLineDash([3, 3]); g.beginPath(); g.moveTo(px, py); g.lineTo(px + 52, py); g.stroke(); g.setLineDash([]);
    // the velocity
    const L = 22 + 40 * speed / 1.6, [ex, ey] = R(L, 0);
    g.strokeStyle = C.green; g.fillStyle = C.green; g.lineWidth = 2.5; g.beginPath(); g.moveTo(px, py); g.lineTo(ex, ey); g.stroke();
    g.beginPath(); g.moveTo(ex + 8 * c, ey - 8 * sn); g.lineTo(ex + 5 * sn, ey + 5 * c); g.lineTo(ex - 5 * sn, ey - 5 * c); g.closePath(); g.fill();
    g.font = 'italic 15px Inter, Arial, sans-serif'; g.fillText('v', ex + 10 * c + 4, ey - 10 * sn + 5);
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H - 2);
  }
  el.classList.add('widget'); el.style.width = W + 'px';
  el.append(canvas);
  if (isPrint()) { draw(cfg.print_s ?? 1.0); return; }
  const t0 = performance.now();
  const shown = () => el.closest('section')?.classList.contains('present') !== false;
  const tick = () => { if (!el.isConnected) return; if (shown()) draw((performance.now() - t0) / 1000 * 0.45); requestAnimationFrame(tick); };
  requestAnimationFrame(tick);
}
