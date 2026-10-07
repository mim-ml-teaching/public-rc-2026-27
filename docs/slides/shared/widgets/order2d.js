// "Order matters": rotate 90° about the origin and translate by (3, 0), in both orders.
// Plain 2D, math axes (y up). config: { angle: 90, t: [3, 0], autoplay: true }
import { C, h, isPrint, arrow } from './util.js';

const HOUSE = [[-0.5, 0], [0.5, 0], [0.5, 0.7], [0, 1.2], [-0.5, 0.7]];
const DOOR = [[0.1, 0], [0.35, 0], [0.35, 0.45], [0.1, 0.45]];

export function mount(el, cfg) {
  const ang = (cfg.angle ?? 90) * Math.PI / 180, tv = cfg.t || [3, 0];
  const W = cfg.width || 470, H = cfg.height || 360, S = 50; // px per unit
  const panels = [
    { title: 'rotate, then translate:  T R p', ops: ['R', 'T'], color: C.blue },
    { title: 'translate, then rotate:  R T p', ops: ['T', 'R'], color: C.accent },
  ].map(p => ({ ...p, canvas: h('canvas', { width: W, height: H }) }));
  const state = { s: isPrint() ? 2 : (cfg.s ?? 0) };

  // map a point through the first `s` (0..2, fractional) operations
  function apply(ops, s, [x, y]) {
    for (let i = 0; i < 2; i++) {
      const u = Math.max(0, Math.min(1, s - i));
      if (u === 0) break;
      if (ops[i] === 'R') { const a = ang * u, c = Math.cos(a), sn = Math.sin(a); [x, y] = [c * x - sn * y, sn * x + c * y]; }
      else { x += tv[0] * u; y += tv[1] * u; }
    }
    return [x, y];
  }

  function drawPanel(p) {
    const g = p.canvas.getContext('2d');
    const O = [W * 0.38, H * 0.58];
    const toC = ([x, y]) => [O[0] + x * S, O[1] - y * S];
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    g.strokeStyle = '#262a33'; g.lineWidth = 1;
    for (let x = O[0] % S; x < W; x += S) { g.beginPath(); g.moveTo(x, 0); g.lineTo(x, H); g.stroke(); }
    for (let y = O[1] % S; y < H; y += S) { g.beginPath(); g.moveTo(0, y); g.lineTo(W, y); g.stroke(); }
    arrow(g, 8, O[1], W - 8, O[1], C.dim, 1.5); arrow(g, O[0], H - 8, O[0], 8, C.dim, 1.5);
    g.fillStyle = C.dim; g.font = '16px Inter, Arial'; g.fillText('x', W - 20, O[1] - 8); g.fillText('y', O[0] + 8, 20);
    g.fillStyle = C.red; g.beginPath(); g.arc(O[0], O[1], 5, 0, 7); g.fill();
    g.fillText('origin (centre of rotation)', O[0] + 8, O[1] + 20);

    const shape = (pts, fill, stroke, dash = []) => {
      g.beginPath(); pts.map(toC).forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y))); g.closePath();
      if (fill) { g.fillStyle = fill; g.fill(); }
      g.setLineDash(dash); g.strokeStyle = stroke; g.lineWidth = 2; g.stroke(); g.setLineDash([]);
    };
    const house = (s, alpha, dash) => {
      const tr = q => apply(p.ops, s, q);
      g.globalAlpha = alpha;
      shape(HOUSE.map(tr), p.color + '55', p.color, dash);
      shape(DOOR.map(tr), '#6b4226', '#6b4226');
      g.globalAlpha = 1;
    };
    house(0, 0.35, [5, 4]);                         // start
    if (state.s > 1) house(1, 0.5, [5, 4]);          // after the first operation
    house(state.s, 1, []);                           // current
    // path of the house's base point
    g.strokeStyle = p.color; g.setLineDash([2, 4]); g.lineWidth = 1.5; g.beginPath();
    for (let s = 0; s <= state.s + 1e-9; s += 0.02) { const [x, y] = toC(apply(p.ops, s, [0, 0.35])); s ? g.lineTo(x, y) : g.moveTo(x, y); }
    g.stroke(); g.setLineDash([]);
    g.fillStyle = C.fg; g.font = '18px Inter, Arial'; g.fillText(p.title, 12, H - 14);
  }
  const draw = () => panels.forEach(drawPanel);

  const slider = h('input', { type: 'range', min: 0, max: 2, step: 0.01, value: state.s });
  slider.addEventListener('input', () => { state.s = parseFloat(slider.value); draw(); });
  const play = h('button', {
    onclick: () => {
      const t0 = performance.now();
      const step = now => {
        const u = Math.min(1, (now - t0) / 2600);
        state.s = 2 * u; slider.value = state.s; draw();
        if (u < 1) requestAnimationFrame(step);
      };
      requestAnimationFrame(step);
    },
  }, '▶ play');

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em' }, panels.map(p => p.canvas)),
    h('div', { class: 'wctl interactive-only' }, play, h('label', {}, 'step'), slider,
      h('span', { class: 'dim' }, 'R = rotation by 90° about the origin, T = translation by (3, 0)')));
  draw();
  if (cfg.autoplay && !isPrint()) setTimeout(() => play.click(), 500);
}
