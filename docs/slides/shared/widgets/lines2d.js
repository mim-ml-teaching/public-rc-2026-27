// Linear maps keep lines straight and parallel lines parallel.
// Drag the tips of the images of e1 (red) and e2 (green): they are the columns of M.
// Buttons switch to a projective map (lines stay lines, parallelism lost) and a non-linear map (lines bend);
// the readout always shows the map actually drawn (M, the homography H, or the non-linear formula).
// config: { width, height, m: [a, b, c, d], modes: ['linear', 'projective', 'nonlinear'], mode, autoplay }
// A single mode hides the mode buttons.
import { C, h, fmt, arrow, isPrint } from './util.js';

export function mount(el, cfg) {
  const W = cfg.width || 560, H = cfg.height || 420, S = 64; // px per unit
  const O = [W / 2, H / 2];
  const modes = cfg.modes || ['linear', 'projective', 'nonlinear'];
  const state = { m: cfg.m || [1, 0.6, 0.3, 1], mode: cfg.mode || modes[0], drag: null, t: 1 };
  const PX = 0.12, PY = 0.08; // projective row: w = 1 + PX x' + PY y', with (x', y') = M (x, y)
  const canvas = h('canvas', { width: W, height: H, style: 'touch-action:none; cursor:grab' });
  const g = canvas.getContext('2d');
  const readout = h('div', { style: 'font-size:1.1em' });
  const note = h('div', { class: 'dim', style: 'min-height:3em; max-width:16em' });

  const toC = ([x, y]) => [O[0] + x * S, O[1] - y * S];
  const toW = ([u, v]) => [(u - O[0]) / S, (O[1] - v) / S];
  function map(p, t = state.t) {
    const q = fullMap(p);
    if (!q) return null;
    return [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t];
  }
  function fullMap([x, y]) {
    const [a, b, c, d] = state.m;
    const lx = a * x + b * y, ly = c * x + d * y;
    if (state.mode === 'projective') { const w = 1 + PX * lx + PY * ly; return w > 0.05 ? [lx / w, ly / w] : null; }
    if (state.mode === 'nonlinear') return [lx + 0.25 * Math.sin(ly * 1.1), ly + 0.06 * lx * lx];
    return [lx, ly];
  }

  function polyline(pts, color, width, { before = false } = {}) {
    g.strokeStyle = color; g.lineWidth = width; g.setLineDash(before ? [6, 6] : []); g.beginPath();
    let pen = false;
    for (const p of pts) {
      const q = before ? p : map(p);
      if (!q) { pen = false; continue; }
      const [u, v] = toC(q);
      if (pen) g.lineTo(u, v); else { g.moveTo(u, v); pen = true; }
    }
    g.stroke(); g.setLineDash([]);
  }

  function draw() {
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    // faint original grid
    g.strokeStyle = '#262a33'; g.lineWidth = 1;
    for (let x = O[0] % S; x < W; x += S) { g.beginPath(); g.moveTo(x, 0); g.lineTo(x, H); g.stroke(); }
    for (let y = O[1] % S; y < H; y += S) { g.beginPath(); g.moveTo(0, y); g.lineTo(W, y); g.stroke(); }
    const N = 120, L = 12;
    const seg = (p0, dir) => Array.from({ length: N + 1 }, (_, i) => { const t = -L + 2 * L * i / N; return [p0[0] + t * dir[0], p0[1] + t * dir[1]]; });
    // two families of parallel lines: 3 blue (horizontal) and 3 orange (diagonal); dashed = before, solid = after
    const blue = [-1.5, 0, 1.5].map(k => seg([0, k], [1, 0]));
    const orange = [-2, 0, 2].map(k => seg([k, 0], [1, 1]));
    blue.forEach(l => polyline(l, 'rgba(90,176,255,0.35)', 1.5, { before: true }));
    orange.forEach(l => polyline(l, 'rgba(242,177,52,0.35)', 1.5, { before: true }));
    blue.forEach(l => polyline(l, C.blue, 3));
    orange.forEach(l => polyline(l, C.accent, 3));
    // unit square
    const sq = [[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]].flatMap((p, i, a) => i ? Array.from({ length: 20 }, (_, j) => [a[i - 1][0] + (p[0] - a[i - 1][0]) * j / 19, a[i - 1][1] + (p[1] - a[i - 1][1]) * j / 19]) : []);
    polyline(sq, C.fg, 1.5, { before: true });
    polyline(sq, C.fg, 2.5);
    // images of e1, e2 with drag handles
    const o = toC(map([0, 0]) || [0, 0]);
    const t1 = map([1, 0]), t2 = map([0, 1]);
    if (t1) { const p = toC(t1); arrow(g, o[0], o[1], p[0], p[1], C.red, 3.5); if (state.mode === 'linear') handle(p, C.red); }
    if (t2) { const p = toC(t2); arrow(g, o[0], o[1], p[0], p[1], C.green, 3.5); if (state.mode === 'linear') handle(p, C.green); }
    g.fillStyle = '#fff'; g.beginPath(); g.arc(o[0], o[1], 4, 0, 7); g.fill();

    const [a, b, c, d] = state.m;
    const M = `\\begin{bmatrix}\\color{#ff6b6b}{${fmt(a)}} & \\color{#5fd38d}{${fmt(b)}}\\\\ \\color{#ff6b6b}{${fmt(c)}} & \\color{#5fd38d}{${fmt(d)}}\\end{bmatrix}`;
    readout.innerHTML = {
      linear: `$M = ${M}$`,
      // H = [1 0 0; 0 1 0; PX PY 1] * [M 0; 0 1]
      projective: `$H = \\begin{bmatrix}${fmt(a)} & ${fmt(b)} & 0\\\\ ${fmt(c)} & ${fmt(d)} & 0\\\\ ${fmt(PX * a + PY * c)} & ${fmt(PX * b + PY * d)} & 1\\end{bmatrix}$`,
      nonlinear: `$\\begin{bmatrix}x'\\\\ y'\\end{bmatrix} = ${M}\\begin{bmatrix}x\\\\ y\\end{bmatrix}$<div style="margin-top:0.4em">$f(x, y) = \\begin{bmatrix}x' + 0.25\\sin(1.1\\,y')\\\\ y' + 0.06\\,x'^2\\end{bmatrix}$</div>`,
    }[state.mode];
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
    note.innerHTML = {
      linear: '<b style="color:#5fd38d">linear</b>: straight lines stay straight, parallel lines stay parallel, the origin stays put. Drag the red and green arrow tips (= columns of M).',
      projective: '<b style="color:#b28dff">projective</b>: lines stay straight, but parallel lines now meet.',
      nonlinear: '<b style="color:#ff6b6b">non-linear</b>: lines bend — not a matrix.',
    }[state.mode];
  }
  function handle([u, v], color) {
    g.strokeStyle = color; g.lineWidth = 2; g.fillStyle = C.bg2;
    g.beginPath(); g.arc(u, v, 9, 0, 7); g.fill(); g.stroke();
  }

  // dragging (coordinates corrected for reveal.js scaling)
  const pos = e => { const r = canvas.getBoundingClientRect(); return [(e.clientX - r.left) / r.width * W, (e.clientY - r.top) / r.height * H]; };
  canvas.addEventListener('pointerdown', e => {
    if (state.mode !== 'linear') return;
    const p = pos(e), [a, b, c, d] = state.m;
    const d1 = Math.hypot(...toC([a, c]).map((v, i) => v - p[i])), d2 = Math.hypot(...toC([b, d]).map((v, i) => v - p[i]));
    if (Math.min(d1, d2) < 20) { state.drag = d1 < d2 ? 1 : 2; canvas.setPointerCapture(e.pointerId); e.stopPropagation(); }
  });
  canvas.addEventListener('pointermove', e => {
    if (!state.drag) return;
    let [x, y] = toW(pos(e));
    x = Math.round(x * 10) / 10; y = Math.round(y * 10) / 10; // snap to 0.1
    const m = state.m;
    state.m = state.drag === 1 ? [x, m[1], y, m[3]] : [m[0], x, m[2], y];
    draw();
  });
  canvas.addEventListener('pointerup', () => { state.drag = null; });

  const btn = (mode, label) => {
    const b = h('button', { onclick: () => { state.mode = mode; buttons.forEach(x => x.classList.toggle('active', x === b)); morph(); } }, label);
    return b;
  };
  const labels = { linear: 'linear M', projective: 'projective', nonlinear: 'non-linear' };
  const buttons = modes.map(m => btn(m, labels[m]));
  buttons[modes.indexOf(state.mode)].classList.add('active');
  const reset = h('button', { onclick: () => { state.m = cfg.m || [1, 0.6, 0.3, 1]; draw(); } }, 'reset');
  const legend = h('div', { class: 'dim' }, 'dashed: before · solid: after');
  function morph() {
    const t0 = performance.now();
    const step = now => {
      const u = Math.min(1, (now - t0) / 1400);
      state.t = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2; draw();
      if (u < 1) requestAnimationFrame(step);
    };
    requestAnimationFrame(step);
  }
  const play = h('button', { onclick: morph }, '▶ morph');

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    canvas,
    h('div', { style: 'display:flex; flex-direction:column; gap:0.6em; width:15em; flex:none' },
      readout, note, legend,
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start' },
        ...(modes.length > 1 ? [h('div', { style: 'display:flex; gap:0.3em; flex-wrap:wrap' }, buttons)] : []),
        h('div', { style: 'display:flex; gap:0.3em' }, play, reset)))));
  draw();
  if (cfg.autoplay && !isPrint()) setTimeout(morph, 400);
}
