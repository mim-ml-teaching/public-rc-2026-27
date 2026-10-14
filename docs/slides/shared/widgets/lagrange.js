// Constrained optimisation on the unit circle: the Lagrange condition ∇f ∥ ∇g, seen as tangency of a
// contour line of f with the constraint curve g(x, y) = x² + y² − 1 = 0.
//
// mode "x2y":      f(x, y) = x² y (the example from the note "Intuition behind the Lagrangian method").
//                  Candidates: (0, ±1) where ∇f = 0, and (±√(2/3), ±1/√3); the maximum is (√(2/3), 1/√3), f = 2/(3√3).
// mode "quadform": L(p) = ‖Ap‖² = pᵀ M p with M = AᵀA (2×2, symmetric positive definite). Candidates are the unit
//                  eigenvectors of M (M p = λ p, L = λ); the minimum is the eigenvector of the smallest eigenvalue.
//
// The point on the circle can be dragged (along the circle) or set with the angle slider. Arrows: ∇f or ∇L (accent),
// ∇g (blue); only their directions are drawn. The purple curve is the contour through the current point.
//
// config: {
//   mode: "x2y" | "quadform"
//   width: 520, height: 420
//   angle: initial angle of the point in degrees (default: 120 for x2y, 100 for quadform; in print: the optimum)
//   candidates: show all candidate points / eigenvectors (default: only in print)
//   lambdas: [1, 4], theta: 30      (quadform) eigenvalues of M and the angle of the first eigenvector, degrees
// }
import { C, h, fmt, arrow, isPrint } from './util.js';
import { LIGHT } from '../theme.js';

const deg = Math.PI / 180;

export function mount(el, cfg) {
  const mode = cfg.mode === 'quadform' ? 'quadform' : 'x2y';
  const W = cfg.width || 520, H = cfg.height || 420;
  const R = 1.6;                                   // world range [-R, R] on the shorter side
  const S = Math.min(W, H) / (2 * R + 0.15);       // px per unit
  const O = [W / 2, H / 2];
  const toC = ([x, y]) => [O[0] + x * S, O[1] - y * S];
  const toW = ([u, v]) => [(u - O[0]) / S, (O[1] - v) / S];

  // ---- the objective ----
  let fun, grad, candidates, optAngle, optName;
  const [l1, l2] = cfg.lambdas || [1, 4];
  const th = (cfg.theta ?? 30) * deg;
  const v1 = [Math.cos(th), Math.sin(th)], v2 = [-Math.sin(th), Math.cos(th)];
  const M = [ // M = l1 v1 v1ᵀ + l2 v2 v2ᵀ
    l1 * v1[0] * v1[0] + l2 * v2[0] * v2[0], l1 * v1[0] * v1[1] + l2 * v2[0] * v2[1],
    l1 * v1[1] * v1[0] + l2 * v2[1] * v2[0], l1 * v1[1] * v1[1] + l2 * v2[1] * v2[1],
  ];
  if (mode === 'x2y') {
    fun = ([x, y]) => x * x * y;
    grad = ([x, y]) => [2 * x * y, x * x];
    const a = Math.sqrt(2 / 3), b = 1 / Math.sqrt(3);
    candidates = [[0, 1], [0, -1], [a, b], [-a, b], [a, -b], [-a, -b]].map(p => ({ p, f: fun(p) }));
    optAngle = Math.atan2(b, a) / deg; optName = 'maximum';
  } else {
    fun = ([x, y]) => x * (M[0] * x + M[1] * y) + y * (M[2] * x + M[3] * y);
    grad = ([x, y]) => [2 * (M[0] * x + M[1] * y), 2 * (M[2] * x + M[3] * y)];
    candidates = [v1, v1.map(c => -c), v2, v2.map(c => -c)].map(p => ({ p, f: fun(p) }));
    optAngle = Math.atan2(v1[1], v1[0]) / deg; optName = 'minimum';
  }
  const fMax = mode === 'x2y' ? 1.2 : Math.max(l1, l2) * 2 * R * R;

  const state = {
    angle: cfg.angle ?? (isPrint() ? optAngle : (mode === 'x2y' ? 120 : 100)),
    cand: cfg.candidates ?? isPrint(),
  };

  // ---- DOM ----
  const canvas = h('canvas', { width: W, height: H, style: 'touch-action:none; cursor:grab' });
  const g = canvas.getContext('2d');
  const readout = h('div', { style: 'font-size:1.05em; line-height:1.55' });
  const note = h('div', { style: 'min-height:4.6em; line-height:1.35' });

  // ---- heatmap (computed once; pixel colours picked per theme) ----
  const heat = document.createElement('canvas');
  heat.width = W; heat.height = H;
  {
    const hg = heat.getContext('2d'), im = hg.createImageData(W, H), d = im.data;
    const bg = LIGHT ? [243, 244, 246] : [30, 33, 40];
    const pos = LIGHT ? [178, 122, 0] : [242, 177, 52], neg = LIGHT ? [28, 110, 208] : [90, 176, 255];
    const amt = LIGHT ? 0.32 : 0.36;
    for (let v = 0; v < H; v++) for (let u = 0; u < W; u++) {
      const f = fun(toW([u + 0.5, v + 0.5]));
      const t = Math.tanh(2.2 * f / fMax), c = t >= 0 ? pos : neg, k = amt * Math.abs(t);
      const i = 4 * (v * W + u);
      for (let j = 0; j < 3; j++) d[i + j] = bg[j] + (c[j] - bg[j]) * k;
      d[i + 3] = 255;
    }
    hg.putImageData(im, 0, 0);
  }

  // ---- contours: x²y = c  ⇔  y = c / x²;  pᵀMp = c  ⇔  ellipse ----
  function contour(c) {
    const lines = [];
    if (mode === 'x2y') {
      if (Math.abs(c) < 1e-9) return [[[-3, 0], [3, 0]], [[0, -3], [0, 3]]];
      for (const s of [-1, 1]) {
        const pts = [];
        for (let i = 0; i <= 400; i++) {
          const x = s * (0.02 + 2.6 * i / 400), y = c / (x * x);
          if (Math.abs(y) < 3) pts.push([x, y]);
        }
        lines.push(pts);
      }
    } else {
      if (c <= 0) return [];
      const pts = [];
      for (let i = 0; i <= 200; i++) {
        const t = 2 * Math.PI * i / 200, a = Math.sqrt(c / l1) * Math.cos(t), b = Math.sqrt(c / l2) * Math.sin(t);
        pts.push([a * v1[0] + b * v2[0], a * v1[1] + b * v2[1]]);
      }
      lines.push(pts);
    }
    return lines;
  }
  function stroke(lines, color, width, dash = []) {
    g.strokeStyle = color; g.lineWidth = width; g.setLineDash(dash);
    for (const pts of lines) {
      g.beginPath();
      pts.forEach((p, i) => { const [u, v] = toC(p); i ? g.lineTo(u, v) : g.moveTo(u, v); });
      g.stroke();
    }
    g.setLineDash([]);
  }
  const levels = mode === 'x2y'
    ? [0, 0.05, -0.05, 0.15, -0.15, 0.3, -0.3, 0.5, -0.5, 0.8, -0.8]
    : [0.5, 1, 2, 3, 4, 6, 8, 11];

  const point = () => [Math.cos(state.angle * deg), Math.sin(state.angle * deg)];

  function draw() {
    g.drawImage(heat, 0, 0);
    for (const c of levels) stroke(contour(c), '#5a606c', 1.2);
    // constraint: the unit circle
    g.strokeStyle = C.blue; g.lineWidth = 3; g.beginPath(); g.arc(O[0], O[1], S, 0, 7); g.stroke();
    // axes ticks
    g.fillStyle = C.dim; g.font = '19px Inter, Arial';
    g.fillText('x', W - 22, O[1] - 8); g.fillText('y', O[0] + 8, 20);

    const p = point(), fp = fun(p);
    // contour through the current point
    stroke(contour(fp), C.purple, 2.5, [8, 5]);

    if (state.cand) {
      for (const { p: q, f } of candidates) {
        const [u, v] = toC(q);
        const best = mode === 'x2y' ? f > 0.3 : Math.abs(f - Math.min(l1, l2)) < 1e-9;
        const worst = mode === 'x2y' ? f < -0.3 : Math.abs(f - Math.max(l1, l2)) < 1e-9;
        const col = best ? C.green : worst ? C.red : C.fg;
        g.fillStyle = col; g.beginPath(); g.arc(u, v, 7, 0, 7); g.fill();
        const txt = (mode === 'x2y' ? 'f = ' : 'L = ') + fmt(f, mode === 'x2y' ? 3 : 2);
        g.font = 'bold 18px Inter, Arial';
        const tw = g.measureText(txt).width, dx = q[0], dy = -q[1];
        const lx = u + dx * 24 - tw / 2 + dx * tw / 2, ly = v + dy * 24 + 6;
        g.fillStyle = 'rgba(21,23,28,0.75)'; g.fillRect(lx - 4, ly - 17, tw + 8, 23);
        g.fillStyle = col; g.fillText(txt, lx, ly);
      }
    }

    // gradients at the point (directions only)
    const [u, v] = toC(p), gf = grad(p), gg = [2 * p[0], 2 * p[1]];
    const nf = Math.hypot(...gf), ng = Math.hypot(...gg);
    if (nf > 1e-6) { const q = toC([p[0] + 0.62 * gf[0] / nf, p[1] + 0.62 * gf[1] / nf]); arrow(g, u, v, q[0], q[1], C.accent, 4); }
    { const q = toC([p[0] + 0.42 * gg[0] / ng, p[1] + 0.42 * gg[1] / ng]); arrow(g, u, v, q[0], q[1], C.blue, 4); }
    g.fillStyle = C.fg; g.strokeStyle = C.bg; g.lineWidth = 2;
    g.beginPath(); g.arc(u, v, 9, 0, 7); g.fill(); g.stroke();
    g.fillStyle = C.fg; g.font = 'italic bold 22px "Times New Roman", serif';
    g.fillText('p', u + (p[0] >= 0 ? 12 : -24), v + (p[1] >= 0 ? 22 : -10));

    // readout
    const fname = mode === 'x2y' ? 'f' : 'L';
    let ang = null;
    if (nf > 1e-6) {
      const c = (gf[0] * gg[0] + gf[1] * gg[1]) / (nf * ng);
      ang = Math.acos(Math.max(-1, Math.min(1, c))) / deg;
    }
    const parallel = ang === null || ang < 1 || ang > 179;
    const Mtex = `M = \\begin{bmatrix}${fmt(M[0])} & ${fmt(M[1])}\\\\ ${fmt(M[2])} & ${fmt(M[3])}\\end{bmatrix}`;
    readout.innerHTML =
      (mode === 'quadform' ? `<div>$${Mtex}$</div>` : '') +
      `<div>$p = (${fmt(p[0])},\\ ${fmt(p[1])})$</div>` +
      `<div>contour: $\\textcolor{${C.purple}}{${fname} = ${fname}(p) = ${fmt(fp, mode === 'x2y' ? 3 : 2)}}$</div>` +
      `<div>angle$(\\textcolor{${C.accent}}{\\nabla ${fname}},\\ \\textcolor{${C.blue}}{\\nabla g}) = ${ang === null ? '—' : fmt(ang, 0) + '^\\circ'}$</div>`;
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });

    let msg;
    if (ang === null) msg = `<b style="color:${C.green}">$\\nabla f = 0$</b>: a candidate with $\\lambda = 0$.`;
    else if (parallel) msg = `<b style="color:${C.green}">$\\nabla ${fname} \\parallel \\nabla g$</b>: the contour touches the circle — a <b>candidate</b>.`;
    else msg = `Not parallel: moving along the circle still changes $${fname}$ — not optimal.`;
    if (state.cand) msg += mode === 'x2y'
      ? ' <span class="dim">All candidates marked; the best one is found by evaluating $f$.</span>'
      : ` <span class="dim">Candidates = unit eigenvectors, $L = \\lambda$. Minimum: smallest eigenvalue $\\lambda = ${fmt(Math.min(l1, l2))}$.</span>`;
    note.innerHTML = msg;
    window.renderMathInElement?.(note, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
    slider.value = String(((state.angle % 360) + 360) % 360);
  }

  // ---- interaction ----
  const pos = e => { const r = canvas.getBoundingClientRect(); return [(e.clientX - r.left) / r.width * W, (e.clientY - r.top) / r.height * H]; };
  let dragging = false;
  const setFromPointer = e => { const [x, y] = toW(pos(e)); state.angle = Math.atan2(y, x) / deg; draw(); };
  canvas.addEventListener('pointerdown', e => {
    const [u, v] = pos(e), r = Math.hypot(u - O[0], v - O[1]);
    if (Math.abs(r - S) < 28) { dragging = true; canvas.setPointerCapture(e.pointerId); e.stopPropagation(); setFromPointer(e); }
  });
  canvas.addEventListener('pointermove', e => { if (dragging) setFromPointer(e); });
  canvas.addEventListener('pointerup', () => { dragging = false; });

  const slider = h('input', { type: 'range', min: 0, max: 360, step: 0.5, value: state.angle });
  slider.addEventListener('input', () => { state.angle = parseFloat(slider.value); draw(); });

  let anim = 0;
  function animateTo(target) {
    // nearest equivalent optimum (for quadform, ±v1 are both minima)
    const targets = mode === 'quadform' ? [target, target + 180] : [target];
    let best = null;
    for (const t of targets) {
      const d = ((t - state.angle) % 360 + 540) % 360 - 180;
      if (best === null || Math.abs(d) < Math.abs(best)) best = d;
    }
    const a0 = state.angle, t0 = performance.now(), id = ++anim;
    const step = now => {
      if (id !== anim) return;
      const u = Math.min(1, (now - t0) / 1200), e = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
      state.angle = a0 + best * e; draw();
      if (u < 1) requestAnimationFrame(step);
    };
    requestAnimationFrame(step);
  }
  const findBtn = h('button', { onclick: () => animateTo(optAngle) }, `find ${optName}`);
  const candBtn = h('button', { onclick: () => { state.cand = !state.cand; candBtn.classList.toggle('active', state.cand); draw(); } },
    mode === 'x2y' ? 'all candidates' : 'eigenvectors');
  candBtn.classList.toggle('active', state.cand);

  // legend: what is dragged and what the curves are
  const sw = svg => `<svg width="30" height="14" style="flex:none; vertical-align:middle">${svg}</svg>`;
  const legend = h('div', { style: 'font-size:0.82em; line-height:1.3; display:flex; flex-direction:column; gap:0.2em' });
  legend.innerHTML = [
    [sw(`<circle cx="15" cy="7" r="6" fill="${C.fg}"/>`), 'point $p$ on the circle $g = 0$<span class="interactive-only">: <b>drag it</b></span>'],
    [sw(`<line x1="1" y1="7" x2="29" y2="7" stroke="${C.purple}" stroke-width="3" stroke-dasharray="7 4"/>`), `the contour line of $${mode === 'x2y' ? 'f' : 'L'}$ through $p$`],
    [sw(`<line x1="1" y1="7" x2="29" y2="7" stroke="#5a606c" stroke-width="2"/>`), `other contour lines of $${mode === 'x2y' ? 'f' : 'L'}$`],
  ].map(([s, t]) => `<div style="display:flex; gap:0.4em; align-items:center">${s}<span>${t}</span></div>`).join('');
  window.renderMathInElement?.(legend, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    canvas,
    h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:15em; flex:none' },
      legend, readout, note,
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.35em' },
                h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', {}, 'angle'), slider),
        h('div', { style: 'display:flex; gap:0.3em; flex-wrap:wrap' }, findBtn, candBtn)))));
  draw();
}
