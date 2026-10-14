// RANSAC for robust line fitting.
// One iteration: draw s = 2 random points, fit the line through them, count the points within ±threshold of it
// (the consensus set). Keep the line with the most inliers; finally refit by least squares on its inliers.
// For comparison: a least-squares fit to all points, pulled away by the outliers.
// Side panel: the iteration-count formula N = log(1 − p) / log(1 − wˢ) with w = inlier fraction, p = 0.99.
//
// Least squares here is the orthogonal (total) least-squares line: the principal direction of the points.
// Click the canvas to add a point.
//
// config: {
//   width: 560, height: 400
//   n: 40               number of points
//   outliers: 0.4       fraction of outliers
//   threshold: 12       inlier band half-width, px
//   seed: 7             data and sampling are deterministic for a given seed
//   autoplay: false     animate a fresh run on mount (not in print)
// }
// Default (and print) state: N iterations already done, best line + its band, least-squares line, refit shown.
import { C, h, fmt, isPrint } from './util.js';

function rng(seed) { // mulberry32
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

const P_SUCCESS = 0.99;
const iterations = (w, s, p = P_SUCCESS) => {
  const ws = Math.pow(w, s);
  if (ws >= 1) return 1;
  if (ws <= 0) return Infinity;
  return Math.ceil(Math.log(1 - p) / Math.log(1 - ws));
};

// Line as (a, b, c) with a x + b y + c = 0 and a² + b² = 1.
function lineThrough(p, q) {
  const dx = q[0] - p[0], dy = q[1] - p[1], L = Math.hypot(dx, dy);
  if (L < 1e-9) return null;
  const a = -dy / L, b = dx / L;
  return [a, b, -(a * p[0] + b * p[1])];
}
function fitTLS(pts) {
  if (pts.length < 2) return null;
  let mx = 0, my = 0;
  pts.forEach(([x, y]) => { mx += x; my += y; });
  mx /= pts.length; my /= pts.length;
  let sxx = 0, sxy = 0, syy = 0;
  pts.forEach(([x, y]) => { const u = x - mx, v = y - my; sxx += u * u; sxy += u * v; syy += v * v; });
  const ang = 0.5 * Math.atan2(2 * sxy, sxx - syy); // principal direction
  const d = [Math.cos(ang), Math.sin(ang)];
  return lineThrough([mx, my], [mx + d[0], my + d[1]]);
}
const dist = (l, [x, y]) => Math.abs(l[0] * x + l[1] * y + l[2]);

export function mount(el, cfg) {
  const W = cfg.width || 560, H = cfg.height || 400;
  const n = cfg.n || 40, thr = cfg.threshold || 12;
  const state = {
    outliers: cfg.outliers ?? 0.4, seed: cfg.seed ?? 7,
    pts: [], k: 0, cur: null, best: null, ls: true, refit: null, sampler: null,
  };

  const canvas = h('canvas', { width: W, height: H, style: 'cursor:crosshair' });
  const g = canvas.getContext('2d');
  const readout = h('div', { style: 'font-size:1.05em; line-height:1.5' });
  const formula = h('div', { style: 'line-height:1.45' });

  function makeData() {
    const r = rng(state.seed * 9973 + 1);
    const gauss = () => Math.sqrt(-2 * Math.log(1 - r())) * Math.cos(2 * Math.PI * r());
    // the line lies off-centre, so the outliers sit mostly on one side and visibly pull the least-squares fit
    const ang = (-25 + 50 * r()) * Math.PI / 180, cx = W * (0.4 + 0.2 * r()), cy = H * (r() < 0.5 ? 0.25 : 0.75) + H * 0.06 * (r() - 0.5);
    const nOut = Math.round(n * state.outliers), nIn = n - nOut;
    const pts = [];
    for (let i = 0; i < nIn; i++) {
      const t = (-0.45 + 0.9 * r()) * W;
      pts.push({ p: [cx + t * Math.cos(ang) + 4 * gauss() * -Math.sin(ang), cy + t * Math.sin(ang) + 4 * gauss() * Math.cos(ang)] });
    }
    for (let i = 0; i < nOut; i++) pts.push({ p: [20 + (W - 40) * r(), 20 + (H - 40) * r()] });
    state.pts = pts.filter(({ p }) => p[0] > 8 && p[0] < W - 8 && p[1] > 8 && p[1] < H - 8);
    reset();
  }
  function reset() {
    state.k = 0; state.cur = null; state.best = null; state.refit = null;
    state.sampler = rng(state.seed * 31 + 5);
  }
  const P = () => state.pts.map(o => o.p);

  function step() {
    const pts = P(), N = pts.length;
    if (N < 2) return;
    let i = Math.floor(state.sampler() * N), j = Math.floor(state.sampler() * (N - 1));
    if (j >= i) j++;
    const line = lineThrough(pts[i], pts[j]);
    if (!line) return;
    const inl = pts.map(p => dist(line, p) <= thr);
    const count = inl.filter(Boolean).length;
    state.k++;
    state.cur = { line, sample: [i, j], inl, count };
    if (!state.best || count > state.best.count) state.best = state.cur;
    state.refit = null;
  }
  function doRefit() {
    if (!state.best) return;
    state.refit = fitTLS(P().filter((_, i) => state.best.inl[i]));
  }

  // ---- drawing ----
  function lineEnds(l) {
    const [a, b, c] = l, pts = [];
    if (Math.abs(b) > 1e-9) { pts.push([-50, -(a * -50 + c) / b], [W + 50, -(a * (W + 50) + c) / b]); }
    else { pts.push([-c / a, -50], [-c / a, H + 50]); }
    return pts;
  }
  function drawLine(l, color, width, dash = []) {
    const [p, q] = lineEnds(l);
    g.strokeStyle = color; g.lineWidth = width; g.setLineDash(dash);
    g.beginPath(); g.moveTo(...p); g.lineTo(...q); g.stroke(); g.setLineDash([]);
  }
  function drawBand(l) {
    const [p, q] = lineEnds(l), [a, b] = l;
    g.fillStyle = 'rgba(242,177,52,0.13)';
    g.beginPath();
    g.moveTo(p[0] + a * thr, p[1] + b * thr); g.lineTo(q[0] + a * thr, q[1] + b * thr);
    g.lineTo(q[0] - a * thr, q[1] - b * thr); g.lineTo(p[0] - a * thr, p[1] - b * thr); g.closePath(); g.fill();
    g.strokeStyle = 'rgba(242,177,52,0.45)'; g.lineWidth = 1; g.setLineDash([5, 5]);
    for (const s of [1, -1]) { g.beginPath(); g.moveTo(p[0] + s * a * thr, p[1] + s * b * thr); g.lineTo(q[0] + s * a * thr, q[1] + s * b * thr); g.stroke(); }
    g.setLineDash([]);
  }

  function draw() {
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    const pts = P(), cur = state.cur;
    if (cur) drawBand(cur.line);
    if (state.ls) { const l = fitTLS(pts); if (l) drawLine(l, C.red, 2.5, [10, 6]); }
    if (state.best && state.best !== cur) drawLine(state.best.line, C.green, 2, [4, 4]);
    if (cur) drawLine(cur.line, C.accent, 2.5);
    if (state.refit) drawLine(state.refit, C.green, 4);
    pts.forEach((p, i) => {
      const inl = cur?.inl[i];
      g.fillStyle = inl ? C.blue : '#7d8594';
      g.beginPath(); g.arc(p[0], p[1], inl ? 6 : 5, 0, 7); g.fill();
    });
    if (cur) for (const i of cur.sample) {
      g.strokeStyle = C.accent; g.lineWidth = 3;
      g.beginPath(); g.arc(pts[i][0], pts[i][1], 11, 0, 7); g.stroke();
    }
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H - 2);

    // readout
    const N = pts.length;
    readout.innerHTML =
      `<div>iteration <b>${state.k}</b></div>` +
      `<div>this sample: <span style="color:${C.blue}">${cur ? cur.count : '—'}</span> inliers</div>` +
      `<div>best so far: <span style="color:${C.green}">${state.best ? state.best.count : '—'}</span> / ${N}</div>`;
    const w = 1 - state.outliers;
    const td = 'style="padding:0.1em 0.5em"';
    const rows = [[2, 'line'], [4, 'P4P'], [6, 'DLT']].map(([s, m]) =>
      `<tr><td ${td}>${m}</td><td ${td}>${s}</td><td ${td}>${fmt(Math.pow(w, s), 3)}</td><td ${td}><b>${iterations(w, s)}</b></td></tr>`).join('');
    formula.innerHTML =
      `<div>$N = \\dfrac{\\log(1-p)}{\\log(1-w^s)}$</div>` +
      `<div class="dim">$p = ${P_SUCCESS}$, inlier fraction $w = ${fmt(w, 2)}$</div>` +
      `<table style="font-size:0.95em; margin:0.2em 0; border-collapse:collapse"><tr><th ${td}>model</th><th ${td}>$s$</th><th ${td}>$w^s$</th><th ${td}>$N$</th></tr>${rows}</table>`;
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
    window.renderMathInElement?.(formula, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
  }

  // ---- controls ----
  let timer = null;
  const stop = () => { if (timer) { clearTimeout(timer); timer = null; } runBtn.textContent = '▶ run'; };
  function run() {
    if (timer) { stop(); return; }
    reset(); draw();
    const N = Math.max(5, Math.min(60, iterations(1 - state.outliers, 2)));
    runBtn.textContent = '■ stop';
    const tick = () => {
      step(); draw();
      if (state.k < N) timer = setTimeout(tick, 380);
      else { timer = setTimeout(() => { doRefit(); state.cur = state.best; draw(); stop(); }, 500); }
    };
    tick();
  }
  const stepBtn = h('button', { onclick: () => { stop(); step(); draw(); } }, 'step');
  const runBtn = h('button', { onclick: run }, '▶ run');
  const refitBtn = h('button', { onclick: () => { stop(); doRefit(); if (state.best) state.cur = state.best; draw(); } }, 'refit on inliers');
  const lsBtn = h('button', { onclick: () => { state.ls = !state.ls; lsBtn.classList.toggle('active', state.ls); draw(); } }, 'least squares');
  lsBtn.classList.toggle('active', state.ls);
  const newBtn = h('button', { onclick: () => { stop(); state.seed++; makeData(); draw(); } }, 'new data');
  const outVal = h('span', { class: 'readout', style: 'display:inline-block; width:2.6em; text-align:right' }, `${Math.round(state.outliers * 100)}%`);
  const outSlider = h('input', { type: 'range', min: 0, max: 0.8, step: 0.05, value: state.outliers, style: 'width:7.5em' });
  outSlider.addEventListener('input', () => {
    stop(); state.outliers = parseFloat(outSlider.value); outVal.textContent = `${Math.round(state.outliers * 100)}%`;
    makeData(); initialRun(); draw();
  });
  canvas.addEventListener('click', e => {
    const r = canvas.getBoundingClientRect();
    state.pts.push({ p: [(e.clientX - r.left) / r.width * W, (e.clientY - r.top) / r.height * H] });
    stop(); reset(); draw();
  });

  const legend = h('div', { style: 'line-height:1.35; font-size:0.92em' },
    h('div', {}, h('span', { style: `color:${C.accent}` }, '━ '), 'current sample + band'),
    h('div', {}, h('span', { style: `color:${C.green}` }, '━ '), 'best / refit on inliers'),
    h('div', {}, h('span', { style: `color:${C.red}` }, '╍ '), 'least squares, all points'));

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${W}px` }, canvas,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, stepBtn, runBtn, refitBtn, lsBtn, newBtn),
      h('div', { class: 'wctl interactive-only', style: 'gap:0.4em' },
        h('label', {}, 'outliers'), outSlider, outVal, h('span', { class: 'dim', style: 'margin-left:0.6em' }, 'click to add a point'))),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:15em; flex:none' },
      readout, legend, formula)));

  // default / print state: N seeded iterations done, best consensus refitted
  function initialRun() {
    reset();
    const N = Math.max(5, Math.min(60, iterations(1 - state.outliers, 2)));
    for (let i = 0; i < N; i++) step();
    doRefit(); state.cur = state.best;
  }
  makeData();
  initialRun();
  draw();
  if (cfg.autoplay && !isPrint()) setTimeout(run, 500);
}
