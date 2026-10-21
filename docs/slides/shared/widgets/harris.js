// What is a good keypoint? The Harris corner detector on a synthetic greyscale image.
// Click a pixel (or a preset): a window of win × win pixels around it is shifted by (Δu, Δv) within ±8 px and
// compared with itself, E(Δu, Δv) = Σ_window (I(x + Δ) − I(x))², shown as a heatmap (flat: ≈ 0 everywhere,
// edge: a valley along the edge, corner: a bowl). For small shifts E ≈ Δᵀ M Δ with the structure tensor
// M = Σ_window [Ix² IxIy; IxIy Iy²]; the second panel draws the level set Δᵀ M Δ = const of this quadratic
// approximation (in the same ±8 px axes, so it is the approximate contour of the heatmap) with the eigenvector
// directions. The third panel is the λ1–λ2 plane with the regions flat / edge / corner given by the Harris
// response R = det M − k (tr M)² = λ1 λ2 − k (λ1 + λ2)², k = 0.05, and the current window marked.
// M and R are reported per pixel of the window (divided by win²), in grey levels² / px².
// "Harris corners": local maxima of R, computed with a 7 × 7 box window at every pixel, above a threshold
// of 1% of the strongest response, with non-maximum suppression.
// The image is 320 × 240, grey levels 0–255, with flat areas, straight and curved edges, corners, a checkerboard
// and a random texture, plus a little noise (σ = 2) so that flat areas are not exactly constant.
//
// config: {
//   width: 560        displayed width of the image (the image is 320 × 240)
//   win: 15           window size for the clicked point (odd)
//   preset: "corner"  initial point: "flat" | "edge" | "corner" | "texture" | [x, y] in image pixels
//   corners: false    show the detected Harris corners (always on in print)
// }
// Print / default state: the preset point with its panels; in print also the corners.
import { C, h, fmt, isPrint } from './util.js';
import { LIGHT } from '../theme.js';

const IW = 320, IH = 240, SH = 8;     // image size, max shift of the SSD surface
const K = 0.05, RT = 1000;            // Harris k; |R| threshold (per-pixel units) between flat and edge / corner
const EMAX = 2500;                    // heatmap scale: mean squared difference per pixel at full colour
const LMAX = 700;                     // λ axis range of the diagram (per-pixel units)
const PRESETS = { flat: [196, 120], edge: [80, 130], corner: [130, 130], texture: [272, 172] };

// ---- the image ----
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

function makeImage() {
  const c = document.createElement('canvas'); c.width = IW; c.height = IH;
  const g = c.getContext('2d');
  const grey = v => `rgb(${v},${v},${v})`;
  g.fillStyle = grey(165); g.fillRect(0, 0, IW, IH);
  g.fillStyle = grey(70); g.beginPath(); g.arc(70, 62, 32, 0, 2 * Math.PI); g.fill();           // disk: edges, no corners
  g.fillStyle = grey(55); g.fillRect(30, 130, 100, 85);                                          // rectangle
  g.fillStyle = grey(235); g.beginPath(); g.moveTo(160, 22); g.lineTo(302, 22); g.lineTo(240, 102); g.closePath(); g.fill();
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {                                      // checkerboard
    g.fillStyle = grey((i + j) & 1 ? 230 : 40); g.fillRect(160 + 15 * i, 140 + 15 * j, 15, 15);
  }
  const d = g.getImageData(0, 0, IW, IH).data;
  const I = new Float32Array(IW * IH);
  for (let i = 0; i < IW * IH; i++) I[i] = d[4 * i];
  // random texture patch: smooth value noise
  const r = rng(11), cell = 6, gw = 16, gh = 18, lat = Array.from({ length: gw * gh }, () => r());
  const sm = t => t * t * (3 - 2 * t);
  for (let y = 130; y < 216; y++) for (let x = 238; x < 306; x++) {
    const fx = (x - 238) / cell, fy = (y - 130) / cell, i = Math.floor(fx), j = Math.floor(fy), u = sm(fx - i), v = sm(fy - j);
    const L = (a, b) => lat[Math.min(gh - 1, b) * gw + Math.min(gw - 1, a)];
    const n = (L(i, j) * (1 - u) + L(i + 1, j) * u) * (1 - v) + (L(i, j + 1) * (1 - u) + L(i + 1, j + 1) * u) * v;
    I[y * IW + x] = 50 + 180 * n;
  }
  const rn = rng(5);
  for (let i = 0; i < IW * IH; i++) {
    const nz = Math.sqrt(-2 * Math.log(1 - rn())) * Math.cos(2 * Math.PI * rn());
    I[i] = Math.max(0, Math.min(255, I[i] + 2 * nz));
  }
  return I;
}

// ---- gradients, structure tensor, Harris response ----
function gradients(I) {
  const Ix = new Float32Array(IW * IH), Iy = new Float32Array(IW * IH);
  for (let y = 0; y < IH; y++) for (let x = 0; x < IW; x++) {
    const xl = Math.max(0, x - 1), xr = Math.min(IW - 1, x + 1), yu = Math.max(0, y - 1), yd = Math.min(IH - 1, y + 1);
    Ix[y * IW + x] = (I[y * IW + xr] - I[y * IW + xl]) / (xr - xl);
    Iy[y * IW + x] = (I[yd * IW + x] - I[yu * IW + x]) / (yd - yu);
  }
  return { Ix, Iy };
}

// Structure tensor of the window centred at (cx, cy), per pixel: [a, b, c] = mean of [Ix², IxIy, Iy²].
function tensorAt(G, cx, cy, win) {
  const r = (win - 1) / 2;
  let a = 0, b = 0, c = 0;
  for (let y = cy - r; y <= cy + r; y++) for (let x = cx - r; x <= cx + r; x++) {
    const i = y * IW + x, gx = G.Ix[i], gy = G.Iy[i];
    a += gx * gx; b += gx * gy; c += gy * gy;
  }
  const n = win * win;
  return [a / n, b / n, c / n];
}

// Eigenvalues (λ1 ≥ λ2) and the unit eigenvector of λ1 of [[a, b], [b, c]].
function eig2([a, b, c]) {
  const m = (a + c) / 2, d = Math.hypot((a - c) / 2, b);
  const l1 = m + d, l2 = m - d;
  const ang = 0.5 * Math.atan2(2 * b, a - c);
  return { l1, l2: Math.max(0, l2), v1: [Math.cos(ang), Math.sin(ang)] };
}
const response = (l1, l2) => l1 * l2 - K * (l1 + l2) * (l1 + l2);
const classify = (l1, l2) => { const R = response(l1, l2); return R > RT ? 'corner' : R < -RT ? 'edge' : 'flat'; };

function harrisCorners(G) {
  // box sums (7 × 7) of Ix², IxIy, Iy² via integral images
  const W1 = IW + 1;
  const integ = f => {
    const S = new Float64Array(W1 * (IH + 1));
    for (let y = 0; y < IH; y++) { let row = 0; for (let x = 0; x < IW; x++) { row += f(y * IW + x); S[(y + 1) * W1 + x + 1] = S[y * W1 + x + 1] + row; } }
    return S;
  };
  const Sa = integ(i => G.Ix[i] * G.Ix[i]), Sb = integ(i => G.Ix[i] * G.Iy[i]), Sc = integ(i => G.Iy[i] * G.Iy[i]);
  const rr = 3, n = 49, box = (S, x, y) => S[(y + rr + 1) * W1 + x + rr + 1] - S[(y - rr) * W1 + x + rr + 1] - S[(y + rr + 1) * W1 + x - rr] + S[(y - rr) * W1 + x - rr];
  const R = new Float32Array(IW * IH);
  let mx = 0;
  for (let y = rr + 1; y < IH - rr - 1; y++) for (let x = rr + 1; x < IW - rr - 1; x++) {
    const a = box(Sa, x, y) / n, b = box(Sb, x, y) / n, c = box(Sc, x, y) / n;
    const v = a * c - b * b - K * (a + c) * (a + c);
    R[y * IW + x] = v; if (v > mx) mx = v;
  }
  const thr = 0.01 * mx, nms = 5, out = [];
  for (let y = 6; y < IH - 6; y++) for (let x = 6; x < IW - 6; x++) {
    const v = R[y * IW + x];
    if (v <= thr) continue;
    let best = true;
    for (let dy = -nms; dy <= nms && best; dy++) for (let dx = -nms; dx <= nms; dx++) {
      const w = R[(y + dy) * IW + x + dx];
      if (w > v || (w === v && (dy < 0 || (dy === 0 && dx < 0)))) { best = false; break; }
    }
    if (best) out.push([x, y, v]);
  }
  return out;
}

// heatmap colours (computed pixels, so the light theme is chosen here)
const STOPS = LIGHT
  ? [[255, 255, 255], [253, 236, 200], [245, 190, 110], [214, 120, 40], [140, 55, 25]]
  : [[24, 26, 33], [58, 52, 120], [178, 80, 120], [242, 177, 52], [255, 240, 200]];
function cmap(t) {
  t = Math.max(0, Math.min(1, t)) * (STOPS.length - 1);
  const i = Math.min(STOPS.length - 2, Math.floor(t)), f = t - i;
  return STOPS[i].map((v, k) => v + (STOPS[i + 1][k] - v) * f);
}

export function mount(el, cfg) {
  const W = cfg.width || 560, H = Math.round(W * IH / IW), S = W / IW;
  const win = Math.max(3, (cfg.win ?? 15) | 1);
  const rWin = (win - 1) / 2, margin = rWin + SH + 1;
  const I = makeImage(), G = gradients(I);
  const state = { p: null, corners: isPrint() ? true : !!cfg.corners, preset: null, cornerList: null };

  // image canvas (2× internal resolution)
  const imgC = document.createElement('canvas'); imgC.width = IW; imgC.height = IH;
  const id = new ImageData(IW, IH);
  for (let i = 0; i < IW * IH; i++) { const v = I[i]; id.data[4 * i] = id.data[4 * i + 1] = id.data[4 * i + 2] = v; id.data[4 * i + 3] = 255; }
  imgC.getContext('2d').putImageData(id, 0, 0);
  const canvas = h('canvas', { width: IW * 2, height: IH * 2, style: `width:${W}px; height:${H}px; cursor:crosshair` });
  const g = canvas.getContext('2d');

  const PS = 176;                                    // side panels (CSS px)
  const mk = () => h('canvas', { width: PS * 2, height: PS * 2, style: `width:${PS}px; height:${PS}px` });
  const cE = mk(), cM = mk(), cL = mk();
  const readout = h('div', { style: 'line-height:1.45; width:176px; overflow:hidden' });
  const cap = t => h('div', { class: 'dim', style: 'margin:0 0 0.15em; font-size:0.92em' }, t);

  function setPoint(p, name = null) {
    state.p = [Math.max(margin, Math.min(IW - 1 - margin, Math.round(p[0]))), Math.max(margin, Math.min(IH - 1 - margin, Math.round(p[1])))];
    state.preset = name;
    presetBtns.forEach(b => b.classList.toggle('active', b.dataset.name === name));
    draw();
  }

  function drawImage() {
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.imageSmoothingEnabled = false;
    g.drawImage(imgC, 0, 0, canvas.width, canvas.height);
    g.setTransform(2, 0, 0, 2, 0, 0);             // image pixel coordinates (the canvas is 2 × IW)
    if (state.corners) {
      if (!state.cornerList) state.cornerList = harrisCorners(G);
      g.lineWidth = 1.6 / S;
      for (const [x, y] of state.cornerList) {
        g.strokeStyle = C.red; g.beginPath(); g.arc(x + 0.5, y + 0.5, 3.2, 0, 2 * Math.PI); g.stroke();
      }
    }
    const [x, y] = state.p;
    g.strokeStyle = C.accent; g.lineWidth = 2.4 / S;
    g.strokeRect(x - rWin, y - rWin, win, win);
    g.setLineDash([3 / S, 3 / S]); g.lineWidth = 1.2 / S;
    g.strokeRect(x - rWin - SH, y - rWin - SH, win + 2 * SH, win + 2 * SH);
    g.setLineDash([]);
    g.fillStyle = C.accent; g.beginPath(); g.arc(x + 0.5, y + 0.5, 1.4, 0, 2 * Math.PI); g.fill();
  }

  // SSD surface E(Δu, Δv), per pixel of the window
  function surface() {
    const [cx, cy] = state.p, n = 2 * SH + 1, E = new Float32Array(n * n);
    for (let dv = -SH; dv <= SH; dv++) for (let du = -SH; du <= SH; du++) {
      let s = 0;
      for (let y = cy - rWin; y <= cy + rWin; y++) for (let x = cx - rWin; x <= cx + rWin; x++) {
        const d = I[(y + dv) * IW + x + du] - I[y * IW + x]; s += d * d;
      }
      E[(dv + SH) * n + du + SH] = s / (win * win);
    }
    return E;
  }

  // shift-plane panels share the axes: Δu right, Δv down, ±SH px
  const n = 2 * SH + 1, cellPx = (PS * 2) / n;
  const toPanel = (du, dv) => [(du + SH + 0.5) * cellPx, (dv + SH + 0.5) * cellPx];
  function shiftAxes(gg) {
    gg.strokeStyle = 'rgba(154,161,174,0.55)'; gg.lineWidth = 1.5;
    const [c0] = toPanel(0, 0);
    gg.beginPath(); gg.moveTo(c0, 0); gg.lineTo(c0, PS * 2); gg.moveTo(0, c0); gg.lineTo(PS * 2, c0); gg.stroke();
    gg.fillStyle = C.dim; gg.font = 'italic 28px "Times New Roman", serif';
    gg.fillText('Δu', PS * 2 - 40, c0 - 8); gg.fillText('Δv', c0 + 8, PS * 2 - 10);
  }

  function drawSurface(E) {
    const gg = cE.getContext('2d');
    const im = new ImageData(n, n);
    for (let i = 0; i < n * n; i++) {
      const [r, gr, b] = cmap(Math.sqrt(E[i] / EMAX));
      im.data[4 * i] = r; im.data[4 * i + 1] = gr; im.data[4 * i + 2] = b; im.data[4 * i + 3] = 255;
    }
    const tmp = document.createElement('canvas'); tmp.width = n; tmp.height = n; tmp.getContext('2d').putImageData(im, 0, 0);
    gg.imageSmoothingEnabled = false;
    gg.drawImage(tmp, 0, 0, PS * 2, PS * 2);
    shiftAxes(gg);
  }

  function drawEllipse(T) {
    const gg = cM.getContext('2d');
    gg.fillStyle = C.bg2; gg.fillRect(0, 0, PS * 2, PS * 2);
    shiftAxes(gg);
    const { l1, l2, v1 } = eig2(T), level = 600;    // Δᵀ M Δ = level
    const r1 = Math.sqrt(level / Math.max(l1, 1e-9)), r2 = Math.sqrt(level / Math.max(l2, 1e-9));
    const [c0] = toPanel(0, 0), sc = cellPx, big = 3 * SH;
    const ang = Math.atan2(v1[1], v1[0]);
    gg.save();
    gg.beginPath(); gg.rect(0, 0, PS * 2, PS * 2); gg.clip();
    gg.translate(c0, c0); gg.rotate(ang);
    gg.fillStyle = 'rgba(242,177,52,0.18)'; gg.strokeStyle = C.accent; gg.lineWidth = 4;
    gg.beginPath(); gg.ellipse(0, 0, Math.min(r1, big) * sc, Math.min(r2, big) * sc, 0, 0, 2 * Math.PI); gg.fill(); gg.stroke();
    // semi-axes along the eigenvectors: short = large λ (steep), long = small λ (shallow)
    gg.lineWidth = 3.5;
    gg.strokeStyle = C.blue; gg.beginPath(); gg.moveTo(0, 0); gg.lineTo(Math.min(r1, big) * sc, 0); gg.stroke();
    gg.strokeStyle = C.green; gg.beginPath(); gg.moveTo(0, 0); gg.lineTo(0, Math.min(r2, big) * sc); gg.stroke();
    gg.restore();
    if (r2 > SH * 1.2) {
      gg.fillStyle = C.dim; gg.font = '24px Inter, Arial';
      gg.fillText(r1 > SH * 1.2 ? 'beyond the panel' : 'long axis: beyond', 10, 28);
    }
  }

  function drawDiagram(l1, l2) {
    const gg = cL.getContext('2d'), N = PS * 2, pad = 34, span = N - pad - 8;
    const sq = v => Math.sqrt(Math.max(0, v) / LMAX);              // square-root axes
    const fromX = px => LMAX * Math.pow((px - pad) / span, 2);
    gg.fillStyle = C.bg2; gg.fillRect(0, 0, N, N);
    const col = { flat: LIGHT ? [236, 238, 241] : [40, 44, 53], edge: LIGHT ? [205, 226, 248] : [32, 60, 92], corner: LIGHT ? [250, 228, 186] : [92, 70, 30] };
    const im = gg.createImageData(span, span);
    for (let j = 0; j < span; j++) for (let i = 0; i < span; i++) {
      const a = fromX(pad + i + 0.5), b = fromX(pad + (span - 1 - j) + 0.5);
      const [r, gr, bb] = col[classify(Math.max(a, b), Math.min(a, b))];
      const k = 4 * (j * span + i); im.data[k] = r; im.data[k + 1] = gr; im.data[k + 2] = bb; im.data[k + 3] = 255;
    }
    gg.putImageData(im, pad, 8);
    gg.strokeStyle = C.dim; gg.lineWidth = 2;
    gg.beginPath(); gg.moveTo(pad, 8); gg.lineTo(pad, 8 + span); gg.lineTo(pad + span, 8 + span); gg.stroke();
    gg.font = '26px Inter, Arial'; gg.fillStyle = C.fg;
    gg.fillText('corner', pad + span * 0.52, 8 + span * 0.3);
    gg.fillText('edge', pad + span * 0.66, 8 + span * 0.93);
    gg.save(); gg.translate(pad + span * 0.1, 8 + span * 0.42); gg.rotate(-Math.PI / 2); gg.fillText('edge', 0, 0); gg.restore();
    gg.fillText('flat', pad + 6, 8 + span - 8);
    gg.fillStyle = C.fg; gg.font = 'italic 30px "Times New Roman", serif';
    gg.fillText('λ₁', pad + span - 30, 8 + span + 27);
    gg.fillText('λ₂', 2, 34);
    // the current window (both orderings are symmetric; plot λ1 on x)
    const x = pad + Math.min(1, sq(l1)) * span, y = 8 + span - Math.min(1, sq(l2)) * span;
    gg.fillStyle = C.accent; gg.strokeStyle = C.bg; gg.lineWidth = 3;
    gg.beginPath(); gg.arc(x, y, 9, 0, 2 * Math.PI); gg.fill(); gg.stroke();
  }

  function draw() {
    drawImage();
    const E = surface(), T = tensorAt(G, state.p[0], state.p[1], win), { l1, l2 } = eig2(T);
    drawSurface(E); drawEllipse(T); drawDiagram(l1, l2);
    const R = response(l1, l2), cls = classify(l1, l2);
    const clsCol = cls === 'corner' ? C.accent : cls === 'edge' ? C.blue : C.dim;
    readout.innerHTML =
      `<div class="readout" style="font-size:0.85em">at (${state.p[0]}, ${state.p[1]})</div>` +
      `<div><span style="color:${C.blue}">λ₁</span> = <span class="readout">${fmt(l1, 0)}</span></div>` +
      `<div><span style="color:${C.green}">λ₂</span> = <span class="readout">${fmt(l2, 0)}</span></div>` +
      `<div>R = <span class="readout">${fmt(R, 0)}</span></div>` +
      `<div style="margin-top:0.2em">→ <b style="color:${clsCol}">${cls}</b></div>`;
  }

  canvas.addEventListener('click', e => {
    const r = canvas.getBoundingClientRect();
    setPoint([(e.clientX - r.left) / r.width * IW, (e.clientY - r.top) / r.height * IH]);
  });
  const presetBtns = ['flat', 'edge', 'corner', 'texture'].map(name => {
    const b = h('button', { onclick: () => setPoint(PRESETS[name], name) }, name);
    b.dataset.name = name; return b;
  });
  const cornerBtn = h('button', { onclick: () => { state.corners = !state.corners; cornerBtn.classList.toggle('active', state.corners); draw(); } }, 'Harris corners');
  cornerBtn.classList.toggle('active', state.corners);

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.9em; align-items:flex-start' },
    h('div', { style: `width:${W}px; flex:none` }, canvas,
      h('div', { class: 'wctl interactive-only' }, ...presetBtns, h('span', { style: 'width:0.6em' }), cornerBtn,
        h('span', { class: 'dim', style: 'margin-left:0.4em' }, 'or click the image'))),
    h('div', { style: `display:grid; grid-template-columns:${PS}px ${PS}px; gap:0.5em 0.7em; flex:none` },
      h('div', {}, cap('shift error E(Δu, Δv)'), cE),
      h('div', {}, cap('quadratic ≈ ΔᵀMΔ'), cM),
      h('div', {}, cap('eigenvalues of M'), cL),
      h('div', { style: 'padding-top:1.3em' }, readout))));

  const p0 = cfg.preset ?? 'corner';
  if (Array.isArray(p0)) setPoint(p0); else setPoint(PRESETS[p0] || PRESETS.corner, PRESETS[p0] ? p0 : 'corner');
}
