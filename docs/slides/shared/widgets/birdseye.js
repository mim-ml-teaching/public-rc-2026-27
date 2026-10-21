// Bird's-eye view: one homography of the ground plane (inverse perspective mapping).
// The scene of scene3d.js (world frame: x right, y down, z forward; ground plane y = 1, i.e. 1 m below the camera).
// A robot camera at the origin, pitched down by θ. A ground point (X, 1, Z) projects as
//   ũ ≡ K R (X, 1, Z)ᵀ = K [r₁ r₃ r₂] (X, Z, 1)ᵀ = H (X, Z, 1)ᵀ     (rᵢ: columns of the world → camera rotation R)
// so ground coordinates (X, Z) → pixels is ONE 3×3 homography H. The top view is a backward warp: for every top-view
// pixel take its ground point (X, Z), map it by H into the camera image and sample (bilinear).
// The same H also follows from the 4-point DLT on the 4 marked ground points (a 1 m × 1 m square in front of the
// robot) and their pixels; the readout shows that both agree. In "drag" mode the 4 image points can be moved and the
// top view uses the DLT from the dragged points (the "4 marked floor points" recipe).
// Only the ground is mapped correctly: the box, the sphere and the striped panel stick out of the plane and are
// smeared away from the camera.
//
// config: {
//   pitch: 25          camera pitch down, degrees (slider 10 … 45)
//   width: 440         displayed width of the camera image (rendered 400×300, f = 300 px)
//   topWidth: 380      displayed width of the top view (5 m wide, 0.5 … 5 m ahead; rendered at 80 px per metre)
//   showH: true        show the matrix H (ground → image) in the side panel
//   square: true       mark the 4 ground points (square) in both views
// }
// Print: the configured state with the square and H, controls hidden.
import { C, h, fmt, isPrint } from './util.js';
import { LIGHT } from '../theme.js';
import { renderView, GROUND_Y } from './scene3d.js';
import { solve } from './linalg.js';

const IW = 400, IH = 300, F = 300;
const K = [[F, 0, IW / 2], [0, F, IH / 2], [0, 0, 1]];
const deg = Math.PI / 180;
const SQ = [[-0.5, 1.2], [0.5, 1.2], [0.5, 2.2], [-0.5, 2.2]];   // ground square (X, Z), metres
const TX0 = -2.5, TX1 = 2.5, TZ0 = 0.5, TZ1 = 5.0, PPM = 80;     // top view extent (m) and pixels per metre
const TW = (TX1 - TX0) * PPM, TH = (TZ1 - TZ0) * PPM;

const mul = (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, a, k) => s + a * B[k][j], 0)));
const ap = (H, x, y) => { const w = H[2][0] * x + H[2][1] * y + H[2][2]; return [(H[0][0] * x + H[0][1] * y + H[0][2]) / w, (H[1][0] * x + H[1][1] * y + H[1][2]) / w, w]; };

// world → camera rotation for a camera pitched down by θ (rows = camera axes in world coordinates)
function rotation(theta) {
  const c = Math.cos(theta), s = Math.sin(theta);
  return [[1, 0, 0], [0, c, -s], [0, s, c]];
}

// ground (X, Z) → pixel: K [r1 r3 r2] with the ground at y = GROUND_Y
function groundH(R) {
  const col = j => R.map(r => r[j]);
  const r1 = col(0), r2 = col(1), r3 = col(2);
  const M = [0, 1, 2].map(i => [r1[i], r3[i], GROUND_Y * r2[i]]);
  return mul(K, M);
}

// 4-point DLT (h33 = 1): src (X, Z) → dst (u, v)
function dlt4(src, dst) {
  const A = [], b = [];
  src.forEach(([x, y], i) => {
    const [u, v] = dst[i];
    A.push([x, y, 1, 0, 0, 0, -u * x, -u * y]); b.push(u);
    A.push([0, 0, 0, x, y, 1, -v * x, -v * y]); b.push(v);
  });
  const p = solve(A, b);
  if (!p) return null;
  return [[p[0], p[1], p[2]], [p[3], p[4], p[5]], [p[6], p[7], 1]];
}
const norm = H => H.map(r => r.map(v => v / H[2][2]));

// 2×2 supersampled camera image
function renderAA(R) {
  const big = renderView({ R, c: [0, 0, 0], f: 2 * F, W: 2 * IW, H: 2 * IH }).rgba, out = new Uint8ClampedArray(IW * IH * 4);
  for (let v = 0; v < IH; v++) for (let u = 0; u < IW; u++) for (let k = 0; k < 4; k++) {
    const p = (dy, dx) => big[4 * ((2 * v + dy) * 2 * IW + 2 * u + dx) + k];
    out[4 * (v * IW + u) + k] = (p(0, 0) + p(0, 1) + p(1, 0) + p(1, 1)) / 4;
  }
  return out;
}

export function mount(el, cfg) {
  const DW = cfg.width || 440, DH = DW * IH / IW, TDW = cfg.topWidth || 380, TDH = TDW * TH / TW;
  const state = { pitch: cfg.pitch ?? 25, showH: cfg.showH ?? true, square: cfg.square ?? true, drag: false, pts: null };
  const cache = new Map();
  const image = pitch => { if (!cache.has(pitch)) cache.set(pitch, renderAA(rotation(pitch * deg))); return cache.get(pitch); };

  const cam = h('canvas', { width: IW * 2, height: IH * 2, style: `width:${DW}px; height:${DH}px; cursor:default` });
  const top = h('canvas', { width: TW, height: TH, style: `width:${TDW}px; height:${TDH}px` });
  const camSrc = document.createElement('canvas'); camSrc.width = IW; camSrc.height = IH;
  const topSrc = document.createElement('canvas'); topSrc.width = TW; topSrc.height = TH;
  const matEl = h('div', { class: 'readout', style: 'white-space:pre; line-height:1.35; font-size:0.9em' });
  const infoEl = h('div', { style: 'line-height:1.35' });
  const cap = (t, color) => h('div', { style: `margin:0 0 0.15em; color:${color}` }, t);

  let geoH = null, H = null;
  function compute() {
    geoH = groundH(rotation(state.pitch * deg));
    if (!state.drag || !state.pts) state.pts = SQ.map(([x, z]) => ap(geoH, x, z).slice(0, 2));
    H = state.drag ? (dlt4(SQ, state.pts) || geoH) : geoH;
  }

  function draw() {
    compute();
    const img = image(state.pitch);
    // camera image
    camSrc.getContext('2d').putImageData(new ImageData(img, IW, IH), 0, 0);
    const g = cam.getContext('2d'); g.imageSmoothingEnabled = false;
    g.drawImage(camSrc, 0, 0, IW * 2, IH * 2);
    if (state.square || state.drag) {
      const P = state.pts.map(([u, v]) => [2 * u, 2 * v]);
      g.strokeStyle = C.accent; g.lineWidth = 4; g.beginPath();
      P.forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y))); g.closePath(); g.stroke();
      P.forEach(([x, y], i) => {
        g.fillStyle = C.accent; g.beginPath(); g.arc(x, y, state.drag ? 17 : 14, 0, 7); g.fill();
        g.fillStyle = '#15171c'; g.font = 'bold 20px Inter, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
        g.fillText(String(i + 1), x, y + 0.5);
      });
      g.textAlign = 'start'; g.textBaseline = 'alphabetic';
    }

    // top view: backward warp, top pixel (i, j) → ground (X, Z) → H → camera pixel
    const out = new ImageData(TW, TH), d = out.data;
    const bgc = LIGHT ? [243, 244, 246] : [30, 33, 40];
    for (let j = 0; j < TH; j++) {
      const Z = TZ1 - (j + 0.5) / PPM;
      for (let i = 0; i < TW; i++) {
        const X = TX0 + (i + 0.5) / PPM, o = 4 * (j * TW + i);
        const [u, v, w] = ap(H, X, Z), x = u - 0.5, y = v - 0.5;
        d[o + 3] = 255;
        if (w <= 0 || x < 0 || y < 0 || x > IW - 1 || y > IH - 1) { d[o] = bgc[0]; d[o + 1] = bgc[1]; d[o + 2] = bgc[2]; continue; }
        const x0 = Math.min(IW - 2, Math.floor(x)), y0 = Math.min(IH - 2, Math.floor(y)), fx = x - x0, fy = y - y0;
        for (let k = 0; k < 3; k++) {
          const p = (yy, xx) => img[4 * (yy * IW + xx) + k];
          d[o + k] = (1 - fy) * ((1 - fx) * p(y0, x0) + fx * p(y0, x0 + 1)) + fy * ((1 - fx) * p(y0 + 1, x0) + fx * p(y0 + 1, x0 + 1));
        }
      }
    }
    topSrc.getContext('2d').putImageData(out, 0, 0);
    const t = top.getContext('2d');
    t.drawImage(topSrc, 0, 0);
    const T = (X, Z) => [(X - TX0) * PPM, (TZ1 - Z) * PPM];
    // metric grid, every 0.5 m (labels every metre)
    t.lineWidth = 1; t.strokeStyle = 'rgba(255,255,255,0.28)';
    for (let X = Math.ceil(TX0 * 2) / 2; X <= TX1; X += 0.5) { const [x] = T(X, 0); t.beginPath(); t.moveTo(x + 0.5, 0); t.lineTo(x + 0.5, TH); t.stroke(); }
    for (let Z = Math.ceil(TZ0 * 2) / 2; Z <= TZ1; Z += 0.5) { const [, y] = T(0, Z); t.beginPath(); t.moveTo(0, y + 0.5); t.lineTo(TW, y + 0.5); t.stroke(); }
    t.font = '13px Inter, sans-serif';
    for (let Z = 1; Z <= 4; Z++) {
      const [, y] = T(0, Z), lab = `${Z} m`, w = t.measureText(lab).width;
      t.fillStyle = C.bg; t.globalAlpha = 0.75; t.fillRect(2, y - 17, w + 6, 16); t.globalAlpha = 1;
      t.fillStyle = C.fg; t.fillText(lab, 5, y - 5);
    }
    if (state.square || state.drag) {
      t.strokeStyle = C.accent; t.lineWidth = 2.5; t.beginPath();
      SQ.map(([X, Z]) => T(X, Z)).forEach(([x, y], i) => (i ? t.lineTo(x, y) : t.moveTo(x, y))); t.closePath(); t.stroke();
      SQ.map(([X, Z]) => T(X, Z)).forEach(([x, y], i) => {
        t.fillStyle = C.accent; t.beginPath(); t.arc(x, y, 8, 0, 7); t.fill();
        t.fillStyle = '#15171c'; t.font = 'bold 12px Inter, sans-serif'; t.textAlign = 'center'; t.textBaseline = 'middle';
        t.fillText(String(i + 1), x, y + 0.5);
      });
      t.textAlign = 'start'; t.textBaseline = 'alphabetic';
    }
    // the robot (camera) at Z = 0, just below the visible area
    const [rx] = T(0, 0);
    t.fillStyle = C.blue; t.beginPath(); t.moveTo(rx, TH - 16); t.lineTo(rx - 10, TH); t.lineTo(rx + 10, TH); t.closePath(); t.fill();

    // side panel
    const Hn = norm(H);
    const sig = v => (Math.abs(v) < 1e-9 ? '0' : Math.abs(v) >= 100 ? fmt(v, 0) : Math.abs(v) >= 10 ? fmt(v, 1) : fmt(v, 3));
    matEl.textContent = 'H (ground → image)\n' + Hn.map(r => r.map(v => sig(v).padStart(6)).join(' ')).join('\n');
    matEl.style.display = state.showH ? '' : 'none';
    const Hd = dlt4(SQ, SQ.map(([x, z]) => ap(geoH, x, z).slice(0, 2)));
    let agree = 0;
    for (let Z = 1; Z <= 4; Z += 0.5) for (let X = -2; X <= 2; X += 0.5) {
      const a = ap(geoH, X, Z), b = ap(Hd, X, Z);
      if (a[2] > 0) agree = Math.max(agree, Math.hypot(a[0] - b[0], a[1] - b[1]));
    }
    infoEl.innerHTML = state.drag
      ? `<b>4-point DLT</b> from the dragged image points to the 1&nbsp;m × 1&nbsp;m square. Move a point off the true corner: the ground gets distorted.`
      : `<b>From the geometry</b>: H = K [r₁ r₃ r₂], camera 1&nbsp;m above the ground, pitch ${fmt(state.pitch, 0)}°.<br>` +
        `<span class="dim">4-point DLT from the marked square gives the same H (max difference ${agree < 0.01 ? '&lt; 0.01' : fmt(agree, 2)} px).</span>`;
    pitchVal.textContent = `${fmt(state.pitch, 0)}°`;
    dragBtn.classList.toggle('active', state.drag);
    sqBtn.classList.toggle('active', state.square);
    hBtn.classList.toggle('active', state.showH);
  }

  // dragging the image points (drag mode)
  let dragging = -1;
  const toImg = e => { const r = cam.getBoundingClientRect(); return [(e.clientX - r.left) / r.width * IW, (e.clientY - r.top) / r.height * IH]; };
  cam.addEventListener('pointerdown', e => {
    if (!state.drag) return;
    const [u, v] = toImg(e);
    let best = -1, bd = 14;
    state.pts.forEach(([x, y], i) => { const dd = Math.hypot(x - u, y - v); if (dd < bd) { bd = dd; best = i; } });
    if (best >= 0) { dragging = best; cam.setPointerCapture(e.pointerId); }
  });
  cam.addEventListener('pointermove', e => {
    if (!state.drag) return;
    const [u, v] = toImg(e);
    if (dragging >= 0) { state.pts[dragging] = [Math.max(0, Math.min(IW, u)), Math.max(0, Math.min(IH, v))]; draw(); return; }
    cam.style.cursor = state.pts.some(([x, y]) => Math.hypot(x - u, y - v) < 14) ? 'grab' : 'default';
  });
  cam.addEventListener('pointerup', () => { dragging = -1; });

  const slider = h('input', { type: 'range', min: 10, max: 45, step: 1, value: state.pitch, style: 'width:8em' });
  const pitchVal = h('span', { class: 'readout', style: 'display:inline-block; width:2.6em' });
  slider.addEventListener('input', () => { state.pitch = +slider.value; state.pts = null; draw(); });
  const dragBtn = h('button', { onclick: () => { state.drag = !state.drag; state.pts = null; draw(); } }, 'drag the 4 points');
  const sqBtn = h('button', { onclick: () => { state.square = !state.square; draw(); } }, '4 ground points');
  const hBtn = h('button', { onclick: () => { state.showH = !state.showH; draw(); } }, 'show H');

  el.classList.add('widget');
  el.append(
    h('div', { style: 'display:flex; gap:0.7em; align-items:flex-start' },
      h('div', {}, cap('camera image (robot camera, 1 m above the floor)', C.blue), cam),
      h('div', {}, cap('top view: one homography, 1 grid cell = 0.5 m', C.accent), top),
      h('div', { style: 'width:13em; flex:none; display:flex; flex-direction:column; gap:0.6em' }, matEl, infoEl)),
    h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' },
      h('label', {}, 'pitch '), slider, pitchVal, h('span', { style: 'margin-left:0.6em' }), sqBtn, hBtn, dragBtn));
  if (isPrint()) { state.square = true; state.showH = true; }
  draw();
}
