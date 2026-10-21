// When are two images related by a homography? Rotation vs translation of the camera.
// Image 1: the scene of scene3d.js from the reference camera (origin, looking along +z). Image 2: a second camera,
// either panned about the vertical axis (pure rotation) or shifted sideways (translation). Image 2 is warped onto
// image 1 by a homography and shown over image 1 (50/50) or as the absolute difference.
//
// Convention: x₁ = camera-1 (= world) coordinates, x₂ = R x₁ + t the same point in camera 2, t = −R c₂.
// The warp is backward, so we use H mapping image-1 pixels to image-2 pixels (H⁻¹ maps image 2 onto image 1):
//   pure rotation (c₂ = 0):            H = K R K⁻¹                exact for every pixel, whatever the depth;
//   translation, plane nᵀx₁ = d:       H = K (R + t nᵀ / d) K⁻¹   exact only for points on that plane.
// Planes: the back wall z = 20 (n = (0, 0, 1), d = 20) and the ground y = 1 (n = (0, 1, 0), d = 1, y points down).
// Everything off the chosen plane appears twice in the overlay (parallax).
//
// config: {
//   motion: "rotate"        "rotate" | "wall" (translate, H of the back wall) | "ground" (translate, H of the ground)
//   amount: 12 / 0.4        pan angle in degrees (rotate) or sideways shift in metres (wall, ground)
//   view: "overlay"         "overlay" (image 1 + warped image 2, 50/50) | "diff" (|image 1 − warped image 2| ×3,
//                           both lightly blurred so that resampling noise does not show; also the mean |diff| readout)
//   motions: ["rotate", "wall", "ground"]   motion buttons to show
//   width: 300              displayed width of each of the three panels (images are 320×240)
//   f: 300                  focal length in pixels
// }
// Print: the configured state, controls hidden.
import { C, h, fmt, isPrint } from './util.js';
import { LIGHT } from '../theme.js';
import { renderView, WALL_Z, GROUND_Y } from './scene3d.js';

const IW = 320, IH = 240;
const deg = Math.PI / 180;
const MOTIONS = {
  rotate: { label: 'rotate', unit: '°', min: 0, max: 25, step: 1, def: 12 },
  wall: { label: 'translate, wall H', unit: ' m', min: 0, max: 1, step: 0.05, def: 0.4 },
  ground: { label: 'translate, ground H', unit: ' m', min: 0, max: 1, step: 0.05, def: 0.4 },
};

// 2×2 supersampled render (anti-aliased, so that resampling by the warp does not light up texture edges)
function renderAA(R, c, f) {
  const big = renderView({ R, c, f: 2 * f, W: 2 * IW, H: 2 * IH }).rgba, out = new Uint8ClampedArray(IW * IH * 4);
  for (let v = 0; v < IH; v++) for (let u = 0; u < IW; u++) for (let k = 0; k < 4; k++) {
    const p = (dy, dx) => big[4 * ((2 * v + dy) * 2 * IW + 2 * u + dx) + k];
    out[4 * (v * IW + u) + k] = (p(0, 0) + p(0, 1) + p(1, 0) + p(1, 1)) / 4;
  }
  return out;
}

const mul = (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, a, k) => s + a * B[k][j], 0)));

// camera 2: world → camera rotation R and centre c
function pose(motion, amount) {
  if (motion === 'rotate') {
    const a = amount * deg, ca = Math.cos(a), sa = Math.sin(a);
    // camera-to-world = rotation about y turning +z towards +x (pan right); R is its transpose
    return { R: [[ca, 0, -sa], [0, 1, 0], [sa, 0, ca]], c: [0, 0, 0] };
  }
  return { R: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], c: [amount, 0, 0] };
}

// H: image-1 pixel → image-2 pixel
function homography(motion, R, c, f) {
  const K = [[f, 0, IW / 2], [0, f, IH / 2], [0, 0, 1]];
  const Ki = [[1 / f, 0, -IW / 2 / f], [0, 1 / f, -IH / 2 / f], [0, 0, 1]];
  let M = R.map(r => r.slice());
  if (motion !== 'rotate') {
    const t = R.map(r => -(r[0] * c[0] + r[1] * c[1] + r[2] * c[2]));
    const [n, d] = motion === 'ground' ? [[0, 1, 0], GROUND_Y] : [[0, 0, 1], WALL_Z];
    M = M.map((r, i) => r.map((v, j) => v + t[i] * n[j] / d));
  }
  return mul(mul(K, M), Ki);
}

// backward warp with bilinear sampling; returns RGBA + validity mask
function warp(img, H) {
  const out = new Float32Array(IW * IH * 3), ok = new Uint8Array(IW * IH);
  for (let v = 0; v < IH; v++) for (let u = 0; u < IW; u++) {
    const x = u + 0.5, y = v + 0.5;
    const w = H[2][0] * x + H[2][1] * y + H[2][2];
    const X = (H[0][0] * x + H[0][1] * y + H[0][2]) / w - 0.5, Y = (H[1][0] * x + H[1][1] * y + H[1][2]) / w - 0.5;
    if (w <= 0 || X < 0 || Y < 0 || X > IW - 1 || Y > IH - 1) continue;
    const x0 = Math.min(IW - 2, Math.floor(X)), y0 = Math.min(IH - 2, Math.floor(Y)), fx = X - x0, fy = Y - y0;
    const i = v * IW + u; ok[i] = 1;
    for (let k = 0; k < 3; k++) {
      const p = (yy, xx) => img[4 * (yy * IW + xx) + k];
      out[3 * i + k] = (1 - fy) * ((1 - fx) * p(y0, x0) + fx * p(y0, x0 + 1)) + fy * ((1 - fx) * p(y0 + 1, x0) + fx * p(y0 + 1, x0 + 1));
    }
  }
  return { out, ok };
}

export function mount(el, cfg) {
  const f = cfg.f ?? 300, TW = cfg.width || 300, TH = TW * IH / IW;
  const motions = (cfg.motions || Object.keys(MOTIONS)).filter(m => MOTIONS[m]);
  const state = { motion: cfg.motion && MOTIONS[cfg.motion] ? cfg.motion : motions[0], view: cfg.view === 'diff' ? 'diff' : 'overlay' };
  const amounts = Object.fromEntries(Object.entries(MOTIONS).map(([k, m]) => [k, m.def]));
  if (cfg.amount !== undefined) amounts[state.motion] = cfg.amount;

  const img1 = renderAA([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0], f);
  const cache = new Map();
  const view2 = (motion, amount) => {
    const P = pose(motion, amount), key = `${P.R.flat().join()}|${P.c.join()}`;
    if (!cache.has(key)) cache.set(key, renderAA(P.R, P.c, f));
    return { rgba: cache.get(key), ...P };
  };

  const src = rgba => { const c = document.createElement('canvas'); c.width = IW; c.height = IH; c.getContext('2d').putImageData(new ImageData(rgba, IW, IH), 0, 0); return c; };
  const s1 = src(img1);
  const tile = () => h('canvas', { width: IW * 2, height: IH * 2, style: `width:${TW}px; height:${TH}px` });
  const c1 = tile(), c2 = tile(), c3 = tile();
  const cap = (t, color) => h('div', { style: `margin:0 0 0.15em; color:${color}` }, t);
  const cap3 = cap('', 'var(--fg)');
  const readout = h('div', { style: 'margin-top:0.35em; min-height:1.4em' });

  const paint = (canvas, source) => { const g = canvas.getContext('2d'); g.imageSmoothingEnabled = false; g.drawImage(source, 0, 0, IW * 2, IH * 2); };

  function draw() {
    const amount = amounts[state.motion];
    const v2 = view2(state.motion, amount);
    paint(c1, s1); paint(c2, src(v2.rgba));
    const H = homography(state.motion, v2.R, v2.c, f);
    const { out, ok } = warp(v2.rgba, H);
    const im = new ImageData(IW, IH), d = im.data;
    // difference of 3×3-box-blurred images: removes the faint residue of bilinear resampling at texture edges,
    // keeps the parallax (displacements of several pixels)
    const blur = (get, valid) => {
      const o = new Float32Array(IW * IH * 3);
      for (let v = 0; v < IH; v++) for (let u = 0; u < IW; u++) {
        const i = v * IW + u; if (!valid(i)) continue;
        for (let k = 0; k < 3; k++) {
          let s = 0, m = 0;
          for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
            const x = u + dx, y = v + dy, j = y * IW + x;
            if (x < 0 || y < 0 || x >= IW || y >= IH || !valid(j)) continue;
            s += get(j, k); m++;
          }
          o[3 * i + k] = s / m;
        }
      }
      return o;
    };
    const b1 = blur((j, k) => img1[4 * j + k], j => ok[j]), b2 = blur((j, k) => out[3 * j + k], j => ok[j]);
    let sum = 0, n = 0;
    for (let i = 0; i < IW * IH; i++) {
      d[4 * i + 3] = 255;
      if (!ok[i]) { // outside image 2: image 1 only, dimmed (overlay) / neutral (difference)
        for (let k = 0; k < 3; k++) d[4 * i + k] = state.view === 'overlay' ? img1[4 * i + k] * 0.3 + (LIGHT ? 178 : 0) : (LIGHT ? 225 : 40);
        continue;
      }
      let e = 0;
      for (let k = 0; k < 3; k++) e += Math.abs(b1[3 * i + k] - b2[3 * i + k]);
      e /= 3; sum += e; n++;
      if (state.view === 'overlay') for (let k = 0; k < 3; k++) d[4 * i + k] = 0.5 * img1[4 * i + k] + 0.5 * out[3 * i + k];
      else { const g = Math.min(255, 3 * e); for (let k = 0; k < 3; k++) d[4 * i + k] = LIGHT ? 255 - g : g; }
    }
    paint(c3, src(d));
    cap3.textContent = state.view === 'overlay' ? 'image 1 + warped image 2 (50/50)' : '|image 1 − warped image 2| (×3)';
    const m = MOTIONS[state.motion];
    const what = state.motion === 'rotate'
      ? `pure rotation by ${fmt(amount, 0)}°: H = K R K⁻¹ holds for every pixel, whatever its depth`
      : `translation by ${fmt(amount, 2)} m: the ${state.motion === 'wall' ? 'back-wall' : 'ground'} plane's H aligns only that plane, everything else appears twice`;
    readout.innerHTML = `${amount === 0 ? 'no motion: identical images' : what}` +
      (isPrint() ? '' : ` <span class="readout" style="margin-left:0.6em">mean |diff| ${fmt(n ? sum / n : 0, 1)}</span>`);
    motionBtns.forEach((b, i) => b.classList.toggle('active', motions[i] === state.motion));
    viewBtns.forEach(b => b.classList.toggle('active', b.dataset.v === state.view));
    slider.min = m.min; slider.max = m.max; slider.step = m.step; slider.value = amount;
    sliderVal.textContent = `${fmt(amount, state.motion === 'rotate' ? 0 : 2)}${m.unit}`;
  }

  const motionBtns = motions.map(k => h('button', { onclick: () => { state.motion = k; draw(); } }, MOTIONS[k].label));
  const viewBtns = [['overlay', 'overlay'], ['diff', 'difference']].map(([v, t]) => h('button', { 'data-v': v, onclick: () => { state.view = v; draw(); } }, t));
  const slider = h('input', { type: 'range', style: 'width:9em' });
  const sliderVal = h('span', { class: 'readout', style: 'display:inline-block; width:4.2em' });
  slider.addEventListener('input', () => { amounts[state.motion] = +slider.value; draw(); });

  el.classList.add('widget');
  el.append(
    h('div', { style: 'display:flex; gap:0.6em; align-items:flex-start' },
      h('div', {}, cap('image 1 (reference camera)', C.accent), c1),
      h('div', {}, cap('image 2 (moved camera)', C.blue), c2),
      h('div', {}, cap3, c3)),
    h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' },
      ...(motions.length > 1 ? motionBtns : []),
      h('span', { style: 'margin-left:0.6em' }, h('label', {}, 'amount ')), slider, sliderVal,
      h('span', { style: 'margin-left:0.6em' }), ...viewBtns),
    readout);
  draw();
}
