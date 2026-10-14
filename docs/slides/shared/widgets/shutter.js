// Global vs rolling shutter, simulated. Three panels: the scene itself (slowed down) with the current read-out row,
// the global-shutter image (every row at the same moment) and the rolling-shutter image (row v taken at time
// t0 + v / H · T_readout, built row by row). Scenes: a spinning propeller (blades bend) and a moving cube (it
// skews). The background grid is static, so it is not distorted.
// Time unit = the read-out time of one frame; the slider sets the motion during one read-out.
// config: { scene: "propeller" | "cube", width (of each panel), duration (ms of one read-out on screen),
//           p: a fixed read-out progress 0…1 (static figure, no animation; print uses p = 1) }
import { C, h, isPrint } from './util.js';
import { LIGHT } from '../theme.js';

const W = 240, H = 180;
const hex = s => [1, 3, 5].map(i => parseInt(s.slice(i, i + 2), 16));
const COL = LIGHT
  ? { bg: hex('#f3f4f6'), grid: hex('#c8ccd3'), blade: hex('#b27a00'), tip: hex('#d23c3c'), hub: hex('#1b1d22'), face: hex('#1c6ed0'), stripe: hex('#e7e9ee'), top: hex('#3f7fb5') }
  : { bg: hex('#1e2128'), grid: hex('#3a3f4b'), blade: hex('#f2b134'), tip: hex('#ff6b6b'), hub: hex('#e7e9ee'), face: hex('#5ab0ff'), stripe: hex('#1e2128'), top: hex('#a9cbe8') };

// colour of pixel (x, y) at time t (in read-out times); `amount` = motion per read-out
const SCENES = {
  propeller: (x, y, t, amount) => {
    const dx = x - W / 2, dy = y - H / 2, r = Math.hypot(dx, dy);
    if (r < 8) return COL.hub;
    if (r < 82) {
      const a = Math.atan2(dy, dx) - 2 * Math.PI * amount * t;      // amount = turns per read-out
      const k = 2 * Math.PI / 3, m = ((a % k) + k) % k;              // three blades
      const halfWidth = 0.09 + 6 / Math.max(r, 1);                   // angular half-width, a bit wider at the hub
      if (m < halfWidth || m > k - halfWidth) return r > 70 ? COL.tip : COL.blade;
    }
    return null;
  },
  cube: (x, y, t, amount) => {
    const s = 64, x0 = 40 + amount * 150 * t, y0 = 70, d = 14;     // amount = fraction of 150 px per read-out
    // front face with vertical stripes, top face drawn as a parallelogram (simple 3D look)
    if (x >= x0 && x < x0 + s && y >= y0 && y < y0 + s) return Math.floor((x - x0) / 8) % 2 ? COL.stripe : COL.face;
    if (y >= y0 - d && y < y0) { const sh = y0 - y; if (x >= x0 + sh && x < x0 + s + sh) return COL.top; }
    if (x >= x0 + s && x < x0 + s + d) { const sh = x - x0 - s; if (y >= y0 - sh && y < y0 + s - sh) return COL.top.map(v => v * 0.8); }
    return null;
  },
};

function background(x, y) { return (x % 20 === 0 || y % 20 === 0) ? COL.grid : COL.bg; }

export function mount(el, cfg) {
  const PW = cfg.width || 280, PH = PW * H / W;
  const state = { scene: cfg.scene || 'propeller', amount: { propeller: 0.25, cube: 0.6 }, t0: 0, p: 0, playing: !isPrint() };
  const dur = cfg.duration ?? 2500;

  const mk = () => h('canvas', { width: W, height: H, style: `width:${PW}px; height:${PH}px; image-rendering:auto; border:1px solid ${C.line}; border-radius:6px; display:block` });
  const live = mk(), glob = mk(), roll = mk();
  const ctx = [live, glob, roll].map(c => c.getContext('2d'));
  const img = [0, 1, 2].map(() => new ImageData(W, H));

  function shade(im, x, y, c) { const i = 4 * (y * W + x); im.data[i] = c[0]; im.data[i + 1] = c[1]; im.data[i + 2] = c[2]; im.data[i + 3] = 255; }
  function renderRow(im, y, t) {
    const f = SCENES[state.scene], a = state.amount[state.scene];
    for (let x = 0; x < W; x++) shade(im, x, y, f(x, y, t, a) || background(x, y));
  }
  function renderAll(im, t) { for (let y = 0; y < H; y++) renderRow(im, y, t); }

  let rowsDone = 0;
  function startFrame() {
    rowsDone = 0;
    renderAll(img[1], state.t0);                       // global shutter: all rows at t0
    const d = img[2].data; for (let i = 0; i < d.length; i += 4) { d[i] = COL.bg[0] * 0.6; d[i + 1] = COL.bg[1] * 0.6; d[i + 2] = COL.bg[2] * 0.6; d[i + 3] = 255; }
  }
  function draw() {
    const row = Math.min(H, Math.floor(state.p * H));
    for (; rowsDone < row; rowsDone++) renderRow(img[2], rowsDone, state.t0 + rowsDone / H);   // rolling: row v at t0 + v/H
    renderAll(img[0], state.t0 + state.p);
    ctx.forEach((g, k) => g.putImageData(img[k], 0, 0));
    // the current read-out row on the live scene and on the rolling image
    if (state.p < 1) for (const g of [ctx[0], ctx[2]]) {
      g.fillStyle = C.red; g.fillRect(0, row - 1, W, 2);
    }
  }

  let last = null, raf = 0;
  function tick(now) {
    raf = 0;
    if (!state.playing || !el.isConnected) return;
    const shown = el.closest('section')?.classList.contains('present') !== false;
    if (last !== null && shown) {
      state.p += (now - last) / dur;
      if (state.p >= 1.35) {                                   // short pause on the finished frame, then the next one
        // the propeller is periodic, so time runs on; the cube moves in a straight line, so each frame starts it again
        state.t0 = state.scene === 'cube' ? 0 : state.t0 + state.p;
        state.p = 0; startFrame();
      }
      draw();
    }
    last = now;
    raf = requestAnimationFrame(tick);
  }
  const play = () => { if (!raf) { last = null; raf = requestAnimationFrame(tick); } };

  // ---- controls ----
  const sceneBtns = [['propeller', 'propeller'], ['cube', 'moving cube']].map(([k, label]) =>
    h('button', { onclick: () => { state.scene = k; sceneBtns.forEach((b, i) => b.classList.toggle('active', i === (k === 'propeller' ? 0 : 1))); syncSlider(); restart(); } }, label));
  sceneBtns[state.scene === 'propeller' ? 0 : 1].classList.add('active');
  const slider = h('input', { type: 'range', min: 0, max: 1, step: 0.01 });
  const sval = h('span', { class: 'readout', style: 'display:inline-block; width:4em' });
  const slabel = h('label', {}, '');
  function syncSlider() {
    const a = state.amount[state.scene];
    slider.value = a;
    slabel.textContent = 'motion during one read-out:';
    sval.textContent = state.scene === 'propeller' ? `${Math.round(a * 360)}°` : `${Math.round(a * 150)} px`;
  }
  slider.addEventListener('input', () => { state.amount[state.scene] = parseFloat(slider.value); syncSlider(); restart(); });
  const pauseBtn = h('button', { onclick: () => { state.playing = !state.playing; pauseBtn.textContent = state.playing ? 'pause' : 'play'; if (state.playing) play(); } }, 'pause');
  function restart() { state.p = 0; state.t0 = 0; startFrame(); draw(); }
  syncSlider();

  const panel = (title, canvas) => h('div', {}, h('div', { style: 'margin:0 0 0.2em; color:var(--fg)' }, title), canvas);
  el.classList.add('widget');
  el.append(
    h('div', { style: 'display:flex; gap:0.7em' },
      panel('the scene (slowed down)', live),
      panel('global shutter: all rows at once', glob),
      panel('rolling shutter: row by row', roll)),
    h('div', { class: 'wctl interactive-only', style: 'gap:0.4em' }, ...sceneBtns, pauseBtn, slabel, slider, sval));

  startFrame();
  if (isPrint() || cfg.p !== undefined) { state.p = isPrint() ? 1 : cfg.p; state.playing = false; pauseBtn.textContent = 'play'; draw(); }
  else { draw(); play(); }
}
