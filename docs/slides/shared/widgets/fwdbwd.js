// Forward vs backward warping of the same image by T = scale + rotation about the image centre.
// Forward: push every input pixel to round(T(x)) -> holes when T enlarges.
// Backward: for every output pixel x, read f(round(T⁻¹(x))) -> every pixel filled.
// config: { scale: 1.8, angle: 20, width: 300, height: 225 }
import { LIGHT } from '../theme.js';
import { C, h } from './util.js';
import { testImage } from './util.js';

export function mount(el, cfg) {
  const W = cfg.width || 300, H = cfg.height || 225;
  const SW = 160, SH = 120; // small source, so that holes are clearly visible
  const src = testImage(SW, SH);
  const sd = src.getContext('2d').getImageData(0, 0, SW, SH).data;
  const state = { scale: cfg.scale ?? 1.8, angle: cfg.angle ?? 20 };

  const mk = () => h('canvas', { width: W, height: H, style: `width:${W * 1.35}px; height:${H * 1.35}px; image-rendering:pixelated` });
  const cf = mk(), cb = mk();
  const caption = (t, s) => h('div', { style: 'margin-top:0.2em' }, h('b', {}, t), h('div', { class: 'dim' }, s));

  function transform() {
    const a = state.angle * Math.PI / 180, s = state.scale, c = Math.cos(a) * s, sn = Math.sin(a) * s;
    // x' = A (x - cs) + co, with cs = source centre, co = output centre
    const A = [c, -sn, sn, c];
    const det = A[0] * A[3] - A[1] * A[2];
    const Ai = [A[3] / det, -A[1] / det, -A[2] / det, A[0] / det];
    return {
      fwd: (x, y) => [A[0] * (x - SW / 2) + A[1] * (y - SH / 2) + W / 2, A[2] * (x - SW / 2) + A[3] * (y - SH / 2) + H / 2],
      inv: (x, y) => [Ai[0] * (x - W / 2) + Ai[1] * (y - H / 2) + SW / 2, Ai[2] * (x - W / 2) + Ai[3] * (y - H / 2) + SH / 2],
    };
  }

  function draw() {
    const { fwd, inv } = transform();
    // forward
    const gf = cf.getContext('2d'), of = gf.createImageData(W, H);
    const empty = LIGHT ? 246 : 0; // no source pixel (background, forward-warping holes)
    for (let i = 0; i < of.data.length; i += 4) { of.data[i] = of.data[i + 1] = of.data[i + 2] = empty; of.data[i + 3] = 255; }
    for (let y = 0; y < SH; y++) for (let x = 0; x < SW; x++) {
      const [u, v] = fwd(x + 0.5, y + 0.5).map(Math.floor);
      if (u < 0 || v < 0 || u >= W || v >= H) continue;
      const si = 4 * (y * SW + x), di = 4 * (v * W + u);
      of.data[di] = sd[si]; of.data[di + 1] = sd[si + 1]; of.data[di + 2] = sd[si + 2];
    }
    gf.putImageData(of, 0, 0);
    // backward
    const gb = cb.getContext('2d'), ob = gb.createImageData(W, H);
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      const [sx, sy] = inv(x + 0.5, y + 0.5).map(Math.floor), di = 4 * (y * W + x);
      ob.data[di] = ob.data[di + 1] = ob.data[di + 2] = empty; ob.data[di + 3] = 255;
      if (sx < 0 || sy < 0 || sx >= SW || sy >= SH) continue;
      const si = 4 * (sy * SW + sx);
      ob.data[di] = sd[si]; ob.data[di + 1] = sd[si + 1]; ob.data[di + 2] = sd[si + 2];
    }
    gb.putImageData(ob, 0, 0);
  }

  const slider = (key, min, max, step, name) => {
    const s = h('input', { type: 'range', min, max, step, value: state[key] });
    s.addEventListener('input', () => { state[key] = parseFloat(s.value); draw(); });
    return h('span', { style: 'display:inline-flex; align-items:center; gap:0.3em' }, h('label', {}, name), s);
  };

  el.classList.add('widget');
  el.append(
    h('div', { style: 'display:flex; gap:1em' },
      h('div', {}, cf, caption('forward: g(T(x)) = f(x)', 'loop over input pixels, push each one — holes')),
      h('div', {}, cb, caption('backward: g(x) = f(T⁻¹(x))', 'loop over output pixels, pull — no holes'))),
    h('div', { class: 'wctl interactive-only' }, slider('scale', 0.5, 3, 0.01, 'scale'), slider('angle', -180, 180, 1, 'rotation')),
  );
  draw();
}
