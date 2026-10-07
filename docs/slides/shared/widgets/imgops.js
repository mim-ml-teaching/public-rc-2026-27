// Gallery of photometric vs geometric operations applied to the test image.
// config: { group: "photometric" | "geometric", tile: [w, h] }
import { h, testImage, inv3 } from './util.js';

// Photometric ops act on the pixel values in code, not via the canvas `filter` property:
// Safari (WebKit) ignores `filter` and would show every tile unchanged. Formulas follow the
// CSS Filter Effects spec, so the result matches what the filters did in Chrome.
const PHOTOMETRIC = [
  ['original', null],
  ['brightness +40%', d => perPixel(d, c => 1.4 * c)],
  ['contrast −50%', d => perPixel(d, c => 0.5 * c + 0.25 * 255)],
  ['greyscale', d => colourMatrix(d, [0.2126, 0.7152, 0.0722, 0.2126, 0.7152, 0.0722, 0.2126, 0.7152, 0.0722])],
  ['hue shift', d => colourMatrix(d, hueRotate(120))],
  ['blur', (d, W, H) => gaussianBlur(d, W, H, 2.5)],
  ['noise', d => {
    let seed = 7;
    const rnd = () => ((seed = (seed * 16807) % 2147483647) / 2147483647 - 0.5);
    for (let i = 0; i < d.length; i += 4) {
      const n = rnd() * 110;
      d[i] += n; d[i + 1] += n; d[i + 2] += n;
    }
  }],
];

const GEOMETRIC = [
  ['original', { m: [1, 0, 0, 1, 0, 0] }],
  ['crop + resize', { crop: [0.45, 0.25, 0.5, 0.5] }],
  ['horizontal flip', { m: [-1, 0, 0, 1, 1, 0] }],
  ['rotate 20°', { rot: 20 }],
  ['scale ×0.6', { m: [0.6, 0, 0, 0.6, 0.2, 0.2] }],
  ['shear', { m: [1, 0, 0.4, 1, -0.2, 0] }],
  ['perspective', { H: [0.8, 0.12, 0.08, 0.0, 0.9, 0.05, -0.25, 0.15, 1] }],
];

export function mount(el, cfg) {
  const [TW, TH] = cfg.tile || [150, 112];
  const ops = cfg.group === 'geometric' ? GEOMETRIC : PHOTOMETRIC;
  const names = cfg.ops;
  const src = testImage(320, 240);
  el.classList.add('widget');
  const row = h('div', { style: 'display:flex; gap:0.5em; flex-wrap:wrap' });
  for (const [label, op] of ops) {
    if (names && !names.includes(label)) continue;
    const c = h('canvas', { width: TW, height: TH });
    const g = c.getContext('2d');
    g.fillStyle = '#000'; g.fillRect(0, 0, TW, TH);
    if (cfg.group === 'geometric') drawGeometric(g, src, op, TW, TH);
    else drawPhotometric(g, src, op, TW, TH);
    row.append(h('div', { style: 'text-align:center' }, c, h('div', { class: 'dim', style: 'margin-top:0.15em' }, label)));
  }
  el.append(row);
}

function drawPhotometric(g, src, op, W, H) {
  g.drawImage(src, 0, 0, W, H);
  if (!op) return;
  const img = g.getImageData(0, 0, W, H);
  op(img.data, W, H); // Uint8ClampedArray: results are rounded and clamped to 0..255
  g.putImageData(img, 0, 0);
}

function perPixel(d, f) {
  for (let i = 0; i < d.length; i += 4) { d[i] = f(d[i]); d[i + 1] = f(d[i + 1]); d[i + 2] = f(d[i + 2]); }
}

function colourMatrix(d, m) {
  for (let i = 0; i < d.length; i += 4) {
    const r = d[i], gr = d[i + 1], b = d[i + 2];
    d[i] = m[0] * r + m[1] * gr + m[2] * b;
    d[i + 1] = m[3] * r + m[4] * gr + m[5] * b;
    d[i + 2] = m[6] * r + m[7] * gr + m[8] * b;
  }
}

function hueRotate(deg) {
  const c = Math.cos(deg * Math.PI / 180), s = Math.sin(deg * Math.PI / 180);
  return [
    0.213 + 0.787 * c - 0.213 * s, 0.715 - 0.715 * c - 0.715 * s, 0.072 - 0.072 * c + 0.928 * s,
    0.213 - 0.213 * c + 0.143 * s, 0.715 + 0.285 * c + 0.140 * s, 0.072 - 0.072 * c - 0.283 * s,
    0.213 - 0.213 * c - 0.787 * s, 0.715 - 0.715 * c + 0.715 * s, 0.072 + 0.928 * c + 0.072 * s,
  ];
}

// Separable Gaussian blur (standard deviation sigma, in pixels), edges clamped.
function gaussianBlur(d, W, H, sigma) {
  const r = Math.ceil(3 * sigma), k = [];
  for (let i = -r; i <= r; i++) k.push(Math.exp(-i * i / (2 * sigma * sigma)));
  const sum = k.reduce((a, b) => a + b);
  const pass = (from, to, dx, dy) => {
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) for (let ch = 0; ch < 3; ch++) {
      let acc = 0;
      for (let i = -r; i <= r; i++) {
        const xx = Math.min(W - 1, Math.max(0, x + i * dx)), yy = Math.min(H - 1, Math.max(0, y + i * dy));
        acc += k[i + r] * from[4 * (yy * W + xx) + ch];
      }
      to[4 * (y * W + x) + ch] = acc / sum;
    }
  };
  const tmp = new Float32Array(d.length);
  pass(d, tmp, 1, 0);
  pass(tmp, d, 0, 1);
}

function drawGeometric(g, src, op, W, H) {
  if (op.crop) {
    const [x, y, w, hh] = op.crop;
    g.drawImage(src, x * src.width, y * src.height, w * src.width, hh * src.height, 0, 0, W, H);
  } else if (op.m) {
    const [a, b, c, d, e, f] = op.m; // in units of the tile size
    g.setTransform(a, b, c, d, e * W, f * H);
    g.drawImage(src, 0, 0, W, H);
    g.setTransform(1, 0, 0, 1, 0, 0);
  } else if (op.rot) {
    g.translate(W / 2, H / 2); g.rotate(op.rot * Math.PI / 180); g.translate(-W / 2, -H / 2);
    g.drawImage(src, 0, 0, W, H);
    g.setTransform(1, 0, 0, 1, 0, 0);
  } else if (op.H) {
    // backward warp with a homography in normalised [0,1]^2 tile coordinates
    const Hm = op.H, sg = src.getContext('2d').getImageData(0, 0, src.width, src.height).data;
    const out = g.createImageData(W, H);
    const inv = inv3(Hm);
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      const X = x / W, Y = y / H;
      const w = inv[6] * X + inv[7] * Y + inv[8];
      const u = (inv[0] * X + inv[1] * Y + inv[2]) / w, v = (inv[3] * X + inv[4] * Y + inv[5]) / w;
      const di = 4 * (y * W + x);
      if (u < 0 || v < 0 || u >= 1 || v >= 1) { out.data[di + 3] = 255; continue; }
      const si = 4 * (Math.floor(v * src.height) * src.width + Math.floor(u * src.width));
      out.data[di] = sg[si]; out.data[di + 1] = sg[si + 1]; out.data[di + 2] = sg[si + 2]; out.data[di + 3] = 255;
    }
    g.putImageData(out, 0, 0);
  }
}
