// Shared scene for the panorama widgets (stitch, panoproj): a real 360° photo seen by a camera that only ROTATES.
// The photo is an equirectangular image (Poly Haven "Canary Wharf", CC0), cropped to the elevation band the views
// use (+40° … −35°). Every view pixel is rendered by following its ray K⁻¹(u, v, 1) rotated to the world and
// sampling the photo bilinearly at the ray's (azimuth, elevation): an exact pure rotation, so every pair of views
// is related exactly by H = K R_j R_iᵀ K⁻¹. Each view gets its own exposure gain and vignetting.
// Frames: world = level frame of the camera centre (x right, y down, z forward, y vertical); all views are tilted
// up by TILT (towards the towers) and then turned about the vertical axis; azimuth = atan2(x, z) (to the right),
// elevation = atan2(−y, √(x² + z²)) (up). R_i maps world → camera i, Rcw = R_iᵀ maps camera i → world.
import { mul3, inv3 } from './util.js';

export const IW = 480, IH = 360, F = 450, OX = IW / 2, OY = IH / 2;   // 56° × 44° field of view
export const K = [F, 0, OX, 0, F, OY, 0, 0, 1], Ki = inv3(K);
export const deg = Math.PI / 180;
export const Rx = a => [1, 0, 0, 0, Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a)];
export const Ry = a => [Math.cos(a), 0, Math.sin(a), 0, 1, 0, -Math.sin(a), 0, Math.cos(a)];
export const Rz = a => [Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a), 0, 0, 0, 1];
export const T3 = m => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];

const SRC = new URL('../../assets/lecture03/canary_wharf_band.jpg', import.meta.url).href;
const EL_TOP = 40 * deg, EL_BOT = -35.02 * deg;   // elevation of the first and last row of the cropped band
const AZ0 = 0.5;
export const TILT = 12 * deg;                      // common upward tilt of all views                                   // azimuth 0 (forward of the reference view) at this fraction of the width

// Views: yaw (deg) of each camera about the vertical axis, small pitch/roll differences (radians), exposure gain.
// yaws: optional list of yaws instead of the presets (then no pitch / roll, gain 1).
export function makeViews(n, yaws) {
  const P = yaws ? { yaw: yaws, pitch: yaws.map(() => 0), roll: yaws.map(() => 0), gain: yaws.map(() => 1) }
    : n === 5
      ? { yaw: [-56, -28, 0, 28, 56], pitch: [0.015, -0.01, 0, 0.012, -0.015], roll: [-0.012, 0.015, 0, -0.01, 0.012], gain: [1.12, 0.9, 1.0, 0.86, 1.1] }
      : { yaw: [-30, 0, 30], pitch: [0.02, 0, -0.015], roll: [0.015, 0, -0.02], gain: [1.15, 1.0, 0.84] };
  return P.yaw.map((y, i) => {
    const Rcw = mul3(Ry(y * deg), mul3(Rx(TILT + P.pitch[i]), Rz(P.roll[i])));  // camera → world
    return { yaw: y, R: T3(Rcw), Rcw, gain: P.gain[i] };
  });
}

// Load the photo once (shared by all widgets): { data: Uint8ClampedArray RGBA, W, H }.
let envPromise = null;
export function loadEnv() {
  if (!envPromise) envPromise = new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => {
      const c = document.createElement('canvas'); c.width = img.naturalWidth; c.height = img.naturalHeight;
      const g = c.getContext('2d', { willReadFrequently: true }); g.drawImage(img, 0, 0);
      resolve({ data: g.getImageData(0, 0, c.width, c.height).data, W: c.width, H: c.height });
    };
    img.onerror = () => reject(new Error('cannot load ' + SRC));
    img.src = SRC;
  });
  return envPromise;
}

// Bilinear colour (0..255) of the world direction d; writes into out[0..2].
export function sampleDir(env, d, out) {
  const { data, W, H } = env;
  const az = Math.atan2(d[0], d[2]), el = Math.atan2(-d[1], Math.hypot(d[0], d[2]));
  let x = (az / (2 * Math.PI) + AZ0) * W - 0.5;
  let y = (EL_TOP - el) / (EL_TOP - EL_BOT) * H - 0.5;
  if (y < 0) y = 0; else if (y > H - 1.001) y = H - 1.001;
  x = ((x % W) + W) % W;
  const ix = Math.floor(x), iy = Math.floor(y), ax = x - ix, ay = y - iy, ix1 = ix + 1 === W ? 0 : ix + 1;
  const i00 = 4 * (iy * W + ix), i10 = 4 * (iy * W + ix1), i01 = i00 + 4 * W, i11 = i10 + 4 * W;
  for (let c = 0; c < 3; c++)
    out[c] = (data[i00 + c] * (1 - ax) + data[i10 + c] * ax) * (1 - ay) + (data[i01 + c] * (1 - ax) + data[i11 + c] * ax) * ay;
  return out;
}

// Render a view (RGBA, IW × IH) with 2 × 2 supersampling; gain and vignetting applied unless raw.
export function renderView(env, v, { raw = false } = {}) {
  const rgba = new Uint8ClampedArray(IW * IH * 4), M = mul3(v.Rcw, Ki), c = [0, 0, 0];
  const r2max = OX * OX + OY * OY, d = [0, 0, 0];
  for (let y = 0; y < IH; y++) for (let x = 0; x < IW; x++) {
    let r = 0, gg = 0, b = 0;
    for (let sy = 0.25; sy < 1; sy += 0.5) for (let sx = 0.25; sx < 1; sx += 0.5) {
      const u = x + sx, w = y + sy;
      d[0] = M[0] * u + M[1] * w + M[2]; d[1] = M[3] * u + M[4] * w + M[5]; d[2] = M[6] * u + M[7] * w + M[8];
      sampleDir(env, d, c); r += c[0]; gg += c[1]; b += c[2];
    }
    const r2 = ((x + 0.5 - OX) ** 2 + (y + 0.5 - OY) ** 2) / r2max;
    const k = raw ? 0.27 : 0.27 * v.gain * (1 - 0.34 * r2), i = 4 * (y * IW + x);
    rgba[i] = r * k; rgba[i + 1] = gg * k; rgba[i + 2] = b * k; rgba[i + 3] = 255;
  }
  return rgba;
}

export const CREDIT = 'panorama: “Canary Wharf”, Andreas Mischok, Poly Haven (CC0)';
