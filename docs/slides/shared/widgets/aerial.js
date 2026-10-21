// Aerial mosaic from a photogrammetry flight: one ground homography per photo.
// A procedural farmland (fields with crop rows and tramlines, a road, a river, tree clumps, a farm), 300 m × 200 m,
// is photographed by a drone flying a lawnmower (serpentine) pattern at altitude A with a nadir camera. Each photo is
// rendered exactly: every pixel's ray is intersected with the ground plane (and, optionally, with a few 3D objects).
// The drone's attitude wobbles slightly (roll, pitch ±3°, yaw ±5°), so each footprint is a slightly irregular quad.
//
// World frame: X east, Y south (down on the map), Z down (right-handed); ground Z = 0, drone at Z = −A.
// Camera frame (OpenCV): x right, y down, z forward. Nadir camera, heading 0: camera axes = world axes, so an
// unrotated photo looks like the map. R_cw = Rz(yaw) Ry(pitch) Rx(roll) holds the camera axes in world coordinates;
// world → camera: x_c = R (X − c), R = R_cwᵀ.
// Ground point (X, Y, 0): x_c = X r1 + Y r2 + t, t = −R c, so ũ ≡ K [r1 r2 t] (X, Y, 1)ᵀ: the ground homography H_i.
// The mosaic is a top view of the ground: each mosaic pixel (X, Y) is sent through H_i into photo i (backward warping)
// and the photos are blended (feather: weight = distance to the photo border; or the last photo wins).
// "GPS/IMU noise": the mosaic uses poses with small errors (σ 1 m horizontally, 0.4 m vertically, 0.5° in
// attitude) while the photos were taken at the true poses: seams and doubled features appear, which is why real
// pipelines match features between photos and refine all poses by bundle adjustment.
// "buildings / trees (3D)": the farm buildings, a silo and a few large trees get their height. The ground homography
// is right only for points on the ground: tops are shifted away from each photo's nadir point (relief displacement,
// Δr = r h / A with r the distance of the displaced image from the nadir point), so they disagree between photos.
//
// config: {
//   width: 540          mosaic width in px (the map and the photo are 320 px wide)
//   altitude: 100       flight altitude A in m (slider 60 … 150)
//   front: 0.75         front (along track) overlap (slider 0.5 … 0.9)
//   side: 0.65          side (across track) overlap (slider 0.3 … 0.8)
//   noise: false        GPS/IMU noise in the poses used for the mosaic
//   buildings: false    3D objects
//   blend: "feather"    or "last"
//   photo: null         index of the selected photo (default: one in the middle of the flight)
// }
// Print: the configured state, controls hidden.
import { C, h, fmt, isPrint } from './util.js';

const deg = Math.PI / 180;
const PW = 400, PH = 300, F = 400 * 100 / 90, POX = PW / 2, POY = PH / 2;   // footprint at 100 m: 90 m × 67.5 m
const MW = 200, MH = 150, MF = F / 2;                                        // photos at half resolution for the mosaic
const AREA = [0, 0, 300, 200];

// ---------------- noise ----------------
function hash2(i, j, s = 0) {
  let n = (i * 374761393 + j * 668265263 + s * 2147483647) | 0;
  n = Math.imul(n ^ (n >>> 13), 1274126177);
  return ((n ^ (n >>> 16)) >>> 0) / 4294967296;
}
function vnoise(x, y, s = 0) {
  const i = Math.floor(x), j = Math.floor(y), fx = x - i, fy = y - j;
  const ux = fx * fx * (3 - 2 * fx), uy = fy * fy * (3 - 2 * fy);
  const a = hash2(i, j, s), b = hash2(i + 1, j, s), c = hash2(i, j + 1, s), d = hash2(i + 1, j + 1, s);
  return a + (b - a) * ux + (c - a) * uy + (a - b - c + d) * ux * uy;
}
function fbm(x, y, oct, s) {
  let v = 0, a = 0.5, f = 1, n = 0;
  for (let o = 0; o < oct; o++) { v += a * vnoise(x * f, y * f, s + o); n += a; a *= 0.5; f *= 2.07; }
  return v / n;
}
function rng(seed) {
  let a = seed >>> 0;
  const u = () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  u.normal = () => Math.sqrt(-2 * Math.log(u() || 1e-12)) * Math.cos(2 * Math.PI * u());
  return u;
}

// ---------------- the scene ----------------
const COLX = [-80, 22, 84, 146, 208, 266, 380], ROWY = [-70, 48, 104, 166, 270];
const bx = (k, Y) => COLX[k] + (k > 0 && k < COLX.length - 1 ? 5 * Math.sin(Y / 37 + 1.7 * k) : 0);
const by = (j, X) => ROWY[j] + (j > 0 && j < ROWY.length - 1 ? 4 * Math.sin(X / 43 + 2.1 * j) : 0);
const roadY = X => 128 + 14 * Math.sin(X / 66 + 0.4) + 4 * Math.sin(X / 23);
const riverX = Y => 236 + 20 * Math.sin(Y / 47 + 0.8) + 7 * Math.sin(Y / 16.5);
// farm: yard and buildings (x0, y0, x1, y1, height, roof colour); a silo; big solitary trees (x, y, radius, height)
const YARD = [52, 136, 114, 168];
const BUILDINGS = [
  { r: [58, 141, 84, 152], h: 9, roof: [0.62, 0.24, 0.17], ridge: 'x', name: 'house' },
  { r: [90, 140, 110, 162], h: 11, roof: [0.55, 0.57, 0.60], ridge: 'y', name: 'barn' },
  { r: [60, 156, 74, 166], h: 6, roof: [0.58, 0.28, 0.20], ridge: 'x', name: 'shed' },
  { r: [78, 157, 84, 163], h: 24, roof: [0.80, 0.80, 0.78], ridge: null, name: 'silo' },
];
const BIG_TREES = [[150, 132, 6, 17], [196, 72, 7, 19], [30, 92, 6, 15], [262, 176, 6.5, 18]];
const CLUMPS = (() => {
  const r = rng(42), out = [];
  // along the river banks
  for (let Y = -60; Y < 260; Y += 6 + 6 * r()) {
    const side = r() < 0.5 ? -1 : 1;
    out.push([riverX(Y) + side * (8 + 4 * r()), Y, 3 + 3.5 * r()]);
  }
  // hedgerow pieces and a small wood
  for (let k = 0; k < 26; k++) out.push([170 + 26 * r(), 10 + 30 * r(), 3 + 4 * r()]);
  for (let k = 0; k < 14; k++) { const X = -40 + 380 * r(); out.push([X, by(2, X) + (r() - 0.5) * 2, 2.5 + 2 * r()]); }
  return out;
})();
const SUN = [0.55, -0.45, -0.70];   // direction to the sun (from SE-ish... light from the upper left of the map)

const clamp01 = v => (v < 0 ? 0 : v > 1 ? 1 : v);
const mix = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
const CROPS = ['wheat', 'green', 'ploughed', 'meadow', 'rapeseed', 'stubble', 'green', 'wheat'];

// tileable fbm noise tables (fast lookups instead of evaluating noise per raster pixel)
const TN = 256;
function makeTable(seed) {
  const t = new Float32Array(TN * TN);
  const wrapNoise = (x, y, per, s) => {
    const i = Math.floor(x), j = Math.floor(y), fx = x - i, fy = y - j;
    const ux = fx * fx * (3 - 2 * fx), uy = fy * fy * (3 - 2 * fy);
    const m = k => ((k % per) + per) % per;
    const a = hash2(m(i), m(j), s), b = hash2(m(i + 1), m(j), s), c = hash2(m(i), m(j + 1), s), d = hash2(m(i + 1), m(j + 1), s);
    return a + (b - a) * ux + (c - a) * uy + (a - b - c + d) * ux * uy;
  };
  for (let y = 0; y < TN; y++) for (let x = 0; x < TN; x++) {
    let v = 0, a = 0.5, n = 0;
    for (let o = 0, per = 8; o < 4; o++, per *= 2) { v += a * wrapNoise(x * per / TN, y * per / TN, per, seed + o); n += a; a *= 0.5; }
    t[y * TN + x] = v / n;
  }
  return t;
}
const NT = [makeTable(11), makeTable(31)];
// fbm value at ground (X, Y) with features of about `size` metres
function nz(k, X, Y, size) {
  const t = NT[k];
  const x = X / size * (TN / 8), y = Y / size * (TN / 8), fi = Math.floor(x), fj = Math.floor(y);
  const fx = x - fi, fy = y - fj, i = fi & (TN - 1), j = fj & (TN - 1), i1 = (i + 1) & (TN - 1), j1 = (j + 1) & (TN - 1);
  const a = t[j * TN + i], b = t[j * TN + i1], c = t[j1 * TN + i], d = t[j1 * TN + i1];
  return a + (b - a) * fx + (c - a) * fy + (a - b - c + d) * fx * fy;
}

// per parcel: crop type and row direction (cached)
const PARCEL = new Map();
function parcel(i, j) {
  const key = i * 16 + j;
  let q = PARCEL.get(key);
  if (!q) {
    const ang = (hash2(i, j, 5) < 0.5 ? 0 : Math.PI / 2) + (hash2(i, j, 3) - 0.5) * 0.35;
    q = { type: CROPS[Math.floor(hash2(i, j, 9) * CROPS.length)], ca: Math.cos(ang), sa: Math.sin(ang) };
    PARCEL.set(key, q);
  }
  return q;
}
const COLS = {   // [dark, bright] per crop
  wheat: [[0.62, 0.52, 0.28], [0.80, 0.69, 0.40]], rapeseed: [[0.70, 0.66, 0.16], [0.92, 0.84, 0.26]],
  meadow: [[0.25, 0.42, 0.17], [0.42, 0.56, 0.26]], ploughed: [[0.30, 0.22, 0.15], [0.48, 0.37, 0.26]],
  stubble: [[0.62, 0.58, 0.44], [0.76, 0.71, 0.55]], soil: [[0.36, 0.30, 0.20]], leaf: [[0.22, 0.40, 0.14], [0.36, 0.52, 0.20]],
};
function fieldColor(X, Y, i, j, dB, out) {
  const { type, ca, sa } = parcel(i, j);
  const along = X * ca + Y * sa, across = -X * sa + Y * ca;
  const n1 = nz(0, X, Y, 22), n2 = nz(1, X, Y, 1.3);
  const row = 0.5 + 0.5 * Math.cos(2 * Math.PI * across / 0.75);   // crop rows 0.75 m apart
  const tram = ((across % 18) + 18) % 18;                           // tramlines every 18 m: two wheel tracks
  const inTram = Math.abs(tram - 8.1) < 0.22 || Math.abs(tram - 9.9) < 0.22;
  let r, g, b, t, A, B;
  if (type === 'green') {
    const L = COLS.leaf, S = COLS.soil[0];
    const lr = L[0][0] + (L[1][0] - L[0][0]) * n1, lg = L[0][1] + (L[1][1] - L[0][1]) * n1, lb = L[0][2] + (L[1][2] - L[0][2]) * n1;
    t = 0.3 + 0.6 * row + 0.1 * n2;
    r = S[0] + (lr - S[0]) * t; g = S[1] + (lg - S[1]) * t; b = S[2] + (lb - S[2]) * t;
  } else {
    [A, B] = COLS[type];
    if (type === 'wheat') t = 0.3 + 0.4 * n1 + 0.2 * row + 0.1 * n2;
    else if (type === 'rapeseed') t = 0.25 + 0.4 * n1 + 0.25 * row + 0.1 * n2;
    else if (type === 'meadow') t = 0.2 + 0.55 * n1 + 0.25 * n2;
    else if (type === 'ploughed') t = 0.25 + 0.45 * (0.5 + 0.5 * Math.cos(2 * Math.PI * across / 1.4)) + 0.3 * n2;
    else t = 0.35 * n1 + 0.35 * n2 + 0.3 * (0.5 + 0.5 * Math.cos(2 * Math.PI * (along + across) / 2.2));
    r = A[0] + (B[0] - A[0]) * t; g = A[1] + (B[1] - A[1]) * t; b = A[2] + (B[2] - A[2]) * t;
  }
  if (inTram && type !== 'meadow') { r = r * 0.3 + 0.28; g = g * 0.3 + 0.238; b = b * 0.3 + 0.175; }
  const wet = nz(1, X + 500, Y, 70);
  if (wet > 0.6) { const k = 1 - 0.6 * (wet - 0.6); r *= k; g *= k; b *= k; }
  if (dB < 1.6) { r = r * 0.2 + 0.4; g = g * 0.2 + 0.368; b = b * 0.2 + 0.288; }   // field margins / tracks
  out[0] = r; out[1] = g; out[2] = b;
}

function canopyColor(X, Y, cx, cy, r, d) {
  const n = nz(0, X, Y, 1.6);
  const lit = clamp01(0.55 - 0.45 * ((X - cx) * 0.6 + (Y - cy) * 0.8) / r);   // brighter towards the sun (upper left)
  const k = 0.45 + 0.45 * n + 0.35 * lit - 0.25 * d * d;
  return [0.12 * k + 0.03, 0.30 * k + 0.05, 0.10 * k + 0.02];
}

function roofColor(b, X, Y) {
  const [x0, y0, x1, y1] = b.r;
  let c = b.roof.slice();
  if (b.ridge === 'x') {           // ridge along x: the two halves lit differently, tile rows
    const m = (y0 + y1) / 2, k = Y < m ? 1.1 : 0.82;
    const tiles = 0.9 + 0.1 * Math.cos(2 * Math.PI * (Y - y0) / 0.6);
    c = c.map(v => v * k * tiles);
    if (Math.abs(Y - m) < 0.25) c = c.map(v => v * 0.75);
  } else if (b.ridge === 'y') {    // metal roof, ridge along y, ribs
    const m = (x0 + x1) / 2, k = X < m ? 1.08 : 0.85;
    const ribs = 0.88 + 0.12 * Math.cos(2 * Math.PI * (Y - y0) / 0.9);
    c = c.map(v => v * k * ribs);
  } else {                          // silo: round top, drawn as a disc on a square base
    const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2, d = Math.hypot(X - cx, Y - cy) / ((x1 - x0) / 2);
    if (d > 1) return [0.45, 0.43, 0.40];
    c = c.map(v => v * (1.05 - 0.35 * d * d - 0.15 * ((X - cx) + (Y - cy)) / (x1 - x0)));
  }
  return c;
}

// ground raster (0.2 m), computed once: base layer per pixel, then shadows, tree crowns and roofs painted on top
const RES = 0.2, RX0 = -80, RY0 = -70, RW = Math.round(460 / RES), RH = Math.round(340 / RES);
let RASTER = null;
function raster() {
  if (RASTER) return RASTER;
  const a = new Uint8ClampedArray(RW * RH * 3), out = [0, 0, 0];
  const Xs = new Float64Array(RW), roadAt = new Float64Array(RW), byAt = ROWY.map(() => new Float64Array(RW));
  for (let i = 0; i < RW; i++) { const X = RX0 + (i + 0.5) * RES; Xs[i] = X; roadAt[i] = roadY(X); ROWY.forEach((_, j) => { byAt[j][i] = by(j, X); }); }
  const put = (o, c) => { a[o] = c[0] * 255; a[o + 1] = c[1] * 255; a[o + 2] = c[2] * 255; };
  for (let jj = 0; jj < RH; jj++) {
    const Y = RY0 + (jj + 0.5) * RES, xr = riverX(Y), bxs = COLX.map((_, k) => bx(k, Y));
    for (let ii = 0; ii < RW; ii++) {
      const X = Xs[ii], o = 3 * (jj * RW + ii), dRoad = Math.abs(Y - roadAt[ii]), dRiv = Math.abs(X - xr);
      if (dRoad < 3) {
        const n = 0.06 * nz(1, X, Y, 0.8);
        let c = [0.34 + n, 0.34 + n, 0.35 + n];
        if (dRoad < 0.12 && ((X % 9) + 9) % 9 < 3) c = [0.88, 0.88, 0.84];   // dashed centre line
        if (dRoad > 2.75) c = [0.80, 0.80, 0.76];                            // edge lines
        put(o, c);
      } else if (dRoad < 4.2) put(o, mix([0.55, 0.52, 0.45], [0.66, 0.62, 0.54], nz(1, X, Y, 0.6)));
      else if (dRiv < 4.5) {
        let c = mix([0.12, 0.22, 0.24], [0.22, 0.34, 0.34], nz(0, X * 0.35, Y * 1.1, 3));
        if (dRiv > 3.8) c = mix(c, [0.35, 0.30, 0.22], 0.6);
        put(o, c);
      } else if (dRiv < 6) put(o, mix([0.36, 0.31, 0.22], [0.30, 0.40, 0.20], nz(1, X, Y, 1)));
      else if (X >= YARD[0] && X <= YARD[2] && Y >= YARD[1] && Y <= YARD[3]) {
        const n = nz(1, X, Y, 0.9);
        put(o, Math.abs(Y - 152.5) < 2 && X < 90 ? [0.45, 0.43, 0.40] : [0.56 + 0.08 * n, 0.54 + 0.08 * n, 0.50 + 0.07 * n]);
      } else if (X > 50 && X < 56 && Y > 128 && Y < 140) put(o, [0.50, 0.48, 0.44]);   // access lane
      else {
        let i = 0; while (i < COLX.length - 2 && X > bxs[i + 1]) i++;
        let j = 0; while (j < ROWY.length - 2 && Y > byAt[j + 1][ii]) j++;
        const dB = Math.min(Math.abs(X - bxs[i]), Math.abs(X - bxs[i + 1]), Math.abs(Y - byAt[j][ii]), Math.abs(Y - byAt[j + 1][ii]));
        fieldColor(X, Y, i, j, dB, out); put(o, out);
      }
    }
  }
  // paint helper over a ground rectangle
  const paint = (x0, y0, x1, y1, fn) => {
    const i0 = Math.max(0, Math.floor((x0 - RX0) / RES)), i1 = Math.min(RW - 1, Math.ceil((x1 - RX0) / RES));
    const j0 = Math.max(0, Math.floor((y0 - RY0) / RES)), j1 = Math.min(RH - 1, Math.ceil((y1 - RY0) / RES));
    for (let jj = j0; jj <= j1; jj++) for (let ii = i0; ii <= i1; ii++) fn(RX0 + (ii + 0.5) * RES, RY0 + (jj + 0.5) * RES, 3 * (jj * RW + ii));
  };
  const darken = k => (X, Y, o) => { a[o] *= k; a[o + 1] *= k; a[o + 2] *= k; };
  const trees = [...CLUMPS, ...BIG_TREES.map(([x, y, r]) => [x, y, r])];
  // shadows (towards the lower right on the map), then crowns, then roofs
  const shadowed = new Uint8Array(RW * RH);
  trees.forEach(([cx, cy, r]) => paint(cx - r, cy - r, cx + 1.6 * r, cy + 1.7 * r, (X, Y, o) => {
    if (Math.hypot(X - cx - 0.5 * r, Y - cy - 0.6 * r) < r && !shadowed[o / 3]) { shadowed[o / 3] = 1; darken(0.6)(X, Y, o); }
  }));
  BUILDINGS.forEach(b => { const [x0, y0, x1, y1] = b.r; paint(x0, y0, x1 + 2.5, y1 + 3, (X, Y, o) => { if (!shadowed[o / 3]) { shadowed[o / 3] = 1; darken(0.6)(X, Y, o); } }); });
  trees.forEach(([cx, cy, r]) => paint(cx - r, cy - r, cx + r, cy + r, (X, Y, o) => {
    const d = Math.hypot(X - cx, Y - cy) / r;
    if (d <= 1) put(o, canopyColor(X, Y, cx, cy, r, d));
  }));
  BUILDINGS.forEach(b => { const [x0, y0, x1, y1] = b.r; paint(x0, y0, x1, y1, (X, Y, o) => { if (X >= x0 && X <= x1 && Y >= y0 && Y <= y1) put(o, roofColor(b, X, Y)); }); });
  return (RASTER = a);
}
function sampleGround(X, Y, out) {
  const R = RASTER;
  let x = (X - RX0) / RES - 0.5, y = (Y - RY0) / RES - 0.5;
  x = x < 0 ? 0 : x > RW - 1.001 ? RW - 1.001 : x; y = y < 0 ? 0 : y > RH - 1.001 ? RH - 1.001 : y;
  const i = x | 0, j = y | 0, fx = x - i, fy = y - j, o = 3 * (j * RW + i), o2 = o + 3 * RW;
  for (let k = 0; k < 3; k++) {
    const a = R[o + k] + (R[o + 3 + k] - R[o + k]) * fx, b = R[o2 + k] + (R[o2 + 3 + k] - R[o2 + k]) * fx;
    out[k] = a + (b - a) * fy;
  }
}

// 3D objects for the "buildings / trees (3D)" mode: boxes (buildings, silo) and spheres (big trees)
const BOXES = BUILDINGS.map(b => ({ lo: [b.r[0], b.r[1], -b.h], hi: [b.r[2], b.r[3], 0], b }));
const SPHERES = BIG_TREES.map(([x, y, r, hgt]) => ({ c: [x, y, -(hgt - r)], r }));
const WALL = [0.70, 0.66, 0.58];

// ---------------- cameras ----------------
const Rx = a => [1, 0, 0, 0, Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a)];
const Ry = a => [Math.cos(a), 0, Math.sin(a), 0, 1, 0, -Math.sin(a), 0, Math.cos(a)];
const Rz = a => [Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a), 0, 0, 0, 1];
function mul(a, b) {
  const r = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) r[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
  return r;
}
const T3 = m => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];

// flight plan: lines along Y (north–south), the long image side across track
function plan(A, front, side) {
  const fw = A * PW / F, fh = A * PH / F;      // footprint (m) of a level photo
  const sx = fw * (1 - side), sy = fh * (1 - front);
  const nx = Math.max(1, Math.ceil((AREA[2] - fw * 0.85) / sx) + 1), ny = Math.max(1, Math.ceil((AREA[3] - fh * 0.85) / sy) + 1);
  const x0 = AREA[2] / 2 - (nx - 1) * sx / 2, y0 = AREA[3] / 2 - (ny - 1) * sy / 2;
  const r = rng(1234), rn = rng(777), photos = [];
  for (let k = 0; k < nx; k++) for (let m = 0; m < ny; m++) {
    const mm = k % 2 ? ny - 1 - m : m;           // serpentine
    const c = [x0 + k * sx + (r() - 0.5) * 2, y0 + mm * sy + (r() - 0.5) * 1.5, -A + (r() - 0.5) * 1];
    const roll = (r() - 0.5) * 6 * deg, pitch = (r() - 0.5) * 6 * deg, yaw = (r() - 0.5) * 10 * deg;
    const Rcw = mul(Rz(yaw), mul(Ry(pitch), Rx(roll)));
    // estimated pose (GPS/IMU): small errors
    const ce = [c[0] + 1.0 * rn.normal(), c[1] + 1.0 * rn.normal(), c[2] + 0.4 * rn.normal()];
    const Rcwe = mul(Rz(yaw + 0.5 * deg * rn.normal()), mul(Ry(pitch + 0.5 * deg * rn.normal()), Rx(roll + 0.5 * deg * rn.normal())));
    photos.push({ c, Rcw, ce, Rcwe, gain: 1 + 0.07 * (r() - 0.5) * 2, line: k });
  }
  return { photos, fw, fh, sx, sy, nx, ny };
}

// ground → photo homography for pose (c, Rcw) and focal f, principal point (ox, oy): H = K [r1 r2 t]
function groundH(c, Rcw, f, ox, oy) {
  const R = T3(Rcw), t = [0, 1, 2].map(i => -(R[3 * i] * c[0] + R[3 * i + 1] * c[1] + R[3 * i + 2] * c[2]));
  const M = [R[0], R[1], t[0], R[3], R[4], t[1], R[6], R[7], t[2]];   // [r1 r2 t]
  return mul([f, 0, ox, 0, f, oy, 0, 0, 1], M);
}

// ray → colour (ground or 3D objects)
function shade(o, d, objects, out) {
  let tBest = (0 - o[2]) / d[2], kind = 0, obj = null, nrm = null;
  if (objects) {
    for (const bx of BOXES) {
      let t0 = -Infinity, t1 = Infinity, ax = -1, sg = 0;
      for (let a = 0; a < 3; a++) {
        const inv = 1 / d[a];
        let ta = (bx.lo[a] - o[a]) * inv, tb = (bx.hi[a] - o[a]) * inv, s = -1;
        if (ta > tb) { const q = ta; ta = tb; tb = q; s = 1; }
        if (ta > t0) { t0 = ta; ax = a; sg = s; }
        if (tb < t1) t1 = tb;
      }
      if (t0 <= t1 && t0 > 0 && t0 < tBest) { tBest = t0; kind = 1; obj = bx; nrm = [0, 0, 0]; nrm[ax] = sg; }
    }
    for (const sp of SPHERES) {
      const oc = [o[0] - sp.c[0], o[1] - sp.c[1], o[2] - sp.c[2]];
      const B = oc[0] * d[0] + oc[1] * d[1] + oc[2] * d[2], A2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
      const D = B * B - A2 * (oc[0] * oc[0] + oc[1] * oc[1] + oc[2] * oc[2] - sp.r * sp.r);
      if (D >= 0) { const t = (-B - Math.sqrt(D)) / A2; if (t > 0 && t < tBest) { tBest = t; kind = 2; obj = sp; } }
    }
  }
  const X = o[0] + tBest * d[0], Y = o[1] + tBest * d[1];
  if (kind === 0) { sampleGround(X, Y, out); return; }
  if (kind === 1) {
    if (nrm[2] === -1) { const c = roofColor(obj.b, X, Y); out[0] = c[0] * 255; out[1] = c[1] * 255; out[2] = c[2] * 255; return; }
    const l = 0.55 + 0.45 * Math.max(0, nrm[0] * -SUN[0] + nrm[1] * -SUN[1]);
    const Z = o[2] + tBest * d[2], k = l * (0.9 + 0.1 * Math.cos(Z * 2));
    out[0] = WALL[0] * 255 * k; out[1] = WALL[1] * 255 * k; out[2] = WALL[2] * 255 * k; return;
  }
  // tree crown
  const Z = o[2] + tBest * d[2];
  const n = [(X - obj.c[0]) / obj.r, (Y - obj.c[1]) / obj.r, (Z - obj.c[2]) / obj.r];
  const l = clamp01(0.35 + 0.65 * Math.max(0, -(n[0] * SUN[0] + n[1] * SUN[1] + n[2] * SUN[2])));
  const nz2 = nz(0, X + 3 * Z, Y, 1.4);
  const k = 0.4 + 0.5 * nz2 + 0.5 * l;
  out[0] = (0.12 * k + 0.03) * 255; out[1] = (0.30 * k + 0.05) * 255; out[2] = (0.10 * k + 0.02) * 255;
}

// render a photo (true pose) at W × H with focal f; gain and vignetting per photo
function renderPhoto(p, W, H, f, objects) {
  const img = new Uint8ClampedArray(W * H * 4), out = [0, 0, 0], R = p.Rcw, ox = W / 2, oy = H / 2;
  for (let v = 0; v < H; v++) for (let u = 0; u < W; u++) {
    const xc = (u + 0.5 - ox) / f, yc = (v + 0.5 - oy) / f;
    const d = [R[0] * xc + R[1] * yc + R[2], R[3] * xc + R[4] * yc + R[5], R[6] * xc + R[7] * yc + R[8]];
    shade(p.c, d, objects, out);
    const rr = ((u - ox) * (u - ox) + (v - oy) * (v - oy)) / (ox * ox + oy * oy), g = p.gain * (1 - 0.22 * rr);
    const i = 4 * (v * W + u);
    img[i] = out[0] * g; img[i + 1] = out[1] * g; img[i + 2] = out[2] * g; img[i + 3] = 255;
  }
  return img;
}

// ---------------- widget ----------------
export async function mount(el, cfg) {
  const print = isPrint();
  const state = {
    A: cfg.altitude ?? 100, front: cfg.front ?? 0.75, side: cfg.side ?? 0.65,
    noise: !!cfg.noise, objects: !!cfg.buildings, blend: cfg.blend === 'last' ? 'last' : 'feather', sel: cfg.photo ?? null,
  };
  const MOS_W = cfg.width || 540, MOS_H = Math.round(MOS_W * 2 / 3);
  const MAPW = 320, MAPWIN = [-50, -40, 350, 240], MAPH = Math.round(MAPW * (MAPWIN[3] - MAPWIN[1]) / (MAPWIN[2] - MAPWIN[0]));
  raster();

  // overview image of the ground for the map (once)
  const ov = document.createElement('canvas');
  ov.width = MAPW * 2; ov.height = MAPH * 2;
  {
    const og = ov.getContext('2d'), id = og.createImageData(ov.width, ov.height), out = [0, 0, 0];
    for (let j = 0; j < ov.height; j++) for (let i = 0; i < ov.width; i++) {
      const X = MAPWIN[0] + (i + 0.5) / ov.width * (MAPWIN[2] - MAPWIN[0]), Y = MAPWIN[1] + (j + 0.5) / ov.height * (MAPWIN[3] - MAPWIN[1]);
      sampleGround(X, Y, out);
      const inside = X >= AREA[0] && X <= AREA[2] && Y >= AREA[1] && Y <= AREA[3], k = inside ? 1 : 0.55, o = 4 * (j * ov.width + i);
      id.data[o] = out[0] * k; id.data[o + 1] = out[1] * k; id.data[o + 2] = out[2] * k; id.data[o + 3] = 255;
    }
    og.putImageData(id, 0, 0);
  }

  const mapC = h('canvas', { width: MAPW, height: MAPH, style: 'cursor:pointer' });
  const photoC = h('canvas', { width: PW, height: PH, style: `width:${MAPW}px; height:${Math.round(MAPW * PH / PW)}px` });
  const mosC = h('canvas', { width: MOS_W, height: MOS_H, style: 'cursor:pointer' });
  const photoCap = h('div', { class: 'readout', style: 'font-size:0.9em' });
  const readout = h('div', { style: 'line-height:1.45; font-size:0.92em' });
  const relief = h('div', { style: 'line-height:1.4; font-size:0.92em; margin-top:0.5em' });

  let P = null, mosaicImg = null, photosLow = [], photosHigh = [], photoFull = null;

  function rebuild() {
    P = plan(state.A, state.front, state.side);
    if (state.sel == null || state.sel >= P.photos.length) state.sel = nearest(118, 128);
    state.sel = Math.max(0, Math.min(P.photos.length - 1, state.sel));
    photosLow = P.photos.map(p => renderPhoto(p, MW, MH, MF, state.objects));
    photosHigh = [];
    mosaic();
    photoFull = null;
  }

  function nearest(X, Y) {
    let best = 0, bd = Infinity;
    P.photos.forEach((p, i) => { const d = Math.hypot(p.c[0] - X, p.c[1] - Y); if (d < bd) { bd = d; best = i; } });
    return best;
  }

  // compose the photos onto a top-view region [X0, Y0] + (W, H) px at s px/m; imgs[i]: photo i rendered at pw × ph, focal f
  function compose(X0, Y0, s, W, Hh, imgs, pw, ph, f, only) {
    const acc = new Float32Array(W * Hh * 4), feather = state.blend === 'feather';
    P.photos.forEach((p, idx) => {
      const img = imgs[idx];
      if (!img || (only && !only.has(idx))) return;
      const H = state.noise ? groundH(p.ce, p.Rcwe, f, pw / 2, ph / 2) : groundH(p.c, p.Rcw, f, pw / 2, ph / 2);
      const fp = footprint(p, state.noise);
      let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
      fp.forEach(([X, Y]) => { x0 = Math.min(x0, X); x1 = Math.max(x1, X); y0 = Math.min(y0, Y); y1 = Math.max(y1, Y); });
      const i0 = Math.max(0, Math.floor((x0 - 3 - X0) * s)), i1 = Math.min(W - 1, Math.ceil((x1 + 3 - X0) * s));
      const j0 = Math.max(0, Math.floor((y0 - 3 - Y0) * s)), j1 = Math.min(Hh - 1, Math.ceil((y1 + 3 - Y0) * s));
      for (let j = j0; j <= j1; j++) {
        const Y = Y0 + (j + 0.5) / s;
        for (let i = i0; i <= i1; i++) {
          const X = X0 + (i + 0.5) / s;
          const w3 = H[6] * X + H[7] * Y + H[8];
          if (w3 <= 0) continue;
          const u = (H[0] * X + H[1] * Y + H[2]) / w3 - 0.5, v = (H[3] * X + H[4] * Y + H[5]) / w3 - 0.5;
          if (u < 0 || v < 0 || u > pw - 1.001 || v > ph - 1.001) continue;
          const a = u | 0, b = v | 0, fx = u - a, fy = v - b, o = 4 * (b * pw + a), o2 = o + 4 * pw;
          const wgt = feather ? Math.min(u + 0.5, pw - 0.5 - u, v + 0.5, ph - 0.5 - v) + 0.01 : 1;
          const q = 4 * (j * W + i);
          for (let k = 0; k < 3; k++) {
            const c1 = img[o + k] + (img[o + 4 + k] - img[o + k]) * fx, c2 = img[o2 + k] + (img[o2 + 4 + k] - img[o2 + k]) * fx;
            const c = c1 + (c2 - c1) * fy;
            if (feather) acc[q + k] += wgt * c; else acc[q + k] = c;
          }
          if (feather) acc[q + 3] += wgt; else acc[q + 3] = 1;
        }
      }
    });
    const id = new ImageData(W, Hh);
    for (let q = 0; q < W * Hh; q++) {
      const w = acc[4 * q + 3];
      if (w > 0) { for (let k = 0; k < 3; k++) id.data[4 * q + k] = acc[4 * q + k] / w; }
      else { id.data[4 * q] = 30; id.data[4 * q + 1] = 33; id.data[4 * q + 2] = 40; }
      id.data[4 * q + 3] = 255;
    }
    return id;
  }

  // the farm, magnified: composed at 5 px/m from full-resolution photos
  const ZOOM = [53, 135, 113, 169], ZS = 6;
  const zoomC = document.createElement('canvas');
  zoomC.width = (ZOOM[2] - ZOOM[0]) * ZS; zoomC.height = (ZOOM[3] - ZOOM[1]) * ZS;
  function mosaic() {
    mosaicImg = compose(0, 0, MOS_W / (AREA[2] - AREA[0]), MOS_W, MOS_H, photosLow, MW, MH, MF);
    const near = new Set();
    P.photos.forEach((p, i) => {
      const fp = footprint(p);
      const xs = fp.map(q => q[0]), ys = fp.map(q => q[1]);
      if (Math.max(...xs) > ZOOM[0] && Math.min(...xs) < ZOOM[2] && Math.max(...ys) > ZOOM[1] && Math.min(...ys) < ZOOM[3]) near.add(i);
    });
    near.forEach(i => { if (!photosHigh[i]) photosHigh[i] = renderPhoto(P.photos[i], PW, PH, F, state.objects); });
    zoomC.getContext('2d').putImageData(compose(ZOOM[0], ZOOM[1], ZS, zoomC.width, zoomC.height, photosHigh, PW, PH, F, near), 0, 0);
  }


  // footprint on the ground: image corners → ground (true pose), or the estimated pose
  function footprint(p, est = false) {
    const c = est ? p.ce : p.c, R = est ? p.Rcwe : p.Rcw;
    return [[0, 0], [PW, 0], [PW, PH], [0, PH]].map(([u, v]) => {
      const xc = (u - POX) / F, yc = (v - POY) / F;
      const d = [R[0] * xc + R[1] * yc + R[2], R[3] * xc + R[4] * yc + R[5], R[6] * xc + R[7] * yc + R[8]];
      const t = -c[2] / d[2];
      return [c[0] + t * d[0], c[1] + t * d[1]];
    });
  }

  const mapXY = ([X, Y]) => [(X - MAPWIN[0]) / (MAPWIN[2] - MAPWIN[0]) * MAPW, (Y - MAPWIN[1]) / (MAPWIN[3] - MAPWIN[1]) * MAPH];

  function drawMap() {
    const g = mapC.getContext('2d');
    g.drawImage(ov, 0, 0, MAPW, MAPH);
    g.strokeStyle = 'rgba(255,255,255,0.8)'; g.lineWidth = 1.5; g.setLineDash([5, 4]);
    const a0 = mapXY([AREA[0], AREA[1]]), a1 = mapXY([AREA[2], AREA[3]]);
    g.strokeRect(a0[0], a0[1], a1[0] - a0[0], a1[1] - a0[1]); g.setLineDash([]);
    // footprints
    g.lineWidth = 1;
    P.photos.forEach(p => {
      const q = footprint(p).map(mapXY);
      g.strokeStyle = P.photos.length > 60 ? 'rgba(90,176,255,0.35)' : 'rgba(90,176,255,0.5)';
      g.beginPath(); q.forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y))); g.closePath(); g.stroke();
    });
    // flight path
    g.strokeStyle = C.accent; g.lineWidth = 2; g.beginPath();
    P.photos.forEach((p, i) => { const [x, y] = mapXY(p.c); if (i) g.lineTo(x, y); else g.moveTo(x, y); }); g.stroke();
    P.photos.forEach(p => { const [x, y] = mapXY(p.c); g.fillStyle = C.accent; g.beginPath(); g.arc(x, y, 2.2, 0, 7); g.fill(); });
    // selected
    const sp = P.photos[state.sel], q = footprint(sp).map(mapXY);
    g.fillStyle = 'rgba(242,177,52,0.22)'; g.strokeStyle = C.accent; g.lineWidth = 2.5;
    g.beginPath(); q.forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y))); g.closePath(); g.fill(); g.stroke();
    const [cx, cy] = mapXY(sp.c); g.fillStyle = '#ffffff'; g.beginPath(); g.arc(cx, cy, 4, 0, 7); g.fill();
    // scale bar
    const [s0] = mapXY([0, 0]), [s1] = mapXY([1, 0]);
    scaleBar(g, s1 - s0, MAPH);
  }

  function scaleBar(g, pxPerM, Hc) {
    const L = 50 * pxPerM;
    g.fillStyle = C.bg; g.fillRect(6, Hc - 22, L + 36, 16);
    g.fillStyle = C.fg; g.fillRect(10, Hc - 15, L, 3);
    g.font = '11px Inter, sans-serif'; g.fillText('50 m', 14 + L, Hc - 10);
  }

  function drawPhoto() {
    if (!photoFull) photoFull = photosHigh[state.sel] || (photosHigh[state.sel] = renderPhoto(P.photos[state.sel], PW, PH, F, state.objects));
    const g = photoC.getContext('2d');
    g.putImageData(new ImageData(photoFull, PW, PH), 0, 0);
    // nadir point (where the vertical through the camera meets the ground)
    if (state.objects) {
      const p = P.photos[state.sel], R = T3(p.Rcw), dv = [0, 0, 1];
      const xc = [R[0] * dv[0] + R[1] * dv[1] + R[2] * dv[2], R[3] * dv[0] + R[4] * dv[1] + R[5] * dv[2], R[6] * dv[0] + R[7] * dv[1] + R[8] * dv[2]];
      const u = F * xc[0] / xc[2] + POX, v = F * xc[1] / xc[2] + POY;
      g.strokeStyle = '#ffffff'; g.lineWidth = 2;
      g.beginPath(); g.moveTo(u - 8, v); g.lineTo(u + 8, v); g.moveTo(u, v - 8); g.lineTo(u, v + 8); g.stroke();
    }
    photoCap.textContent = `photo ${state.sel + 1} of ${P.photos.length} (line ${P.photos[state.sel].line + 1})`;
  }

  function drawMosaic() {
    const g = mosC.getContext('2d');
    g.putImageData(mosaicImg, 0, 0);
    const s = MOS_W / (AREA[2] - AREA[0]);
    const q = footprint(P.photos[state.sel], state.noise).map(([X, Y]) => [X * s, Y * s]);
    g.strokeStyle = C.accent; g.lineWidth = 2;
    g.beginPath(); q.forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y))); g.closePath(); g.stroke();
    // zoom inset of the farm (top right) and its frame in the mosaic
    const zw = Math.round(MOS_W * 0.46), zh = Math.round(zw * zoomC.height / zoomC.width), zx = MOS_W - zw - 6, zy = 6;
    g.strokeStyle = C.fg; g.lineWidth = 1.5; g.setLineDash([4, 3]);
    g.strokeRect(ZOOM[0] * s, ZOOM[1] * s, (ZOOM[2] - ZOOM[0]) * s, (ZOOM[3] - ZOOM[1]) * s); g.setLineDash([]);
    g.drawImage(zoomC, zx, zy, zw, zh);
    g.strokeStyle = C.fg; g.lineWidth = 2; g.strokeRect(zx, zy, zw, zh);
    g.fillStyle = C.bg; g.fillRect(zx, zy + zh - 18, 92, 18);
    g.fillStyle = C.fg; g.font = '12px Inter, sans-serif'; g.fillText(`farm, zoomed ×${fmt(zw / (ZOOM[2] - ZOOM[0]) / s, 1)}`, zx + 4, zy + zh - 5);
    scaleBar(g, s, MOS_H);
  }

  function drawReadout() {
    const gsd = state.A / F * 100;
    readout.innerHTML =
      `altitude <b>${fmt(state.A, 0)} m</b><br>` +
      `footprint ${fmt(P.fw, 0)} × ${fmt(P.fh, 0)} m<br>` +
      `GSD = A / f = <b>${fmt(gsd, 1)} cm/px</b><br>` +
      `overlap: front ${fmt(state.front * 100, 0)}%, side ${fmt(state.side * 100, 0)}%<br>` +
      `photos every ${fmt(P.sy, 1)} m, lines ${fmt(P.sx, 1)} m apart<br>` +
      `<b>${P.photos.length} photos</b> (${P.nx} lines × ${P.ny})<br>` +
      `<span class="dim">mosaic: ground homography of each photo, Hᵢ = K [r₁ r₂ t], from ${state.noise ? 'noisy' : 'exact'} GPS + IMU poses</span>`;
    if (state.objects) {
      // relief displacement of the object top that moves most in the selected photo (among those in view)
      const p = P.photos[state.sel], A = -p.c[2], R = T3(p.Rcw);
      const tops = [...BUILDINGS.map(b => ({ name: b.name, X: (b.r[0] + b.r[2]) / 2, Y: (b.r[1] + b.r[3]) / 2, h: b.h })),
        ...BIG_TREES.map(([x, y, , hh]) => ({ name: 'tree', X: x, Y: y, h: hh }))];
      let best = null;
      for (const o of tops) {
        const q = [o.X - p.c[0], o.Y - p.c[1], -o.h - p.c[2]], xc = [0, 1, 2].map(k => R[3 * k] * q[0] + R[3 * k + 1] * q[1] + R[3 * k + 2] * q[2]);
        const u = F * xc[0] / xc[2] + POX, v = F * xc[1] / xc[2] + POY;
        if (u < 0 || v < 0 || u > PW || v > PH) continue;
        const rho = Math.hypot(o.X - p.c[0], o.Y - p.c[1]), rd = rho * A / (A - o.h), dr = rd - rho;
        if (!best || dr > best.dr) best = { ...o, rd, dr };
      }
      relief.innerHTML = '<b>relief displacement</b>: the ground H draws a point at height h too far from the nadir point (+ in the photo) by' +
        '<br>Δr = r · h / A' + (best
          ? `<br><span class="dim">here: the ${best.name}, h = ${best.h} m, r = ${fmt(best.rd, 0)} m → Δr = ${fmt(best.dr, 1)} m</span>`
          : '<br><span class="dim">(no tall object in this photo)</span>');
    } else relief.innerHTML = '';
  }

  function drawAll() { drawMap(); drawPhoto(); drawMosaic(); drawReadout(); syncButtons(); }

  // controls
  let pending = null;
  const slider = (min, max, step, val, show, set) => {
    const out = h('span', { class: 'readout', style: 'display:inline-block; width:3.3em; text-align:right; white-space:nowrap' }, show(val));
    const inp = h('input', { type: 'range', min, max, step, value: val, style: 'width:4.6em' });
    inp.addEventListener('input', () => {
      const v = parseFloat(inp.value); out.textContent = show(v); set(v);
      clearTimeout(pending); pending = setTimeout(() => { rebuild(); drawAll(); }, 120);
    });
    return [inp, out];
  };
  const [aS, aV] = slider(60, 150, 5, state.A, v => `${v} m`, v => { state.A = v; });
  const [fS, fV] = slider(0.5, 0.9, 0.05, state.front, v => `${Math.round(v * 100)}%`, v => { state.front = v; });
  const [sS, sV] = slider(0.3, 0.8, 0.05, state.side, v => `${Math.round(v * 100)}%`, v => { state.side = v; });
  const noiseB = h('button', { onclick: () => { state.noise = !state.noise; mosaic(); drawAll(); } }, 'GPS/IMU noise');
  const objB = h('button', { onclick: () => { state.objects = !state.objects; rebuild(); drawAll(); } }, 'buildings / trees (3D)');
  const featherB = h('button', { onclick: () => { state.blend = 'feather'; mosaic(); drawAll(); } }, 'feather');
  const lastB = h('button', { onclick: () => { state.blend = 'last'; mosaic(); drawAll(); } }, 'last wins');
  const prevB = h('button', { onclick: () => select(state.sel - 1) }, '◀');
  const nextB = h('button', { onclick: () => select(state.sel + 1) }, '▶');
  function syncButtons() {
    noiseB.classList.toggle('active', state.noise); objB.classList.toggle('active', state.objects);
    featherB.classList.toggle('active', state.blend === 'feather'); lastB.classList.toggle('active', state.blend === 'last');
  }
  function select(i) {
    state.sel = (i + P.photos.length) % P.photos.length; photoFull = null;
    drawMap(); drawPhoto(); drawMosaic(); drawReadout();
  }
  function pickAt(X, Y) {
    select(nearest(X, Y));
  }
  mapC.addEventListener('click', e => {
    const r = mapC.getBoundingClientRect(), x = (e.clientX - r.left) / r.width * MAPW, y = (e.clientY - r.top) / r.height * MAPH;
    pickAt(MAPWIN[0] + x / MAPW * (MAPWIN[2] - MAPWIN[0]), MAPWIN[1] + y / MAPH * (MAPWIN[3] - MAPWIN[1]));
  });
  mosC.addEventListener('click', e => {
    const r = mosC.getBoundingClientRect(), s = MOS_W / (AREA[2] - AREA[0]);
    pickAt((e.clientX - r.left) / r.width * MOS_W / s, (e.clientY - r.top) / r.height * MOS_H / s);
  });

  const lab = (t, col) => h('div', { style: `color:${col}; font-size:0.95em; margin-bottom:0.15em` }, t);
  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.7em; align-items:flex-start' },
    h('div', { style: `width:${MAPW}px` },
      lab('flight plan: footprints of all photos', C.blue), mapC,
      lab('the selected photo (nadir camera)', C.accent), photoC, photoCap),
    h('div', { style: `width:${MOS_W}px` },
      lab('mosaic: each photo warped to the ground by its homography', C.green), mosC,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.35em' },
        h('label', {}, 'altitude'), aS, aV, h('span', { style: 'width:0.6em' }), h('label', {}, 'photo'), prevB, nextB),
      h('div', { class: 'wctl interactive-only', style: 'gap:0.35em' },
        h('label', {}, 'overlap: front'), fS, fV, h('label', {}, 'side'), sS, sV),
      h('div', { class: 'wctl interactive-only', style: 'gap:0.35em' },
        noiseB, objB),
      h('div', { class: 'wctl interactive-only', style: 'gap:0.35em' }, h('label', {}, 'blend'), featherB, lastB)),
    h('div', { style: 'width:15em; flex:none' }, readout, relief)));

  rebuild();
  drawAll();
  // Print: replace the canvases by still images and free them (large 2D canvases next to other figures stall the
  // headless print renderer, as in stitch.js).
  if (print) {
    for (const c of [mapC, photoC, mosC]) {
      const flat = document.createElement('canvas'); flat.width = c.width; flat.height = c.height;
      const fg = flat.getContext('2d'); fg.fillStyle = C.bg2; fg.fillRect(0, 0, flat.width, flat.height); fg.drawImage(c, 0, 0);
      const img = new Image();
      img.src = flat.toDataURL('image/jpeg', 0.9);
      flat.width = flat.height = 0;
      img.style.cssText = c.style.cssText + `; width:${c.clientWidth || c.width}px; height:${c.clientHeight || c.height}px; display:block; margin:0`;
      img.className = c.className;
      await img.decode().catch(() => {});
      c.replaceWith(img);
      c.width = c.height = 0;
    }
    zoomC.width = zoomC.height = 0; ov.width = ov.height = 0;
  }
  if (print) { /* configured state, controls hidden by CSS */ }
}
