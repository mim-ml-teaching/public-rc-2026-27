// Ray-cast synthetic scene shared by the stereo and homography widgets (stereomatch, parallax).
// World frame (metres): x right, y down, z forward; the reference camera sits at the origin looking along +z.
// The scene: textured ground (plane y = 1), a brick back wall (plane z = 20) with a uniform (textureless) board,
// a box, a sphere and a panel with periodic stripes (z = 5). Textures live in object space, so all views agree.
//
// renderView({ R, c, f, W = 320, H = 240, ox = W / 2, oy = H / 2 }):
//   R: 3×3 world → camera rotation (array of rows), c: camera centre in world coordinates,
//   camera frame x right, y down, z forward; pixel (u, v) looks along Rᵀ K⁻¹ (u + ½, v + ½, 1).
//   Returns { rgba: Uint8ClampedArray RGBA, grey: Float32Array, z: Float32Array (depth z_c, Infinity = no hit) }.

export const WALL_Z = 20;
const WS = WALL_Z / 8;   // back wall depth; texture scale relative to the original z = 8
export const GROUND_Y = 1.0;

// ---------------- procedural textures (object space, so both views agree) ----------------
function hash3(i, j, k) {
  let n = (i * 374761393 + j * 668265263 + k * 1274126177) | 0;
  n = Math.imul(n ^ (n >>> 13), 1274126177);
  return ((n ^ (n >>> 16)) >>> 0) / 4294967296;
}
const smooth = t => t * t * (3 - 2 * t);
function vnoise(x, y, z) {
  const i = Math.floor(x), j = Math.floor(y), k = Math.floor(z);
  const fx = smooth(x - i), fy = smooth(y - j), fz = smooth(z - k);
  const l = (a, b, t) => a + (b - a) * t;
  const c = (di, dj, dk) => hash3(i + di, j + dj, k + dk);
  return l(l(l(c(0, 0, 0), c(1, 0, 0), fx), l(c(0, 1, 0), c(1, 1, 0), fx), fy),
    l(l(c(0, 0, 1), c(1, 0, 1), fx), l(c(0, 1, 1), c(1, 1, 1), fx), fy), fz);
}
function fbm(x, y, z, oct = 3) {
  let s = 0, a = 0.5, f = 1;
  for (let o = 0; o < oct; o++) { s += a * vnoise(x * f, y * f, z * f); a *= 0.5; f *= 2.03; }
  return s / (1 - Math.pow(0.5, oct));
}

const MAT = {
  ground: p => {
    const n = fbm(p[0] * 7, 0, p[2] * 7), d = hash3(Math.floor(p[0] * 18), 7, Math.floor(p[2] * 18));
    const k = 0.45 + 0.7 * n + (d > 0.93 ? 0.35 : 0);
    return [0.36 * k, 0.40 * k, 0.30 * k];
  },
  wall: p0 => {
    // texture designed for a wall at z = 8, scaled with the distance so it looks the same in the image
    const p = [p0[0] / WS, p0[1] / WS];
    // uniform board: no texture at all
    if (p[0] > -3.4 && p[0] < -0.9 && p[1] > -2.3 && p[1] < -0.5) return [0.86, 0.86, 0.83];
    const row = Math.floor(p[1] / 0.25), off = row & 1 ? 0.25 : 0;
    const col = Math.floor((p[0] + off) / 0.5);
    const fx = (p[0] + off) / 0.5 - col, fy = p[1] / 0.25 - row;
    if (fx < 0.06 || fy < 0.12) return [0.72, 0.70, 0.66];   // mortar
    const v = 0.6 + 0.5 * hash3(col, row, 3) + 0.35 * (fbm(p[0] * 12, p[1] * 12, 1) - 0.5);
    return [0.62 * v, 0.33 * v, 0.24 * v];
  },
  stripes: p => (Math.floor(p[0] / 0.075) & 1 ? [0.15, 0.17, 0.22] : [0.85, 0.85, 0.8]),
  box: p => {
    const c = (Math.floor(p[0] * 9) + Math.floor(p[1] * 9) + Math.floor(p[2] * 9)) & 1;
    const n = 0.7 + 0.6 * fbm(p[0] * 20, p[1] * 20, p[2] * 20);
    return c ? [0.25 * n, 0.45 * n, 0.75 * n] : [0.85 * n, 0.8 * n, 0.55 * n];
  },
  sphere: p => {
    const n = fbm(p[0] * 9 + 5, p[1] * 9, p[2] * 9, 4);
    const d = hash3(Math.floor(p[0] * 25), Math.floor(p[1] * 25), Math.floor(p[2] * 25)) > 0.85 ? 0.5 : 0;
    const k = 0.35 + 0.9 * n + d;
    return [0.95 * k, 0.55 * k, 0.25 * k];
  },
};

// ---------------- scene and ray casting ----------------
// axis-aligned rectangles: { axis, at, lo:[..], hi:[..] } — plane coordinate[axis] = at, bounds on the other two
// The back wall is far (z = 20) so that it barely moves between the views, while the sphere and the box jump.
export const RECTS = [
  { axis: 1, at: GROUND_Y, lo: [-8 * WS, 0, 0.5], hi: [8 * WS, 0, WALL_Z], n: [0, -1, 0], mat: 'ground' },
  { axis: 2, at: WALL_Z, lo: [-8 * WS, -6 * WS, 0], hi: [8 * WS, 1, 0], n: [0, 0, -1], mat: 'wall' },
  { axis: 2, at: 5.0, lo: [0.9, -0.3, 0], hi: [2.4, 1, 0], n: [0, 0, -1], mat: 'stripes' },
];
const BOX = { lo: [-1.7, 0.2, 3.0], hi: [-0.8, 1.0, 3.7], mat: 'box' };
const SPHERE = { c: [0.25, 0.55, 2.8], r: 0.45, mat: 'sphere' };
const LIGHTDIR = (() => { const v = [-0.45, -1, -0.6], n = Math.hypot(...v); return v.map(x => x / n); })();

function cast(o, d) {
  let best = null;
  const hit = (t, n, mat) => { if (t > 1e-4 && (!best || t < best.t)) best = { t, n, mat }; };
  for (const r of RECTS) {
    if (Math.abs(d[r.axis]) < 1e-9) continue;
    const t = (r.at - o[r.axis]) / d[r.axis];
    let ok = true;
    for (let a = 0; a < 3; a++) if (a !== r.axis) { const v = o[a] + t * d[a]; if (v < r.lo[a] || v > r.hi[a]) ok = false; }
    if (ok) hit(t, r.n, r.mat);
  }
  // box (slabs)
  let t0 = -Infinity, t1 = Infinity, ax = -1, sg = 0;
  for (let a = 0; a < 3; a++) {
    const inv = 1 / d[a];
    let ta = (BOX.lo[a] - o[a]) * inv, tb = (BOX.hi[a] - o[a]) * inv, s = -1;
    if (ta > tb) { [ta, tb] = [tb, ta]; s = 1; }
    if (ta > t0) { t0 = ta; ax = a; sg = s; }
    t1 = Math.min(t1, tb);
  }
  if (t0 <= t1 && t0 > 0) { const n = [0, 0, 0]; n[ax] = sg; hit(t0, n, BOX.mat); }
  // sphere
  const oc = [o[0] - SPHERE.c[0], o[1] - SPHERE.c[1], o[2] - SPHERE.c[2]];
  const A = d[0] * d[0] + d[1] * d[1] + d[2] * d[2], B = oc[0] * d[0] + oc[1] * d[1] + oc[2] * d[2];
  const D = B * B - A * (oc[0] * oc[0] + oc[1] * oc[1] + oc[2] * oc[2] - SPHERE.r * SPHERE.r);
  if (D >= 0) {
    const t = (-B - Math.sqrt(D)) / A;
    const p = [o[0] + t * d[0], o[1] + t * d[1], o[2] + t * d[2]];
    hit(t, [(p[0] - SPHERE.c[0]) / SPHERE.r, (p[1] - SPHERE.c[1]) / SPHERE.r, (p[2] - SPHERE.c[2]) / SPHERE.r], SPHERE.mat);
  }
  return best;
}

export function renderView({ R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], c = [0, 0, 0], f = 300, W = 320, H = 240, ox = W / 2, oy = H / 2 } = {}) {
  const rgba = new Uint8ClampedArray(W * H * 4), grey = new Float32Array(W * H), zb = new Float32Array(W * H);
  const o = c.slice();
  for (let v = 0; v < H; v++) for (let u = 0; u < W; u++) {
    const dc = [(u + 0.5 - ox) / f, (v + 0.5 - oy) / f, 1];
    // world direction Rᵀ dc
    const d = [0, 1, 2].map(k => R[0][k] * dc[0] + R[1][k] * dc[1] + R[2][k] * dc[2]);
    const hit = cast(o, d), i = v * W + u;
    let col = [0.1, 0.1, 0.12], z = Infinity;
    if (hit) {
      const p = [o[0] + hit.t * d[0], o[1] + hit.t * d[1], o[2] + hit.t * d[2]];
      const alb = MAT[hit.mat](p);
      const k = 0.4 + 0.6 * Math.max(0, hit.n[0] * LIGHTDIR[0] + hit.n[1] * LIGHTDIR[1] + hit.n[2] * LIGHTDIR[2]);
      col = alb.map(x => Math.min(1, x * k));
      z = R[2][0] * (p[0] - o[0]) + R[2][1] * (p[1] - o[1]) + R[2][2] * (p[2] - o[2]);
    }
    rgba[4 * i] = col[0] * 255; rgba[4 * i + 1] = col[1] * 255; rgba[4 * i + 2] = col[2] * 255; rgba[4 * i + 3] = 255;
    grey[i] = 255 * (0.299 * col[0] + 0.587 * col[1] + 0.114 * col[2]);
    zb[i] = z;
  }
  return { rgba, grey, z: zb };
}
