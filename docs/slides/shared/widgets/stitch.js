// Image stitching: matches → RANSAC homographies → warping → blending.
// The views (480 × 360, f = 450 px) are rendered from a real 360° photo (panoenv.js, Poly Haven "Canary Wharf", CC0)
// by a camera that only ROTATES (yaw −30°, 0°, +30°, small pitch/roll), so every pair of views is related exactly by
// H = K R_j R_iᵀ K⁻¹. Each view gets its own exposure gain and vignetting, so seams are visible without blending.
// The photo loads asynchronously: mount() returns a promise that resolves when the figure is drawn.
// Frames: world = reference camera frame (x right, y down, z forward); R_i maps world → camera i.
//
// Steps (buttons):
//   images   the views (coloured frames = the colours of the warped borders later)
//   matches  correspondences between a pair of neighbouring views: textured pixels (sampled ∝ gradient magnitude, as a
//            detector would) projected into the other view + noise (σ = 0.5 px), plus wrong matches (random pixels)
//   RANSAC   homography by RANSAC (s = 4, DLT on Hartley-normalised points: A (2n×9) h = 0, h = eigenvector of the
//            smallest eigenvalue of AᵀA), threshold on the transfer error, refit on the inliers;
//            inliers green, outliers red; readout: estimated H, inliers, iterations, error vs the true H
//   warp     all views backward-warped into the frame of the middle view (H chained between neighbours),
//            canvas sized from the warped corners, warped borders drawn; later views overwrite earlier ones
//   blend    blending modes: none (overwrite: hard seams), average, feather (weights = distance to the image
//            border, i.e. a distance transform, normalised per pixel)
// (The views also share an upward tilt of 12°; see panoenv.js.)
//
// config: {
//   width: 820, height: 400   main canvas (side panel 15em on the right)
//   step: "blend"             initial step: "images" | "matches" | "ransac" | "warp" | "blend"
//   blend: "feather"          initial blending: "none" | "average" | "feather"
//   pair: 1                   pair shown in matches/RANSAC: index i means views i and i+1 (0-based); default: the
//                             pair right of the middle view
//   views: 3                  3 (yaw −30, 0, +30°) or 5 (yaw −56 … +56°: the planar panorama stretches, "bow tie")
//   projection: "planar"      "planar" (the frame of the middle view) or "cylindrical" (angle × height)
//   projButton: false         show buttons to switch planar / cylindrical
//   matches: 50               matches per pair;  outliers: 0.3 fraction of wrong matches
//   threshold: 3              RANSAC inlier threshold, px;  seed: 3
//   steps: [...]              which step buttons to show (default: all five)
// }
// Default state: step "blend", blending "feather"; print shows the configured step and blending.
import { C, h, fmt, isPrint, mul3, inv3, apply3 } from './util.js';
import { symEig, rng } from './linalg.js';
import { IW, IH, F, OX, OY, K, Ki, makeViews, loadEnv, renderView } from './panoenv.js';

const VIEW_COL = [C.red, C.green, C.blue, C.purple, C.accent];

// ---------------- homography estimation ----------------
function normT(pts) {
  let cx = 0, cy = 0;
  pts.forEach(p => { cx += p[0]; cy += p[1]; });
  cx /= pts.length; cy /= pts.length;
  let md = 0; pts.forEach(p => { md += Math.hypot(p[0] - cx, p[1] - cy); });
  const s = Math.SQRT2 / (md / pts.length || 1);
  return [s, 0, -s * cx, 0, s, -s * cy, 0, 0, 1];
}
// DLT: H with b ~ H a
function dlt(a, b) {
  const Ta = normT(a), Tb = normT(b);
  const M = Array.from({ length: 9 }, () => new Array(9).fill(0));
  const add = r => { for (let i = 0; i < 9; i++) if (r[i]) for (let j = 0; j < 9; j++) M[i][j] += r[i] * r[j]; };
  for (let k = 0; k < a.length; k++) {
    const [x, y] = apply3(Ta, a[k][0], a[k][1]), [X, Y] = apply3(Tb, b[k][0], b[k][1]);
    add([-x, -y, -1, 0, 0, 0, X * x, X * y, X]);
    add([0, 0, 0, -x, -y, -1, Y * x, Y * y, Y]);
  }
  const hn = symEig(M).vecs[0];
  const Tbi = inv3(Tb);
  let H = mul3(Tbi, mul3(hn, Ta));
  if (Math.abs(H[8]) < 1e-12) return null;
  return H.map(v => v / H[8]);
}
const transferErr = (H, a, b) => { const p = apply3(H, a[0], a[1]); return p[2] > 0 ? Math.hypot(p[0] - b[0], p[1] - b[1]) : Infinity; };

function ransac(m, thr, seed) {
  const r = rng(seed), n = m.length, a = m.map(o => o.a), b = m.map(o => o.b);
  let best = null, N = 1000, k = 0;
  while (k < N && k < 1000) {
    k++;
    const idx = [];
    while (idx.length < 4) { const i = Math.floor(r() * n); if (!idx.includes(i)) idx.push(i); }
    const H = dlt(idx.map(i => a[i]), idx.map(i => b[i]));
    if (!H) continue;
    const inl = m.map((_, i) => transferErr(H, a[i], b[i]) < thr);
    const cnt = inl.filter(Boolean).length;
    if (!best || cnt > best.cnt) {
      best = { H, inl, cnt, sample: idx };
      const w = cnt / n;   // adaptive N; with no inliers yet keep the maximum
      if (w > 0) N = w >= 1 ? k : Math.min(1000, Math.ceil(Math.log(0.01) / Math.log(1 - Math.pow(w, 4))));
    }
  }
  const ia = a.filter((_, i) => best.inl[i]), ib = b.filter((_, i) => best.inl[i]);
  const H = dlt(ia, ib) || best.H;
  const inl = m.map((_, i) => transferErr(H, a[i], b[i]) < thr);
  return { H, inl, cnt: inl.filter(Boolean).length, iters: k, N };
}

// ---------------- widget ----------------
export async function mount(el, cfg) {
  const gen = el.dataset.gen;
  el.classList.add('widget');
  const loading = h('div', { class: 'readout', style: `width:${cfg.width || 820}px; height:${cfg.height || 400}px; display:flex; align-items:center; justify-content:center; background:${C.bg2}; border:1px solid ${C.line}; border-radius:8px` }, 'loading the panorama…');
  el.append(loading);
  const env = await loadEnv();
  if (el.dataset.gen !== gen) return; // unmounted while loading
  loading.remove();
  const W = cfg.width || 820, H = cfg.height || 400;
  const nViews = cfg.views === 5 ? 5 : 3, ref = (nViews - 1) / 2;
  const thr = cfg.threshold ?? 3, nMatch = cfg.matches ?? 50, outFrac = cfg.outliers ?? 0.3;
  const state = {
    step: cfg.step || 'blend',
    blend: cfg.blend || 'feather',
    pair: cfg.pair ?? ref, proj: cfg.projection === 'cylindrical' ? 'cyl' : 'planar', seed: cfg.seed ?? 3,
  };
  const views = makeViews(nViews);
  views.forEach(v => { v.img = renderView(env, v); });
  // gradient magnitude of each view: matches are drawn at textured pixels, like detected keypoints
  views.forEach(v => {
    const gm = new Float32Array(IW * IH), im = v.img;
    const L = i => 0.299 * im[4 * i] + 0.587 * im[4 * i + 1] + 0.114 * im[4 * i + 2];
    let mx = 0;
    for (let y = 1; y < IH - 1; y++) for (let x = 1; x < IW - 1; x++) {
      const i = y * IW + x, gx = L(i + 1) - L(i - 1), gy = L(i + IW) - L(i - IW);
      gm[i] = Math.hypot(gx, gy); if (gm[i] > mx) mx = gm[i];
    }
    v.grad = gm; v.gmax = mx || 1;
  });
  const Htrue = (i, j) => mul3(K, mul3(views[j].R, mul3(views[i].Rcw, Ki))); // x_j ~ H x_i
  const norm9 = m => m.map(v => v / m[8]);

  // matches and RANSAC for each neighbouring pair (i, i+1): estimate H mapping i → i+1
  let pairs = [];
  // mean distance between H x and H_true x over a grid of pixels x (only those that land inside the other view)
  function overlapErr(He, Ht, inside = true) {
    let s = 0, n = 0;
    for (let y = 10; y < IH; y += 20) for (let x = 10; x < IW; x += 20) {
      const q = apply3(Ht, x, y);
      if (q[2] <= 0 || (inside && (q[0] < 0 || q[0] > IW || q[1] < 0 || q[1] > IH))) continue;
      const p = apply3(He, x, y);
      if (inside) s += Math.hypot(p[0] - q[0], p[1] - q[1]);
      else { const b = apply3(inv3(Ht), p[0], p[1]); s += Math.hypot(b[0] - x, b[1] - y); } // in pixels of the source view
      n++;
    }
    return n ? s / n : 0;
  }
  function computeMatches() {
    const r = rng(state.seed * 7919 + 13);
    pairs = [];
    for (let i = 0; i + 1 < nViews; i++) {
      const Ht = Htrue(i, i + 1), m = [];
      const nOut = Math.round(nMatch * outFrac);
      let guard = 0;
      while (m.length < nMatch - nOut && guard++ < 5000) {
        const p = [10 + r() * (IW - 20), 10 + r() * (IH - 20)], q = apply3(Ht, p[0], p[1]);
        if (q[2] <= 0 || q[0] < 8 || q[0] > IW - 8 || q[1] < 8 || q[1] > IH - 8) continue;
        const gi = Math.floor(p[1]) * IW + Math.floor(p[0]);
        if (r() > Math.pow(views[i].grad[gi] / views[i].gmax, 0.7) * 2.5) continue; // prefer textured pixels
        m.push({ a: [p[0] + 0.5 * r.normal(), p[1] + 0.5 * r.normal()], b: [q[0] + 0.5 * r.normal(), q[1] + 0.5 * r.normal()], good: true });
      }
      for (let k = 0; k < nOut; k++) {
        // wrong matches: a pixel in the overlap of view i, matched to a random pixel of view i+1
        let p, q, g2 = 0;
        do { p = [10 + r() * (IW - 20), 10 + r() * (IH - 20)]; q = apply3(Ht, p[0], p[1]); }
        while ((q[0] < 0 || q[0] > IW || q[1] < 0 || q[1] > IH) && g2++ < 200);
        m.push({ a: p, b: [10 + r() * (IW - 20), 10 + r() * (IH - 20)], good: false });
      }
      // shuffle
      for (let k = m.length - 1; k > 0; k--) { const j = Math.floor(r() * (k + 1)); [m[k], m[j]] = [m[j], m[k]]; }
      const rs = ransac(m, thr, state.seed * 31 + i);
      pairs.push({ i, m, rs, Ht, err: overlapErr(rs.H, Ht) });
    }
    // chain to the reference view: Hs[k] maps view k → view ref
    const Hs = new Array(nViews);
    Hs[ref] = [1, 0, 0, 0, 1, 0, 0, 0, 1];
    for (let k = ref + 1; k < nViews; k++) Hs[k] = norm9(mul3(Hs[k - 1], inv3(pairs[k - 1].rs.H)));
    for (let k = ref - 1; k >= 0; k--) Hs[k] = norm9(mul3(Hs[k + 1], pairs[k].rs.H));
    views.forEach((v, k) => {
      v.H = Hs[k]; v.Hi = inv3(Hs[k]);
      const Ht = Htrue(k, ref);
      v.err = overlapErr(v.H, Ht, false);
    });
    pano = null;
  }

  // ---------------- panorama ----------------
  // pano coordinates: planar = pixels of the reference view; cylindrical = (f·angle, f·height) about the ref axis
  const fromPano = (X, Y) => state.proj === 'planar' ? [X, Y, 1]
    : (() => { const t = X / F, c = Math.cos(t), s = Math.sin(t); return [F * s + OX * c, Y + OY * c, c]; })();
  function toPano(v, x, y) {
    const H3 = v.H, q = [H3[0] * x + H3[1] * y + H3[2], H3[3] * x + H3[4] * y + H3[5], H3[6] * x + H3[7] * y + H3[8]];
    if (state.proj === 'planar') return q[2] > 1e-6 ? [q[0] / q[2], q[1] / q[2]] : null;
    const d = [Ki[0] * q[0] + Ki[1] * q[1] + Ki[2] * q[2], Ki[4] * q[1] + Ki[5] * q[2], q[2]];
    return [F * Math.atan2(d[0], d[2]), F * d[1] / Math.hypot(d[0], d[2])];
  }
  function border(v, nPer = 24) {
    const pts = [];
    const add = (x, y) => { const p = toPano(v, x, y); if (p) pts.push(p); };
    for (let k = 0; k < nPer; k++) add(IW * k / nPer, 0);
    for (let k = 0; k < nPer; k++) add(IW, IH * k / nPer);
    for (let k = 0; k < nPer; k++) add(IW - IW * k / nPer, IH);
    for (let k = 0; k < nPer; k++) add(0, IH - IH * k / nPer);
    return pts;
  }
  let pano = null; // { s, x0, y0, borders, imgs: {mode: ImageData} }
  function layout() {
    const bs = views.map(v => border(v));
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    const LIM = 6 * F;
    bs.flat().forEach(([x, y]) => {
      x = Math.max(-LIM, Math.min(LIM + IW, x)); y = Math.max(-LIM, Math.min(LIM + IH, y));
      x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y);
    });
    const pad = 6, s = Math.min((W - 2 * pad) / (x1 - x0), (H - 2 * pad) / (y1 - y0));
    const ox = (W - s * (x1 - x0)) / 2, oy = (H - s * (y1 - y0)) / 2;
    pano = { s, x0, y0, ox, oy, borders: bs, imgs: {} };
  }
  const feather = (u, v) => Math.max(0, Math.min(u, IW - u, v, IH - v)) / OY;
  function panoImage(mode) {
    if (!pano) layout();
    if (pano.imgs[mode]) return pano.imgs[mode];
    const PW = W * DPR, PH = H * DPR;
    const img = new ImageData(PW, PH), d = img.data, { s, x0, y0, ox, oy } = pano;
    for (let py = 0; py < PH; py++) for (let px = 0; px < PW; px++) {
      const X = ((px + 0.5) / DPR - ox) / s + x0, Y = ((py + 0.5) / DPR - oy) / s + y0;
      const q = fromPano(X, Y);
      let r = 0, g = 0, b = 0, ws = 0;
      for (let k = 0; k < nViews; k++) {
        const Hi = views[k].Hi;
        const w3 = Hi[6] * q[0] + Hi[7] * q[1] + Hi[8] * q[2];
        if (w3 <= 1e-9) continue;
        const u = (Hi[0] * q[0] + Hi[1] * q[1] + Hi[2] * q[2]) / w3, v = (Hi[3] * q[0] + Hi[4] * q[1] + Hi[5] * q[2]) / w3;
        if (u < 0 || v < 0 || u > IW || v > IH) continue;
        // bilinear sample
        const fx = Math.min(IW - 1.001, Math.max(0, u - 0.5)), fy = Math.min(IH - 1.001, Math.max(0, v - 0.5));
        const ix = Math.floor(fx), iy = Math.floor(fy), ax = fx - ix, ay = fy - iy, src = views[k].img;
        const i00 = 4 * (iy * IW + ix), i10 = i00 + 4, i01 = i00 + 4 * IW, i11 = i01 + 4;
        const sm = c => (src[i00 + c] * (1 - ax) + src[i10 + c] * ax) * (1 - ay) + (src[i01 + c] * (1 - ax) + src[i11 + c] * ax) * ay;
        const wt = mode === 'feather' ? feather(u, v) + 1e-6 : 1;
        if (mode === 'none') { r = sm(0); g = sm(1); b = sm(2); ws = 1; }
        else { r += wt * sm(0); g += wt * sm(1); b += wt * sm(2); ws += wt; }
      }
      if (ws > 0) { const i = 4 * (py * PW + px); d[i] = r / ws; d[i + 1] = g / ws; d[i + 2] = b / ws; d[i + 3] = 255; }
    }
    pano.imgs[mode] = img;
    return img;
  }

  // ---------------- drawing ----------------
  // drawn at 2× (or the screen's) pixel density: the photo has finer detail than one CSS pixel
  const DPR = Math.max(2, Math.min(3, window.devicePixelRatio || 1));
  const canvas = h('canvas', { width: W * DPR, height: H * DPR, style: `width:${W}px; height:${H}px` });
  const g = canvas.getContext('2d');
  g.setTransform(DPR, 0, 0, DPR, 0, 0); g.imageSmoothingQuality = 'high';
  const viewCanvases = views.map(v => {
    const c = h('canvas', { width: IW, height: IH });
    c.getContext('2d').putImageData(new ImageData(v.img, IW, IH), 0, 0);
    return c;
  });
  const panel = h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:15em; flex:none; line-height:1.4' });
  const inset = h('canvas', { width: 160, height: 120, style: 'width:160px; height:120px' });
  (() => { // feather weight of one image (white = 1)
    const ig = inset.getContext('2d'), im = ig.createImageData(160, 120);
    for (let y = 0; y < 120; y++) for (let x = 0; x < 160; x++) {
      const w = feather((x + 0.5) * 2, (y + 0.5) * 2), i = 4 * (y * 160 + x), c = 30 + 225 * w;
      im.data[i] = im.data[i + 1] = im.data[i + 2] = c; im.data[i + 3] = 255;
    }
    ig.putImageData(im, 0, 0);
  })();

  function frame(x, y, w, hh, col, lw = 3) { g.strokeStyle = col; g.lineWidth = lw; g.strokeRect(x, y, w, hh); }
  function bg() { g.fillStyle = C.bg2; g.fillRect(0, 0, W, H); }
  const viewName = k => `view ${k + 1}`;

  function drawImages() {
    bg();
    const cols = nViews, gap = 10, s = Math.min((W - gap * (cols + 1)) / (cols * IW), (H - 60) / IH);
    const w = IW * s, hh = IH * s, x0 = (W - cols * w - (cols - 1) * gap) / 2, y0 = (H - hh) / 2;
    views.forEach((v, k) => {
      const x = x0 + k * (w + gap);
      g.drawImage(viewCanvases[k], x, y0, w, hh);
      frame(x, y0, w, hh, VIEW_COL[k]);
      g.fillStyle = C.fg; g.font = '18px Inter, Arial, sans-serif'; g.textAlign = 'center';
      g.fillText(`${viewName(k)}: yaw ${v.yaw > 0 ? '+' : ''}${v.yaw}°`, x + w / 2, y0 + hh + 24);
    });
    g.textAlign = 'left';
  }
  function pairGeom() {
    const gap = 20, s = Math.min((W - gap) / (2 * IW), H / IH);
    const w = IW * s, hh = IH * s, x0 = (W - 2 * w - gap) / 2, y0 = (H - hh) / 2;
    return { s, w, hh, xa: x0, xb: x0 + w + gap, y0 };
  }
  function drawPair(showRansac) {
    bg();
    const P = pairs[state.pair], { s, w, hh, xa, xb, y0 } = pairGeom();
    g.drawImage(viewCanvases[P.i], xa, y0, w, hh); frame(xa, y0, w, hh, VIEW_COL[P.i]);
    g.drawImage(viewCanvases[P.i + 1], xb, y0, w, hh); frame(xb, y0, w, hh, VIEW_COL[P.i + 1]);
    const lines = (filter, col, lw, dash) => {
      g.strokeStyle = col; g.lineWidth = lw; g.setLineDash(dash);
      P.m.forEach((o, k) => {
        if (!filter(k)) return;
        g.beginPath(); g.moveTo(xa + o.a[0] * s, y0 + o.a[1] * s); g.lineTo(xb + o.b[0] * s, y0 + o.b[1] * s); g.stroke();
      });
      g.setLineDash([]);
      P.m.forEach((o, k) => {
        if (!filter(k)) return;
        g.fillStyle = col;
        for (const [x, y] of [[xa + o.a[0] * s, y0 + o.a[1] * s], [xb + o.b[0] * s, y0 + o.b[1] * s]]) {
          g.beginPath(); g.arc(x, y, 4, 0, 7); g.fill();
        }
      });
    };
    if (!showRansac) lines(() => true, C.accent, 1.6, []);
    else {
      lines(k => !P.rs.inl[k], C.red, 1.3, [5, 4]);
      lines(k => P.rs.inl[k], C.green, 1.8, []);
    }
    g.font = '18px Inter, Arial, sans-serif'; g.fillStyle = C.fg; g.textAlign = 'center';
    g.fillText(viewName(P.i), xa + w / 2, y0 - 8 > 14 ? y0 - 8 : y0 + hh + 22);
    g.fillText(viewName(P.i + 1), xb + w / 2, y0 - 8 > 14 ? y0 - 8 : y0 + hh + 22);
    g.textAlign = 'left';
  }
  function drawPano(mode, borders) {
    bg();
    const img = panoImage(mode);
    g.putImageData(img, 0, 0);   // device pixels, same size as the canvas
    if (borders) {
      const { s, x0, y0, ox, oy } = pano;
      pano.borders.forEach((b, k) => {
        g.strokeStyle = VIEW_COL[k]; g.lineWidth = 2.5; g.beginPath();
        b.forEach(([x, y], j) => { const X = ox + (x - x0) * s, Y = oy + (y - y0) * s; j ? g.lineTo(X, Y) : g.moveTo(X, Y); });
        g.closePath(); g.stroke();
      });
    }
  }

  const mat = Hm => {
    const r = Hm.map(v => (Math.abs(v) < 0.0095 && v !== 0 ? v.toExponential(1) : fmt(v, 2)));
    return `<div class="readout" style="display:grid; grid-template-columns:repeat(3, 4.6em); text-align:right; gap:0 0.3em; font-size:0.85em; color:${C.fg}">` +
      r.map(t => `<span>${t}</span>`).join('') + '</div>';
  };
  const sub = (a, b) => `_{${a}\\to ${b}}`;

  function drawPanel() {
    const st = state.step, P = pairs[state.pair];
    let html = '';
    if (st === 'images') {
      html = `<div><b>${nViews} views from one centre</b>: the camera only rotates (yaw ${views.map(v => (v.yaw > 0 ? '+' : '') + v.yaw + '°').join(', ')}).</div>` +
        `<div class="dim">Each view has its own exposure and vignetting (darker corners), like real photos.</div>`;
    } else if (st === 'matches') {
      const bad = P.m.filter(o => !o.good).length;
      html = `<div><b>${P.m.length} matches</b> between ${viewName(P.i)} and ${viewName(P.i + 1)}.</div>` +
        `<div class="dim">From a feature matcher: most are right, but ${bad} (${Math.round(100 * bad / P.m.length)}%) are wrong. Which ones?</div>`;
    } else if (st === 'ransac') {
      html = `<div>RANSAC, $s = 4$, threshold ${thr} px:</div>` +
        `<div><span style="color:${C.green}">inliers</span> <b>${P.rs.cnt}</b> / ${P.m.length}, <span style="color:${C.red}">outliers</span> ${P.m.length - P.rs.cnt}</div>` +
        `<div class="dim">${P.rs.iters} iterations (adaptive $N$)</div>` +
        `<div style="margin-top:0.3em">$H${sub(P.i + 1, P.i + 2)}$ (refit on the inliers):</div>${mat(P.rs.H)}` +
        `<div style="margin-top:0.3em">true $KR_jR_i^\\top K^{-1}$:</div>${mat(norm9(P.Ht))}` +
        `<div class="dim">mean error in the overlap: ${fmt(P.err, 2)} px</div>`;
    } else {
      const errs = views.map((v, k) => k === ref ? '' : `<div style="color:${VIEW_COL[k]}">$H${sub(k + 1, ref + 1)}$: ${fmt(v.err, 2)} px</div>`).join('');
      const projTxt = state.proj === 'planar' ? `the image plane of ${viewName(ref)}` : 'a cylinder around the camera (angle × height)';
      if (st === 'warp') {
        html = `<div>Every view is <b>backward-warped</b> onto ${projTxt}, with $H$ chained between neighbours.</div>` +
          `<div class="dim">Later views overwrite earlier ones. Coloured: the warped borders.</div>` +
          `<div style="margin-top:0.3em">mean error vs the true $H$ (in source pixels):</div>${errs}`;
      } else {
        const txt = {
          none: 'Each pixel from one image (the last one): <b>hard seams</b> from exposure and vignetting.',
          average: 'Average of all images covering the pixel: seams soften, but steps remain where an image starts.',
          feather: 'Weighted average, weight = <b>distance to the image border</b> (a distance transform): smooth transitions.',
        }[state.blend];
        html = `<div>${txt}</div>`;
      }
    }
    panel.innerHTML = html;
    if (st === 'blend' && state.blend === 'feather') {
      panel.append(h('div', { class: 'dim', style: 'margin-top:0.3em' }, 'weight of one image:'), inset);
    }
    window.renderMathInElement?.(panel, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
  }

  function draw() {
    const st = state.step;
    if (st === 'images') drawImages();
    else if (st === 'matches') drawPair(false);
    else if (st === 'ransac') drawPair(true);
    else if (st === 'warp') drawPano('none', true);
    else drawPano(state.blend, false);
    stepBtns.forEach(([k, b]) => b.classList.toggle('active', k === st));
    blendBtns.forEach(([k, b]) => b.classList.toggle('active', k === state.blend));
    pairBtns.forEach(([k, b]) => b.classList.toggle('active', k === state.pair));
    projBtns.forEach(([k, b]) => b.classList.toggle('active', k === state.proj));
    blendRow.style.visibility = st === 'blend' ? 'visible' : 'hidden';
    pairRow.style.display = st === 'matches' || st === 'ransac' ? '' : 'none';
    projRow.style.display = (st === 'warp' || st === 'blend') && cfg.projButton ? '' : 'none';
    drawPanel();
  }

  // ---------------- controls ----------------
  const STEP_LABEL = { images: 'images', matches: 'matches', ransac: 'RANSAC', warp: 'warp', blend: 'blend' };
  const stepList = cfg.steps || Object.keys(STEP_LABEL);
  const stepBtns = stepList.map(k => [k, h('button', { onclick: () => { state.step = k; draw(); } }, STEP_LABEL[k])]);
  const blendBtns = ['none', 'average', 'feather'].map(k => [k, h('button', { onclick: () => { state.blend = k; draw(); } }, k)]);
  const pairBtns = pairs.length ? [] : Array.from({ length: nViews - 1 }, (_, i) =>
    [i, h('button', { onclick: () => { state.pair = i; draw(); } }, `${i + 1}–${i + 2}`)]);
  const projBtns = [['planar', 'planar'], ['cyl', 'cylindrical']].map(([k, t]) =>
    [k, h('button', { onclick: () => { if (state.proj !== k) { state.proj = k; pano = null; } draw(); } }, t)]);
  const newBtn = h('button', { onclick: () => { state.seed++; computeMatches(); draw(); } }, 'new matches');
  const blendRow = h('span', { style: 'display:inline-flex; gap:0.3em; align-items:center; margin-left:0.6em' },
    h('label', {}, 'blending'), ...blendBtns.map(b => b[1]));
  const pairRow = h('span', { style: 'display:inline-flex; gap:0.3em; align-items:center; margin-left:0.6em' },
    h('label', {}, 'pair'), ...pairBtns.map(b => b[1]), newBtn);
  const projRow = h('span', { style: 'display:inline-flex; gap:0.3em; align-items:center; margin-left:0.6em' },
    h('label', {}, 'surface'), ...projBtns.map(b => b[1]));

  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${W}px` }, canvas,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, ...stepBtns.map(b => b[1]), pairRow, blendRow, projRow)),
    panel));

  computeMatches();
  if (state.pair > pairs.length - 1) state.pair = pairs.length - 1;
  draw();
  // Print: keep a still image and free the canvases (several large canvases next to WebGL figures stall the
  // headless print renderer).
  if (isPrint()) {
    const flat = h('canvas', { width: canvas.width, height: canvas.height }), fg = flat.getContext('2d');
    fg.fillStyle = C.bg2; fg.fillRect(0, 0, flat.width, flat.height); fg.drawImage(canvas, 0, 0);  // JPEG has no alpha
    const img = new Image();
    img.src = flat.toDataURL('image/jpeg', 0.92);
    flat.width = flat.height = 0;
    img.style.cssText = `width:${W}px; height:${H}px; display:block; border-radius:8px; margin:0`;
    await img.decode().catch(() => {});
    canvas.replaceWith(img);
    [canvas, ...viewCanvases].forEach(c => { c.width = c.height = 0; });
  }
}
