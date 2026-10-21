// Drone mapping, animated: a drone flies a lawnmower pattern over a real orthophoto and takes nadir photos; each
// photo goes onto the map by its ground homography, so the mosaic grows as the drone flies.
// Ground: DOP20 orthophoto, Bayerische Vermessungsverwaltung (CC BY 4.0), a 500 m × 333 m crop at 0.26 m/px
// (assets/lecture03/drone_ortho.jpg).
// World frame as in aerial.js: X east, Y south (down on the map), Z down; ground Z = 0, drone at Z = −A.
// Camera frame (OpenCV): x right, y down, z forward (down). Heading ψ = flight direction (0: east, the photo looks like
// the map; westbound lines give photos turned by 180°). R_cw = Rz(ψ) Ry(pitch) Rx(roll) holds the camera axes in world
// coordinates; world → camera x_c = R (X − c), R = R_cwᵀ, t = −R c. A ground point (X, Y, 0) gives
// ũ ≡ K [r1 r2 t] (X, Y, 1)ᵀ: the ground homography H_i of photo i.
// Each photo is rendered from the orthophoto by intersecting every pixel's ray with the ground (2 × 2 supersampling),
// with its own exposure gain and some vignetting, so the blending has something to hide. The mosaic sends every map
// pixel through H_i into the photo (backward warping) and feathers (weight = distance to the photo border).
// The poses are exact (as after bundle adjustment); aerial.js shows what noisy GPS/IMU poses and 3D objects do.
// config: {
//   width: 720          mosaic width in px (the whole 500 m map)
//   photoWidth: 320     displayed width of the photo
//   info: false         readout with the flight numbers and H_i of the last photo
//   autoplay: true      fly when the slide is shown (never in print)
//   speed: 1            1 = about 18 s for the whole flight; buttons 1×, 2×, 4×
//   t: null             progress 0 … 1 (print default 0.6; disables autoplay)
// }
// Animation runs only while the slide is shown (section.present) and stops when the widget is unmounted.
import { C, h, fmt, isPrint } from './util.js';

const SRC = new URL('../../assets/lecture03/drone_ortho.jpg', import.meta.url).href;
const MAP_W = 500;                                   // metres; the height follows from the image (≈ 333 m)
const deg = Math.PI / 180;
const A = 150;                                       // altitude (m)
const PW = 400, PH = 300, F = 440, OX = PW / 2, OY = PH / 2;    // footprint at 150 m: 136 m × 102 m
const FRONT = 0.7, SIDE = 0.6;                       // overlaps
const SPEED = 28;                                    // m of path per second at 1×

let orthoPromise = null;
function loadOrtho() {
  if (!orthoPromise) orthoPromise = new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => {
      const c = document.createElement('canvas'); c.width = img.naturalWidth; c.height = img.naturalHeight;
      const g = c.getContext('2d', { willReadFrequently: true }); g.drawImage(img, 0, 0);
      resolve({ data: g.getImageData(0, 0, c.width, c.height).data, W: c.width, H: c.height, mpp: MAP_W / c.width });
    };
    img.onerror = () => reject(new Error('cannot load ' + SRC));
    img.src = SRC;
  });
  return orthoPromise;
}

function rng(seed) {
  let a = seed >>> 0;
  return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
}
const Rz = a => [Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a), 0, 0, 0, 1];
const Ry = a => [Math.cos(a), 0, Math.sin(a), 0, 1, 0, -Math.sin(a), 0, Math.cos(a)];
const Rx = a => [1, 0, 0, 0, Math.cos(a), -Math.sin(a), 0, Math.sin(a), Math.cos(a)];
function mul(a, b) {
  const r = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) r[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
  return r;
}
const T3 = m => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];

// ---------------- flight plan: lines along X, a half-circle turn between them ----------------
function plan(mapH) {
  const fw = PW * A / F, fh = PH * A / F;
  const sx = (1 - FRONT) * fw, sy = (1 - SIDE) * fh;
  const x0 = fw / 2 + 12, x1 = MAP_W - fw / 2 - 12, y0 = fh / 2 + 12, y1 = mapH - fh / 2 - 12;
  const nx = Math.floor((x1 - x0) / sx) + 1, ny = Math.floor((y1 - y0) / sy) + 1;
  const dx = (x1 - x0) / (nx - 1), dy = (y1 - y0) / (ny - 1);
  const r = rng(7), path = [], photos = [];
  let s = 0;
  const push = (x, y) => {
    if (path.length) { const p = path[path.length - 1]; s += Math.hypot(x - p.x, y - p.y); }
    path.push({ x, y, s });
  };
  for (let j = 0; j < ny; j++) {
    const y = y0 + j * dy, east = j % 2 === 0;
    const xa = east ? x0 : x1, xb = east ? x1 : x0, dir = east ? 1 : -1;
    for (let k = 0; k < nx; k++) {
      const x = xa + dir * k * dx;
      push(x, y);
      const n = () => 2 * r() - 1;
      photos.push({
        c: [x + 1.2 * n(), y + 1.2 * n(), -(A + 1.5 * n())], line: j,
        Rcw: mul(Rz((east ? 0 : 180) * deg + 4 * deg * n()), mul(Ry(2.5 * deg * n()), Rx(2.5 * deg * n()))),
        gain: 0.9 + 0.16 * r(), s,
      });
    }
    if (j < ny - 1) {   // half circle beyond the end of the line
      const cx = xb, cy = y + dy / 2, rad = dy / 2;
      for (let a = 1; a < 24; a++) {
        const th = -Math.PI / 2 + Math.PI * a / 24;
        push(cx + dir * rad * Math.cos(th), cy + rad * Math.sin(th));
      }
    }
  }
  return { path, photos, fw, fh, nx, ny, sx: dx, sy: dy, len: s };
}

function posAt(path, s) {
  let lo = 0, hi = path.length - 1;
  if (s <= 0) return { x: path[0].x, y: path[0].y, hd: Math.atan2(path[1].y - path[0].y, path[1].x - path[0].x) };
  if (s >= path[hi].s) { const a = path[hi - 1], b = path[hi]; return { x: b.x, y: b.y, hd: Math.atan2(b.y - a.y, b.x - a.x) }; }
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (path[m].s <= s) lo = m; else hi = m; }
  const a = path[lo], b = path[hi], u = (s - a.s) / (b.s - a.s || 1);
  return { x: a.x + u * (b.x - a.x), y: a.y + u * (b.y - a.y), hd: Math.atan2(b.y - a.y, b.x - a.x) };
}

// ground homography H_i = K [r1 r2 t] (flat, row-major), ground (X, Y) → photo pixel
function groundH(p) {
  const R = T3(p.Rcw), c = p.c;
  const t = [0, 1, 2].map(i => -(R[3 * i] * c[0] + R[3 * i + 1] * c[1] + R[3 * i + 2] * c[2]));
  const M = [R[0], R[1], t[0], R[3], R[4], t[1], R[6], R[7], t[2]];
  return mul([F, 0, OX, 0, F, OY, 0, 0, 1], M);
}
// photo pixel → ground point (ray from the camera centre to Z = 0)
function toGround(p, u, v) {
  const d = [(u - OX) / F, (v - OY) / F, 1], R = p.Rcw;
  const w = [R[0] * d[0] + R[1] * d[1] + R[2] * d[2], R[3] * d[0] + R[4] * d[1] + R[5] * d[2], R[6] * d[0] + R[7] * d[1] + R[8] * d[2]];
  const k = -p.c[2] / w[2];
  return [p.c[0] + k * w[0], p.c[1] + k * w[1]];
}

function sampleOrtho(o, X, Y, out) {
  let x = X / o.mpp - 0.5, y = Y / o.mpp - 0.5;
  if (x < 0 || y < 0 || x > o.W - 1.001 || y > o.H - 1.001) { out[0] = out[1] = out[2] = 0; return false; }
  const ix = Math.floor(x), iy = Math.floor(y), ax = x - ix, ay = y - iy, d = o.data;
  const i00 = 4 * (iy * o.W + ix), i10 = i00 + 4, i01 = i00 + 4 * o.W, i11 = i01 + 4;
  for (let c = 0; c < 3; c++)
    out[c] = (d[i00 + c] * (1 - ax) + d[i10 + c] * ax) * (1 - ay) + (d[i01 + c] * (1 - ax) + d[i11 + c] * ax) * ay;
  return true;
}

function renderPhoto(o, p) {
  const rgba = new Uint8ClampedArray(PW * PH * 4), s = [0, 0, 0], acc = [0, 0, 0];
  for (let v = 0; v < PH; v++) for (let u = 0; u < PW; u++) {
    acc[0] = acc[1] = acc[2] = 0;
    for (const [du, dv] of [[0.25, 0.25], [0.75, 0.25], [0.25, 0.75], [0.75, 0.75]]) {
      const [X, Y] = toGround(p, u + du, v + dv);
      sampleOrtho(o, X, Y, s);
      acc[0] += s[0]; acc[1] += s[1]; acc[2] += s[2];
    }
    const r2 = ((u - OX) ** 2 + (v - OY) ** 2) / (OX * OX + OY * OY);
    const g = 0.25 * p.gain * (1 - 0.28 * r2), i = 4 * (v * PW + u);
    rgba[i] = acc[0] * g; rgba[i + 1] = acc[1] * g; rgba[i + 2] = acc[2] * g; rgba[i + 3] = 255;
  }
  return rgba;
}

export async function mount(el, cfg) {
  const print = isPrint();
  const o = await loadOrtho();
  const mapH = o.H * o.mpp;
  const P = plan(mapH);
  const MW = cfg.width || 720, sc = MW / MAP_W, MH = Math.round(mapH * sc);
  const PWD = cfg.photoWidth || 320;
  const state = {
    s: (cfg.t ?? (print ? 0.6 : 0)) * P.len, speed: cfg.speed ?? 1, playing: false, added: 0, flash: 0, endWait: 0,
  };

  // ---- mosaic buffers ----
  const sum = new Float32Array(MW * MH * 3), wsum = new Float32Array(MW * MH);
  const mosaicImg = new ImageData(MW, MH);
  const photoCache = new Map();
  const photoOf = i => { if (!photoCache.has(i)) photoCache.set(i, renderPhoto(o, P.photos[i])); return photoCache.get(i); };
  function clearMosaic() { sum.fill(0); wsum.fill(0); mosaicImg.data.fill(0); state.added = 0; photoCache.clear(); }
  function addPhoto(i) {
    const p = P.photos[i], img = photoOf(i), H = groundH(p);
    const q = [[0, 0], [PW, 0], [PW, PH], [0, PH]].map(([u, v]) => toGround(p, u, v));
    const xa = Math.max(0, Math.floor(Math.min(...q.map(a => a[0])) * sc)), xb = Math.min(MW - 1, Math.ceil(Math.max(...q.map(a => a[0])) * sc));
    const ya = Math.max(0, Math.floor(Math.min(...q.map(a => a[1])) * sc)), yb = Math.min(MH - 1, Math.ceil(Math.max(...q.map(a => a[1])) * sc));
    const md = mosaicImg.data;
    for (let y = ya; y <= yb; y++) for (let x = xa; x <= xb; x++) {
      const X = (x + 0.5) / sc, Y = (y + 0.5) / sc;
      const w3 = H[6] * X + H[7] * Y + H[8];
      const u = (H[0] * X + H[1] * Y + H[2]) / w3 - 0.5, v = (H[3] * X + H[4] * Y + H[5]) / w3 - 0.5;
      if (u < 0 || v < 0 || u > PW - 1.001 || v > PH - 1.001) continue;
      const iu = Math.floor(u), iv = Math.floor(v), au = u - iu, av = v - iv;
      const i00 = 4 * (iv * PW + iu), i10 = i00 + 4, i01 = i00 + 4 * PW, i11 = i01 + 4;
      const w = Math.min(u + 0.5, PW - 0.5 - u, v + 0.5, PH - 0.5 - v) + 0.5, j = y * MW + x;
      for (let c = 0; c < 3; c++)
        sum[3 * j + c] += w * ((img[i00 + c] * (1 - au) + img[i10 + c] * au) * (1 - av) + (img[i01 + c] * (1 - au) + img[i11 + c] * au) * av);
      wsum[j] += w;
      md[4 * j] = sum[3 * j] / wsum[j]; md[4 * j + 1] = sum[3 * j + 1] / wsum[j]; md[4 * j + 2] = sum[3 * j + 2] / wsum[j]; md[4 * j + 3] = 255;
    }
  }
  // bring the mosaic to the current position along the path
  function update() {
    let n = 0;
    while (n < P.photos.length && P.photos[n].s <= state.s + 1e-6) n++;
    if (n < state.added) clearMosaic();
    const before = state.added;
    while (state.added < n) addPhoto(state.added++);
    if (state.added > before && !print) state.flash = 1;
  }

  // ---- drawing ----
  const mosC = h('canvas', { width: MW, height: MH });
  const photoC = h('canvas', { width: PW, height: PH, style: `width:${PWD}px; height:${Math.round(PWD * PH / PW)}px` });
  const photoCap = h('div', { class: 'readout', style: 'margin-top:0.25em; white-space:nowrap' });
  const readout = h('div', { class: 'readout', style: `line-height:1.35; width:${PWD}px; white-space:pre; font-size:0.92em` });
  const quad = i => [[0, 0], [PW, 0], [PW, PH], [0, PH]].map(([u, v]) => toGround(P.photos[i], u, v)).map(([X, Y]) => [X * sc, Y * sc]);

  function drawDrone(g, x, y, hd) {
    g.save(); g.translate(x, y); g.rotate(hd);
    g.strokeStyle = '#15171c'; g.lineWidth = 6;
    g.beginPath(); g.moveTo(-9, -9); g.lineTo(9, 9); g.moveTo(-9, 9); g.lineTo(9, -9); g.stroke();
    g.strokeStyle = '#ffffff'; g.lineWidth = 2.5;
    g.beginPath(); g.moveTo(-9, -9); g.lineTo(9, 9); g.moveTo(-9, 9); g.lineTo(9, -9); g.stroke();
    for (const [a, b] of [[-9, -9], [9, 9], [-9, 9], [9, -9]]) {
      g.beginPath(); g.arc(a, b, 5, 0, 2 * Math.PI); g.fillStyle = 'rgba(21,23,28,0.55)'; g.fill();
      g.strokeStyle = '#ffffff'; g.lineWidth = 1.5; g.stroke();
    }
    g.fillStyle = C.accent; g.beginPath(); g.moveTo(14, 0); g.lineTo(6, -4); g.lineTo(6, 4); g.closePath(); g.fill();
    g.restore();
  }

  function draw() {
    const g = mosC.getContext('2d');
    g.clearRect(0, 0, MW, MH);
    g.putImageData(mosaicImg, 0, 0);
    // planned path ahead
    g.strokeStyle = C.dim; g.lineWidth = 1.2; g.setLineDash([4, 4]);
    g.beginPath(); P.path.forEach((p, i) => (i ? g.lineTo(p.x * sc, p.y * sc) : g.moveTo(p.x * sc, p.y * sc))); g.stroke();
    g.setLineDash([]);
    // stations: taken (filled) / to come (hollow)
    P.photos.forEach((p, i) => {
      const [x, y] = [p.c[0] * sc, p.c[1] * sc];
      g.beginPath(); g.arc(x, y, 2.4, 0, 2 * Math.PI);
      if (i < state.added) { g.fillStyle = C.accent; g.fill(); } else { g.strokeStyle = C.dim; g.lineWidth = 1; g.stroke(); }
    });
    // footprint of the last photo
    if (state.added > 0) {
      const q = quad(state.added - 1);
      g.beginPath(); q.forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y))); g.closePath();
      if (state.flash > 0) { g.fillStyle = `rgba(242,177,52,${0.35 * state.flash})`; g.fill(); }
      g.strokeStyle = C.accent; g.lineWidth = 2.2; g.stroke();
    }
    const d = posAt(P.path, state.s);
    drawDrone(g, d.x * sc, d.y * sc, d.hd);
    // scale bar
    const L = 100 * sc;
    g.fillStyle = 'rgba(21,23,28,0.7)'; g.fillRect(8, MH - 30, L + 60, 22);
    g.strokeStyle = '#ffffff'; g.lineWidth = 3; g.beginPath(); g.moveTo(14, MH - 15); g.lineTo(14 + L, MH - 15); g.stroke();
    g.fillStyle = '#ffffff'; g.font = '13px Inter, sans-serif'; g.fillText('100 m', 20 + L, MH - 11);

    // the photo
    const pg = photoC.getContext('2d');
    if (state.added > 0) {
      const i = state.added - 1, p = P.photos[i];
      pg.putImageData(new ImageData(photoOf(i), PW, PH), 0, 0);
      photoCap.textContent = `photo ${i + 1} of ${P.photos.length}, flying ${p.line % 2 ? 'west' : 'east'}`;
    } else {
      pg.clearRect(0, 0, PW, PH);
      photoCap.textContent = `photo – of ${P.photos.length}`;
    }
    if (cfg.info) {
      let s = `altitude ${A} m, nadir camera\nfootprint ${fmt(P.fw, 0)} × ${fmt(P.fh, 0)} m\n` +
        `overlap ${FRONT * 100}% along, ${SIDE * 100}% across\n${P.ny} lines × ${P.nx} photos\n`;
      if (state.added > 0) {
        const H = groundH(P.photos[state.added - 1]), k = 1 / H[8];
        const sig = v => (Math.abs(v) >= 100 ? fmt(v, 0) : Math.abs(v) >= 1 ? fmt(v, 2) : fmt(v, 4));
        s += `\nH (map m → photo px), h₃₃ = 1\n` + [0, 1, 2].map(r => [0, 1, 2].map(c => sig(H[3 * r + c] * k).padStart(8)).join(' ')).join('\n');
      }
      readout.textContent = s;
    }
  }

  // ---- animation ----
  let raf = null, last = null;
  const shown = () => mosC.closest('section')?.classList.contains('present') !== false;
  function tick(now) {
    raf = null;
    if (!mosC.isConnected) return;
    const dt = last == null ? 0 : Math.min(0.1, (now - last) / 1000);
    last = now;
    if (shown()) {
      if (state.playing) {
        state.s = Math.min(P.len, state.s + dt * SPEED * state.speed);
        update();
        if (state.s >= P.len) { state.playing = false; state.endWait = cfg.loop === false ? 0 : 4; }
        syncControls();
      } else if (state.endWait > 0) {
        state.endWait -= dt;
        if (state.endWait <= 0) { state.endWait = 0; state.s = 0; update(); play(); }
      }
      if (state.flash > 0) state.flash = Math.max(0, state.flash - dt * 4);
      draw();
    }
    if (state.playing || state.flash > 0 || state.endWait > 0) raf = requestAnimationFrame(tick);
  }
  const kick = () => { if (!raf && !print) { last = null; raf = requestAnimationFrame(tick); } };
  function play() {
    if (state.s >= P.len) { state.s = 0; update(); }
    state.playing = true; syncControls(); kick();
  }
  function pause() { state.playing = false; state.endWait = 0; syncControls(); }

  const playBtn = h('button', { style: 'width:5.6em', onclick: () => (state.playing ? pause() : play()) }, '▶ fly');
  const restartBtn = h('button', { onclick: () => { state.s = 0; update(); draw(); play(); } }, '⟲ restart');
  const scrub = h('input', { type: 'range', min: 0, max: 1000, step: 1, value: 0, style: 'width:10em' });
  scrub.addEventListener('input', () => { pause(); state.s = scrub.value / 1000 * P.len; update(); draw(); });
  const speeds = [1, 2, 4].map(v => h('button', { onclick: () => { state.speed = v; syncControls(); } }, `${v}×`));
  function syncControls() {
    playBtn.textContent = state.playing ? '⏸ pause' : '▶ fly';
    scrub.value = Math.round(1000 * state.s / P.len);
    speeds.forEach((b, i) => b.classList.toggle('active', [1, 2, 4][i] === state.speed));
  }

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start; justify-content:center' },
    h('div', { style: `width:${MW}px; flex:none` }, mosC,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' },
        playBtn, restartBtn, scrub, h('label', { style: 'margin-left:0.3em' }, 'speed'), ...speeds)),
    h('div', { style: `display:flex; flex-direction:column; gap:0.3em; width:${PWD}px; flex:none` },
      h('div', { class: 'readout' }, 'the drone\'s latest photo'), photoC, photoCap, cfg.info ? readout : '')));

  update();
  syncControls();
  draw();
  if (!print && cfg.t == null && (cfg.autoplay ?? true)) play();
}
