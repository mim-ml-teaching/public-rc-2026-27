// The camera as a motion sensor: a camera moves through the ray-cast scene of scene3d.js (ground, box, sphere, striped
// panel, brick wall 20 m away); tracked feature points slide across its image, and the motion between frames, chained
// over time, gives the trajectory.
// Left: the camera image with feature tracks (the last frames of each visible point; near points sweep long tracks,
// the far wall hardly moves: parallax). Right: the top view (x right, z forward): the scene objects, the true path and
// the path chained from noisy frame-to-frame motions (small errors accumulate: Part 4, drift).
// Frames and conventions as in scene3d.js: world x right, y down, z forward, ground y = 1 (the camera is 1 m above it);
// camera yaw ψ about the vertical axis, R_cw = Ry(ψ), world → camera R = R_cwᵀ.
// Feature points are 3D points on the objects (projected exactly, visible when the depth buffer agrees), a stand-in for
// detected and tracked features (Part 2).
// config: {
//   width: 480          displayed width of the camera image (rendered 320 × 240)
//   topWidth: 330       top view size (square)
//   autoplay: true      play when the slide is shown (never in print)
//   t: null             progress 0 … 1 (print default 0.7; disables autoplay)
// }
import { C, h, fmt, isPrint } from './util.js';
import { renderView } from './scene3d.js';

const W = 320, H = 240, F = 300, OX = W / 2, OY = H / 2;
const N = 180, FPS = 12, TAIL = 10;
const deg = Math.PI / 180;

function rng(seed) {
  let a = seed >>> 0;
  const u = () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  u.normal = () => Math.sqrt(-2 * Math.log(u() || 1e-12)) * Math.cos(2 * Math.PI * u());
  return u;
}

// ---- the path: forward with a slalom, yaw follows the heading plus a slow look-around ----
function poseAt(k) {
  const t = k / N;
  const xf = t => 0.35 + 0.35 * Math.sin(1.6 * Math.PI * t + 0.5) - 0.5 * t, zf = t => -1.1 + 2.4 * t;
  const e = 1e-3, x = xf(t), z = zf(t), dx = xf(t + e) - x, dz = zf(t + e) - z;
  const yaw = 0.3 * Math.atan2(dx, dz) + 3 * deg * Math.sin(2 * Math.PI * 1.2 * t);
  return { c: [x, 0.02 * Math.sin(2 * Math.PI * 2.5 * t), z], yaw };
}
const Rwc = yaw => [[Math.cos(yaw), 0, -Math.sin(yaw)], [0, 1, 0], [Math.sin(yaw), 0, Math.cos(yaw)]];   // world → camera = Ry(yaw)ᵀ
function project(p, P) {
  const R = Rwc(p.yaw), d = [P[0] - p.c[0], P[1] - p.c[1], P[2] - p.c[2]];
  const x = R[0][0] * d[0] + R[0][2] * d[2], y = d[1], z = R[2][0] * d[0] + R[2][2] * d[2];
  return z > 0.15 ? [F * x / z + OX, F * y / z + OY, z] : null;
}

// ---- feature points on the objects ----
function features() {
  const r = rng(11), pts = [];
  for (let i = 0; i < 45; i++) pts.push([-4 + 8 * r(), 1, 0.7 + 11 * r() ** 1.3]);                          // ground
  for (let i = 0; i < 30; i++) pts.push([-7 + 14 * r(), -5 + 5.9 * r(), 20]);                               // wall
  for (let i = 0; i <= 3; i++) for (let j = 0; j <= 2; j++) pts.push([-1.7 + 0.3 * i, 0.2 + 0.4 * j, 3.0]); // box front
  for (let i = 0; i <= 3; i++) pts.push([-1.7 + 0.3 * i, 0.2, 3.35]);                                       // box top
  for (let i = 0; i < 7; i++) pts.push([0.9 + 0.075 * 2 * (1 + 2 * i) / 2 + 0.0375, -0.3 + 1.3 * r(), 5]);  // panel stripes
  for (let i = 0; i < 8; i++) {                                                                              // sphere (front)
    const a = (r() - 0.5) * 2.2, b = (r() - 0.5) * 1.8;
    pts.push([0.25 + 0.45 * Math.sin(a) * Math.cos(b), 0.55 + 0.45 * Math.sin(b), 2.8 - 0.45 * Math.cos(a) * Math.cos(b)]);
  }
  return pts;
}

export async function mount(el, cfg) {
  const print = isPrint();
  const DW = cfg.width || 480, TW = cfg.topWidth || 330;
  const poses = Array.from({ length: N + 1 }, (_, k) => poseAt(k));
  const pts = features();

  // chained estimate: true frame-to-frame motion with small errors in heading, direction and length
  const est = [{ x: poses[0].c[0], z: poses[0].c[2], yaw: poses[0].yaw }];
  const r = rng(5);
  for (let k = 1; k <= N; k++) {
    const a = poses[k - 1], b = poses[k], e = est[k - 1];
    const dyaw = b.yaw - a.yaw + 0.18 * deg * r.normal();
    // motion in the previous camera frame
    const dx = b.c[0] - a.c[0], dz = b.c[2] - a.c[2];
    const lx = Math.cos(a.yaw) * dx - Math.sin(a.yaw) * dz, lz = Math.sin(a.yaw) * dx + Math.cos(a.yaw) * dz;
    const s = 1 + 0.03 * r.normal(), th = 1.0 * deg * r.normal();
    const mx = s * (Math.cos(th) * lx + Math.sin(th) * lz), mz = s * (-Math.sin(th) * lx + Math.cos(th) * lz);
    est.push({ x: e.x + Math.cos(e.yaw) * mx + Math.sin(e.yaw) * mz, z: e.z - Math.sin(e.yaw) * mx + Math.cos(e.yaw) * mz, yaw: e.yaw + dyaw });
  }

  const state = { k: Math.round((cfg.t ?? (print ? 0.7 : 0)) * N), playing: false, endWait: 0 };
  const camC = h('canvas', { width: W, height: H, style: `width:${DW}px; height:${Math.round(DW * H / W)}px` });
  const topC = h('canvas', { width: TW, height: TW });
  const readout = h('div', { class: 'readout', style: `width:${TW}px; line-height:1.35; white-space:pre; font-size:0.92em` });
  let rendered = -1, frame = null;

  function drawCam() {
    const k = Math.round(state.k);
    if (rendered !== k) {
      const p = poses[k], R = Rwc(p.yaw);
      frame = renderView({ R, c: p.c, f: F, W, H, ox: OX, oy: OY });
      rendered = k;
    }
    const g = camC.getContext('2d');
    g.putImageData(new ImageData(frame.rgba, W, H), 0, 0);
    let nvis = 0;
    for (const P of pts) {
      const q = project(poses[k], P);
      if (!q || q[0] < 0 || q[1] < 0 || q[0] >= W || q[1] >= H) continue;
      const zb = frame.z[Math.floor(q[1]) * W + Math.floor(q[0])];
      if (!(Math.abs(zb - q[2]) < 0.03 * q[2] + 0.02)) continue;
      nvis++;
      // the track over the last frames
      g.strokeStyle = C.accent; g.lineWidth = 1.6; g.beginPath();
      let first = true;
      for (let j = Math.max(0, k - TAIL); j <= k; j++) {
        const s = project(poses[j], P);
        if (!s) { first = true; continue; }
        if (first) { g.moveTo(s[0], s[1]); first = false; } else g.lineTo(s[0], s[1]);
      }
      g.stroke();
      g.fillStyle = C.green; g.strokeStyle = '#15171c'; g.lineWidth = 1;
      g.beginPath(); g.arc(q[0], q[1], 2.6, 0, 2 * Math.PI); g.fill(); g.stroke();
    }
    return nvis;
  }

  // top view: x ∈ [−2.6, 2.8], z ∈ [−1.8, 5.6] → square canvas
  const X0 = -2.6, X1 = 2.8, Z0 = -1.8, Z1 = 5.6;
  const sc = Math.min(TW / (X1 - X0), TW / (Z1 - Z0));
  const T = (x, z) => [(x - X0) * sc + (TW - (X1 - X0) * sc) / 2, TW - (z - Z0) * sc];
  function drawTop() {
    const g = topC.getContext('2d'), k = Math.round(state.k);
    g.clearRect(0, 0, TW, TW);
    // objects
    g.fillStyle = 'rgba(90,176,255,0.35)'; g.strokeStyle = C.blue; g.lineWidth = 1.5;
    let [a, b] = T(-1.7, 3.7), [c, d] = T(-0.8, 3.0);
    g.fillRect(a, b, c - a, d - b); g.strokeRect(a, b, c - a, d - b);
    [a, b] = T(0.25, 2.8); g.beginPath(); g.arc(a, b, 0.45 * sc, 0, 2 * Math.PI); g.fillStyle = 'rgba(242,177,52,0.35)'; g.fill(); g.strokeStyle = C.accent; g.stroke();
    [a, b] = T(0.9, 5); [c, d] = T(2.4, 5); g.strokeStyle = C.fg; g.lineWidth = 4; g.beginPath(); g.moveTo(a, b); g.lineTo(c, d); g.stroke();
    g.fillStyle = C.dim; g.font = '12px Inter, sans-serif';
    g.fillText('box', T(-1.7, 3.9)[0], T(-1.7, 3.9)[1]); g.fillText('sphere', T(0.75, 2.6)[0], T(0.75, 2.6)[1]); g.fillText('panel', T(0.95, 5.25)[0], T(0.95, 5.25)[1]);
    g.fillText('brick wall 20 m ahead ↑', 8, 16);
    // paths
    const line = (arr, col, dash, w) => {
      g.strokeStyle = col; g.lineWidth = w; g.setLineDash(dash); g.beginPath();
      arr.forEach(([x, z], i) => { const [u, v] = T(x, z); i ? g.lineTo(u, v) : g.moveTo(u, v); }); g.stroke(); g.setLineDash([]);
    };
    line(poses.map(p => [p.c[0], p.c[2]]), C.dim, [4, 4], 1.2);
    line(est.slice(0, k + 1).map(p => [p.x, p.z]), C.accent, [], 2.5);
    // camera: true (outline) and estimated (filled), with the field of view
    const cam = (x, z, yaw, col, fill) => {
      const [u, v] = T(x, z), L = 0.55 * sc, hf = Math.atan(OX / F);
      g.save(); g.translate(u, v); g.rotate(yaw);
      g.beginPath(); g.moveTo(0, 0); g.lineTo(-L * Math.tan(hf), -L); g.lineTo(L * Math.tan(hf), -L); g.closePath();
      if (fill) { g.fillStyle = col; g.globalAlpha = 0.35; g.fill(); g.globalAlpha = 1; }
      g.strokeStyle = col; g.lineWidth = 1.8; g.stroke();
      g.restore();
    };
    cam(poses[k].c[0], poses[k].c[2], poses[k].yaw, C.fg, false);
    cam(est[k].x, est[k].z, est[k].yaw, C.accent, true);
    // legend
    g.font = '12px Inter, sans-serif';
    g.fillStyle = C.dim; g.fillText('╍ true path', 8, TW - 26);
    g.fillStyle = C.accent; g.fillText('━ chained from the images', 8, TW - 10);
  }

  function draw() {
    const nvis = drawCam();
    drawTop();
    const k = Math.round(state.k), j = Math.max(0, k - 1);
    const a = poses[j], b = poses[k];
    const turn = (b.yaw - a.yaw) / deg, dist = Math.hypot(b.c[0] - a.c[0], b.c[2] - a.c[2]);
    const err = Math.hypot(est[k].x - b.c[0], est[k].z - b.c[2]);
    readout.textContent =
      `frame ${k} of ${N}, ${nvis} tracked points\n` +
      `since the last frame: turned ${fmt(turn, 2)}°,\n  moved ${fmt(100 * dist, 1)} cm\n` +
      `chained: ${fmt(100 * err, 0)} cm off after ${fmt(pathLen(k), 1)} m`;
  }
  const cum = [0];
  for (let k = 1; k <= N; k++) cum.push(cum[k - 1] + Math.hypot(poses[k].c[0] - poses[k - 1].c[0], poses[k].c[2] - poses[k - 1].c[2]));
  const pathLen = k => cum[k];

  // ---- animation ----
  let raf = null, last = null;
  const shown = () => camC.closest('section')?.classList.contains('present') !== false;
  function tick(now) {
    raf = null;
    if (!camC.isConnected) return;
    const dt = last == null ? 0 : Math.min(0.1, (now - last) / 1000);
    last = now;
    if (shown()) {
      if (state.playing) {
        state.k = Math.min(N, state.k + dt * FPS);
        if (state.k >= N) { state.playing = false; state.endWait = 2.5; }
        syncControls();
      } else if (state.endWait > 0) {
        state.endWait -= dt;
        if (state.endWait <= 0) { state.endWait = 0; state.k = 0; play(); }
      }
      draw();
    }
    if (state.playing || state.endWait > 0) raf = requestAnimationFrame(tick);
  }
  const kick = () => { if (!raf && !print) { last = null; raf = requestAnimationFrame(tick); } };
  function play() { if (state.k >= N) state.k = 0; state.playing = true; syncControls(); kick(); }
  function pause() { state.playing = false; state.endWait = 0; syncControls(); }
  const playBtn = h('button', { style: 'width:5.6em', onclick: () => (state.playing ? pause() : play()) }, '▶ play');
  const scrub = h('input', { type: 'range', min: 0, max: N, step: 1, value: state.k, style: 'width:14em' });
  scrub.addEventListener('input', () => { pause(); state.k = parseFloat(scrub.value); draw(); });
  function syncControls() { playBtn.textContent = state.playing ? '⏸ pause' : '▶ play'; scrub.value = Math.round(state.k); }

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start; justify-content:center' },
    h('div', { style: `width:${DW}px; flex:none` }, camC,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, playBtn, scrub)),
    h('div', { style: `display:flex; flex-direction:column; gap:0.4em; width:${TW}px; flex:none` }, topC, readout)));
  syncControls();
  draw();
  if (!print && cfg.t == null && (cfg.autoplay ?? true)) play();
}
