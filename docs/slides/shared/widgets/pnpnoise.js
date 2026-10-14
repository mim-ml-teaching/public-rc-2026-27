// PnP under noise: how precise is the pose of a square marker? The 4 detected corners get new Gaussian noise
// (σ px) many times per second; each time the pose is solved again (homography → pose, then Levenberg–Marquardt
// on the reprojection error from that start and from its "flipped" twin, the better one wins, as IPPE does).
// The estimates pile up:
//   - position error in the camera frame (mm): top view (Δx, Δz) and side view (Δy, Δz), same scale, with the
//     2σ ellipse — the cloud is long along the line of sight (depth);
//   - orientation error as a rotation vector in the marker frame (deg): tilts about the marker's x and y axes;
//     the hollow red ring is the flipped twin of the true pose, where the "pose flips" land.
// The image view zooms onto the marker, so the noise circles are shown relative to the marker size.
// Marker frame (as in OpenCV ArUco): x right, y up, z out of the marker towards the camera.
// config: { dist (m, default 0.5), noise (px, default 1), yaw (deg, about the camera's vertical axis, default 10),
//           pitch (deg, about the camera's horizontal axis, default 0), width (display px of the image, default 400),
//           prefill: number of solves to show at once (default 0; print uses 300) }
import { C, h, fmt, isPrint } from './util.js';
import { matmul, matvec, transpose, solve, rodrigues, logR, rng } from './linalg.js';

const IW = 640, IH = 480, F = 800, OX = 320, OY = 240, S = 0.1; // marker side 10 cm
const deg = Math.PI / 180;
const CORNERS = [[-S / 2, S / 2, 0], [S / 2, S / 2, 0], [S / 2, -S / 2, 0], [-S / 2, -S / 2, 0]];
const BITS = ['000000', '011010', '010110', '001100', '011100', '000000'];
const Rx180 = [[1, 0, 0], [0, -1, 0], [0, 0, -1]];
const Ry = a => [[Math.cos(a), 0, Math.sin(a)], [0, 1, 0], [-Math.sin(a), 0, Math.cos(a)]];
const Rx = a => [[1, 0, 0], [0, Math.cos(a), -Math.sin(a)], [0, Math.sin(a), Math.cos(a)]];
const HIST = 300, PERIOD = 55; // samples kept, ms between samples

const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = a => a.map(v => v / Math.hypot(...a));
const col = (R, j) => [R[0][j], R[1][j], R[2][j]];
const project = (R, t, X) => { const p = matvec(R, X).map((v, i) => v + t[i]); return [F * p[0] / p[2] + OX, F * p[1] / p[2] + OY]; };

// rotation about a unit axis
const axisAngle = (k, a) => rodrigues(k.map(v => v * a));

// The pose with the marker normal mirrored about the line of sight: the second solution of a planar target.
function flipped(R, t) {
  const n = col(R, 2), v = norm(t.map(x => -x));
  const n2 = v.map((vi, i) => 2 * dot(n, v) * vi - n[i]);
  const ax = cross(n, n2), s = Math.hypot(...ax);
  if (s < 1e-9) return R;
  return matmul(axisAngle(ax.map(x => x / s), Math.atan2(s, dot(n, n2))), R);
}

// Homography (h33 = 1) from the marker plane to normalised image coordinates, then R, t from K⁻¹H = λ[r1 r2 t].
function poseFromHomography(x) {
  const A = [], b = [];
  CORNERS.forEach(([X, Y], i) => {
    const xn = (x[i][0] - OX) / F, yn = (x[i][1] - OY) / F;
    A.push([X, Y, 1, 0, 0, 0, -xn * X, -xn * Y]); b.push(xn);
    A.push([0, 0, 0, X, Y, 1, -yn * X, -yn * Y]); b.push(yn);
  });
  const hv = solve(A, b);
  if (!hv) return null;
  const H = [[hv[0], hv[1], hv[2]], [hv[3], hv[4], hv[5]], [hv[6], hv[7], 1]];
  const h1 = col(H, 0), h2 = col(H, 1), h3 = col(H, 2);
  let lam = 2 / (Math.hypot(...h1) + Math.hypot(...h2));
  if (h3[2] * lam < 0) lam = -lam;                      // the marker is in front of the camera
  const a = norm(h1), c = norm(h2);
  const p = norm(a.map((v, i) => v + c[i])), q = norm(a.map((v, i) => v - c[i]));
  const r1 = p.map((v, i) => (v + q[i]) / Math.SQRT2), r2 = p.map((v, i) => (v - q[i]) / Math.SQRT2), r3 = cross(r1, r2);
  return { R: [0, 1, 2].map(i => [r1[i], r2[i], r3[i]]), t: h3.map(v => v * lam) };
}

// Levenberg–Marquardt on the reprojection error; rotation as a small update rodrigues(δ) · R.
function refine(R0, t0, x) {
  const res = (R, t) => CORNERS.flatMap((X, i) => { const p = project(R, t, X); return [p[0] - x[i][0], p[1] - x[i][1]]; });
  const sq = r => r.reduce((s, v) => s + v * v, 0);
  const apply = (R, t, d) => [matmul(rodrigues(d.slice(0, 3)), R), t.map((v, i) => v + d[3 + i])];
  let R = R0, t = t0, r = res(R, t), err = sq(r), mu = 1e-3;
  for (let it = 0; it < 25; it++) {
    const J = r.map(() => new Array(6).fill(0));
    for (let k = 0; k < 6; k++) {
      const e = k < 3 ? 1e-6 : 1e-7 * Math.max(1, Math.abs(t[k - 3]));
      const d = new Array(6).fill(0); d[k] = e;
      res(...apply(R, t, d)).forEach((v, i) => { J[i][k] = (v - r[i]) / e; });
    }
    const Jt = transpose(J), JtJ = matmul(Jt, J), Jtr = matvec(Jt, r);
    let ok = false;
    for (; mu < 1e8; mu *= 10) {
      const d = solve(JtJ.map((row, i) => row.map((v, j) => v + (i === j ? mu * (JtJ[i][i] + 1e-9) : 0))), Jtr.map(v => -v));
      if (!d) continue;
      const [R2, t2] = apply(R, t, d), r2 = res(R2, t2), e2 = sq(r2);
      if (t2[2] > 0 && e2 < err) {
        const small = err - e2 < 1e-10 * (1 + err);
        R = R2; t = t2; r = r2; err = e2; mu = Math.max(mu / 10, 1e-9); ok = !small;
        break;
      }
    }
    if (!ok) break;
  }
  return { R, t, err };
}

function solvePnP(x) {
  const s = poseFromHomography(x);
  if (!s) return null;
  const a = refine(s.R, s.t, x), b = refine(flipped(s.R, s.t), s.t, x);
  return a.err <= b.err ? a : b;
}

const niceCeil = v => { const p = 10 ** Math.floor(Math.log10(v)); return p * [1, 1.5, 2, 3, 4, 5, 6, 8, 10].find(m => m * p >= v - 1e-12); };
const nice = v => { const p = 10 ** Math.floor(Math.log10(v)), m = v / p; return (m <= 1 ? 1 : m <= 2 ? 2 : m <= 5 ? 5 : 10) * p; };
const std = a => { if (a.length < 2) return NaN; const m = a.reduce((s, v) => s + v, 0) / a.length; return Math.sqrt(a.reduce((s, v) => s + (v - m) ** 2, 0) / (a.length - 1)); };

export function mount(el, cfg) {
  const DW = cfg.width || 400, PS = 236; // image display width, scatter plot size (px)
  const state = { dist: cfg.dist ?? 0.5, noise: cfg.noise ?? 1, yaw: cfg.yaw ?? 10, pitch: cfg.pitch ?? 0, playing: !isPrint(), hist: [], x: null };
  let rand = rng(cfg.seed ?? (Date.now() & 0xffff));

  const mk = (w, hh) => h('canvas', { width: w * 2, height: hh * 2, style: `width:${w}px; height:${hh}px` });
  const img = mk(DW, DW * IH / IW), top = mk(PS, PS), side = mk(PS, PS), tilt = mk(PS, PS);
  const readout = h('div', { style: `width:${PS}px; font-size:0.95em; line-height:1.4` });

  const truth = () => ({ R: matmul(matmul(Ry(state.yaw * deg), Rx(state.pitch * deg)), Rx180), t: [0, 0, state.dist] });

  function sample() {
    const T = truth();
    const x = CORNERS.map(X => project(T.R, T.t, X).map(v => v + state.noise * rand.normal()));
    const est = solvePnP(x);
    state.x = x;
    if (!est) return;
    const dp = est.t.map((v, i) => 1000 * (v - T.t[i]));                // mm, camera frame
    const w = logR(matmul(transpose(T.R), est.R)).map(v => v / deg);      // deg, marker frame
    // flipped: closer to the twin of the true pose than to the true pose (only when the twin is distinct)
    const n = col(est.R, 2), nT = col(T.R, 2), nF = col(flipped(T.R, T.t), 2);
    const far = Math.acos(Math.min(1, dot(nT, nF))) > 3 * deg;
    const flip = far && dot(n, nF) > dot(n, nT);
    state.hist.push({ dp, w, flip, est });
    if (state.hist.length > HIST) state.hist.shift();
  }

  // ---- drawing ----
  function drawImage() {
    const g = img.getContext('2d'), T = truth(), k = img.width / IW;
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.fillStyle = '#0d0f13'; g.fillRect(0, 0, img.width, img.height);
    // zoom onto the marker so that it fills about half of the view
    const sizePx = F * S / state.dist, zoom = Math.max(1, 0.5 * IH / sizePx), c = project(T.R, T.t, [0, 0, 0]);
    g.setTransform(k * zoom, 0, 0, k * zoom, k * (IW / 2 - zoom * c[0]), k * (IH / 2 - zoom * c[1]));
    const cell = S / 8, qz = S / 2 + cell; // white quiet zone around the printed marker
    g.fillStyle = '#ffffff'; g.beginPath();
    [[-qz, qz], [qz, qz], [qz, -qz], [-qz, -qz]].map(([a, b]) => project(T.R, T.t, [a, b, 0])).forEach(([u, v], m) => (m ? g.lineTo(u, v) : g.moveTo(u, v)));
    g.closePath(); g.fill();
    for (let i = 0; i < 8; i++) for (let j = 0; j < 8; j++) {
      const white = i > 0 && i < 7 && j > 0 && j < 7 && BITS[j - 1][i - 1] === '1';
      const x0 = -S / 2 + i * cell, y0 = S / 2 - j * cell;
      const Q = [[x0, y0, 0], [x0 + cell, y0, 0], [x0 + cell, y0 - cell, 0], [x0, y0 - cell, 0]].map(X => project(T.R, T.t, X));
      g.fillStyle = white ? '#ffffff' : '#000000';
      g.beginPath(); Q.forEach(([u, v], m) => (m ? g.lineTo(u, v) : g.moveTo(u, v))); g.closePath(); g.fill();
      g.strokeStyle = g.fillStyle; g.lineWidth = 0.6 / zoom; g.stroke();
    }
    const last = state.hist[state.hist.length - 1];
    if (last) {
      const Q = CORNERS.map(X => project(last.est.R, last.est.t, X));
      g.strokeStyle = last.flip ? C.red : C.blue; g.lineWidth = 2.5 / zoom; g.setLineDash([8 / zoom, 6 / zoom]);
      g.beginPath(); Q.forEach(([u, v], m) => (m ? g.lineTo(u, v) : g.moveTo(u, v))); g.closePath(); g.stroke(); g.setLineDash([]);
    }
    if (state.x) state.x.forEach(([u, v]) => {
      g.strokeStyle = C.accent; g.lineWidth = 2.5 / zoom; g.beginPath(); g.arc(u, v, Math.max(2 * state.noise, 1.5), 0, 7); g.stroke();
      g.fillStyle = C.accent; g.beginPath(); g.arc(u, v, 2 / zoom, 0, 7); g.fill();
    });
    g.setTransform(k, 0, 0, k, 0, 0);
    g.font = '24px Inter, Arial';
    g.fillStyle = C.accent; g.fillText('○ detected corners (radius 2σ)', 12, 30);
    g.fillStyle = C.blue; g.fillText('-- marker at the estimated pose', 12, 60);
    g.fillStyle = C.dim; g.fillText(`zoom ×${fmt(zoom, 1)} · marker ${Math.round(sizePx)} px wide`, 12, IH - 14);
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, IW - 2, IH - 2);
  }

  // scatter of 2D points (a list of [x, y, flip]) with axes, ticks and the 2σ ellipse of the non-flipped ones
  function scatter(cv, pts, range, unit, xl, yl, extra) {
    const g = cv.getContext('2d'), W = cv.width, Hh = cv.height, k = W / PS;
    g.setTransform(k, 0, 0, k, 0, 0);
    g.fillStyle = C.bg2; g.fillRect(0, 0, PS, PS);
    const m = 26, half = (PS - 2 * m) / 2, cx = PS / 2 + 6, cy = PS / 2 - 6;
    const X = v => cx + v / range * half, Y = v => cy - v / range * half;
    // grid
    const step = nice(range / 2);
    g.strokeStyle = C.line; g.lineWidth = 1; g.font = '10px Inter, Arial'; g.fillStyle = C.dim;
    for (let v = -Math.floor(range / step) * step; v <= range + 1e-9; v += step) {
      g.beginPath(); g.moveTo(X(v), cy - half); g.lineTo(X(v), cy + half); g.stroke();
      g.beginPath(); g.moveTo(cx - half, Y(v)); g.lineTo(cx + half, Y(v)); g.stroke();
    }
    g.strokeStyle = C.dim; g.beginPath(); g.moveTo(cx - half, cy); g.lineTo(cx + half, cy); g.moveTo(cx, cy - half); g.lineTo(cx, cy + half); g.stroke();
    g.textAlign = 'center';
    g.fillText(`${fmt(step, step < 1 ? 1 : 0)}`, X(step), cy + half + 11);
    g.fillText(`−${fmt(step, step < 1 ? 1 : 0)}`, X(-step), cy + half + 11);
    g.font = '12px Inter, Arial'; g.fillStyle = C.fg;
    g.fillText(xl, cx, PS - 3);
    g.save(); g.translate(10, cy); g.rotate(-Math.PI / 2); g.fillText(yl, 0, 0); g.restore();
    g.textAlign = 'left'; g.font = '10px Inter, Arial'; g.fillStyle = C.dim; g.fillText(unit, PS - 30, 12);
    // points, older ones fainter
    g.save(); g.beginPath(); g.rect(cx - half, cy - half, 2 * half, 2 * half); g.clip();
    pts.forEach(([a, b, flip], i) => {
      g.globalAlpha = 0.2 + 0.8 * (i + 1) / pts.length;
      g.fillStyle = flip ? C.red : C.blue; g.beginPath(); g.arc(X(a), Y(b), 2.2, 0, 7); g.fill();
    });
    g.globalAlpha = 1;
    // 2σ ellipse of the main cluster
    const q = pts.filter(p => !p[2]);
    if (q.length > 5) {
      const ma = q.reduce((s, p) => s + p[0], 0) / q.length, mb = q.reduce((s, p) => s + p[1], 0) / q.length;
      let saa = 0, sbb = 0, sab = 0;
      q.forEach(([a, b]) => { saa += (a - ma) ** 2; sbb += (b - mb) ** 2; sab += (a - ma) * (b - mb); });
      saa /= q.length - 1; sbb /= q.length - 1; sab /= q.length - 1;
      const tr = (saa + sbb) / 2, dd = Math.sqrt(((saa - sbb) / 2) ** 2 + sab * sab);
      const l1 = tr + dd, l2 = Math.max(0, tr - dd), ang = Math.atan2(l1 - saa, sab || 1e-12);
      g.strokeStyle = C.accent; g.lineWidth = 2;
      g.beginPath();
      for (let s = 0; s <= 64; s++) {
        const th = s / 64 * 2 * Math.PI, u = 2 * Math.sqrt(l1) * Math.cos(th), v = 2 * Math.sqrt(l2) * Math.sin(th);
        const a = ma + u * Math.cos(ang) - v * Math.sin(ang), b = mb + u * Math.sin(ang) + v * Math.cos(ang);
        s ? g.lineTo(X(a), Y(b)) : g.moveTo(X(a), Y(b));
      }
      g.stroke();
    }
    if (extra) extra(g, X, Y);
    g.restore();
    g.fillStyle = C.fg; g.beginPath(); g.arc(cx, cy, 3, 0, 7); g.fill();
  }

  function draw() {
    drawImage();
    const H = state.hist, T = truth();
    const dp = H.map(s => s.dp), w = H.map(s => s.w);
    const okp = H.filter(s => !s.flip).map(s => s.dp);
    const rp = niceCeil(1.15 * Math.max(0.05, ...(okp.length ? okp : dp).flatMap(p => p.map(Math.abs))));
    scatter(top, H.map(s => [s.dp[0], s.dp[2], s.flip]), rp, 'mm', 'Δx (right)', 'Δz (depth, away)');
    scatter(side, H.map(s => [s.dp[1], s.dp[2], s.flip]), rp, 'mm', 'Δy (down)', 'Δz (depth, away)');
    const wf = logR(matmul(transpose(T.R), flipped(T.R, T.t))).map(v => v / deg);
    const okw = H.filter(s => !s.flip).map(s => s.w);
    const spread = Math.max(0.2, ...(okw.length ? okw : w).flatMap(p => [Math.abs(p[0]), Math.abs(p[1])]));
    const twin = Math.max(Math.abs(wf[0]), Math.abs(wf[1])), showTwin = twin > 0.5 && twin < 4 * spread;
    const rw = niceCeil(1.15 * Math.max(spread, showTwin ? twin + 1 : 0));
    scatter(tilt, H.map(s => [s.w[1], s.w[0], s.flip]), rw, 'deg', 'tilt about marker y', 'tilt about marker x', (g, X, Y) => {
      if (showTwin) { g.strokeStyle = C.red; g.lineWidth = 2; g.beginPath(); g.arc(X(wf[1]), Y(wf[0]), 7, 0, 7); g.stroke(); }
      else if (H.some(s => s.flip)) {
        g.fillStyle = C.red; g.font = '11px Inter, Arial'; g.textAlign = 'center';
        g.fillText(`flips: ${Math.round(Math.hypot(wf[0], wf[1]))}° away, off the plot`, PS / 2 + 6, 36); g.textAlign = 'left';
      }
    });
    const n = H.length, nf = H.filter(s => s.flip).length;
    const main = H.filter(s => !s.flip), sd = (key, i) => std(main.map(s => s[key][i]));
    const num = v => (Number.isFinite(v) ? fmt(v, v < 10 ? 2 : 1) : '—');
    const cell = 'text-align:right; font-family:var(--mono); padding:0.05em 0.3em';
    const tr = (name, i, c) => `<tr><td style="padding-right:0.3em; color:${c || 'inherit'}">${name}</td><td style="${cell}; color:${c || 'inherit'}">${num(sd('dp', i))}</td><td style="${cell}">${num(sd('w', i))}</td></tr>`;
    readout.innerHTML =
      `<div class="dim" style="margin-bottom:0.2em">σ over ${n} solves</div>` +
      `<table style="border-collapse:collapse; font-size:1em; margin:0">` +
      `<tr class="dim"><td></td><td style="${cell}">pos. mm</td><td style="${cell}">rot. °</td></tr>` +
      tr('x', 0) + tr('y', 1) + tr('z', 2, C.accent) + `</table>` +
      `<div style="margin-top:0.3em; color:${nf ? C.red : 'inherit'}">flipped: ${n ? Math.round(100 * nf / n) : 0}% (not in σ)</div>` +
      `<div class="dim" style="font-size:0.85em; margin-top:0.3em; line-height:1.3">pos.: camera axes (z = depth)<br>rot.: marker axes (z = normal)</div>`;
  }

  // ---- animation ----
  let last = null, acc = 0, raf = 0;
  function tick(now) {
    raf = 0;
    if (!el.isConnected) return;
    const shown = el.closest('section')?.classList.contains('present') !== false;
    if (state.playing && shown && last !== null) {
      acc += now - last;
      if (acc >= PERIOD) { acc = Math.min(acc - PERIOD, PERIOD); sample(); draw(); }
    }
    last = now;
    if (state.playing) raf = requestAnimationFrame(tick);
  }
  const play = () => { if (!raf) { last = null; raf = requestAnimationFrame(tick); } };

  // ---- controls ----
  const clear = () => { state.hist = []; sample(); draw(); };
  const slider = (key, min, max, step, name, unit, digits) => {
    const s = h('input', { type: 'range', min, max, step, value: state[key], style: 'width:9em' });
    const val = h('span', { class: 'readout', style: 'display:inline-block; width:3.6em; text-align:right' });
    const upd = () => { val.textContent = fmt(state[key], digits) + unit; };
    s.addEventListener('input', () => { state[key] = +s.value; upd(); clear(); });
    upd();
    return h('div', { style: 'display:flex; align-items:center; gap:0.35em' }, h('label', { style: 'width:4.4em; flex:none' }, name), s, val);
  };
  const pauseBtn = h('button', { onclick: () => { state.playing = !state.playing; pauseBtn.textContent = state.playing ? 'pause' : 'play'; if (state.playing) play(); } }, state.playing ? 'pause' : 'play');

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${DW}px; flex:none` }, img,
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.15em' },
        slider('yaw', 0, 70, 1, 'yaw', '°', 0),
        slider('pitch', 0, 60, 1, 'pitch', '°', 0),
        slider('dist', 0.25, 3, 0.05, 'distance', ' m', 2),
        slider('noise', 0, 3, 0.1, 'noise σ', ' px', 1),
        h('div', { style: 'display:flex; gap:0.3em' }, pauseBtn, h('button', { onclick: clear }, 'clear')))),
    h('div', { style: `display:grid; grid-template-columns:${PS}px ${PS}px; gap:0.5em 0.6em; flex:none` },
      h('div', {}, h('div', { class: 'dim', style: 'margin-bottom:0.15em' }, 'position error, top (mm)'), top),
      h('div', {}, h('div', { class: 'dim', style: 'margin-bottom:0.15em' }, 'position error, side (mm)'), side),
      h('div', {}, h('div', { class: 'dim', style: 'margin-bottom:0.15em' }, 'orientation error (deg)'), tilt),
      readout)));

  const pre = isPrint() ? HIST : cfg.prefill || 0;
  if (isPrint()) rand = rng(7);
  for (let i = 0; i < Math.max(1, pre); i++) sample();
  draw();
  if (!isPrint()) play();
}
