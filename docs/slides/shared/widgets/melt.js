// A melting ice cream next to a deforming probability density, both driven by one time t ∈ [0, 1].
// Left: the scoop is a smooth blob (a particle cloud rendered as metaballs) carried by a velocity field: it slumps over
// the cone, drips down beside it and spreads into a puddle on the table. Sprinkles ride along as tracers.
// Right: the picture of "Deforming probability: example". X ~ N(0, 1) (bottom, hanging down) is sent by φ_t to the
// deformed density on top; the comb shows where φ_t sends equally spaced points. φ_t = (1 − t)·id + t·T with T the
// monotone (optimal transport) map from N(0, 1) to a two-bump mixture, so at t = 1 the deformed density is the "data".
// config: { panel: 'both' | 'ice' | 'density', width: 440 (left panel, or the single panel), width2: 700 (right panel), height: 370,
//           controls: 'row' (slider + button + readout) | 'overlay' (a pause button in the canvas corner),
//           samples: 0 (n > 0: n samples of X as dots under the x axis, and their images φ_t(x) as dots under the deformed density), t: 0, print_t }
import { LIGHT } from '../theme.js';
import { C, h, isPrint } from './util.js';
import { rng } from './flowlib.js';
import { loopPlayer } from './loopplayer.js';

const STEPS = 80, T = 3.2, RIM = 1.0, TABLE = -2.3, YT = 2.7, YB = -2.7;
const SPRINKLE = ['#5ab0ff', '#5fd38d', '#ffd34d', '#b28dff', '#ff6b6b'];
const X0 = -4, X1 = 4, MIX = [[0.6, -1.5, 0.6], [0.4, 1.5, 0.7]];     // the "data" density: weights, means, sds
const pX = x => Math.exp(-x * x / 2) / Math.sqrt(2 * Math.PI);
const pM = x => MIX.reduce((s, [w, m, sd]) => s + w * Math.exp(-((x - m) ** 2) / (2 * sd * sd)) / (sd * Math.sqrt(2 * Math.PI)), 0);
// T = F_M^{-1} ∘ F_X on a grid (both CDFs by the trapezoid rule, the inverse by walking the monotone grids together)
const GN = 4000, G0 = -6, G1 = 6, GD = (G1 - G0) / GN, GXS = Array.from({ length: GN + 1 }, (_, i) => G0 + i * GD);
const cum = f => { const F = new Float64Array(GN + 1); for (let i = 1; i <= GN; i++) F[i] = F[i - 1] + 0.5 * (f(GXS[i - 1]) + f(GXS[i])) * GD; return F; };
const TG = (() => { const FX = cum(pX), FM = cum(pM), out = new Float64Array(GN + 1); let j = 0;
  for (let i = 0; i <= GN; i++) { const u = FX[i]; while (j < GN - 1 && FM[j + 1] < u) j++; const a = FM[j], b = FM[j + 1], f = b > a ? Math.min(1, Math.max(0, (u - a) / (b - a))) : 0; out[i] = GXS[j] + f * GD; }
  return out; })();
const Tmap = x => { const p = (x - G0) / GD, i = Math.max(0, Math.min(GN - 1, Math.floor(p))), f = p - i; return TG[i] + f * (TG[i + 1] - TG[i]); };
const phi = (x, t) => (1 - t) * x + t * Tmap(x);
const dphi = (x, t) => (1 - t) + t * pX(x) / Math.max(pM(Tmap(x)), 1e-6);   // T′ = p_X / (p_M ∘ T)

export function mount(el, cfg) {
  const PANEL = cfg.panel || 'both', ICE = PANEL !== 'density', DEN = PANEL !== 'ice', OVER = cfg.controls === 'overlay';
  const W = cfg.width || 440, W2 = cfg.width2 || (PANEL === 'density' && cfg.width) || 700, H = cfg.height || 370;
  const st = { t: cfg.t ?? 0 };
  const sx = x => W / 2 + x * H / (YT - YB), sy = y => (YT - y) / (YT - YB) * H;   // one scale for both axes: a wider canvas shows more table
  const NS = cfg.samples || 0, xs = [];   // samples of X (the same first ones as in gauss1d), shown as dot histograms in the density panel
  { const r = rng(5); while (xs.length < NS) { const z = Math.sqrt(-2 * Math.log(Math.max(r(), 1e-12))) * Math.cos(2 * Math.PI * r()); if (Math.abs(z) < 3.4) xs.push(z); } }
  const hf = xs.map((_, i) => 0.06 + 0.9 * ((i * 0.6180339887) % 1));   // height fraction of each dot under the curve (low-discrepancy, so the fill is even)
  const cv1 = h('canvas', { width: W, height: H, style: `width:${W}px; height:${H}px` }), cv2 = h('canvas', { width: W2, height: H, style: `width:${W2}px; height:${H}px` });
  const g1 = cv1.getContext('2d'), g2 = cv2.getContext('2d'), cvMain = ICE ? cv1 : cv2;
  // ---- the ice cream: particles carried by a velocity field, trajectories precomputed ----
  const r = rng(11), pts = [];
  while (pts.length < 1000) { const x = 2.3 * r() - 1.15, y = 2.3 * r() - 1.15; if (x * x + y * y <= 1.15 * 1.15) pts.push([x, y + 1.16]); }
  const hs = pts.map(() => 0.05 + 0.3 * r());                       // rest height of each particle: the puddle has thickness
  const px = pts.map(() => 0.3 + 2.8 * r());                         // where on the table each particle settles (|x|): the puddle spreads over the whole table
  const vel = (x, y, i) => {
    const sgn = x < 0 ? -1 : 1, floor = TABLE + 0.6 * hs[i];
    if (y > -0.05 && Math.abs(x) <= RIM) return [0.6 * x, -0.8 * (y - hs[i])];   // on top: spread and flatten
    if (y > floor + 0.05) return [0.35 * sgn, -(1.2 - 2.4 * Math.min(0, y))];     // over the rim: drip down beside the cone, speeding up (the stream thins)
    return [1.2 * (sgn * px[i] - x), -1.7 * (y - floor)];                          // on the table: the puddle spreads over the table
  };
  const traj = pts.map((p, i) => { const path = [p]; let [x, y] = p; for (let k = 0; k < STEPS; k++) { const [vx, vy] = vel(x, y, i); x += vx * T / STEPS; y += vy * T / STEPS; path.push([x, y]); } return path; });
  // metaballs: soft discs summed on an offscreen canvas, then the alpha channel is thresholded into one smooth shape
  const KR = 24, sprite = document.createElement('canvas'); sprite.width = sprite.height = 2 * KR;
  { const gs = sprite.getContext('2d'), grad = gs.createRadialGradient(KR, KR, 0, KR, KR, KR);
    grad.addColorStop(0, 'rgba(0,0,0,0.5)'); grad.addColorStop(0.5, 'rgba(0,0,0,0.22)'); grad.addColorStop(1, 'rgba(0,0,0,0)');
    gs.fillStyle = grad; gs.fillRect(0, 0, 2 * KR, 2 * KR); }
  const off = document.createElement('canvas'); off.width = W; off.height = H; const go = off.getContext('2d', { willReadFrequently: true });
  const pink = LIGHT ? [255, 122, 170] : [255, 158, 196];   // written as pixels, so the light theme's colour mapping does not apply
  function blob(P) {
    go.clearRect(0, 0, W, H); go.globalCompositeOperation = 'lighter';
    for (const [x, y] of P) go.drawImage(sprite, sx(x) - KR, sy(y) - KR);
    go.globalCompositeOperation = 'source-over';
    const img = go.getImageData(0, 0, W, H), d = img.data, lo = 120, hi = 150;
    for (let i = 0; i < d.length; i += 4) {
      const a = d[i + 3];
      if (a < lo) { d[i + 3] = 0; continue; }
      d[i] = pink[0]; d[i + 1] = pink[1]; d[i + 2] = pink[2]; d[i + 3] = a >= hi ? 255 : Math.round(255 * (a - lo) / (hi - lo));
    }
    go.putImageData(img, 0, 0); g1.drawImage(off, 0, 0);
  }
  // ---- the density panel ----
  const ML = 40, MR = 20, sx2 = x => ML + (x - X0) / (X1 - X0) * (W2 - ML - MR);
  const yY = 0.44 * H, yX = 0.64 * H, SC = 0.5 * H;   // the y axis (top, deformed density upwards) and the x axis (bottom, p_X hanging down)

  function draw() {
    if (ICE) drawIce();
    if (DEN) drawDensity();
    readout.textContent = `t = ${st.t.toFixed(2)}`;
  }
  function drawIce() {
    const k = Math.round(st.t * STEPS), P = traj.map(p => p[k]);
    g1.fillStyle = C.bg2; g1.fillRect(0, 0, W, H);
    g1.strokeStyle = '#5a606c'; g1.lineWidth = 2; g1.beginPath(); g1.moveTo(0, sy(TABLE)); g1.lineTo(W, sy(TABLE)); g1.stroke();   // table
    g1.fillStyle = '#c98500'; g1.beginPath(); g1.moveTo(sx(-RIM), sy(0)); g1.lineTo(sx(RIM), sy(0)); g1.lineTo(sx(0), sy(TABLE)); g1.closePath(); g1.fill();
    g1.strokeStyle = 'rgba(21,23,28,0.35)'; g1.lineWidth = 1;                                            // waffle pattern
    for (let d = -2.5; d <= 2.5; d += 0.35) { g1.beginPath(); g1.moveTo(sx(-RIM), sy(d)); g1.lineTo(sx(RIM), sy(d - 2.3)); g1.stroke(); g1.beginPath(); g1.moveTo(sx(RIM), sy(d)); g1.lineTo(sx(-RIM), sy(d - 2.3)); g1.stroke(); }
    blob(P);
    P.forEach(([x, y], i) => { if (i % 17 === 0) { g1.fillStyle = SPRINKLE[i % SPRINKLE.length]; g1.beginPath(); g1.arc(sx(x), sy(y), 3, 0, 7); g1.fill(); } });
  }
  function drawDensity() {
    const t = st.t;
    g2.fillStyle = C.bg2; g2.fillRect(0, 0, W2, H);
    g2.font = '13px Inter, Arial';
    for (const [y, lab] of [[yY, 'y = φ_t(x)'], [yX, 'x']]) {
      g2.strokeStyle = '#5a606c'; g2.lineWidth = 1; g2.beginPath(); g2.moveTo(ML, y); g2.lineTo(W2 - MR, y); g2.stroke();
      g2.fillStyle = C.dim; g2.textAlign = 'right'; g2.fillText(lab, W2 - MR - 2, y - 5);
      for (let v = -3; v <= 3; v++) { g2.beginPath(); g2.moveTo(sx2(v), y - 3); g2.lineTo(sx2(v), y + 3); g2.stroke(); }
    }
    g2.textAlign = 'center'; g2.fillStyle = C.dim;
    for (let v = -3; v <= 3; v++) g2.fillText(String(v), sx2(v), yX - 7);
    // p_X hanging down from the x axis
    g2.beginPath(); g2.moveTo(sx2(X0), yX);
    for (let x = X0; x <= X1; x += 0.02) g2.lineTo(sx2(x), yX + pX(x) * SC);
    g2.lineTo(sx2(X1), yX); g2.closePath(); g2.fillStyle = 'rgba(90,176,255,0.18)'; g2.fill(); g2.strokeStyle = C.blue; g2.lineWidth = 2; g2.stroke();
    // comb: where φ_t sends equally spaced points
    g2.strokeStyle = 'rgba(154,161,174,0.55)'; g2.lineWidth = 1;
    for (let x = -3.6; x <= 3.6; x += 0.15) { g2.beginPath(); g2.moveTo(sx2(x), yX); g2.lineTo(sx2(phi(x, t)), yY); g2.stroke(); }
    // the deformed density, upwards from the y axis: the point (φ_t(x), p_X(x) / φ_t′(x))
    g2.beginPath(); g2.moveTo(sx2(phi(X0, t)), yY);
    for (let x = X0; x <= X1; x += 0.005) g2.lineTo(sx2(phi(x, t)), yY - pX(x) / Math.max(dphi(x, t), 0.05) * SC);
    g2.lineTo(sx2(phi(X1, t)), yY); g2.closePath(); g2.fillStyle = 'rgba(242,177,52,0.18)'; g2.fill(); g2.strokeStyle = C.accent; g2.lineWidth = 2; g2.stroke();
    if (NS) {   // the samples fill the area under each curve (fixed height fractions), so the cloud moves with the distribution
      const dot = (x, y, color) => { g2.fillStyle = color; g2.beginPath(); g2.arc(sx2(x), y, 2.6, 0, 7); g2.fill(); };
      xs.forEach((x, i) => { dot(x, yX + hf[i] * pX(x) * SC, 'rgba(90,176,255,0.9)'); dot(phi(x, t), yY - hf[i] * pX(x) / Math.max(dphi(x, t), 0.05) * SC, 'rgba(242,177,52,0.9)'); });
    }
    g2.textAlign = 'left'; g2.fillStyle = C.accent; g2.fillText(`deformed density at time t = ${t.toFixed(2)}` + (t >= 1 ? ' · the data distribution' : ''), ML + 6, 16);
    g2.fillStyle = C.blue; g2.fillText('known density: X ~ N(0, 1)', ML + 6, H - 8);
  }
  // plays in a loop by itself (cfg.loop: false → only on demand): 0 → 1 in 4.5 s, a pause at the data, again from 0
  const loop = loopPlayer({ dur: 4500, alive: () => cvMain.isConnected, step: t => { st.t = t; slider.value = t; draw(); } });
  const setBtn = () => { playBtn.textContent = loop.playing ? (OVER ? '⏸' : '⏸ pause') : (OVER ? '▶' : '▶ play'); };
  const play = () => { if (loop.playing) loop.stop(); else loop.start(st.t); setBtn(); };
  const reset = (autoplay = cfg.loop !== false) => { loop.stop(); st.t = cfg.t ?? 0; slider.value = st.t; if (autoplay && !isPrint()) loop.start(st.t); setBtn(); };
  // fixed widths for the button and the readout, so that the slider never moves while the text changes
  const playBtn = h('button', { onclick: play, style: OVER ? 'min-width:2.4em; position:absolute; top:6px; right:6px; opacity:0.85' : 'min-width:6.5em' }, OVER ? '▶' : '▶ play');
  const slider = h('input', { type: 'range', min: 0, max: 1, step: 0.01, value: st.t, style: 'width:14em', oninput: e => { loop.stop(); setBtn(); st.t = +e.target.value; draw(); } });
  const readout = h('span', { class: 'dim', style: 'display:inline-block; width:5em; font-variant-numeric:tabular-nums' });
  const cap = txt => h('div', { style: `color:${C.blue}; font-weight:600; margin-bottom:0.3em` }, txt);
  el.classList.add('widget');
  const ctl = h('div', { class: 'wctl', style: 'justify-content:center; gap:0.8em; margin-top:0.4em' },
    h('span', { class: 'interactive-only', style: 'display:flex; gap:0.5em; align-items:center' }, h('label', {}, 'time t'), slider, playBtn), readout);
  if (PANEL === 'both') {
    el.append(h('div', { style: 'display:flex; gap:1.2em; justify-content:center; align-items:flex-start' },
      h('div', {}, cap('Imagine deforming a substance'), cv1), h('div', {}, cap('Probability is our substance'), cv2)), ctl);
  } else if (OVER) { el.style.position = 'relative'; el.append(cvMain, h('span', { class: 'interactive-only' }, playBtn)); }
  else el.append(cvMain, ctl);
  el.closest('.reveal')?.addEventListener('slidechanged', ev => { if (!cvMain.isConnected) return; if (ev.currentSlide?.contains(el)) { reset(); draw(); } else if (loop.playing) { loop.stop(); setBtn(); } });
  if (isPrint() && cfg.print_t !== undefined) { st.t = cfg.print_t; slider.value = st.t; }
  draw();
  if (!isPrint() && el.closest('section')?.classList.contains('present')) reset();   // mounted on the current slide: start now
}
