// Stereo matching on a rendered, rectified stereo pair.
// A small ray-caster renders the same synthetic scene from a left camera at x = 0 and a right camera at x = b
// (same K, both looking along +z; camera frame x right, y down, z forward), so corresponding points lie on the
// same image row. Ground-truth disparity of a left pixel: d = b f / z.
// Block matching: SAD of greyscale windows (box filter via an integral image per disparity), winner-takes-all.
// Optional left–right consistency check marks pixels whose left and right disparities disagree (occlusions,
// ambiguous matches). Click the left image (or a disparity map) to see the window, the scan line and the
// matching cost as a function of d.
// The scene: textured ground, a brick back wall with a uniform (textureless) board, a box, a sphere and a
// panel with periodic stripes (ambiguous matches).
//
// config: {
//   width: 300        displayed width of each of the four tiles (images are 320×240)
//   b: 0.2, f: 300    baseline (m) and focal length (px)
//   maxD: 40          disparity search range 0 … maxD
//   win: 9            window size (odd, slider 3 … 31)
//   click: [186, 168] preset pixel in the left image (null: none)
//   lrcheck: false    left–right consistency check
//   lrButton: true    show the button that toggles the left–right check
//   mode: "flip"      only the two views, switched with buttons or blinking (vertical guide lines show the shift);
//                     print shows them side by side
//   blink: false      (flip) start blinking; it only runs while the slide is shown
// }
import { C, h, fmt, isPrint } from './util.js';
import { LIGHT } from '../theme.js';
import { renderView } from './scene3d.js';

const IW = 320, IH = 240;

// Render one view: camera centre (cx, 0, 0), looking along +z (scene and ray caster in scene3d.js).
const render = (cx, f) => renderView({ c: [cx, 0, 0], f, W: IW, H: IH });

// ---------------- block matching ----------------
// Returns { dl: Int16Array left disparities (−1 = invalid), dr: right disparities }.
function blockMatch(L, R, maxD, win) {
  const N = IW * IH, r = win >> 1;
  const best = new Float32Array(N).fill(Infinity), dl = new Int16Array(N);
  const bestR = new Float32Array(N).fill(Infinity), dr = new Int16Array(N);
  const S = new Float64Array((IW + 1) * (IH + 1));
  const BIG = 255;
  for (let d = 0; d <= maxD; d++) {
    // integral image of |L(x, y) − R(x − d, y)|
    for (let y = 0; y < IH; y++) {
      let row = 0;
      for (let x = 0; x < IW; x++) {
        const c = x - d >= 0 ? Math.abs(L[y * IW + x] - R[y * IW + x - d]) : BIG;
        row += c;
        S[(y + 1) * (IW + 1) + x + 1] = S[y * (IW + 1) + x + 1] + row;
      }
    }
    for (let y = 0; y < IH; y++) {
      const y0 = Math.max(0, y - r), y1 = Math.min(IH - 1, y + r);
      for (let x = 0; x < IW; x++) {
        const x0 = Math.max(0, x - r), x1 = Math.min(IW - 1, x + r);
        const sum = S[(y1 + 1) * (IW + 1) + x1 + 1] - S[y0 * (IW + 1) + x1 + 1] - S[(y1 + 1) * (IW + 1) + x0] + S[y0 * (IW + 1) + x0];
        const c = sum / ((x1 - x0 + 1) * (y1 - y0 + 1)), i = y * IW + x;
        if (c < best[i]) { best[i] = c; dl[i] = d; }
        if (x - d >= 0) { const j = i - d; if (c < bestR[j]) { bestR[j] = c; dr[j] = d; } }
      }
    }
  }
  return { dl, dr };
}

// SAD cost of the window at left pixel (x, y) for every d (for the cost curve)
function costCurve(L, R, x, y, maxD, win) {
  const r = win >> 1, out = [];
  for (let d = 0; d <= maxD; d++) {
    let s = 0, n = 0;
    for (let yy = Math.max(0, y - r); yy <= Math.min(IH - 1, y + r); yy++)
      for (let xx = Math.max(0, x - r); xx <= Math.min(IW - 1, x + r); xx++) {
        s += xx - d >= 0 ? Math.abs(L[yy * IW + xx] - R[yy * IW + xx - d]) : 255; n++;
      }
    out.push(s / n);
  }
  return out;
}

// turbo colormap (polynomial approximation by A. Mikhailov), t in [0, 1]
function turbo(t) {
  t = Math.max(0, Math.min(1, t));
  const r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));
  const g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));
  const b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));
  return [r, g, b].map(v => Math.max(0, Math.min(255, v * 255)));
}

// ---------------------------------------------------------------------------------------------
export function mount(el, cfg) {
  if (cfg.mode === 'flip') return mountFlip(el, cfg);
  const TW = cfg.width || 300, TH = TW * IH / IW;
  const b = cfg.b ?? 0.2, f = cfg.f ?? 300, maxD = cfg.maxD ?? 40;
  const state = { win: cfg.win ?? 9, click: cfg.click === undefined ? [186, 168] : cfg.click, lr: !!cfg.lrcheck };
  const invalid = LIGHT ? [175, 175, 175] : [120, 120, 120]; // grey: not in the colormap

  const left = render(0, f), right = render(b, f);
  const gt = new Float32Array(IW * IH);
  for (let i = 0; i < gt.length; i++) gt[i] = isFinite(left.z[i]) ? b * f / left.z[i] : -1;

  // offscreen sources (native resolution) and display canvases (2× for crisp overlays)
  const src = () => { const c = document.createElement('canvas'); c.width = IW; c.height = IH; return c; };
  const sL = src(), sR = src(), sG = src(), sD = src();
  sL.getContext('2d').putImageData(new ImageData(left.rgba, IW, IH), 0, 0);
  sR.getContext('2d').putImageData(new ImageData(right.rgba, IW, IH), 0, 0);
  const paintDisp = (canvas, disp) => {
    const im = new ImageData(IW, IH);
    for (let i = 0; i < IW * IH; i++) {
      const c = disp[i] < 0 ? invalid : turbo(disp[i] / maxD);
      im.data[4 * i] = c[0]; im.data[4 * i + 1] = c[1]; im.data[4 * i + 2] = c[2]; im.data[4 * i + 3] = 255;
    }
    canvas.getContext('2d').putImageData(im, 0, 0);
  };
  paintDisp(sG, gt);

  const tile = () => h('canvas', { width: IW * 2, height: IH * 2, style: `width:${TW}px; height:${TH}px; cursor:crosshair` });
  const cL = tile(), cR = tile(), cG = tile(), cD = tile();
  const cap = (t) => h('div', { style: 'margin:0 0 0.15em; color:var(--fg)' }, t);
  const plot = h('canvas', { width: 300, height: 170 });
  const stats = h('div', { style: 'line-height:1.45' });
  const bar = h('canvas', { width: 300, height: 46 });

  let disp = null;
  function compute() {
    const t0 = performance.now();
    const { dl, dr } = blockMatch(left.grey, right.grey, maxD, state.win);
    disp = new Int16Array(dl);
    if (state.lr) for (let y = 0; y < IH; y++) for (let x = 0; x < IW; x++) {
      const i = y * IW + x, d = dl[i];
      if (x - d < 0 || Math.abs(dr[i - d] - d) > 1) disp[i] = -1;
    }
    el.dataset.ms = (performance.now() - t0).toFixed(0);
    paintDisp(sD, disp);
    // statistics (left margin x < maxD excluded: no right-image pixels to compare with)
    let bad = 0, inv = 0, n = 0;
    for (let y = 0; y < IH; y++) for (let x = maxD; x < IW; x++) {
      const i = y * IW + x; if (gt[i] < 0) continue;
      n++;
      if (disp[i] < 0) inv++; else if (Math.abs(disp[i] - gt[i]) > 1) bad++;
    }
    stats.innerHTML = `<div>wrong by &gt; 1 px: <b>${fmt(100 * bad / n, 1)}%</b></div>` +
      (state.lr ? `<div>rejected by L–R check: <b>${fmt(100 * inv / n, 1)}%</b></div>` : '');
  }

  function blit(c, s) { const g = c.getContext('2d'); g.imageSmoothingEnabled = false; g.drawImage(s, 0, 0, IW * 2, IH * 2); return g; }
  function draw() {
    const gL = blit(cL, sL), gR = blit(cR, sR), gG = blit(cG, sG), gD = blit(cD, sD);
    const gp = plot.getContext('2d');
    gp.fillStyle = C.bg2; gp.fillRect(0, 0, plot.width, plot.height);
    if (!state.click) { gp.fillStyle = C.dim; gp.font = '19px Inter, Arial'; gp.fillText('click the left image', 60, 90); return; }
    const [x, y] = state.click, r = state.win >> 1, S = 2;
    const i = y * IW + x, dBest = disp[i], dTrue = gt[i];
    const box = (g, cx, color, dash = []) => {
      const rect = [(cx - r) * S, (y - r) * S, (2 * r + 1) * S, (2 * r + 1) * S];
      g.strokeStyle = '#000'; g.lineWidth = 6; g.strokeRect(...rect);
      g.strokeStyle = color; g.lineWidth = 3; g.setLineDash(dash); g.strokeRect(...rect); g.setLineDash([]);
    };
    // left: window; right: scan line, true match (green dashed), best match (accent)
    box(gL, x, C.accent);
    for (const g of [gG, gD]) {
      g.beginPath(); g.arc(x * S + 1, y * S + 1, 9, 0, 7);
      g.strokeStyle = '#000'; g.lineWidth = 6; g.stroke(); g.strokeStyle = '#fff'; g.lineWidth = 3; g.stroke();
    }
    gR.strokeStyle = 'rgba(255,255,255,0.8)'; gR.lineWidth = 2; gR.setLineDash([8, 6]);
    gR.beginPath(); gR.moveTo(0, y * S + 1); gR.lineTo(IW * S, y * S + 1); gR.stroke(); gR.setLineDash([]);
    gR.strokeStyle = 'rgba(255,255,255,0.5)'; gR.lineWidth = 1.5;
    gR.strokeRect((x - maxD - r) * S, (y - r) * S, (maxD + 2 * r + 1) * S, (2 * r + 1) * S);
    if (dTrue >= 0) box(gR, x - Math.round(dTrue), C.green, [6, 5]);
    if (dBest >= 0) box(gR, x - dBest, C.accent);

    // cost curve
    const cost = costCurve(left.grey, right.grey, x, y, maxD, state.win);
    const W = plot.width, H = plot.height, pl = 40, pb = 30, pt = 30;
    const cmax = Math.max(...cost.filter(c => c < 200), 1) * 1.1;
    const px = d => pl + d / maxD * (W - pl - 12), py = c => H - pb - Math.min(c, cmax) / cmax * (H - pb - pt);
    gp.strokeStyle = C.dim; gp.lineWidth = 1;
    gp.beginPath(); gp.moveTo(pl, pt - 6); gp.lineTo(pl, H - pb); gp.lineTo(W - 8, H - pb); gp.stroke();
    if (dTrue >= 0) { gp.strokeStyle = C.green; gp.setLineDash([5, 4]); gp.lineWidth = 2; gp.beginPath(); gp.moveTo(px(dTrue), pt - 6); gp.lineTo(px(dTrue), H - pb); gp.stroke(); gp.setLineDash([]); }
    gp.strokeStyle = C.blue; gp.lineWidth = 2.5; gp.beginPath();
    cost.forEach((c, d) => (d ? gp.lineTo(px(d), py(c)) : gp.moveTo(px(d), py(c)))); gp.stroke();
    const dm = cost.indexOf(Math.min(...cost));
    gp.fillStyle = C.accent; gp.beginPath(); gp.arc(px(dm), py(cost[dm]), 6, 0, 7); gp.fill();
    gp.fillStyle = C.fg; gp.font = '19px Inter, Arial';
    gp.fillText('SAD cost vs disparity d', 8, 20);
    gp.fillStyle = C.dim; gp.font = '18px Inter, Arial';
    gp.fillText('0', pl - 4, H - 10); gp.fillText(String(maxD), W - 28, H - 10); gp.fillText('d', (pl + W) / 2, H - 10);
    gp.fillStyle = C.green; gp.fillText(`true ${fmt(dTrue, 1)}`, Math.min(px(dTrue) + 5, W - 80), pt + 10);
    gp.fillStyle = C.accent; gp.fillText(`best ${dm}`, Math.min(px(dm) + 8, W - 60), Math.max(pt + 28, py(cost[dm]) - 8));
  }

  function colorbar() {
    const g = bar.getContext('2d'), W = bar.width;
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, bar.height);
    for (let x = 0; x < W - 20; x++) { const c = turbo(x / (W - 21)); g.fillStyle = `rgb(${c.map(Math.round).join(',')})`; g.fillRect(10 + x, 4, 1, 16); }
    g.fillStyle = C.dim; g.font = '18px Inter, Arial';
    g.fillText(`d = 0 (far)`, 10, 40); g.fillText(`${maxD} px (near)`, W - 110, 40);
  }

  // clicks on the left image or the disparity maps (coordinates corrected for reveal.js scaling)
  for (const c of [cL, cG, cD]) c.addEventListener('click', e => {
    const r = c.getBoundingClientRect();
    state.click = [Math.min(IW - 1, Math.floor((e.clientX - r.left) / r.width * IW)), Math.min(IH - 1, Math.floor((e.clientY - r.top) / r.height * IH))];
    draw();
  });

  const val = h('span', { class: 'readout', style: 'display:inline-block; width:4.5em; text-align:right' });
  const s = h('input', { type: 'range', min: 3, max: 31, step: 2, value: state.win });
  const setWin = () => { val.textContent = `${state.win}×${state.win}`; };
  s.addEventListener('input', () => { state.win = parseInt(s.value, 10); setWin(); compute(); draw(); });
  const lrBtn = h('button', { onclick: () => { state.lr = !state.lr; lrBtn.classList.toggle('active', state.lr); compute(); draw(); } }, 'left–right check');
  lrBtn.classList.toggle('active', state.lr);
  setWin();

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `display:grid; grid-template-columns:${TW}px ${TW}px; gap:0.3em 0.6em` },
      h('div', {}, cap('left image'), cL), h('div', {}, cap('right image'), cR),
      h('div', {}, cap('true disparity'), cG), h('div', {}, cap('block matching'), cD)),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.45em; width:15em; flex:none' },
      h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', {}, 'window'), val),
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.3em; margin-top:0' }, s, ...(cfg.lrButton === false ? [] : [lrBtn])),
      plot, stats, bar,
      h('div', { class: 'dim interactive-only' }, 'Click the left image.'))));
  compute(); colorbar(); draw();
}

// The two views of the stereo pair, one at a time: near objects jump, far ones barely move.
function mountFlip(el, cfg) {
  const TW = cfg.width || 560, TH = TW * IH / IW;
  const b = cfg.b ?? 0.2, f = cfg.f ?? 300;
  const views = [render(0, f), render(b, f)].map(v => {
    const c = document.createElement('canvas'); c.width = IW; c.height = IH;
    c.getContext('2d').putImageData(new ImageData(v.rgba, IW, IH), 0, 0);
    return c;
  });
  const names = ['left camera', 'right camera'];
  const state = { view: 0, guides: true, blink: null };

  const draw = (canvas, k) => {
    const g = canvas.getContext('2d');
    g.imageSmoothingEnabled = true;
    g.drawImage(views[k], 0, 0, canvas.width, canvas.height);
    const s = canvas.width / IW;
    if (state.guides) {
      g.strokeStyle = 'rgba(255,255,255,0.35)'; g.lineWidth = 1;
      for (let x = 40; x < IW; x += 40) { g.beginPath(); g.moveTo(x * s + 0.5, 0); g.lineTo(x * s + 0.5, canvas.height); g.stroke(); }
    }
    g.font = `${Math.round(13 * s)}px Inter, Arial`;
    const tw = g.measureText(names[k]).width;
    g.fillStyle = 'rgba(13,15,19,0.75)'; g.fillRect(6 * s, 6 * s, tw + 10 * s, 20 * s);
    g.fillStyle = k ? C.blue : C.accent; g.fillText(names[k], 11 * s, 21 * s);
  };

  el.classList.add('widget');
  if (isPrint()) {
    const w = Math.round(TW * 0.62);
    const cs = [0, 1].map(k => { const c = h('canvas', { width: IW * 2, height: IH * 2, style: `width:${w}px; height:${w * IH / IW}px` }); draw(c, k); return c; });
    el.append(h('div', { style: 'display:flex; gap:0.6em' }, ...cs));
    return;
  }
  const canvas = h('canvas', { width: IW * 2, height: IH * 2, style: `width:${TW}px; height:${TH}px` });
  const btn = names.map((n, k) => h('button', { onclick: () => { stopBlink(); show(k); } }, n));
  const blinkBtn = h('button', { onclick: () => (state.blink ? stopBlink() : startBlink()) }, 'blink');
  const guideBtn = h('button', { class: 'active', onclick: () => { state.guides = !state.guides; guideBtn.classList.toggle('active', state.guides); show(state.view); } }, 'guide lines');
  function show(k) { state.view = k; draw(canvas, k); btn.forEach((x, i) => x.classList.toggle('active', i === k)); }
  function startBlink() {
    blinkBtn.classList.add('active');
    state.blink = setInterval(() => {
      if (!el.isConnected) return stopBlink();
      if (el.closest('section')?.classList.contains('present') === false) return;   // slide not shown
      show(1 - state.view);
    }, cfg.period ?? 600);
  }
  function stopBlink() { clearInterval(state.blink); state.blink = null; blinkBtn.classList.remove('active'); }
  el.append(canvas, h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, ...btn, blinkBtn, guideBtn));
  show(0);
  if (cfg.blink) startBlink();
}
