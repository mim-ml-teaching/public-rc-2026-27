// Rectified stereo seen from above: two cameras at x = 0 (left) and x = b (right), both looking along +z.
// Drag the point P: its pixel column in each camera, the disparity d = u_l − u_r and the depth z = b f / d.
// With "matching error" on, each pixel is only known to ±e px: the region of 3D points consistent with both
// measurements (quadrilateral between the four boundary rays) grows like z².
// The x axis is stretched (factor written on the canvas) so that a 0.3 m baseline and a 10 m depth fit together;
// all rays stay straight lines under this scaling.
//
// config: {
//   width: 600, height: 460     canvas size
//   b: 0.3                      baseline in metres (slider bMin … 1)
//   bMin: 0.1                   smallest baseline on the slider
//   stretch: 8                  stretch factor of the x axis
//   f: 500                      focal length in pixels; images are 640 px wide, o_x = 320
//   point: [0.8, 7]             P = (x, z) in the left camera frame, metres
//   uncertainty: true           show the uncertainty region and the inset plot
//   err: 1                      matching error ±e in pixels (slider 0.25 … 3)
// }
import { C, h, fmt, arrow } from './util.js';

const IW = 640, OX = 320, ZMAX = 11, ZMIN = 0.5;

export function mount(el, cfg) {
  const W = cfg.width || 600, H = cfg.height || 460;
  const state = {
    b: cfg.b ?? 0.3, f: cfg.f ?? 500, P: (cfg.point || [0.8, 7]).slice(),
    unc: cfg.uncertainty ?? true, err: cfg.err ?? 1, drag: false,
  };
  const canvas = h('canvas', { width: W, height: H, style: 'touch-action:none; cursor:grab' });
  const g = canvas.getContext('2d');
  const readout = h('div', { style: 'font-size:0.95em; line-height:1.6' });

  // ---- scene ↔ canvas mapping (cameras at the bottom, z up, x stretched) ----
  const Y0 = H - 140, YTOP = 22;
  const sz = (Y0 - YTOP) / ZMAX, STRETCH = cfg.stretch ?? 8, sx = sz * STRETCH;
  const cx = () => W / 2 - 40;                              // canvas x of the baseline midpoint
  const X = x => cx() + (x - state.b / 2) * sx;
  const Y = z => Y0 - z * sz;
  const toScene = ([u, v]) => [(u - cx()) / sx + state.b / 2, (Y0 - v) / sz];

  const proj = (x, z, camX) => state.f * (x - camX) / z + OX;   // pixel column in a camera at camX
  // intersection of the left ray through column u1 and the right ray through column u2
  const meet = (u1, u2) => { const d = u1 - u2; if (d <= 1e-9) return null; const z = state.b * state.f / d; return [z * (u1 - OX) / state.f, z]; };

  function rulers(ul, ur) {
    const x0 = 150, x1 = W - 24, s = (x1 - x0) / IW;
    const row = (y, name, u, color) => {
      g.fillStyle = C.fg; g.font = '22px Inter, Arial'; g.textBaseline = 'middle';
      g.fillText(name, 12, y);
      g.fillStyle = C.bg3; g.fillRect(x0, y - 9, x1 - x0, 18);
      g.strokeStyle = C.line; g.lineWidth = 1; g.strokeRect(x0 + 0.5, y - 9.5, x1 - x0, 18);
      g.fillStyle = C.dim; g.font = '18px Inter, Arial'; g.textAlign = 'center';
      for (let k = 0; k <= IW; k += 160) {
        g.fillRect(x0 + k * s - 0.5, y + 9, 1, 5);
        g.fillText(String(k), x0 + k * s, y + 22);
      }
      g.textAlign = 'left';
      if (u >= 0 && u <= IW) {
        const p = x0 + u * s;
        g.fillStyle = color; g.fillRect(p - 2, y - 13, 4, 26);
        g.font = 'italic 20px Inter, Arial'; g.textAlign = p > x1 - 90 ? 'right' : 'left';
        g.fillText(`${name[0] === 'l' ? 'u_l' : 'u_r'} = ${fmt(u, 1)}`, p + (g.textAlign === 'right' ? -8 : 8), y - 22);
        g.textAlign = 'left';
      } else {
        g.fillStyle = C.red; g.font = '18px Inter, Arial'; g.fillText('not in view', x0 + 6, y);
      }
    };
    row(H - 92, 'left image', ul, C.blue);
    row(H - 36, 'right image', ur, C.green);
  }

  // inset: depth uncertainty as a function of z (own canvas in the side panel)
  const plot = h('canvas', { width: 300, height: 150 });
  const gp = plot.getContext('2d');
  function inset() {
    plot.style.display = state.unc ? 'block' : 'none';
    if (!state.unc) return;
    const w = plot.width, hh = plot.height, pad = 34;
    gp.fillStyle = C.bg2; gp.fillRect(0, 0, w, hh);
    const dz = z => 2 * state.err * z * z / (state.b * state.f); // ≈ z_far − z_near
    const ymax = dz(ZMAX);
    const px = z => pad + z / ZMAX * (w - pad - 12), py = v => hh - 26 - Math.min(v, ymax) / ymax * (hh - 66);
    gp.strokeStyle = C.dim; gp.lineWidth = 1;
    gp.beginPath(); gp.moveTo(px(0), py(ymax)); gp.lineTo(px(0), py(0)); gp.lineTo(px(ZMAX), py(0)); gp.stroke();
    gp.strokeStyle = C.accent; gp.lineWidth = 2.5; gp.beginPath();
    for (let i = 0; i <= 60; i++) { const z = ZMAX * i / 60; i ? gp.lineTo(px(z), py(dz(z))) : gp.moveTo(px(z), py(dz(z))); }
    gp.stroke();
    const z = state.P[1];
    gp.fillStyle = C.red; gp.beginPath(); gp.arc(px(z), py(dz(z)), 6, 0, 7); gp.fill();
    gp.fillStyle = C.fg; gp.font = '21px Inter, Arial'; gp.textBaseline = 'alphabetic';
    gp.fillText('depth uncertainty Δz vs z', 8, 22);
    gp.fillStyle = C.dim; gp.font = '19px Inter, Arial';
    gp.fillText('z', px(ZMAX) - 4, hh - 6); gp.fillText('0', px(0) - 4, hh - 6); gp.fillText(`${ZMAX} m`, px(ZMAX) - 50, hh - 6);
    gp.fillText(`${fmt(ymax, 1)} m`, 2, py(ymax) + 14);
  }

  function draw() {
    const { b, P } = state;
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    // depth grid
    g.font = '18px Inter, Arial'; g.textBaseline = 'middle';
    for (let z = 2; z <= 10; z += 2) {
      g.strokeStyle = '#262a33'; g.lineWidth = 1; g.beginPath(); g.moveTo(0, Y(z)); g.lineTo(W, Y(z)); g.stroke();
      g.fillStyle = C.dim; g.fillText(`z = ${z} m`, 8, Y(z) - 9);
    }
    // axes (camera frame, seen from above)
    const ax = W - 80;
    arrow(g, ax, Y0 - 6, ax + 46, Y0 - 6, C.red, 2.5);
    arrow(g, ax, Y0 - 6, ax, Y0 - 52, C.blue, 2.5);
    g.font = 'italic 22px "Times New Roman", serif';
    g.fillStyle = C.red; g.fillText('x', ax + 50, Y0 - 6);
    g.fillStyle = C.blue; g.fillText('z', ax + 7, Y0 - 50);

    const ul = proj(P[0], P[1], 0), ur = proj(P[0], P[1], b);
    // rays from the cameras to P
    g.lineWidth = state.unc ? 1.2 : 2;
    g.strokeStyle = C.blue; g.beginPath(); g.moveTo(X(0), Y0); g.lineTo(X(P[0]), Y(P[1])); g.stroke();
    g.strokeStyle = C.green; g.beginPath(); g.moveTo(X(b), Y0); g.lineTo(X(P[0]), Y(P[1])); g.stroke();
    // uncertainty region
    if (state.unc) {
      const e = state.err;
      const near = meet(ul + e, ur - e), far = meet(ul - e, ur + e), s1 = meet(ul - e, ur - e), s2 = meet(ul + e, ur + e);
      const zCap = ZMAX * 1.6;
      const farPt = far && far[1] < zCap ? far : (() => { const z = zCap; return [z * (ul - e - OX) / state.f, z]; })();
      if (near && s1 && s2) {
        // the region is often only a few pixels wide: thick outline, and bars at its near and far ends
        g.fillStyle = 'rgba(242,177,52,0.55)'; g.strokeStyle = C.accent; g.lineWidth = 5; g.lineJoin = 'round';
        g.beginPath(); [near, s1, farPt, s2].forEach(([x, z], i) => (i ? g.lineTo(X(x), Y(z)) : g.moveTo(X(x), Y(z)))); g.closePath(); g.fill(); g.stroke();
        const dir = [X(farPt[0]) - X(near[0]), Y(farPt[1]) - Y(near[1])], dl = Math.hypot(...dir) || 1, nx = -dir[1] / dl * 11, ny = dir[0] / dl * 11;
        g.lineWidth = 3;
        for (const [x, z] of [near, farPt]) { g.beginPath(); g.moveTo(X(x) - nx, Y(z) - ny); g.lineTo(X(x) + nx, Y(z) + ny); g.stroke(); }
        g.fillStyle = C.accent; g.font = '19px Inter, Arial';
        const zf = far && far[1] < zCap ? fmt(far[1], 1) : '∞';
        g.fillText(`z ∈ [${fmt(near[1], 1)}, ${zf}] m`, Math.max(X(s1[0]), X(s2[0])) + 14, Y(P[1]) + 22);
        // boundary rays (faint)
        g.strokeStyle = 'rgba(242,177,52,0.35)'; g.lineWidth = 1;
        for (const [cam, u] of [[0, ul - e], [0, ul + e], [b, ur - e], [b, ur + e]]) {
          const z = Math.min(zCap, farPt[1] + 0.6); g.beginPath(); g.moveTo(X(cam), Y0); g.lineTo(X(cam + z * (u - OX) / state.f), Y(z)); g.stroke();
        }
      }
    }
    // cameras: small glyphs (not to scale) with the image line on top
    // (close cameras: narrower glyphs, labels pushed outwards so they do not overlap)
    const sep = X(b) - X(0), gw = Math.max(5, Math.min(13, sep / 2 - 1)), close = sep < 56;
    for (const [cam, col, name, side] of [[0, C.blue, 'left', -1], [b, C.green, 'right', 1]]) {
      g.fillStyle = col; g.beginPath(); g.moveTo(X(cam), Y0); g.lineTo(X(cam) - gw, Y0 + 20); g.lineTo(X(cam) + gw, Y0 + 20); g.closePath(); g.fill();
      g.fillStyle = C.fg; g.font = '18px Inter, Arial';
      if (close) { g.textAlign = side < 0 ? 'right' : 'left'; g.fillText(name, X(cam) + side * (gw + 4), Y0 + 32); }
      else { g.textAlign = 'center'; g.fillText(name, X(cam), Y0 + 32); }
      g.textAlign = 'left';
    }
    // baseline
    g.strokeStyle = C.fg; g.lineWidth = 1.5; g.beginPath(); g.moveTo(X(0), Y0 + 3); g.lineTo(X(b), Y0 + 3); g.stroke();
    g.fillStyle = C.fg; g.font = 'italic 21px "Times New Roman", serif'; g.textAlign = 'center';
    g.fillText('b', (X(0) + X(b)) / 2, Y0 - 10); g.textAlign = 'left';
    // the point
    g.beginPath(); g.arc(X(P[0]), Y(P[1]), state.unc ? 5 : 7, 0, 7);
    if (state.unc) { g.strokeStyle = C.red; g.lineWidth = 2.5; g.stroke(); }
    else { g.fillStyle = C.red; g.fill(); g.strokeStyle = C.bg2; g.lineWidth = 2; g.stroke(); }
    g.fillStyle = C.red; g.font = 'italic 23px "Times New Roman", serif'; g.fillText('P', X(P[0]) + 11, Y(P[1]) - 8);
    // stretch note
    g.fillStyle = C.dim; g.font = '18px Inter, Arial'; g.textAlign = 'right'; g.fillText(`top view · x axis stretched ×${STRETCH}`, W - 10, 22); g.textAlign = 'left';

    inset();
    rulers(ul, ur);

    const d = ul - ur, z = b * state.f / d, dd = 2 * state.err;
    readout.innerHTML =
      `<div>$u_l = f\\,x/z + o_x = ${fmt(ul, 1)}$</div>` +
      `<div>$u_r = f\\,(x - b)/z + o_x = ${fmt(ur, 1)}$</div>` +
      `<div style="margin-top:0.3em">$d = u_l - u_r = ${fmt(d, 1)}$ px</div>` +
      `<div>$z = \\dfrac{b\\,f}{d} = \\dfrac{${fmt(b, 2)} \\cdot ${state.f}}{${fmt(d, 1)}} = ${fmt(z, 2)}$ m</div>` +
      (state.unc ? `<div style="margin-top:0.3em; color:${C.accent}">$\\Delta z \\approx \\dfrac{z^2}{b f}\\,\\Delta d = ${fmt(z * z * dd / (b * state.f), 2)}$ m</div>` +
        `<div class="dim" style="font-size:0.85em">$\\Delta d = 2e = ${fmt(dd, 1)}$ px (each pixel $\\pm ${fmt(state.err, 1)}$)</div>` : '');
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
  }

  // ---- dragging (coordinates corrected for reveal.js scaling) ----
  const pos = e => { const r = canvas.getBoundingClientRect(); return [(e.clientX - r.left) / r.width * W, (e.clientY - r.top) / r.height * H]; };
  const setP = e => {
    let [x, z] = toScene(pos(e));
    z = Math.max(ZMIN, Math.min(ZMAX, z));
    const xr = (W / 2) / sx;
    x = Math.max(state.b / 2 - xr, Math.min(state.b / 2 + xr * 0.6, x));
    state.P = [x, z]; draw();
  };
  canvas.addEventListener('pointerdown', e => {
    const [u, v] = pos(e);
    if (Math.hypot(u - X(state.P[0]), v - Y(state.P[1])) < 28 || v < Y0 - 4) {
      state.drag = true; canvas.setPointerCapture(e.pointerId); e.stopPropagation(); setP(e);
    }
  });
  canvas.addEventListener('pointermove', e => { if (state.drag) setP(e); });
  canvas.addEventListener('pointerup', () => { state.drag = false; });

  const slider = (key, min, max, step, name, unit) => {
    const s = h('input', { type: 'range', min, max, step, value: state[key] });
    const val = h('span', { class: 'readout', style: 'display:inline-block; width:3.6em; text-align:right' }, fmt(state[key], 2) + unit);
    s.addEventListener('input', () => { state[key] = parseFloat(s.value); val.textContent = fmt(state[key], 2) + unit; draw(); });
    return h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', { style: 'width:4.2em' }, name), s, val);
  };
  const uncBtn = h('button', { onclick: () => { state.unc = !state.unc; uncBtn.classList.toggle('active', state.unc); draw(); } }, 'matching error');
  uncBtn.classList.toggle('active', state.unc);

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    canvas,
    h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:15em; flex:none' },
      readout, plot,
      h('div', { class: 'dim' }, 'Drag P.'),
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.3em' },
        slider('b', cfg.bMin ?? 0.1, 1, 0.01, 'baseline', ' m'),
        h('div', { style: 'display:flex; gap:0.3em' }, uncBtn),
        slider('err', 0.25, 3, 0.25, 'error ±', ' px')))));
  draw();
}
