// The projection x_w → pixel in steps, for students to walk through: (1) the point in the world frame W,
// (2) the same point in the camera frame C: x_c = R x_w + t, (3) division by the depth: the normalized image
// plane z_c = 1, (4) K: pixels, with the pixel frame (origin top-left, u right, v down) on that plane and in the
// image. Every step shows its formula with numbers. Print shows the last step.
// config: { X: [x, y, z] (world point), C: [x, y, z], target: [x, y, z], f, ox, oy, step (0–3), width, height }   (world: z up)
import { C as COL, h, fmt, isPrint } from './util.js';
import { THREE, makeStage, label, line, arrow3, viewButtons } from './three-util.js';
import { T } from './gripper.js';

const sub = (a, b) => a.map((v, i) => v - b[i]);
const add = (a, b) => a.map((v, i) => v + b[i]);
const scl = (a, s) => a.map(v => v * s);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => scl(a, 1 / Math.hypot(...a));
const hex = c => '#' + c.toString(16).padStart(6, '0');
const AX = [0xff6b6b, 0x5fd38d, 0x5ab0ff];
const DIM = 0x5a606c;

const STEPS = [
  'the point in the world frame $W$',
  'move to the camera frame $C$: $\\;x_c = R\\,x_w + t$',
  'divide by the depth $z_c$: the plane $z_c = 1$',
  'apply $K$: pixels',
];

export function mount(el, cfg) {
  const Xw = cfg.X || [0.9, 0.7, 0.8];
  const Cw = cfg.C || [2.0, -2.2, 1.7];
  const target = cfg.target || [-0.5, 0.9, -0.3];
  const f = cfg.f ?? 500, ox = cfg.ox ?? 320, oy = cfg.oy ?? 240, IW = 640, IH = 480;
  const W = cfg.width || 520, H = cfg.height || 330;
  const state = { step: isPrint() ? 3 : (cfg.step ?? 0) };

  // camera: rows of R are the camera axes in world coordinates (x right, y down, z forward)
  const fwd = norm(sub(target, Cw)), right = norm(cross(fwd, [0, 0, 1])), down = cross(fwd, right);
  const R = [right, down, fwd];
  const t = R.map(r => -dot(r, Cw));
  const Xc = R.map((r, i) => dot(r, Xw) + t[i]);
  const xn = [Xc[0] / Xc[2], Xc[1] / Xc[2], 1];
  const uv = [f * xn[0] + ox, f * xn[1] + oy];
  const toWorld = pc => add(Cw, add(add(scl(R[0], pc[0]), scl(R[1], pc[1])), scl(R[2], pc[2])));

  // ---- 3D view ----
  const canvas = h('canvas', { width: W, height: H });
  const view3d = { label: '3D view', position: cfg.camera || T(4.3, -2.2, 3.5), target: cfg.lookAt || T(1.2, -0.8, 0.7), fov: 42 };
  const viewCam = { label: 'behind the camera', position: T(...add(Cw, add(scl(fwd, -2.2), [0, 0, 0.5]))), target: T(...add(Cw, scl(fwd, 1))), fov: 50 };
  const stage = makeStage(canvas, { position: view3d.position, target: view3d.target });
  const { scene, render } = stage;
  const lab = (group, text, p, color, size = 0.3, s = '') => { const l = label(text, { color, size, sub: s }); l.position.set(...T(...p)); group.add(l); };
  const grid = new THREE.GridHelper(8, 8, 0x3a3f4b, 0x2a2e36); grid.position.set(...T(0.5, -1, 0)); scene.add(grid);
  const dyn = new THREE.Group(); scene.add(dyn);

  function frame(origin, axes, len, sub_, active, name, nameAt) {
    axes.forEach((d, i) => {
      const col = active ? AX[i] : DIM;
      dyn.add(arrow3(T(...origin), T(...add(origin, scl(d, len))), col, 0.13));
      lab(dyn, ['x', 'y', 'z'][i], add(origin, scl(d, len * 1.15)), hex(col), active ? 0.4 : 0.3, sub_);
    });
    lab(dyn, name, nameAt, active ? COL.fg : hex(DIM), 0.4);
  }
  // dashed path from `origin` along `axes` with the given components
  function staircase(origin, axes, comps) {
    let p = origin;
    comps.forEach((c, i) => { const q = add(p, scl(axes[i], c)); dyn.add(line([T(...p), T(...q)], AX[i], { dashed: true })); p = q; });
  }

  function draw3d() {
    dyn.clear();
    const s = state.step;
    const E = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    frame([0, 0, 0], E, 1.1, 'w', s === 0, 'W', [-0.3, -0.3, -0.1]);
    frame(Cw, R, 0.7, 'c', s >= 1, 'C', add(Cw, [0, 0.1, 0.4]));
    const pin = new THREE.Mesh(new THREE.SphereGeometry(0.06, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff }));
    pin.position.set(...T(...Cw)); dyn.add(pin);

    // the point
    const P = new THREE.Mesh(new THREE.SphereGeometry(0.08, 16, 12), new THREE.MeshStandardMaterial({ color: 0xf2b134 }));
    P.position.set(...T(...Xw)); dyn.add(P);
    lab(dyn, 'x', add(Xw, [0.15, 0, 0.25]), COL.accent, 0.4, s === 0 ? 'w' : 'c');
    if (s === 0) staircase([0, 0, 0], E, Xw);
    if (s === 1) staircase(Cw, R, Xc);

    // normalized image plane z_c = 1, the ray, the intersection
    if (s >= 2) {
      const hw = IW / 2 / f, hh = IH / 2 / f, x0 = -ox / f, y0 = -oy / f;
      const cs = [[x0, y0], [x0 + IW / f, y0], [x0 + IW / f, y0 + IH / f], [x0, y0 + IH / f]].map(([x, y]) => toWorld([x, y, 1]));
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(cs.flatMap(p => T(...p)), 3));
      geo.setIndex([0, 1, 2, 0, 2, 3]);
      dyn.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x9aa1ae, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false })));
      dyn.add(line([...cs, cs[0]].map(p => T(...p)), 0x9aa1ae));
      cs.forEach(c => dyn.add(line([T(...Cw), T(...c)], 0x4a505c)));
      dyn.add(line([T(...Cw), T(...Xw)], 0xf2b134));
      const q = toWorld(xn);
      const Q = new THREE.Mesh(new THREE.SphereGeometry(0.06, 16, 12), new THREE.MeshBasicMaterial({ color: 0xb28dff }));
      Q.position.set(...T(...q)); dyn.add(Q);
      if (s === 2) {
        lab(dyn, 'normalized plane', add(cs[3], scl(R[1], 0.2)), COL.dim, 0.24);
        staircase(Cw, R, xn);
      }
      if (s === 3) {
        // pixel frame: origin at the top-left corner, u along x_c, v along y_c
        dyn.add(arrow3(T(...cs[0]), T(...add(cs[0], scl(R[0], 0.6))), 0xe7e9ee, 0.1));
        dyn.add(arrow3(T(...cs[0]), T(...add(cs[0], scl(R[1], 0.6))), 0xe7e9ee, 0.1));
        lab(dyn, 'u', add(cs[0], scl(R[0], 0.72)), COL.fg, 0.3);
        lab(dyn, 'v', add(cs[0], scl(R[1], 0.72)), COL.fg, 0.3);
        const pp = toWorld([0, 0, 1]);
        const PP = new THREE.Mesh(new THREE.SphereGeometry(0.04, 12, 10), new THREE.MeshBasicMaterial({ color: 0xe7e9ee }));
        PP.position.set(...T(...pp)); dyn.add(PP);
        dyn.add(line([T(...Cw), T(...pp)], 0x5ab0ff, { dashed: true }));
      }
      if (s === 2) lab(dyn, 'x', add(q, scl(R[1], -0.2)), COL.purple, 0.34, 'n');
      else lab(dyn, '(u, v)', add(q, scl(R[1], -0.2)), COL.purple, 0.3);
    }
    render();
  }

  // ---- image inset (pixels) ----
  const S2 = 0.36, img = h('canvas', { width: IW * S2, height: IH * S2, style: `width:${IW * S2}px; height:${IH * S2}px; border:1px solid #3a3f4b; border-radius:4px` });
  const g = img.getContext('2d');
  const proj = p => { const c = R.map((r, i) => dot(r, p) + t[i]); return c[2] > 0.05 ? [(f * c[0] / c[2] + ox) * S2, (f * c[1] / c[2] + oy) * S2] : null; };
  function drawImage() {
    g.fillStyle = '#0d0f13'; g.fillRect(0, 0, img.width, img.height);
    const s = state.step;
    if (s < 3) {
      g.fillStyle = COL.dim; g.font = '15px Inter, Arial'; g.textAlign = 'center';
      g.fillText('the image appears in step 4', img.width / 2, img.height / 2); g.textAlign = 'left';
      return;
    }
    const seg = (a, b, col, w = 1) => { const A = proj(a), B = proj(b); if (!A || !B) return; g.strokeStyle = col; g.lineWidth = w; g.beginPath(); g.moveTo(...A); g.lineTo(...B); g.stroke(); };
    for (let i = -4; i <= 4; i++) { seg([i + 0.5, -5, 0], [i + 0.5, 3, 0], '#2a2e36'); seg([-3.5, i - 1, 0], [4.5, i - 1, 0], '#2a2e36'); }
    [[1.1, 0, 0], [0, 1.1, 0], [0, 0, 1.1]].forEach((d, i) => seg([0, 0, 0], d, hex(AX[i]), 2));
    // pixel frame and principal point
    g.strokeStyle = COL.fg; g.fillStyle = COL.fg; g.lineWidth = 2; g.font = 'italic 16px "Times New Roman", serif';
    g.beginPath(); g.moveTo(2, 2); g.lineTo(46, 2); g.moveTo(2, 2); g.lineTo(2, 46); g.stroke();
    g.fillText('u', 50, 14); g.fillText('v', 8, 60);
    const [px, py] = [ox * S2, oy * S2];
    g.beginPath(); g.moveTo(px - 6, py); g.lineTo(px + 6, py); g.moveTo(px, py - 6); g.lineTo(px, py + 6); g.stroke();
    g.fillStyle = COL.dim; g.font = '12px Inter, Arial'; g.fillText('principal point', px + 6, py + 16);
    const [u, v] = [uv[0] * S2, uv[1] * S2];
    g.strokeStyle = COL.purple; g.setLineDash([4, 3]); g.lineWidth = 1.2;
    g.beginPath(); g.moveTo(u, 0); g.lineTo(u, v); g.moveTo(0, v); g.lineTo(u, v); g.stroke(); g.setLineDash([]);
    g.fillStyle = COL.accent; g.beginPath(); g.arc(u, v, 5, 0, 7); g.fill();
    g.fillStyle = COL.purple; g.font = '13px Inter, Arial'; const txt = `(${Math.round(uv[0])}, ${Math.round(uv[1])})`, tw = g.measureText(txt).width;
    g.fillText(txt, u + 8 + tw > img.width - 4 ? u - 8 - tw : u + 8, v - 8);
  }

  // ---- formulas ----
  const chain = h('div', { style: 'font-size:1.1em' });
  const formula = h('div', { style: 'font-size:0.95em; min-height:5.6em' });
  const title = h('div', { style: 'font-size:0.9em; color:' + COL.accent });
  const m = v => fmt(v, 2), col = v => `\\begin{bmatrix}${v.join('\\\\')}\\end{bmatrix}`;
  function drawText() {
    const s = state.step;
    const c = (k, tex) => (k === s ? `\\textcolor{${COL.accent}}{${tex}}` : k < s ? tex : `\\textcolor{${COL.dim}}{${tex}}`);
    chain.innerHTML = `$${c(0, 'x_w')} \\xrightarrow{${c(1, '[R\\,|\\,t]')}} ${c(1, 'x_c')} \\xrightarrow{${c(2, '\\div z_c')}} ${c(2, 'x_n')} \\xrightarrow{${c(3, 'K')}} ${c(3, '(u, v)')}$`;
    title.innerHTML = `step ${s + 1} / 4: ${STEPS[s]}`;
    formula.innerHTML = [
      `$x_w = ${col(Xw.map(m))}$ <span class="small">coordinates along the world axes (dashed)</span>`,
      `$x_c = R\\,x_w + t = ${col(Xc.map(m))}$ <span class="small">the same point, along the camera axes</span>`,
      `$x_n = \\dfrac{x_c}{z_c} = ${col(xn.map(m))}$ <span class="small">on the ray, at depth 1; the depth is lost</span>`,
      `$\\begin{bmatrix}u\\\\v\\\\1\\end{bmatrix} = \\underbrace{\\begin{bmatrix}${f}&0&${ox}\\\\0&${f}&${oy}\\\\0&0&1\\end{bmatrix}}_{K} x_n = ${col([Math.round(uv[0]), Math.round(uv[1]), 1])}$`,
    ][s];
    for (const e of [chain, title, formula]) window.renderMathInElement?.(e, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
  }

  function update() { draw3d(); drawImage(); drawText(); prev.disabled = state.step === 0; next.disabled = state.step === 3; }
  const prev = h('button', { onclick: () => { state.step = Math.max(0, state.step - 1); update(); } }, '◀ back');
  const next = h('button', { onclick: () => { state.step = Math.min(3, state.step + 1); update(); } }, 'next step ▶');

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.9em; align-items:flex-start' },
    h('div', {}, canvas, h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, viewButtons(stage, [view3d, viewCam]))),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.35em; width:24em; flex:none' },
      chain, title,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em; margin-top:0' }, prev, next),
      formula, img)));
  update();
}
