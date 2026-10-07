// Pinhole camera: a 3D world (left) and the image the camera records (right), computed with
// our own projection u = fx x_c / z_c + o_x, v = fy y_c / z_c + o_y (not with three.js' camera).
//
// World frame: z up, ground plane z = 0. Camera frame (OpenCV): x right, y down, z forward.
// p_c = R (p_w − C), R rows = (right, down, forward) of the camera.
//
// config: {
//   scene: "house" | "rails" | "ambiguity"
//   controls: ["f", "o", "pose", "rays", "physical", "click"]   which controls to show
//   f: 500, ox: 320, oy: 240, yaw: 90, pitch: 0, pos: [0, -5, 1.2]
//   railYaw: 0          rails direction (degrees from +y, in the ground plane)
//   showVP: false       vanishing point + horizon (rails scene)
//   physical: false     show the physical (inverted) image plane behind the pinhole
//   rays: false         projection rays from the house corners to the pinhole
//   click: [u, v]       an initial back-projected pixel
// }
import { C as COL, h, fmt, isPrint } from './util.js';
import { THREE, makeStage, label, line, arrow3, viewButtons } from './three-util.js';

const deg = Math.PI / 180;
const T = (x, y, z) => [x, z, -y];              // world (z up) -> three.js (y up)
const IW = 640, IH = 480;                       // sensor size in pixels

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => scl(a, 1 / Math.hypot(...a));

// ---------------- scenes: lists of coloured 3D segments in world coordinates ----------------
// A face: polygon in world coordinates, outward normal, colour; decals are drawn right on top of it.
function face(pts, color, inside, decals = []) {
  let n = norm(cross(sub(pts[1], pts[0]), sub(pts[2], pts[0])));
  if (dot(n, sub(pts[0], inside)) < 0) { pts = pts.slice().reverse(); n = scl(n, -1); }
  return { pts, color, n, decals };
}
function boxFaces(x0, x1, y0, y1, z0, z1, color, decalsByFace = {}) {
  const c = [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2];
  return [
    ['front', [[x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1]]],
    ['back', [[x0, y1, z0], [x1, y1, z0], [x1, y1, z1], [x0, y1, z1]]],
    ['left', [[x0, y0, z0], [x0, y1, z0], [x0, y1, z1], [x0, y0, z1]]],
    ['right', [[x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z1]]],
    ['top', [[x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]]],
  ].map(([k, P]) => face(P, color, c, decalsByFace[k] || []));
}

function houseScene(solid = false) {
  const segs = [], pts = [];
  const box = (x0, x1, y0, y1, z0, z1, c) => {
    const P = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]];
    if (!solid) [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]]
      .forEach(([a, b]) => segs.push([P[a], P[b], c]));
    return P;
  };
  const P = box(-1, 1, 1, 3, 0, 1.4, 0xe9dcc0);
  pts.push(...P.slice(4));
  const ridge = [[-1, 2, 2.2], [1, 2, 2.2]];
  pts.push(...ridge);
  let faces = [];
  if (solid) {
    const door = { pts: [[0.25, 0.99, 0], [0.65, 0.99, 0], [0.65, 0.99, 0.8], [0.25, 0.99, 0.8]], color: 0x6b4226 };
    const win = { pts: [[-0.7, 0.99, 0.55], [-0.2, 0.99, 0.55], [-0.2, 0.99, 1.0], [-0.7, 0.99, 1.0]], color: 0x5ab0ff };
    const house = boxFaces(-1, 1, 1, 3, 0, 1.4, 0xe9dcc0, { front: [door, win] }).filter((_, i) => i !== 4); // no top: roof
    const inside = [0, 2, 1.2];
    const roof = [
      face([[-1, 1, 1.4], [1, 1, 1.4], ridge[1], ridge[0]], 0xc0392b, inside),
      face([[-1, 3, 1.4], [1, 3, 1.4], ridge[1], ridge[0]], 0xc0392b, inside),
      face([[-1, 1, 1.4], [-1, 3, 1.4], ridge[0]], 0xe9dcc0, inside),
      face([[1, 1, 1.4], [1, 3, 1.4], ridge[1]], 0xe9dcc0, inside),
    ];
    faces = [...house, ...roof, ...boxFaces(-2.4, -2.2, -0.6, -0.4, 0, 1.6, 0x3f9e5a)];
  } else {
    segs.push([ridge[0], ridge[1], 0xff6b6b]);
    [[P[4], ridge[0]], [P[7], ridge[0]], [P[5], ridge[1]], [P[6], ridge[1]]].forEach(([a, b]) => segs.push([a, b, 0xff6b6b]));
    // door on the front face (y = 1)
    segs.push([[0.25, 1, 0], [0.25, 1, 0.8], 0xb07a4a], [[0.25, 1, 0.8], [0.65, 1, 0.8], 0xb07a4a], [[0.65, 1, 0.8], [0.65, 1, 0], 0xb07a4a]);
    // a post on the left, closer to the camera
    box(-2.4, -2.2, -0.6, -0.4, 0, 1.6, 0x5fd38d);
  }
  // ground grid
  for (let x = -4; x <= 4; x += 1) segs.push([[x, -3, 0], [x, 8, 0], 0x3a3f4b]);
  for (let y = -3; y <= 8; y += 1) segs.push([[-4, y, 0], [4, y, 0], 0x3a3f4b]);
  return { segs, pts, faces };
}

function railsScene(railYaw = 0) {
  const segs = [];
  const d = [Math.sin(railYaw * deg), Math.cos(railYaw * deg), 0], n = [d[1], -d[0], 0];
  const L = 80;
  for (const s of [-0.6, 0.6]) {
    const a = add(scl(n, s), scl(d, -2)), b = add(scl(n, s), scl(d, L));
    segs.push([a, b, 0xc9ccd4]);
  }
  for (let t = -2; t < L; t += 1.2) segs.push([add(scl(n, -0.9), scl(d, t)), add(scl(n, 0.9), scl(d, t)), 0x8a5a3a]);
  return { segs, pts: [], dir: d };
}

function ambiguityScene() {
  // a small cube close to the camera and a big cube far away that project to the same image square
  const segs = [];
  const cube = (cx, cy, cz, s, c, style) => {
    const P = [];
    for (const dz of [-1, 1]) for (const [dx, dy] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) P.push([cx + dx * s, cy + dy * s, cz + dz * s]);
    [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]].forEach(([a, b]) => segs.push([P[a], P[b], c, style]));
  };
  // camera at (0,-5,1.2) looking +y: similar cubes along the same ray, scaled with distance
  const C0 = [0, -5, 1.2], dirn = norm([0.25, 1, 0.08]);
  const near = add(C0, scl(dirn, 3)), far = add(C0, scl(dirn, 9));
  // far cube first (thick), near cube on top (thin, dashed): in the image they coincide exactly
  cube(...far, 0.6, 0xf2b134, { w: 7 }); cube(...near, 0.2, 0x5ab0ff, { w: 2.5, dash: [8, 6] });
  for (let x = -4; x <= 6; x += 1) segs.push([[x, -3, 0], [x, 10, 0], 0x3a3f4b]);
  for (let y = -3; y <= 10; y += 1) segs.push([[-4, y, 0], [6, y, 0], 0x3a3f4b]);
  return { segs, pts: [] };
}

// ---------------------------------------------------------------------------------------------
export function mount(el, cfg) {
  const controlsWanted = new Set(cfg.controls || ['f', 'o', 'rays', 'physical', 'click']);
  const state = {
    f: cfg.f ?? 500, ox: cfg.ox ?? IW / 2, oy: cfg.oy ?? IH / 2,
    yaw: cfg.yaw ?? 90, pitch: cfg.pitch ?? 0, pos: cfg.pos || [0, -5, 1.2],
    rays: !!cfg.rays, physical: !!cfg.physical, showVP: !!cfg.showVP,
    click: cfg.click || null, railYaw: cfg.railYaw ?? 0,
  };
  const makeScene = () => cfg.scene === 'rails' ? railsScene(state.railYaw) : cfg.scene === 'ambiguity' ? ambiguityScene() : houseScene(cfg.solid !== false);
  let world = makeScene();

  // ---- DOM ----
  const W3 = cfg.width || 560, H3 = cfg.height || 420;
  const c3 = h('canvas', { width: W3, height: H3 });
  const img = h('canvas', { width: IW, height: IH, style: `width:${cfg.imageWidth || 400}px; height:${(cfg.imageWidth || 400) * IH / IW}px; cursor:crosshair` });
  const g = img.getContext('2d');
  const readout = h('div', { style: 'font-size:0.95em; line-height:1.5' });
  const ctl = h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.25em' });

  const view3d = { label: '3D view', position: cfg.camera || T(-6, -9.5, 5.5), target: cfg.target || T(0, -1, 0.8), fov: 40 };
  const viewSide = { label: 'side view', position: T(9, -2, 1.3), target: T(0, -2, 1.1), fov: 40 };
  const viewTop = { label: 'top view', position: add(T(0, -1, 16), [0, 0, 0.01]), target: T(0, -1, 0), fov: 40 };
  const stage = makeStage(c3, { position: view3d.position, target: view3d.target });
  const { scene, render } = stage;

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', {}, c3, h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, viewButtons(stage, [view3d, viewSide, viewTop]))),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.4em' }, img, readout, ctl)));

  // ---- static 3D content ----
  const worldGroup = new THREE.Group(); scene.add(worldGroup);
  const dyn = new THREE.Group(); scene.add(dyn);
  const buildWorld = () => {
    worldGroup.clear();
    for (const [a, b, c] of world.segs) worldGroup.add(line([T(...a), T(...b)], c));
    const mesh = (pts, color, offset = 0) => {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(pts.flatMap(p => T(...p)), 3));
      geo.setIndex(pts.slice(2).flatMap((_, i) => [0, i + 1, i + 2]));
      geo.computeVertexNormals();
      const m = new THREE.MeshStandardMaterial({ color, roughness: 0.8, side: THREE.DoubleSide, flatShading: true,
        polygonOffset: offset !== 0, polygonOffsetFactor: offset, polygonOffsetUnits: offset });
      worldGroup.add(new THREE.Mesh(geo, m));
    };
    for (const f of world.faces || []) {
      mesh(f.pts, f.color);
      f.decals.forEach(d => mesh(d.pts, d.color, -2));
      worldGroup.add(line([...f.pts, f.pts[0]].map(p => T(...p)), 0x1b1d22));
    }
  };
  buildWorld();
  const tex = new THREE.CanvasTexture(img); tex.colorSpace = THREE.SRGBColorSpace;
  const planeMat = new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide, transparent: true, opacity: 0.92 });

  // ---- camera model ----
  function cameraR() {
    const y = state.yaw * deg, p = state.pitch * deg;
    const fwd = [Math.cos(p) * Math.cos(y), Math.cos(p) * Math.sin(y), Math.sin(p)];
    const right = norm(cross(fwd, [0, 0, 1]));
    const down = cross(fwd, right);
    return [right, down, fwd]; // rows
  }
  const toCam = (R, p) => { const d = sub(p, state.pos); return [dot(R[0], d), dot(R[1], d), dot(R[2], d)]; };
  const toWorldDir = (R, v) => add(add(scl(R[0], v[0]), scl(R[1], v[1])), scl(R[2], v[2]));
  const project = pc => [state.f * pc[0] / pc[2] + state.ox, state.f * pc[1] / pc[2] + state.oy];
  const backproject = (u, v) => [(u - state.ox) / state.f, (v - state.oy) / state.f, 1]; // K⁻¹ [u v 1]ᵀ

  // Filled faces: back-face culling, near-plane clipping, far-to-near order, simple Lambert shading.
  const LIGHT = norm([-0.4, -0.6, 0.7]);
  function shade(color, n) {
    const k = 0.55 + 0.45 * Math.max(0, dot(n, LIGHT));
    const r = (color >> 16) & 255, gg = (color >> 8) & 255, b = color & 255;
    return `rgb(${Math.round(r * k)},${Math.round(gg * k)},${Math.round(b * k)})`;
  }
  function clipNear(P, near) {
    const out = [];
    for (let i = 0; i < P.length; i++) {
      const a = P[i], b = P[(i + 1) % P.length];
      if (a[2] >= near) out.push(a);
      if ((a[2] >= near) !== (b[2] >= near)) out.push(add(a, scl(sub(b, a), (near - a[2]) / (b[2] - a[2]))));
    }
    return out;
  }
  function fillPoly(Pc, fill, stroke) {
    const Q = Pc.map(project);
    g.beginPath(); Q.forEach(([u, v], i) => (i ? g.lineTo(u, v) : g.moveTo(u, v))); g.closePath();
    g.fillStyle = fill; g.fill();
    if (stroke) { g.strokeStyle = stroke; g.lineWidth = 1.2; g.stroke(); }
  }
  function drawFaces(R, near) {
    const vis = [];
    for (const f of world.faces) {
      if (dot(f.n, sub(f.pts[0], state.pos)) >= 0) continue; // facing away from the camera
      const Pc = clipNear(f.pts.map(p => toCam(R, p)), near);
      if (Pc.length < 3) continue;
      vis.push({ f, Pc, depth: Pc.reduce((s, p) => s + p[2], 0) / Pc.length });
    }
    vis.sort((a, b) => b.depth - a.depth);
    for (const { f, Pc } of vis) {
      fillPoly(Pc, shade(f.color, f.n), '#1b1d22');
      for (const d of f.decals) {
        const Dc = clipNear(d.pts.map(p => toCam(R, p)), near);
        if (Dc.length >= 3) fillPoly(Dc, shade(d.color, f.n), null);
      }
    }
  }

  function drawImage(R) {
    g.fillStyle = '#0d0f13'; g.fillRect(0, 0, IW, IH);
    const near = 0.05;
    for (const [a, b, c, style] of world.segs) {
      let pa = toCam(R, a), pb = toCam(R, b);
      if (pa[2] < near && pb[2] < near) continue;
      if (pa[2] < near || pb[2] < near) { // clip to the near plane
        const t = (near - pa[2]) / (pb[2] - pa[2]);
        const pm = add(pa, scl(sub(pb, pa), t));
        if (pa[2] < near) pa = pm; else pb = pm;
      }
      const [u0, v0] = project(pa), [u1, v1] = project(pb);
      g.strokeStyle = '#' + c.toString(16).padStart(6, '0'); g.lineWidth = style?.w ?? (c === 0x3a3f4b ? 1 : 2.5);
      g.setLineDash(style?.dash || []);
      g.beginPath(); g.moveTo(u0, v0); g.lineTo(u1, v1); g.stroke();
      g.setLineDash([]);
    }
    if (world.faces?.length) drawFaces(R, near);
    // principal point and pixel axes
    g.strokeStyle = COL.purple; g.lineWidth = 2;
    g.beginPath(); g.moveTo(state.ox - 12, state.oy); g.lineTo(state.ox + 12, state.oy); g.moveTo(state.ox, state.oy - 12); g.lineTo(state.ox, state.oy + 12); g.stroke();
    g.fillStyle = COL.dim; g.font = '22px Inter, Arial';
    g.fillText('u →', 10, 26); g.fillText('v ↓', 10, 54);
    g.fillStyle = COL.purple; g.fillText('principal point', state.ox + 10, state.oy - 10);

    if (state.showVP && world.dir) {
      // vanishing point of the rails: projection of the direction (a point at infinity)
      const dc = [dot(R[0], world.dir), dot(R[1], world.dir), dot(R[2], world.dir)];
      if (dc[2] > 1e-6) {
        const [u, v] = project(dc);
        g.fillStyle = COL.accent; g.beginPath(); g.arc(u, v, 8, 0, 7); g.fill();
        g.fillText('vanishing point', u + 12, v - 12);
      }
      // horizon: image of the ground plane's line at infinity, l = K⁻ᵀ R n
      const n = [R[0][2], R[1][2], R[2][2]];
      const a = n[0] / state.f, b = n[1] / state.f, c = n[2] - state.ox * n[0] / state.f - state.oy * n[1] / state.f;
      if (Math.abs(b) > 1e-9) {
        g.strokeStyle = COL.accent; g.setLineDash([10, 8]); g.lineWidth = 1.5;
        g.beginPath(); g.moveTo(0, -c / b); g.lineTo(IW, -(a * IW + c) / b); g.stroke(); g.setLineDash([]);
        g.fillStyle = COL.accent; g.fillText('horizon', IW - 100, -(a * (IW - 100) + c) / b - 10);
      }
    }
    if (state.click) {
      const [u, v] = state.click;
      g.strokeStyle = COL.accent; g.lineWidth = 3;
      g.beginPath(); g.arc(u, v, 9, 0, 7); g.stroke();
    }
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, IW - 2, IH - 2);
  }

  function draw() {
    const R = cameraR();
    drawImage(R);
    tex.needsUpdate = true;
    dyn.clear();
    const Cw = state.pos;
    // camera body + its axes
    const bodyL = 0.35;
    dyn.add(arrow3(T(...Cw), T(...add(Cw, scl(R[0], bodyL))), 0xff6b6b, 0.08));
    dyn.add(arrow3(T(...Cw), T(...add(Cw, scl(R[1], bodyL))), 0x5fd38d, 0.08));
    dyn.add(arrow3(T(...Cw), T(...add(Cw, scl(R[2], bodyL * 1.6))), 0x5ab0ff, 0.08));
    const pin = new THREE.Mesh(new THREE.SphereGeometry(0.05, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff }));
    pin.position.set(...T(...Cw)); dyn.add(pin);

    // image planes: virtual (in front, depth dv) and optionally physical (behind, inverted)
    const dv = 0.9 * state.f / 500;
    const cornerAt = (u, v, depth) => add(Cw, toWorldDir(R, scl(backproject(u, v), depth)));
    const quad = depth => {
      const P = [[0, 0], [IW, 0], [IW, IH], [0, IH]].map(([u, v]) => T(...cornerAt(u, v, depth)));
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(P.flat(), 3));
      geo.setAttribute('uv', new THREE.Float32BufferAttribute([0, 1, 1, 1, 1, 0, 0, 0], 2));
      geo.setIndex([0, 1, 2, 0, 2, 3]);
      dyn.add(new THREE.Mesh(geo, planeMat));
      P.forEach(p => dyn.add(line([T(...Cw), p], 0x9aa1ae)));
      dyn.add(line([...P, P[0]], 0x9aa1ae));
    };
    quad(dv);
    if (state.physical) quad(-dv);
    const lf = label(`f = ${Math.round(state.f)} px`, { color: COL.dim, size: 0.22, font: '44px Inter, Arial' });
    lf.position.set(...T(...add(Cw, toWorldDir(R, [0, -0.75 * dv, dv])))); dyn.add(lf);

    // projection rays from house corners
    if (state.rays && world.pts.length) for (const p of world.pts) dyn.add(line([T(...p), T(...Cw)], 0xf2b134));

    // vanishing ray: through the pinhole, parallel to the rails
    if (state.showVP && world.dir) dyn.add(line([T(...Cw), T(...add(Cw, scl(world.dir, 60)))], 0xf2b134, { dashed: true }));

    // back-projected ray of the clicked pixel
    let rayText = '';
    if (state.click) {
      const [u, v] = state.click;
      const dcam = backproject(u, v), dw = norm(toWorldDir(R, dcam));
      dyn.add(line([T(...Cw), T(...add(Cw, scl(dw, 14)))], 0xf2b134));
      dyn.add(line([T(...Cw), T(...add(Cw, scl(dw, -2 * dv)))], 0xf2b134, { dashed: true }));
      rayText = `<div>pixel $(${Math.round(u)}, ${Math.round(v)})$ → ray $\\;z\\,(${fmt(dcam[0], 3)},\\ ${fmt(dcam[1], 3)},\\ 1),\\ z>0$</div>`;
    }
    readout.innerHTML =
      `<div>$K = \\begin{bmatrix}${fmt(state.f, 0)}&0&${fmt(state.ox, 0)}\\\\0&${fmt(state.f, 0)}&${fmt(state.oy, 0)}\\\\0&0&1\\end{bmatrix}$</div>` + rayText;
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
    render();
  }

  // ---- controls ----
  const slider = (key, min, max, step, name, fmtv = v => Math.round(v)) => {
    const s = h('input', { type: 'range', min, max, step, value: state[key] });
    const val = h('span', { class: 'readout', style: 'display:inline-block; width:3.5em; text-align:right' }, String(fmtv(state[key])));
    s.addEventListener('input', () => {
      state[key] = parseFloat(s.value); val.textContent = String(fmtv(state[key]));
      if (key === 'railYaw') { world = makeScene(); buildWorld(); }
      draw();
    });
    return h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', { style: 'width:5.5em' }, name), s, val);
  };
  const toggle = (key, name) => {
    const b = h('button', { onclick: () => { state[key] = !state[key]; b.classList.toggle('active', state[key]); draw(); } }, name);
    b.classList.toggle('active', state[key]);
    return b;
  };
  if (controlsWanted.has('f')) ctl.append(slider('f', 150, 1200, 1, 'focal f'));
  if (controlsWanted.has('o')) ctl.append(slider('ox', 100, 540, 1, h('span', {}, 'o', h('sub', {}, 'x'))), slider('oy', 80, 400, 1, h('span', {}, 'o', h('sub', {}, 'y'))));
  if (controlsWanted.has('pose')) ctl.append(slider('yaw', 30, 150, 1, 'cam yaw'), slider('pitch', -40, 40, 1, 'cam pitch'));
  if (controlsWanted.has('railYaw')) ctl.append(slider('railYaw', -60, 60, 1, 'rails dir'));
  const toggles = [];
  if (controlsWanted.has('rays')) toggles.push(toggle('rays', 'projection rays'));
  if (controlsWanted.has('physical')) toggles.push(toggle('physical', 'physical image plane'));
  if (controlsWanted.has('vp')) toggles.push(toggle('showVP', 'vanishing point'));
  if (toggles.length) ctl.append(h('div', { style: 'display:flex; gap:0.3em; flex-wrap:wrap' }, toggles));
  if (controlsWanted.has('click')) {
    ctl.append(h('div', { class: 'dim' }, 'click the image to back-project a pixel'));
    img.addEventListener('click', e => {
      const r = img.getBoundingClientRect();
      state.click = [(e.clientX - r.left) / r.width * IW, (e.clientY - r.top) / r.height * IH];
      draw();
    });
  }
  draw();
}
