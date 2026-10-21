// Warping onto a common surface: where the pixels of a panorama go.
// One camera centre (the origin); 3 or 5 views from the panorama scene (panoenv.js: a real 360° photo, the camera only
// rotates, all views tilted up by 12°), drawn as small textured frames near the centre with the rays through their
// corners. Each view is projected along its rays onto the panorama surface:
//   plane     the image plane of the reference (middle) view, extended: a pixel with ray d lands at d / (n·d)
//             (n = the reference optical axis, the plane at distance f). As a ray approaches 90° from n, n·d → 0
//             and the point runs off to infinity: the outer views stretch, the planar panorama's width grows like
//             2 f tan(yaw + half the field of view) and is infinite from 90° on (those parts are not drawn).
//   cylinder  a vertical cylinder of radius f around the centre: d lands at d / √(d_x² + d_z²), i.e. at
//             (angle θ = atan2(d_x, d_z), height h = d_y / √(d_x² + d_z²)). "unroll" bends the cylinder into the
//             flat strip u = f θ, v = f h (the cylindrical panorama): its width 2 f (yaw + half FOV) stays finite.
// Inset: the top view (centre, the views, the surface, the edge rays) and the panorama width at the horizon.
// World frame as in panoenv.js (x right, y down, z forward, level); three.js: T(x, y, z) = (x, −y, −z). Unit = f.
//
// config: {
//   width: 640, height: 380   3D view size
//   surface: "plane"          "plane" | "cylinder"
//   surfaces: ["plane", "cylinder"]   surface buttons shown (one entry: no buttons)
//   views: 3                  3 (yaw −y, 0, +y) or 5 (−y, −y/2, 0, y/2, y)
//   yaw: 30                   yaw of the outer views, degrees (slider 0 … 85)
//   unroll: 0                 cylinder only: 0 = cylinder, 1 = unrolled strip
//   frames: true              show the small camera images and the corner rays
// }
// mount() is async (the photo loads first) and resolves when the figure is drawn. Print: the configured state.
import { C, h, fmt, mul3 } from './util.js';
import { THREE, makeStage, label, line, viewButtons } from './three-util.js';
import { IW, IH, F, OX, OY, Ki, makeViews, loadEnv, sampleDir, deg } from './panoenv.js';

const T = (x, y, z) => [x, -y, -z];
const VIEW_HEX = [0xff6b6b, 0x5fd38d, 0x5ab0ff, 0xb28dff, 0xf2b134];
const VIEW_CSS = [C.red, C.green, C.blue, C.purple, C.accent];
const TW = 240, TH = 180;           // texture size of a view
const S = 0.45;                     // distance of the drawn camera images from the centre (the surface is at 1 = f)
const NU = 40, NV = 24;             // grid of a projected image
const LIM = 9;                      // plane: points farther than this (in f) are not drawn
const HALF = Math.atan(OX / F);     // half the horizontal field of view

export async function mount(el, cfg) {
  const gen = el.dataset.gen;
  const W = cfg.width || 640, H = cfg.height || 380;
  el.classList.add('widget');
  const loading = h('div', { class: 'readout', style: `width:${W}px; height:${H}px; display:flex; align-items:center; justify-content:center; background:${C.bg2}; border:1px solid ${C.line}; border-radius:8px` }, 'loading the panorama…');
  el.append(loading);
  const env = await loadEnv();
  if (el.dataset.gen !== gen) return;
  loading.remove();

  const nViews = cfg.views === 5 ? 5 : 3, ref = (nViews - 1) / 2;
  const surfaces = cfg.surfaces || ['plane', 'cylinder'];
  const state = { surface: cfg.surface || surfaces[0], yaw: cfg.yaw ?? 30, t: cfg.unroll ?? 0, frames: cfg.frames ?? true };

  const canvas = h('canvas', { width: W, height: H });
  const view3d = { label: '3D view', position: T(-1.15, -1.35, -1.45), target: T(0, -0.3, 1.0), fov: 45 };
  const viewTop = { label: 'top view', position: [0, 7.5, -1.2 - 0.01], target: [0, 0, -1.2], fov: 45 };
  const stage = makeStage(canvas, { position: cfg.camera || view3d.position, target: cfg.target || view3d.target, fov: 45 });
  const { scene, render } = stage;
  const dyn = new THREE.Group(); scene.add(dyn);

  // ---------------- view textures (cached per yaw) ----------------
  const texCache = new Map();
  function texture(v) {
    const key = `${v.yaw.toFixed(2)}|${v.R.map(x => x.toFixed(4)).join(',')}`;
    if (texCache.has(key)) return texCache.get(key);
    const c = h('canvas', { width: TW, height: TH }), g = c.getContext('2d'), im = g.createImageData(TW, TH);
    const M = mul3(v.Rcw, Ki), d = [0, 0, 0], col = [0, 0, 0], sx = IW / TW;
    for (let y = 0; y < TH; y++) for (let x = 0; x < TW; x++) {
      const u = (x + 0.5) * sx, w = (y + 0.5) * sx;
      d[0] = M[0] * u + M[1] * w + M[2]; d[1] = M[3] * u + M[4] * w + M[5]; d[2] = M[6] * u + M[7] * w + M[8];
      sampleDir(env, d, col);
      const i = 4 * (y * TW + x);
      im.data[i] = col[0] * 1.08; im.data[i + 1] = col[1] * 1.08; im.data[i + 2] = col[2] * 1.08; im.data[i + 3] = 255;
    }
    g.putImageData(im, 0, 0);
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    texCache.set(key, tex);
    return tex;
  }

  const yawsFor = y => nViews === 5 ? [-y, -y / 2, 0, y / 2, y] : [-y, 0, y];
  // world ray of pixel (u, v) of view V
  const ray = (V, u, v) => { const M = V.M; return [M[0] * u + M[1] * v + M[2], M[3] * u + M[4] * v + M[5], M[6] * u + M[7] * v + M[8]]; };

  // surface point of a ray (null: not on the surface)
  function onSurface(d, n) {
    if (state.surface === 'plane') {
      const s = d[0] * n[0] + d[1] * n[1] + d[2] * n[2];
      if (s < 0.1) return null;
      const p = [d[0] / s, d[1] / s, d[2] / s];
      return Math.hypot(...p) > LIM ? null : p;
    }
    const r = Math.hypot(d[0], d[2]), th = Math.atan2(d[0], d[2]), hh = d[1] / r;
    return bend(th, hh);
  }
  // cylinder (θ, h) → 3D, bent towards the flat strip by state.t (radius 1 / (1 − t), tangent at θ = 0)
  function bend(th, hh) {
    const t = Math.min(state.t, 0.999), R = 1 / (1 - t);
    if (t < 1e-4) return [Math.sin(th), hh, Math.cos(th)];
    return [R * Math.sin(th / R), hh, 1 - R + R * Math.cos(th / R)];
  }

  function imageMesh(V, tex, n) {
    const pos = [], uv = [], ok = [];
    for (let j = 0; j <= NV; j++) for (let i = 0; i <= NU; i++) {
      const u = IW * i / NU, v = IH * j / NV, p = onSurface(ray(V, u, v), n);
      ok.push(!!p); pos.push(...(p ? T(...p) : [0, 0, 0])); uv.push(i / NU, 1 - j / NV);
    }
    const idx = [], at = (i, j) => j * (NU + 1) + i;
    for (let j = 0; j < NV; j++) for (let i = 0; i < NU; i++) {
      const a = at(i, j), b = at(i + 1, j), c = at(i + 1, j + 1), d = at(i, j + 1);
      if (ok[a] && ok[b] && ok[c]) idx.push(a, b, c);
      if (ok[a] && ok[c] && ok[d]) idx.push(a, c, d);
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geo.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
    geo.setIndex(idx);
    return new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide }));
  }
  // border of a view on the surface, split where it leaves the surface
  function borderLines(V, n, color) {
    const runs = [[]], N = 48;
    const pts = [];
    for (let k = 0; k < N; k++) pts.push([IW * k / N, 0]);
    for (let k = 0; k < N; k++) pts.push([IW, IH * k / N]);
    for (let k = 0; k < N; k++) pts.push([IW - IW * k / N, IH]);
    for (let k = 0; k <= N; k++) pts.push([0, IH - IH * k / N]);
    pts.forEach(([u, v]) => {
      const p = onSurface(ray(V, u, v), n);
      if (p) runs[runs.length - 1].push(T(...p)); else if (runs[runs.length - 1].length) runs.push([]);
    });
    runs.filter(r => r.length > 1).forEach(r => dyn.add(line(r, color)));
  }

  // ---------------- inset: top view ----------------
  const IN = 230;
  const inset = h('canvas', { width: 2 * IN, height: 2 * IN, style: `width:${IN}px; height:${IN}px; border:1px solid ${C.line}; border-radius:6px; background:${C.bg2}` });
  const readout = h('div', { class: 'readout', style: `line-height:1.4; width:${IN}px; font-size:0.85em` });
  function drawInset(views) {
    const g = inset.getContext('2d'), s = 2 * IN / 4.4, cx = IN, cy = 2 * IN - 120;
    const P = (x, z) => [cx + x * s, cy - z * s];
    g.setTransform(1, 0, 0, 1, 0, 0); g.clearRect(0, 0, 2 * IN, 2 * IN);
    g.lineWidth = 3; g.strokeStyle = C.dim;
    if (state.surface === 'plane') { g.beginPath(); g.moveTo(...P(-2.3, 1)); g.lineTo(...P(2.3, 1)); g.stroke(); }
    else { g.beginPath(); g.arc(cx, cy, s, 0, 2 * Math.PI); g.stroke(); }
    views.forEach((V, k) => {
      const a = V.yaw * deg;
      g.strokeStyle = VIEW_CSS[k]; g.lineWidth = 2;
      for (const e of [a - HALF, a + HALF]) {
        // plane: the ray meets z = f only if it points forward (cos e > 0); drawn up to the inset's border
        const hits = state.surface !== 'plane' || Math.cos(e) > 1e-3;
        const L = state.surface === 'plane' ? (hits ? Math.min(1 / Math.cos(e), 2.15 / Math.abs(Math.sin(e) || 1e-9)) : 0.9) : 1;
        const x = L * Math.sin(e), z = L * Math.cos(e);
        g.setLineDash(hits && !(state.surface === 'plane' && Math.abs(Math.tan(e)) > 2.15) ? [] : [6, 6]);
        g.beginPath(); g.moveTo(cx, cy); g.lineTo(...P(x, z)); g.stroke();
      }
      g.setLineDash([]);
      // the view's image segment at distance S
      const c = [Math.sin(a), Math.cos(a)], t = [Math.cos(a), -Math.sin(a)], hw = S * Math.tan(HALF);
      g.lineWidth = 6; g.beginPath(); g.moveTo(...P(S * c[0] - hw * t[0], S * c[1] - hw * t[1])); g.lineTo(...P(S * c[0] + hw * t[0], S * c[1] + hw * t[1])); g.stroke();
    });
    g.fillStyle = C.fg; g.beginPath(); g.arc(cx, cy, 7, 0, 7); g.fill();
    g.font = '32px Inter, Arial, sans-serif'; g.fillStyle = C.dim;
    g.fillText('top view', 14, 40);
    g.fillText(state.surface === 'plane' ? 'plane (distance f)' : 'cylinder (radius f)', 14, 80);
    const edge = state.yaw * deg + HALF;
    const wPlane = Math.cos(edge) > 1e-3 && edge < Math.PI / 2 ? 2 * F * Math.tan(edge) : Infinity;
    const wCyl = 2 * F * edge;
    readout.innerHTML =
      `<div>edges at ±${fmt(edge / deg, 1)}°; panorama width:</div>` +
      `<div style="color:${state.surface === 'plane' ? C.accent : C.dim}">plane 2f·tan: ${isFinite(wPlane) ? Math.round(wPlane) + ' px' : '∞'}</div>` +
      `<div style="color:${state.surface === 'cylinder' ? C.accent : C.dim}">cylinder 2f·angle: ${Math.round(wCyl)} px</div>` +
      `<div class="dim">f = ${F} px; one image: ${IW} px</div>`;
  }

  // ---------------- scene ----------------
  function clearDyn() {
    dyn.traverse(o => {
      o.geometry?.dispose();
      if (o.material) { if (o.material.map && o.isSprite) o.material.map.dispose(); o.material.dispose(); }
    });
    dyn.clear();
  }
  function update() {
    clearDyn();
    const views = makeViews(nViews, yawsFor(state.yaw));
    views.forEach(V => { V.M = mul3(V.Rcw, Ki); });
    const n = [views[ref].Rcw[2], views[ref].Rcw[5], views[ref].Rcw[8]]; // reference optical axis
    // the surface itself
    if (state.surface === 'plane') {
      // the reference image plane, a large translucent rectangle (u, v along the reference axes)
      const a = [views[ref].Rcw[0], views[ref].Rcw[3], views[ref].Rcw[6]], b = [views[ref].Rcw[1], views[ref].Rcw[4], views[ref].Rcw[7]];
      const cs = [[-3.6, -1.6], [3.6, -1.6], [3.6, 1.2], [-3.6, 1.2]].map(([x, y]) => T(n[0] + x * a[0] + y * b[0], n[1] + x * a[1] + y * b[1], n[2] + x * a[2] + y * b[2]));
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(cs.flat(), 3)); geo.setIndex([0, 1, 2, 0, 2, 3]);
      dyn.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x9aa1ae, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false })));
      dyn.add(line([...cs, cs[0]], 0x4a505c));
    } else {
      // the cylinder band (bent by the unroll parameter), with a few vertical lines
      const th0 = -Math.PI * 0.95, th1 = Math.PI * 0.95, h0 = -1.05, h1 = 0.25, NT = 72;
      const pos = [], idx = [];
      for (let i = 0; i <= NT; i++) for (const hh of [h0, h1]) pos.push(...T(...bend(th0 + (th1 - th0) * i / NT, hh)));
      for (let i = 0; i < NT; i++) idx.push(2 * i, 2 * i + 1, 2 * i + 3, 2 * i, 2 * i + 3, 2 * i + 2);
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3)); geo.setIndex(idx);
      dyn.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x9aa1ae, transparent: true, opacity: 0.08, side: THREE.DoubleSide, depthWrite: false })));
      for (const hh of [h0, h1]) dyn.add(line(Array.from({ length: NT + 1 }, (_, i) => T(...bend(th0 + (th1 - th0) * i / NT, hh))), 0x4a505c));
      for (let a = -150; a <= 150; a += 30) dyn.add(line([T(...bend(a * deg, h0)), T(...bend(a * deg, h1))], 0x3a3f4b));
    }
    views.forEach((V, k) => {
      const tex = texture(V);
      dyn.add(imageMesh(V, tex, n));
      borderLines(V, n, VIEW_HEX[k]);
      if (state.frames) {
        // the camera image at distance S, and the rays through its corners to the surface
        const cs = [[0, 0], [IW, 0], [IW, IH], [0, IH]].map(([u, v]) => ray(V, u, v));
        const q = cs.map(d => T(S * d[0], S * d[1], S * d[2]));
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(q.flat(), 3));
        geo.setAttribute('uv', new THREE.Float32BufferAttribute([0, 1, 1, 1, 1, 0, 0, 0], 2));
        geo.setIndex([0, 1, 2, 0, 2, 3]);
        dyn.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide })));
        dyn.add(line([...q, q[0]], VIEW_HEX[k]));
        cs.forEach(d => {
          const p = onSurface(d, n);
          const end = p || [d[0] * 4, d[1] * 4, d[2] * 4];
          const l = line([[0, 0, 0], T(...end)], VIEW_HEX[k], { dashed: !p });
          l.material.transparent = true; l.material.opacity = 0.55; dyn.add(l);
        });
      }
    });
    const c = new THREE.Mesh(new THREE.SphereGeometry(0.035, 16, 12), new THREE.MeshBasicMaterial({ color: 0xe7e9ee }));
    dyn.add(c);
    const lab = label('centre', { color: C.fg, size: 0.13, font: '48px Inter, Arial, sans-serif' }); lab.position.set(...T(0, 0.12, 0)); dyn.add(lab);
    render();
    drawInset(views);
    surfBtns.forEach(([k, b]) => b.classList.toggle('active', k === state.surface));
    unrollBtn.style.display = state.surface === 'cylinder' ? '' : 'none';
    unrollBtn.textContent = state.t > 0.5 ? 'roll up' : 'unroll';
  }

  // ---------------- controls ----------------
  const surfBtns = surfaces.map(k => [k, h('button', { onclick: () => { state.surface = k; if (k === 'plane') state.t = 0; update(); } }, k)]);
  let anim = 0;
  const unrollBtn = h('button', {
    onclick: () => {
      const from = state.t, to = from > 0.5 ? 0 : 1, start = performance.now(), id = ++anim;
      const step = now => {
        if (id !== anim) return;
        const u = Math.min(1, (now - start) / 1400), e = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
        state.t = from + (to - from) * e; update();
        if (u < 1) requestAnimationFrame(step);
      };
      requestAnimationFrame(step);
    },
  }, 'unroll');
  const yawIn = h('input', { type: 'range', min: 0, max: 85, step: 1, value: state.yaw, style: 'width:7em' });
  const yawVal = h('span', { class: 'readout', style: 'display:inline-block; width:3.2em; text-align:right' }, `${state.yaw}°`);
  yawIn.addEventListener('input', () => { state.yaw = +yawIn.value; yawVal.textContent = `${state.yaw}°`; update(); });
  const framesBtn = h('button', { class: state.frames ? 'active' : '', onclick: () => { state.frames = !state.frames; framesBtn.classList.toggle('active', state.frames); update(); } }, 'camera images + rays');

  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${W}px; flex:none` }, canvas,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' },
        ...(surfaces.length > 1 ? surfBtns.map(b => b[1]) : []), unrollBtn,
        h('label', { style: 'margin-left:0.5em' }, 'outer views at ±'), yawIn, yawVal)),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; flex:none' }, inset, readout,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em; margin-top:0' }, viewButtons(stage, [view3d, viewTop]), framesBtn))));
  update();
}
