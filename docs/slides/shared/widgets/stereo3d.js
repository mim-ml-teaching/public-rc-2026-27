// Rectified stereo in 3D: why a point appears in the same image row in both cameras.
// Two identical cameras (left at the origin, right at (b, 0, 0); camera frame x right, y down, z forward), both
// looking along +z, with their (virtual) image planes in front of them, drawn with a few image rows. A point P,
// its rays to both centres and the plane through P and both centres. That plane contains the baseline (parallel
// to x), so it cuts both image planes along the same horizontal line: the row v of P, the same in both images.
// Below: the two images side by side with the shared row.
// config: { P: [x, y, z], b, f, width, height, plane: true }
import { C as COL, h, fmt } from './util.js';
import { THREE, makeStage, label, line, arrow3, viewButtons } from './three-util.js';

// camera frame -> three.js: X = z (forward), Y = -y (up), Z = x   (a proper rotation, as in pinhole3d)
const T = (x, y, z) => [z, -y, x];
const IW = 640, IH = 480;
const S = 0.5;                        // distance of the drawn image planes from the centres

export function mount(el, cfg) {
  const f = cfg.f ?? 500, ox = IW / 2, oy = IH / 2;
  const state = { P: (cfg.P || [0.45, -0.35, 2.2]).slice(), b: cfg.b ?? 0.8, plane: cfg.plane ?? true };
  const W = cfg.width || 560, H = cfg.height || 330;

  const canvas = h('canvas', { width: W, height: H });
  const view3d = { label: '3D view', position: cfg.camera || T(-1.25, -1.05, -1.05), target: cfg.target || T(0.45, -0.12, 0.95), fov: 42 };
  const viewFront = { label: 'from behind the cameras', position: T(0.4, -0.05, -2.2), target: T(0.4, -0.05, 1.0), fov: 40 };
  const stage = makeStage(canvas, { position: view3d.position, target: view3d.target });
  const { scene, render } = stage;
  const dyn = new THREE.Group(); scene.add(dyn);
  const lab = (text, p, color, size = 0.12, sub = '') => { const l = label(text, { color, size, sub }); l.position.set(...T(...p)); dyn.add(l); };
  const ground = new THREE.GridHelper(8, 16, 0x3a3f4b, 0x2a2e36); ground.position.set(...T(0.3, 0.9, 2.5)); scene.add(ground);

  const rowsDrawn = [60, 120, 180, 240, 300, 360, 420];
  function camera(cx, color, name) {
    const hw = S * ox / f, hh = S * oy / f;
    const cs = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]].map(([x, y]) => [cx + x, y, S]);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(cs.flatMap(p => T(...p)), 3));
    geo.setIndex([0, 1, 2, 0, 2, 3]);
    dyn.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x9aa1ae, transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false })));
    dyn.add(line([...cs, cs[0]].map(p => T(...p)), color));
    cs.forEach(c => dyn.add(line([T(cx, 0, 0), T(...c)], 0x4a505c)));
    for (const v of rowsDrawn) { const y = S * (v - oy) / f; dyn.add(line([T(cx - hw, y, S), T(cx + hw, y, S)], 0x3a3f4b)); }
    const m = new THREE.Mesh(new THREE.SphereGeometry(0.03, 14, 10), new THREE.MeshBasicMaterial({ color }));
    m.position.set(...T(cx, 0, 0)); dyn.add(m);
    lab(name, [cx - 0.05, 0.14, -0.02], '#' + color.toString(16).padStart(6, '0'), 0.14);
  }

  // images below the 3D view
  const SC = 0.27, mk = () => h('canvas', { width: IW * SC * 2, height: IH * SC * 2, style: `width:${IW * SC}px; height:${IH * SC}px; border:1px solid #3a3f4b; border-radius:4px; display:block` });
  const imL = mk(), imR = mk();
  const readout = h('div', { style: 'font-size:0.92em; line-height:1.45' });

  function project(cx) { const [x, y, z] = state.P; return [f * (x - cx) / z + ox, f * y / z + oy]; }

  function drawImage(cv, uv, color, name) {
    const g = cv.getContext('2d'), k = cv.width / IW;
    g.fillStyle = '#0d0f13'; g.fillRect(0, 0, cv.width, cv.height);
    g.strokeStyle = '#2a2e36'; g.lineWidth = 1.5;
    for (const v of rowsDrawn) { g.beginPath(); g.moveTo(0, v * k); g.lineTo(cv.width, v * k); g.stroke(); }
    const [u, v] = uv;
    g.strokeStyle = COL.accent; g.lineWidth = 3; g.setLineDash([10, 6]);
    g.beginPath(); g.moveTo(0, v * k); g.lineTo(cv.width, v * k); g.stroke(); g.setLineDash([]);
    if (u >= 0 && u <= IW && v >= 0 && v <= IH) { g.fillStyle = COL.red; g.beginPath(); g.arc(u * k, v * k, 9, 0, 7); g.fill(); }
    g.font = `${Math.round(30)}px Inter, Arial`; g.fillStyle = color; g.fillText(name, 12, 36);
    g.fillStyle = COL.fg; g.fillText(`u = ${Math.round(u)}`, Math.min(Math.max(u * k - 50, 8), cv.width - 150), v * k > 90 ? v * k - 18 : v * k + 40);
  }

  function update() {
    dyn.clear();
    const [x, y, z] = state.P, b = state.b;
    camera(0, 0x5ab0ff, 'left');
    camera(b, 0x5fd38d, 'right');
    // baseline
    dyn.add(line([T(0, 0, 0), T(b, 0, 0)], 0xe7e9ee));
    lab('b', [b / 2 - 0.02, 0.11, 0], COL.fg, 0.15);
    // P and the rays
    const Pm = new THREE.Mesh(new THREE.SphereGeometry(0.045, 16, 12), new THREE.MeshStandardMaterial({ color: 0xff6b6b }));
    Pm.position.set(...T(x, y, z)); dyn.add(Pm);
    lab('P', [x + 0.08, y - 0.08, z], COL.red, 0.2);
    dyn.add(line([T(0, 0, 0), T(x, y, z)], 0x5ab0ff));
    dyn.add(line([T(b, 0, 0), T(x, y, z)], 0x5fd38d));
    // image points on the planes and the shared row
    const pl = [S * x / z, S * y / z, S], pr = [b + S * (x - b) / z, S * y / z, S];
    for (const p of [pl, pr]) {
      const m = new THREE.Mesh(new THREE.SphereGeometry(0.018, 12, 10), new THREE.MeshBasicMaterial({ color: 0xff6b6b }));
      m.position.set(...T(...p)); dyn.add(m);
    }
    const hw = S * ox / f, yr = S * y / z;
    dyn.add(line([T(-hw, yr, S), T(hw, yr, S)], 0xf2b134, { width: 2 }));
    dyn.add(line([T(b - hw, yr, S), T(b + hw, yr, S)], 0xf2b134, { width: 2 }));
    dyn.add(line([T(hw, yr, S), T(b - hw, yr, S)], 0xf2b134, { dashed: true }));
    lab('same row', [b + hw + 0.04, yr + 0.02, S], COL.accent, 0.13);
    // the plane through P and both centres
    if (state.plane) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute([T(0, 0, 0), T(b, 0, 0), T(x, y, z)].flat(), 3));
      geo.setIndex([0, 1, 2]);
      dyn.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0xb28dff, transparent: true, opacity: 0.22, side: THREE.DoubleSide, depthWrite: false })));
    }
    render();

    const [ul, vl] = project(0), [ur, vr] = project(b);
    drawImage(imL, [ul, vl], COL.blue, 'left image');
    drawImage(imR, [ur, vr], COL.green, 'right image');
    readout.innerHTML =
      `<div>$v_l = v_r = ${fmt(vl, 1)}$</div>` +
      `<div>$u_l = ${fmt(ul, 1)},\\ u_r = ${fmt(ur, 1)}$</div>` +
      `<div>$d = u_l - u_r = ${fmt(ul - ur, 1)}$ px</div>`;
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
  }

  const slider = (name, min, max, step, get, set, unit = '') => {
    const s = h('input', { type: 'range', min, max, step, value: get(), style: 'width:6.5em' });
    const val = h('span', { class: 'readout', style: 'display:inline-block; width:4.2em; text-align:right' });
    const upd = () => { val.textContent = fmt(get(), 2) + unit; };
    s.addEventListener('input', () => { set(parseFloat(s.value)); upd(); update(); });
    upd();
    return h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', { style: 'width:2.6em; flex:none' }, name), s, val);
  };
  const planeBtn = h('button', { class: state.plane ? 'active' : '', onclick: () => { state.plane = !state.plane; planeBtn.classList.toggle('active', state.plane); update(); } }, 'plane through P and both centres');

  el.classList.add('widget');
  el.append(canvas,
    h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, viewButtons(stage, [view3d, viewFront]), planeBtn),
    h('div', { style: 'display:flex; gap:0.8em; margin-top:0.4em; align-items:flex-start' },
      h('div', { style: 'display:flex; gap:0.4em; flex:none' }, imL, imR),
      h('div', { style: 'display:flex; flex-direction:column; gap:0.3em; min-width:0' },
        readout,
        h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.1em; margin-top:0' },
          slider('P: x', -0.6, 1.4, 0.01, () => state.P[0], v => { state.P[0] = v; }, ' m'),
          slider('P: y', -0.6, 0.5, 0.01, () => state.P[1], v => { state.P[1] = v; }, ' m'),
          slider('P: z', 1.0, 4.0, 0.01, () => state.P[2], v => { state.P[2] = v; }, ' m'),
          slider('b', 0.3, 1.2, 0.01, () => state.b, v => { state.b = v; }, ' m')))));
  update();
}
