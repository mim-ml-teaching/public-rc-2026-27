// Euler angles (ZYX: yaw ψ, pitch θ, roll φ) on a quadcopter, with nested gimbal rings, and gimbal lock at θ = ±90°.
// World frame NED (x north, y east, z down), body frame FRD; R = Rz(ψ) Ry(θ) Rx(φ) maps body to world.
// Rings (as in lecture 1's rot3d "euler"): each ring is drawn in the plane perpendicular to its rotation axis
//   yaw ring (blue): axis z, frame Rz(ψ); pitch ring (green): axis y′, frame Rz(ψ)Ry(θ); roll ring (red): axis x″, frame R.
// At θ = ±90° the roll axis x″ coincides with ±z: the roll ring lies in the yaw ring, and ψ and φ do the same thing
// (only ψ − φ, or ψ + φ, matters). The quaternion readout changes smoothly through that pose.
// Scene: everything is built in NED inside a group whose fixed matrix maps NED to three.js (y up): (x, y, z) → (y, −z, −x).
// config: { width: 560, height: 420, yaw: 30, pitch: 30, roll: 0 }
import { C, h, fmt, isPrint } from './util.js';
import { THREE, makeStage, label, line, arrow3 } from './three-util.js';
import { deg, Rx, Ry, Rz, mul, I, toQuat } from './rotmath.js';

const setR = (obj, R) => {
  obj.matrixAutoUpdate = false;
  obj.matrix.set(R[0], R[1], R[2], 0, R[3], R[4], R[5], 0, R[6], R[7], R[8], 0, 0, 0, 0, 1);
  obj.matrixWorldNeedsUpdate = true;
};

function makeQuad() {
  const g = new THREE.Group();
  const mat = (c, o = {}) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.55, metalness: 0.2, ...o });
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.42, 0.3, 0.14), mat(0x3a3f4b)); g.add(body);
  const motors = [[1, 1, 'CCW'], [-1, -1, 'CCW'], [1, -1, 'CW'], [-1, 1, 'CW']];
  for (const [x, y, spin] of motors) {
    const px = 0.62 * x, py = 0.62 * y;
    const len = Math.hypot(px, py);
    const arm = new THREE.Mesh(new THREE.BoxGeometry(len, 0.06, 0.05), mat(0x9aa1ae));
    arm.position.set(px / 2, py / 2, 0); arm.rotation.z = Math.atan2(py, px); g.add(arm);
    const disc = new THREE.Mesh(new THREE.CylinderGeometry(0.3, 0.3, 0.02, 40), mat(spin === 'CCW' ? 0xb28dff : 0x5fd38d, { transparent: true, opacity: 0.55 }));
    disc.rotation.x = Math.PI / 2; disc.position.set(px, py, -0.08); g.add(disc);
    const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.06, 0.12, 20), mat(0xe7e9ee));
    hub.rotation.x = Math.PI / 2; hub.position.set(px, py, -0.03); g.add(hub);
  }
  // nose marker and body axes (x forward red, y right green, z down blue)
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.07, 0.2, 20), mat(0xf2b134));
  nose.rotation.z = -Math.PI / 2; nose.position.set(0.3, 0, 0); g.add(nose);
  g.add(arrow3([0, 0, 0], [1.15, 0, 0], 0xff6b6b, 0.12));
  g.add(arrow3([0, 0, 0], [0, 1.0, 0], 0x5fd38d, 0.12));
  g.add(arrow3([0, 0, 0], [0, 0, 0.8], 0x5ab0ff, 0.12));
  return g;
}

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 560, H = cfg.height || 420;
  const canvas = h('canvas', { width: W, height: H });
  const stage = makeStage(canvas, { position: cfg.camera || [-3.3, 2.5, 3.3], target: [0, 0.1, 0] });
  const { scene, render } = stage;
  const ned = new THREE.Group();
  ned.matrixAutoUpdate = false;
  ned.matrix.set(0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 1);
  scene.add(ned);
  // world axes N, E, D (dim)
  [[[2.3, 0, 0], 0xff6b6b, 'N', C.red], [[0, 2.3, 0], 0x5fd38d, 'E', C.green], [[0, 0, 1.6], 0x5ab0ff, 'D', C.blue]].forEach(([p, c, n, cc]) => {
    const l = line([[0, 0, 0], p], c); l.material.transparent = true; l.material.opacity = 0.4; ned.add(l);
    const lab = label(n, { color: cc, size: 0.34, font: '600 48px Inter, Arial' }); lab.position.set(...p.map(v => v * 1.08)); ned.add(lab);
  });
  const quad = new THREE.Group(), model = makeQuad(); model.scale.setScalar(1.3); quad.add(model); ned.add(quad);
  const ringMat = c => new THREE.MeshStandardMaterial({ color: c, roughness: 0.5, emissive: c, emissiveIntensity: 0.25 });
  const mkRing = (r, c) => { const m = new THREE.Mesh(new THREE.TorusGeometry(r, 0.03, 12, 96), ringMat(c)); ned.add(m); return m; };
  const A = { z: I(), y: Rx(90 * deg), x: Ry(90 * deg) };   // torus normal (local z) → the ring's axis
  const rings = { z: mkRing(1.75, 0x5ab0ff), y: mkRing(1.55, 0x5fd38d), x: mkRing(1.35, 0xff6b6b) };

  const st = { yaw: cfg.yaw ?? 30, pitch: cfg.pitch ?? (isPrint() ? 90 : 30), roll: cfg.roll ?? 0 };
  const readout = h('div', { class: 'readout', style: 'line-height:1.5' });
  const warn = h('div', { style: `color:${C.red}; font-weight:600; min-height:4.2em; font-size:0.85em; line-height:1.3` });
  const sliders = {};
  const slider = (key, name, min, max, color) => {
    const v = h('span', { class: 'readout', style: 'display:inline-block; width:3.2em; text-align:right' });
    const s = h('input', { type: 'range', min, max, step: 1, value: st[key], style: 'width:9em', oninput: e => { st[key] = +e.target.value; draw(); } });
    sliders[key] = { s, v };
    return h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', { style: `width:4.6em; color:${color}` }, name), s, v);
  };
  const anim = (key, to) => {
    const from = st[key], t0 = performance.now();
    const step = now => {
      const u = Math.min(1, (now - t0) / 1200), e = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
      st[key] = from + (to - from) * e; sliders[key].s.value = st[key]; draw();
      if (u < 1) requestAnimationFrame(step);
    };
    requestAnimationFrame(step);
  };
  el.append(h('div', { style: 'display:flex; gap:1em; align-items:flex-start' }, canvas,
    h('div', { style: 'width:300px; flex:none; display:flex; flex-direction:column; gap:0.4em; font-size:0.85em' },
      h('div', { class: 'small' }, 'blue: yaw ψ about z · green: pitch θ about y′ · red: roll φ about x″'),
      readout, warn,
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.2em' },
        slider('yaw', 'yaw ψ', -180, 180, C.blue), slider('pitch', 'pitch θ', -90, 90, C.green), slider('roll', 'roll φ', -180, 180, C.red),
        h('div', { style: 'display:flex; gap:0.3em' },
          h('button', { onclick: () => anim('pitch', 90) }, 'pitch → 90°'), h('button', { onclick: () => anim('pitch', 0) }, 'level')),
        h('div', { class: 'dim' }, 'drag to orbit')))));

  function draw() {
    const y = st.yaw * deg, p = st.pitch * deg, r = st.roll * deg;
    const F1 = Rz(y), F2 = mul(F1, Ry(p)), R = mul(F2, Rx(r));
    setR(rings.z, mul(F1, A.z)); setR(rings.y, mul(F2, A.y)); setR(rings.x, mul(R, A.x));
    setR(quad, R);
    const q = toQuat(R);
    readout.innerHTML = `ψ ${fmt(st.yaw, 0)}°, θ ${fmt(st.pitch, 0)}°, φ ${fmt(st.roll, 0)}°<br>` +
      `quaternion (w, x, y, z):<br>(${q.map(v => fmt(v, 2)).join(', ')})`;
    for (const k of ['yaw', 'pitch', 'roll']) sliders[k].v.textContent = `${Math.round(st[k])}°`;
    const lock = Math.abs(Math.abs(st.pitch) - 90) < 2;
    warn.textContent = lock
      ? `Gimbal lock: the roll ring lies in the yaw ring. ψ and φ turn about the same axis; only ${st.pitch > 0 ? 'ψ − φ' : 'ψ + φ'} = ${fmt(st.pitch > 0 ? st.yaw - st.roll : st.yaw + st.roll, 0)}° matters.`
      : '';
    render();
  }
  draw();
}
