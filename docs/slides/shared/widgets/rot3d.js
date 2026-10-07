// 3D rotation widgets. config.mode:
//   "elementary"  – sliders for a single rotation about x, y or z
//   "noncommute"  – two objects: Rx(90°) then Rz(90°) vs Rz(90°) then Rx(90°)
//   "euler"       – ZYX Euler angles with gimbal rings; shows gimbal lock at pitch = ±90°
//   "inspector"   – axis–angle input; readouts in all representations
//   "oneaxis"     – Euler's rotation theorem: Rx(α), Ry(β), Rz(γ) in turn vs one rotation by θ about k
//   "mirror"      – a gripper and its mirror image (static)
import { C, h, fmt, isPrint } from './util.js';
import { THREE, makeStage, label, line, arrow3 } from './three-util.js';
import { deg, Rx, Ry, Rz, mul, I, axisAngle, toAxisAngle, toQuat, toEulerZYX, eulerZYX } from './rotmath.js';
import { T, setRotation, makeGripper } from './gripper.js';

const makeBody = scale => makeGripper(scale);

function worldAxes(scene, len = 2, at = [0, 0, 0], names = ['x', 'y', 'z']) {
  const cols = [[0xff6b6b, C.red], [0x5fd38d, C.green], [0x5ab0ff, C.blue]];
  [[len, 0, 0], [0, len, 0], [0, 0, len]].forEach((d, i) => {
    const p0 = T(...at), p1 = T(at[0] + d[0], at[1] + d[1], at[2] + d[2]);
    const l = line([p0, p1], cols[i][0]); l.material.transparent = true; l.material.opacity = 0.45; scene.add(l);
    const lab = label(names[i], { color: cols[i][1], size: 0.24 });
    lab.position.set(...T(at[0] + d[0] * 1.08, at[1] + d[1] * 1.08, at[2] + d[2] * 1.08)); scene.add(lab);
  });
}

const matTeX = (R, d = 2) =>
  `\\begin{bmatrix}${[0, 1, 2].map(r => [0, 1, 2].map(c => fmt(R[3 * r + c], d)).join('&')).join('\\\\')}\\end{bmatrix}`;

function tex(el, s) {
  el.innerHTML = s;
  window.renderMathInElement?.(el, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
}

function slider(state, key, min, max, step, name, onChange) {
  const s = h('input', { type: 'range', min, max, step, value: state[key] });
  const val = h('span', { class: 'readout', style: 'display:inline-block; width:3.5em; text-align:right' });
  const upd = () => { val.textContent = `${Math.round(state[key])}°`; };
  s.addEventListener('input', () => { state[key] = parseFloat(s.value); upd(); onChange(); });
  upd();
  const row = h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', { style: 'width:4.2em' }, name), s, val);
  row.set = v => { state[key] = v; s.value = v; upd(); };
  return row;
}

function animateValue(from, to, dur, onStep, done) {
  const t0 = performance.now();
  const step = now => {
    const u = Math.min(1, (now - t0) / dur);
    const e = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
    onStep(from + (to - from) * e);
    if (u < 1) requestAnimationFrame(step); else done?.();
  };
  requestAnimationFrame(step);
}

export function mount(el, cfg) {
  const mode = cfg.mode || 'inspector';
  const W = cfg.width || 600, H = cfg.height || 440;
  const canvas = h('canvas', { width: W, height: H });
  const stage = makeStage(canvas, {
    position: cfg.camera || ({ noncommute: [0.4, 2.7, 5.9], oneaxis: [0.3, 2.1, 4.9], euler: [3.7, 2.9, 4.2], mirror: [0.3, 2.3, 4.6] }[mode] || [2.9, 2.2, 3.2]),
    target: cfg.target || [0, 0.2, 0],
  });
  const side = h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; min-width:14em' });
  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' }, canvas, side));
  ({ elementary, noncommute, oneaxis, euler, inspector, mirror })[mode](stage, side, cfg);
}

// ---------------------------------------------------------------------------
function elementary({ scene, render }, side, cfg) {
  worldAxes(scene, 2.1);
  const body = makeBody(); scene.add(body);
  const state = { axis: cfg.axis || 'z', angle: isPrint() ? 40 : 0 };
  const readout = h('div', { style: 'font-size:1.15em' });
  const draw = () => {
    const a = state.angle * deg;
    const R = { x: Rx, y: Ry, z: Rz }[state.axis](a);
    setRotation(body, R);
    tex(readout, `$R_${state.axis}(${Math.round(state.angle)}^\\circ) = ${matTeX(R)}$`);
    render();
  };
  const btns = ['x', 'y', 'z'].map(ax => h('button', {
    onclick: () => { state.axis = ax; btns.forEach(b => b.classList.toggle('active', b.textContent === `about ${ax}`)); draw(); },
  }, `about ${ax}`));
  btns.forEach(b => b.classList.toggle('active', b.textContent === `about ${state.axis}`));
  side.append(readout, h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start' },
    h('div', { style: 'display:flex; gap:0.3em' }, btns),
    slider(state, 'angle', -180, 180, 1, 'angle', draw),
    h('div', { class: 'dim' }, 'drag to orbit')));
  draw();
}

// ---------------------------------------------------------------------------
function noncommute({ scene, render }, side, cfg) {
  const off = 2.2;
  const A = { name: cfg.first || 'x', R: Rx }, B = { name: cfg.second || 'z', R: Rz };
  const L = makeBody(0.8), Rb = makeBody(0.8);
  scene.add(L, Rb);
  worldAxes(scene, 1.4, [-off, 0, 0]); worldAxes(scene, 1.4, [off, 0, 0]);
  const lab1 = label('Rx(90°) → Rz(90°)', { color: C.fg, size: 0.34, font: '44px Inter, Arial' });
  lab1.position.set(...T(-off, 0, 2.1)); scene.add(lab1);
  const lab2 = label('Rz(90°) → Rx(90°)', { color: C.fg, size: 0.34, font: '44px Inter, Arial' });
  lab2.position.set(...T(off, 0, 2.1)); scene.add(lab2);
  const state = { p: isPrint() ? 2 : 0 };
  const readout = h('div', { style: 'font-size:1.0em; line-height:1.9' });
  const draw = () => {
    const a1 = Math.min(1, state.p) * 90 * deg, a2 = Math.max(0, state.p - 1) * 90 * deg;
    // rotations about the fixed world axes: the later one multiplies from the left
    const left = mul(Rz(a2), Rx(a1)), right = mul(Rx(a2), Rz(a1));
    setRotation(L, left, T(-off, 0, 0)); setRotation(Rb, right, T(off, 0, 0));
    tex(readout,
      `left, $R_x$ first: $R_z R_x = ${matTeX(mul(Rz(90 * deg), Rx(90 * deg)), 0)}$<br>` +
      `right, $R_z$ first: $R_x R_z = ${matTeX(mul(Rx(90 * deg), Rz(90 * deg)), 0)}$`);
    render();
  };
  const s = h('input', { type: 'range', min: 0, max: 2, step: 0.01, value: state.p });
  s.addEventListener('input', () => { state.p = parseFloat(s.value); draw(); });
  const play = h('button', { onclick: () => animateValue(0, 2, 2400, v => { state.p = v; s.value = v; draw(); }) }, '▶ play both');
  side.append(readout, h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start' },
    h('div', {}, h('label', {}, 'progress '), s), play));
  draw();
  if (cfg.autoplay && !isPrint()) setTimeout(() => play.click(), 500);
}

// ---------------------------------------------------------------------------
// Euler's rotation theorem: three rotations about the fixed axes, one after another (left), end in the
// same pose as a single rotation by θ about the axis k of their product (right).
function oneaxis({ scene, render }, side, cfg) {
  const off = 2.0;
  const L = makeBody(0.8), Rb = makeBody(0.8);
  scene.add(L, Rb);
  worldAxes(scene, 1.4, [-off, 0, 0]); worldAxes(scene, 1.4, [off, 0, 0]);
  const lab1 = label('three rotations', { color: C.fg, size: 0.28, font: '44px Inter, Arial' });
  lab1.position.set(...T(-off, 0, -1.25)); scene.add(lab1);
  const lab2 = label('one rotation about k', { color: C.fg, size: 0.28, font: '44px Inter, Arial' });
  lab2.position.set(...T(off, 0, -1.25)); scene.add(lab2);
  const axisGroup = new THREE.Group(); scene.add(axisGroup);
  const state = { a: cfg.a ?? 60, b: cfg.b ?? 45, c: cfg.c ?? 90, p: isPrint() ? 3 : 0 };
  const readout = h('div', { style: 'font-size:1.0em; line-height:1.9' });
  const draw = () => {
    const a = state.a * deg, b = state.b * deg, c = state.c * deg;
    const R = mul(Rz(c), mul(Ry(b), Rx(a)));          // Rx first, Rz last (fixed axes: later multiplies from the left)
    const { axis: k, angle: th } = toAxisAngle(R);
    const u = i => Math.min(1, Math.max(0, state.p - i)); // progress of step i (0, 1, 2)
    setRotation(L, mul(Rz(c * u(2)), mul(Ry(b * u(1)), Rx(a * u(0)))), T(-off, 0, 0));
    setRotation(Rb, axisAngle(k, th * state.p / 3), T(off, 0, 0));
    axisGroup.clear();
    const at = (t) => T(off + k[0] * t, k[1] * t, k[2] * t);
    axisGroup.add(line([at(-1.6), at(1.8)], 0xb28dff));
    axisGroup.add(arrow3(at(1.5), at(1.9), 0xb28dff, 0.16));
    const lk = label('k', { color: C.purple, size: 0.3 }); lk.position.set(...at(2.05)); axisGroup.add(lk);
    const step = state.p >= 3 ? 3 : Math.floor(state.p);
    const hl = (i, s) => (step === i ? `\\color{${C.accent}}{${s}}` : s);
    tex(readout,
      `<div>left: $${hl(0, `R_x(${state.a}^\\circ)`)}$, then $${hl(1, `R_y(${state.b}^\\circ)`)}$, then $${hl(2, `R_z(${state.c}^\\circ)`)}$</div>` +
      `<div>$R = R_z R_y R_x = ${matTeX(R)}$</div>` +
      `<div>right: $\\theta = ${fmt(th / deg, 0)}^\\circ$ about $k = (${k.map(v => fmt(v)).join(', ')})$</div>`);
    render();
  };
  const s = h('input', { type: 'range', min: 0, max: 3, step: 0.01, value: state.p });
  s.addEventListener('input', () => { state.p = parseFloat(s.value); draw(); });
  const play = h('button', { onclick: () => animateValue(0, 3, 4500, v => { state.p = v; s.value = v; draw(); }) }, '▶ play both');
  const angle = (key, name) => slider(state, key, -180, 180, 1, name, () => { state.p = 3; s.value = 3; draw(); });
  side.append(readout, h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start' },
    h('div', {}, h('label', {}, 'progress '), s), play,
    angle('a', 'α (x)'), angle('b', 'β (y)'), angle('c', 'γ (z)')));
  draw();
  if (cfg.autoplay && !isPrint()) setTimeout(() => play.click(), 500);
}

// ---------------------------------------------------------------------------
function euler({ scene, render }, side, cfg) {
  worldAxes(scene, 2.3);
  const body = makeBody(0.75); scene.add(body);
  const ringMat = c => new THREE.MeshStandardMaterial({ color: c, roughness: 0.5, emissive: c, emissiveIntensity: 0.25 });
  const mkRing = (r, c) => { const m = new THREE.Mesh(new THREE.TorusGeometry(r, 0.035, 12, 96), ringMat(c)); scene.add(m); return m; };
  // torus lies in three's XY plane: normal = three +Z = math −y. A_e rotates that normal onto axis e.
  const A = { z: Rx(90 * deg), y: I(), x: Rz(90 * deg) };
  const rings = { z: mkRing(1.9, 0x5ab0ff), y: mkRing(1.7, 0x5fd38d), x: mkRing(1.5, 0xff6b6b) };
  const state = { yaw: cfg.yaw ?? 30, pitch: cfg.pitch ?? (isPrint() ? 90 : 20), roll: cfg.roll ?? 0 };
  const readout = h('div', { style: 'font-size:1.0em' });
  const warn = h('div', { style: `color:${C.red}; font-weight:600; min-height:2.6em` });
  const draw = () => {
    const y = state.yaw * deg, p = state.pitch * deg, r = state.roll * deg;
    const F1 = Rz(y), F2 = mul(F1, Ry(p)), F3 = mul(F2, Rx(r));
    setRotation(rings.z, mul(F1, A.z)); setRotation(rings.y, mul(F2, A.y)); setRotation(rings.x, mul(F3, A.x));
    setRotation(body, F3);
    tex(readout, `$R = R_z(\\psi)\\,R_y(\\theta)\\,R_x(\\varphi) = ${matTeX(F3)}$`);
    warn.textContent = Math.abs(Math.abs(state.pitch) - 90) < 3
      ? 'Gimbal lock: the roll ring (red) lies in the yaw ring (blue) — ψ and φ now rotate about the same axis.'
      : '';
    render();
  };
  const sy = slider(state, 'yaw', -180, 180, 1, 'yaw ψ', draw);
  const sp = slider(state, 'pitch', -90, 90, 1, 'pitch θ', draw);
  const sr = slider(state, 'roll', -180, 180, 1, 'roll φ', draw);
  const lock = h('button', { onclick: () => animateValue(state.pitch, 90, 1200, v => { sp.set(v); draw(); }) }, 'pitch → 90°');
  side.append(readout, warn, h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start' },
    h('div', { class: 'dim' }, 'blue: yaw about z · green: pitch about y′ · red: roll about x″'),
    sy, sp, sr, lock));
  draw();
}

// ---------------------------------------------------------------------------
function inspector({ scene, render }, side, cfg) {
  worldAxes(scene, 2.1);
  const body = makeBody(); scene.add(body);
  const axisGroup = new THREE.Group(); scene.add(axisGroup);
  const state = { az: cfg.az ?? 40, el: cfg.el ?? 35, angle: cfg.angle ?? 120, t: 1 };
  const readout = h('div', { style: 'font-size:0.95em; line-height:1.55' });
  const draw = () => {
    const az = state.az * deg, elv = state.el * deg;
    const k = [Math.cos(elv) * Math.cos(az), Math.cos(elv) * Math.sin(az), Math.sin(elv)];
    const th = state.angle * deg * state.t;
    const R = axisAngle(k, th);
    setRotation(body, R);
    axisGroup.clear();
    axisGroup.add(line([T(...k.map(v => -2 * v)), T(...k.map(v => 2.2 * v))], 0xb28dff));
    axisGroup.add(arrow3(T(...k.map(v => 1.9 * v)), T(...k.map(v => 2.3 * v)), 0xb28dff, 0.16));
    const lk = label('k', { color: C.purple, size: 0.3 }); lk.position.set(...T(...k.map(v => 2.45 * v))); axisGroup.add(lk);
    const q = toQuat(R), e = toEulerZYX(R), aa = toAxisAngle(R);
    const w = aa.axis.map(v => v * aa.angle);
    tex(readout,
      `<div>matrix: $${matTeX(R)}$</div>` +
      `<div>axis–angle: $k = (${k.map(v => fmt(v)).join(', ')}),\\ \\theta = ${fmt(th / deg, 0)}^\\circ$</div>` +
      `<div>rotation vector: $\\omega = \\theta k = (${w.map(v => fmt(v)).join(', ')})$</div>` +
      `<div>quaternion $(w,x,y,z)$: $(${q.map(v => fmt(v)).join(', ')})$</div>` +
      `<div>Euler ZYX $(\\psi,\\theta,\\varphi)$: $(${[e.yaw, e.pitch, e.roll].map(v => fmt(v / deg, 0) + '^\\circ').join(', ')})$</div>`);
    render();
  };
  const play = h('button', { onclick: () => animateValue(0, 1, 1500, v => { state.t = v; draw(); }) }, '▶ rotate from identity');
  side.append(readout, h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start' },
    slider(state, 'az', -180, 180, 1, 'axis az', draw),
    slider(state, 'el', -90, 90, 1, 'axis el', draw),
    slider(state, 'angle', -180, 180, 1, 'angle θ', draw),
    play));
  draw();
}

// ---------------------------------------------------------------------------
function mirror({ scene, render }, side) {
  const off = 1.7;
  const D = [-1, 0, 0, 0, 1, 0, 0, 0, 1];                 // reflection x -> -x (det = -1)
  const orig = makeBody(0.8), mir = makeBody(0.8);
  scene.add(orig, mir);
  setRotation(orig, I(), T(-off, 0, 0));
  setRotation(mir, D, T(off, 0, 0));
  const l1 = label('original: right-handed', { color: C.fg, size: 0.3, font: '44px Inter, Arial' });
  l1.position.set(...T(-off, 0, 1.7)); scene.add(l1);
  const l2 = label('mirror image: left-handed', { color: C.fg, size: 0.3, font: '44px Inter, Arial' });
  l2.position.set(...T(off, 0, 1.7)); scene.add(l2);
  side.append(h('div', { class: 'small', style: 'font-size:0.95em' }, 'det = −1 for the reflection, +1 for every rotation: no rotation can undo it.'));
  render();
}
