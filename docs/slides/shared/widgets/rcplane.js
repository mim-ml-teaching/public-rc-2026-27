// An RC plane in 3D with a Mode-2 transmitter. Three modes:
//   "fly":       drag the sticks (left: throttle ↕, rudder ↔; right: elevator ↕, ailerons ↔; the right stick and
//                the rudder spring back to the centre). Toy dynamics with body rates:
//                  p = 1.3·δa − 1.5·φ (dihedral levels the wings), q = 0.9·δe − 1.5·θ (a trimmed trainer returns to
//                  level flight), r = 0.6·δr + (g/V)·sin φ (a banked plane turns); full stick gives about 50° of bank
//                  or 35° of pitch, released sticks level the plane within about a second; the ground scrolls with V.
//   "surfaces":  pick one input; it is moved back and forth, the surface deflects, the air force on it is drawn,
//                and the plane rotates about the matching body axis (the deflection sets a rotation *rate*).
//   "stability": sticks centred; a gust disturbs the nose up, the nose sideways or the wings; the tail, the fin
//                and the dihedral bring it back (damped second order for pitch and yaw, first order for roll),
//                with the restoring force drawn and the angle plotted against time.
// Frames: body FRD (x forward, y right, z down), world NED; built inside a group that maps NED to three.js
// (x, y, z) → (y, −z, −x), as in gimbalquad. R = Rz(ψ) Ry(θ) Rx(φ).
// config: { mode: "fly" | "surfaces" | "stability", width: 640, height: 420, input: "ailerons" }
import { C, h, fmt, isPrint } from './util.js';
import { THREE, makeStage, line, arrow3 } from './three-util.js';
import { deg, Rx, Ry, Rz, mul, I, toEulerZYX } from './rotmath.js';

const setR = (obj, R) => {
  obj.matrixAutoUpdate = false;
  obj.matrix.set(R[0], R[1], R[2], 0, R[3], R[4], R[5], 0, R[6], R[7], R[8], 0, 0, 0, 0, 1);
  obj.matrixWorldNeedsUpdate = true;
};
const COL = { ail: 0xff6b6b, ele: 0x5fd38d, rud: 0x5ab0ff, thr: 0xf2b134 };
const DIH = 6 * deg;                                     // dihedral of each wing half

function box(sx, sy, sz, color, pos = [0, 0, 0], o = {}) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.15, ...o }));
  m.position.set(...pos);
  return m;
}
// a control surface hinged at `hinge`; the mesh lies behind the hinge (−x); returns the hinge group
function surface(parent, hinge, size, color) {
  const g = new THREE.Group(); g.position.set(...hinge);
  g.add(box(size[0], size[1], size[2], color, [-size[0] / 2, 0, 0], { emissive: color, emissiveIntensity: 0.2 }));
  parent.add(g);
  return g;
}

function makePlane() {
  const P = new THREE.Group(), grey = 0x9aa1ae, light = 0xd9dce2;
  P.add(box(0.72, 0.15, 0.18, light, [0.06, 0, 0]));                 // fuselage, front
  P.add(box(0.7, 0.07, 0.08, light, [-0.64, 0, -0.03]));             // tail boom
  P.add(box(0.2, 0.12, 0.07, 0x5a606c, [0.16, 0, -0.12]));           // canopy
  const spinner = new THREE.Mesh(new THREE.ConeGeometry(0.06, 0.12, 20), new THREE.MeshStandardMaterial({ color: 0xf2b134 }));
  spinner.rotation.z = -Math.PI / 2; spinner.position.set(0.48, 0, 0); P.add(spinner);
  const prop = new THREE.Group(); prop.position.set(0.45, 0, 0);
  prop.add(box(0.02, 0.62, 0.05, 0x3a3f4b)); P.add(prop);
  const disc = new THREE.Mesh(new THREE.CircleGeometry(0.31, 40), new THREE.MeshBasicMaterial({ color: 0xf2b134, transparent: true, opacity: 0, side: THREE.DoubleSide }));
  disc.rotation.y = Math.PI / 2; disc.position.set(0.455, 0, 0); P.add(disc);
  for (const s of [-1, 1]) {                                          // landing gear
    const strut = box(0.03, 0.03, 0.2, grey, [0.2, 0.12 * s, 0.17]); strut.rotation.x = -0.45 * s; P.add(strut);
    const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.03, 20), new THREE.MeshStandardMaterial({ color: 0x272b34 }));
    wheel.position.set(0.2, 0.17 * s, 0.26); P.add(wheel);
  }
  // wing: two halves with dihedral, high wing; ailerons on the outer trailing edge
  const wing = new THREE.Group(); wing.position.set(0, 0, -0.12); P.add(wing);
  const ail = {};
  for (const s of [-1, 1]) {
    const half = new THREE.Group(); half.rotation.x = -s * DIH; wing.add(half);
    half.add(box(0.22, 1.0, 0.03, light, [0.04, 0.5 * s, 0]));       // main panel, x −0.07…0.15
    half.add(box(0.08, 0.55, 0.03, light, [-0.11, 0.275 * s, 0]));  // inboard trailing part
    ail[s] = surface(half, [-0.07, 0.775 * s, 0], [0.085, 0.44, 0.022], COL.ail);
  }
  // tail
  P.add(box(0.16, 0.72, 0.02, light, [-0.8, 0, -0.03]));             // stabilizer
  const ele = surface(P, [-0.88, 0, -0.03], [0.085, 0.72, 0.018], COL.ele);
  P.add(box(0.18, 0.02, 0.33, light, [-0.83, 0, -0.215]));           // fin
  const rud = new THREE.Group(); rud.position.set(-0.92, 0, -0.215);
  rud.add(box(0.085, 0.018, 0.33, COL.rud, [-0.0425, 0, 0], { emissive: COL.rud, emissiveIntensity: 0.2 })); P.add(rud);
  return { P, prop, disc, ail, ele, rud };
}

// Mode-2 transmitter (SVG). values: thr 0…1, rud/ail/ele −1…1 (ele +1 = stick pulled back = nose up).
function transmitter(onChange, interactive) {
  const NS = 'http://www.w3.org/2000/svg', S = (t, a = {}, txt) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); if (txt !== undefined) e.textContent = txt; return e; };
  const W = 300, Hh = 190, pad = 92, svg = S('svg', { viewBox: `0 0 ${W} ${Hh}`, width: W, height: Hh, style: 'touch-action:none; user-select:none' });
  svg.append(S('rect', { x: 2, y: 22, width: W - 4, height: Hh - 24, rx: 22, fill: C.bg3, stroke: C.line, 'stroke-width': 2 }));
  svg.append(S('line', { x1: 60, y1: 22, x2: 50, y2: 2, stroke: C.dim, 'stroke-width': 3 }));   // antenna
  const v = { thr: 0.5, rud: 0, ail: 0, ele: 0 };
  const pads = [{ x0: 32, keyX: 'rud', keyY: 'thr', lx: 'rudder', ly: 'throttle', cx: C.blue, cy: C.accent },
                { x0: W - 32 - pad, keyX: 'ail', keyY: 'ele', lx: 'ailerons', ly: 'elevator', cx: C.red, cy: C.green }];
  const y0 = 52;
  for (const p of pads) {
    p.g = S('g'); svg.append(p.g);
    p.g.append(S('rect', { x: p.x0, y: y0, width: pad, height: pad, rx: 10, fill: C.bg2, stroke: C.line }));
    p.g.append(S('line', { x1: p.x0 + 8, y1: y0 + pad / 2, x2: p.x0 + pad - 8, y2: y0 + pad / 2, stroke: p.cx, 'stroke-opacity': 0.5, 'stroke-width': 2 }));
    p.g.append(S('line', { x1: p.x0 + pad / 2, y1: y0 + 8, x2: p.x0 + pad / 2, y2: y0 + pad - 8, stroke: p.cy, 'stroke-opacity': 0.5, 'stroke-width': 2 }));
    p.g.append(S('text', { x: p.x0 + pad / 2, y: y0 + pad + 18, fill: p.cx, 'font-size': 14, 'text-anchor': 'middle', 'font-family': 'Inter, Arial' }, `↔ ${p.lx}`));
    p.g.append(S('text', { x: p.x0 + pad / 2, y: y0 - 8, fill: p.cy, 'font-size': 14, 'text-anchor': 'middle', 'font-family': 'Inter, Arial' }, `↕ ${p.ly}`));
    p.stem = S('line', { stroke: C.dim, 'stroke-width': 5, 'stroke-linecap': 'round' }); p.g.append(p.stem);
    p.knob = S('circle', { r: 13, fill: C.fg, stroke: C.bg, 'stroke-width': 2, style: interactive ? 'cursor:grab' : '' }); p.g.append(p.knob);
  }
  const pos = p => {                                       // value → knob position
    // screen up = throttle up; elevator +1 = stick pulled back towards the pilot = knob DOWN (nose up)
    const ux = v[p.keyX], up = p.keyY === 'thr' ? 2 * v.thr - 1 : -v.ele;
    return [p.x0 + pad / 2 + ux * (pad / 2 - 14), y0 + pad / 2 - up * (pad / 2 - 14)];
  };
  const draw = () => pads.forEach(p => {
    const [x, y] = pos(p), cx = p.x0 + pad / 2, cy = y0 + pad / 2;
    p.stem.setAttribute('x1', cx); p.stem.setAttribute('y1', cy); p.stem.setAttribute('x2', x); p.stem.setAttribute('y2', y);
    p.knob.setAttribute('cx', x); p.knob.setAttribute('cy', y);
  });
  if (interactive) {
    let active = null;
    const toVal = (p, ev) => {
      const r = svg.getBoundingClientRect(), sx = (ev.clientX - r.left) * W / r.width, sy = (ev.clientY - r.top) * Hh / r.height;
      const clamp = t => Math.max(-1, Math.min(1, t));
      const ux = clamp((sx - p.x0 - pad / 2) / (pad / 2 - 14)), uy = clamp((sy - y0 - pad / 2) / (pad / 2 - 14));
      v[p.keyX] = ux;
      if (p.keyY === 'thr') v.thr = (1 - uy) / 2; else v.ele = uy;   // right stick pulled down (towards the pilot) = nose up
      draw(); onChange(v);
    };
    pads.forEach(p => p.g.addEventListener('pointerdown', ev => { active = p; svg.setPointerCapture(ev.pointerId); toVal(p, ev); }));
    svg.addEventListener('pointermove', ev => { if (active) toVal(active, ev); });
    const release = () => {                                 // springs: right stick and rudder back to the centre
      if (!active) return;
      active = null; v.rud = 0; v.ail = 0; v.ele = 0; draw(); onChange(v);
    };
    svg.addEventListener('pointerup', release); svg.addEventListener('pointercancel', release);
  }
  draw();
  return { svg, v, set(nv) { Object.assign(v, nv); draw(); } };
}

export function mount(el, cfg) {
  el.classList.add('widget');
  const mode = cfg.mode || 'fly', W = cfg.width || 640, H = cfg.height || 420, print = isPrint();
  const canvas = h('canvas', { width: W, height: H });
  const cam = { fly: [-3.6, 1.9, 2.6], surfaces: [-2.6, 1.6, 2.9], stability: [-0.6, 0.9, 3.6] }[mode];
  const stage = makeStage(canvas, { position: cfg.camera || cam, target: [0, 0, 0] });
  const { scene, render } = stage;
  const ned = new THREE.Group(); ned.matrixAutoUpdate = false;
  ned.matrix.set(0, 1, 0, 0, 0, 0, -1, 0, -1, 0, 0, 0, 0, 0, 0, 1);
  scene.add(ned);
  const body = new THREE.Group(); ned.add(body);
  const pl = makePlane(); pl.P.scale.setScalar(1.25); body.add(pl.P);
  const sc = 1.25;

  // ground (three.js horizontal plane), only in fly mode
  // a checkerboard mesh (GridHelper lines vanished near the camera at grazing angles); it scrolls via the texture
  const gc = document.createElement('canvas'); gc.width = gc.height = 64;
  const gg = gc.getContext('2d'); gg.fillStyle = '#23272e'; gg.fillRect(0, 0, 64, 64);
  gg.fillStyle = '#2e333c'; gg.fillRect(0, 0, 32, 32); gg.fillRect(32, 32, 32, 32);
  const gtex = new THREE.CanvasTexture(gc); gtex.wrapS = gtex.wrapT = THREE.RepeatWrapping; gtex.repeat.set(20, 20);
  gtex.magFilter = THREE.NearestFilter; gtex.anisotropy = 8; gtex.colorSpace = THREE.SRGBColorSpace;
  const grid = new THREE.Mesh(new THREE.PlaneGeometry(80, 80), new THREE.MeshBasicMaterial({ map: gtex }));
  grid.rotation.x = -Math.PI / 2;
  if (mode === 'fly') scene.add(grid);

  // force arrows (body frame), axis line
  const arrow = (color) => { const a = arrow3([0, 0, 0], [0, 0, -0.5], color, 0.12); a.visible = false; body.add(a); return a; };
  const fa = { ailR: arrow(COL.ail), ailL: arrow(COL.ail), ele: arrow(COL.ele), rud: arrow(COL.rud), thr: arrow(COL.thr), wing: arrow(0xf2b134) };
  const setArrow = (a, from, dir, len) => {
    if (Math.abs(len) < 0.02) { a.visible = false; return; }
    a.visible = true; a.position.set(...from);
    const d = new THREE.Vector3(...dir).multiplyScalar(Math.sign(len)).normalize();
    a.setDirection(d); a.setLength(Math.abs(len), Math.min(0.14, Math.abs(len) * 0.4), Math.min(0.09, Math.abs(len) * 0.25));
  };
  const axisLine = {};
  [['x', [[-1.9, 0, 0], [1.9, 0, 0]], COL.ail], ['y', [[0, -1.9, 0], [0, 1.9, 0]], COL.ele], ['z', [[0, 0, -1.4], [0, 0, 1.4]], COL.rud]].forEach(([k, pts, c]) => {
    const l = line(pts, c, { dashed: true }); l.visible = false; ned.add(l); axisLine[k] = l;
  });

  // ---- state ----
  const st = { R: I(), defl: { ail: 0, ele: 0, rud: 0 }, thr: 0.5, propA: 0, n: 0, e: 0, alt: 1.6,
    input: cfg.input || 'ailerons', t: 0, gust: null, trace: [] };
  const tx = transmitter(v => { st.cmd = { ...v }; }, mode === 'fly');
  st.cmd = { ...tx.v };

  function pose(defl, thr) {                                  // surfaces and propeller
    pl.ail[1].rotation.y = -0.45 * defl.ail;                  // stick right: right aileron up (TE up = −)
    pl.ail[-1].rotation.y = 0.45 * defl.ail;                  // left aileron down
    pl.ele.rotation.y = -0.45 * defl.ele;                     // stick back: elevator TE up
    pl.rud.rotation.z = -0.45 * defl.rud;                     // rudder right (+): TE to the right (−rotation about z)
    pl.disc.material.opacity = 0.1 + 0.3 * thr;
    pl.prop.rotation.x = st.propA;
  }
  const yA = 0.775 * sc, zA = (-0.12 - 0.775 * Math.sin(DIH)) * sc;

  // ---- side panel ----
  const readout = h('div', { class: 'readout', style: 'width:300px; line-height:1.5; font-size:0.8em; min-height:3em' });
  const traceSvg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  traceSvg.setAttribute('viewBox', '0 0 300 130'); traceSvg.setAttribute('width', 300); traceSvg.setAttribute('height', 130);
  const ctl = h('div', { class: 'wctl interactive-only', style: 'flex-wrap:wrap; gap:0.3em; width:300px' });
  const side = h('div', { style: 'width:300px; flex:none; display:flex; flex-direction:column; gap:0.5em; font-size:0.85em' });
  const INPUTS = { ailerons: ['ail', C.red, 'roll'], elevator: ['ele', C.green, 'pitch'], rudder: ['rud', C.blue, 'yaw'], throttle: ['thr', C.accent, 'speed'] };
  const GUSTS = { 'nose up': 'pitch', 'nose sideways': 'yaw', 'roll': 'roll' };
  if (mode === 'fly') {
    side.append(tx.svg, readout, h('div', { class: 'small dim interactive-only' }, 'drag the sticks; the right stick and the rudder spring back'),
      h('div', { class: 'wctl interactive-only' }, h('button', { onclick: () => { st.R = I(); st.alt = 1.6; } }, 'level again')));
  } else if (mode === 'surfaces') {
    const btns = Object.entries(INPUTS).map(([name, [, col]]) => {
      const b = h('button', { style: `border-color:${col}`, onclick: () => { st.input = name; st.t = 0; st.R = I(); btns.forEach(x => x.classList.toggle('active', x === b)); } }, name);
      b.classList.toggle('active', name === st.input);
      return b;
    });
    ctl.append(...btns);
    side.append(tx.svg, ctl, readout);
  } else {
    const btns = Object.entries(GUSTS).map(([name, kind]) => h('button', { onclick: () => startGust(kind) }, `gust: ${name}`));
    ctl.append(...btns);
    side.append(h('div', { class: 'small' }, 'sticks centred, nobody touches them'), ctl, traceSvg, readout);
  }
  el.append(h('div', { style: 'display:flex; gap:1em; align-items:flex-start' }, canvas, side));

  // ---- stability: disturbance offsets relative to straight and level flight ----
  const GP = { pitch: { w: 4.0, z: 0.35, a0: 25 }, yaw: { w: 3.0, z: 0.22, a0: 30 }, roll: { tau: 1.4, a0: 35 } };
  function startGust(kind) { st.gust = { kind, a: GP[kind].a0 * deg, v: 0, t: 0 }; st.trace = []; }
  function stepGust(dt) {
    const g = st.gust; if (!g) return;
    if (g.kind === 'roll') g.a += dt * (-g.a / GP.roll.tau);
    else { const p = GP[g.kind]; const acc = -p.w * p.w * g.a - 2 * p.z * p.w * g.v; g.v += acc * dt; g.a += g.v * dt; }
    g.t += dt;
    if (g.t < 6) st.trace.push([g.t, g.a / deg]);
  }
  function drawTrace() {
    const NS = 'http://www.w3.org/2000/svg', g = st.gust;
    traceSvg.replaceChildren();
    const X = t => 40 + 250 * t / 6, Y = a => 65 - a * 1.5;
    const mk = (t, a) => { const e = document.createElementNS(NS, t); for (const k in a) e.setAttribute(k, a[k]); return e; };
    traceSvg.append(mk('rect', { x: 0, y: 0, width: 300, height: 130, rx: 6, fill: C.bg2 }));
    traceSvg.append(mk('line', { x1: 40, y1: 65, x2: 292, y2: 65, stroke: C.line }));
    [[30, '30°'], [-30, '−30°'], [0, '0°']].forEach(([a, t]) => { const e = mk('text', { x: 34, y: Y(a) + 4, fill: C.dim, 'font-size': 12, 'text-anchor': 'end' }); e.textContent = t; traceSvg.append(e); });
    const lab = mk('text', { x: 292, y: 124, fill: C.dim, 'font-size': 12, 'text-anchor': 'end' }); lab.textContent = 'time 0…6 s'; traceSvg.append(lab);
    if (!g || st.trace.length < 2) return;
    const col = { pitch: C.green, yaw: C.blue, roll: C.red }[g.kind];
    traceSvg.append(mk('path', { d: st.trace.map(([t, a], i) => `${i ? 'L' : 'M'}${X(t).toFixed(1)},${Y(a).toFixed(1)}`).join(''), fill: 'none', stroke: col, 'stroke-width': 2.5 }));
    const tl = mk('text', { x: 46, y: 16, fill: col, 'font-size': 13 }); tl.textContent = { pitch: 'pitch θ', yaw: 'yaw ψ (nose vs flight path)', roll: 'roll φ' }[g.kind]; traceSvg.append(tl);
  }

  // ---- main loop ----
  let last = performance.now();
  function frame(now, dtForced) {
    const dt = dtForced ?? Math.min(0.05, (now - last) / 1000); last = now;
    let R = st.R, defl = { ail: 0, ele: 0, rud: 0 }, thr = 0.5;
    Object.values(fa).forEach(a => { a.visible = false; });
    Object.values(axisLine).forEach(l => { l.visible = false; });
    if (mode === 'fly') {
      const c = st.cmd; thr = c.thr;
      for (const k of ['ail', 'ele', 'rud']) st.defl[k] += (c[k] - st.defl[k]) * Math.min(1, dt / 0.08);
      defl = st.defl;
      const { pitch: th, roll: ph } = toEulerZYX(R), V = 6 + 14 * thr;
      const p = 1.3 * defl.ail - 1.5 * ph, q = 0.9 * defl.ele - 1.5 * th, r = 0.6 * defl.rud + 9.81 / V * Math.sin(ph);
      const w = Math.hypot(p, q, r) * dt;
      if (w > 1e-9) {                                       // R ← R · exp([ω]× dt), Rodrigues
        const k = [p * dt / w, q * dt / w, r * dt / w], c1 = Math.cos(w), s1 = Math.sin(w), v1 = 1 - c1;
        const Rw = [c1 + k[0] * k[0] * v1, k[0] * k[1] * v1 - k[2] * s1, k[0] * k[2] * v1 + k[1] * s1,
          k[1] * k[0] * v1 + k[2] * s1, c1 + k[1] * k[1] * v1, k[1] * k[2] * v1 - k[0] * s1,
          k[2] * k[0] * v1 - k[1] * s1, k[2] * k[1] * v1 + k[0] * s1, c1 + k[2] * k[2] * v1];
        R = st.R = mul(R, Rw);
      }
      // move over the ground along the nose direction (scaled down so the grid motion stays readable)
      const f = [R[0], R[3], R[6]];
      st.n += f[0] * V * dt * 0.25; st.e += f[1] * V * dt * 0.25; st.alt = Math.max(0.9, Math.min(4, st.alt - f[2] * V * dt * 0.25));
      grid.position.y = -st.alt; gtex.offset.set(st.e / 4, st.n / 4);    // texture period 80 / 20 = 4 units; a larger offset moves the pattern south/west, i.e. backwards
      const { yaw: ps, pitch: t2, roll: p2 } = toEulerZYX(R);
      readout.innerHTML = `roll φ ${fmt(p2 / deg, 0).padStart(4)}°   pitch θ ${fmt(t2 / deg, 0).padStart(4)}°<br>heading ψ ${fmt(((ps / deg) + 360) % 360, 0).padStart(4)}°   speed ${fmt(V, 0)} m/s`;
    } else if (mode === 'surfaces') {
      st.t += dt;
      const [key, , name] = INPUTS[st.input], T = 4, u = Math.sin(2 * Math.PI * st.t / T);
      if (key === 'thr') {
        thr = 0.5 + 0.5 * u; tx.set({ thr, ail: 0, ele: 0, rud: 0 });
        setArrow(fa.thr, [0.62 * sc, 0, 0], [1, 0, 0], 0.25 + 0.7 * thr);
        R = I();
        readout.innerHTML = `throttle ${fmt(100 * thr, 0)}%: more thrust, more airspeed<br>(more lift: the plane climbs)`;
      } else {
        defl[key] = u; tx.set({ thr: 0.5, ail: 0, ele: 0, rud: 0, [key]: u });
        // rotation angle = integral of the deflection: rate ∝ deflection
        const A = 0.35 * (1 - Math.cos(2 * Math.PI * st.t / T));
        R = key === 'ail' ? Rx(A) : key === 'ele' ? Ry(A) : Rz(A);
        axisLine[{ ail: 'x', ele: 'y', rud: 'z' }[key]].visible = true;
        const L = 0.8 * u;
        if (key === 'ail') {                                  // right aileron up → less lift (down), left down → more lift (up)
          setArrow(fa.ailR, [-0.1 * sc, yA, zA], [0, 0, 1], L);
          setArrow(fa.ailL, [-0.1 * sc, -yA, zA], [0, 0, -1], L);
        } else if (key === 'ele') setArrow(fa.ele, [-1.1 * sc, 0, -0.03 * sc], [0, 0, 1], L);   // TE up → tail pushed down
        else setArrow(fa.rud, [-1.17 * sc, 0, -0.215 * sc], [0, -1, 0], L);                        // TE right → tail pushed left
        readout.innerHTML = `${st.input}: ${name} ${u >= 0 ? (key === 'ele' ? 'up' : 'right') : (key === 'ele' ? 'down' : 'left')}<br>` +
          `held deflection = rotation <i>rate</i>; centre the stick and the rotation stops`;
      }
    } else {
      stepGust(dt);
      const g = st.gust, a = g ? g.a : 0;
      R = !g ? I() : g.kind === 'pitch' ? Ry(a) : g.kind === 'yaw' ? Rz(a) : Rx(a);
      if (g) {
        const F = 1.6 * a;                                    // restoring force, drawn in the body frame
        if (g.kind === 'pitch') setArrow(fa.ele, [-0.8 * sc, 0, -0.03 * sc], [0, 0, -1], F);           // nose up → tail lifted → nose down
        else if (g.kind === 'yaw') setArrow(fa.rud, [-0.85 * sc, 0, -0.2 * sc], [0, 1, 0], F);         // nose right → fin pushed right → nose left
        else setArrow(fa.wing, [0, a > 0 ? yA : -yA, zA], [0, 0, -1], Math.abs(F) * 0.8);              // lower wing gets more lift
        readout.innerHTML = { pitch: 'the <b style="color:#5fd38d">horizontal tail</b> meets the air at a larger angle and lifts: the nose goes back down',
          yaw: 'the <b style="color:#5ab0ff">fin</b> acts like a weather vane: the side wind pushes the tail and the nose turns back into the wind',
          roll: 'with <b style="color:#f2b134">dihedral</b> the lower wing meets the air at a larger angle: more lift, the wings level (slowly)' }[g.kind];
      } else readout.innerHTML = 'press a gust button';
      if (!print) drawTrace();
    }
    st.propA += dt * (8 + 50 * thr);
    pose(defl, thr);
    setR(body, R);
    render();
  }

  if (print) {                                                  // a sensible static frame
    if (mode === 'fly') { st.cmd = { thr: 0.6, ail: 0.6, ele: 0.2, rud: 0 }; tx.set(st.cmd); for (let i = 0; i < 20; i++) frame(0, 0.03); }
    else if (mode === 'surfaces') { st.t = 0.8; frame(0, 0.001); }
    else { startGust('pitch'); for (let i = 0; i < 12; i++) frame(0, 0.025); drawTrace(); }
    return;
  }
  const loop = now => {
    if (!el.isConnected) return;
    const sec = el.closest('section');
    if (!sec || sec.classList.contains('present')) frame(now); else last = now;
    requestAnimationFrame(loop);
  };
  if (mode === 'stability') startGust('pitch');
  requestAnimationFrame(loop);
}
