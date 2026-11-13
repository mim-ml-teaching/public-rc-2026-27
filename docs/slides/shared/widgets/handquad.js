// Fly a planar quadcopter by hand, motor by motor, with the keyboard (side view, left and right motor).
// Keys (only while a flight runs; Esc stops it and gives the keys back to the slides):
//   stabilizer OFF: W / S = left motor more / less, ↑ / ↓ = right motor more / less (held keys change the thrust at
//                   60 %/s, like two throttle levers).
//   stabilizer ON:  W / S = throttle, ← / → = tilt left / right (held); an attitude PD at 1 kHz sets the motor difference.
// Goal: stay inside the dashed box; timer and best time. Slow motion 1×, ½×, ¼× (keys keep acting in real time):
// with time slowed down a human can keep up — the problem is reaction time, not skill.
// Model: m = 1 kg, arm l = 0.2 m, J = 0.01 kg m² (as planarquad); each motor 0 … m g (thrust-to-weight 2:1); the right
// motor gives 2% less thrust than commanded; a slowly varying gust torque (OU process, σ 0.01 N m, τ 0.5 s).
// θ̈ = (l (f_left − f_right) + d) / J; ÿ = (f_l + f_r) sin θ / m; z̈ = (f_l + f_r) cos θ / m − g.
// config: { width: 640, height: 300 }
import { C, h, fmt, isPrint } from './util.js';
import { S, rng } from './plotsvg.js';

const G = 9.81, M = 1, L = 0.2, J = 0.01, FMAX = M * G, DT = 0.001, RATE = 0.6;
const BOX = { y0: -1.6, y1: 1.6, z0: 1.2, z1: 3.4 };

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 640, H = cfg.height || 300, PX = 58;
  const svg = S('svg', { width: W, height: H, viewBox: `0 0 ${W} ${H}`, style: 'display:block; flex:none; font-family:Inter, Arial, sans-serif' });
  const st = { s: [0.5, 0.5], thr: 0.5, tilt: 0, stab: false, run: false, speed: 1, keys: {}, best: 0 };
  let x, rand, crashed, t, inBox, trail;
  function reset() {
    x = { y: 0, z: 2.3, th: 0, vy: 0, vz: 0, w: 0, d: 0, fl: FMAX / 2, fr: FMAX / 2 };
    rand = rng(3 + Math.floor(Math.random() * 1000));          // plotsvg's rng gives N(0, 1) samples
    crashed = null; t = 0; inBox = 0; trail = []; st.s = [0.5, 0.5]; st.thr = 0.5; st.tilt = 0;
  }
  function inputs(dt) {
    const k = st.keys, d = RATE * dt;
    if (st.stab) {
      if (k.w) st.thr += d;
      if (k.s) st.thr -= d;
      st.thr = Math.max(0, Math.min(1, st.thr));
      st.tilt = (k.right ? 1 : 0) - (k.left ? 1 : 0);
    } else {
      if (k.w) st.s[0] += d;
      if (k.s) st.s[0] -= d;
      if (k.up) st.s[1] += d;
      if (k.down) st.s[1] -= d;
      st.s = st.s.map(v => Math.max(0, Math.min(1, v)));
    }
  }
  function step() {
    let fl, fr;
    if (st.stab) {
      const thr = st.thr * 2 * FMAX, cmd = st.tilt * 30 * Math.PI / 180;    // tilt right = right side down = θ > 0
      const tau = J * (60 * (cmd - x.th) - 12 * x.w);
      fl = thr / 2 + tau / (2 * L); fr = thr / 2 - tau / (2 * L);
    } else { fl = st.s[0] * FMAX; fr = st.s[1] * FMAX; }
    fl = Math.max(0, Math.min(FMAX, fl)); fr = Math.max(0, Math.min(FMAX, fr)) * 0.98;
    x.d += (-x.d / 0.5) * DT + 0.01 * Math.sqrt(2 / 0.5 * DT) * rand();
    const T = fl + fr;
    x.w += (L * (fl - fr) + x.d) / J * DT; x.th += x.w * DT;
    x.vy += T * Math.sin(x.th) / M * DT; x.vz += (T * Math.cos(x.th) / M - G) * DT;
    x.y += x.vy * DT; x.z += x.vz * DT; t += DT; x.fl = fl; x.fr = fr;
    if (x.y > BOX.y0 && x.y < BOX.y1 && x.z > BOX.z0 && x.z < BOX.z1) inBox += DT;
    const xmax = (W / 2 - 20) / PX;
    if (x.z <= 0.1 || Math.abs(x.th) > Math.PI / 2 || Math.abs(x.y) > xmax || x.z > (H - 30) / PX)
      crashed = Math.abs(x.th) > Math.PI / 2 ? 'flipped over' : x.z <= 0.1 ? 'hit the ground' : 'flew away';
  }

  // ---- keyboard: only while flying and while this slide is shown (captured before reveal.js sees the keys) ----
  const KEYMAP = { KeyW: 'w', KeyS: 's', ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right' };
  const visible = () => { const sec = el.closest('section'); return el.isConnected && (!sec || sec.classList.contains('present')); };
  const onKey = down => ev => {
    if (!st.run || !visible()) return;
    if (ev.code === 'Escape') {
      if (down) { st.run = false; startBtn.textContent = '▶ start'; }
      ev.stopImmediatePropagation(); ev.preventDefault(); return;
    }
    const k = KEYMAP[ev.code]; if (!k) return;
    st.keys[k] = down; ev.stopImmediatePropagation(); ev.preventDefault();
  };
  window.addEventListener('keydown', onKey(true), true);
  window.addEventListener('keyup', onKey(false), true);

  // ---- panel ----
  const bars = [0, 1].map(() => h('div', { style: `height:12px; background:${C.accent}; border-radius:3px; width:50%` }));
  const barLab = [h('div', { class: 'small', style: 'width:8.5em' }), h('div', { class: 'small', style: 'width:8.5em' })];
  const read = h('div', { class: 'readout', style: 'line-height:1.5; min-height:4.5em' });
  const startBtn = h('button', { onclick: () => { reset(); st.run = true; st.keys = {}; startBtn.textContent = '↻ restart'; startBtn.blur(); } }, '▶ start');
  const stabBtn = h('button', { onclick: () => { st.stab = !st.stab; upd(); stabBtn.blur(); } });
  const spd = [[1, '1×'], [0.5, '½×'], [0.25, '¼×']].map(([v, l]) => {
    const b = h('button', { onclick: () => { st.speed = v; spd.forEach(o => o.classList.toggle('active', o === b)); b.blur(); } }, l);
    if (v === 1) b.classList.add('active');
    return b;
  });
  const keysHelp = h('div', { class: 'small', style: 'line-height:1.35' });
  function upd() {
    stabBtn.textContent = `stabilizer: ${st.stab ? 'ON' : 'OFF'}`; stabBtn.classList.toggle('active', st.stab);
    keysHelp.innerHTML = st.stab
      ? '<b>W / S</b> throttle · <b>← / →</b> tilt<br>the controller handles the motors · <b>Esc</b> stop'
      : '<b>W / S</b> left motor · <b>↑ / ↓</b> right motor<br>held keys change the thrust · <b>Esc</b> stop';
  }
  upd();
  el.append(h('div', { style: 'display:flex; gap:1em; align-items:flex-start' }, svg,
    h('div', { style: 'width:300px; flex:none; display:flex; flex-direction:column; gap:0.45em; font-size:0.8em' },
      keysHelp,
      ...[0, 1].map(i => h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, barLab[i],
        h('div', { style: `width:130px; height:12px; background:${C.bg3}; border-radius:3px` }, bars[i]))),
      h('div', { class: 'wctl interactive-only', style: 'flex-wrap:wrap; gap:0.3em' }, startBtn, stabBtn),
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em; align-items:center' }, h('label', {}, 'time:'), ...spd),
      read)));

  function draw() {
    svg.replaceChildren();
    const gy = H - 18, X = y => W / 2 + y * PX, Z = z => gy - z * PX;
    svg.append(S('rect', { x: 0, y: 0, width: W, height: H, fill: C.bg2, rx: 8 }));
    svg.append(S('rect', { x: X(BOX.y0), y: Z(BOX.z1), width: (BOX.y1 - BOX.y0) * PX, height: (BOX.z1 - BOX.z0) * PX, fill: 'none', stroke: C.green, 'stroke-dasharray': '6 5', 'stroke-opacity': 0.7 }));
    svg.append(S('line', { x1: 0, x2: W, y1: gy, y2: gy, stroke: C.dim, 'stroke-width': 2 }));
    if (trail.length > 1) svg.append(S('path', { d: trail.map(([a, b], i) => `${i ? 'L' : 'M'}${X(a).toFixed(1)},${Z(b).toFixed(1)}`).join(''), fill: 'none', stroke: C.accent, 'stroke-opacity': 0.45, 'stroke-width': 2, 'stroke-dasharray': '4 4' }));
    const g = S('g', { transform: `translate(${X(x.y)},${Z(x.z)}) rotate(${(x.th * 180 / Math.PI).toFixed(2)})` });
    const a = L * PX * 1.6;
    g.append(S('rect', { x: -a, y: -4, width: 2 * a, height: 8, rx: 3, fill: C.dim }));
    g.append(S('rect', { x: -14, y: -9, width: 28, height: 18, rx: 4, fill: C.bg3, stroke: C.fg }));
    for (const [sx, f, key] of [[-1, x.fl, 'W/S'], [1, x.fr, '↑/↓']]) {
      g.append(S('rect', { x: sx * a - 22, y: -12, width: 44, height: 5, rx: 2, fill: C.blue, 'fill-opacity': 0.7 }));
      const len = 46 * f / FMAX;
      g.append(S('line', { x1: sx * a, x2: sx * a, y1: -14, y2: -14 - len, stroke: C.accent, 'stroke-width': 4 }));
      g.append(S('path', { d: `M${sx * a - 6},${-12 - len} L${sx * a},${-22 - len} L${sx * a + 6},${-12 - len}Z`, fill: C.accent }));
      if (!st.stab) g.append(S('text', { x: sx * a, y: 22, fill: C.dim, 'font-size': 12, 'text-anchor': 'middle' }, key));
    }
    svg.append(g);
    if (crashed) svg.append(S('text', { x: W / 2, y: 32, fill: C.red, 'font-size': 22, 'font-weight': 700, 'text-anchor': 'middle' }, `${crashed} after ${fmt(t, 1)} s`));
    else if (!st.run) svg.append(S('text', { x: W / 2, y: 32, fill: C.dim, 'font-size': 16, 'text-anchor': 'middle' }, 'press start, then keep it in the box'));
    const vals = st.stab ? [st.thr, 0.5 + st.tilt / 2] : st.s;
    barLab[0].textContent = st.stab ? `throttle ${Math.round(100 * st.thr)}%` : `left motor ${Math.round(100 * st.s[0])}%`;
    barLab[1].textContent = st.stab ? `tilt ${st.tilt > 0 ? 'right' : st.tilt < 0 ? 'left' : '0'}` : `right motor ${Math.round(100 * st.s[1])}%`;
    bars.forEach((b, i) => { b.style.width = `${Math.round(100 * vals[i])}%`; });
    read.innerHTML = `in the air ${fmt(t, 1)} s<br>in the box ${fmt(inBox, 1)} s (best ${fmt(st.best, 1)} s)<br>tilt θ ${fmt(x.th * 180 / Math.PI, 0)}°`;
  }

  reset();
  if (isPrint()) { st.run = true; for (let i = 0; i < 1300 && !crashed; i++) { step(); if (i % 20 === 0) trail.push([x.y, x.z]); } draw(); return; }
  let last = performance.now();
  const loop = now => {
    if (!el.isConnected) return;
    const vis = visible(), dtReal = Math.min(0.05, (now - last) / 1000);
    last = now;
    if (vis && st.run && !crashed) {
      inputs(dtReal);                                     // keys act in real time: slow motion gives the pilot more time
      const n = Math.round(dtReal * st.speed / DT);
      for (let i = 0; i < n && !crashed; i++) { step(); if (Math.round(t / DT) % 20 === 0) trail.push([x.y, x.z]); }
      st.best = Math.max(st.best, inBox);
      if (crashed) { st.run = false; startBtn.textContent = '▶ start'; }
    }
    if (vis) draw();
    requestAnimationFrame(loop);
  };
  draw();
  requestAnimationFrame(loop);
}
