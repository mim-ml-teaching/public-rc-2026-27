// A planar quadcopter (side view: horizontal y, height z, tilt θ, two motors) flown by the lecture's cascade:
//   position PD (50 Hz) → desired tilt and thrust → attitude P (400 Hz) → desired rate → rate P (1 kHz) → torque → mixer
// with limits: tilt ±35°, rate ±250°/s, each motor 0 to m g (thrust-to-weight 2:1). Click the scene to set a target.
// "controller sees": the true state, or an estimate: tilt delayed 20 ms + 1.5° bias + 0.5° noise, gyro + 3°/s noise,
// position and velocity delayed 100 ms + slowly wandering noise (σ 0.15 m, 0.1 m/s). Fixed seed.
// m = 1 kg, arm 0.2 m, J = 0.01 kg m². θ > 0 tilts the thrust to +y (clockwise on the screen); torque = l (f1 − f2),
// f1 = left motor.
// config: { width: 1180, height: 420, kpos: 2, katt: 8, est: false, start: [0, 1], target: [4.5, 3] }
import { C, h, fmt, isPrint } from './util.js';
import { S, rng } from './plotsvg.js';

const G = 9.81, M = 1, L = 0.2, J = 0.01, FMAX = M * G, KR = 40, DT = 0.001;
const TILT = 35 * Math.PI / 180, RATE = 250 * Math.PI / 180;

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 1180, H = cfg.height || 420, PW = 330, SW = W - PW - 20;
  const st = { kpos: cfg.kpos ?? 2, katt: cfg.katt ?? 8, est: cfg.est ?? false, target: (cfg.target || [4.5, 3]).slice() };
  const start = cfg.start || [0, 1];
  const svg = S('svg', { width: SW, height: H, viewBox: `0 0 ${SW} ${H}`, style: 'display:block; flex:none; cursor:crosshair; font-family:Inter, Arial, sans-serif' });
  const read = h('div', { class: 'readout', style: 'line-height:1.5; width:100%' });
  const sliders = [];
  const slider = (key, label, min, max, step) => {
    const v = h('span', { class: 'readout', style: 'display:inline-block; width:3em; text-align:right' });
    const s = h('input', { type: 'range', min, max, step, value: st[key], style: 'width:9em', oninput: e => { st[key] = +e.target.value; v.textContent = fmt(st[key], 1); } });
    v.textContent = fmt(st[key], 1);
    sliders.push(h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', { style: 'width:5.4em' }, label), s, v));
  };
  slider('kpos', 'position', 0.5, 12, 0.5);
  slider('katt', 'attitude', 2, 40, 1);
  const bTrue = h('button', { onclick: () => { st.est = false; mark(); } }, 'true state');
  const bEst = h('button', { onclick: () => { st.est = true; mark(); } }, 'noisy, delayed estimate');
  const mark = () => { bTrue.classList.toggle('active', !st.est); bEst.classList.toggle('active', st.est); };
  const bReset = h('button', { onclick: () => { reset(); } }, 'reset');
  el.append(h('div', { style: 'display:flex; gap:20px; align-items:flex-start' }, svg,
    h('div', { style: `width:${PW}px; flex:none; font-size:0.8em; display:flex; flex-direction:column; gap:0.4em` },
      h('div', { class: 'small' }, 'click the scene to set a target'), read,
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.3em' },
        h('div', { class: 'dim' }, 'gains'), ...sliders,
        h('div', { class: 'dim' }, 'the controller sees'), h('div', { style: 'display:flex; gap:0.3em; flex-wrap:wrap' }, bTrue, bEst), bReset))));
  mark();

  // ---- world ↔ screen
  const ymin = -1, ymax = 8, zmax = (ymax - ymin) * (H - 30) / SW;
  const sc = SW / (ymax - ymin), SX = y => (y - ymin) * sc, SY = z => H - 30 - z * sc;
  svg.addEventListener('click', e => {
    const r = svg.getBoundingClientRect(), px = (e.clientX - r.left) * SW / r.width, py = (e.clientY - r.top) * H / r.height;
    st.target = [Math.max(ymin + 0.3, Math.min(ymax - 0.3, px / sc + ymin)), Math.max(0.3, Math.min(zmax - 0.3, (H - 30 - py) / sc))];
  });

  // ---- simulation state
  let s, ctl, hist, trace, etrace, t, rn, pnY, pnZ, vnY, vnZ;
  function reset() {
    s = { y: start[0], z: start[1], vy: 0, vz: 0, th: 0, w: 0, f1: FMAX / 2, f2: FMAX / 2 };
    ctl = { thd: 0, F: M * G, wd: 0 };
    hist = []; trace = []; etrace = []; t = 0; rn = rng(5); pnY = 0; pnZ = 0; vnY = 0; vnZ = 0;
    st.target = (cfg.target || [4.5, 3]).slice();
  }
  const delayed = n => hist[Math.max(0, hist.length - 1 - n)] || s;
  function estimate() {
    if (!st.est) return { ...s };
    const a = 0.002;                       // wandering noise: first-order, correlation time 0.5 s
    pnY = (1 - a) * pnY + Math.sqrt(2 * a) * 0.15 * rn(); pnZ = (1 - a) * pnZ + Math.sqrt(2 * a) * 0.15 * rn();
    vnY = (1 - a) * vnY + Math.sqrt(2 * a) * 0.1 * rn(); vnZ = (1 - a) * vnZ + Math.sqrt(2 * a) * 0.1 * rn();
    const d20 = delayed(20), d100 = delayed(100);
    return { y: d100.y + pnY, z: d100.z + pnZ, vy: d100.vy + vnY, vz: d100.vz + vnZ,
      th: d20.th + 1.5 * Math.PI / 180 + 0.5 * Math.PI / 180 * rn(), w: s.w + 3 * Math.PI / 180 * rn() };
  }
  function step(k) {
    const e = estimate();
    if (k % 20 === 0) {                    // position loop, 50 Hz
      const kd = 1.4 * Math.sqrt(st.kpos);
      const ay = st.kpos * (st.target[0] - e.y) - kd * e.vy, az = st.kpos * (st.target[1] - e.z) - kd * e.vz;
      ctl.thd = Math.max(-TILT, Math.min(TILT, Math.atan2(ay, G + az)));
      ctl.F = Math.max(0.2 * M * G, Math.min(1.8 * M * G, M * (G + az) / Math.max(0.5, Math.cos(e.th))));
      if (k % 100 === 0) etrace.push([e.y, e.z]);
    }
    if (k % 3 === 0) ctl.wd = Math.max(-RATE, Math.min(RATE, st.katt * (ctl.thd - e.th)));   // attitude, ~400 Hz
    const tau = J * KR * (ctl.wd - e.w), d = tau / (2 * L);                                    // rate, 1 kHz
    s.f1 = Math.max(0, Math.min(FMAX, ctl.F / 2 + d)); s.f2 = Math.max(0, Math.min(FMAX, ctl.F / 2 - d));
    // dynamics
    const F = s.f1 + s.f2, ay = F * Math.sin(s.th) / M, az = F * Math.cos(s.th) / M - G, al = L * (s.f1 - s.f2) / J;
    s.vy += ay * DT; s.vz += az * DT; s.w += al * DT;
    s.y += s.vy * DT; s.z += s.vz * DT; s.th += s.w * DT;
    if (s.z < 0) { s.z = 0; s.vz = Math.max(0, s.vz); s.vy *= 0.9; }
    hist.push({ ...s }); if (hist.length > 200) hist.shift();
    if (k % 50 === 0) { trace.push([s.y, s.z]); if (trace.length > 400) trace.shift(); if (etrace.length > 400) etrace.shift(); }
  }
  let k = 0;
  const advance = sec => { for (let i = 0; i < Math.round(sec / DT); i++) step(k++); t += sec; };

  function draw() {
    svg.replaceChildren();
    svg.append(S('rect', { x: 0, y: 0, width: SW, height: H, fill: C.bg2, stroke: C.line }));
    for (let gy = Math.ceil(ymin); gy <= ymax; gy++) svg.append(S('line', { x1: SX(gy), x2: SX(gy), y1: 0, y2: H - 30, stroke: C.line, 'stroke-width': 0.6 }));
    for (let gz = 1; gz < zmax; gz++) svg.append(S('line', { x1: 0, x2: SW, y1: SY(gz), y2: SY(gz), stroke: C.line, 'stroke-width': 0.6 }));
    svg.append(S('rect', { x: 0, y: H - 30, width: SW, height: 30, fill: C.bg3 }));
    svg.append(S('text', { x: SW - 8, y: H - 10, fill: C.dim, 'font-size': 13, 'text-anchor': 'end' }, 'grid 1 m'));
    const [ty, tz] = st.target;
    svg.append(S('path', { d: `M${SX(ty) - 12},${SY(tz)} H${SX(ty) + 12} M${SX(ty)},${SY(tz) - 12} V${SY(tz) + 12}`, stroke: C.accent, 'stroke-width': 2.5 }));
    svg.append(S('circle', { cx: SX(ty), cy: SY(tz), r: 8, fill: 'none', stroke: C.accent, 'stroke-width': 2 }));
    if (st.est && etrace.length > 1) svg.append(S('polyline', { points: etrace.map(([y, z]) => `${SX(y).toFixed(1)},${SY(z).toFixed(1)}`).join(' '), fill: 'none', stroke: C.purple, 'stroke-width': 1.2, opacity: 0.7 }));
    if (trace.length > 1) svg.append(S('polyline', { points: trace.map(([y, z]) => `${SX(y).toFixed(1)},${SY(z).toFixed(1)}`).join(' '), fill: 'none', stroke: C.blue, 'stroke-width': 2 }));
    // the vehicle (drawn larger than its 0.4 m span)
    const g = S('g', { transform: `translate(${SX(s.y)} ${SY(s.z)}) rotate(${s.th * 180 / Math.PI})` });
    const hw = 0.36 * sc;
    g.append(S('line', { x1: -hw, x2: hw, y1: 0, y2: 0, stroke: C.fg, 'stroke-width': 5, 'stroke-linecap': 'round' }));
    g.append(S('rect', { x: -12, y: -8, width: 24, height: 14, rx: 3, fill: C.bg3, stroke: C.fg }));
    [[-hw, s.f1], [hw, s.f2]].forEach(([x, f]) => {
      const sat = f >= FMAX - 1e-6 || f <= 1e-6, len = f / FMAX * 0.9 * sc;
      g.append(S('rect', { x: x - 16, y: -10, width: 32, height: 5, fill: C.dim }));
      g.append(S('line', { x1: x, x2: x, y1: -12, y2: -12 - len, stroke: sat ? C.red : C.green, 'stroke-width': 4 }));
    });
    svg.append(g);
    const deg = s.th * 180 / Math.PI;
    read.innerHTML = `t ${fmt(t, 1)} s · tilt ${fmt(deg, 0)}°<br>motors ${fmt(s.f1 / FMAX, 2)}, ${fmt(s.f2 / FMAX, 2)} of max<br>` +
      `distance to target ${fmt(Math.hypot(ty - s.y, tz - s.z), 2)} m<br>` +
      `<span style="color:${C.blue}">true path</span>${st.est ? ` · <span style="color:${C.purple}">what it sees</span>` : ''}`;
  }

  reset();
  if (isPrint() || cfg.static) { advance(6); draw(); return; }
  draw();
  let last = performance.now();
  const loop = now => {
    if (!el.isConnected) return;
    const dt = Math.min(0.05, (now - last) / 1000); last = now;
    if (!document.hidden && el.offsetParent !== null) { advance(dt); draw(); }
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}
