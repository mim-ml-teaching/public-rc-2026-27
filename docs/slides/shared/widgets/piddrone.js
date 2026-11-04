// A drone holding an altitude with a P, PD or PID controller, simulated live. Left: the drone on its vertical axis
// with the thrust (blue) and the weight (orange); right: the altitude over time, with the target H.
// Physics: m z̈ = F − m g − μ ż (a little air resistance), the ground at z = 0. The controller:
//   e = H − z,  F = kp·e + kd·ė + ki·∫e dt  (no clamping: the formulas of the lecture, nothing else).
// Integration: explicit Euler with the step dt (or RK4 to compare): the same simulation serves Part 2, where a too
// large dt makes the P-controlled drone oscillate ever more instead of settling.
// config: { mode: 'P' | 'PD' | 'PID', kp: 12, kd: 4, ki: 4, H: 2, dt: 0.01, method: 'euler' | 'rk4', tmax: 20,
//           width: 760, height: 330, autoplay: true, controls: 'all' | 'gains' | 'dt' | 'none', speed: 1, loop: false,
//           labels: false }   // labels: true names H, e, F, the drone and z on the left, in the colours of the block diagram
// Optional z0 / v0 set the initial state; restartOnEntry resets a comparison on slide entry.
// controls: 'playback' shows just play/pause and reset.
// startOnFragment: a selector for a fade-out prediction question; revealing it starts the run.
// fixedMode keeps gain controls within the selected controller; previewTime shows a completed run without autoplay.
// showTerms adds a synchronized plot of the weighted P/I/D force contributions; terms optionally selects its curves.
import { C, h, fmt, isPrint } from './util.js';

const M = 1, G = 9.81, MU = 0.3, ZMAX = 4.2, CRASH_V = 1.5;  // a landing faster than CRASH_V m/s breaks the drone
const PRESETS = { P: { kp: 12, kd: 0, ki: 0 }, PD: { kp: 12, kd: 4, ki: 0 }, PID: { kp: 12, kd: 4, ki: 4 } };
// Match the lecture's --pid-p / --pid-i / --pid-d tokens. These are separate from H, z, e and F.
const TERM_STYLES = [
  { name: 'P', color: '#ff7868', dash: [], index: 2 },
  { name: 'I', color: '#70dce3', dash: [7, 4], index: 3 },
  { name: 'D', color: '#e8a2d8', dash: [2, 4], index: 4 },
];

export function mount(el, cfg) {
  const labels = !!cfg.labels;
  const W = cfg.width || 760, H_ = cfg.height || 330, LEFT = labels ? 230 : 150;
  const ZC = labels ? C.purple : C.blue, MGC = labels ? C.dim : C.accent;   // the altitude z and the weight mg
  const print = isPrint();
  const showTerms = !!cfg.showTerms;
  let termMin = -10, termMax = 30;
  const st = { mode: cfg.mode || 'PID', H: cfg.H ?? 2, dt: cfg.dt ?? 0.01, method: cfg.method || 'euler', tmax: cfg.tmax ?? 20, speed: cfg.speed ?? 1, playing: false };
  Object.assign(st, PRESETS[st.mode]);
  for (const k of ['kp', 'kd', 'ki']) if (cfg[k] !== undefined) st[k] = cfg[k];
  const sim = { t: 0, z: 0, v: 0, I: 0, hist: [], diverged: false, crashed: false, lifted: false };

  const contributions = (z, v, I) => [st.kp * (st.H - z), st.ki * I, -st.kd * v];
  const force = (z, v, I) => contributions(z, v, I).reduce((sum, term) => sum + term, 0);
  const accel = (z, v, I) => force(z, v, I) / M - G - MU * v / M;
  function sample() {
    const terms = contributions(sim.z, sim.v, sim.I);
    for (const value of terms) if (Number.isFinite(value)) {
      termMin = Math.min(termMin, Math.floor(value / 10) * 10);
      termMax = Math.max(termMax, Math.ceil(value / 10) * 10);
    }
    return [sim.t, Math.max(-1, Math.min(ZMAX + 3, sim.z)), ...terms];
  }
  function step() {
    const { z, v, I } = sim, dt = st.dt, e = st.H - z;
    let z1, v1, I1;
    if (st.method === 'rk4') {
      const f = (z, v, I) => [v, accel(z, v, I), st.H - z];
      const k1 = f(z, v, I), k2 = f(z + dt / 2 * k1[0], v + dt / 2 * k1[1], I + dt / 2 * k1[2]);
      const k3 = f(z + dt / 2 * k2[0], v + dt / 2 * k2[1], I + dt / 2 * k2[2]), k4 = f(z + dt * k3[0], v + dt * k3[1], I + dt * k3[2]);
      z1 = z + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]); v1 = v + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]); I1 = I + dt / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
    } else {  // explicit Euler: the new state from the derivatives at the old one
      z1 = z + dt * v; v1 = v + dt * accel(z, v, I); I1 = I + dt * e;
    }
    if (z1 <= 0 && v1 < 0) {  // the ground: a hard landing after lift-off is a crash, the flight ends
      if (sim.lifted && v1 < -CRASH_V) sim.crashed = true;
      z1 = 0; v1 = 0;
    }
    if (z1 > 0.5) sim.lifted = true;
    sim.z = z1; sim.v = v1; sim.I = I1; sim.t += dt;
    if (!isFinite(z1) || Math.abs(z1) > 1e4) sim.diverged = true;
    if (sim.hist.length === 0 || sim.t - sim.hist[sim.hist.length - 1][0] >= 0.02 - 1e-9) sim.hist.push(sample());
  }
  function reset() { playbackTime = 0; sim.t = 0; sim.z = cfg.z0 ?? 0; sim.v = cfg.v0 ?? 0; sim.I = 0; termMin = -10; termMax = 30; sim.hist = [sample()]; sim.diverged = false; sim.crashed = false; sim.lifted = sim.z > 0.5; }
  function runTo(t) { while (sim.t < t - 1e-9 && !sim.diverged && !sim.crashed && sim.t < st.tmax) step(); }

  // ---- drawing
  const canvas = h('canvas', { width: W, height: H_ }), g = canvas.getContext('2d');
  const readout = h('div', { class: 'readout', style: 'line-height:1.45' });
  const PT = showTerms ? 48 : 28, PB = 34, PL = LEFT + 46, PR = showTerms ? Math.round(W * 0.59) : W - 14;
  const Y = z => H_ - PB - (z / ZMAX) * (H_ - PT - PB);
  const X = t => PL + (t / st.tmax) * (PR - PL);
  function drawTerms() {
    const L = PR + 76, R = W - 14, bottom = H_ - PB;
    const x = t => L + t / st.tmax * (R - L);
    const y = value => bottom - (value - termMin) / (termMax - termMin) * (bottom - PT);
    const selected = cfg.terms ?? (st.mode || cfg.mode || 'PID');
    const visible = TERM_STYLES.filter(term => selected.includes(term.name));
    g.font = '14px Inter, Arial, sans-serif'; g.textAlign = 'center'; g.fillStyle = C.fg;
    g.fillText('Altitude', (PL + PR) / 2, 19);
    g.fillText('Controller terms (N)', (L + R) / 2, 19);
    visible.forEach((term, i) => {
      const lx = L + 14 + i * 105;
      g.strokeStyle = term.color; g.lineWidth = 2.5; g.setLineDash(term.dash);
      g.beginPath(); g.moveTo(lx, 36); g.lineTo(lx + 25, 36); g.stroke(); g.setLineDash([]);
      g.fillStyle = term.color; g.textAlign = 'left'; g.fillText(term.name, lx + 32, 41);
    });
    g.strokeStyle = C.line; g.lineWidth = 1; g.strokeRect(L, PT, R - L, bottom - PT);
    g.strokeStyle = C.dim; g.beginPath(); g.moveTo(L, y(0)); g.lineTo(R, y(0)); g.stroke();
    g.fillStyle = C.dim; g.font = '12px Inter, Arial, sans-serif'; g.textAlign = 'right';
    for (let i = 0; i <= 4; i++) {
      const value = termMin + (termMax - termMin) * i / 4;
      g.fillText(fmt(value, 1), L - 7, y(value) + 4);
    }
    g.textAlign = 'center';
    for (let t = 0; t <= st.tmax; t += 5) g.fillText(`${t}`, x(t), bottom + 14);
    g.fillText('time (s)', (L + R) / 2, H_ - 6);
    g.save(); g.beginPath(); g.rect(L, PT, R - L, bottom - PT); g.clip();
    for (const term of visible) {
      g.strokeStyle = term.color; g.lineWidth = 2.5; g.setLineDash(term.dash); g.beginPath();
      sim.hist.forEach((point, i) => {
        if (!Number.isFinite(point[term.index])) return;
        i ? g.lineTo(x(point[0]), y(point[term.index])) : g.moveTo(x(point[0]), y(point[term.index]));
      });
      g.stroke(); g.setLineDash([]);
    }
    g.restore();
  }
  function drone(x, y, F) {
    g.strokeStyle = C.fg; g.lineWidth = 2.5; g.lineCap = 'round';
    g.beginPath(); g.moveTo(x - 22, y); g.lineTo(x + 22, y); g.stroke();
    for (const dx of [-22, 22]) { g.beginPath(); g.moveTo(x + dx, y); g.lineTo(x + dx, y - 7); g.stroke(); g.beginPath(); g.moveTo(x + dx - 11, y - 8); g.lineTo(x + dx + 11, y - 8); g.stroke(); }
    g.fillStyle = C.fg; g.beginPath(); g.arc(x, y + 1, 4.5, 0, 7); g.fill();
    // forces: thrust up (blue), weight down (orange), scaled so that mg is 36 px
    const s = 36 / (M * G);
    const arrow = (y0, len, color) => { if (Math.abs(len) < 1) return; g.strokeStyle = color; g.fillStyle = color; g.lineWidth = 3; g.beginPath(); g.moveTo(x, y0); g.lineTo(x, y0 - len); g.stroke(); const d = Math.sign(len); g.beginPath(); g.moveTo(x, y0 - len - 7 * d); g.lineTo(x - 5, y0 - len); g.lineTo(x + 5, y0 - len); g.closePath(); g.fill(); };
    const up = Math.max(-60, Math.min(90, F * s));
    arrow(y - 10, up, C.blue);
    arrow(y + 12, -M * G * s, MGC);
    if (labels) {
      g.font = '13px Inter, Arial, sans-serif'; g.textAlign = 'left';
      if (Math.abs(up) >= 12) { g.fillStyle = C.blue; g.fillText('F', x + 9, y - 10 - up / 2 + 5); }
      g.fillStyle = MGC; g.fillText('mg', x + 9, y + 12 + 36 / 2 + 5);
    }
  }
  function draw() {
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H_);
    // left: the axis
    const ax = 70;
    g.strokeStyle = C.dim; g.lineWidth = 1.2; g.beginPath(); g.moveTo(ax, Y(0)); g.lineTo(ax, Y(ZMAX)); g.stroke();
    g.fillStyle = '#5a606c'; g.fillRect(16, Y(0), LEFT - 20, 3);  // the ground
    g.strokeStyle = C.green; g.setLineDash([6, 4]); g.lineWidth = 1.5; g.beginPath(); g.moveTo(14, Y(st.H)); g.lineTo(LEFT - 6, Y(st.H)); g.stroke(); g.setLineDash([]);
    g.font = '13px Inter, Arial, sans-serif'; g.fillStyle = C.green; g.textAlign = 'left'; g.fillText(`H = ${fmt(st.H, 1)} m`, 14, Y(st.H) - 6);
    g.fillStyle = C.dim; g.fillText('z', ax + 6, Y(ZMAX) + 12);
    for (let z = 0; z <= ZMAX; z += 1) { g.fillStyle = C.dim; g.textAlign = 'right'; g.fillText(`${z}`, ax - 6, Y(z) + 4); g.fillRect(ax - 3, Y(z), 6, 1); }
    const zd = Math.max(-0.3, Math.min(ZMAX + 0.2, sim.z));
    if (labels && !sim.crashed) {   // the altitude z as a dashed line like H, the error e = H − z as a double arrow between them
      const y0 = Y(zd), y1 = Y(st.H), bx = ax + 90;
      g.strokeStyle = ZC; g.setLineDash([6, 4]); g.lineWidth = 1.5; g.beginPath(); g.moveTo(14, y0); g.lineTo(LEFT - 6, y0); g.stroke(); g.setLineDash([]);
      g.font = '13px Inter, Arial, sans-serif'; g.fillStyle = ZC; g.textAlign = 'right'; g.fillText('z', LEFT - 8, y0 + (y0 > y1 ? 15 : -5));
      const L = Math.abs(y0 - y1), d = Math.sign(y1 - y0);
      if (L >= 6) {   // the arrow vanishes with the error
        g.strokeStyle = C.accent; g.fillStyle = C.accent; g.lineWidth = 2; g.beginPath(); g.moveTo(bx, y0); g.lineTo(bx, y1); g.stroke();
        if (L >= 16) for (const [y, dir] of [[y0, -d], [y1, d]]) { g.beginPath(); g.moveTo(bx, y); g.lineTo(bx - 4, y - dir * 7); g.lineTo(bx + 4, y - dir * 7); g.closePath(); g.fill(); }
        g.textAlign = 'left'; g.fillText('e', bx + 9, (y0 + y1) / 2 + 5);
      }
    }
    if (sim.crashed) { g.save(); g.translate(ax + 46, Y(0) - 6); g.rotate(-0.5); drone(0, 0, 0); g.restore(); }
    else drone(ax + 46, Y(zd), force(sim.z, sim.v, sim.I));
    // right: the plot
    g.strokeStyle = C.line; g.lineWidth = 1; g.strokeRect(PL, PT, PR - PL, H_ - PT - PB);
    g.strokeStyle = C.green; g.setLineDash([6, 4]); g.lineWidth = 1.5; g.beginPath(); g.moveTo(PL, Y(st.H)); g.lineTo(PR, Y(st.H)); g.stroke(); g.setLineDash([]);
    g.fillStyle = C.dim; g.textAlign = 'center'; g.font = '12px Inter, Arial, sans-serif';
    for (let t = 0; t <= st.tmax; t += 5) { g.fillText(`${t}`, X(t), H_ - PB + 14); }
    g.fillText('time (s)', (PL + PR) / 2, H_ - 6);
    g.textAlign = 'right'; for (let z = 0; z <= ZMAX; z += 1) g.fillText(`${z}`, PL - 5, Y(z) + 4);
    g.save(); g.translate(PL - 32, (PT + H_ - PB) / 2); g.rotate(-Math.PI / 2); g.textAlign = 'center'; g.fillText('altitude z (m)', 0, 0); g.restore();
    g.strokeStyle = ZC; g.lineWidth = 2; g.beginPath();
    sim.hist.forEach(([t, z], i) => { const x = X(t), y = Y(z); i ? g.lineTo(x, y) : g.moveTo(x, y); });
    g.stroke();
    if (sim.diverged) { g.fillStyle = C.red; g.font = 'bold 15px Inter, Arial, sans-serif'; g.textAlign = 'center'; g.fillText('diverged: the simulation blew up', (PL + PR) / 2, PT + 22); }
    if (sim.crashed) { g.fillStyle = C.red; g.font = 'bold 15px Inter, Arial, sans-serif'; g.textAlign = 'center'; g.fillText(`crashed into the floor at t = ${fmt(sim.t, 1)} s`, (PL + PR) / 2, PT + 22); }
    if (showTerms) drawTerms();
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H_ - 2);
    const F = force(sim.z, sim.v, sim.I);
    const pad = (v, d, n) => fmt(v, d).padStart(n, ' ').replace(/ /g, '&nbsp;');  // fixed width: the numbers do not jump
    readout.innerHTML = `<div><span style="color:${C.blue}">F = ${pad(F, 1, 6)} N</span> · <span style="color:${MGC}">mg = ${fmt(M * G, 1)} N</span> · <span style="color:${labels ? ZC : 'inherit'}">z = ${pad(sim.z, 2, 5)} m</span> · <span style="color:${labels ? C.accent : 'inherit'}">e = ${pad(st.H - sim.z, 2, 6)} m</span> · t = ${pad(sim.t, 1, 4)} s</div>` +
      (cfg.compactReadout ? '' : `<div class="dim">F = ${fmt(st.kp, 1)}·e${st.kd ? ` + ${fmt(st.kd, 1)}·ė` : ''}${st.ki ? ` + ${fmt(st.ki, 1)}·∫e` : ''} · ${st.method === 'rk4' ? 'RK4' : 'explicit Euler'}, Δt = ${st.dt}</div>`);
  }

  // ---- playback in real time
  let raf = 0, last = null, playbackTime = 0;
  const shown = () => el.closest('section')?.classList.contains('present') !== false;
  function tick(now) {
    if (!st.playing || !canvas.isConnected) { raf = 0; return; }
    // Accumulate wall time independently: rounding each frame up to a physics
    // step would make larger time steps play faster than smaller ones.
    if (last != null && shown()) { playbackTime += Math.min(0.1, (now - last) / 1000) * st.speed; runTo(playbackTime); draw(); }
    last = now;
    if (sim.t >= st.tmax || sim.diverged || sim.crashed) {
      st.playing = false; sync(); draw(); raf = 0;
      if (cfg.loop) setTimeout(() => { if (canvas.isConnected && shown()) play(); }, 1500);   // hold, then start over
      return;
    }
    raf = requestAnimationFrame(tick);
  }
  function play() { if (sim.t >= st.tmax || sim.diverged || sim.crashed) reset(); playbackTime = sim.t; st.playing = true; last = null; sync(); if (!raf) raf = requestAnimationFrame(tick); }
  function pause() { st.playing = false; sync(); }

  // ---- controls
  const controls = cfg.controls || 'all';
  const modeBtns = Object.keys(PRESETS).map(m => h('button', { onclick: () => { st.mode = m; Object.assign(st, PRESETS[m]); syncSliders(); reset(); play(); } }, m));
  const playBtn = h('button', { style: 'width:5.4em', onclick: () => (st.playing ? pause() : play()) }, '▶ play');
  const resetBtn = h('button', { onclick: () => { pause(); reset(); draw(); } }, 'reset');
  const slider = (label, key, min, max, stepv, fmtv) => {
    const out = h('span', { class: 'readout', style: 'display:inline-block; width:3.4em; text-align:right' }, fmtv(st[key]));
    const inp = h('input', { type: 'range', min, max, step: stepv, value: st[key], style: 'width:6em' });
    inp.addEventListener('input', () => { st[key] = parseFloat(inp.value); out.textContent = fmtv(st[key]); if (key !== 'dt' && !cfg.fixedMode) st.mode = ''; reset(); play(); });
    return { row: [h('label', { style: 'margin-left:0.5em' }, label), inp, out], inp, out };
  };
  const sl = { kp: slider('kp', 'kp', 0, 30, 0.5, v => fmt(v, 1)), kd: slider('kd', 'kd', 0, 10, 0.25, v => fmt(v, 2)), ki: slider('ki', 'ki', 0, 10, 0.25, v => fmt(v, 2)), dt: slider('Δt', 'dt', 0.005, 0.1, 0.005, v => fmt(v, 3)) };
  const methodBtn = h('button', { onclick: () => { st.method = st.method === 'euler' ? 'rk4' : 'euler'; sync(); reset(); play(); } }, 'Euler');
  function syncSliders() { for (const k of ['kp', 'kd', 'ki', 'dt']) { sl[k].inp.value = st[k]; sl[k].out.textContent = k === 'dt' ? fmt(st[k], 3) : fmt(st[k], k === 'kp' ? 1 : 2); } }
  function sync() {
    playBtn.textContent = st.playing ? '⏸ pause' : '▶ play';
    modeBtns.forEach(b => b.classList.toggle('active', b.textContent === st.mode));
    methodBtn.textContent = st.method === 'euler' ? 'explicit Euler' : 'RK4';
    sl.kd.inp.disabled = st.mode === 'P'; sl.ki.inp.disabled = st.mode === 'P' || st.mode === 'PD';
  }
  el.classList.add('widget');
  el.style.width = W + 'px';
  el.append(h('div', { style: `width:${W}px` }, canvas, cfg.readout === false ? [] : [readout]));
  if (controls !== 'none') {
    const row1 = [playBtn, resetBtn];
    if (controls === 'all' || controls === 'gains') {
      row1.push(h('span', { style: 'margin-left:0.5em' }), ...(cfg.fixedMode ? [] : modeBtns), ...sl.kp.row);
      if (!cfg.fixedMode || st.mode !== 'P') row1.push(...sl.kd.row);
      if (!cfg.fixedMode || st.mode === 'PID') row1.push(...sl.ki.row);
    }
    if (controls === 'all' || controls === 'dt') row1.push(h('span', { style: 'margin-left:0.5em' }), methodBtn, ...sl.dt.row);
    el.append(h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, ...row1));
  }
  reset(); sync(); syncSliders();
  if (print) { runTo(st.tmax); draw(); return; }
  if (cfg.previewTime) runTo(Math.min(cfg.previewTime, st.tmax));
  draw();
  if (cfg.startOnFragment) {
    const deck = el.closest('.reveal'), section = el.closest('section');
    const trigger = section.querySelector(cfg.startOnFragment);
    let runningStage = false;
    const events = ['slidechanged', 'fragmentshown', 'fragmenthidden'];
    const update = () => {
      if (!canvas.isConnected) { events.forEach(ev => deck?.removeEventListener(ev, update)); return; }
      const revealed = trigger?.classList.contains('visible') ?? false;
      el.classList.toggle('awaiting-prediction', !revealed);
      const active = shown() && revealed;
      if (active && !runningStage) { reset(); draw(); play(); }
      else if (!active) { pause(); if (!revealed) { reset(); draw(); } }
      runningStage = active;
    };
    events.forEach(ev => deck?.addEventListener(ev, update));
    update();
  } else if (cfg.autoplay ?? true) {
    // start when the slide is shown (and restart on every return to it)
    const deck = el.closest('.reveal');
    const maybe = () => { if (!canvas.isConnected) { deck?.removeEventListener('slidechanged', maybe); return; } if (shown()) { if (cfg.restartOnEntry) reset(); play(); } else pause(); };
    deck?.addEventListener('slidechanged', maybe);
    setTimeout(maybe, 300);
  }
}
