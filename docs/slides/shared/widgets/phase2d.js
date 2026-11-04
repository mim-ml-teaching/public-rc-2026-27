// Phase space of a second-order system, q = (x, v): the vector field f(q) = (v, a(x, v)) and the trajectories that
// follow it. Click to choose the initial state. The exact trajectory (RK4 with a tiny step, faint) next to the one
// obtained with the chosen method and step size dt, drawn step by step: Euler with a large step spirals outwards
// on an undamped oscillator, where the exact motion is a closed orbit.
// Systems: "spring" (ẍ = −x − c ẋ), "pendulum" (θ̈ = −sin θ − c θ̇).
// config: { system: 'pendulum', c: 0 (damping), dt: 0.2, method: 'euler' | 'rk4', q0: [2, 0], steps: 200,
//           width: 460, height: 400, autoplay: false, controls: true, loop: false (start over after a short hold),
//           side: false (true: the readout and the controls in a column to the right of the canvas) }
// Optional: showExact: false, readout: false, showDirection: true; speed sets simulated seconds per real second.
// stepwise: true pauses each Euler update into state / direction / advance, with a magnified phase-plane view.
// accelerateSteps: true keeps the first four updates slow, then progressively shortens playback to 4 ms per update.
// range sets both axis limits; selectInitial: false locks the initial state; restartOnEntry resets playback.
// pendulum: true with side: true adds the physical pendulum synchronized with the phase-space state.
import { C, h, fmt, isPrint } from './util.js';

const SYSTEMS = {
  spring: { label: 'spring: ẍ = −x − c·ẋ', a: (x, v, c) => -x - c * v, R: 3.2, xl: 'x', vl: 'ẋ' },
  pendulum: { label: 'pendulum: θ̈ = −sin θ − c·θ̇', a: (x, v, c) => -Math.sin(x) - c * v, R: 4.5, xl: 'θ', vl: 'θ̇' },
};

export function mount(el, cfg) {
  const W = cfg.width || 460, H = cfg.height || 400;
  const showExact = cfg.showExact ?? true;
  const stepwise = cfg.stepwise ?? false, stepDuration = 2.4;
  const stepDurationAt = k => cfg.accelerateSteps ? Math.max(0.004, stepDuration * 0.85 ** Math.max(0, k - 3)) : stepDuration;
  let phase = 0;
  const stepCards = stepwise ? [...el.closest('section').querySelectorAll('[data-integration-step]')] : [];
  const st = { system: cfg.system || 'pendulum', c: cfg.c ?? 0, dt: cfg.dt ?? 0.2, method: cfg.method || 'euler', q0: (cfg.q0 || [2, 0]).slice(), steps: cfg.steps ?? 200, k: 0, playing: false };
  const sys = () => SYSTEMS[st.system];
  const f = (x, v) => [v, sys().a(x, v, st.c)];
  const stepEuler = (q, dt) => { const d = f(q[0], q[1]); return [q[0] + dt * d[0], q[1] + dt * d[1]]; };
  const stepRK4 = (q, dt) => {
    const k1 = f(q[0], q[1]), k2 = f(q[0] + dt / 2 * k1[0], q[1] + dt / 2 * k1[1]), k3 = f(q[0] + dt / 2 * k2[0], q[1] + dt / 2 * k2[1]), k4 = f(q[0] + dt * k3[0], q[1] + dt * k3[1]);
    return [q[0] + dt / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]), q[1] + dt / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])];
  };
  let path = [], exact = [];
  function compute() {
    path = [st.q0.slice()];
    for (let i = 0; i < st.steps; i++) { const q = (st.method === 'rk4' ? stepRK4 : stepEuler)(path[path.length - 1], st.dt); if (!isFinite(q[0]) || Math.hypot(q[0], q[1]) > 1e3) break; path.push(q); }
    exact = [];
    if (showExact) {
      exact = [st.q0.slice()];
      const T = st.steps * st.dt, n = Math.ceil(T / 0.01);
      for (let i = 0; i < n; i++) exact.push(stepRK4(exact[exact.length - 1], T / n));
    }
  }

  const canvas = h('canvas', { width: W, height: H, style: 'cursor:crosshair' }), g = canvas.getContext('2d');
  const pendulum = cfg.pendulum ? h('canvas', { width: 300, height: H }) : null;
  function drawPendulum([theta, omega]) {
    if (!pendulum) return;
    const p = pendulum.getContext('2d'), cx = 150, cy = H * 0.38, length = 100;
    const x = cx + length * Math.sin(theta), y = cy + length * Math.cos(theta);
    p.fillStyle = C.bg2; p.fillRect(0, 0, 300, H);
    p.font = '16px Inter, Arial, sans-serif'; p.textAlign = 'center'; p.fillStyle = C.fg;
    p.fillText('Pendulum', cx, 25);
    p.strokeStyle = C.dim; p.lineWidth = 1; p.setLineDash([4, 4]);
    p.beginPath(); p.moveTo(cx, cy); p.lineTo(cx, cy + length + 20); p.stroke(); p.setLineDash([]);
    p.strokeStyle = C.accent; p.lineWidth = 2;
    p.beginPath(); p.arc(cx, cy, 32, Math.PI / 2, Math.PI / 2 - theta, theta > 0); p.stroke();
    p.fillStyle = C.accent; p.fillText('θ', cx + 46 * Math.sin(theta / 2), cy + 46 * Math.cos(theta / 2));
    p.strokeStyle = C.fg; p.lineWidth = 3; p.beginPath(); p.moveTo(cx, cy); p.lineTo(x, y); p.stroke();
    p.fillStyle = C.dim; p.beginPath(); p.arc(cx, cy, 5, 0, 2 * Math.PI); p.fill();
    p.fillStyle = C.fg; p.beginPath(); p.arc(x, y, 10, 0, 2 * Math.PI); p.fill();
    const velocity = Math.max(-60, Math.min(60, omega * 25));
    if (Math.abs(velocity) > 2) {
      const dx = velocity * Math.cos(theta), dy = -velocity * Math.sin(theta), angle = Math.atan2(dy, dx);
      p.strokeStyle = C.blue; p.lineWidth = 2.5; p.beginPath(); p.moveTo(x, y); p.lineTo(x + dx, y + dy);
      for (const sign of [-1, 1]) { p.moveTo(x + dx, y + dy); p.lineTo(x + dx - 8 * Math.cos(angle + sign * 0.5), y + dy - 8 * Math.sin(angle + sign * 0.5)); }
      p.stroke();
    }
    p.fillStyle = C.accent; p.fillText(`θ = ${fmt(theta, 2)} rad`, cx, H - 65);
    p.fillStyle = C.blue; p.fillText(`ω = ${fmt(omega, 2)} rad/s`, cx, H - 40);
    p.fillStyle = C.dim; p.font = '13px Inter, Arial, sans-serif'; p.fillText('Same state q = (θ, ω)', cx, H - 15);
  }
  const readout = h('div', { class: 'readout', style: 'line-height:1.45' });
  const R = () => cfg.range ?? sys().R;
  const plotW = stepwise ? W / 2 : W;
  const sx = x => plotW / 2 + x / R() * (plotW / 2 - 10), sy = v => H / 2 - v / R() * (stepwise ? plotW / 2 - 10 : H / 2 - 10);
  function drawStepDetail() {
    const k = Math.min(st.k, path.length - 2), q = path[k], next = path[k + 1];
    const stage = st.k >= path.length - 1 ? 2 : phase < 0.3 ? 0 : phase < 0.6 ? 1 : 2;
    const fraction = st.k >= path.length - 1 ? 1 : Math.max(0, Math.min(1, (phase - 0.6) / 0.3));
    const left = W / 2 + 20, top = 65, width = W / 2 - 40, height = H - 110;
    // Follow the state at a fixed magnification, retaining a few completed steps.
    // Camera motion uses the same interpolation as the displayed update, so it
    // remains continuous when one step finishes and the next begins.
    const span = Math.max(0.01, Math.hypot(...f(...path[0])) * st.dt * 12);
    const cameraProgress = Math.max(0, k - 3 + fraction);
    const cameraIndex = Math.floor(cameraProgress), blend = cameraProgress - cameraIndex;
    const cameraA = path[cameraIndex], cameraB = path[Math.min(cameraIndex + 1, path.length - 1)];
    const cx = cameraA[0] + blend * (cameraB[0] - cameraA[0]), cv = cameraA[1] + blend * (cameraB[1] - cameraA[1]);
    const zoom = Math.min(width, height) / span;
    const zx = x => left + width / 2 + (x - cx) * zoom;
    const zy = v => top + height / 2 - (v - cv) * zoom;
    g.fillStyle = C.fg; g.font = '15px Inter, Arial, sans-serif'; g.textAlign = 'center';
    g.fillText('Phase plane', plotW / 2, 25);
    g.fillText('Single steps (magnified)', left + width / 2, 25);
    g.fillStyle = C.dim; g.font = '13px Inter, Arial, sans-serif';
    g.fillText(`Δt = ${fmt(st.dt, 3)}`, left + width / 2, 46);
    g.strokeStyle = C.line; g.lineWidth = 1; g.strokeRect(left, top, width, height);
    g.textAlign = 'left'; g.fillText('x', left + width - 12, top + height + 18);
    g.fillText('ẋ', left + 5, top + 16);
    g.save(); g.beginPath(); g.rect(left + 1, top + 1, width - 2, height - 2); g.clip();
    g.strokeStyle = C.line; g.lineWidth = 1;
    for (let i = 0; i < 5; i++) for (let j = 0; j < 5; j++) {
      const x = cx + (i - 2) * span / 5, v = cv + (j - 2) * span / 5, d = f(x, v), norm = Math.hypot(...d) || 1;
      const px = zx(x), py = zy(v), dx = 13 * d[0] / norm, dy = -13 * d[1] / norm;
      g.beginPath(); g.moveTo(px - dx / 2, py - dy / 2); g.lineTo(px + dx / 2, py + dy / 2); g.stroke();
    }
    g.strokeStyle = C.accent; g.lineWidth = 2; g.beginPath();
    const first = Math.max(0, k - 20);
    for (let i = first; i <= k; i++) { const p = path[i]; i > first ? g.lineTo(zx(p[0]), zy(p[1])) : g.moveTo(zx(p[0]), zy(p[1])); }
    g.stroke();
    for (let i = first; i <= k; i++) { const p = path[i]; g.fillStyle = C.accent; g.beginPath(); g.arc(zx(p[0]), zy(p[1]), 3, 0, 7); g.fill(); }
    const x = zx(q[0]), y = zy(q[1]), nx = zx(next[0]), ny = zy(next[1]);
    if (stage >= 1) {
      const dx = nx - x, dy = ny - y, a = Math.atan2(dy, dx), ax = x + dx * 1.8, ay = y + dy * 1.8;
      g.strokeStyle = C.blue; g.lineWidth = 2.5; g.beginPath(); g.moveTo(x, y); g.lineTo(ax, ay);
      for (const sign of [-1, 1]) { g.moveTo(ax, ay); g.lineTo(ax - 7 * Math.cos(a + sign * 0.5), ay - 7 * Math.sin(a + sign * 0.5)); }
      g.stroke();
    }
    if (stage === 2) {
      g.strokeStyle = C.accent; g.lineWidth = 3; g.beginPath(); g.moveTo(x, y); g.lineTo(x + fraction * (nx - x), y + fraction * (ny - y)); g.stroke();
      g.fillStyle = C.accent; g.beginPath(); g.arc(x + fraction * (nx - x), y + fraction * (ny - y), 4, 0, 7); g.fill();
    }
    g.fillStyle = C.blue; g.beginPath(); g.arc(x, y, 4, 0, 7); g.fill();
    g.restore();
    g.fillStyle = C.dim; g.textAlign = 'center'; g.font = '13px Inter, Arial, sans-serif';
    g.fillText(`Step ${k + 1} of ${path.length - 1}`, left + width / 2, H - 12);
    stepCards.forEach((card, i) => card.classList.toggle('active', i === stage));
    el.dataset.integrationStage = String(stage);
    el.dataset.integrationIndex = String(k);
  }
  function draw() {
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    // the field
    const m = 15, r = R();
    let vmax = 0; const arrows = [];
    for (let i = 0; i < m; i++) for (let j = 0; j < m; j++) { const x = -r + (i + 0.5) * 2 * r / m, v = -r + (j + 0.5) * 2 * r / m; const d = f(x, v); arrows.push([x, v, d]); vmax = Math.max(vmax, Math.hypot(d[0], d[1])); }
    for (const [x, v, d] of arrows) {
      const L = Math.hypot(d[0], d[1]) || 1, s = 0.42 * (2 * r / m) * Math.min(1, L / vmax * 2) / L;
      const x1 = sx(x), y1 = sy(v), x2 = sx(x + d[0] * s), y2 = sy(v + d[1] * s);
      g.strokeStyle = '#4a505c'; g.lineWidth = 1.2; g.beginPath(); g.moveTo(x1, y1); g.lineTo(x2, y2); g.stroke();
      g.fillStyle = '#4a505c'; g.beginPath(); g.arc(x2, y2, 1.8, 0, 7); g.fill();
    }
    // axes
    g.strokeStyle = C.dim; g.lineWidth = 1; g.beginPath(); g.moveTo(sx(-r), sy(0)); g.lineTo(sx(r), sy(0)); g.moveTo(sx(0), sy(-r)); g.lineTo(sx(0), sy(r)); g.stroke();
    g.fillStyle = pendulum ? C.accent : C.dim; g.font = '13px Inter, Arial, sans-serif'; g.textAlign = 'left'; g.fillText(sys().xl, sx(r) - 14, sy(0) - 6);
    g.fillStyle = pendulum ? C.blue : C.dim; g.fillText(pendulum ? 'ω' : sys().vl, sx(0) + 6, sy(r) + (pendulum ? 38 : 14));
    // exact
    if (showExact) { g.strokeStyle = C.green; g.globalAlpha = 0.7; g.lineWidth = 2; g.beginPath(); exact.forEach((q, i) => (i ? g.lineTo(sx(q[0]), sy(q[1])) : g.moveTo(sx(q[0]), sy(q[1])))); g.stroke(); g.globalAlpha = 1; }
    // the numerical path up to step k
    const shown = path.slice(0, st.k + 1);
    g.strokeStyle = C.accent; g.lineWidth = 2; g.beginPath(); shown.forEach((q, i) => (i ? g.lineTo(sx(q[0]), sy(q[1])) : g.moveTo(sx(q[0]), sy(q[1])))); g.stroke();
    if (st.dt >= 0.05) { g.fillStyle = C.accent; shown.forEach(q => { g.beginPath(); g.arc(sx(q[0]), sy(q[1]), 3, 0, 7); g.fill(); }); }
    const last = shown[shown.length - 1];
    if (pendulum) {
      drawPendulum(last);
      g.fillStyle = C.fg; g.font = '16px Inter, Arial, sans-serif'; g.textAlign = 'center'; g.fillText('Phase space', W / 2, 25);
      g.setLineDash([4, 4]); g.lineWidth = 1.5;
      g.strokeStyle = C.accent; g.beginPath(); g.moveTo(sx(last[0]), sy(last[1])); g.lineTo(sx(last[0]), sy(0)); g.stroke();
      g.strokeStyle = C.blue; g.beginPath(); g.moveTo(sx(last[0]), sy(last[1])); g.lineTo(sx(0), sy(last[1])); g.stroke(); g.setLineDash([]);
    }
    if (cfg.showDirection && (!stepwise || phase >= 0.3)) {
      const d = f(last[0], last[1]);
      const x1 = sx(last[0]), y1 = sy(last[1]), x2 = sx(last[0] + 0.3 * d[0]), y2 = sy(last[1] + 0.3 * d[1]);
      const a = Math.atan2(y2 - y1, x2 - x1);
      g.strokeStyle = C.blue; g.lineWidth = 2.5; g.beginPath(); g.moveTo(x1, y1); g.lineTo(x2, y2);
      for (const side of [-1, 1]) { g.moveTo(x2, y2); g.lineTo(x2 - 8 * Math.cos(a + side * 0.5), y2 - 8 * Math.sin(a + side * 0.5)); }
      if (Math.hypot(d[0], d[1]) > 1e-8) g.stroke();
    }
    g.fillStyle = C.fg; g.beginPath(); g.arc(sx(last[0]), sy(last[1]), 5, 0, 7); g.fill();
    g.fillStyle = C.green; g.beginPath(); g.arc(sx(st.q0[0]), sy(st.q0[1]), 4, 0, 7); g.fill();
    if (stepwise) drawStepDetail();
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H - 2);
    readout.innerHTML = `<div>${sys().label}${st.c ? `, c = ${fmt(st.c, 2)}` : ', c = 0 (no damping)'} · <span class="dim">click to choose the initial state</span></div>` +
      `<div><span style="color:${C.accent}">●</span> ${st.method === 'rk4' ? 'RK4' : 'explicit Euler'}, Δt = ${fmt(st.dt, st.dt < 0.01 ? 3 : 2)}: step ${String(st.k).padStart(3, ' ').replace(/ /g, '&nbsp;')} of ${path.length - 1}, t = ${fmt(st.k * st.dt, 1).padStart(5, ' ').replace(/ /g, '&nbsp;')}${showExact ? ` · <span style="color:${C.green}">—</span> exact` : ''}</div>`;
  }
  canvas.addEventListener('click', ev => { if (stepwise || cfg.selectInitial === false) return; const b = canvas.getBoundingClientRect(); const x = (ev.clientX - b.left) / b.width * W, y = (ev.clientY - b.top) / b.height * H; st.q0 = [(x - W / 2) / (W / 2 - 10) * R(), -(y - H / 2) / (H / 2 - 10) * R()]; st.k = 0; compute(); draw(); if (cfg.autoplay ?? true) play(); });

  let raf = 0, acc = 0, last = null;
  const shown = () => el.closest('section')?.classList.contains('present') !== false;
  function tick(now) {
    if (!st.playing || !canvas.isConnected) { raf = 0; return; }
    if (last != null && shown()) {
      acc += Math.min(0.1, (now - last) / 1000);
      let per = stepwise ? stepDurationAt(st.k) : cfg.speed > 0 ? st.dt / cfg.speed : Math.max(0.02, st.dt / 1.5);
      while (acc >= per && st.k < path.length - 1) {
        st.k++; acc -= per;
        if (stepwise) per = stepDurationAt(st.k);
      }
      phase = stepwise ? acc / per : 0;
      draw();
    }
    last = now;
    if (st.k >= path.length - 1) {
      st.playing = false; sync(); raf = 0;
      if (cfg.loop) setTimeout(() => { if (canvas.isConnected && shown()) { st.k = 0; acc = 0; phase = 0; draw(); play(); } }, 1500);   // hold, then start over
      return;
    }
    raf = requestAnimationFrame(tick);
  }
  function play() { if (st.k >= path.length - 1) { st.k = 0; acc = 0; phase = 0; } st.playing = true; last = null; sync(); if (!raf) raf = requestAnimationFrame(tick); }
  function pause() { st.playing = false; sync(); }

  const playBtn = h('button', { style: 'width:5.4em', onclick: () => (st.playing ? pause() : play()) }, '▶ play');
  const stepBtn = h('button', { onclick: () => {
    pause();
    if (stepwise) {
      if (phase < 0.3) phase = 0.3;
      else if (phase < 0.6) phase = 0.9;
      else { if (st.k < path.length - 1) st.k++; phase = 0; }
      acc = phase * stepDurationAt(st.k);
    } else if (st.k < path.length - 1) st.k++;
    draw();
  } }, stepwise ? 'next stage' : '1 step');
  const resetBtn = h('button', { onclick: () => { pause(); st.k = 0; acc = 0; phase = 0; draw(); } }, 'reset');
  const sysBtns = Object.keys(SYSTEMS).map(s => h('button', { onclick: () => { st.system = s; st.k = 0; compute(); sync(); draw(); } }, s));
  const methodBtn = h('button', { onclick: () => { st.method = st.method === 'euler' ? 'rk4' : 'euler'; st.k = 0; compute(); sync(); draw(); } }, '');
  const dtOut = h('span', { class: 'readout', style: 'display:inline-block; width:3em; text-align:right' }, fmt(st.dt, 2));
  const dtS = h('input', { type: 'range', min: 0.01, max: 1, step: 0.01, value: st.dt, style: 'width:7em' });
  dtS.addEventListener('input', () => { st.dt = parseFloat(dtS.value); dtOut.textContent = fmt(st.dt, 2); st.k = 0; compute(); draw(); });
  const cOut = h('span', { class: 'readout', style: 'display:inline-block; width:3em; text-align:right' }, fmt(st.c, 2));
  const cS = h('input', { type: 'range', min: 0, max: 1, step: 0.05, value: st.c, style: 'width:5em' });
  cS.addEventListener('input', () => { st.c = parseFloat(cS.value); cOut.textContent = fmt(st.c, 2); st.k = 0; compute(); draw(); });
  function sync() { playBtn.textContent = st.playing ? '⏸ pause' : '▶ play'; sysBtns.forEach(b => b.classList.toggle('active', b.textContent === st.system)); methodBtn.textContent = st.method === 'euler' ? 'explicit Euler' : 'RK4'; }
  el.classList.add('widget');
  if (cfg.side) {   // the readout and the controls in a column at the right of the canvas
    el.style.width = (W + 280 + (pendulum ? 320 : 0)) + 'px';
    const col = h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:260px' }, cfg.readout === false ? [] : [readout]);
    if (cfg.controls ?? true) col.append(h('div', { class: 'wctl interactive-only', style: 'gap:0.3em; margin:0' }, playBtn, stepBtn, resetBtn));
    if (cfg.controls !== 'playback' && (cfg.controls ?? true)) col.append(
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em; margin:0' }, ...sysBtns, methodBtn),
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em; margin:0' }, h('label', {}, 'Δt'), dtS, dtOut),
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em; margin:0' }, h('label', {}, 'damping c'), cS, cOut));
    el.append(h('div', { style: 'display:flex; gap:20px; align-items:flex-start' }, pendulum ? [pendulum] : [], canvas, col));
  } else {
    el.style.width = W + 'px';
    el.append(h('div', { style: `width:${W}px` }, canvas, cfg.readout === false ? [] : [readout]));
    if (cfg.controls === 'playback') el.append(h('div', { class: 'wctl interactive-only' }, playBtn, stepBtn, resetBtn));
    else if (cfg.controls ?? true) el.append(h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, playBtn, stepBtn, resetBtn, h('span', { style: 'margin-left:0.4em' }), ...sysBtns, methodBtn, h('label', { style: 'margin-left:0.4em' }, 'Δt'), dtS, dtOut, h('label', { style: 'margin-left:0.4em' }, 'damping c'), cS, cOut));
  }
  compute(); sync();
  if (isPrint()) { st.k = path.length - 1; draw(); return; }
  draw();
  if (stepwise || cfg.selectInitial === false) canvas.style.cursor = 'default';
  if (cfg.autoplay ?? true) { const deck = el.closest('.reveal'); const maybe = () => { if (!canvas.isConnected) { deck?.removeEventListener('slidechanged', maybe); return; } if (shown()) { if (stepwise || cfg.restartOnEntry) { st.k = 0; acc = 0; phase = 0; draw(); } play(); } else pause(); }; deck?.addEventListener('slidechanged', maybe); setTimeout(maybe, 300); }
}
