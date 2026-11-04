// Integration and playback are deliberately separate: only changing dt builds
// a new trajectory. Rendering selects stored states without advancing the ODE.
import { C, h, isPrint } from './util.js';

const DURATION = 12, DAMPING = 0.15, STEPS_PER_SECOND = 20;
export function integratePendulum(dt) {
  const path = [[2.4, 0]];
  for (let k = 0; k < Math.round(DURATION / dt); k++) {
    const [theta, omega] = path[k];
    path.push([theta + dt * omega, omega + dt * (-Math.sin(theta) - DAMPING * omega)]);
  }
  return path;
}

export function mount(el) {
  const W = 540, H = 350, R = 3.2;
  const phase = h('canvas', { width: W, height: H, role: 'img', 'aria-label': 'Precomputed pendulum trajectory in phase space' });
  const pendulum = h('canvas', { width: W, height: H, role: 'img', 'aria-label': 'Pendulum playing back the selected trajectory' });
  const background = h('canvas', { width: W, height: H });
  const g = phase.getContext('2d'), p = pendulum.getContext('2d'), bg = background.getContext('2d');
  const sx = theta => W / 2 + theta / R * (W / 2 - 45);
  const sy = omega => H / 2 - omega / R * (H / 2 - 40);
  let dt = 0.05, speed = 1, elapsed = 0, wallElapsed = 0, path = [], playing = true, last = null;
  const shown = () => el.closest('section')?.classList.contains('present') !== false;

  const selectStyle = 'font:inherit; color:var(--fg); background:var(--bg-3); border:1px solid var(--line); border-radius:6px; padding:0.15em 0.35em';
  const stepSelect = h('select', { 'aria-label': 'Physics time step', style: selectStyle },
    ...[0.01, 0.05, 0.1, 0.2].map(value => h('option', { value }, `${value} s`)));
  stepSelect.value = String(dt);
  const speedSelect = h('select', { 'aria-label': 'Playback speed', style: selectStyle },
    ...[0.25, 0.5, 1, 2, 4, 8].map(value => h('option', { value }, `${value}×`)));
  speedSelect.value = String(speed);
  const pathInfo = h('div', { class: 'readout', style: 'margin-top:0.5em' });
  const number = width => h('span', { style: `display:inline-block; width:${width}ch; text-align:right; font-variant-numeric:tabular-nums; font-family:monospace` });
  const simTime = number(5), stateNumber = number(4), stateTotal = number(4);
  const wallTime = number(6), fullRun = number(6);
  const playbackInfo = h('div', { class: 'readout', style: 'margin-top:0.5em; white-space:nowrap' },
    h('div', {}, 't = ', simTime, ' s · state ', stateNumber, ' / ', stateTotal),
    h('div', { style: 'margin-top:0.3em' }, 'Wall time: ', wallTime, ' s · full run: ', fullRun, ' s'));
  const play = h('button', { type: 'button', style: 'min-width:5.4em', onclick: () => {
    if (elapsed >= DURATION) elapsed = wallElapsed = 0;
    playing = !playing; last = null; sync(); draw();
  } }, '⏸ pause');
  const replay = h('button', { type: 'button', onclick: () => { elapsed = wallElapsed = 0; playing = true; last = null; sync(); draw(); } }, 'replay');
  function sync() {
    const label = playing ? '⏸ pause' : elapsed >= DURATION ? '▶ replay' : '▶ play';
    if (play.textContent !== label) play.textContent = label;
  }

  function compute() {
    path = integratePendulum(dt);
    // This background, including the entire trajectory, is rendered only once
    // per integration. Playback draws a marker over the same cached image.
    bg.fillStyle = C.bg2; bg.fillRect(0, 0, W, H);
    bg.strokeStyle = C.line; bg.lineWidth = 1;
    for (let i = -3; i <= 3; i++) for (let j = -3; j <= 3; j++) {
      const dx = j, dy = -Math.sin(i) - DAMPING * j;
      const norm = Math.hypot(dx, dy) || 1;
      const x = sx(i), y = sy(j), ux = dx / norm * 10, uy = -dy / norm * 10;
      bg.beginPath(); bg.moveTo(x - ux / 2, y - uy / 2); bg.lineTo(x + ux / 2, y + uy / 2); bg.stroke();
    }
    bg.strokeStyle = C.dim;
    bg.beginPath(); bg.moveTo(sx(-R), sy(0)); bg.lineTo(sx(R), sy(0)); bg.moveTo(sx(0), sy(-R)); bg.lineTo(sx(0), sy(R)); bg.stroke();
    bg.font = '15px Inter, Arial, sans-serif'; bg.textAlign = 'left';
    bg.fillStyle = C.accent; bg.fillText('θ', sx(R) - 12, sy(0) - 10);
    bg.fillStyle = C.blue; bg.fillText('ω', sx(0) + 9, sy(R) + 4);
    bg.font = '12px Inter, Arial, sans-serif'; bg.fillStyle = C.dim;
    for (const value of [-2, -1, 1, 2]) {
      bg.textAlign = 'center'; bg.fillText(String(value), sx(value), sy(0) + 18);
      bg.textAlign = 'right'; bg.fillText(String(value), sx(0) - 8, sy(value) + 4);
    }
    bg.strokeStyle = C.accent; bg.lineWidth = 2; bg.beginPath();
    path.forEach(([theta, omega], i) => i ? bg.lineTo(sx(theta), sy(omega)) : bg.moveTo(sx(theta), sy(omega)));
    bg.stroke();
    if (dt >= 0.1) for (const [theta, omega] of path) {
      bg.fillStyle = C.accent; bg.beginPath(); bg.arc(sx(theta), sy(omega), 2, 0, 2 * Math.PI); bg.fill();
    }
    bg.fillStyle = C.green; bg.beginPath(); bg.arc(sx(path[0][0]), sy(path[0][1]), 5, 0, 2 * Math.PI); bg.fill();
    pathInfo.textContent = `${path.length} stored states · ${DURATION} s · explicit Euler`;
  }

  function draw() {
    const index = Math.min(path.length - 1, Math.floor(elapsed / dt + 1e-9));
    const [theta, omega] = path[index];
    g.drawImage(background, 0, 0);
    g.fillStyle = C.fg; g.beginPath(); g.arc(sx(theta), sy(omega), 6, 0, 2 * Math.PI); g.fill();
    g.strokeStyle = C.bg2; g.lineWidth = 2; g.stroke();

    p.fillStyle = C.bg2; p.fillRect(0, 0, W, H);
    const cx = W / 2, cy = 145, length = 115;
    const x = cx + length * Math.sin(theta), y = cy + length * Math.cos(theta);
    p.strokeStyle = C.dim; p.lineWidth = 1; p.setLineDash([4, 4]);
    p.beginPath(); p.moveTo(cx, cy); p.lineTo(cx, cy + length + 20); p.stroke(); p.setLineDash([]);
    p.strokeStyle = C.accent; p.lineWidth = 2;
    p.beginPath(); p.arc(cx, cy, 32, Math.PI / 2, Math.PI / 2 - theta, theta > 0); p.stroke();
    p.fillStyle = C.accent; p.font = '16px Inter, Arial, sans-serif'; p.textAlign = 'center';
    p.fillText('θ', cx + 46 * Math.sin(theta / 2), cy + 46 * Math.cos(theta / 2));
    p.strokeStyle = C.fg; p.lineWidth = 3; p.beginPath(); p.moveTo(cx, cy); p.lineTo(x, y); p.stroke();
    p.fillStyle = C.dim; p.beginPath(); p.arc(cx, cy, 5, 0, 2 * Math.PI); p.fill();
    p.fillStyle = C.fg; p.beginPath(); p.arc(x, y, 10, 0, 2 * Math.PI); p.fill();
    const velocity = omega * 25;
    if (Math.abs(velocity) > 2) {
      const dx = velocity * Math.cos(theta), dy = -velocity * Math.sin(theta), angle = Math.atan2(dy, dx);
      p.strokeStyle = C.blue; p.lineWidth = 2.5; p.beginPath(); p.moveTo(x, y); p.lineTo(x + dx, y + dy);
      for (const sign of [-1, 1]) { p.moveTo(x + dx, y + dy); p.lineTo(x + dx - 8 * Math.cos(angle + sign * 0.5), y + dy - 8 * Math.sin(angle + sign * 0.5)); }
      p.stroke();
    }
    p.font = '16px monospace'; p.textAlign = 'left';
    p.fillStyle = C.accent; p.fillText(`θ = ${theta.toFixed(2).padStart(5)} rad`, cx - 85, H - 46);
    p.fillStyle = C.blue; p.fillText(`ω = ${omega.toFixed(2).padStart(5)} rad/s`, cx - 85, H - 21);
    simTime.textContent = (index * dt).toFixed(2);
    stateNumber.textContent = String(index + 1); stateTotal.textContent = String(path.length);
    wallTime.textContent = wallElapsed.toFixed(2);
    fullRun.textContent = ((path.length - 1) / (STEPS_PER_SECOND * speed)).toFixed(2);
  }

  stepSelect.addEventListener('change', () => { dt = Number(stepSelect.value); elapsed = wallElapsed = 0; last = null; compute(); sync(); draw(); });
  speedSelect.addEventListener('change', () => { speed = Number(speedSelect.value); last = null; draw(); });
  const panel = (title, canvas, controls, info) => h('div', {},
    h('h3', { style: 'font-size:24px; color:var(--fg); margin:0 0 12px' }, title), canvas, controls, info);
  el.classList.add('widget'); el.style.width = '1120px';
  el.append(h('div', { style: 'display:grid; grid-template-columns:540px 540px; gap:40px' },
    panel('1 · Precompute the trajectory', phase,
      h('div', { class: 'wctl interactive-only', style: 'min-height:38px' }, h('label', {}, 'Physics time step Δt ', stepSelect)), pathInfo),
    panel('2 · Play back the stored states', pendulum,
      h('div', { class: 'wctl interactive-only', style: 'min-height:38px' }, play, replay, h('label', { style: 'margin-left:8px' }, 'Playback speed ', speedSelect)), playbackInfo)));
  compute();
  if (isPrint()) { elapsed = 3; draw(); return; }
  sync(); draw();
  function tick(now) {
    if (!phase.isConnected) return;
    if (shown() && playing && last !== null) {
      // Fixed stored steps per wall second: a larger dt needs fewer steps to
      // cover the same simulated duration; faster playback skips ahead faster.
      const rate = STEPS_PER_SECOND * speed * dt;
      const advance = Math.min(Math.max(0, (now - last) / 1000), (DURATION - elapsed) / rate);
      wallElapsed += advance;
      elapsed = Math.min(DURATION, elapsed + advance * rate);
      if (DURATION - elapsed < 1e-9) elapsed = DURATION;
      draw();
      if (elapsed >= DURATION) { playing = false; sync(); }
    }
    last = now; requestAnimationFrame(tick);
  }
  const deck = el.closest('.reveal');
  const onSlide = () => {
    if (!phase.isConnected) { deck?.removeEventListener('slidechanged', onSlide); return; }
    last = null;
    if (shown()) { elapsed = wallElapsed = 0; playing = true; sync(); draw(); }
  };
  deck?.addEventListener('slidechanged', onSlide);
  requestAnimationFrame(tick);
}
