// A varied sequence of climbs, hovers and descents under prescribed thrust, with no feedback controller.
// Smooth transitions give an exact motion satisfying m z̈ = F(t) − mg − μ ż.
import { C, h, isPrint } from './util.js';

const M = 1, G = 9.81, MU = 0.3, INITIAL_Z = 1.1;
// [next altitude, travel time, hover time]; finish where we started for a seamless loop.
const MOVES = [[2.35, 2.1, 1.1], [1.5, 1.5, 0.4], [2.6, 1.9, 0.8],
  [2.1, 1.2, 0.3], [1.05, 2.3, 0.8], [1.65, 1.8, 0.4], [INITIAL_Z, 1.4, 0.5]];
const PERIOD = MOVES.reduce((sum, [, travel, hover]) => sum + travel + hover, 0);
function motion(time) {
  let local = time % PERIOD, from = INITIAL_Z;
  for (const [to, travel, hover] of MOVES) {
    if (local < travel) {
      const u = local / travel, distance = to - from;
      // Quintic interpolation: both velocity and acceleration vanish at each hover.
      return {
        z: from + distance * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5),
        v: distance / travel * 30 * u ** 2 * (1 - u) ** 2,
        acceleration: distance / travel ** 2 * 60 * u * (1 - u) * (1 - 2 * u),
      };
    }
    local -= travel;
    if (local < hover) return { z: to, v: 0, acceleration: 0 };
    local -= hover; from = to;
  }
  return { z: INITIAL_Z, v: 0, acceleration: 0 };
}

export function mount(el) {
  const canvas = h('canvas', { width: 300, height: 280 }), g = canvas.getContext('2d');
  let t = 0, last = null, playing = true;
  const shown = () => el.closest('section')?.classList.contains('present') !== false;
  function arrow(x, y, length, color, label) {
    const end = y - length, sign = Math.sign(length), head = Math.min(8, Math.abs(length) * 0.6);
    g.strokeStyle = color; g.fillStyle = color; g.lineWidth = 2.5;
    g.beginPath(); g.moveTo(x, y); g.lineTo(x, end);
    g.moveTo(x - head * 0.6, end + sign * head); g.lineTo(x, end); g.lineTo(x + head * 0.6, end + sign * head); g.stroke();
    g.textAlign = 'left'; g.fillText(label, x + 10, (y + end) / 2 + 5);
  }
  function draw() {
    const { z, v, acceleration } = motion(t);
    const force = M * G + MU * v + M * acceleration;
    const x = 150, y = 250 - 60 * z;
    g.fillStyle = C.bg2; g.fillRect(0, 0, 300, 280);
    g.font = '16px Inter, Arial, sans-serif';
    arrow(45, 250, 220, C.dim, '');
    g.fillStyle = C.dim; g.fillText('z', 55, 38);
    g.strokeStyle = C.line; g.lineWidth = 3; g.beginPath(); g.moveTo(15, 250); g.lineTo(280, 250); g.stroke();
    g.strokeStyle = C.green; g.lineWidth = 1.5; g.setLineDash([6, 4]);
    g.beginPath(); g.moveTo(45, 130); g.lineTo(270, 130); g.stroke(); g.setLineDash([]);
    g.fillStyle = C.green; g.fillText('H', 275, 135);
    arrow(x, y - 15, force * 5, C.blue, 'F(t)');
    arrow(x, y + 12, -M * G * 5, C.accent, 'mg');
    // Drag opposes the instantaneous motion and vanishes at a turning point.
    if (Math.abs(v) > 0.05) arrow(x - 65, y, -MU * v * 5, C.dim, 'μv');
    g.strokeStyle = C.fg; g.fillStyle = C.fg; g.lineWidth = 2.5;
    g.beginPath(); g.moveTo(x - 23, y); g.lineTo(x + 23, y);
    for (const side of [-1, 1]) {
      const rotor = x + side * 23;
      g.moveTo(rotor, y); g.lineTo(rotor, y - 8);
      g.moveTo(rotor - 12, y - 8); g.lineTo(rotor + 12, y - 8);
    }
    g.stroke(); g.beginPath(); g.arc(x, y, 4.5, 0, 2 * Math.PI); g.fill();
  }
  function tick(now) {
    if (!canvas.isConnected) return;
    if (shown() && playing && last !== null) { t += Math.min(0.1, (now - last) / 1000); draw(); }
    last = now; requestAnimationFrame(tick);
  }
  const play = h('button', { onclick: () => { playing = !playing; play.textContent = playing ? '⏸ pause' : '▶ play'; } }, '⏸ pause');
  const reset = h('button', { onclick: () => { t = 0; draw(); } }, 'reset');
  el.classList.add('widget'); el.style.width = '300px';
  el.append(canvas, h('div', { class: 'wctl interactive-only', style: 'margin:0; gap:0.3em' }, play, reset));
  draw();
  if (!isPrint()) requestAnimationFrame(tick);
}
