// A pie chart that changes in time, with a legend: the share of the world population by level of education.
// The numbers are illustrative (a smooth trend from "no education" towards "post-secondary"), looping over the years.
// config: { width: 360, height: 300, print_t: 1, velocity: false (true: next to every legend entry a bar with the rate
//           of change of its share, d(share)/dt: right = growing, left = shrinking) }
import { C, h, isPrint } from './util.js';

const CATS = [['no education', '#ff6b6b'], ['primary', '#f2b134'], ['secondary', '#5ab0ff'], ['post-secondary', '#5fd38d']];
// illustrative trend: no education falls, secondary and post-secondary rise, primary rises and then falls
// (its rate of change changes sign exactly once); every state sums to 1
const A = [0.36, 0.30, 0.26, 0.08], B = [0.10, 0.22, 0.42, 0.26], BUMP = 0.14, SPLIT = [0.4, 0, 0.4, 0.2];
function sharesAt(u) {
  u = Math.max(0, Math.min(1, u)); const bump = BUMP * Math.sin(Math.PI * u);
  return A.map((a, i) => a + u * (B[i] - a) + (i === 1 ? bump : -bump * SPLIT[i]));
}
const Y0 = 1970, Y1 = 2020;

export function mount(el, cfg) {
  const W = cfg.width || 360, H = cfg.height || 300;
  const canvas = h('canvas', { width: W, height: H }), g = canvas.getContext('2d');
  const R = Math.min(H / 2 - 30, 130), cx = R + 24, cy = H / 2 + 8;
  function draw(t) {
    const sh = sharesAt(t), h_ = 1e-3, rate = sharesAt(t + h_).map((a, i) => (a - sharesAt(t - h_)[i]) / (2 * h_));   // d(share)/dt, per whole run
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    let a0 = -Math.PI / 2;
    sh.forEach((v, i) => {
      const a1 = a0 + v * 2 * Math.PI;
      g.fillStyle = CATS[i][1]; g.beginPath(); g.moveTo(cx, cy); g.arc(cx, cy, R, a0, a1); g.closePath(); g.fill();
      g.strokeStyle = C.bg2; g.lineWidth = 2; g.stroke();
      const am = (a0 + a1) / 2; if (v > 0.07) { g.fillStyle = '#15171c'; g.font = 'bold 13px Inter, Arial, sans-serif'; g.textAlign = 'center'; g.fillText(`${Math.round(v * 100)}%`, cx + 0.64 * R * Math.cos(am), cy + 0.64 * R * Math.sin(am) + 4); }
      a0 = a1;
    });
    // the legend
    g.textAlign = 'left'; g.font = '14px Inter, Arial, sans-serif';
    CATS.forEach(([name, col], i) => {
      const y = cy - 52 + i * 34; g.fillStyle = col; g.fillRect(cx + R + 24, y - 11, 14, 14); g.fillStyle = C.fg; g.fillText(name, cx + R + 46, y + 1);
      if (cfg.velocity) {   // the velocity of the share: a bar around a zero line
        const x0 = cx + R + 270, v = rate[i] * 260;
        g.strokeStyle = C.dim; g.lineWidth = 1; g.beginPath(); g.moveTo(x0, y - 12); g.lineTo(x0, y + 5); g.stroke();
        g.fillStyle = v >= 0 ? C.green : C.red; g.fillRect(Math.min(x0, x0 + v), y - 8, Math.abs(v), 9);
      }
    });
    if (cfg.velocity) { g.fillStyle = C.dim; g.font = '12px Inter, Arial, sans-serif'; g.textAlign = 'center'; g.fillText('rate of change of the share', cx + R + 270, cy - 76); }
    g.textAlign = 'left'; g.fillStyle = C.dim; g.font = '13px Inter, Arial, sans-serif'; g.fillText(`year ${Math.round(Y0 + t * (Y1 - Y0))} (illustrative)`, 14, 20);
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, W - 2, H - 2);
  }
  el.classList.add('widget'); el.style.width = W + 'px';
  el.append(canvas);
  if (isPrint()) { draw(cfg.print_t ?? 1); return; }
  const t0 = performance.now(), DUR = 18000, HOLD = 2000;
  const shown = () => el.closest('section')?.classList.contains('present') !== false;
  const tick = now => { if (!el.isConnected) return; if (shown()) { const p = (now - t0) % (DUR + 2 * HOLD); draw(p < HOLD ? 0 : p < HOLD + DUR ? (p - HOLD) / DUR : 1); } requestAnimationFrame(tick); };
  requestAnimationFrame(tick);
}
