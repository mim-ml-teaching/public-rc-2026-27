// The standard normal density with samples drawn from it one by one, stacking into a dot histogram under the curve:
// sampling from a simple distribution is one function call. Plays in a loop by itself (cfg.loop: false → on demand).
// config: { width: 328, height: 200, n: 300, controls: 'row' | 'overlay' (a pause button in the corner), t: 0, print_t: 1 }
import { C, h, isPrint } from './util.js';
import { rng } from './flowlib.js';
import { loopPlayer } from './loopplayer.js';

const XR = 3.4, BIN = 0.15, R = 2.6, STEP = 2 * R + 0.8;   // x range, bin width, dot radius, dot pitch

export function mount(el, cfg) {
  const W = cfg.width || 328, H = cfg.height || 200, N = cfg.n || 300, OVER = cfg.controls === 'overlay';
  const st = { t: cfg.t ?? 0 };
  const canvas = h('canvas', { width: W, height: H, style: `width:${W}px; height:${H}px` }), g = canvas.getContext('2d');
  const r = rng(5), xs = [];   // N(0, 1) samples by Box–Muller, in a fixed order
  while (xs.length < N) { const z = Math.sqrt(-2 * Math.log(Math.max(r(), 1e-12))) * Math.cos(2 * Math.PI * r()); if (Math.abs(z) < XR) xs.push(z); }
  const sx = x => (x + XR) / (2 * XR) * W, yA = H - 30, pdf = x => Math.exp(-x * x / 2) / Math.sqrt(2 * Math.PI);
  const SC = N * BIN * STEP;   // the curve's height matches the full dot histogram
  function draw() {
    const n = Math.round(st.t * N);
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    g.strokeStyle = '#5a606c'; g.lineWidth = 1; g.beginPath(); g.moveTo(0, yA); g.lineTo(W, yA); g.stroke();
    g.font = '12px Inter, Arial'; g.fillStyle = C.dim; g.textAlign = 'center';
    for (let v = -3; v <= 3; v++) { g.beginPath(); g.moveTo(sx(v), yA); g.lineTo(sx(v), yA + 4); g.stroke(); g.fillText(String(v), sx(v), yA + 17); }
    g.beginPath(); g.moveTo(sx(-XR), yA);
    for (let x = -XR; x <= XR; x += 0.02) g.lineTo(sx(x), yA - pdf(x) * SC);
    g.lineTo(sx(XR), yA); g.closePath(); g.fillStyle = 'rgba(90,176,255,0.12)'; g.fill(); g.strokeStyle = C.blue; g.lineWidth = 2; g.stroke();
    const counts = new Map(); let last = null;
    g.fillStyle = C.blue;
    for (let i = 0; i < n; i++) {
      const b = Math.floor((xs[i] + XR) / BIN), c = counts.get(b) || 0; counts.set(b, c + 1);
      last = [sx((b + 0.5) * BIN - XR), yA - R - 1 - c * STEP];
      g.beginPath(); g.arc(last[0], last[1], R, 0, 7); g.fill();
    }
    if (last) { g.fillStyle = C.accent; g.beginPath(); g.arc(last[0], last[1], R + 1, 0, 7); g.fill(); }   // the newest sample
    g.font = '13px Inter, Arial'; g.textAlign = 'left'; g.fillStyle = C.blue; g.fillText('x ~ N(0, 1)', 8, 17);
    g.fillStyle = C.dim; g.fillText(`${n} samples`, 8, 34);   // under the label: the top-right corner may hold the pause button
  }
  const loop = loopPlayer({ dur: 6000, hold: 2000, hold0: 400, alive: () => canvas.isConnected, step: t => { st.t = t; slider.value = t; draw(); } });
  const setBtn = () => { playBtn.textContent = loop.playing ? '⏸' : '▶'; };
  const play = () => { if (loop.playing) loop.stop(); else loop.start(st.t); setBtn(); };
  const reset = (autoplay = cfg.loop !== false) => { loop.stop(); st.t = cfg.t ?? 0; slider.value = st.t; if (autoplay && !isPrint()) loop.start(st.t); setBtn(); };
  const playBtn = h('button', { onclick: play, style: OVER ? 'min-width:2.4em; position:absolute; top:6px; right:6px; opacity:0.85' : 'min-width:2.4em' }, '▶');
  const slider = h('input', { type: 'range', min: 0, max: 1, step: 0.005, value: st.t, style: 'width:10em', oninput: e => { loop.stop(); setBtn(); st.t = +e.target.value; draw(); } });
  el.classList.add('widget');
  if (OVER) { el.style.position = 'relative'; el.append(canvas, h('span', { class: 'interactive-only' }, playBtn)); }
  else el.append(canvas, h('div', { class: 'wctl interactive-only', style: 'gap:0.5em; margin-top:0.3em' }, slider, playBtn));
  el.closest('.reveal')?.addEventListener('slidechanged', ev => { if (!canvas.isConnected) return; if (ev.currentSlide?.contains(el)) { reset(); draw(); } else if (loop.playing) { loop.stop(); setBtn(); } });
  if (isPrint()) { st.t = cfg.print_t ?? 1; slider.value = st.t; }
  draw();
  if (!isPrint() && el.closest('section')?.classList.contains('present')) reset();   // mounted on the current slide: start now
}
