// The simplest fusion: a complementary filter for one angle (pitch). 60 s at 100 Hz, fixed random seed.
// Truth: level, then rocking ±15° (period 6 s, three periods) from 5 s to 23 s, a 10° hold, level again; from 35 s to 45 s the
// vehicle accelerates forward at 2 m/s² while level (a car or a ground robot).
// Gyro: ω + bias 0.5°/s + white noise σ 0.5°/s. Accelerometer tilt: atan2 of the specific force = θ + atan(a/g) +
// noise σ 2° (so it is wrong by 11.5° while accelerating).
// Filter: θ̂ ← α (θ̂ + ω_m dt) + (1 − α) θ_acc, α = τ / (τ + dt): the gyro is trusted on time scales shorter than τ,
// the accelerometer on longer ones. RMS errors over the whole run are computed for the three estimates.
// config: { width: 900, height: 360, tau: 1 }
import { C, h, fmt } from './util.js';
import { S, plot, rng, legend } from './plotsvg.js';

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 900, H = cfg.height || 360, st = { tau: cfg.tau ?? 1 };
  const svg = S('svg', { width: W, height: H, viewBox: `0 0 ${W} ${H}`, style: 'display:block' });
  const readout = h('div', { class: 'readout', style: 'line-height:1.45; min-height:1.6em' });
  const val = h('span', { class: 'readout', style: 'display:inline-block; width:5em' });
  const toTau = v => Math.pow(10, v), fromTau = t => Math.log10(t);
  const sl = h('input', { type: 'range', min: -1.3, max: 1.5, step: 0.05, value: fromTau(st.tau), style: 'width:14em', oninput: e => { st.tau = toTau(+e.target.value); draw(); } });
  el.append(svg, readout, h('div', { class: 'wctl interactive-only' }, h('label', {}, 'time constant τ '), sl, val));

  const dt = 0.01, N = 6000, ts = Float64Array.from({ length: N + 1 }, (_, i) => i * dt);
  const truth = t => (t < 5 ? 0 : t < 23 ? 15 * Math.sin(2 * Math.PI * (t - 5) / 6) : t < 25 ? 10 * (t - 23) / 2 : t < 30 ? 10 : t < 32 ? 10 * (32 - t) / 2 : 0);
  const accel = t => (t >= 35 && t < 45 ? 2 : 0);
  const tru = new Float64Array(N + 1), gyr = new Float64Array(N + 1), acc = new Float64Array(N + 1);
  {
    const r = rng(11), q = rng(12);
    for (let i = 0; i <= N; i++) {
      const t = ts[i];
      tru[i] = truth(t);
      gyr[i] = (truth(t + dt / 2) - truth(t - dt / 2)) / dt + 0.5 + 0.5 * r();   // °/s
      acc[i] = tru[i] + Math.atan(accel(t) / 9.81) * 180 / Math.PI + 2 * q();
    }
  }
  const gyroOnly = new Float64Array(N + 1);
  for (let i = 1; i <= N; i++) gyroOnly[i] = gyroOnly[i - 1] + gyr[i - 1] * dt;
  const rms = e => Math.sqrt(e.reduce((s, v, i) => s + (v - tru[i]) ** 2, 0) / e.length);

  function draw() {
    svg.replaceChildren();
    const a = st.tau / (st.tau + dt), cf = new Float64Array(N + 1);
    cf[0] = acc[0];
    for (let i = 1; i <= N; i++) cf[i] = a * (cf[i - 1] + gyr[i - 1] * dt) + (1 - a) * acc[i];
    const x0 = 60, y0 = 24, w = W - 80, hh = H - 92;
    const { X, Y } = plot(svg, { x0, y0, w, h: hh, xmin: 0, xmax: 60, ymin: -25, ymax: 40, xticks: [0, 10, 20, 30, 40, 50, 60], yticks: [-20, 0, 20, 40],
      xlabel: 'time (s)', ylabel: 'pitch (°)', series: [] });
    svg.append(S('rect', { x: X(35), y: y0, width: X(45) - X(35), height: hh, fill: C.red, 'fill-opacity': 0.12 }));
    svg.append(S('text', { x: X(40), y: y0 + 18, fill: C.red, 'font-size': 14, 'text-anchor': 'middle', 'font-family': 'Inter, Arial, sans-serif' }, 'accelerating, level'));
    plot(svg, { x0, y0, w, h: hh, xmin: 0, xmax: 60, ymin: -25, ymax: 40, bare: true, series: [
      { xs: ts, ys: acc, color: C.blue, width: 1, opacity: 0.45 },
      { xs: ts, ys: gyroOnly, color: C.accent, width: 2 },
      { xs: ts, ys: tru, color: C.fg, width: 2, dash: '6 5' },
      { xs: ts, ys: cf, color: C.green, width: 2.8 }] });
    [['true', C.fg, '6 5'], ['gyro integrated', C.accent], ['accelerometer tilt', C.blue], ['complementary filter', C.green]]
      .forEach((it, i) => legend(svg, x0 + i * 200, H - 8, [it]));
    val.textContent = `${fmt(st.tau, st.tau < 1 ? 2 : 1)} s`;
    readout.innerHTML = `RMS error over 60 s: gyro only <b style="color:${C.accent}">${fmt(rms(gyroOnly), 1)}°</b> · accelerometer only <b style="color:${C.blue}">${fmt(rms(acc), 1)}°</b> · ` +
      `complementary filter <b style="color:${C.green}">${fmt(rms(cf), 1)}°</b>`;
  }
  draw();
}
