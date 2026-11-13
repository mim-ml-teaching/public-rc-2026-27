// Integrating IMU readings: the errors grow without bound. Simulated at 100 Hz for 60 s, fixed random seed.
// mode "gyro": the vehicle rocks, true angle 20° sin(2π t / 10 s). Gyro reading ω_m = ω + b + n, n white with std σ
//   (per 10 ms sample). The integrated angle drifts by b·t from the bias plus a random walk from the noise
//   (std σ·dt·√N). Faint lines: other noise draws.
// mode "accel": the vehicle stands still. Position by integrating the horizontal accelerometer twice. Two error
//   sources: a bias b (error ½ b t²) and an attitude error ε, which leaks gravity into the horizontal axis
//   (g sin ε, error ½ g sin ε t²). Noise σ = 0.02 m/s² is added to both.
// config: { mode: "gyro" | "accel", width: 620, height: 330, bias, noise, tilt }
import { C, h, fmt } from './util.js';
import { S, plot, rng, legend } from './plotsvg.js';

export function mount(el, cfg) {
  el.classList.add('widget');
  const mode = cfg.mode || 'gyro', W = cfg.width || 620, H = cfg.height || 330;
  const st = mode === 'gyro' ? { bias: cfg.bias ?? 0.1, noise: cfg.noise ?? 0.3 } : { bias: cfg.bias ?? 0.01, tilt: cfg.tilt ?? 1 };
  const svg = S('svg', { width: W, height: H, viewBox: `0 0 ${W} ${H}`, style: 'display:block' });
  const readout = h('div', { class: 'readout', style: 'line-height:1.45; min-height:3em' });
  const ctl = h('div', { class: 'wctl interactive-only' });
  el.append(svg, readout, ctl);
  const dt = 0.01, N = 6000, ts = Float64Array.from({ length: N + 1 }, (_, i) => i * dt);

  const slider = (key, label, min, max, step, unit) => {
    const v = h('span', { class: 'readout', style: 'display:inline-block; width:5.5em' });
    const s = h('input', { type: 'range', min, max, step, value: st[key], style: 'width:9em', oninput: e => { st[key] = +e.target.value; draw(); } });
    ctl.append(h('label', { style: 'margin-left:0.6em' }, label), s, v);
    return () => { v.textContent = `${st[key]} ${unit}`; };
  };
  const upd = mode === 'gyro'
    ? [slider('bias', 'bias b', 0, 0.5, 0.01, '°/s'), slider('noise', 'noise σ', 0, 2, 0.05, '°/s')]
    : [slider('bias', 'bias b', 0, 0.05, 0.001, 'm/s²'), slider('tilt', 'attitude error ε', 0, 3, 0.1, '°')];

  function gyroRun(seed) {
    const r = rng(seed), est = new Float64Array(N + 1), tru = new Float64Array(N + 1);
    for (let i = 1; i <= N; i++) {
      const t = ts[i];
      tru[i] = 20 * Math.sin(2 * Math.PI * t / 10);
      const w = 20 * 2 * Math.PI / 10 * Math.cos(2 * Math.PI * (t - dt / 2) / 10);   // midpoint rate: exact without errors
      est[i] = est[i - 1] + (w + st.bias + st.noise * r()) * dt;
    }
    return { tru, est };
  }
  function accRun(seed, b, eps) {
    const r = rng(seed), p = new Float64Array(N + 1);
    let v = 0;
    const a0 = b + 9.81 * Math.sin(eps * Math.PI / 180);
    for (let i = 1; i <= N; i++) { const a = a0 + 0.02 * r(); v += a * dt; p[i] = p[i - 1] + v * dt; }
    return p;
  }

  function draw() {
    svg.replaceChildren();
    upd.forEach(f => f());
    if (mode === 'gyro') {
      const runs = [1, 2, 3, 4, 5].map(gyroRun), main = runs[0];
      const err = main.est[N] - main.tru[N];
      const ymax = Math.max(40, Math.ceil((20 + Math.abs(st.bias) * 60 + 3 * st.noise * 0.01 * Math.sqrt(N)) / 10) * 10);
      plot(svg, { x0: 60, y0: 30, w: W - 80, h: H - 80, xmin: 0, xmax: 60, ymin: -ymax, ymax, xticks: [0, 10, 20, 30, 40, 50, 60], yticks: [-ymax, -20, 0, 20, ymax],
        xlabel: 'time (s)', ylabel: 'angle (°)',
        series: [...runs.slice(1).map(r => ({ xs: ts, ys: r.est, color: C.accent, width: 1.2, opacity: 0.3 })),
          { xs: ts, ys: main.tru, color: C.fg, width: 2, dash: '6 5' }, { xs: ts, ys: main.est, color: C.accent, width: 2.5 }] });
      legend(svg, 80, 52, [['true angle', C.fg, '6 5'], ['integrated gyro', C.accent]]);
      readout.innerHTML = `error after 60 s: <b style="color:${C.accent}">${fmt(err, 1)}°</b> · from the bias alone: b·t = ${fmt(st.bias * 60, 1)}° · ` +
        `noise random walk (1σ): σ·dt·√N = ${fmt(st.noise * dt * Math.sqrt(N), 2)}°`;
    } else {
      const pb = accRun(7, st.bias, 0), pt = accRun(8, 0, st.tilt), pa = accRun(9, st.bias, st.tilt);
      const top = Math.max(pa[N], pb[N], pt[N], 1), ymax = top > 100 ? Math.ceil(top / 100) * 100 : Math.ceil(top / 10) * 10;
      const yt = [0, ymax / 2, ymax];
      plot(svg, { x0: 70, y0: 30, w: W - 90, h: H - 80, xmin: 0, xmax: 60, ymin: 0, ymax, xticks: [0, 10, 20, 30, 40, 50, 60], yticks: yt,
        xlabel: 'time (s)', ylabel: 'position error (m), vehicle standing still',
        series: [{ xs: ts, ys: pb, color: C.blue, width: 2 }, { xs: ts, ys: pt, color: C.red, width: 2 }, { xs: ts, ys: pa, color: C.accent, width: 3 }] });
      legend(svg, 90, 52, [[`bias ${st.bias} m/s² only`, C.blue], [`attitude error ${st.tilt}° only`, C.red], ['both', C.accent]]);
      readout.innerHTML = `after 60 s: bias ½ b t² = <b style="color:${C.blue}">${fmt(0.5 * st.bias * 3600, 1)} m</b> · ` +
        `attitude error ½ g sin ε · t² = <b style="color:${C.red}">${fmt(0.5 * 9.81 * Math.sin(st.tilt * Math.PI / 180) * 3600, 0)} m</b> · simulated (both + noise): ${fmt(pa[N], 0)} m`;
    }
  }
  draw();
}
