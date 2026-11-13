// Sensors arrive at different rates, and a filter predicts between them. Top: one lane per sensor over 8 s (IMU 1 kHz,
// drawn thinned; magnetometer 100 Hz; barometer 50 Hz; GPS at 1, 5 or 10 Hz, each fix arriving 100 ms after it was
// measured). Bottom: a 1D position filter (Kalman filter, state = position and velocity) that predicts with the
// accelerometer at 1 kHz and corrects with each GPS fix when it arrives (the fix is applied at its measurement time and
// the filter re-runs the last 100 ms of IMU data). Plotted: errors w.r.t. the truth. The band is ±σ of the filter;
// it grows while predicting and shrinks at every fix; during an outage it keeps growing. Red: IMU only (dead reckoning).
// Truth: a(t) = 1.5 sin(0.8 t) m/s². Accelerometer: bias 0.15 m/s² + white noise 0.5 m/s². GPS: white, σ 1 m.
// Fixed seed. config: { width: 1180, height: 420, rate: 5, outage: true }
import { C, h, fmt } from './util.js';
import { S, plot, rng } from './plotsvg.js';

const T = 8, DT = 0.001, N = Math.round(T / DT), LAT = 0.1, SG = 1.0, QA = 0.25;

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 1180, H = cfg.height || 420;
  const st = { rate: cfg.rate || 5, outage: cfg.outage ?? true };
  const svg = S('svg', { width: W, height: H, viewBox: `0 0 ${W} ${H}`, style: 'display:block; font-family:Inter, Arial, sans-serif' });
  const readout = h('div', { class: 'readout', style: 'min-height:1.5em' });
  const rb = [1, 5, 10].map(r => h('button', { onclick: () => { st.rate = r; draw(); } }, `${r} Hz`));
  const ob = h('button', { onclick: () => { st.outage = !st.outage; draw(); } }, 'GPS outage 3 to 5 s');
  el.append(svg, h('div', { style: 'display:flex; gap:1.5em; align-items:center; font-size:0.8em' },
    h('div', { class: 'wctl interactive-only', style: 'margin:0' }, h('label', {}, 'GPS rate'), ...rb, ob), readout));

  // truth and the accelerometer (fixed for all settings)
  const acc = new Float64Array(N), pT = new Float64Array(N + 1), vT = new Float64Array(N + 1);
  {
    const r = rng(21);
    for (let i = 0; i < N; i++) {
      const t = i * DT, a = 1.5 * Math.sin(0.8 * t);
      acc[i] = a + 0.15 + 0.5 * r();
      vT[i + 1] = vT[i] + a * DT; pT[i + 1] = pT[i] + vT[i] * DT + 0.5 * a * DT * DT;
    }
  }
  const gpsNoise = Array.from({ length: 200 }, (() => { const r = rng(33); return () => SG * r(); })());

  function run() {
    const step = Math.round(1000 / st.rate);           // GPS period in IMU steps
    const lat = Math.round(LAT / DT);
    const X = new Float64Array(2 * (N + 1)), P = new Float64Array(3 * (N + 1));   // per step: p, v; P11, P12, P22
    let p = 0, v = 0, a11 = 1, a12 = 0, a22 = 0.25;
    const save = i => { X[2 * i] = p; X[2 * i + 1] = v; P[3 * i] = a11; P[3 * i + 1] = a12; P[3 * i + 2] = a22; };
    const predict = i => {
      const a = acc[i];
      p += v * DT + 0.5 * a * DT * DT; v += a * DT;
      const n11 = a11 + 2 * DT * a12 + DT * DT * a22 + QA * DT ** 3 / 3, n12 = a12 + DT * a22 + QA * DT * DT / 2, n22 = a22 + QA * DT;
      a11 = n11; a12 = n12; a22 = n22;
    };
    const est = new Float64Array(N + 1), sig = new Float64Array(N + 1), fixes = [];
    save(0); est[0] = p; sig[0] = Math.sqrt(a11);
    let k = 0;
    for (let i = 0; i < N; i++) {
      predict(i); save(i + 1);
      const j = i + 1 - lat;                             // a fix measured at step j arrives now
      if (j > 0 && j % step === 0) {
        const tm = j * DT, out = st.outage && tm >= 3 && tm < 5;
        const z = pT[j] + gpsNoise[k++ % gpsNoise.length];
        fixes.push({ t: tm, e: z - pT[j], out });
        if (!out) {
          // rewind to the measurement time, update, re-run the IMU steps since then
          p = X[2 * j]; v = X[2 * j + 1]; a11 = P[3 * j]; a12 = P[3 * j + 1]; a22 = P[3 * j + 2];
          const s = a11 + SG * SG, k1 = a11 / s, k2 = a12 / s, y = z - p;
          p += k1 * y; v += k2 * y;
          const n11 = (1 - k1) * a11, n12 = (1 - k1) * a12, n22 = a22 - k2 * a12;
          a11 = n11; a12 = n12; a22 = n22;
          save(j);
          for (let m = j; m <= i; m++) { predict(m); save(m + 1); }
        }
      }
      est[i + 1] = p - pT[i + 1]; sig[i + 1] = Math.sqrt(a11);
    }
    // IMU only
    const imu = new Float64Array(N + 1);
    { let pp = 0, vv = 0; for (let i = 0; i < N; i++) { pp += vv * DT + 0.5 * acc[i] * DT * DT; vv += acc[i] * DT; imu[i + 1] = pp - pT[i + 1]; } }
    return { est, sig, imu, fixes };
  }

  function draw() {
    rb.forEach((b, i) => b.classList.toggle('active', [1, 5, 10][i] === st.rate));
    ob.classList.toggle('active', st.outage);
    svg.replaceChildren();
    const x0 = 185, w = W - x0 - 20, X = t => x0 + t / T * w;
    // ---- lanes
    const lanes = [['IMU 1000 Hz', C.accent, 1000, 20], ['magnetometer 100 Hz', C.purple, 100, 1], ['barometer 50 Hz', C.green, 50, 1], [`GPS ${st.rate} Hz`, C.blue, st.rate, 1]];
    lanes.forEach(([name, col, rate, thin], li) => {
      const y = 14 + li * 30;
      svg.append(S('text', { x: x0 - 10, y: y + 13, fill: col, 'font-size': 15, 'text-anchor': 'end' }, name));
      svg.append(S('line', { x1: x0, x2: x0 + w, y1: y + 18, y2: y + 18, stroke: C.line }));
      let d = '';
      const n = Math.round(T * rate);
      for (let k = 0; k <= n; k += thin) {
        const t = k / rate, out = li === 3 && st.outage && t >= 3 && t < 5;
        if (li === 3) {
          svg.append(S('line', { x1: X(t), x2: X(t), y1: y + 2, y2: y + 18, stroke: out ? C.red : col, 'stroke-width': 2.5, opacity: out ? 0.35 : 1 }));
          if (!out && t + LAT <= T) svg.append(S('path', { d: `M${X(t)},${y + 10} L${X(t + LAT)},${y + 10}`, stroke: col, 'stroke-width': 1.5, 'stroke-dasharray': '2 2' }));
        } else d += `M${X(t).toFixed(1)},${y + 6}V${y + 18}`;
      }
      if (d) svg.append(S('path', { d, stroke: col, 'stroke-width': 1, opacity: 0.8 }));
    });
    svg.append(S('text', { x: x0 + w, y: 10, fill: C.dim, 'font-size': 13, 'text-anchor': 'end' }, 'IMU: every 20th sample drawn · GPS: measured, then arrives 100 ms later (dotted)'));
    // ---- estimate error
    const R = run(), ts = Array.from({ length: N + 1 }, (_, i) => i * DT);
    const ph = H - 136 - 66, ymax = 4;
    const { Y } = plot(svg, { x0, y0: 140, w, h: ph, xmin: 0, xmax: T, ymin: -ymax, ymax, bare: false, xticks: [0, 2, 4, 6, 8], yticks: [-4, -2, 0, 2, 4],
      xlabel: 'time (s)', series: [] });
    svg.append(S('text', { x: x0 - 56, y: 140 + ph / 2, fill: C.dim, 'font-size': 14, 'text-anchor': 'middle', transform: `rotate(-90 ${x0 - 56} ${140 + ph / 2})` }, 'position error (m)'));
    if (st.outage) {
      svg.append(S('rect', { x: X(3), y: 140, width: X(5) - X(3), height: ph, fill: C.red, 'fill-opacity': 0.1 }));
      svg.append(S('text', { x: (X(3) + X(5)) / 2, y: 140 + ph - 10, fill: C.red, 'font-size': 15, 'text-anchor': 'middle' }, 'GPS outage'));
    }
    let band = '';
    for (let i = 0; i <= N; i += 5) band += (band ? 'L' : 'M') + X(ts[i]).toFixed(1) + ',' + Y(R.est[i] + R.sig[i]).toFixed(1);
    for (let i = N; i >= 0; i -= 5) band += 'L' + X(ts[i]).toFixed(1) + ',' + Y(R.est[i] - R.sig[i]).toFixed(1);
    svg.append(S('path', { d: band + 'Z', fill: C.accent, 'fill-opacity': 0.22, stroke: 'none' }));
    plot(svg, { x0, y0: 140, w, h: ph, xmin: 0, xmax: T, ymin: -ymax, ymax, bare: true, series: [
      { xs: ts, ys: R.imu, color: C.red, width: 2, dash: '6 4' },
      { xs: ts, ys: R.est, color: C.accent, width: 2.5 }] });
    for (const f of R.fixes) if (!f.out) svg.append(S('circle', { cx: X(f.t), cy: Y(f.e), r: 3.5, fill: C.blue }));
    // legend
    const lg = [['filter estimate ± σ', C.accent, 'none'], ['GPS fixes', C.blue, 'dot'], ['IMU only', C.red, '6 4']];
    lg.forEach(([t, c, dash], i) => {
      const lx = x0 + 12 + i * 210, ly = H - 12;
      if (dash === 'dot') svg.append(S('circle', { cx: lx + 12, cy: ly - 5, r: 4, fill: c }));
      else svg.append(S('line', { x1: lx, x2: lx + 26, y1: ly - 5, y2: ly - 5, stroke: c, 'stroke-width': 3, 'stroke-dasharray': dash }));
      svg.append(S('text', { x: lx + 34, y: ly, fill: C.fg, 'font-size': 15 }, t));
    });
    const iEnd = Math.round(5 / DT) - 1, iMid = Math.round(2.5 / DT);
    readout.innerHTML = `σ before the outage ${fmt(R.sig[iMid], 2)} m · ` + (st.outage ? `at its end <b style="color:${C.red}">${fmt(R.sig[iEnd], 2)} m</b> · ` : '') + `GPS alone σ ${SG} m`;
  }
  draw();
}
