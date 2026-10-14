// Why 3-axis magnetometer fusion needs motion: a Kalman filter for the six magnetic states of EKF3,
// the Earth field B (NED) and the body bias d, with the measurement  m = R_iᵀ B + d + noise.
// The attitude R_i is taken as known here (EKF3 estimates it jointly). Each reading gives 3 numbers for 6 unknowns,
// so B and d separate only when R_i changes: hover → nothing separates; level turns → the horizontal parts do,
// but B_D and d_z always add up on the body z axis; manoeuvres (roll and pitch too) → everything.
// "WMM prior" starts B at the table value with σ = 50 mG (EK3_MAG_EF_LIM); without it σ_B = 200 mG.
// The plot shows the standard deviations σ (log scale) of the six states over a 60 s flight; below: the attitude.
//
// config: { width (760), motion ("turns"), wmm (false) }
import { C, h, fmt } from './util.js';
import { CITIES, fieldNED, euler, transpose, deg, rng } from './magutil.js';

const DT = 0.1, TEND = 60, RN = 50;              // s, s, measurement noise σ (mG, EK3_MAG_M_NSE)
const B_TRUE = fieldNED(CITIES.Warsaw), D_TRUE = [60, -40, 80];
const NAMES = ['B_N', 'B_E', 'B_D', 'd_x', 'd_y', 'd_z'];
const COLS = [C.red, C.green, C.blue, C.red, C.green, C.blue];

const MOTIONS = {
  hover: t => [2 * Math.sin(0.7 * t), 2 * Math.sin(0.5 * t + 1), 30 + 3 * Math.sin(0.2 * t)],
  turns: t => [0, 0, 12 * t],
  manoeuvres: t => [25 * Math.sin(0.5 * t), 20 * Math.sin(0.37 * t + 1), 12 * t],
};

function run(motion, wmm) {
  const r = rng(21);
  const sB = wmm ? 50 : 200, sD = 200;
  const x = [...B_TRUE.map((b, i) => b + (wmm ? [30, -20, 25][i] : [120, -90, 150][i])), 0, 0, 0];
  const P = [...Array(6)].map((_, i) => [...Array(6)].map((_, j) => (i === j ? (i < 3 ? sB * sB : sD * sD) : 0)));
  const hist = [], att = [];
  for (let k = 0; k <= TEND / DT; k++) {
    const t = k * DT, [ro, pi, ya] = MOTIONS[motion](t);
    const Rt = transpose(euler(ro * deg, pi * deg, ya * deg));
    // process noise: Earth field 1 mG/s, bias 0.1 mG/s (EK3_MAGE_P_NSE, EK3_MAGB_P_NSE)
    for (let i = 0; i < 6; i++) P[i][i] += (i < 3 ? 1 : 0.1) ** 2 * DT;
    for (let a = 0; a < 3; a++) {           // sequential scalar updates, as FuseMagnetometer
      const H = [Rt[a][0], Rt[a][1], Rt[a][2], a === 0 ? 1 : 0, a === 1 ? 1 : 0, a === 2 ? 1 : 0];
      const z = H[0] * B_TRUE[0] + H[1] * B_TRUE[1] + H[2] * B_TRUE[2] + D_TRUE[a] + RN * r.normal();
      const PH = P.map(row => row.reduce((s, v, j) => s + v * H[j], 0));
      const S = H.reduce((s, v, i) => s + v * PH[i], 0) + RN * RN;
      const K = PH.map(v => v / S), nu = z - H.reduce((s, v, i) => s + v * x[i], 0);
      for (let i = 0; i < 6; i++) x[i] += K[i] * nu;
      for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) P[i][j] -= K[i] * PH[j];
    }
    hist.push(P.map((row, i) => Math.sqrt(Math.max(row[i], 1e-9))));
    att.push([ro, pi, ((ya + 180) % 360 + 360) % 360 - 180]);
  }
  return { hist, att, x, sigma: hist.at(-1) };
}

export function mount(el, cfg) {
  const W = cfg.width || 760, H1 = 270, H2 = 100;
  const state = { motion: cfg.motion || 'turns', wmm: cfg.wmm ?? false };
  const canvas = h('canvas', { width: W, height: H1 + H2 });
  const g = canvas.getContext('2d');
  const table = h('div', {});
  const note = h('div', { style: 'line-height:1.3; min-height:3.6em' });

  function draw() {
    const res = run(state.motion, state.wmm);
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H1 + H2);
    const x0 = 60, x1 = W - 16, y0 = 30, y1 = H1 - 30;
    const X = t => x0 + (x1 - x0) * t / TEND;
    const lo = Math.log10(1), hi = Math.log10(300);
    const Y = s => y1 - (y1 - y0) * (Math.log10(Math.max(1, Math.min(300, s))) - lo) / (hi - lo);
    g.font = '14px Inter, Arial'; g.lineWidth = 1;
    for (const s of [1, 3, 10, 30, 100, 300]) {
      g.strokeStyle = C.line; g.beginPath(); g.moveTo(x0, Y(s)); g.lineTo(x1, Y(s)); g.stroke();
      g.fillStyle = C.dim; g.fillText(String(s), x0 - 34, Y(s) + 5);
    }
    for (let t = 0; t <= TEND; t += 10) { g.fillStyle = C.dim; g.fillText(`${t} s`, X(t) - 10, y1 + 18); }
    g.fillStyle = C.fg; g.font = '600 15px Inter, Arial'; g.fillText('σ of each state (mG, log scale)', x0, 20);
    for (let i = 0; i < 6; i++) {
      g.strokeStyle = COLS[i]; g.lineWidth = 2.5; g.setLineDash(i < 3 ? [] : [7, 5]);
      g.beginPath();
      res.hist.forEach((s, k) => { const u = X(k * DT), v = Y(s[i]); k ? g.lineTo(u, v) : g.moveTo(u, v); });
      g.stroke();
    }
    g.setLineDash([]);
    // legend
    g.font = '14px Inter, Arial';
    NAMES.forEach((n, i) => {
      const lx = x1 - 300 + (i % 3) * 100, yy = i < 3 ? 14 : 30;
      g.strokeStyle = COLS[i]; g.lineWidth = 2.5; g.setLineDash(i < 3 ? [] : [7, 5]);
      g.beginPath(); g.moveTo(lx, yy); g.lineTo(lx + 26, yy); g.stroke(); g.setLineDash([]);
      g.fillStyle = C.fg; g.fillText(n.replace('_', ''), lx + 30, yy + 5);
    });
    // attitude strip
    const a0 = H1 + 8, a1 = H1 + H2 - 18, AY = d => (a0 + a1) / 2 - (a1 - a0) / 2 * d / 180;
    g.strokeStyle = C.line; g.lineWidth = 1; g.beginPath(); g.moveTo(x0, AY(0)); g.lineTo(x1, AY(0)); g.stroke();
    g.fillStyle = C.dim; g.fillText('attitude', 6, AY(0) + 5);
    [[0, C.red, 'roll'], [1, C.green, 'pitch'], [2, C.purple, 'yaw']].forEach(([j, col, n], q) => {
      g.strokeStyle = col; g.lineWidth = 1.8; g.beginPath();
      res.att.forEach((a, k) => { const u = X(k * DT), v = AY(a[j]); k && Math.abs(a[j] - res.att[k - 1][j]) < 90 ? g.lineTo(u, v) : g.moveTo(u, v); });
      g.stroke();
      g.fillStyle = col; g.fillText(n, x0 + 6 + q * 60, a1 + 14);
    });

    const err = res.x.map((v, i) => v - (i < 3 ? B_TRUE[i] : D_TRUE[i - 3]));
    table.innerHTML = `<table style="font-size:1em; margin:0"><tr><th></th><th>σ (mG)</th><th>error</th></tr>` +
      NAMES.map((n, i) => `<tr><td style="color:${COLS[i]}">$${n}$</td><td class="readout">${fmt(res.sigma[i], 0)}</td><td class="readout">${fmt(err[i], 0)}</td></tr>`).join('') +
      `</table>`;
    window.renderMathInElement?.(table, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
    const msg = {
      hover: 'Constant attitude: only the sums $R^\\top B + d$ are measured — no state is learned.',
      turns: 'Level turns: the horizontal parts separate, but $B_D$ and $d_z$ always land on body $z$ together.',
      manoeuvres: 'Roll and pitch as well: all six states are learned.',
    }[state.motion];
    note.innerHTML = msg + (state.wmm ? ' <span class="dim">The WMM prior pins $B$ to about ±50 mG, so $d$ is known to about the same accuracy even without motion; motion does much better.</span>' : '');
    window.renderMathInElement?.(note, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
  }

  const mbtns = [['hover', 'hover'], ['turns', 'level turns'], ['manoeuvres', 'manoeuvres']].map(([m, txt]) => {
    const b = h('button', { onclick: () => { state.motion = m; mbtns.forEach(x => x.classList.toggle('active', x === b)); draw(); } }, txt);
    b.classList.toggle('active', m === state.motion);
    return b;
  });
  const wbtn = h('button', { onclick: () => { state.wmm = !state.wmm; wbtn.classList.toggle('active', state.wmm); draw(); } }, 'WMM prior on B');
  wbtn.classList.toggle('active', state.wmm);

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${W}px; flex:none` }, canvas,
      h('div', { class: 'wctl interactive-only' }, mbtns, wbtn)),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:13em; flex:none' }, table, note)));
  draw();
}
