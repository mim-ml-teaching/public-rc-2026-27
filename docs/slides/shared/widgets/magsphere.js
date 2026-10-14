// Magnetometer calibration in 3D. The vehicle is rotated in the Earth field; each reading m_i (sensor frame) is
//   m_i = W b_i + h + noise,   b_i = R_iᵀ e   (e = Earth field in NED, R_i = attitude body → NED)
// with hard iron h (a shift) and soft iron / gain W (symmetric, a stretch). Ideal readings lie on a sphere of radius
// |e| = F around 0 (dim wireframe); real ones on a shifted, stretched ellipsoid. "fit" runs a least-squares ellipsoid
// fit (magutil.fitEllipsoid) and draws it (purple); "correct" moves every sample to s·M(m + o), s = F / r (green),
// i.e. what ArduPilot's COMPASS_OFS, DIA/ODI and SCALE do. "collect" replays the sampling, one reading at a time.
// motion "yaw" (flat spin) shows why the vehicle must be turned in every direction: the fit becomes ill-conditioned.
//
// config: { width, height, dist (camera distance factor, 1), city ("Warsaw"), hard (mG, 0), soft (0–0.4, 0), noise (mG, 3), motion ("all"|"tilts"|"yaw"),
//           n (300), fit (false), correct (false), controls (["hard","soft","noise","motion","fit","correct","collect"]) }
import { C, h, fmt, isPrint } from './util.js';
import { THREE, makeStage, label, line, arrow3 } from './three-util.js';
import { T } from './gripper.js';
import { CITIES, fieldNED, attitudes, transpose, matvec, matmul, euler, add, norm, deg, rng, fitEllipsoid, apAccept } from './magutil.js';
import { solve } from './linalg.js';

const U = 250;                                            // mG per scene unit
const HARD_DIR = (() => { const d = [0.6, -0.5, 0.62], l = Math.hypot(...d); return d.map(v => v / l); })();
const SOFT_AXES = euler(20 * deg, -15 * deg, 35 * deg);
const softMatrix = s => { // W = Q diag(1 + s, 1 − 0.6 s, 1 + 0.3 s) Qᵀ
  const D = [[1 + s, 0, 0], [0, 1 - 0.6 * s, 0], [0, 0, 1 + 0.3 * s]];
  return matmul(matmul(SOFT_AXES, D), transpose(SOFT_AXES));
};
const sup = n => String(n).replace(/\d/g, d => '⁰¹²³⁴⁵⁶⁷⁸⁹'[d]);
const vec = v => `(${v.map(x => fmt(x, 0)).join(', ')})`;

export function mount(el, cfg) {
  const W = cfg.width || 560, H = cfg.height || 440;
  const city = CITIES[cfg.city || 'Warsaw'], e = fieldNED(city), F = city.F;
  const ctl = cfg.controls || ['hard', 'soft', 'noise', 'motion', 'fit', 'correct', 'collect'];
  const state = {
    hard: cfg.hard ?? 0, soft: cfg.soft ?? 0, noise: cfg.noise ?? 3, motion: cfg.motion || 'all', n: cfg.n || 300, seed: 5,
    fit: cfg.fit ?? false, correct: cfg.correct ?? false, shown: cfg.n || 300, morph: cfg.correct ? 1 : 0,
  };

  const canvas = h('canvas', { width: W, height: H });
  const k = cfg.dist ?? 0.82;
  const stage = makeStage(canvas, { position: T(5.4 * k, -6.8 * k, 4.0 * k), target: T(0.3, -0.3, 0.3), fov: 38 });
  const { scene, render } = stage;

  // sensor axes at the origin and the ideal sphere (radius F, centred at 0)
  [[[1, 0, 0], 0xff6b6b, 'x'], [[0, 1, 0], 0x5fd38d, 'y'], [[0, 0, 1], 0x5ab0ff, 'z']].forEach(([d, col, name]) => {
    scene.add(arrow3(T(0, 0, 0), T(...d.map(v => v * 2.9)), col, 0.12));
    const l = label(name, { color: '#' + col.toString(16).padStart(6, '0'), size: 0.26 });
    l.position.set(...T(...d.map(v => v * 3.15))); scene.add(l);
  });
  const ideal = new THREE.LineSegments(new THREE.WireframeGeometry(new THREE.SphereGeometry(F / U, 18, 10)),
    new THREE.LineBasicMaterial({ color: 0x5a606c, transparent: true, opacity: 0.35 }));
  scene.add(ideal);
  const origin = new THREE.Mesh(new THREE.SphereGeometry(0.05, 12, 8), new THREE.MeshBasicMaterial({ color: 0xe7e9ee }));
  scene.add(origin);

  // samples (one geometry, updated in place)
  const NMAX = 400;
  const geo = new THREE.BufferGeometry();
  const posAttr = new THREE.BufferAttribute(new Float32Array(NMAX * 3), 3);
  geo.setAttribute('position', posAttr);
  const ptsMat = new THREE.PointsMaterial({ color: 0xf2b134, size: 0.075 });
  scene.add(new THREE.Points(geo, ptsMat));
  const dyn = new THREE.Group(); scene.add(dyn);

  // ---- side panel ----
  const readout = h('div', { style: 'line-height:1.45' });
  const verdict = h('div', { style: 'min-height:2.8em; line-height:1.3' });

  let raw = [], corr = [], fit = null, Mt = null;
  function simulate() {
    const r = rng(100 + state.seed), Rs = attitudes(state.n, state.motion, state.seed);
    const Wm = softMatrix(state.soft), hv = HARD_DIR.map(v => v * state.hard);
    raw = Rs.map(R => add(add(matvec(Wm, matvec(transpose(R), e)), hv), [0, 0, 0].map(() => state.noise * r.normal())));
    fit = fitEllipsoid(raw);
    if (fit.ok) {
      const s = F / fit.r;
      corr = raw.map(m => matvec(fit.M, add(m, fit.o)).map(v => v * s));
      Mt = [solve(fit.M, [1, 0, 0]), solve(fit.M, [0, 1, 0]), solve(fit.M, [0, 0, 1])]; // columns of M⁻¹
    } else corr = raw;
  }

  function ellipsoidLines() {
    // x = r M⁻¹ u − o for unit u (M symmetric, so M⁻¹ is too: columns = rows)
    const g = new THREE.WireframeGeometry(new THREE.SphereGeometry(1, 24, 14)), p = g.attributes.position;
    for (let i = 0; i < p.count; i++) {
      const u = [p.getX(i), -p.getZ(i), p.getY(i)];      // three.js → math frame
      const x = [0, 1, 2].map(k => fit.r * (Mt[0][k] * u[0] + Mt[1][k] * u[1] + Mt[2][k] * u[2]) - fit.o[k]);
      p.setXYZ(i, ...T(...x.map(v => v / U)));
    }
    return new THREE.LineSegments(g, new THREE.LineBasicMaterial({ color: 0xb28dff, transparent: true, opacity: 0.7 }));
  }

  function draw() {
    const k = Math.min(state.shown, raw.length), t = state.morph;
    for (let i = 0; i < k; i++) {
      const m = raw[i], c = corr[i];
      posAttr.setXYZ(i, ...T(...[0, 1, 2].map(j => (m[j] + (c[j] - m[j]) * t) / U)));
    }
    posAttr.needsUpdate = true; geo.setDrawRange(0, k); geo.computeBoundingSphere();
    ptsMat.color.set(t > 0.5 ? 0x5fd38d : 0xf2b134);
    dyn.clear();
    if (state.shown < raw.length && k > 0) {                 // the current reading during "collect"
      dyn.add(arrow3(T(0, 0, 0), T(...raw[k - 1].map(v => v / U)), 0xe7e9ee, 0.16));
    }
    const fitDone = state.shown >= raw.length;
    if (state.fit && fit.ok && fitDone && t < 0.5) {
      dyn.add(ellipsoidLines());
      const cm = new THREE.Mesh(new THREE.SphereGeometry(0.07, 12, 8), new THREE.MeshBasicMaterial({ color: 0xb28dff }));
      cm.position.set(...T(...fit.o.map(v => -v / U))); dyn.add(cm);
      dyn.add(line([T(0, 0, 0), T(...fit.o.map(v => -v / U))], 0xb28dff, { dashed: true }));
    }

    // readout
    const hv = HARD_DIR.map(v => v * state.hard);
    let html = `<div class="dim">${cfg.city || 'Warsaw'}: $F = ${F}$ mG · ${state.shown < raw.length ? `${k} / ${raw.length}` : raw.length} samples</div>`;
    html += `<div>true hard iron $h = ${vec(hv)}$</div>`;
    html += `<div>true soft iron: stretch ${fmt(100 * state.soft, 0)} %</div>`;
    if (state.fit && fitDone) {
      if (fit.ok) {
        const M = fit.M, acc = apAccept(fit);
        html += `<div style="margin-top:0.35em; color:${C.purple}">fitted ellipsoid</div>`;
        html += `<div>offsets $o = ${vec(fit.o)}$</div>`;
        html += `<div>DIA $= (${[M[0][0], M[1][1], M[2][2]].map(v => fmt(v, 3)).join(', ')})$</div>`;
        html += `<div>ODI $= (${[M[0][1], M[0][2], M[1][2]].map(v => fmt(v, 3)).join(', ')})$</div>`;
        html += `<div>radius $r = ${fmt(fit.r, 0)}$ mG → SCALE $= F/r = ${fmt(F / fit.r, 3)}$</div>`;
        html += `<div>fitness (RMS) $= ${fmt(fit.fitness, 1)}$ mG</div>`;
        verdict.innerHTML = acc.pass
          ? `<b style="color:${C.green}">accepted</b> <span class="dim">(ArduPilot criteria, CAL_FIT = 16)</span>`
          : `<b style="color:${C.red}">rejected</b>: ${acc.why}`;
      } else verdict.innerHTML = `<b style="color:${C.red}">fit failed</b>: the samples do not determine an ellipsoid`;
      if (fit.cond > 1e5) verdict.innerHTML += `<div style="color:${C.red}">ill-conditioned: cond ≈ 10${sup(Math.round(Math.log10(fit.cond)))} (all directions: ≈ 10²)</div>`;
    } else verdict.innerHTML = '';
    readout.innerHTML = html;
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
    render();
  }

  // ---- animations ----
  let anim = 0;
  function collect() {
    const id = ++anim, t0 = performance.now(), dur = 6000;
    state.morph = 0; corrBtn?.classList.remove('active'); state.correct = false;
    const step = now => {
      if (id !== anim || !canvas.isConnected) return;
      const u = Math.min(1, (now - t0) / dur);
      state.shown = Math.max(1, Math.round(u * raw.length)); draw();
      if (u < 1) requestAnimationFrame(step);
    };
    requestAnimationFrame(step);
  }
  function morphTo(target) {
    const id = ++anim, t0 = performance.now(), a = state.morph;
    state.shown = raw.length;
    const step = now => {
      if (id !== anim || !canvas.isConnected) return;
      const u = Math.min(1, (now - t0) / 1400), ee = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
      state.morph = a + (target - a) * ee; draw();
      if (u < 1) requestAnimationFrame(step);
    };
    requestAnimationFrame(step);
  }

  // ---- controls ----
  const slider = (key, lab, min, max, stepv, unit, scale = 1) => {
    const val = h('span', { class: 'readout', style: 'display:inline-block; width:4.4em' }, `${fmt(state[key] * scale, 0)} ${unit}`);
    const s = h('input', { type: 'range', min, max, step: stepv, value: state[key] });
    s.addEventListener('input', () => { state[key] = +s.value; val.textContent = `${fmt(state[key] * scale, 0)} ${unit}`; refresh(); });
    return h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', { style: 'width:4.6em' }, lab), s, val);
  };
  const refresh = () => { ++anim; state.shown = raw.length; simulate(); state.shown = raw.length; if (state.correct) state.morph = 1; draw(); };
  const toggle = (key, text, on) => {
    const b = h('button', { onclick: () => { state[key] = !state[key]; b.classList.toggle('active', state[key]); on?.(); } }, text);
    b.classList.toggle('active', state[key]);
    return b;
  };
  const fitBtn = toggle('fit', 'fit ellipsoid', () => draw());
  const corrBtn = toggle('correct', 'apply correction', () => morphTo(state.correct ? 1 : 0));
  const motionBtns = [['all', 'all directions'], ['tilts', 'tilts ≤ 20°'], ['yaw', 'flat spin']].map(([m, txt]) => {
    const b = h('button', { onclick: () => { state.motion = m; motionBtns.forEach(x => x.classList.toggle('active', x === b)); refresh(); } }, txt);
    b.classList.toggle('active', state.motion === m);
    return b;
  });
  const rows = [];
  if (ctl.includes('hard')) rows.push(slider('hard', 'hard iron', 0, 400, 10, 'mG'));
  if (ctl.includes('soft')) rows.push(slider('soft', 'soft iron', 0, 0.4, 0.01, '%', 100));
  if (ctl.includes('noise')) rows.push(slider('noise', 'noise σ', 0, 30, 1, 'mG'));
  if (ctl.includes('motion')) rows.push(h('div', { style: 'display:flex; gap:0.3em; flex-wrap:wrap' }, motionBtns));
  const acts = [];
  if (ctl.includes('collect')) acts.push(h('button', { onclick: collect }, '▶ collect'));
  if (ctl.includes('fit')) acts.push(fitBtn);
  if (ctl.includes('correct')) acts.push(corrBtn);
  acts.push(h('button', { onclick: () => { state.seed++; refresh(); } }, 'new samples'));
  rows.push(h('div', { style: 'display:flex; gap:0.3em; flex-wrap:wrap' }, acts));

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${W}px; flex:none` }, canvas,
      h('div', { class: 'dim', style: 'margin-top:0.2em' }, 'grey: ideal sphere (radius F, centre 0)')),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.45em; width:14em; flex:none' },
      readout, verdict,
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.35em' }, rows))));
  simulate();
  state.shown = raw.length;
  draw();
  if (isPrint()) return;
}
