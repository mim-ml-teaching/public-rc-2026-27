// How calibration detects a wrong compass orientation (CompassCalibrator::calculate_orientation).
// The compass is mounted rotated by Q relative to the body (here: yaw 90°, optionally with a small extra tilt).
// Each (already calibrated) reading m_i = Qᵀ R_iᵀ e + noise is stored together with the IMU attitude R_i.
// For a candidate orientation C the reading is mapped to the Earth frame: ê_i = R_i C m_i. With the right C every
// ê_i is the same vector e (a tight cluster); with a wrong C the points smear over a sphere. ArduPilot picks the
// candidate with the smallest variance and accepts it if the runner-up's variance is > 2× larger.
// Scene: Earth frame drawn as x = north, y = west, z = up; the grey arrow is the true field e.
//
// config: { width, height, candidate ("None"), tilt (deg, 0) }
import { C, h, fmt, isPrint } from './util.js';
import { THREE, makeStage, label, arrow3 } from './three-util.js';
import { T } from './gripper.js';
import { CITIES, fieldNED, attitudes, transpose, matvec, matmul, Rx, Ry, Rz, deg, rng } from './magutil.js';

const CANDS = [
  ['None', Rz(0)], ['Yaw 45', Rz(45 * deg)], ['Yaw 90', Rz(90 * deg)], ['Yaw 180', Rz(180 * deg)], ['Yaw 270', Rz(270 * deg)],
  ['Roll 180', Rx(180 * deg)], ['Pitch 180', Ry(180 * deg)], ['Yaw 90 Roll 180', matmul(Rz(90 * deg), Rx(180 * deg))],
];
const U = 250; // mG per scene unit
const show = v => T(v[0] / U, -v[1] / U, -v[2] / U); // NED → (north, west, up)

export function mount(el, cfg) {
  const W = cfg.width || 520, H = cfg.height || 420;
  const state = { cand: cfg.candidate || (isPrint() ? 'Yaw 90' : 'None'), tilt: cfg.tilt ?? 0 };
  const e = fieldNED(CITIES.Warsaw, true);
  const Rs = attitudes(300, 'all', 11), r = rng(4);
  const noise = Rs.map(() => [r.normal(), r.normal(), r.normal()].map(v => 4 * v));

  const canvas = h('canvas', { width: W, height: H });
  const { scene, render } = makeStage(canvas, { position: T(-4.4, -5.8, 3.3), target: T(0.3, 0, -0.4), fov: 40 });
  scene.add(new THREE.GridHelper(5, 10, 0x3a3f4b, 0x2a2e36));
  const ideal = new THREE.LineSegments(new THREE.WireframeGeometry(new THREE.SphereGeometry(CITIES.Warsaw.F / U, 20, 12)),
    new THREE.LineBasicMaterial({ color: 0x5a606c, transparent: true, opacity: 0.3 }));
  scene.add(ideal);
  scene.add(arrow3(T(0, 0, 0), show(e), 0x9aa1ae, 0.16));
  const el_ = label('e (WMM)', { color: C.dim, size: 0.24, font: '56px Inter, Arial' }); el_.position.set(...show(e).map((v, k) => v + [0.2, 0.25, 0][k])); scene.add(el_);
  const nl = label('N', { color: C.dim, size: 0.3, font: '600 64px Inter, Arial' }); nl.position.set(...T(2.6, 0, 0)); scene.add(nl);

  const geo = new THREE.BufferGeometry();
  const posAttr = new THREE.BufferAttribute(new Float32Array(Rs.length * 3), 3);
  geo.setAttribute('position', posAttr);
  const mat = new THREE.PointsMaterial({ color: 0xf2b134, size: 0.08 });
  scene.add(new THREE.Points(geo, mat));

  const bars = h('canvas', { width: 310, height: 250 });
  const gb = bars.getContext('2d');
  const note = h('div', { style: 'min-height:3em; line-height:1.3' });

  function earthPoints(Cm, Q) {
    return Rs.map((R, i) => {
      const m = matvec(transpose(Q), matvec(transpose(R), e)).map((v, k) => v + noise[i][k]);
      return matvec(R, matvec(Cm, m));
    });
  }
  const spread = pts => {
    const mean = [0, 1, 2].map(k => pts.reduce((s, p) => s + p[k], 0) / pts.length);
    return Math.sqrt(pts.reduce((s, p) => s + (p[0] - mean[0]) ** 2 + (p[1] - mean[1]) ** 2 + (p[2] - mean[2]) ** 2, 0) / pts.length);
  };

  function draw() {
    const Q = matmul(Rz(90 * deg), Rx(state.tilt * deg));     // true mounting
    const all = CANDS.map(([name, Cm]) => ({ name, s: spread(earthPoints(Cm, Q)) }));
    const pts = earthPoints(CANDS.find(c => c[0] === state.cand)[1], Q);
    pts.forEach((p, i) => posAttr.setXYZ(i, ...show(p)));
    posAttr.needsUpdate = true; geo.computeBoundingSphere();
    const sorted = [...all].sort((a, b) => a.s - b.s), best = sorted[0];
    mat.color.set(state.cand === best.name ? 0x5fd38d : 0xf2b134);
    render();

    // bar chart of spreads
    const Wb = bars.width, Hb = bars.height, x0 = 128, rowH = (Hb - 30) / all.length, smax = Math.max(...all.map(a => a.s));
    gb.fillStyle = C.bg2; gb.fillRect(0, 0, Wb, Hb);
    gb.font = '15px Inter, Arial';
    all.forEach((a, i) => {
      const y = 8 + i * rowH, w = Math.max(2, (Wb - x0 - 50) * a.s / smax);
      gb.fillStyle = a.name === best.name ? C.green : a.name === state.cand ? C.accent : C.dim;
      gb.fillRect(x0, y + 4, w, rowH - 8);
      gb.fillStyle = a.name === state.cand ? C.accent : C.fg;
      gb.fillText(a.name, 6, y + rowH / 2 + 5);
      gb.fillStyle = C.dim; gb.fillText(fmt(a.s, 0), x0 + w + 6, y + rowH / 2 + 5);
    });
    gb.fillStyle = C.dim; gb.fillText('RMS spread of ê_i (mG)', 6, Hb - 6);
    const conf = (sorted[1].s / best.s) ** 2;
    note.innerHTML = state.cand === best.name
      ? `<b style="color:${C.green}">best candidate</b>: spread ${fmt(best.s, 0)} mG ≈ noise${state.tilt ? ' + tilt' : ''}. Variance ratio to runner-up ${fmt(conf, 0)} > 2 → accepted.`
      : `Points smear over the sphere: <b>${state.cand}</b> is wrong.`;
  }

  const btns = CANDS.map(([name]) => {
    const b = h('button', { onclick: () => { state.cand = name; btns.forEach(x => x.classList.toggle('active', x === b)); draw(); } }, name);
    b.classList.toggle('active', name === state.cand);
    return b;
  });
  const tiltBtn = h('button', { onclick: () => { state.tilt = state.tilt ? 0 : 3; tiltBtn.classList.toggle('active', !!state.tilt); draw(); } }, 'mount tilted by extra 3°');
  tiltBtn.classList.toggle('active', !!state.tilt);

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${W}px; flex:none` }, canvas,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.25em' }, h('label', {}, 'candidate:'), btns)),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.45em; width:15em; flex:none' },
      bars, note, h('div', { class: 'wctl interactive-only' }, tiltBtn))));
  draw();
}
