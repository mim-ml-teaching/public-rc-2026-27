// Rigid vs non-rigid motion of a bar with three marked points p, q, r and the distances between them.
// Every vertex is mapped by g_t (math frame, z up); the table compares |g(a) − g(b)| with |a − b|.
// config: { mode: "rigid" | "stretch" | "twist", width, height }
import { C, h, fmt, isPrint } from './util.js';
import { THREE, makeStage, label, line } from './three-util.js';
import { T } from './gripper.js';
import { axisAngle, apply } from './rotmath.js';

const P = { p: [-1, -0.3, 0.2], q: [1, -0.3, 0.2], r: [-1, 0.3, 0.2] };
const PAIRS = [['p', 'q'], ['q', 'r'], ['p', 'r']];
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

const MAPS = {
  // rotation by 70° about a tilted axis + translation
  rigid: (x, t) => {
    const R = axisAngle(normalize([0.3, 0.5, 1]), 70 * Math.PI / 180 * t);
    const y = apply(R, x);
    return [y[0] + 0.6 * t, y[1] + 0.8 * t, y[2] + 0.5 * t];
  },
  // stretch along the bar, squash vertically
  stretch: (x, t) => [x[0] * (1 + 0.5 * t) + 0.3 * t, x[1], x[2] * (1 - 0.4 * t) + 0.4 * t],
  // twist about the bar's long axis: angle grows with x
  twist: (x, t) => {
    const a = 1.1 * t * x[0], c = Math.cos(a), s = Math.sin(a);
    return [x[0], c * x[1] - s * x[2], s * x[1] + c * x[2] + 0.3 * t];
  },
};
function normalize(v) { const l = Math.hypot(...v); return v.map(x => x / l); }

export function mount(el, cfg) {
  const W = cfg.width || 540, H = cfg.height || 380;
  const fixed = cfg.t !== undefined;
  const state = { mode: cfg.mode || 'rigid', t: fixed ? cfg.t : isPrint() ? 1 : 0 };
  const canvas = h('canvas', { width: W, height: H });
  const { scene, render } = makeStage(canvas, { position: T(0.9, -3.0, 2.5), target: T(0.35, 0.35, 0.35) });

  const grid = new THREE.GridHelper(6, 12, 0x3a3f4b, 0x2a2e36); grid.position.set(0, -0.6, 0); scene.add(grid);

  // the bar: a subdivided box whose vertices we move every frame
  const geo = new THREE.BoxGeometry(2, 0.4, 0.6, 16, 2, 2); // three.js sizes: x, y(up)=math z, z=−math y
  const orig = [];
  const pos = geo.attributes.position;
  for (let i = 0; i < pos.count; i++) orig.push([pos.getX(i), -pos.getZ(i), pos.getY(i)]); // back to math frame
  const bar = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: 0x8a93a6, roughness: 0.6, transparent: true, opacity: 0.85 }));
  scene.add(bar);
  const ghost = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(2, 0.4, 0.6)),
    new THREE.LineDashedMaterial({ color: 0x9aa1ae, dashSize: 0.08, gapSize: 0.06 }));
  ghost.computeLineDistances(); scene.add(ghost);

  const balls = {}, labels = {};
  for (const k of Object.keys(P)) {
    balls[k] = new THREE.Mesh(new THREE.SphereGeometry(0.07, 16, 12), new THREE.MeshBasicMaterial({ color: 0xf2b134, depthTest: false }));
    balls[k].renderOrder = 6; scene.add(balls[k]);
    labels[k] = label(k, { color: C.accent, size: 0.3 }); scene.add(labels[k]);
  }
  const dyn = new THREE.Group(); scene.add(dyn);

  const rows = PAIRS.map(([a, b]) => {
    const now = h('td', { class: 'readout' }), mark = h('td', {});
    return { a, b, d0: dist(P[a], P[b]), tr: h('tr', {}, h('td', {}, `|${a} − ${b}|`), h('td', { class: 'readout' }, fmt(dist(P[a], P[b]))), now, mark), now, mark };
  });
  const table = h('table', { style: 'font-size:1em' },
    h('tr', {}, h('th', {}, 'distance'), h('th', {}, 'before'), h('th', {}, 'now'), h('th', {}, '')), rows.map(r => r.tr));
  const verdict = h('div', { style: 'font-weight:600; min-height:1.4em' });

  function draw() {
    const g = x => MAPS[state.mode](x, state.t);
    orig.forEach((x, i) => pos.setXYZ(i, ...T(...g(x))));
    pos.needsUpdate = true; geo.computeVertexNormals(); geo.computeBoundingSphere();
    const cur = Object.fromEntries(Object.entries(P).map(([k, x]) => [k, g(x)]));
    for (const k of Object.keys(P)) {
      balls[k].position.set(...T(...cur[k]));
      labels[k].position.set(...T(cur[k][0], cur[k][1], cur[k][2] + 0.28));
    }
    dyn.clear();
    PAIRS.forEach(([a, b]) => { const l = line([T(...cur[a]), T(...cur[b])], 0xf2b134); l.material.depthTest = false; l.renderOrder = 5; dyn.add(l); });
    let rigid = true;
    rows.forEach(r => {
      const d = dist(cur[r.a], cur[r.b]), same = Math.abs(d - r.d0) < 5e-3;
      rigid &&= same;
      r.now.textContent = fmt(d);
      r.now.style.color = same ? C.green : C.red;
      r.mark.textContent = same ? '✓' : '✗';
      r.mark.style.color = same ? C.green : C.red;
    });
    verdict.innerHTML = rigid ? `<span style="color:${C.green}">all distances preserved</span>` : `<span style="color:${C.red}">distances change: the object deforms</span>`;
    render();
  }

  // ping-pong animation
  let t0 = performance.now();
  const loop = now => {
    if (!canvas.isConnected) return;
    const u = (Math.max(0, now - t0) / 4000) % 1;
    const e = u < 0.5 ? 2 * u : 2 - 2 * u;
    state.t = e < 0.5 ? 2 * e * e : 1 - Math.pow(-2 * e + 2, 2) / 2;
    draw();
    requestAnimationFrame(loop);
  };

  const btns = [['rigid', 'rigid motion'], ['stretch', 'stretch'], ['twist', 'twist']].map(([m, txt]) => {
    const b = h('button', { onclick: () => { state.mode = m; t0 = performance.now(); btns.forEach(x => x.classList.toggle('active', x === b)); } }, txt);
    b.classList.toggle('active', m === state.mode);
    return b;
  });

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    canvas,
    h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:15em; flex:none' },
      table, verdict,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, btns),
      h('div', { class: 'dim' }, 'dashed: starting position'))));
  draw();
  if (!isPrint() && !fixed) requestAnimationFrame(loop);
}
