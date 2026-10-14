// P3P: three points seen by a calibrated camera can be explained by up to four poses. The camera (centre at the
// origin, camera frame x right, y down, z forward) sees three pixels = three rays. Four triangles, all with the
// same side lengths, have their corners on these rays: each gives exactly the same image (the true one and three
// others, tilted differently and at different distances).
// The solutions were computed numerically (law of cosines along the rays) for this configuration.
// By default the poses are shown one by one (the others faint); print shows all four.
// config: { width, height, show: "cycle" | "all" | 0..3 }
import { C as COL, h, isPrint } from './util.js';
import { THREE, makeStage, label, line, viewButtons } from './three-util.js';

// camera frame -> three.js: X = z (forward), Y = -y (up), Z = x   (a proper rotation, as in pinhole3d)
const T = (x, y, z) => [z, -y, x];
const SOLUTIONS = [
  [[0.9372, -0.3357, 2.7629], [-0.163, 0.648, 2.9641], [-0.2076, -0.8966, 1.9857]],   // the true pose
  [[0.6653, -0.2383, 1.9613], [-0.157, 0.6241, 2.8549], [-0.2748, -1.1869, 2.6288]],
  [[0.9145, -0.3276, 2.6961], [-0.1047, 0.4164, 1.9048], [-0.2825, -1.22, 2.702]],
  [[0.9578, -0.3431, 2.8238], [-0.1602, 0.6369, 2.9135], [-0.267, -1.1531, 2.5539]],
];
const COLORS = [0xf2b134, 0x5ab0ff, 0x5fd38d, 0xb28dff];
const hex = c => '#' + c.toString(16).padStart(6, '0');
const F = 400, IW = 640, IH = 480;           // the camera that took the image
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

export function mount(el, cfg) {
  const W = cfg.width || 620, H = cfg.height || 380;
  const state = { show: cfg.show ?? (isPrint() ? 'all' : 'cycle'), k: 0 };
  const canvas = h('canvas', { width: W, height: H });
  const viewSide = { label: 'side view', position: T(3.6, -1.6, 1.3), target: T(0.25, -0.32, 1.7), fov: 46 };
  const viewCam = { label: 'from behind the camera', position: T(0.15, -0.2, -1.6), target: T(0.3, -0.25, 2.5), fov: 46 };
  const stage = makeStage(canvas, { position: viewSide.position, target: viewSide.target });
  const { scene, render } = stage;
  const dyn = new THREE.Group(); scene.add(dyn);

  // camera: centre, a small image plane and the three rays (fixed)
  const S = 0.35, hw = S * IW / 2 / F, hh = S * IH / 2 / F;
  const cs = [[-hw, -hh, S], [hw, -hh, S], [hw, hh, S], [-hw, hh, S]];
  scene.add(line([...cs, cs[0]].map(p => T(...p)), 0x9aa1ae));
  cs.forEach(c => scene.add(line([T(0, 0, 0), T(...c)], 0x4a505c)));
  const cam = new THREE.Mesh(new THREE.SphereGeometry(0.035, 14, 10), new THREE.MeshBasicMaterial({ color: 0xe7e9ee }));
  scene.add(cam);
  const lc = label('camera', { color: COL.fg, size: 0.13, font: '48px Inter, Arial' }); lc.position.set(...T(0, 0.16, 0)); scene.add(lc);
  SOLUTIONS[0].forEach((X, i) => {
    const far = X.map(v => v * 1.18);
    scene.add(line([T(0, 0, 0), T(...far)], 0x9aa1ae));
    const m = new THREE.Mesh(new THREE.SphereGeometry(0.012, 10, 8), new THREE.MeshBasicMaterial({ color: 0xff6b6b }));
    m.position.set(...T(...X.map(v => v * S / X[2]))); scene.add(m);
    const l = label(`ray ${i + 1}`, { color: COL.dim, size: 0.11, font: '48px Inter, Arial' }); l.position.set(...T(...far.map(v => v * 1.03))); scene.add(l);
  });

  // the image (identical for every solution)
  const SC = 0.3, img = h('canvas', { width: IW * SC * 2, height: IH * SC * 2, style: `width:${IW * SC}px; height:${IH * SC}px; border:1px solid #3a3f4b; border-radius:4px; display:block` });
  {
    const g = img.getContext('2d'), k = img.width / IW;
    g.fillStyle = '#0d0f13'; g.fillRect(0, 0, img.width, img.height);
    const px = SOLUTIONS[0].map(([x, y, z]) => [F * x / z + IW / 2, F * y / z + IH / 2]);
    g.strokeStyle = '#4a505c'; g.lineWidth = 2; g.setLineDash([8, 6]);
    g.beginPath(); px.forEach(([u, v], i) => (i ? g.lineTo(u * k, v * k) : g.moveTo(u * k, v * k))); g.closePath(); g.stroke(); g.setLineDash([]);
    g.font = '30px Inter, Arial';
    px.forEach(([u, v], i) => { g.fillStyle = COL.red; g.beginPath(); g.arc(u * k, v * k, 9, 0, 7); g.fill(); g.fillStyle = COL.fg; g.fillText(String(i + 1), u * k + 14, v * k - 10); });
  }

  const readout = h('div', { style: 'font-size:0.92em; line-height:1.45' });
  const btns = [];
  // 'cycle' (default): every pose in turn, the others faint; 'all': all four equally; 0..3: one pose
  function update() {
    dyn.clear();
    const focus = state.show === 'cycle' ? state.k : state.show;
    SOLUTIONS.forEach((P, s) => {
      if (typeof state.show === 'number' && state.show !== s) return;
      const hi = focus === 'all' || focus === s;
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(P.flatMap(p => T(...p)), 3));
      geo.setIndex([0, 1, 2]);
      dyn.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: COLORS[s], transparent: true, opacity: hi ? (focus === 'all' ? 0.25 : 0.45) : 0.05, side: THREE.DoubleSide, depthWrite: false })));
      dyn.add(line([...P, P[0]].map(p => T(...p)), hi ? COLORS[s] : 0x4a505c, { width: 2 }));
      P.forEach(p => {
        const m = new THREE.Mesh(new THREE.SphereGeometry(hi ? 0.035 : 0.018, 14, 10), new THREE.MeshBasicMaterial({ color: hi ? COLORS[s] : 0x5a606c }));
        m.position.set(...T(...p)); dyn.add(m);
      });
    });
    render();
    btns.forEach(([b, v]) => b.classList.toggle('active', state.show === v));
    const P = typeof focus === 'number' ? SOLUTIONS[focus] : SOLUTIONS[0];
    const sides = [[0, 1], [1, 2], [2, 0]].map(([a, b]) => dist(P[a], P[b]).toFixed(2));
    readout.innerHTML =
      (typeof focus === 'number'
        ? `<div style="font-size:1.25em"><b style="color:${hex(COLORS[focus])}">pose ${focus + 1} of 4</b>${focus === 0 ? ' (the true one)' : ''}</div>`
        : `<div style="font-size:1.25em"><b>all four poses</b></div>`) +
      `<div>side lengths: <b>${sides.join(', ')}</b> m</div>` +
      `<div>distances along the rays: ${P.map(p => Math.hypot(...p).toFixed(2)).join(', ')} m</div>` +
      `<div class="dim">every pose: the same triangle, the same three pixels</div>`;
  }

  const set = v => { state.show = v; update(); };
  btns.push([h('button', { onclick: () => set('cycle') }, '▶ one by one'), 'cycle']);
  btns.push([h('button', { onclick: () => set('all') }, 'all four'), 'all']);
  SOLUTIONS.forEach((_, s) => btns.push([h('button', { style: `border-color:${hex(COLORS[s])}`, onclick: () => set(s) }, `pose ${s + 1}`), s]));

  // step through the poses while the slide is shown
  const timer = setInterval(() => {
    if (!el.isConnected) { clearInterval(timer); return; }
    if (state.show !== 'cycle' || el.closest('section')?.classList.contains('present') === false) return;
    state.k = (state.k + 1) % SOLUTIONS.length; update();
  }, 1800);

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', {}, canvas, h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, ...viewButtons(stage, [viewSide, viewCam]))),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:16em; flex:none' },
      h('div', {}, h('div', { style: 'margin-bottom:0.2em' }, 'the image'), img),
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em; margin-top:0' }, ...btns.map(([b]) => b)),
      readout)));
  update();
}
