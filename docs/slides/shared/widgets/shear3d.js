// "Translation in 2D is a linear map (a shear) in 3D."
// The 3D map (x̃, ỹ, z̃) ↦ (x̃ + tx z̃, ỹ + ty z̃, z̃) fixes the plane z̃ = 0 and slides the plane z̃ = 1
// (where the image lives) by (tx, ty). config: { tx, ty, autoplay }
import { C, h, fmt, isPrint, testImage } from './util.js';
import { THREE, makeStage, label, line, arrow3, viewButtons } from './three-util.js';

const T = (x, y, z) => [x, z, -y]; // math frame (z̃ up) -> three.js (y up)

export function mount(el, cfg) {
  const TX = cfg.tx ?? 0.9, TY = cfg.ty ?? 0.5;
  const state = { s: isPrint() ? 1 : (cfg.s ?? 0), guides: true }; // guides: 3D-only helper lines
  const canvas = h('canvas', { width: cfg.width || 640, height: cfg.height || 440 });
  const view3d = { label: '3D view', position: cfg.camera || [-1.7, 2.3, 3.5], target: cfg.target || [0.9, 0.7, -0.7], fov: 40, onSelect: () => { state.guides = true; draw(); } };
  const viewTop = { label: 'top view', position: [1.2, 14, -0.9 + 0.01], target: [1.2, 0, -0.9], fov: 14, onSelect: () => { state.guides = false; draw(); } };
  const start = cfg.view === 'top' ? viewTop : view3d;
  if (start === viewTop) state.guides = false;
  const stage = makeStage(canvas, { position: start.position, target: start.target, fov: start.fov });
  const { scene, render } = stage;

  [[T(2.6, 0, 0), 'x̃', 0xff6b6b, C.red], [T(0, 2.2, 0), 'ỹ', 0x5fd38d, C.green], [T(0, 0, 1.9), 'z̃', 0x5ab0ff, C.blue]]
    .forEach(([tip, name, hex, css]) => {
      scene.add(arrow3([0, 0, 0], tip, hex, 0.12));
      const l = label(name, { color: css }); l.position.set(...tip.map(v => v * 1.08)); scene.add(l);
    });

  // fixed floor z̃ = 0
  const floor = new THREE.GridHelper(4, 8, 0x4a505c, 0x30343d);
  floor.position.set(1, 0, -1); scene.add(floor);
  const l0 = label('z̃ = 0 (fixed)', { color: C.dim, size: 0.2 }); l0.position.set(...T(3.0, 2.2, 0)); scene.add(l0);

  // image texture on the plane z̃ = 1
  const tex = new THREE.CanvasTexture(testImage(320, 240));
  tex.colorSpace = THREE.SRGBColorSpace;
  const imgW = 1.6, imgH = 1.2;
  const quadGeom = new THREE.BufferGeometry();
  quadGeom.setAttribute('uv', new THREE.Float32BufferAttribute([0, 0, 1, 0, 1, 1, 0, 1], 2));
  quadGeom.setIndex([0, 1, 2, 0, 2, 3]);
  const quad = new THREE.Mesh(quadGeom, new THREE.MeshBasicMaterial({ map: tex, side: THREE.DoubleSide }));
  scene.add(quad);
  // ghost of the image before the shear
  const ghost = [[0, 0], [imgW, 0], [imgW, imgH], [0, imgH], [0, 0]].map(([x, y]) => T(x, y, 1));
  scene.add(line(ghost, 0x9aa1ae, { dashed: true }));

  const dyn = new THREE.Group(); scene.add(dyn);
  const volMat = new THREE.MeshBasicMaterial({ color: 0xb28dff, transparent: true, opacity: 0.1, side: THREE.DoubleSide, depthWrite: false });
  const sliceMat = new THREE.MeshBasicMaterial({ color: 0xb28dff, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false });
  const readout = h('div', { class: 'readout', style: 'font-size:1.05em; line-height:1.5' });

  function draw() {
    const tx = TX * state.s, ty = TY * state.s;
    const S = (x, y, z) => T(x + tx * z, y + ty * z, z);
    // image corners (image spans [0,imgW]x[0,imgH] on z̃ = 1)
    const corners = [[0, 0], [imgW, 0], [imgW, imgH], [0, imgH]].map(([x, y]) => S(x, y, 1));
    quadGeom.setAttribute('position', new THREE.Float32BufferAttribute(corners.flat(), 3));
    quadGeom.attributes.position.needsUpdate = true;
    quadGeom.computeBoundingSphere();

    dyn.clear();
    // sheared unit-ish box and vertical "columns" of space: straight lines stay straight, origin stays put
    if (state.guides) {
      for (const x of [0, 0.8, 1.6]) for (const y of [0, 0.6, 1.2])
        dyn.add(line([S(x, y, 0), S(x, y, 1.5)], x === 0 && y === 0 ? 0xf2b134 : 0x5a606c));
      const box = [[0, 0], [1.6, 0], [1.6, 1.2], [0, 1.2], [0, 0]];
      dyn.add(line(box.map(([x, y]) => S(x, y, 1.5)), 0x5a606c));
      // the sheared volume itself: the map acts on all of space, not only on the image plane
      const V = [];
      for (const z of [0, 1.5]) for (const [x, y] of [[0, 0], [1.6, 0], [1.6, 1.2], [0, 1.2]]) V.push(...S(x, y, z));
      const vol = new THREE.BufferGeometry();
      vol.setAttribute('position', new THREE.Float32BufferAttribute(V, 3));
      vol.setIndex([0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6, 0, 4, 5, 0, 5, 1, 1, 5, 6, 1, 6, 2, 2, 6, 7, 2, 7, 3, 3, 7, 4, 3, 4, 0]);
      dyn.add(new THREE.Mesh(vol, volMat));
      // a slice half-way up moves half as far
      const half = [[0, 0], [1.6, 0], [1.6, 1.2], [0, 1.2], [0, 0]].map(([x, y]) => S(x, y, 0.5));
      dyn.add(line(half, 0xb28dff));
      const sliceGeo = new THREE.BufferGeometry();
      sliceGeo.setAttribute('position', new THREE.Float32BufferAttribute(half.slice(0, 4).flat(), 3));
      sliceGeo.setIndex([0, 1, 2, 0, 2, 3]);
      dyn.add(new THREE.Mesh(sliceGeo, sliceMat));
      // rays from the origin through the image corners: homogeneous points = rays
      corners.forEach(c => dyn.add(line([[0, 0, 0], c], 0xb28dff, { dashed: true })));
    }
    // translation vector on the image plane
    if (state.s > 0.02) dyn.add(arrow3(T(0, 0, 1), S(0, 0, 1), 0xf2b134, 0.12));

    readout.innerHTML =
      `<div>$\\begin{bmatrix}1&0&${fmt(tx)}\\\\0&1&${fmt(ty)}\\\\0&0&1\\end{bmatrix}$</div>` +
      `<div class="dim" style="margin-top:0.4em">plane z̃ = 1 moves by (${fmt(tx)}, ${fmt(ty)})<br>plane z̃ = 0 does not move</div>`;
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }] });
    render();
  }

  const slider = h('input', { type: 'range', min: 0, max: 1, step: 0.01, value: state.s });
  slider.addEventListener('input', () => { state.s = parseFloat(slider.value); draw(); });
  let anim;
  const play = h('button', {
    onclick: () => {
      cancelAnimationFrame(anim);
      const t0 = performance.now();
      const step = now => {
        const u = Math.min(1, (now - t0) / 1600);
        state.s = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
        slider.value = state.s; draw();
        if (u < 1) anim = requestAnimationFrame(step);
      };
      anim = requestAnimationFrame(step);
    },
  }, '▶ translate image');

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    canvas,
    h('div', { style: 'display:flex; flex-direction:column; gap:0.6em; width:13em; flex:none' },
      readout,
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start' },
        h('div', {}, h('label', {}, 'amount '), slider), play,
        h('div', { style: 'display:flex; gap:0.3em' }, viewButtons(stage, [viewTop, view3d])),
        h('div', { class: 'dim' }, 'drag to orbit')),
    ),
  ));
  draw();
  if (cfg.autoplay && !isPrint()) setTimeout(() => play.click(), 400);
}
