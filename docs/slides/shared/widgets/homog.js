// Homogeneous coordinates of a 2D point: the ray through the origin and (x, y, 1).
// Every point λ(x, y, 1), λ ≠ 0, represents the same 2D point. config: { x, y, lambda }
import { C, h, fmt } from './util.js';
import { THREE, makeStage, label, line, arrow3, viewButtons } from './three-util.js';

// math (x̃, ỹ, z̃) -> three.js (X, Y, Z): z̃ is "up"
const T = (x, y, z) => [x, z, -y];

export function mount(el, cfg) {
  const state = { x: cfg.x ?? 1.2, y: cfg.y ?? 0.8, lambda: cfg.lambda ?? 1.8 };
  const canvas = h('canvas', { width: cfg.width || 640, height: cfg.height || 440 });
  const view3d = { label: '3D view', position: cfg.camera || [1.0, 1.55, 6.6], target: cfg.target || [1.15, 1.0, -0.8], fov: 40 };
  const viewTop = { label: 'top view', position: [1, 16, -1 + 0.01], target: [1, 1, -1], fov: 16 };
  const start = cfg.view === 'top' ? viewTop : view3d;
  const stage = makeStage(canvas, { position: start.position, target: start.target, fov: start.fov });
  const { scene, render } = stage;

  [[T(2.4, 0, 0), 'x̃', 0xff6b6b, C.red], [T(0, 2.4, 0), 'ỹ', 0x5fd38d, C.green], [T(0, 0, 2.4), 'z̃', 0x5ab0ff, C.blue]]
    .forEach(([tip, name, hex, css]) => {
      scene.add(arrow3([0, 0, 0], tip, hex, 0.14));
      const l = label(name, { color: css }); l.position.set(...tip.map(v => v * 1.08)); scene.add(l);
    });

  // plane z̃ = 1
  const plane = new THREE.Mesh(
    new THREE.PlaneGeometry(4, 4),
    new THREE.MeshBasicMaterial({ color: 0x9aa1ae, transparent: true, opacity: 0.16, side: THREE.DoubleSide, depthWrite: false }),
  );
  plane.rotation.x = -Math.PI / 2; plane.position.set(1, 1, -1);
  scene.add(plane);
  const planeGrid = new THREE.GridHelper(4, 8, 0x5a606c, 0x3a3f4b);
  planeGrid.position.set(1, 1, -1); scene.add(planeGrid);
  const planeLabel = label('z̃ = 1', { color: C.dim, size: 0.22 });
  planeLabel.position.set(...T(2.7, -0.8, 1.02)); scene.add(planeLabel);

  const group = new THREE.Group(); scene.add(group);
  const readout = h('div', { class: 'readout', style: 'font-size:1.05em; line-height:1.6' });

  function rebuild() {
    group.clear();
    const { x, y, lambda: l } = state;
    const far = 2.6;
    group.add(line([T(0, 0, 0), T(far * x, far * y, far)], 0xf2b134));       // the ray L
    group.add(line([T(-0.4 * x, -0.4 * y, -0.4), T(0, 0, 0)], 0xf2b134, { dashed: true }));
    const pt = (p, color, r = 0.06) => {
      const m = new THREE.Mesh(new THREE.SphereGeometry(r, 20, 14), new THREE.MeshStandardMaterial({ color }));
      m.position.set(...p); group.add(m); return m;
    };
    pt(T(x, y, 1), 0xff6b6b, 0.075);                  // p on the plane
    pt(T(l * x, l * y, l), 0x5ab0ff);                 // p̃ = λ(x, y, 1)
    // drop lines from p̃ to show its coordinates
    group.add(line([T(l * x, l * y, l), T(l * x, l * y, 0)], 0x5ab0ff, { dashed: true }));
    group.add(line([T(l * x, l * y, 0), T(l * x, 0, 0)], 0x5ab0ff, { dashed: true }));
    group.add(line([T(l * x, l * y, 0), T(0, l * y, 0)], 0x5ab0ff, { dashed: true }));
    const lp = label('p(x, y)', { color: C.red, size: 0.24 }); lp.position.set(...T(x + 0.33, y - 0.19, 1.1)); group.add(lp);
    const lq = label('p̃(x̃, ỹ, z̃)', { color: C.blue, size: 0.24 }); lq.position.set(...T(l * x - 0.28, l * y + 0.15, l + 0.12)); group.add(lq);
    const lL = label('L', { color: C.accent, size: 0.26 }); lL.position.set(...T(far * x + 0.1, far * y, far + 0.1)); group.add(lL);
    readout.innerHTML =
      `p̃ = (<span style="color:${C.blue}">${fmt(l * x)}, ${fmt(l * y)}, ${fmt(l)}</span>)` +
      `<br>p = (x̃/z̃, ỹ/z̃) = (<span style="color:${C.red}">${fmt(x)}, ${fmt(y)}</span>)`;
    render();
  }

  const slider = (key, min, max, step, name) => {
    const s = h('input', { type: 'range', min, max, step, value: state[key] });
    s.addEventListener('input', () => { state[key] = parseFloat(s.value); rebuild(); });
    return h('div', {}, h('label', {}, name + ' '), s);
  };

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    canvas,
    h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:14em; flex:none' },
      readout,
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start' },
        slider('lambda', 0.2, 2.5, 0.01, 'λ (scale)'),
        slider('x', -1, 2, 0.01, 'x'),
        slider('y', -1, 2, 0.01, 'y'),
        h('div', { style: 'display:flex; gap:0.3em' }, viewButtons(stage, [viewTop, view3d])),
        h('div', { class: 'dim' }, 'drag to orbit'),
      ),
    ),
  ));
  rebuild();
}
