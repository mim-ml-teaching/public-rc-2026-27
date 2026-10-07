// The perspective-projection figure in 3D: camera frame (x right, y down, z forward), virtual image plane
// at z = f, a point P and its image. "top view" looks along +y (from above) and reproduces the 2D figure.
// config: { P: [x, y, z], f, width, height }
import { C, h } from './util.js';
import { THREE, makeStage, label, line, arrow3, viewButtons } from './three-util.js';

// camera frame -> three.js: X = z (right), Y = -y (up), Z = x   (a proper rotation)
const T = (x, y, z) => [z, -y, x];

export function mount(el, cfg) {
  const [Px, Py, Pz] = cfg.P || [1.1, -0.55, 3.6];
  const f = cfg.f ?? 1.2;
  const W = cfg.width || 460, H = cfg.height || 300;
  const canvas = h('canvas', { width: W, height: H });
  const view3d = { label: '3D view', position: cfg.camera || [-2.1, 2.7, 3.1], target: cfg.target || [1.5, -0.25, 0.35], fov: 40 };
  const viewTop = { label: 'top view (= the 2D figure)', position: [2.0, 13, 0.4 + 0.01], target: [2.0, 0, 0.4], fov: 26 };
  const stage = makeStage(canvas, { position: view3d.position, target: view3d.target });
  const { scene, render } = stage;
  const add = o => { scene.add(o); return o; };
  const lab = (text, p, color, size = 0.24) => { const l = label(text, { color, size }); l.position.set(...T(...p)); add(l); };

  // camera axes
  add(arrow3(T(0, 0, 0), T(1.2, 0, 0), 0xff6b6b, 0.12)); lab('x', [1.35, 0, 0], C.red);
  add(arrow3(T(0, 0, 0), T(0, 0.9, 0), 0x5fd38d, 0.12)); lab('y', [0, 1.05, 0], C.green);
  add(arrow3(T(0, 0, 0), T(0, 0, 4.6), 0x5ab0ff, 0.12)); lab('z', [0, 0, 4.8], C.blue);
  const pin = add(new THREE.Mesh(new THREE.SphereGeometry(0.06, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff })));
  pin.position.set(...T(0, 0, 0)); lab('pinhole', [-0.25, -0.25, -0.2], C.fg, 0.2);

  // virtual image plane at z = f
  const pw = 1.6, ph = 1.2;
  const corners = [[-pw / 2, -ph / 2], [pw / 2, -ph / 2], [pw / 2, ph / 2], [-pw / 2, ph / 2]].map(([x, y]) => T(x, y, f));
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(corners.flat(), 3));
  geo.setIndex([0, 1, 2, 0, 2, 3]);
  add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0x9aa1ae, transparent: true, opacity: 0.22, side: THREE.DoubleSide, depthWrite: false })));
  add(line([...corners, corners[0]], 0x9aa1ae));
  lab('virtual image plane', [-pw / 2 + 0.2, -ph / 2 - 0.22, f], C.dim, 0.2);

  // point, ray, image point
  const P = add(new THREE.Mesh(new THREE.SphereGeometry(0.08, 16, 12), new THREE.MeshStandardMaterial({ color: 0xff6b6b })));
  P.position.set(...T(Px, Py, Pz)); lab('P', [Px + 0.2, Py - 0.15, Pz], C.red, 0.3);
  add(line([T(0, 0, 0), T(Px * 1.15, Py * 1.15, Pz * 1.15)], 0xf2b134));
  const xi = f * Px / Pz, yi = f * Py / Pz;
  const pi = add(new THREE.Mesh(new THREE.SphereGeometry(0.06, 16, 12), new THREE.MeshBasicMaterial({ color: 0xf2b134 })));
  pi.position.set(...T(xi, yi, f)); lab('(xᵢ, yᵢ)', [xi + 0.15, yi - 0.2, f], C.accent, 0.22);

  // coordinates of P: drop along y to the x–z plane, then x_c and z_c
  add(line([T(Px, Py, Pz), T(Px, 0, Pz)], 0x5fd38d, { dashed: true }));
  add(line([T(Px, 0, Pz), T(0, 0, Pz)], 0xff6b6b, { dashed: true })); lab('x_c', [Px / 2, 0.2, Pz + 0.15], C.red, 0.22);
  add(line([T(xi, yi, f), T(xi, 0, f)], 0x5fd38d, { dashed: true }));
  add(line([T(xi, 0, f), T(0, 0, f)], 0xf2b134, { dashed: true }));
  lab('z_c', [-0.2, 0.25, Pz * 0.62], C.blue, 0.22);
  lab('f', [-0.2, 0.25, f / 2], C.fg, 0.26);

  // where the top view looks from
  add(arrow3(T(0.2, -1.5, 3.0), T(0.2, -0.9, 3.0), 0x9aa1ae, 0.14));
  lab('top view: looking along y', [0.2, -1.68, 3.0], C.dim, 0.18);

  el.classList.add('widget');
  el.append(canvas, h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, viewButtons(stage, [view3d, viewTop])));
  render();
}
