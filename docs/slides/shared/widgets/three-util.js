// three.js helpers shared by 3D widgets.
import * as THREE from 'three';
import { OrbitControls } from '../../vendor/three/controls/OrbitControls.js';
import { isPrint } from './util.js';

export { THREE };

// A scene with a camera, orbit controls and a render-on-demand loop.
export function makeStage(canvas, { fov = 40, position = [4, 3, 5], target = [0, 0, 0], bg = 0x1e2128 } = {}) {
  // Logical (CSS) size comes from the canvas' width/height attributes. Read it BEFORE setPixelRatio, which
  // enlarges the drawing buffer (canvas.width) — re-reading it afterwards double-scaled on 2× (Retina) displays.
  const w = canvas.width, h = canvas.height;
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  renderer.setSize(w, h, true);   // true: also set CSS width/height, so layout size stays w × h
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(bg);
  const camera = new THREE.PerspectiveCamera(fov, w / h, 0.05, 200);
  camera.position.set(...position);
  scene.add(new THREE.AmbientLight(0xffffff, 0.7));
  const dl = new THREE.DirectionalLight(0xffffff, 1.2); dl.position.set(3, 6, 4); scene.add(dl);
  const controls = new OrbitControls(camera, canvas);
  controls.target.set(...target);
  controls.enableDamping = !isPrint();
  controls.update();
  let pending = false;
  const render = () => {
    if (pending) return;
    pending = true;
    requestAnimationFrame(() => { pending = false; if (controls.update()) render(); renderer.render(scene, camera); });
  };
  controls.addEventListener('change', render);
  return { renderer, scene, camera, controls, render };
}

// Buttons that fly the camera to preset views: [{ label, position, target, fov }]
// (a top-down view: position straight above the target, nudged slightly towards +Z so "up" is defined)
export function viewButtons(stage, views) {
  const { camera, controls, render } = stage;
  return views.map(v => {
    const b = document.createElement('button');
    b.textContent = v.label;
    b.addEventListener('click', () => {
      v.onSelect?.();
      const p0 = camera.position.clone(), t0 = controls.target.clone(), f0 = camera.fov;
      const p1 = new THREE.Vector3(...v.position), t1 = new THREE.Vector3(...v.target), f1 = v.fov ?? 40;
      const start = performance.now();
      const step = now => {
        const u = Math.min(1, (now - start) / 900), e = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
        camera.position.lerpVectors(p0, p1, e);
        controls.target.lerpVectors(t0, t1, e);
        camera.fov = f0 + (f1 - f0) * e; camera.updateProjectionMatrix();
        controls.update(); render();
        if (u < 1) requestAnimationFrame(step);
      };
      requestAnimationFrame(step);
    });
    return b;
  });
}

// Text sprite that always faces the camera.
export function label(text, { color = '#e7e9ee', size = 0.28, font = 'italic 64px "Times New Roman", serif' } = {}) {
  const c = document.createElement('canvas');
  const g = c.getContext('2d');
  g.font = font;
  const w = Math.ceil(g.measureText(text).width) + 16;
  c.width = w; c.height = 84;
  g.font = font; g.fillStyle = color; g.textBaseline = 'middle';
  g.fillText(text, 8, 44);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  s.scale.set(size * w / 84, size, 1);
  s.renderOrder = 10;
  return s;
}

export function line(points, color, { dashed = false, width = 1 } = {}) {
  const geom = new THREE.BufferGeometry().setFromPoints(points.map(p => new THREE.Vector3(...p)));
  const mat = dashed
    ? new THREE.LineDashedMaterial({ color, dashSize: 0.12, gapSize: 0.08, linewidth: width })
    : new THREE.LineBasicMaterial({ color, linewidth: width });
  const l = new THREE.Line(geom, mat);
  if (dashed) l.computeLineDistances();
  return l;
}

export function arrow3(from, to, color, headLength = 0.18) {
  const dir = new THREE.Vector3(...to).sub(new THREE.Vector3(...from));
  const len = dir.length();
  return new THREE.ArrowHelper(dir.normalize(), new THREE.Vector3(...from), len, color, Math.min(headLength, len * 0.3), Math.min(headLength, len * 0.3) * 0.55);
}

export function axes(scene, len = 2, names = ['x', 'y', 'z'], colors = [0xff6b6b, 0x5fd38d, 0x5ab0ff]) {
  const dirs = [[len, 0, 0], [0, len, 0], [0, 0, len]];
  dirs.forEach((d, i) => {
    scene.add(arrow3([0, 0, 0], d, colors[i], 0.14));
    const l = label(names[i], { color: '#' + colors[i].toString(16).padStart(6, '0') });
    l.position.set(...d.map(v => v * 1.08));
    scene.add(l);
  });
}
