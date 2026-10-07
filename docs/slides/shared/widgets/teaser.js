// Looping teaser animations for the "Today we mathematically model" slide.
//   mode "grasp": a gripper moves (translation + rotation) to grasp a box and lift it.
//   mode "pixel": a camera watches a moving 3D point; its ray and pixel follow; then the point
//                 slides along its ray and the pixel does not move (depth is lost).
// config: { mode, width, height }
import { C, h, isPrint } from './util.js';
import { THREE, makeStage, label, line } from './three-util.js';
import { T, makeGripper, quatFromMath } from './gripper.js';

const ease = u => (u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2);
const lerp3 = (a, b, t) => a.map((v, i) => v + (b[i] - v) * t);
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => { const l = Math.hypot(...a); return a.map(v => v / l); };

function table(scene) {
  const top = new THREE.Mesh(new THREE.BoxGeometry(...T(3.2, 2.2, 0.08).map(Math.abs)),
    new THREE.MeshStandardMaterial({ color: 0x3a3f4b, roughness: 0.9 }));
  top.position.set(...T(0, 0, -0.04)); scene.add(top);
  const grid = new THREE.GridHelper(3.2, 16, 0x4a505c, 0x30343d);
  grid.position.set(0, 0.002, 0); grid.scale.set(1, 1, 2.2 / 3.2); scene.add(grid);
}

// rotation (math frame) of a gripper pointing down (tool z = −world z) with tool x at angle `yaw`
function downR(yaw) {
  const x = [Math.cos(yaw), Math.sin(yaw), 0], z = [0, 0, -1], y = cross(z, x);
  return [x[0], y[0], z[0], x[1], y[1], z[1], x[2], y[2], z[2]];
}

function loop(dur, step, at, canvas) {
  if (at !== undefined) { step(at); return; }
  if (isPrint()) { step(0.62); return; }
  const t0 = performance.now();
  const frame = now => { if (canvas && !canvas.isConnected) return; step((Math.max(0, now - t0) / dur) % 1); requestAnimationFrame(frame); };
  requestAnimationFrame(frame);
}

export function mount(el, cfg) {
  const W = cfg.width || 520, H = cfg.height || 330;
  const canvas = h('canvas', { width: W, height: H });
  el.classList.add('widget');
  el.style.display = 'inline-block';
  el.append(canvas);
  (cfg.mode === 'pixel' ? pixel : grasp)(canvas, el, cfg);
}

// ---------------------------------------------------------------------------
function grasp(canvas, el, cfg) {
  const { scene, render, controls } = makeStage(canvas, { position: T(2.3, -2.9, 2.0), target: T(-0.15, 0.05, 0.55) });
  controls.enableZoom = false;
  table(scene);
  const S = 0.5;                                     // gripper scale
  const objW = 0.2, objD = 0.26, objH = 0.3, yaw = 0.6;
  const obj = new THREE.Mesh(new THREE.BoxGeometry(...T(objD, objW, objH).map(Math.abs)),
    new THREE.MeshStandardMaterial({ color: 0xf2b134, roughness: 0.6 }));
  obj.rotation.y = yaw; // about three's Y = math z
  scene.add(obj);
  const objPos = [0.35, 0.15, objH / 2];
  const grip = makeGripper(S); scene.add(grip);
  grip.matrixAutoUpdate = true;

  // keyframes: [position of tool origin, rotation, opening]
  const above = 0.22 * S;                            // pads are 0.22·S below the tool origin
  const open = 0.36, closed = objW / (2 * S) + 0.08;
  const Rstart = [0.36, -0.8, 0.48, 0.93, 0.31, -0.19, 0, 0.51, 0.86]; // some tilted start pose
  const K = [
    { p: [-0.9, -0.5, 1.1], R: Rstart, o: open },
    { p: [objPos[0], objPos[1], objPos[2] + above + 0.45], R: downR(yaw + Math.PI / 2), o: open },
    { p: [objPos[0], objPos[1], objPos[2] + above], R: downR(yaw + Math.PI / 2), o: open },
    { p: [objPos[0], objPos[1], objPos[2] + above], R: downR(yaw + Math.PI / 2), o: closed },
    { p: [objPos[0], objPos[1], objPos[2] + above + 0.5], R: downR(yaw + Math.PI / 2), o: closed },
  ];
  const times = [0, 0.38, 0.55, 0.65, 0.85, 1.0]; // last segment: hold, then restart
  const Q = K.map(k => quatFromMath(k.R));

  // dashed path of the tool origin
  const pathPts = [];
  for (let i = 0; i < K.length - 1; i++) for (let s = 0; s <= 1; s += 0.05) pathPts.push(T(...lerp3(K[i].p, K[i + 1].p, ease(s))));
  scene.add(line(pathPts, 0xb28dff, { dashed: true }));
  const lab = label('pose = rotation R + translation t', { color: C.dim, size: 0.16, font: '44px Inter, Arial' });
  lab.position.set(...T(-0.9, -0.5, 1.45)); scene.add(lab);

  const q = new THREE.Quaternion();
  loop(6000, u => {
    let i = times.findIndex((t, j) => u >= t && u < times[j + 1]);
    if (i >= K.length - 1) i = K.length - 2;
    const s = Math.min(1, (u - times[i]) / (times[i + 1] - times[i])), e = ease(s);
    const a = K[i], b = K[Math.min(i + 1, K.length - 1)];
    const p = u >= times[K.length - 1] ? K[K.length - 1].p : lerp3(a.p, b.p, e);
    q.slerpQuaternions(Q[i], Q[Math.min(i + 1, K.length - 1)], u >= times[K.length - 1] ? 1 : e);
    grip.position.set(...T(...p)); grip.quaternion.copy(q);
    grip.setOpening(u >= times[K.length - 1] ? closed : a.o + (b.o - a.o) * e);
    const lift = i >= 3 ? p[2] - (objPos[2] + above) : 0;  // the box rides along after closing
    obj.position.set(...T(objPos[0], objPos[1], objPos[2] + lift));
    render();
  }, cfg.at, canvas);
}

// ---------------------------------------------------------------------------
function pixel(canvas, el, cfg) {
  const { scene, render, controls } = makeStage(canvas, { position: T(3.4, -3.0, 2.4), target: T(0.4, 0.2, 0.5) });
  controls.enableZoom = false;
  table(scene);
  // camera (math frame): centre C, looking at the table
  const Cw = [-1.6, -1.1, 1.3], look = [0.4, 0.4, 0.2];
  const fwd = norm(sub(look, Cw)), right = norm(cross(fwd, [0, 0, 1])), down = cross(fwd, right);
  const f = 0.55, IW = 0.64, IH = 0.48;              // virtual image plane at distance f (world units)
  const toCam = p => { const d = sub(p, Cw); return [dot(right, d), dot(down, d), dot(fwd, d)]; };
  const fromCam = ([x, y, z]) => Cw.map((c, i) => c + right[i] * x + down[i] * y + fwd[i] * z);
  // camera body and image plane
  const corners = [[-IW / 2, -IH / 2], [IW / 2, -IH / 2], [IW / 2, IH / 2], [-IW / 2, IH / 2]].map(([x, y]) => fromCam([x, y, f]));
  corners.forEach(c => scene.add(line([T(...Cw), T(...c)], 0x9aa1ae)));
  scene.add(line([...corners, corners[0]].map(p => T(...p)), 0x9aa1ae));
  const pg = new THREE.BufferGeometry();
  pg.setAttribute('position', new THREE.Float32BufferAttribute(corners.flatMap(p => T(...p)), 3));
  pg.setIndex([0, 1, 2, 0, 2, 3]);
  scene.add(new THREE.Mesh(pg, new THREE.MeshBasicMaterial({ color: 0x9aa1ae, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false })));
  const pin = new THREE.Mesh(new THREE.SphereGeometry(0.04, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff }));
  pin.position.set(...T(...Cw)); scene.add(pin);
  const lc = label('camera', { color: C.dim, size: 0.16, font: '44px Inter, Arial' }); lc.position.set(...T(Cw[0], Cw[1], Cw[2] + 0.35)); scene.add(lc);

  const P = new THREE.Mesh(new THREE.SphereGeometry(0.06, 16, 12), new THREE.MeshStandardMaterial({ color: 0xff6b6b }));
  const pix = new THREE.Mesh(new THREE.SphereGeometry(0.03, 12, 10), new THREE.MeshBasicMaterial({ color: 0xf2b134 }));
  scene.add(P, pix);
  const dyn = new THREE.Group(); scene.add(dyn);

  // inset: the recorded image with the pixel and its trail
  const inset = h('canvas', { width: 192, height: 144, style: 'position:absolute; right:8px; top:8px; border:1px solid #5a606c; border-radius:4px; background:#0d0f13' });
  el.style.position = 'relative'; el.append(inset);
  const gi = inset.getContext('2d');
  const trail = [];
  // trajectory of the point in 3D (green; the ray and the pixel are yellow)
  const path3 = s => [0.4 + 0.75 * Math.cos(2 * Math.PI * s), 0.3 + 0.5 * Math.sin(2 * Math.PI * s), 0.35 + 0.25 * Math.sin(4 * Math.PI * s)];
  scene.add(line(Array.from({ length: 121 }, (_, i) => T(...path3(i / 120))), 0x5fd38d, { dashed: true }));
  // ...plus the stretch of the ray the point slides along (k from 0.55 to 1.45, see below)
  const b0 = path3(0), along = k => T(...Cw.map((c, i) => c + (b0[i] - c) * k));
  const raySeg = line([along(0.55), along(1.45)], 0x5fd38d, { dashed: true });
  raySeg.material.depthTest = false; raySeg.renderOrder = 5;   // drawn over the yellow ray it lies on
  scene.add(raySeg);

  loop(7000, u => {
    // 0–0.6: the point moves on the table; 0.6–1: it slides along its ray (pixel fixed)
    let Pw;
    const path = s => [0.4 + 0.75 * Math.cos(2 * Math.PI * s), 0.3 + 0.5 * Math.sin(2 * Math.PI * s), 0.35 + 0.25 * Math.sin(4 * Math.PI * s)];
    if (u < 0.6) Pw = path(u / 0.6);
    else {
      // scale the point about the camera centre: farther, then nearer, back to the start
      const base = path(0), k = 1 + 0.45 * Math.sin(2 * Math.PI * (u - 0.6) / 0.4);
      Pw = Cw.map((c, i) => c + (base[i] - c) * k);
    }
    const pc = toCam(Pw), img = [f * pc[0] / pc[2], f * pc[1] / pc[2]];
    P.position.set(...T(...Pw));
    pix.position.set(...T(...fromCam([img[0], img[1], f])));
    dyn.clear();
    dyn.add(line([T(...Cw), T(...Pw)], 0xf2b134));
    if (u < 0.01) trail.length = 0;
    if (u < 0.6) trail.push(img);
    // inset drawing
    const ux = x => (x / IW + 0.5) * inset.width, vy = y => (y / IH + 0.5) * inset.height;
    gi.fillStyle = '#0d0f13'; gi.fillRect(0, 0, inset.width, inset.height);
    gi.strokeStyle = 'rgba(95,211,141,0.7)'; gi.lineWidth = 1.5; gi.setLineDash([4, 3]); gi.beginPath();
    trail.forEach(([x, y], i) => (i ? gi.lineTo(ux(x), vy(y)) : gi.moveTo(ux(x), vy(y)))); gi.stroke(); gi.setLineDash([]);
    gi.fillStyle = C.accent; gi.beginPath(); gi.arc(ux(img[0]), vy(img[1]), 4, 0, 7); gi.fill();
    gi.fillStyle = C.dim; gi.font = '11px Inter, Arial';
    gi.fillText(u < 0.6 ? 'image: pixel follows the point' : 'point moves along its ray: same pixel', 6, inset.height - 8);
    render();
  }, cfg.at, canvas);
}
