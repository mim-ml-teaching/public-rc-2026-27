// The camera is not at the origin: world frame W, camera frame C (x right, y down, z forward) at the camera
// centre C, the translation t (world origin seen from the camera, measured along the camera axes) and the
// rotation R (an arc turning a copy of the world axes onto the camera axes). Turning the camera about its
// centre changes R and t, while C stays. A point P with known world coordinates is carried through the whole
// chain: x_c = R x_w + t, then K gives its pixel (shown on the camera's image plane and in the readout).
// config: { C: [x, y, z], target: [x, y, z], P: [x, y, z], f, ox, oy, width, height }   (world frame, z up)
import { C as COL, h, fmt } from './util.js';
import { THREE, makeStage, label, line, arrow3, viewButtons } from './three-util.js';
import { T } from './gripper.js';
import { axisAngle, toAxisAngle, apply, deg } from './rotmath.js';

const sub = (a, b) => a.map((v, i) => v - b[i]);
const add = (a, b) => a.map((v, i) => v + b[i]);
const scl = (a, s) => a.map(v => v * s);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => scl(a, 1 / Math.hypot(...a));
const hex = c => '#' + c.toString(16).padStart(6, '0');
const AX = [0xff6b6b, 0x5fd38d, 0x5ab0ff];

export function mount(el, cfg) {
  const Cw = cfg.C || [2.4, -3.6, 1.9];
  const target = cfg.target || [3.0, 0.6, 0.5];
  const Pw = cfg.P || [2.0, 0.5, 1.0];
  const f = cfg.f ?? 500, ox = cfg.ox ?? 320, oy = cfg.oy ?? 240;
  const W = cfg.width || 560, H = cfg.height || 340;
  const state = { yaw: 0, comps: true, pw: true };

  const canvas = h('canvas', { width: W, height: H });
  const view3d = { label: '3D view', position: cfg.camera || T(6.5, -1.2, 4.6), target: cfg.lookAt || T(1.2, -1.8, 0.9), fov: 45 };
  const viewTop = { label: 'top view', position: T(1.2, -1.5, 11), target: T(1.2, -1.5 + 0.01, 0), fov: 50 };
  const stage = makeStage(canvas, { position: view3d.position, target: view3d.target });
  const { scene, render } = stage;
  const lab = (group, text, p, color, size = 0.3, s = '') => { const l = label(text, { color, size, sub: s }); l.position.set(...T(...p)); group.add(l); };

  // static: ground grid, world frame
  const grid = new THREE.GridHelper(8, 8, 0x3a3f4b, 0x2a2e36); grid.position.set(...T(0.5, -1, 0)); scene.add(grid);
  ['x', 'y', 'z'].forEach((n, i) => {
    const d = [0, 0, 0]; d[i] = 1.3;
    scene.add(arrow3(T(0, 0, 0), T(...d), AX[i], 0.14));
    lab(scene, n, scl(d, 1.14), hex(AX[i]), 0.42, 'w');
  });
  lab(scene, 'W', [-0.35, -0.3, -0.05], COL.fg, 0.34);
  const dyn = new THREE.Group(); scene.add(dyn);

  // camera rotation: rows = camera axes in world coordinates (as in camera3d)
  function cameraR() {
    const f0 = norm(sub(target, Cw));
    const fwd = apply(axisAngle([0, 0, 1], state.yaw * deg), f0);
    const right = norm(cross(fwd, [0, 0, 1]));
    const down = cross(fwd, right);
    return [right, down, fwd];
  }

  const readout = h('div', { style: 'font-size:0.8em; margin-top:0.2em' });

  function update() {
    dyn.clear();
    const R = cameraR();
    const t = R.map(r => -dot(r, Cw));
    const L = 1.0;

    // camera body: a small frustum along z_c, its image plane matches K (640 × 480 px)
    const d = 0.55, hw = d * ox / f, hh = d * oy / f;
    const corners = [[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]].map(([x, y]) => add(Cw, add(add(scl(R[0], x), scl(R[1], y)), scl(R[2], d))));
    corners.forEach(c => dyn.add(line([T(...Cw), T(...c)], 0x9aa1ae)));
    dyn.add(line([...corners, corners[0]].map(p => T(...p)), 0x9aa1ae));
    const pin = new THREE.Mesh(new THREE.SphereGeometry(0.07, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff }));
    pin.position.set(...T(...Cw)); dyn.add(pin);

    // copy of the world axes at the camera centre (dashed) and the camera axes (solid)
    ['x', 'y', 'z'].forEach((n, i) => {
      const e = [0, 0, 0]; e[i] = L * 0.85;
      dyn.add(line([T(...Cw), T(...add(Cw, e))], AX[i], { dashed: true }));
      dyn.add(arrow3(T(...Cw), T(...add(Cw, scl(R[i], L))), AX[i], 0.14));
      lab(dyn, n, add(Cw, scl(R[i], L * 1.18)), hex(AX[i]), 0.42, 'c');
    });
    lab(dyn, 'C', add(Cw, [0.05, 0.25, -0.38]), COL.purple, 0.36);

    // R: the rotation taking the world axes to the camera axes is Rᵀ (columns = camera axes); draw its arc
    // for the world axis most perpendicular to the rotation axis
    const Q = [R[0][0], R[1][0], R[2][0], R[0][1], R[1][1], R[2][1], R[0][2], R[1][2], R[2][2]];
    const { axis, angle } = toAxisAngle(Q);
    let k = 0, best = -1;
    for (let i = 0; i < 3; i++) { const e = [0, 0, 0]; e[i] = 1; const s = Math.hypot(...cross(axis, e)); if (s > best) { best = s; k = i; } }
    const ek = [0, 0, 0]; ek[k] = 1;
    const r = 0.62, N = 40;
    const arc = [...Array(N + 1)].map((_, j) => add(Cw, scl(apply(axisAngle(axis, angle * j / N), ek), r)));
    dyn.add(line(arc.slice(0, N - 1).map(p => T(...p)), 0xe7e9ee, { width: 2 }));
    dyn.add(arrow3(T(...arc[N - 4]), T(...arc[N]), 0xe7e9ee, 0.14));
    const mid = arc[N >> 1], out = norm(sub(mid, Cw));
    lab(dyn, 'R', add(mid, scl(out, 0.22)), COL.fg, 0.36);

    // t: from the camera centre to the world origin; components along the camera axes
    dyn.add(arrow3(T(...Cw), T(0, 0, 0), 0xf2b134, 0.22));
    lab(dyn, 't', add(scl(Cw, 0.5), [0.25, 0.2, 0.15]), COL.accent, 0.4);
    if (state.comps) {
      let p = Cw;
      [0, 1, 2].forEach(i => {
        const q = add(p, scl(R[i], t[i]));
        dyn.add(line([T(...p), T(...q)], AX[i], { dashed: true }));
        p = q;
      });
    }

    // the point P: world coordinates (dashed along the world axes), its ray and its image
    const Pc = R.map((r, i) => dot(r, Pw) + t[i]);
    const Pm = new THREE.Mesh(new THREE.SphereGeometry(0.09, 16, 12), new THREE.MeshStandardMaterial({ color: 0xff6b6b }));
    Pm.position.set(...T(...Pw)); dyn.add(Pm);
    lab(dyn, 'P', add(Pw, [0.15, 0, 0.28]), COL.red, 0.42);
    if (state.pw) {
      let q = [0, 0, 0];
      [0, 1, 2].forEach(i => { const e = [0, 0, 0]; e[i] = Pw[i]; const r2 = add(q, e); dyn.add(line([T(...q), T(...r2)], AX[i], { dashed: true })); q = r2; });
    }
    dyn.add(line([T(...Cw), T(...Pw)], 0xff6b6b));
    let uv = null;
    if (Pc[2] > 0.05) {
      uv = [f * Pc[0] / Pc[2] + ox, f * Pc[1] / Pc[2] + oy];
      const pi = add(Cw, add(add(scl(R[0], d * Pc[0] / Pc[2]), scl(R[1], d * Pc[1] / Pc[2])), scl(R[2], d)));
      const pm = new THREE.Mesh(new THREE.SphereGeometry(0.045, 12, 10), new THREE.MeshBasicMaterial({ color: 0xff6b6b }));
      pm.position.set(...T(...pi)); dyn.add(pm);
    }

    const m = v => fmt(v, 2);
    const inImage = uv && uv[0] >= 0 && uv[0] <= 2 * ox && uv[1] >= 0 && uv[1] <= 2 * oy;
    readout.innerHTML =
      `<div style="display:flex; gap:1.1em; align-items:center">` +
      `<div>$R = \\begin{bmatrix}${R.map(row => row.map(m).join('&')).join('\\\\')}\\end{bmatrix}$</div>` +
      `<div>$t = -RC = \\begin{bmatrix}${t.map(m).join('\\\\')}\\end{bmatrix}$</div>` +
      `<div>$C = \\begin{bmatrix}${Cw.map(m).join('\\\\')}\\end{bmatrix}$</div></div>` +
      `<div style="display:flex; gap:0.7em; align-items:center; margin-top:0.3em">` +
      `<div>$\\textcolor{${COL.red}}{x_w} = \\begin{bmatrix}${Pw.map(m).join('\\\\')}\\end{bmatrix}$</div><div>→</div>` +
      `<div>$x_c = R\\,x_w + t = \\begin{bmatrix}${Pc.map(m).join('\\\\')}\\end{bmatrix}$</div><div>→</div>` +
      `<div>${uv ? `$\\begin{aligned}u &= f\\,x_c/z_c + o_x = ${fmt(uv[0], 0)}\\\\ v &= f\\,y_c/z_c + o_y = ${fmt(uv[1], 0)}\\end{aligned}$` + (inImage ? '' : ' <span class="small">(outside the image)</span>') : 'behind the camera'}</div></div>` +
      `<div class="small" style="margin-top:0.15em">$K$: $f = ${f}$, $(o_x, o_y) = (${ox}, ${oy})$, image $${2 * ox} \\times ${2 * oy}$</div>`;
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
    render();
  }

  const yaw = h('input', { type: 'range', min: -60, max: 60, step: 1, value: 0 });
  const yawVal = h('span', { class: 'readout', style: 'display:inline-block; width:3.2em; text-align:right' }, '0°');
  yaw.addEventListener('input', () => { state.yaw = +yaw.value; yawVal.textContent = `${state.yaw}°`; update(); });
  const pwBtn = h('button', { class: 'active' }, 'P along the world axes');
  pwBtn.addEventListener('click', () => { state.pw = !state.pw; pwBtn.classList.toggle('active', state.pw); update(); });
  const comps = h('button', { class: 'active' }, 't along the camera axes');
  comps.addEventListener('click', () => { state.comps = !state.comps; comps.classList.toggle('active', state.comps); update(); });

  el.classList.add('widget');
  el.style.maxWidth = W + 'px';
  el.append(canvas,
    h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, viewButtons(stage, [view3d, viewTop]), comps, pwBtn),
    h('div', { class: 'wctl interactive-only', style: 'gap:0.4em' }, h('label', {}, 'turn the camera about C'), yaw, yawVal),
    readout);
  update();
}
