// Epipolar geometry of a general (not rectified) camera pair: where can the match of a pixel lie?
// Left camera at the origin; the right camera sits about 1 m to the right, slightly higher and behind, turned
// (verging) towards the left one and rolled. Camera frames as in OpenCV (x right, y down, z forward); all 3D math in the left
// camera frame, mapped to three.js by T(...).
// Convention (as on the slides): x_r = R x_l + t (R, t: left → right), E = [t]× R, x_rᵀ E x_l = 0,
// F = K⁻ᵀ E K⁻¹ (same K for both cameras), epipolar line in the right image l_r = F ũ_l.
// A pixel of the left image defines a ray; P slides along it (its depth is unknown to the right camera). The images
// of P for all depths lie on the epipolar line in the right image: it runs from the epipole e_r (the image of the
// left centre, depth → 0) to the image of the ray's point at infinity (depth → ∞).
// 3D view: both cameras with their (virtual) image planes, the baseline, the epipolar plane through both centres and
// P, the epipoles, the epipolar line on the right image plane. Planes are extended (faint) where an epipole falls
// outside the image. Below: the two images (click the left one to choose another pixel) and the numbers.
// config: {
//   width: 600, height: 330     3D view size
//   depth: 2.5                  depth z_l of P (m), slider 0.3 … 10
//   pixel: [280, 200]           the pixel in the left image (640 × 480 image)
//   plane: true                 show the epipolar plane
//   samples: true               faint images of P at other depths
//   camera, target              three.js coordinates of the initial 3D viewpoint (default: above, behind the left camera)
// }
// View buttons: "3D view" and "behind the right camera" (looking along its optical axis: the ray of the pixel, the
// epipolar plane seen edge-on and the epipolar line coincide).
import { C as COL, h, fmt } from './util.js';
import { THREE, makeStage, label, line, viewButtons } from './three-util.js';
import { matmul, transpose, matvec } from './linalg.js';

// left camera frame -> three.js: X = z (forward), Y = -y (up), Z = x  (a proper rotation, as in stereo3d)
const T = (x, y, z) => [z, -y, x];
const IW = 640, IH = 480, F = 500, OX = IW / 2, OY = IH / 2;
const K = [[F, 0, OX], [0, F, OY], [0, 0, 1]];
const Kinv = [[1 / F, 0, -OX / F], [0, 1 / F, -OY / F], [0, 0, 1]];
const S = 0.7;                                   // distance of the drawn image planes from the centres
const deg = Math.PI / 180;

// The right camera: centre c_r and axes (columns of Rc) in the left frame. Yaw 30° towards the left camera,
// pitch 8° down, roll 10°. Then R = Rcᵀ, t = −R c_r.
const Ry = a => [[Math.cos(a), 0, Math.sin(a)], [0, 1, 0], [-Math.sin(a), 0, Math.cos(a)]];
const Rx = a => [[1, 0, 0], [0, Math.cos(a), -Math.sin(a)], [0, Math.sin(a), Math.cos(a)]];
const Rz = a => [[Math.cos(a), -Math.sin(a), 0], [Math.sin(a), Math.cos(a), 0], [0, 0, 1]];
const CR = [1.0, -0.3, -0.6];
const RC = matmul(matmul(Ry(-30 * deg), Rx(-8 * deg)), Rz(10 * deg));
const R = transpose(RC);
const TV = matvec(R, CR).map(v => -v);
const cross = ([a, b, c]) => [[0, -c, b], [c, 0, -a], [-b, a, 0]];
const E = matmul(cross(TV), R);
const FM = (() => { const M = matmul(matmul(transpose(Kinv), E), Kinv); const n = Math.hypot(...M.flat()); return M.map(r => r.map(v => v / n)); })();

const toRight = p => matvec(R, p).map((v, i) => v + TV[i]);           // left frame -> right frame
const rightToLeft = q => matvec(RC, q).map((v, i) => v + CR[i]);       // right frame -> left frame
const proj = x => [F * x[0] / x[2] + OX, F * x[1] / x[2] + OY];
const SAMPLES = [0.4, 0.6, 0.9, 1.4, 2.2, 3.5, 6];

// clip the line a u + b v + c = 0 to the image rectangle
function clipLine([a, b, c]) {
  const pts = [];
  const add = (u, v) => { if (u >= -1e-6 && u <= IW + 1e-6 && v >= -1e-6 && v <= IH + 1e-6) pts.push([u, v]); };
  if (Math.abs(b) > 1e-12) { add(0, -c / b); add(IW, -(c + a * IW) / b); }
  if (Math.abs(a) > 1e-12) { add(-c / a, 0); add(-(c + b * IH) / a, IH); }
  if (pts.length < 2) return null;
  let best = [pts[0], pts[1]], bd = -1;
  for (const p of pts) for (const q of pts) { const d = Math.hypot(p[0] - q[0], p[1] - q[1]); if (d > bd) { bd = d; best = [p, q]; } }
  return best;
}

export function mount(el, cfg) {
  const state = { depth: cfg.depth ?? 2.5, pixel: (cfg.pixel || [280, 200]).slice(), plane: cfg.plane ?? true, samples: cfg.samples ?? true };
  const W = cfg.width || 600, H = cfg.height || 330;

  const canvas = h('canvas', { width: W, height: H });
  const view3d = { label: '3D view', position: cfg.camera || T(0.3, -2.0, -3.2), target: cfg.target || T(0.5, 0.1, 1.5), fov: 34 };
  // behind the right camera, looking along its optical axis: the ray of the left pixel is seen as the epipolar line
  const zr = RC.map(r => r[2]);
  const viewBack = { label: 'behind the right camera', position: T(...CR.map((c, i) => c - 1.7 * zr[i] + (i === 1 ? -0.06 : 0))), target: T(...CR.map((c, i) => c + 3 * zr[i])), fov: 42 };
  const stage = makeStage(canvas, { position: view3d.position, target: view3d.target, fov: view3d.fov });
  const { scene, render } = stage;
  const ground = new THREE.GridHelper(10, 20, 0x3a3f4b, 0x2a2e36); ground.position.set(...T(0.5, 0.9, 2.5)); scene.add(ground);
  const dyn = new THREE.Group(); scene.add(dyn);
  const hex = c => '#' + c.toString(16).padStart(6, '0');
  const lab = (text, p, color, size = 0.13, sub = '') => { const l = label(text, { color, size, sub }); l.position.set(...T(...p)); dyn.add(l); };
  const dot = (p, color, r = 0.02, opacity = 1) => {
    const m = new THREE.Mesh(new THREE.SphereGeometry(r, 14, 10), new THREE.MeshBasicMaterial({ color, transparent: opacity < 1, opacity }));
    m.position.set(...T(...p)); dyn.add(m);
  };
  const poly = (pts, color, opacity) => {
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pts.flatMap(p => T(...p)), 3));
    geo.setIndex(pts.slice(2).flatMap((_, i) => [0, i + 1, i + 2]));
    dyn.add(new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color, transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false })));
  };
  // pixel (u, v) of a camera -> 3D point on its drawn image plane, in the left frame
  const onPlane = (toLeft, u, v) => toLeft([S * (u - OX) / F, S * (v - OY) / F, S]);
  const leftId = q => q;

  // epipoles (pixels) and their 3D points on the planes
  const eR = proj(TV), eL = proj(CR);        // e_r = K t, e_l = K c_r (c_r is behind the left camera: z < 0)

  function drawCamera(toLeft, centre, color, name, extra) {
    const cs = [[0, 0], [IW, 0], [IW, IH], [0, IH]].map(([u, v]) => onPlane(toLeft, u, v));
    poly(cs, 0x9aa1ae, 0.16);
    dyn.add(line([...cs, cs[0]].map(p => T(...p)), color));
    cs.forEach(c => dyn.add(line([T(...centre), T(...c)], 0x4a505c)));
    // extended plane (faint) if a point of interest lies outside the image
    if (extra && (extra[0] < 0 || extra[0] > IW || extra[1] < 0 || extra[1] > IH)) {
      const u0 = Math.min(0, extra[0]) - 30, u1 = Math.max(IW, extra[0]) + 30, v0 = Math.min(0, extra[1]) - 30, v1 = Math.max(IH, extra[1]) + 30;
      const xs = [[u0, v0], [u1, v0], [u1, v1], [u0, v1]].map(([u, v]) => onPlane(toLeft, u, v));
      poly(xs, 0x9aa1ae, 0.06);
      const l = line([...xs, xs[0]].map(p => T(...p)), 0x4a505c, { dashed: true }); dyn.add(l);
    }
    dot(centre, color, 0.035);
    lab(name, [centre[0] - 0.02, centre[1] - 0.16, centre[2]], hex(color), 0.2);
  }

  // 2D images
  const SC = 0.3, mk = () => h('canvas', { width: IW * SC * 2, height: IH * SC * 2, style: `width:${IW * SC}px; height:${IH * SC}px; border:1px solid #3a3f4b; border-radius:4px; display:block` });
  const imL = mk(), imR = mk();
  imL.style.cursor = 'crosshair';
  imL.addEventListener('click', e => {
    const r = imL.getBoundingClientRect();
    state.pixel = [Math.round((e.clientX - r.left) / r.width * IW), Math.round((e.clientY - r.top) / r.height * IH)];
    update();
  });
  const readout = h('div', { style: 'font-size:0.66em; line-height:1.5; width:18em' });

  function frame2d(cv, name, color) {
    const g = cv.getContext('2d'), k = cv.width / IW;
    g.fillStyle = '#0d0f13'; g.fillRect(0, 0, cv.width, cv.height);
    g.strokeStyle = '#2a2e36'; g.lineWidth = 1.5;
    for (let u = 80; u < IW; u += 80) { g.beginPath(); g.moveTo(u * k, 0); g.lineTo(u * k, cv.height); g.stroke(); }
    for (let v = 80; v < IH; v += 80) { g.beginPath(); g.moveTo(0, v * k); g.lineTo(cv.width, v * k); g.stroke(); }
    g.font = '30px Inter, Arial'; g.fillStyle = color; g.fillText(name, 12, 36);
    return [g, k];
  }
  const disc = (g, k, [u, v], r, fill, stroke) => {
    g.beginPath(); g.arc(u * k, v * k, r, 0, 7);
    if (fill) { g.fillStyle = fill; g.fill(); }
    if (stroke) { g.strokeStyle = stroke; g.lineWidth = 3; g.stroke(); }
  };
  const tag = (g, k, [u, v], text, color, dx = 14, dy = -12) => {
    g.font = 'italic 30px "Times New Roman", serif'; g.fillStyle = color;
    g.fillText(text, Math.min(Math.max(u * k + dx, 6), g.canvas.width - 70), Math.min(Math.max(v * k + dy, 30), g.canvas.height - 8));
  };

  function update() {
    dyn.clear();
    const ul = [state.pixel[0], state.pixel[1], 1];
    const dir = matvec(Kinv, ul);                         // ray direction with z = 1
    const P = dir.map(v => v * state.depth);
    const zMax = SAMPLES[SAMPLES.length - 1];
    const Pfar = dir.map(v => v * zMax);
    const O = [0, 0, 0];

    // epipolar line in the right image, through F
    const lr = matvec(FM, ul);
    const seg = clipLine(lr);
    const vInf = proj(matvec(R, dir));                   // image of the ray's point at infinity: K R d

    drawCamera(leftId, O, 0x5ab0ff, 'left', eL);
    drawCamera(rightToLeft, CR, 0x5fd38d, 'right', eR);

    // baseline, extended through both centres to the epipoles
    const eL3 = CR.map(v => v * S / CR[2]);               // baseline ∩ left image plane (z_l = S): behind O_l
    const eR3 = rightToLeft(TV.map(v => v * S / TV[2])); // baseline ∩ right image plane
    dyn.add(line([T(...O), T(...CR)], 0xe7e9ee, { width: 2 }));
    dyn.add(line([T(...O), T(...eL3)], 0xe7e9ee, { dashed: true }));
    lab('baseline', [0.3, 0.12, -0.3], COL.fg, 0.17);
    dot(eL3, 0xb28dff, 0.03); dot(eR3, 0xb28dff, 0.03);
    lab('e', [eL3[0] - 0.02, eL3[1] - 0.14, eL3[2]], COL.purple, 0.24, 'l');
    lab('e', [eR3[0], eR3[1] - 0.14, eR3[2]], COL.purple, 0.24, 'r');

    // the ray of the pixel (depth unknown), samples along it, P
    dyn.add(line([T(...O), T(...Pfar)], 0x5ab0ff, { dashed: true }));
    dyn.add(line([T(...O), T(...P)], 0x5ab0ff, { width: 2 }));
    dyn.add(line([T(...CR), T(...P)], 0x5fd38d, { width: 2 }));
    if (state.samples) for (const z of SAMPLES) {
      const Q = dir.map(v => v * z);
      dot(Q, 0xff6b6b, 0.022, 0.4);
      const q = toRight(Q);
      if (q[2] > 0) dot(rightToLeft(q.map(v => v * S / q[2])), 0xff6b6b, 0.012, 0.5);
    }
    dot(P, 0xff6b6b, 0.045);
    lab('P', [P[0] + 0.12, P[1] - 0.14, P[2]], COL.red, 0.36);
    dot(dir.map(v => v * S), 0xff6b6b, 0.02);            // its pixel on the left plane
    const pr = toRight(P);
    if (pr[2] > 0) dot(rightToLeft(pr.map(v => v * S / pr[2])), 0xff6b6b, 0.02);

    // epipolar line drawn on the right image plane, and the epipolar plane
    if (seg) dyn.add(line(seg.map(([u, v]) => T(...onPlane(rightToLeft, u, v))), 0xf2b134, { width: 2 }));
    if (state.plane) {
      poly([eL3, CR, Pfar], 0xb28dff, 0.2);
      lab('epipolar plane', [Pfar[0] * 0.55 + 0.45, Pfar[1] * 0.55 - 0.1, Pfar[2] * 0.55], COL.purple, 0.28);
    }
    render();

    // ---- left image
    {
      const [g, k] = frame2d(imL, 'left image', COL.blue);
      disc(g, k, state.pixel, 9, COL.red);
      tag(g, k, state.pixel, 'ũₗ', COL.red);
      g.font = '24px Inter, Arial'; g.fillStyle = COL.dim; g.fillText('click: choose a pixel', 12, imL.height - 12);
    }
    // ---- right image
    {
      const [g, k] = frame2d(imR, 'right image', COL.green);
      if (seg) {
        g.strokeStyle = COL.accent; g.lineWidth = 3.5;
        g.beginPath(); g.moveTo(seg[0][0] * k, seg[0][1] * k); g.lineTo(seg[1][0] * k, seg[1][1] * k); g.stroke();
      }
      if (state.samples) for (const z of SAMPLES) {
        const q = toRight(dir.map(v => v * z));
        if (q[2] > 0) disc(g, k, proj(q), 6, 'rgba(255,107,107,0.45)');
      }
      if (vInf[0] >= 0 && vInf[0] <= IW && vInf[1] >= 0 && vInf[1] <= IH && matvec(R, dir)[2] > 0) { disc(g, k, vInf, 8, null, COL.red); tag(g, k, vInf, '∞', COL.red, -34, 30); }
      disc(g, k, eR, 9, COL.purple); tag(g, k, eR, 'eᵣ', COL.purple, 14, 30);
      if (pr[2] > 0) disc(g, k, proj(pr), 9, COL.red);
    }

    const [a, b, c] = lr.map(v => v / Math.hypot(lr[0], lr[1]));
    const ur = proj(pr);
    const res = [ur[0], ur[1], 1].reduce((s, x, i) => s + x * lr[i], 0) / Math.hypot(lr[0], lr[1]);
    const sg = (x, d) => (x < 0 ? '- ' : '+ ') + fmt(Math.abs(x), d);
    readout.innerHTML =
      `<div>$\\tilde{\\mathbf u}_l = (${state.pixel[0]},\\ ${state.pixel[1]},\\ 1)$</div>` +
      `<div>epipolar line $\\mathbf l_r = F\\tilde{\\mathbf u}_l$:</div>` +
      `<div style="color:${COL.accent}">$${fmt(a, 3)}\\,u ${sg(b, 3)}\\,v ${sg(c, 1)} = 0$</div>` +
      `<div style="margin-top:0.3em">$P$ in the right image:</div><div>$\\tilde{\\mathbf u}_r = (${fmt(ur[0], 1)},\\ ${fmt(ur[1], 1)},\\ 1)$</div>` +
      `<div>$\\tilde{\\mathbf u}_r^\\top F\\,\\tilde{\\mathbf u}_l = ${fmt(Math.abs(res) < 1e-6 ? 0 : res, 4)}$ for every depth</div>` +
      `<div class="dim" style="margin-top:0.3em">epipole $e_r = (${fmt(eR[0], 0)},\\ ${fmt(eR[1], 0)})$</div>`;
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
  }

  const depthVal = h('span', { class: 'readout', style: 'display:inline-block; width:3.6em; text-align:right' });
  const depthSlider = h('input', { type: 'range', min: 0.3, max: 10, step: 0.01, value: state.depth, style: 'width:8em' });
  const setDepthText = () => { depthVal.textContent = fmt(state.depth, 2) + ' m'; };
  depthSlider.addEventListener('input', () => { state.depth = parseFloat(depthSlider.value); setDepthText(); update(); });
  setDepthText();
  const toggle = (key, text) => {
    const b = h('button', { class: state[key] ? 'active' : '', onclick: () => { state[key] = !state[key]; b.classList.toggle('active', state[key]); update(); } }, text);
    return b;
  };

  el.classList.add('widget');
  el.append(canvas,
    h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, viewButtons(stage, [view3d, viewBack]), toggle('plane', 'epipolar plane'), toggle('samples', 'other depths')),
    h('div', { style: 'display:flex; gap:0.8em; margin-top:0.4em; align-items:flex-start' },
      h('div', { style: 'display:flex; gap:0.4em; flex:none' }, imL, imR),
      h('div', { style: 'display:flex; flex-direction:column; gap:0.3em; min-width:0' },
        h('div', { class: 'wctl interactive-only', style: 'margin-top:0; gap:0.4em; align-items:center' }, h('label', {}, 'depth of P'), depthSlider, depthVal),
        readout)));
  update();
}
