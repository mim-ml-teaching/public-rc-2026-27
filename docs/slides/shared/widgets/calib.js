// Camera calibration by DLT: a checkerboard cube seen by a camera with known (hidden) K, R, t.
// Detected corners = exact projections + Gaussian noise. From the 3D–2D correspondences we build A (2 rows per
// point), take p = eigenvector of AᵀA with the smallest eigenvalue (Hartley-normalised), reshape it to P and split
// P = K [R | t] by an RQ decomposition. The panel compares the estimate with the truth and shows the eigenvalues
// of AᵀA: one is ≈ 0 for a 3D target; for points on a single plane several are ≈ 0 (DLT is degenerate).
//
// World frame: z up, cube [0, 6]³ in units of one square; the camera sees the faces y = 0, x = 0 and z = 6.
// config: { width (display px of the 640×480 image, default 600), noise (px, default 1), points: 6 | 12 | "all",
//           planar: false, seed }
import { C, h, fmt } from './util.js';
import { matmul, matvec, symEig, rq3, det3, rng } from './linalg.js';

const IW = 640, IH = 480, N = 6;
const sub = (a, b) => a.map((v, i) => v - b[i]);
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = a => { const l = Math.hypot(...a); return a.map(v => v / l); };

// True camera: looks at the cube corner region from the front-left, above.
const Ktrue = [[800, 0, 330], [0, 800, 235], [0, 0, 1]];
const Cw = [-8.5, -11, 10.5];
const fwd = norm(sub([3, 3, 2.6], Cw)), right = norm(cross(fwd, [0, 0, 1])), down = cross(fwd, right);
const Rtrue = [right, down, fwd];
const ttrue = matvec(Rtrue, Cw).map(v => -v);
const Ptrue = matmul(Ktrue, Rtrue.map((r, i) => [...r, ttrue[i]]));
const proj = (P, X) => { const x = matvec(P, [...X, 1]); return [x[0] / x[2], x[1] / x[2]]; };

// Faces: origin, two in-plane unit directions, outward normal.
const FACES = [
  { name: 'front', o: [0, 0, 0], a: [1, 0, 0], b: [0, 0, 1], n: [0, -1, 0], shade: 1.0 },
  { name: 'left', o: [0, 0, 0], a: [0, 1, 0], b: [0, 0, 1], n: [-1, 0, 0], shade: 0.78 },
  { name: 'top', o: [0, 0, N], a: [1, 0, 0], b: [0, 1, 0], n: [0, 0, 1], shade: 1.12 },
];
const at = (f, i, j) => f.o.map((v, k) => v + i * f.a[k] + j * f.b[k]);
const CORNERS = FACES.map(f => { const pts = []; for (let i = 1; i < N; i++) for (let j = 1; j < N; j++) pts.push(at(f, i, j)); return pts; });

function pickPoints(count, planar) {
  const spread = [0, 24, 12, 4, 20, 2, 22, 10, 14, 6, 18, 8]; // indices into a face's 5×5 corner grid
  if (planar) return count === 'all' ? CORNERS[0] : spread.slice(0, count).map(i => CORNERS[0][i]);
  if (count === 'all') return CORNERS.flat();
  return CORNERS.flatMap(face => spread.slice(0, count / 3).map(i => face[i]));
}

// Normalised DLT. Returns P (3×4), the eigenvalues of ÃᵀÃ and the decomposition.
function dlt(X, x) {
  const n = X.length;
  const mean = (pts, d) => [...Array(d)].map((_, k) => pts.reduce((s, p) => s + p[k], 0) / n);
  const mx = mean(x, 2), mX = mean(X, 3);
  const sx = Math.SQRT2 / (x.reduce((s, p) => s + Math.hypot(p[0] - mx[0], p[1] - mx[1]), 0) / n);
  const sX = Math.sqrt(3) / (X.reduce((s, p) => s + Math.hypot(p[0] - mX[0], p[1] - mX[1], p[2] - mX[2]), 0) / n);
  const xn = x.map(p => [(p[0] - mx[0]) * sx, (p[1] - mx[1]) * sx]);
  const Xn = X.map(p => [(p[0] - mX[0]) * sX, (p[1] - mX[1]) * sX, (p[2] - mX[2]) * sX]);
  const AtA = [...Array(12)].map(() => new Array(12).fill(0));
  const addRow = r => { for (let i = 0; i < 12; i++) for (let j = 0; j < 12; j++) AtA[i][j] += r[i] * r[j]; };
  for (let k = 0; k < n; k++) {
    const [X1, X2, X3] = Xn[k], [u, v] = xn[k], Xh = [X1, X2, X3, 1];
    addRow([...Xh, 0, 0, 0, 0, ...Xh.map(a => -u * a)]);
    addRow([0, 0, 0, 0, ...Xh, ...Xh.map(a => -v * a)]);
  }
  const { vals, vecs } = symEig(AtA);
  const p = vecs[0];
  const Pn = [p.slice(0, 4), p.slice(4, 8), p.slice(8, 12)];
  // undo the normalisation: P = T⁻¹ Pn U
  const Tinv = [[1 / sx, 0, mx[0]], [0, 1 / sx, mx[1]], [0, 0, 1]];
  const U = [[sX, 0, 0, -sX * mX[0]], [0, sX, 0, -sX * mX[1]], [0, 0, sX, -sX * mX[2]], [0, 0, 0, 1]];
  let P = matmul(matmul(Tinv, Pn), U);
  // fix the sign so that the points are in front of the camera, then decompose
  if (det3(P.map(r => r.slice(0, 3))) < 0) P = P.map(r => r.map(v => -v));
  const { K: K0, R: R0 } = rq3(P.map(r => r.slice(0, 3)));
  const s = K0[2][2];
  const K = K0.map(r => r.map(v => v / s));
  const t = matvec([[1 / K[0][0], -K[0][1] / (K[0][0] * K[1][1]), (K[0][1] * K[1][2] - K[1][1] * K[0][2]) / (K[0][0] * K[1][1])],
    [0, 1 / K[1][1], -K[1][2] / K[1][1]], [0, 0, 1]], P.map(r => r[3] / s));
  const Cest = [0, 1, 2].map(j => -(R0[0][j] * t[0] + R0[1][j] * t[1] + R0[2][j] * t[2]));
  return { P, vals, K, R: R0, C: Cest };
}

export function mount(el, cfg) {
  const DW = cfg.width || 600;
  const state = { noise: cfg.noise ?? 1, points: cfg.points ?? 'all', planar: !!cfg.planar, seed: cfg.seed ?? 7 };
  const canvas = h('canvas', { width: IW, height: IH, style: `width:${DW}px; height:${DW * IH / IW}px` });
  const g = canvas.getContext('2d');
  const spec = h('canvas', { width: 300, height: 150 });
  const gs = spec.getContext('2d');
  const table = h('div', { style: 'font-size:0.95em' });
  const warn = h('div', { style: 'color:' + C.red });

  function drawCube() {
    g.fillStyle = '#0d0f13'; g.fillRect(0, 0, IW, IH);
    for (const f of FACES) {
      for (let i = 0; i < N; i++) for (let j = 0; j < N; j++) {
        const Q = [at(f, i, j), at(f, i + 1, j), at(f, i + 1, j + 1), at(f, i, j + 1)].map(X => proj(Ptrue, X));
        const white = (i + j) % 2 === 0, k = f.shade;
        const c = white ? [214, 217, 224] : [58, 62, 72];
        g.fillStyle = `rgb(${c.map(v => Math.min(255, Math.round(v * k))).join(',')})`;
        g.beginPath(); Q.forEach(([u, v], m) => (m ? g.lineTo(u, v) : g.moveTo(u, v))); g.closePath(); g.fill();
      }
    }
  }

  function draw() {
    drawCube();
    const r = rng(state.seed);
    const X = pickPoints(state.points, state.planar);
    const x = X.map(P => { const [u, v] = proj(Ptrue, P); return [u + state.noise * r.normal(), v + state.noise * r.normal()]; });
    const est = dlt(X, x);
    // detected corners (accent circles) and reprojections of the estimate (blue crosses)
    let sse = 0;
    X.forEach((P, k) => {
      const [u, v] = x[k], [ue, ve] = proj(est.P, P);
      sse += (u - ue) ** 2 + (v - ve) ** 2;
      g.strokeStyle = C.accent; g.lineWidth = 2.5; g.beginPath(); g.arc(u, v, 6, 0, 7); g.stroke();
      if (Number.isFinite(ue) && Math.abs(ue) < 5 * IW && Math.abs(ve) < 5 * IH) {
        g.strokeStyle = C.blue; g.lineWidth = 2; g.beginPath();
        g.moveTo(ue - 6, ve - 6); g.lineTo(ue + 6, ve + 6); g.moveTo(ue - 6, ve + 6); g.lineTo(ue + 6, ve - 6); g.stroke();
      }
    });
    // world frame at the cube corner
    const o = proj(Ptrue, [0, 0, 0]);
    [[[2.2, 0, 0], C.red, 'x'], [[0, 2.2, 0], C.green, 'y'], [[0, 0, 2.2], C.blue, 'z']].forEach(([d, col]) => {
      const q = proj(Ptrue, d);
      g.strokeStyle = col; g.lineWidth = 4; g.beginPath(); g.moveTo(...o); g.lineTo(...q); g.stroke();
    });
    g.font = '26px Inter, Arial'; g.fillStyle = C.accent; g.fillText('○ detected corner', 14, 34);
    g.fillStyle = C.blue; g.fillText('× reprojected with estimated P', 14, 66);
    g.strokeStyle = '#5a606c'; g.lineWidth = 2; g.strokeRect(1, 1, IW - 2, IH - 2);

    // readout: truth vs estimate
    const K = est.K, rms = Math.sqrt(sse / X.length), dC = Math.hypot(...sub(est.C, Cw));
    const row = (name, t, e, d = 1) => `<tr><td>${name}</td><td>${t}</td><td style="color:${C.accent}">${Number.isFinite(e) ? fmt(e, d) : '—'}</td></tr>`;
    table.innerHTML = `<table style="font-size:1em; margin:0"><tr><th></th><th>true</th><th>estimate</th></tr>` +
      row('$f_x$', 800, K[0][0], 0) + row('$f_y$', 800, K[1][1], 0) + row('$o_x$', 330, K[0][2], 0) + row('$o_y$', 235, K[1][2], 0) +
      row('skew $s$', 0, K[0][1], 1) + row('camera pos. error', 0, dC, 2) + `</table>` +
      `<div style="margin-top:0.3em">${X.length} points · RMS reprojection error <b>${Number.isFinite(rms) ? fmt(rms, 2) + ' px' : '—'}</b></div>`;
    window.renderMathInElement?.(table, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
    const vals = est.vals.map(v => Math.max(v, 1e-13));
    warn.textContent = state.planar ? 'All points on one plane: several eigenvalues ≈ 0 — the solution is not unique, the estimate is meaningless.'
      : (X.length < 6 ? 'Fewer than 6 points: not enough equations.' : '');
    drawSpectrum(vals);
  }

  function drawSpectrum(vals) {
    const W = spec.width, H = spec.height, pad = 22;
    gs.fillStyle = C.bg2; gs.fillRect(0, 0, W, H);
    const lo = -12, hi = Math.log10(vals[11]) + 0.5;
    const y = v => Math.min(H - pad - 4, H - pad - (Math.log10(v) - lo) / (hi - lo) * (H - pad - 8));
    const bw = (W - 20) / 12;
    vals.forEach((v, i) => {
      gs.fillStyle = i === 0 ? C.accent : (v / vals[11] < 1e-7 ? C.red : C.dim);
      gs.fillRect(12 + i * bw + 2, y(v), bw - 4, H - pad - y(v));
    });
    gs.fillStyle = C.dim; gs.font = '17px Inter, Arial';
    gs.fillText('eigenvalues of AᵀA (log scale)', 10, H - 4);
  }

  // ---- controls ----
  const noise = h('input', { type: 'range', min: 0, max: 4, step: 0.1, value: state.noise });
  const noiseVal = h('span', { class: 'readout', style: 'display:inline-block; width:4.2em' }, `${fmt(state.noise, 1)} px`);
  noise.addEventListener('input', () => { state.noise = +noise.value; noiseVal.textContent = `${fmt(state.noise, 1)} px`; draw(); });
  const group = (opts, key) => {
    const bs = opts.map(([v, label]) => {
      const b = h('button', { onclick: () => { state[key] = v; bs.forEach(x => x.classList.toggle('active', x === b)); draw(); } }, label);
      b.classList.toggle('active', state[key] === v);
      return b;
    });
    return h('div', { style: 'display:flex; gap:0.3em; flex-wrap:wrap' }, bs);
  };
  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${DW}px; flex:none` }, canvas,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.4em 0.8em' },
        h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', {}, 'noise σ'), noise, noiseVal),
        h('div', { style: 'display:flex; align-items:center; gap:0.4em' }, h('label', {}, 'points'), group([[6, '6'], [12, '12'], ['all', 'all']], 'points')),
        group([[false, '3 faces'], [true, 'one face only']], 'planar'),
        h('button', { onclick: () => { state.seed++; draw(); } }, 'new noise'))),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.35em; width:15em; flex:none' }, table, spec, warn)));
  draw();
}
