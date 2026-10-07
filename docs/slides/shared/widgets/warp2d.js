// 2D transformation playground: an image warped by a 3x3 matrix (backward warping, per pixel).
// config: {
//   presets: ["identity","rotate",...]   buttons to show (default: all)
//   preset: "skew"                       initial preset
//   origin: "topleft" | "center"         where the image sits relative to the origin
//   editable: true                       show editable matrix entries
//   linearOnly: false                    show only the 2x2 part (hide 3rd row/column)
//   t: 1                                 initial morph parameter (0 = identity, 1 = full transform)
//   tasks: [{ title, goal: [9] | "corners": [[x,y] x4] | "involution" }]
//                                        exercise mode: show the goal, check the student's matrix
// }
import { C, h, isPrint, I3, inv3, mul3, apply3, lerp9, fmt, testImage, arrow } from './util.js';

const deg = Math.PI / 180;
const PRESETS = {
  identity: { label: 'identity', f: () => I3() },
  flip: { label: 'swap x↔y', lerp: [0, 1, 0, 1, 0, 0, 0, 0, 1] },
  mirror: { label: 'mirror', lerp: [-1, 0, 0, 0, 1, 0, 0, 0, 1] },
  scale: { label: 'scale', f: t => [1 + 0.5 * t, 0, 0, 0, 1 - 0.35 * t, 0, 0, 0, 1] },
  skew: { label: 'skew', f: t => [1, 0.5 * t, 0, 0, 1, 0, 0, 0, 1] },
  rotate: { label: 'rotate 30°', f: t => rot(30 * deg * t) },
  translate: { label: 'translate', f: t => [1, 0, 110 * t, 0, 1, 70 * t, 0, 0, 1] },
  rigid: { label: 'rotate + translate', f: t => { const r = rot(-25 * deg * t); r[2] = 120 * t; r[5] = 60 * t; return r; } },
  similarity: { label: 'similarity', f: t => { const r = rot(20 * deg * t).map((x, i) => (i < 6 && i % 3 < 2 ? x * (1 - 0.3 * t) : x)); r[2] = 60 * t; r[5] = 30 * t; return r; } },
  affine: { label: 'affine', lerp: [0.9, 0.45, 40, -0.25, 0.8, 60, 0, 0, 1] },
  projective: { label: 'projective', lerp: [0.8, 0.1, 60, -0.05, 0.75, 70, -0.0012, 0.0006, 1] },
};

// Solve for H (h33 = 1) mapping 4 points src[i] -> dst[i]: 8x8 linear system, Gaussian elimination.
export function homographyFrom4(srcPts, dstPts) {
  const A = [], b = [];
  for (let i = 0; i < 4; i++) {
    const [x, y] = srcPts[i], [u, v] = dstPts[i];
    A.push([x, y, 1, 0, 0, 0, -u * x, -u * y]); b.push(u);
    A.push([0, 0, 0, x, y, 1, -v * x, -v * y]); b.push(v);
  }
  for (let c = 0; c < 8; c++) {
    let piv = c;
    for (let r = c + 1; r < 8; r++) if (Math.abs(A[r][c]) > Math.abs(A[piv][c])) piv = r;
    [A[c], A[piv]] = [A[piv], A[c]]; [b[c], b[piv]] = [b[piv], b[c]];
    for (let r = 0; r < 8; r++) if (r !== c) {
      const f = A[r][c] / A[c][c];
      for (let k = c; k < 8; k++) A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  return [...b.map((v, i) => v / A[i][i]), 1];
}

function rot(a) { const c = Math.cos(a), s = Math.sin(a); return [c, -s, 0, s, c, 0, 0, 0, 1]; }

function presetMatrix(name, t, custom) {
  if (name === 'custom') return lerp9(I3(), custom, t);
  const p = PRESETS[name];
  return p.f ? p.f(t) : lerp9(I3(), p.lerp, t);
}

function classify(m) {
  const eps = 1e-6;
  if (Math.abs(m[6]) > eps || Math.abs(m[7]) > eps) return { name: 'projective (homography)', color: C.purple };
  const [a, b, c, d] = [m[0], m[1], m[3], m[4]];
  const det = a * d - b * c;
  const ata00 = a * a + c * c, ata11 = b * b + d * d, ata01 = a * b + c * d;
  const noT = Math.abs(m[2]) < eps && Math.abs(m[5]) < eps;
  let name;
  if (Math.abs(ata01) < 1e-3 && Math.abs(ata00 - ata11) < 1e-3) {
    if (Math.abs(ata00 - 1) < 1e-3) name = det > 0 ? 'rigid (rotation + translation)' : 'isometry with reflection';
    else name = det > 0 ? 'similarity' : 'similarity with reflection';
  } else name = 'affine';
  if (noT) name += ' — origin fixed (linear)';
  return { name, color: name.startsWith('rigid') ? C.green : C.accent };
}

export function mount(el, cfg) {
  const presetNames = cfg.presets || Object.keys(PRESETS);
  const origin = cfg.origin || 'center';
  const editable = cfg.editable !== false;
  const linearOnly = !!cfg.linearOnly;
  const CW = cfg.width || 640, CH = cfg.height || 440;
  const SW = 320, SH = 240;
  // world region shown on the canvas, and where the source image sits in world coordinates
  const view = origin === 'topleft' ? { x0: -110, y0: -70 } : { x0: -CW / 2, y0: -CH / 2 };
  const src = origin === 'topleft' ? { ox: 0, oy: 0 } : { ox: -SW / 2, oy: -SH / 2 };

  const img = testImage(SW, SH);
  const srcData = img.getContext('2d').getImageData(0, 0, SW, SH).data;

  const tasks = cfg.tasks || null;
  const corners0 = [[0, 0], [SW, 0], [SW, SH], [0, SH]].map(([x, y]) => [x + src.ox, y + src.oy]);
  if (tasks) tasks.forEach(t => { if (t.corners) t.goal = homographyFrom4(corners0, t.corners); });
  const state = tasks
    ? { preset: 'custom', t: 1, custom: I3(), task: 0 }
    : { preset: cfg.preset || presetNames[0], t: cfg.t ?? 1, custom: I3() };

  const canvas = h('canvas', { width: CW, height: CH });
  const g = canvas.getContext('2d');
  const off = document.createElement('canvas'); off.width = CW; off.height = CH;
  const og = off.getContext('2d');
  const out = og.createImageData(CW, CH);

  // matrix editor
  const cells = [];
  const matrixEl = h('div', { class: 'matrix', style: linearOnly ? 'grid-template-columns: repeat(2, auto)' : '' });
  for (let i = 0; i < 9; i++) {
    const r = Math.floor(i / 3), c = i % 3;
    const inp = h('input', { type: 'number', step: c === 2 && r < 2 ? '10' : r === 2 && c < 2 ? '0.0005' : '0.1' });
    if (r === 2) inp.classList.add(c < 2 ? 'persp' : 'fixed');
    if (linearOnly && (r === 2 || c === 2)) inp.style.display = 'none';
    if (!editable) inp.disabled = true;
    inp.addEventListener('input', () => {
      const m = cells.map(x => parseFloat(x.value) || 0);
      state.custom = m; state.preset = 'custom'; state.t = 1; slider.value = 1;
      syncButtons(); draw(false);
    });
    cells.push(inp); matrixEl.append(inp);
  }

  const buttons = presetNames.map(n => h('button', {
    onclick: () => { state.preset = n; animate(); },
  }, PRESETS[n].label));
  function syncButtons() { buttons.forEach((b, i) => b.classList.toggle('active', presetNames[i] === state.preset)); }

  const slider = h('input', { type: 'range', min: 0, max: 1, step: 0.01, value: state.t });
  slider.addEventListener('input', () => { state.t = parseFloat(slider.value); draw(); });
  const play = h('button', { onclick: animate }, '▶ morph from identity');
  const classEl = h('div', { class: 'readout' });

  // exercise mode
  const taskTitle = h('div', { style: 'font-size:1.15em; max-width:19em' });
  const statusEl = h('div', { style: 'font-size:1.2em; font-weight:600; min-height:1.5em' });
  const extraEl = h('div', { class: 'readout' });
  const taskBtns = (tasks || []).map((t, i) => h('button', { onclick: () => selectTask(i) }, `Task ${i + 1}`));
  function selectTask(i) {
    state.task = i; state.custom = I3(); state.preset = 'custom'; state.t = 1;
    taskBtns.forEach((b, j) => b.classList.toggle('active', j === i));
    taskTitle.textContent = tasks[i].title;
    draw();
  }
  const solutionBtn = h('button', {
    onclick: () => {
      const t = tasks[state.task];
      state.custom = t.goal ? t.goal.slice() : (t.solution || I3()).slice();
      draw();
    },
  }, 'show a solution');
  const resetBtn = h('button', { onclick: () => { state.custom = I3(); draw(); } }, 'reset to identity');

  el.classList.add('widget');
  const side = tasks
    ? h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; min-width:13em' },
        h('div', { class: 'wctl', style: 'gap:0.3em' }, taskBtns),
        taskTitle,
        h('div', {}, h('span', { class: 'readout' }, 'M = '), matrixEl),
        statusEl, extraEl, classEl,
        h('div', { class: 'wctl interactive-only', style: 'gap:0.3em' }, resetBtn, solutionBtn))
    : h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; min-width:13em' },
        h('div', {}, h('span', { class: 'readout' }, 'M = '), matrixEl),
        classEl,
        h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start' },
          h('div', { style: 'display:flex; flex-wrap:wrap; gap:0.3em' }, buttons),
          h('div', {}, h('label', {}, 't '), slider),
          play,
        ));
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' }, canvas, side));

  let anim = null;
  function animate() {
    if (isPrint()) { state.t = 1; draw(); return; }
    cancelAnimationFrame(anim);
    const t0 = performance.now(), dur = 1200;
    const step = now => {
      const u = Math.min(1, (now - t0) / dur);
      state.t = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2; // ease in-out
      slider.value = state.t; draw();
      if (u < 1) anim = requestAnimationFrame(step);
    };
    syncButtons();
    anim = requestAnimationFrame(step);
  }

  function draw(updateCells = true) {
    const M = presetMatrix(state.preset, state.t, state.custom);
    if (updateCells) M.forEach((v, i) => { if (document.activeElement !== cells[i]) cells[i].value = fmt(v, i >= 6 && i < 8 ? 4 : 2); });
    const cls = classify(M);
    classEl.innerHTML = '';
    classEl.append(h('span', { style: `color:${cls.color}` }, cls.name));

    // background grid + axes
    g.fillStyle = C.bg2; g.fillRect(0, 0, CW, CH);
    g.strokeStyle = '#262a33'; g.lineWidth = 1;
    const gx = ((-view.x0) % 40 + 40) % 40, gy = ((-view.y0) % 40 + 40) % 40;
    for (let x = gx; x < CW; x += 40) { g.beginPath(); g.moveTo(x + 0.5, 0); g.lineTo(x + 0.5, CH); g.stroke(); }
    for (let y = gy; y < CH; y += 40) { g.beginPath(); g.moveTo(0, y + 0.5); g.lineTo(CW, y + 0.5); g.stroke(); }

    // warped image by backward mapping
    const Minv = inv3(M);
    const d = out.data;
    d.fill(0);
    if (Minv) {
      for (let cy = 0; cy < CH; cy++) {
        const Y = cy + view.y0;
        for (let cx = 0; cx < CW; cx++) {
          const X = cx + view.x0;
          const w = Minv[6] * X + Minv[7] * Y + Minv[8];
          if (Math.abs(w) < 1e-9) continue;
          const px = (Minv[0] * X + Minv[1] * Y + Minv[2]) / w;
          const py = (Minv[3] * X + Minv[4] * Y + Minv[5]) / w;
          if (M[6] * px + M[7] * py + M[8] <= 0) continue; // point would map "behind" the plane
          const sx = Math.floor(px - src.ox), sy = Math.floor(py - src.oy);
          if (sx < 0 || sy < 0 || sx >= SW || sy >= SH) continue;
          const si = 4 * (sy * SW + sx), di = 4 * (cy * CW + cx);
          d[di] = srcData[si]; d[di + 1] = srcData[si + 1]; d[di + 2] = srcData[si + 2]; d[di + 3] = 255;
        }
      }
    }
    og.putImageData(out, 0, 0);

    // ghost of the original image position
    g.save(); g.globalAlpha = 0.18; g.drawImage(img, src.ox - view.x0, src.oy - view.y0); g.restore();
    g.setLineDash([6, 5]); g.strokeStyle = C.dim; g.lineWidth = 1.5;
    g.strokeRect(src.ox - view.x0, src.oy - view.y0, SW, SH); g.setLineDash([]);
    g.drawImage(off, 0, 0);

    // axes through the world origin (x right, y down, like pixel coordinates)
    const O = [-view.x0, -view.y0];
    arrow(g, O[0] - (origin === 'center' ? CW / 2 - 10 : 60), O[1], CW - 12, O[1], C.dim, 1.5);
    arrow(g, O[0], O[1] - (origin === 'center' ? CH / 2 - 10 : 40), O[0], CH - 12, C.dim, 1.5);
    g.fillStyle = C.dim; g.font = '15px Inter, sans-serif';
    g.fillText('x', CW - 22, O[1] - 8); g.fillText('y', O[0] + 8, CH - 16);

    // images of the basis vectors and of the origin
    const L = 80;
    const p0 = apply3(M, 0, 0), p1 = apply3(M, L, 0), p2 = apply3(M, 0, L);
    if (p0[2] > 0 && p1[2] > 0 && p2[2] > 0) {
      const toC = p => [p[0] - view.x0, p[1] - view.y0];
      const [a, b, c] = [toC(p0), toC(p1), toC(p2)];
      if (Math.hypot(a[0] - O[0], a[1] - O[1]) > 2) arrow(g, O[0], O[1], a[0], a[1], C.purple, 2);
      arrow(g, a[0], a[1], b[0], b[1], C.red, 3.5);
      arrow(g, a[0], a[1], c[0], c[1], C.green, 3.5);
    }
    if (tasks) drawTask(M);
  }

  const toCanvas = ([x, y]) => [x - view.x0, y - view.y0];
  function numbered(pts, color, dashed) {
    g.setLineDash(dashed ? [8, 6] : []); g.strokeStyle = color; g.lineWidth = 2.5;
    g.beginPath(); pts.map(toCanvas).forEach(([x, y], i) => (i ? g.lineTo(x, y) : g.moveTo(x, y))); g.closePath(); g.stroke();
    g.setLineDash([]);
    pts.map(toCanvas).forEach(([x, y], i) => {
      g.fillStyle = color; g.beginPath(); g.arc(x, y, 11, 0, 7); g.fill();
      g.fillStyle = C.bg; g.font = 'bold 14px Inter, sans-serif'; g.textAlign = 'center'; g.textBaseline = 'middle';
      g.fillText(String(i + 1), x, y + 0.5);
      g.textAlign = 'start'; g.textBaseline = 'alphabetic';
    });
  }
  function drawTask(M) {
    const t = tasks[state.task];
    const mapped = corners0.map(([x, y]) => apply3(M, x, y));
    const visible = mapped.every(p => p[2] > 0);
    let ok = false;
    extraEl.innerHTML = '';
    if (t.goal) {
      const target = corners0.map(([x, y]) => apply3(t.goal, x, y));
      numbered(target.map(p => [p[0], p[1]]), C.accent, true);
      const err = visible ? Math.max(...mapped.map((p, i) => Math.hypot(p[0] - target[i][0], p[1] - target[i][1]))) : Infinity;
      ok = err < 4;
      extraEl.textContent = visible ? `max corner error: ${fmt(err, 1)} px` : '';
    } else if (t.goal === undefined && t.check === 'involution') {
      let M2 = mul3(M, M);
      const s2 = Math.abs(M2[8]) > 1e-9 ? M2[8] : 1, Mn = Math.abs(M[8]) > 1e-9 ? M[8] : 1;
      M2 = M2.map(v => v / s2);
      const errI = Math.max(...M2.map((v, i) => Math.abs(v - I3()[i]) * (i % 3 === 2 && i < 6 ? 0.01 : 1)));
      const distI = Math.max(...M.map((v, i) => Math.abs(v / Mn - I3()[i]) * (i % 3 === 2 && i < 6 ? 0.01 : 1)));
      ok = errI < 0.02 && distI > 0.05;
      extraEl.innerHTML = `M² = [${[0, 1, 2].map(r => M2.slice(3 * r, 3 * r + 3).map(v => fmt(v, 2)).join(', ')).join(' ; ')}]` +
        (distI <= 0.05 ? '<br>(M is the identity — not allowed)' : '');
    }
    if (visible) numbered(mapped.map(p => [p[0], p[1]]), '#ffffff', false);
    statusEl.innerHTML = ok ? `<span style="color:${C.green}">✓ goal reached</span>` : `<span style="color:${C.dim}">✗ not yet</span>`;
  }

  syncButtons();
  if (tasks) selectTask(0); else draw();
  if (cfg.autoplay && !isPrint()) setTimeout(animate, 300);
}
