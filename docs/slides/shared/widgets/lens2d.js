// Pinhole vs thin lens, side view, with two objects at different distances.
// Pinhole: rays go straight, every object is sharp for any sensor position (but little light).
// Thin lens (paraxial): a ray hitting the lens at height y_h leaves with slope s_out = s_in − y_h / f,
// so rays from an object at distance a meet again at b = 1 / (1/f − 1/a): each depth has its own focus.
// config: { aperture: 0.45, b: <sensor distance>, width, height }
import { C, h, fmt } from './util.js';

const F = 1.0, PIN = 0.035;
const OBJECTS = [
  { name: 'near', a: 1.6, tip: 0.45, c: [95, 211, 141] },
  { name: 'far', a: 3.0, tip: -0.75, c: [242, 177, 52] },
].map(o => ({ ...o, b: 1 / (1 / F - 1 / o.a) }));   // near: b ≈ 2.67, far: b = 1.5

export function mount(el, cfg) {
  const W = cfg.width || 640, H = cfg.height || 400;
  const state = { ap: cfg.aperture ?? 0.45, b: cfg.b ?? OBJECTS[1].b };
  const canvas = h('canvas', { width: W, height: H });
  const g = canvas.getContext('2d');
  const readout = h('div', { style: 'line-height:1.5; min-height:8.5em' });
  const amax = Math.max(...OBJECTS.map(o => o.a)), bmax = 3.0;
  const X0 = 20, S = (W - X0 - 70) / (amax + bmax + 0.2);
  const LX = X0 + (amax + 0.1) * S;                  // lens / pinhole position in px
  const rowH = H / 2;

  function row(top, kind) {
    const Y0 = top + rowH / 2;
    const toC = (x, y) => [LX + x * S, Y0 - y * S]; // x measured from the lens (negative = object side)
    const half = kind === 'pinhole' ? PIN : state.ap;
    g.fillStyle = C.fg; g.font = '600 16px Inter, Arial';
    g.fillText(kind === 'pinhole' ? 'pinhole' : 'lens', 8, top + 20);
    // optical axis
    g.strokeStyle = '#30343d'; g.lineWidth = 1; g.beginPath(); g.moveTo(0, Y0); g.lineTo(W, Y0); g.stroke();
    // objects: arrows from the axis to the tip
    OBJECTS.forEach(o => {
      const [x0, y0] = toC(-o.a, 0), [, yt] = toC(-o.a, o.tip);
      g.strokeStyle = `rgb(${o.c})`; g.lineWidth = 3; g.beginPath(); g.moveTo(x0, y0); g.lineTo(x0, yt); g.stroke();
      g.fillStyle = `rgb(${o.c})`; g.beginPath(); g.arc(x0, yt, 5, 0, 7); g.fill();
      g.font = '13px Inter, Arial'; g.fillText(o.name, x0 - 14, o.tip > 0 ? yt - 10 : yt + 20);
    });
    // barrier / lens
    g.fillStyle = '#5a606c';
    g.fillRect(LX - 3, top + 4, 6, Y0 - half * S - top - 4);
    g.fillRect(LX - 3, Y0 + half * S, 6, top + rowH - 4 - (Y0 + half * S));
    if (kind === 'lens') { g.fillStyle = 'rgba(178,141,255,0.45)'; g.beginPath(); g.ellipse(LX, Y0, 7, half * S, 0, 0, 7); g.fill(); }
    // ray fans from each tip, through the aperture, to the sensor
    const spots = OBJECTS.map(o => {
      const N = 9; let lo = Infinity, hi = -Infinity;
      for (let i = 0; i < N; i++) {
        const yh = -half + 2 * half * i / (N - 1);
        const sIn = (yh - o.tip) / o.a;
        const sOut = kind === 'lens' ? sIn - yh / F : sIn;
        const ys = yh + sOut * state.b;
        lo = Math.min(lo, ys); hi = Math.max(hi, ys);
        g.strokeStyle = `rgba(${o.c},0.45)`; g.lineWidth = 1.1;
        g.beginPath(); g.moveTo(...toC(-o.a, o.tip)); g.lineTo(...toC(0, yh)); g.lineTo(...toC(state.b, ys)); g.stroke();
      }
      if (kind === 'lens') {                         // where this object comes into focus
        const [fx, fy] = toC(o.b, -o.tip * o.b / o.a);
        g.strokeStyle = `rgb(${o.c})`; g.lineWidth = 1.5; g.setLineDash([3, 3]);
        g.beginPath(); g.moveTo(fx, top + 8); g.lineTo(fx, top + rowH - 8); g.stroke(); g.setLineDash([]);
      }
      return { o, lo, hi, half };
    });
    // sensor with recorded spots: intensity ∝ light gathered / spot size
    const [sx] = toC(state.b, 0);
    g.fillStyle = '#0d0f13'; g.fillRect(sx - 4, top + 6, 8, rowH - 12);
    spots.forEach(({ o, lo, hi }) => {
      const size = Math.max(hi - lo, 0.015), alpha = Math.min(1, (2 * half) / size * 0.6);
      const [, y1] = toC(0, hi), [, y2] = toC(0, lo);
      g.fillStyle = `rgba(${o.c},${alpha})`; g.fillRect(sx - 4, y1 - 1, 8, Math.max(2, y2 - y1 + 2));
    });
    g.fillStyle = C.dim; g.font = '13px Inter, Arial'; g.fillText('sensor', sx + 8, top + rowH - 10);
    return spots;
  }

  function draw() {
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    g.strokeStyle = C.line; g.beginPath(); g.moveTo(0, rowH); g.lineTo(W, rowH); g.stroke();
    const sp = row(0, 'pinhole'), sl = row(rowH, 'lens');
    const line = (s, kind) => {
      const size = s.hi - s.lo, sharp = kind === 'pinhole' ? true : size < 0.04;
      return `<span style="color:rgb(${s.o.c})">${s.o.name}</span> ${sharp ? '✓ sharp' : `✗ blur ${fmt(size, 2)}`}`;
    };
    readout.innerHTML =
      `<div><b>pinhole</b> (light ×1): ${sp.map(s => line(s, 'pinhole')).join(' · ')}</div>` +
      `<div style="margin-top:0.3em"><b>lens</b> (light ×${fmt(state.ap / PIN, 0)}): ${sl.map(s => line(s, 'lens')).join(' · ')}</div>` +
      `<div class="dim" style="margin-top:0.3em">dashed lines: where each object is in focus, $\\frac{1}{f} = \\frac{1}{a} + \\frac{1}{b}$ — the nearer object focuses farther back</div>`;
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
  }

  const bSlider = h('input', { type: 'range', min: 1.2, max: 3.0, step: 0.01, value: state.b });
  bSlider.addEventListener('input', () => { state.b = parseFloat(bSlider.value); draw(); });
  const apSlider = h('input', { type: 'range', min: 0.05, max: 0.9, step: 0.01, value: state.ap });
  apSlider.addEventListener('input', () => { state.ap = parseFloat(apSlider.value); draw(); });
  const focusOn = o => h('button', {
    onclick: () => {                                 // glide the sensor to this object's focus distance
      const b0 = state.b, t0 = performance.now();
      const step = now => {
        const u = Math.min(1, (now - t0) / 900), e = u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2;
        state.b = b0 + (o.b - b0) * e; bSlider.value = state.b; draw();
        if (u < 1) requestAnimationFrame(step);
      };
      requestAnimationFrame(step);
    },
  }, `focus on ${o.name}`);

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    canvas,
    h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:15em; flex:none' },
      readout,
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.3em' },
        h('div', { style: 'display:flex; gap:0.3em' }, OBJECTS.map(focusOn)),
        h('div', {}, h('label', {}, 'sensor position '), bSlider),
        h('div', {}, h('label', {}, 'aperture '), apSlider)))));
  draw();
}
