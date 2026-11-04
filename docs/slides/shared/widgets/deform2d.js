// Two panels, one deformation. A square of "substance" (a checkerboard mesh) and a cloud of probability mass
// (samples of a Gaussian) are carried by the same time-dependent map φ_t, t ∈ [0, 1], so the audience sees that
// deforming probability is deforming a substance, and that it is a continuous process.
// Maps: "knead" (a hand-made velocity field: a twist plus two attractors) or "flow" (the learned flow-matching
// field that carries the Gaussian onto the two moons: the goal of the lecture).
// config: { map: "knead" | "flow", width: 400 (per panel), height: 400, t: 0, print_t, autoplay }
import { C, h, fmt, isPrint } from './util.js';
import { rng, gauss, importNet, velocity } from './flowlib.js';

const MAPS = { knead: 'hand-made: twist and knead', flow: 'learned flow: Gaussian → two moons' };
const STEPS = 48, R = 3.2, G = 14;   // integration steps, world half-width, grid cells per side

export async function mount(el, cfg) {
  const W = cfg.width || 400, H = cfg.height || 400;
  const st = { map: cfg.map || 'knead', t: cfg.t ?? 0, playing: false };
  const sx = x => (x + R) / (2 * R) * W, sy = y => H - (y + R) / (2 * R) * H;
  const cv1 = h('canvas', { width: W, height: H }), cv2 = h('canvas', { width: W, height: H });
  const g1 = cv1.getContext('2d'), g2 = cv2.getContext('2d');
  const r = rng(5);
  const cloud = Array.from({ length: 700 }, () => [gauss(r), gauss(r)]);
  const nodes = [];
  for (let i = 0; i <= G; i++) for (let j = 0; j <= G; j++) nodes.push([-2 + 4 * i / G, -2 + 4 * j / G]);
  let net = null, trajN, trajC;

  const knead = (t, x, y) => {
    const w = 1.3 * Math.exp(-(x * x + y * y) / 5);           // a twist, strongest in the middle
    let vx = -w * y, vy = w * x;
    for (const [ax, ay] of [[-1.6, 0.9], [1.6, -0.9]]) {      // two attractors: mass piles up there
      const dx = ax - x, dy = ay - y, k = Math.exp(-(dx * dx + dy * dy) / 2.5);
      vx += k * dx; vy += k * dy;
    }
    return [vx, vy];
  };
  const field = (t, x, y) => (st.map === 'flow' ? velocity(net, t, x, y) : knead(t, x, y));
  const integrate = pts => pts.map(p => {
    const path = [p.slice()]; let [x, y] = p;
    for (let k = 0; k < STEPS; k++) { const [vx, vy] = field(k / STEPS, x, y); x += vx / STEPS; y += vy / STEPS; path.push([x, y]); }
    return path;
  });
  async function prepare() {
    if (st.map === 'flow' && !net) net = importNet(await (await fetch('../../assets/lecture05/data/flow_gauss_moons.json')).json());
    trajN = integrate(nodes); trajC = integrate(cloud);
  }
  function draw() {
    const k = Math.round(st.t * STEPS), P = (i, j) => trajN[i * (G + 1) + j][k];
    g1.fillStyle = C.bg2; g1.fillRect(0, 0, W, H);
    for (let i = 0; i < G; i++) for (let j = 0; j < G; j++) {
      const [a, b, c, d] = [P(i, j), P(i + 1, j), P(i + 1, j + 1), P(i, j + 1)];
      g1.fillStyle = (i + j) % 2 ? '#c98500' : '#e9dcc0';
      g1.beginPath(); g1.moveTo(sx(a[0]), sy(a[1])); g1.lineTo(sx(b[0]), sy(b[1])); g1.lineTo(sx(c[0]), sy(c[1])); g1.lineTo(sx(d[0]), sy(d[1])); g1.closePath();
      g1.fill(); g1.strokeStyle = 'rgba(21,23,28,0.45)'; g1.lineWidth = 1; g1.stroke();
    }
    g2.fillStyle = C.bg2; g2.fillRect(0, 0, W, H);
    g2.fillStyle = C.blue;
    for (const p of trajC) { const [x, y] = p[k]; g2.beginPath(); g2.arc(sx(x), sy(y), 2.6, 0, 7); g2.fill(); }
    readout.innerHTML = `time <b>t = ${fmt(st.t, 2)}</b> · ${MAPS[st.map]}`;
  }
  function play() {
    st.playing = !st.playing; playBtn.textContent = st.playing ? '⏸ pause' : '▶ play';
    if (st.t >= 1) st.t = 0;
    let last = performance.now();
    const tick = now => {
      if (!st.playing || !cv1.isConnected) return;
      st.t = Math.min(1, st.t + (now - last) / 4000); last = now; slider.value = st.t; draw();
      if (st.t < 1) requestAnimationFrame(tick); else { st.playing = false; playBtn.textContent = '▶ replay'; }
    };
    if (st.playing) requestAnimationFrame(tick);
  }
  const reset = () => { st.playing = false; playBtn.textContent = '▶ play'; st.t = cfg.t ?? 0; slider.value = st.t; };
  const playBtn = h('button', { onclick: play }, '▶ play');
  const slider = h('input', { type: 'range', min: 0, max: 1, step: 0.01, value: st.t, style: 'width:14em', oninput: e => { st.t = +e.target.value; draw(); } });
  const sel = h('select', { style: 'font:inherit; background:#272b34; color:#e7e9ee; border:1px solid #3a3f4b; border-radius:6px' },
    Object.entries(MAPS).map(([k, v]) => { const o = h('option', { value: k }, v); if (k === st.map) o.selected = true; return o; }));
  sel.addEventListener('change', async () => { st.map = sel.value; reset(); await prepare(); draw(); });
  const readout = h('span', { class: 'dim' });
  const cap = (txt) => h('div', { style: `color:${C.blue}; font-weight:600; margin-bottom:0.3em` }, txt);
  el.classList.add('widget');
  el.append(
    h('div', { style: 'display:flex; gap:1.2em; justify-content:center' },
      h('div', {}, cap('Imagine deforming a substance'), cv1), h('div', {}, cap('Probability is our substance'), cv2)),
    h('div', { class: 'wctl', style: 'justify-content:center; gap:0.8em; margin-top:0.4em' },
      h('span', { class: 'interactive-only', style: 'display:flex; gap:0.5em; align-items:center' }, h('label', {}, 'time t'), slider, playBtn, h('label', {}, 'map'), sel), readout));
  el.closest('.reveal')?.addEventListener('slidechanged', ev => { if (cv1.isConnected && ev.currentSlide?.contains(el)) { reset(); draw(); } });
  await prepare();
  if (isPrint() && cfg.print_t !== undefined) { st.t = cfg.print_t; slider.value = st.t; }
  draw();
  if (cfg.autoplay && !isPrint()) {
    // play whenever the slide is shown, and start over after a short hold at the end
    const shown = () => el.closest('section')?.classList.contains('present') !== false;
    const loop = () => { if (!cv1.isConnected) return; if (shown() && !st.playing) { if (st.t >= 1) st.t = 0; play(); } setTimeout(loop, st.t >= 1 ? 1500 : 300); };
    el.closest('.reveal')?.addEventListener('slidechanged', () => { if (!shown() && st.playing) play(); });
    setTimeout(loop, 400);
  }
}
