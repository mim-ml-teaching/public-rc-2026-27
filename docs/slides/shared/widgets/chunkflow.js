// π0-style action chunks, inference interleaved with execution, computed exactly (no network). The gripper has to get from
// the start (bottom) to the box (top) past two obstacles; the demonstrations go around each on the left or on the right
// (four strategies). The task takes four chunks of 50 positions: the policy infers a chunk from the current observation (the
// gripper position), the robot executes it open loop, then the next inference starts from where the gripper is now.
// An inference = A^0 ~ N(0, I) carried to a chunk by 10 Euler steps of the marginal flow of the conditional OT paths
// (the flow-matching target of the lecture, weighted by how well each demonstration fits the observation: chunklib.js).
// The slide's chain A^0 → A^τ → A^1 (elements with data-pi = 0 | tau | 1, and a span data-tau for the current τ) lights up during an
// inference and goes dark while the robot executes. Two episodes with different choices at the obstacles.
// config: { n: 10 (Euler steps), episodes: [[-1, 1], [1, 1]] (sides around obstacle 1 and 2), width: 540, height: 500, seed: 5 }
import { C, h, isPrint } from './util.js';
import { loopPlayer } from './loopplayer.js';
import { rng } from './flowlib.js';
import { K, D, NCH, START, OBSTACLES, makeDemos, episode } from './chunklib.js';

const XR = 3.2, YR = 3.0;
const BLUE = [90, 176, 255], GREEN = [95, 211, 141], RED = [255, 107, 107];

export function mount(el, cfg) {
  const W = cfg.width || 540, H = cfg.height || 500, N = cfg.n || 10;
  const canvas = h('canvas', { width: W, height: H, style: `width:${W}px; height:${H}px` }), g = canvas.getContext('2d');
  const sx = x => (x + XR) / (2 * XR) * W, sy = y => H - (y + YR) / (2 * YR) * H;
  const r = rng(cfg.seed || 5), demos = makeDemos(r);
  const eps = (cfg.episodes || [[-1, 1], [1, 1]]).map(want => episode(r, demos, N, want).infs);
  // timeline: per inference noise on show, N Euler steps, a short hold, then the robot executes the chunk; the very first inference slowly
  const segs = []; let acc = 0;
  eps.forEach((infs, e) => infs.forEach((inf, j) => {
    const slow = e === 0 && j === 0, tN = slow ? 1200 : 400, tS = slow ? 600 : 220, tE = slow ? 400 : 250, tX = 1200, tail = j === NCH - 1 ? 1400 : 0;
    const dur = tN + N * tS + tE + tX + tail; segs.push({ e, j, inf, t0: acc, tN, tS, tE, tX, dur }); acc += dur;
  }));
  const T_RUN = acc, EP0_END = segs.filter(s => s.e === 0).reduce((a, s) => a + s.dur, 0);
  // the chain on the slide
  const sec = el.closest('section'), items = sec ? [...sec.querySelectorAll('[data-pi]')] : [], tauEl = sec?.querySelector('[data-tau]');
  const light = (key, tau) => { items.forEach(e => e.classList.toggle('on', e.dataset.pi === key)); if (tauEl) tauEl.textContent = key === 'tau' ? `τ = ${tau.toFixed(2)}` : ''; };
  const lerpC = (a, b, f) => a.map((v, i) => Math.round(v + (b[i] - v) * f));
  const tauColor = (tau, al) => { const c = tau < 0.5 ? lerpC(BLUE, GREEN, tau * 2) : lerpC(GREEN, RED, (tau - 0.5) * 2); return `rgba(${c[0]},${c[1]},${c[2]},${al})`; };
  const ease = u => u * u * (3 - 2 * u);
  function polyline(v, color, lw, dotR) {
    g.strokeStyle = color; g.lineWidth = lw; g.lineJoin = 'round'; g.beginPath();
    for (let k = 0; k < K; k++) (k ? g.lineTo : g.moveTo).call(g, sx(v[2 * k]), sy(v[2 * k + 1]));
    g.stroke(); g.fillStyle = color; for (let k = 0; k < K; k++) { g.beginPath(); g.arc(sx(v[2 * k]), sy(v[2 * k + 1]), dotR, 0, 7); g.fill(); }
  }
  function draw(ms) {
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    // the scene: two obstacles, the box at the top, the start at the bottom
    g.fillStyle = '#3a3f4b'; g.strokeStyle = '#5a606c'; g.lineWidth = 2;
    for (const o of OBSTACLES) { g.beginPath(); g.roundRect(sx(o.x - o.hw), sy(o.y + o.hh), sx(o.x + o.hw) - sx(o.x - o.hw), sy(o.y - o.hh) - sy(o.y + o.hh), 10); g.fill(); g.stroke(); }
    g.fillStyle = '#c98500'; g.strokeStyle = 'rgba(21,23,28,0.5)'; g.beginPath(); g.roundRect(sx(-0.34), sy(2.95), sx(0.34) - sx(-0.34), sy(2.3) - sy(2.95), 5); g.fill(); g.stroke();
    g.strokeStyle = C.dim; g.lineWidth = 3; g.beginPath(); g.arc(sx(START[0]), sy(START[1] - 0.2), 11, 0, 7); g.stroke();
    // where we are: inference j of episode e, and the phase inside it
    let si = 0; for (let i = 0; i < segs.length; i++) if (segs[i].t0 <= ms) si = i;
    const s = segs[si], q = ms - s.t0, infs = eps[s.e], F = s.inf.path[N];
    const p1 = s.tN, p2 = p1 + N * s.tS, p3 = p2 + s.tE, p4 = p3 + s.tX;
    let k = 0, u = 0, tau = 0, key = '0', phase = 'infer';
    if (q >= p4) { k = N - 1; u = 1; tau = 1; key = null; phase = 'done'; }
    else if (q >= p3) { k = N - 1; u = 1; tau = 1; key = null; phase = 'exec'; }
    else if (q >= p2) { k = N - 1; u = 1; tau = 1; key = '1'; }
    else if (q >= p1) { const w = q - p1; k = Math.floor(w / s.tS); u = ease(Math.min(1, (w - k * s.tS) / (s.tS * 0.78))); tau = (k + u) / N; key = 'tau'; }
    light(key, tau);
    for (let i = 0; i < s.j; i++) polyline(infs[i].path[N], 'rgba(255,107,107,0.9)', 3, 2);   // chunks executed so far
    const a = s.inf.path[k], b = s.inf.path[k + 1], cur = new Float64Array(D); for (let m = 0; m < D; m++) cur[m] = a[m] + u * (b[m] - a[m]);
    polyline(cur, tauColor(tau, 0.95), 3, 2.6);
    // the gripper: at the observation until the chunk is executed, then at its end
    let gx = s.inf.o[0], gy = s.inf.o[1];
    if (phase === 'exec' || phase === 'done') { const f = phase === 'done' ? K - 1 : Math.min(1, (q - p3) / s.tX) * (K - 1), k0 = Math.min(K - 2, Math.floor(f)), w = f - k0; gx = F[2 * k0] + w * (F[2 * k0 + 2] - F[2 * k0]); gy = F[2 * k0 + 1] + w * (F[2 * k0 + 3] - F[2 * k0 + 1]); }
    g.fillStyle = C.fg; g.beginPath(); g.arc(sx(gx), sy(gy), 8, 0, 7); g.fill(); g.strokeStyle = '#ff6b6b'; g.lineWidth = 3; g.stroke();
    g.textAlign = 'left'; g.fillStyle = C.fg; g.font = 'bold 17px Inter, Arial';
    g.fillText(phase === 'infer' ? `inference ${s.j + 1} of ${NCH}` : phase === 'exec' ? `executing chunk ${s.j + 1}` : s.j === NCH - 1 ? `${NCH} chunks executed` : `inference ${s.j + 1} of ${NCH}`, 12, 24);
    g.font = '14px Inter, Arial'; g.fillStyle = C.dim; g.fillText('one chunk = 50 positions: infer, execute, infer again', 12, H - 12);
  }
  let last = 0;
  const drawTracked = ms => { last = Math.min(1, ms / T_RUN); draw(ms); };
  const loop = loopPlayer({ dur: T_RUN, hold: 600, hold0: 500, alive: () => canvas.isConnected, step: t => drawTracked(t * T_RUN) });
  const setBtn = () => { btn.textContent = loop.playing ? '⏸' : '▶'; };
  const btn = h('button', { style: 'min-width:2.4em; position:absolute; top:8px; right:8px; opacity:0.85', onclick: () => { if (loop.playing) loop.stop(); else loop.start(last); setBtn(); } }, '⏸');
  el.classList.add('widget'); el.style.position = 'relative'; el.style.width = `${W}px`;
  el.append(canvas, h('span', { class: 'interactive-only' }, btn));
  if (isPrint()) { draw(EP0_END - 10); return; }
  draw(0);
  const run = () => { loop.stop(); loop.start(0); setBtn(); };
  el.closest('.reveal')?.addEventListener('slidechanged', ev => { if (!canvas.isConnected) return; if (ev.currentSlide?.contains(el)) run(); else { loop.stop(); setBtn(); } });
  if (sec?.classList.contains('present')) run();
}
