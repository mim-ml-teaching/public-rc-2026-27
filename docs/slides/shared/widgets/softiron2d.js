// Soft iron, level flight, seen in two frames at once.
// The Earth's (horizontal) field points to magnetic north and is fixed in the world. A steel spar runs along the wing:
// it is part of the aircraft and turns with it, and so does the compass next to it. The spar is magnetised mostly along
// its length, so it adds k (u·b) u to the field b at the compass (u = spar direction):  m = (I + k u uᵀ) b.
//   left:  world, top view, north up — the aircraft turns, the field stays; the yellow reading is pulled towards the wing
//   right: aircraft frame, nose up — what the compass sees: the field turns the other way, the spar stays; the tip of
//          the reading follows the dashed ellipse (stretched along the spar) instead of the dotted circle
// Grey: true field b; purple: induced part along the spar; yellow: reading m. Heading ψ is the one control.
//
// spar "wing" (along body y) or "fuselage" (along body x): the same effect turned by 90°, heading error of opposite sign.
//
// config: { width (panel width, 390), height (330), k (0.6), heading (deg, 50), spar ("wing") }
import { C, h, fmt, arrow } from './util.js';

const deg = Math.PI / 180;
// aircraft outline in body coordinates (x forward, y right)
const FUS = [[1.0, 0], [-0.9, 0]], WING = [[0.1, -0.95], [0.1, 0.95]], TAIL = [[-0.85, -0.32], [-0.85, 0.32]];

export function mount(el, cfg) {
  const PW = cfg.width || 390, H = cfg.height || 330, W = 2 * PW + 12;
  const state = { k: cfg.k ?? 0.6, psi: cfg.heading ?? 50, spar: cfg.spar || 'wing', playing: false };
  const canvas = h('canvas', { width: W, height: H });
  const g = canvas.getContext('2d');
  const readout = h('div', { class: 'readout', style: 'font-size:0.85em; min-height:1.4em' });

  function panel(x0, title, sub) {
    g.fillStyle = C.bg2; g.fillRect(x0, 0, PW, H);
    g.strokeStyle = C.line; g.lineWidth = 1; g.strokeRect(x0 + 0.5, 0.5, PW - 1, H - 1);
    g.fillStyle = C.fg; g.font = '600 16px Inter, Arial'; g.fillText(title, x0 + 12, 22);
    g.fillStyle = C.dim; g.font = '14px Inter, Arial'; g.fillText(sub, x0 + 12, 40);
  }
  function aircraft(P, toFrame, scale) {     // P: screen map of the panel frame; toFrame: body (x, y) → panel frame
    const seg = (a, b, col, w) => {
      g.strokeStyle = col; g.lineWidth = w; g.lineCap = 'round'; g.beginPath();
      g.moveTo(...P(toFrame(a.map(v => v * scale)))); g.lineTo(...P(toFrame(b.map(v => v * scale)))); g.stroke();
    };
    seg(...FUS, '#5a606c', 9); seg(...WING, '#5a606c', 9); seg(...TAIL, '#5a606c', 7);
    if (state.spar === 'wing') seg(...WING, C.purple, 4); else seg(...FUS, C.purple, 4); // the steel spar
    g.lineCap = 'butt';
    const c = P(toFrame([0.25 * scale, 0])); g.fillStyle = C.green; g.fillRect(c[0] - 6, c[1] - 6, 12, 12); // compass
  }

  function draw() {
    const k = state.k, p = state.psi * deg;
    // with the spar along the fuselage the ellipse is tall: shrink so that it fits the panel
    const S = Math.min(Math.min(PW, H) * 0.31, state.spar === 'wing' ? Infinity : (H / 2 - 44) / (1 + k));
    // body-frame vectors (x forward, y right); the field points to magnetic north
    const b = [Math.cos(p), -Math.sin(p)];
    const ind = state.spar === 'wing' ? [0, k * b[1]] : [k * b[0], 0];  // k (u·b) u, u = body y (wing) or x (fuselage)
    const m = [b[0] + ind[0], b[1] + ind[1]];
    const toWorld = ([x, y]) => [x * Math.cos(p) - y * Math.sin(p), x * Math.sin(p) + y * Math.cos(p)]; // → (north, east)
    const vecs = (P, F) => {
      const sc = v => F(v).map(x => x * S);
      arrow(g, ...P([0, 0]), ...P(sc(b)), C.dim, 3);
      if (Math.hypot(...ind) > 0.03) arrow(g, ...P(sc(b)), ...P(sc(m)), C.purple, 3);
      arrow(g, ...P([0, 0]), ...P(sc(m)), C.accent, 3.5);
    };

    // ---- left: world, north up ----
    panel(0, 'World (top view, north up)', 'field fixed · aircraft turns');
    const cw = [PW / 2, H / 2 + 26];
    const Pw = ([n, e]) => [cw[0] + e, cw[1] - n];
    g.fillStyle = C.dim; g.font = '600 18px Inter, Arial'; g.fillText('N ↑', 12, 64);
    aircraft(Pw, toWorld, S * 0.7);
    vecs(Pw, toWorld);

    // ---- right: aircraft frame, nose up ----
    const x1 = PW + 12;
    panel(x1, 'Aircraft frame (what the compass sees)', 'aircraft fixed · field turns');
    const cb = [x1 + PW / 2, H / 2 + 26];
    const Pb = ([x, y]) => [cb[0] + y, cb[1] - x];
    aircraft(Pb, v => v, S * 0.7);
    g.setLineDash([2, 5]); g.strokeStyle = C.dim; g.lineWidth = 1.5; g.beginPath(); g.arc(cb[0], cb[1], S, 0, 7); g.stroke();
    g.setLineDash([7, 5]); g.strokeStyle = C.accent; g.globalAlpha = 0.7;
    const wingSpar = state.spar === 'wing';
    g.beginPath(); g.ellipse(cb[0], cb[1], wingSpar ? S * (1 + k) : S, wingSpar ? S : S * (1 + k), 0, 0, 7); g.stroke(); g.globalAlpha = 1; g.setLineDash([]);
    vecs(Pb, v => v);

    // legend
    g.font = '14px Inter, Arial';
    [[C.dim, 'true field b (north)'], [C.purple, 'added by the spar'], [C.accent, 'reading m']].forEach(([c, t], i) => {
      g.fillStyle = c; g.fillRect(12 + i * 170, H - 20, 14, 4); g.fillText(t, 32 + i * 170, H - 14);
    });

    const ps = ((state.psi % 360) + 360) % 360, toFus = Math.min(ps % 180, 180 - (ps % 180));
    const toSpar = state.spar === 'wing' ? 90 - toFus : toFus;
    const hdgErr = ((Math.atan2(-m[1], m[0]) - p) / deg + 540) % 360 - 180;
    readout.textContent = `heading ${fmt(ps, 0)}° · angle field–spar ${fmt(toSpar, 0)}° · |m|/|b| ${fmt(Math.hypot(...m), 2)} · heading error ${fmt(hdgErr, 1)}°`;
  }

  const hs = h('input', { type: 'range', min: 0, max: 360, step: 1, value: state.psi });
  hs.addEventListener('input', () => { state.psi = +hs.value; draw(); });
  const ks = h('input', { type: 'range', min: 0, max: 1, step: 0.05, value: state.k, style: 'width:6em' });
  ks.addEventListener('input', () => { state.k = +ks.value; draw(); });
  const sparBtns = [['wing', 'spar in the wing'], ['fuselage', 'spar along the fuselage']].map(([v, t]) => {
    const b = h('button', { onclick: () => { state.spar = v; sparBtns.forEach(x => x.classList.toggle('active', x === b)); draw(); } }, t);
    b.classList.toggle('active', state.spar === v);
    return b;
  });
  const play = h('button', { onclick: () => { state.playing = !state.playing; play.textContent = state.playing ? '❚❚ stop' : '▶ turn the aircraft'; if (state.playing) loop(); } }, '▶ turn the aircraft');
  function loop() {
    if (!state.playing || !canvas.isConnected) return;
    state.psi = (state.psi + 0.5) % 360; hs.value = state.psi; draw();
    requestAnimationFrame(loop);
  }

  el.classList.add('widget');
  el.append(h('div', { style: `width:${W}px` }, canvas, readout,
    h('div', { class: 'wctl interactive-only' }, h('label', {}, 'heading ψ'), hs, h('label', {}, 'k (exaggerated)'), ks, play),
    h('div', { class: 'wctl interactive-only' }, sparBtns)));
  draw();
}
