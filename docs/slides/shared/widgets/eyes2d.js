// Hunters and prey, seen from above: two eyes on a head, each with a horizontal field of view θ, their visual axes
// turned outwards by ±δ/2 from straight ahead, a baseline b apart. Every point around the head is coloured by which
// eyes see it: both (binocular: stereo depth), one (monocular), none (blind). With wide-set eyes (large b) the
// binocular region starts further in front of the head.
// Far from the head: total field ≈ min(360°, θ + δ), binocular ≈ θ − δ (in front) and θ + δ − 360° (behind).
// Presets are approximate published values (total field, binocular field); see the speaker notes for sources.
// config: { preset: "human", size: 400 }
import { C, h, fmt } from './util.js';
import { LIGHT } from '../theme.js';

// total horizontal field T and binocular overlap O (degrees) → θ = (T + O) / 2, δ = (T − O) / 2; b in pixels
const PRESETS = {
  human: { name: 'human', T: 200, O: 120, b: 34, head: 'round' },
  cat: { name: 'cat', T: 200, O: 130, b: 28, head: 'round' },
  owl: { name: 'tawny owl', T: 201, O: 48, b: 30, head: 'round' },
  horse: { name: 'horse', T: 350, O: 60, b: 50, head: 'long' },
  rabbit: { name: 'rabbit', T: 360, O: 24, b: 34, head: 'long' },
  pigeon: { name: 'pigeon', T: 340, O: 27, b: 22, head: 'round' },
};

const hex = s => [1, 3, 5].map(i => parseInt(s.slice(i, i + 2), 16));
// (light colours avoid the exact values in theme.js' table, which would recolour the legend swatches only)
const COL = LIGHT
  ? { bg: hex('#f3f4f6'), both: hex('#e9c46a'), left: hex('#aacce9'), right: hex('#a8dcb9'), blind: hex('#d5d8de') }
  : { bg: hex('#1e2128'), both: hex('#b9862a'), left: hex('#2f5a82'), right: hex('#2f6b49'), blind: hex('#15171c') };

export function mount(el, cfg) {
  const S = cfg.size || 400;
  const p0 = PRESETS[cfg.preset || 'human'];
  const state = { theta: (p0.T + p0.O) / 2, delta: (p0.T - p0.O) / 2, b: p0.b, head: p0.head, preset: cfg.preset || 'human' };

  const canvas = h('canvas', { width: S, height: S, style: `width:${S}px; height:${S}px; display:block; border:1px solid ${C.line}; border-radius:8px` });
  const g = canvas.getContext('2d');
  const im = new ImageData(S, S);
  const readout = h('div', { style: 'line-height:1.45' });
  const cx = S / 2, cy = S * 0.66, deg = Math.PI / 180;

  // angle of a direction measured from straight ahead (up on the canvas), clockwise positive
  const ang = (dx, dy) => Math.atan2(dx, -dy);
  const wrap = a => { a = (a + Math.PI) % (2 * Math.PI); return (a < 0 ? a + 2 * Math.PI : a) - Math.PI; };

  function draw() {
    const th = state.theta * deg / 2, ax = state.delta * deg / 2, eL = [cx - state.b / 2, cy], eR = [cx + state.b / 2, cy];
    for (let y = 0; y < S; y++) for (let x = 0; x < S; x++) {
      const l = Math.abs(wrap(ang(x - eL[0], y - eL[1]) + ax)) <= th;   // left eye looks to the left: axis −δ/2
      const r = Math.abs(wrap(ang(x - eR[0], y - eR[1]) - ax)) <= th;
      const c = l && r ? COL.both : l ? COL.left : r ? COL.right : COL.blind;
      const i = 4 * (y * S + x); im.data[i] = c[0]; im.data[i + 1] = c[1]; im.data[i + 2] = c[2]; im.data[i + 3] = 255;
    }
    g.putImageData(im, 0, 0);

    // head
    g.fillStyle = C.bg3; g.strokeStyle = C.dim; g.lineWidth = 2; g.beginPath();
    if (state.head === 'long') g.ellipse(cx, cy - 4, Math.max(18, state.b / 2 + 6), 34, 0, 0, 7);
    else g.ellipse(cx, cy, Math.max(18, state.b / 2 + 8), 24, 0, 0, 7);
    g.fill(); g.stroke();
    // eyes and their visual axes
    for (const [e, a, col] of [[eL, -ax, C.blue], [eR, ax, C.green]]) {
      g.strokeStyle = col; g.lineWidth = 2; g.setLineDash([6, 4]);
      g.beginPath(); g.moveTo(e[0], e[1]); g.lineTo(e[0] + 60 * Math.sin(a), e[1] - 60 * Math.cos(a)); g.stroke(); g.setLineDash([]);
      g.fillStyle = col; g.beginPath(); g.arc(e[0], e[1], 5, 0, 7); g.fill();
    }
    // labels
    g.font = '15px Inter, Arial'; g.fillStyle = C.fg; g.textAlign = 'center';
    g.fillText('ahead', cx, 18);
    g.textAlign = 'left';

    const T = Math.min(360, state.theta + state.delta), O = Math.max(0, state.theta - state.delta), back = Math.max(0, state.theta + state.delta - 360);
    const p = PRESETS[state.preset];
    readout.innerHTML =
      `<div><b>${p ? p.name : 'custom'}</b></div>` +
      `<div>total field ≈ <b>${fmt(T, 0)}°</b></div>` +
      `<div style="color:${C.accent}">binocular (stereo) ≈ <b>${fmt(O, 0)}°</b>${back > 0 ? ` (+ ${fmt(back, 0)}° behind)` : ''}</div>` +
      `<div class="dim">blind ≈ ${fmt(360 - T, 0)}°</div>`;
  }

  const legend = h('div', { style: 'display:flex; flex-direction:column; gap:0.15em; font-size:0.85em' },
    ...[['both', 'both eyes: stereo'], ['left', 'left eye only'], ['right', 'right eye only'], ['blind', 'blind']].map(([k, t]) =>
      h('div', { style: 'display:flex; gap:0.4em; align-items:center' },
        h('span', { style: `display:inline-block; width:1.1em; height:0.8em; border-radius:2px; background:rgb(${COL[k].join(',')}); border:1px solid ${C.line}` }), t)));

  const sliders = [];
  const slider = (key, min, max, step, name, unit) => {
    const s = h('input', { type: 'range', min, max, step, value: state[key], style: 'width:7.5em' });
    const val = h('span', { class: 'readout', style: 'display:inline-block; width:3.4em; text-align:right' });
    const upd = () => { s.value = state[key]; val.textContent = fmt(state[key], 0) + unit; };
    s.addEventListener('input', () => { state[key] = parseFloat(s.value); state.preset = null; btns.forEach(b => b.classList.remove('active')); upd(); draw(); });
    upd(); sliders.push(upd);
    return h('div', { style: 'display:flex; align-items:center; gap:0.35em' }, h('label', { style: 'width:5.6em; flex:none' }, name), s, val);
  };
  const btns = Object.entries(PRESETS).map(([k, p]) => {
    const b = h('button', { onclick: () => {
      Object.assign(state, { theta: (p.T + p.O) / 2, delta: (p.T - p.O) / 2, b: p.b, head: p.head, preset: k });
      btns.forEach(x => x.classList.toggle('active', x === b)); sliders.forEach(u => u()); draw();
    } }, p.name);
    if (k === state.preset) b.classList.add('active');
    return b;
  });

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    canvas,
    h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:15.5em; flex:none' },
      readout, legend,
      h('div', { class: 'wctl interactive-only', style: 'gap:0.25em; margin-top:0' }, ...btns),
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.1em; margin-top:0' },
        slider('theta', 90, 220, 1, 'one eye θ', '°'),
        slider('delta', 0, 180, 1, 'axes apart δ', '°'),
        slider('b', 10, 180, 1, 'baseline', ' px')))));
  draw();
}
