// What a magnetometer on a small fixed-wing reads while the aircraft rotates.
// World (scene) frame: x = north, y = west, z = up; body frame: x forward, y left, z up. R = attitude (body → world).
// The Earth field e is fixed in the world (grey dashed). The sensor reports a vector in the body frame:
//   ideal:     m = Rᵀ e
//   hard iron: m = Rᵀ e + h              (h: a magnet on the aircraft, fixed in the body — red arrows: one on the
//                                          aircraft at the compass, one moved to the tip of e to show e + R h)
//   soft iron: m = (I + k u uᵀ) Rᵀ e     (a steel wing spar along u = body y gathers the field along its length,
//                                          k exaggerated for visibility — purple arrow = the induced part)
// The reading is drawn back in the world frame (R m, accent) so it can be compared with e; the trail is the path of
// its tip during the last seconds. Ideal: R m = e always, the tip does not move.
// Time comes from the page clock, so several copies on one slide rotate in sync.
//
// config: { effect: "ideal" | "hard" | "soft", width, height, compact (false), motion ("tumble" | "turn"), city }
import { C, h, fmt, isPrint } from './util.js';
import { THREE, makeStage, label, line, arrow3 } from './three-util.js';
import { T, setRotation } from './gripper.js';
import { CITIES, fieldNED, euler, transpose, matvec, deg } from './magutil.js';

const U = 2.3 / 500;                                    // scene units per mG
const HARD = (() => { const d = [0.75, -0.25, 0.45], l = Math.hypot(...d); return d.map(v => v * 230 / l); })(); // mG, body
const K_SOFT = 0.8;                                     // exaggerated: real soft iron is a few %
const TRAIL = 220;

const ang = (a, b) => Math.acos(Math.max(-1, Math.min(1, (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (Math.hypot(...a) * Math.hypot(...b))))) / deg;

function box(sx, sy, sz, color, at) {                  // sizes and position in the math (body) frame
  const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sz, sy), new THREE.MeshStandardMaterial({ color, roughness: 0.6 }));
  m.position.set(...T(...at));
  return m;
}

function makePlane(effect) {
  const g = new THREE.Group();
  g.add(box(1.5, 0.14, 0.14, 0x8a93a6, [-0.1, 0, 0]));                   // fuselage
  g.add(box(0.32, 2.0, 0.03, 0xc9ccd4, [0.05, 0, 0.04]));                // wing
  g.add(box(0.2, 0.7, 0.025, 0xc9ccd4, [-0.78, 0, 0.04]));               // horizontal tail
  g.add(box(0.22, 0.025, 0.28, 0xc9ccd4, [-0.78, 0, 0.19]));             // fin
  const prop = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, 0.01, 24), new THREE.MeshStandardMaterial({ color: 0x5a606c, transparent: true, opacity: 0.6 }));
  prop.rotation.z = Math.PI / 2; prop.position.set(...T(0.67, 0, 0)); g.add(prop);
  g.add(box(0.1, 0.07, 0.04, 0x5fd38d, [0.15, 0, 0.1]));                 // compass (GPS mast module)
  if (effect === 'hard') {                                               // a magnet near the nose (motor)
    g.add(box(0.09, 0.08, 0.08, 0xff6b6b, [0.5, 0, 0.0]));
    g.add(box(0.09, 0.08, 0.08, 0x5ab0ff, [0.41, 0, 0.0]));
    // h drawn on the aircraft itself, starting at the compass: it is fixed in the body and turns with the plane
    const u = HARD.map(v => v / Math.hypot(...HARD)), c0 = [0.15, 0, 0.12];
    g.add(arrow3(T(...c0), T(...c0.map((v, i) => v + 0.55 * u[i])), 0xff6b6b, 0.1));
  }
  if (effect === 'soft') g.add(box(0.035, 2.1, 0.035, 0xb28dff, [0.05, 0, 0.065])); // steel spar along the wing
  // body axes (small)
  [[[0.9, 0, 0], 0xff6b6b], [[0, 0.6, 0], 0x5fd38d], [[0, 0, 0.5], 0x5ab0ff]].forEach(([d, c]) => g.add(arrow3(T(0, 0, 0), T(...d), c, 0.08)));
  return g;
}

function onTop(obj) {
  obj.traverse(o => { if (o.material) { o.material.depthTest = false; o.material.transparent = true; } o.renderOrder = 8; });
  return obj;
}

export function mount(el, cfg) {
  const effect = cfg.effect || 'ideal', compact = !!cfg.compact;
  const W = cfg.width || 560, H = cfg.height || 420;
  const city = CITIES[cfg.city || 'Warsaw'];
  const eN = fieldNED(city), E = [eN[0], -eN[1], -eN[2]];   // world frame (north, west, up)
  const motion = cfg.motion || 'tumble';

  const canvas = h('canvas', { width: W, height: H });
  const { scene, render } = makeStage(canvas, { position: T(-2.7, -3.4, 1.2), target: T(0.3, 0, -0.8), fov: compact ? 46 : 40 });
  const grid = new THREE.GridHelper(6, 12, 0x3a3f4b, 0x2a2e36); grid.position.y = -2.6; scene.add(grid);
  const nl = label('N', { color: C.dim, size: 0.32, font: '600 64px Inter, Arial' }); nl.position.set(...T(3.2, 0, -2.6)); scene.add(nl);
  const model = makePlane(effect); model.scale.setScalar(1.5);
  const plane = new THREE.Group(); plane.add(model); scene.add(plane);  // outer group carries the attitude

  const eArrow = onTop(arrow3(T(0, 0, 0), T(...E.map(v => v * U)), 0x9aa1ae, 0.16)); scene.add(eArrow);
  const dyn = new THREE.Group(); scene.add(dyn);
  if (!compact) { const le = label('e', { color: C.dim, size: 0.3 }); le.position.set(...T(E[0] * U + 0.2, E[1] * U, E[2] * U)); scene.add(le); }
  const trail = [];

  const readout = h('div', { style: 'line-height:1.45' });
  const state = { playing: !isPrint(), t: isPrint() ? 2.2 : 0, offset: 0 };

  function reading(R) {
    const b = matvec(transpose(R), E);                      // field in the body frame
    if (effect === 'hard') return { m: b.map((v, i) => v + HARD[i]), extra: HARD };
    if (effect === 'soft') { const ind = [0, K_SOFT * b[1], 0]; return { m: b.map((v, i) => v + ind[i]), extra: ind }; }
    return { m: b, extra: [0, 0, 0] };
  }

  let lastText = 0;
  function frame(t) {
    const a = motion === 'turn' ? [0, 0, 30 * t] : [45 * Math.sin(0.7 * t), 30 * Math.sin(0.5 * t + 1), 25 * t];
    const R = euler(a[0] * deg, a[1] * deg, a[2] * deg);
    setRotation(plane, R.flat());
    const { m, extra } = reading(R);
    const mw = matvec(R, m), xw = matvec(R, extra);         // back in the world frame
    dyn.traverse(o => { o.geometry?.dispose(); o.material?.dispose(); });
    dyn.clear();
    if (effect !== 'ideal') dyn.add(onTop(arrow3(T(...E.map(v => v * U)), T(...mw.map(v => v * U)), effect === 'hard' ? 0xff6b6b : 0xb28dff, 0.14)));
    dyn.add(onTop(arrow3(T(0, 0, 0), T(...mw.map(v => v * U)), 0xf2b134, 0.2)));
    trail.push(T(...mw.map(v => v * U))); if (trail.length > TRAIL) trail.shift();
    if (trail.length > 1) dyn.add(onTop(line(trail, 0xf2b134)));
    render();

    const now = performance.now();
    if (now - lastText > 120 || !state.playing) {
      lastText = now;
      const mag = Math.hypot(...m), dirErr = ang(mw, E);
      const hm = Math.atan2(mw[1], mw[0]), he = Math.atan2(E[1], E[0]);
      const hdgErr = ((hm - he) / deg + 540) % 360 - 180;
      readout.innerHTML = compact
        ? `<span class="readout" style="font-size:0.9em">|m| ${fmt(mag, 0)} mG · heading err. ${fmt(hdgErr, 0)}°</span>`
        : `<div>$|m| = ${fmt(mag, 0)}$ mG <span class="dim">(F = ${city.F})</span></div>` +
          `<div>direction error ${fmt(dirErr, 0)}°</div>` +
          `<div>heading error <b>${fmt(hdgErr, 0)}°</b></div>` +
          `<div class="dim" style="margin-top:0.3em">roll ${fmt(a[0], 0)}°, pitch ${fmt(a[1], 0)}°, yaw ${fmt(((a[2] % 360) + 360) % 360, 0)}°</div>`;
      if (!compact) window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
    }
  }

  const loop = () => {
    if (!canvas.isConnected) return;
    if (state.playing) { state.t = performance.now() / 1000 - state.offset; frame(state.t); }
    requestAnimationFrame(loop);
  };
  const playBtn = h('button', {
    onclick: () => {
      state.playing = !state.playing;
      if (state.playing) state.offset = performance.now() / 1000 - state.t;
      playBtn.textContent = state.playing ? '❚❚ pause' : '▶ play';
    },
  }, state.playing ? '❚❚ pause' : '▶ play');
  const clearBtn = h('button', { onclick: () => { trail.length = 0; } }, 'clear trail');

  const legend = {
    ideal: `<span style="color:${C.dim}">grey: Earth field $e$</span> · <span style="color:${C.accent}">yellow: the reading $R\\,m$</span>`,
    hard: `<span style="color:${C.dim}">grey: $e$</span> · <span style="color:${C.red}">red: hard iron $R\\,h$</span> · <span style="color:${C.accent}">yellow: reading</span>`,
    soft: `<span style="color:${C.dim}">grey: $e$</span> · <span style="color:${C.purple}">purple: induced by the spar</span> · <span style="color:${C.accent}">yellow: reading</span>`,
  }[effect];
  const leg = h('div', { class: 'dim', style: 'margin-top:0.2em' }); leg.innerHTML = legend;
  window.renderMathInElement?.(leg, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });

  el.classList.add('widget');
  if (compact) {
    el.append(h('div', { style: `width:${W}px` }, canvas, readout, h('div', { class: 'wctl interactive-only' }, playBtn, clearBtn)));
  } else {
    el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
      h('div', { style: `width:${W}px; flex:none` }, canvas, leg),
      h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:13em; flex:none' },
        readout, h('div', { class: 'wctl interactive-only' }, playBtn, clearBtn))));
  }
  // the page clock (offset 0) keeps several copies on one slide in sync
  frame(state.playing ? performance.now() / 1000 : state.t);
  if (!isPrint()) requestAnimationFrame(loop);
}
