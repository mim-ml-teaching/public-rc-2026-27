// The local Earth magnetic field as a 3D vector: total intensity F, inclination I (below the horizon), declination D
// (magnetic north east of true north), horizontal part H = F cos I (green) and vertical part Z = F sin I (blue).
// Values for five cities from ArduPilot's built-in table (magutil.CITIES).
// Scene frame: x = true north, y = west, z = up; the field arrow is drawn with length ∝ F.
//
// config: { width, height, city ("Warsaw") }
import { C, h, fmt } from './util.js';
import { THREE, makeStage, viewButtons, label, line, arrow3 } from './three-util.js';
import { T } from './gripper.js';
import { CITIES, deg } from './magutil.js';

const L = 2.6 / 520; // scene units per mG

export function mount(el, cfg) {
  const W = cfg.width || 560, H = cfg.height || 430;
  const state = { city: cfg.city || 'Warsaw' };
  const canvas = h('canvas', { width: W, height: H });
  const view3d = { label: '3D view', position: T(-2.0, -4.6, 1.9), target: T(1.0, -0.1, -0.9), fov: 40 };
  // straight down, north up: nudge the camera slightly south so that "up" on screen is north
  const viewTop = { label: 'top view (north up)', position: T(1.39, 0, 5.4), target: T(1.4, 0, 0), fov: 40 };
  const stage = makeStage(canvas, { position: view3d.position, target: view3d.target, fov: 40 });
  const { scene, render } = stage;

  const grid = new THREE.GridHelper(6, 12, 0x3a3f4b, 0x2a2e36); scene.add(grid);       // ground z = 0
  const ground = new THREE.Mesh(new THREE.CircleGeometry(3, 48), new THREE.MeshBasicMaterial({ color: 0x5a606c, transparent: true, opacity: 0.12, side: THREE.DoubleSide }));
  ground.rotation.x = -Math.PI / 2; scene.add(ground);
  for (const [txt, p] of [['N', [3.2, 0, 0]], ['S', [-3.2, 0, 0]], ['E', [0, -3.2, 0]], ['W', [0, 3.2, 0]]]) {
    const l = label(txt, { color: C.dim, size: 0.3, font: '600 64px Inter, Arial' }); l.position.set(...T(...p)); scene.add(l);
  }
  scene.add(line([T(0, 0, 0), T(2.9, 0, 0)], 0xe7e9ee));
  const tn = label('true north', { color: C.fg, size: 0.22, font: '56px Inter, Arial' }); tn.position.set(...T(2.4, 0.45, 0.12)); scene.add(tn);

  const dyn = new THREE.Group(); scene.add(dyn);
  const readout = h('div', { style: 'line-height:1.5' });

  function arc(center, r, a0, a1, plane, color) { // plane(a) → unit direction
    const pts = [];
    for (let i = 0; i <= 40; i++) { const a = a0 + (a1 - a0) * i / 40, d = plane(a); pts.push(T(...center.map((c, k) => c + r * d[k]))); }
    return line(pts, color, { width: 2 });
  }

  function draw() {
    const c = CITIES[state.city], d = c.D * deg, inc = c.I * deg;
    const Hm = c.F * Math.cos(inc), Zm = c.F * Math.sin(inc);
    const hdir = [Math.cos(d), -Math.sin(d), 0];                    // magnetic north (y = west, so east is −y)
    const hTip = hdir.map(v => v * Hm * L), fTip = [hTip[0], hTip[1], -Zm * L];
    dyn.clear();
    dyn.add(line([T(0, 0, 0), T(...hdir.map(v => v * 2.9))], 0xb28dff, { dashed: true }));
    dyn.add(arrow3(T(0, 0, 0), T(...hTip), 0x5fd38d, 0.16));
    dyn.add(arrow3(T(...hTip), T(...fTip), 0x5ab0ff, 0.16));
    dyn.add(arrow3(T(0, 0, 0), T(...fTip), 0xf2b134, 0.2));
    // declination arc on the ground, inclination arc in the vertical plane through magnetic north
    dyn.add(arc([0, 0, 0], 1.5, 0, -d, a => [Math.cos(a), Math.sin(a), 0], 0xb28dff));
    dyn.add(arc([0, 0, 0], 0.75, 0, -inc, a => [hdir[0] * Math.cos(a), hdir[1] * Math.cos(a), Math.sin(a)], 0xf2b134));
    const put = (txt, p, color, size = 0.28) => { const s = label(txt, { color, size }); s.position.set(...T(...p)); dyn.add(s); };
    put('D', [1.75, -0.2, 0.18], C.purple);
    put('I', [0.95 * hdir[0], 0.95 * hdir[1], -0.45], C.accent);
    put('B', [fTip[0] * 0.55 - 0.25, fTip[1] * 0.55, fTip[2] * 0.55], C.accent, 0.32);
    put('H', [hTip[0] * 0.6, hTip[1] * 0.6 - 0.25, 0.18], C.green);
    put('Z', [hTip[0] + 0.22, hTip[1], fTip[2] / 2], C.blue);
    const lab = label('magnetic north', { color: C.purple, size: 0.22, font: '56px Inter, Arial' });
    lab.position.set(...T(...hdir.map(v => v * 2.5).map((v, k) => v + [0, -0.55, 0.12][k]))); dyn.add(lab);

    readout.innerHTML = `<table style="font-size:1em; margin:0">
      <tr><td>total $F$</td><td class="readout" style="white-space:nowrap">${c.F} mG</td></tr>
      <tr><td>inclination $I$</td><td class="readout" style="white-space:nowrap">${fmt(c.I, 1)}°</td></tr>
      <tr><td>declination $D$</td><td class="readout" style="white-space:nowrap">+${fmt(c.D, 1)}°</td></tr>
      <tr><td style="color:${C.green}">horizontal $H = F\\cos I$</td><td class="readout" style="white-space:nowrap">${fmt(Hm, 0)} mG</td></tr>
      <tr><td style="color:${C.blue}">vertical $Z = F\\sin I$</td><td class="readout" style="white-space:nowrap">${fmt(Zm, 0)} mG</td></tr></table>
      <div style="margin-top:0.4em">Only $H$ points north: <b>${fmt(100 * Hm / c.F, 0)} %</b> of the field.</div>`;
    window.renderMathInElement?.(readout, { delimiters: [{ left: '$', right: '$', display: false }], throwOnError: false });
    render();
  }

  const btns = Object.keys(CITIES).map(name => {
    const b = h('button', { onclick: () => { state.city = name; btns.forEach(x => x.classList.toggle('active', x === b)); draw(); } }, name);
    b.classList.toggle('active', name === state.city);
    return b;
  });
  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    h('div', { style: `width:${W}px; flex:none` }, canvas,
      h('div', { class: 'wctl interactive-only' }, viewButtons(stage, [view3d, viewTop]),
        h('span', { class: 'dim', style: 'margin-left:0.4em' }, 'drag to rotate · ground = horizontal plane'))),
    h('div', { style: 'display:flex; flex-direction:column; gap:0.5em; width:15em; flex:none' },
      h('div', { class: 'wctl interactive-only', style: 'gap:0.3em; margin-top:0' }, btns), readout)));
  draw();
}
