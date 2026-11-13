// Hover flight time of a multicopter against battery mass (momentum theory).
//   total mass m = m_frame + m_batt, thrust T = m g, N rotors of diameter d, disk area A = N π d² / 4
//   ideal hover power P_ideal = T^1.5 / sqrt(2 ρ A); electrical power P = P_ideal / (FM · η)
//   usable energy E = m_batt · e · DoD; flight time t = E / P
// t(m_batt) ∝ m_batt / (m_frame + m_batt)^1.5 has its maximum at m_batt = 2 m_frame.
// The motors are assumed to give a maximum thrust of `twFrame` × the frame mass (kgf); above m = max thrust / 2
// the thrust-to-weight ratio drops below 2:1 (shaded). The top panel shows the hover power for every battery mass
// (∝ (m_frame + m_batt)^1.5, while the energy grows only linearly with m_batt); the bottom panel the hover time.
// config: { width: 1100, height: 400, frame: 1, batt: 0.5, prop: 10, n: 4, e: 150, dod: 0.8, fm: 0.6, eta: 0.8, twFrame: 4 }
import { C, h, fmt } from './util.js';
import { S, plot } from './plotsvg.js';

export function mount(el, cfg) {
  el.classList.add('widget');
  const W = cfg.width || 1100, H = cfg.height || 400;
  const st = { frame: cfg.frame ?? 1, batt: cfg.batt ?? 0.5, prop: cfg.prop ?? 10 };
  const n = cfg.n || 4, e = cfg.e || 150, dod = cfg.dod || 0.8, fm = cfg.fm || 0.6, eta = cfg.eta || 0.8, tw = cfg.twFrame || 4;
  const g = 9.81, rho = 1.225;
  const power = (m, prop) => {
    const T = m * g, d = prop * 0.0254, A = n * Math.PI * d * d / 4;
    return Math.pow(T, 1.5) / Math.sqrt(2 * rho * A) / (fm * eta);
  };
  const minutes = (mb, mf, prop) => (mb * e * dod) / power(mf + mb, prop) * 60;

  const svg = S('svg', { width: W - 330, height: H, viewBox: `0 0 ${W - 330} ${H}`, style: 'display:block; flex:none' });
  const read = h('div', { class: 'readout', style: 'line-height:1.55; width:300px' });
  const rows = [];
  const slider = (key, label, min, max, step, unit) => {
    const v = h('span', { class: 'readout', style: 'display:inline-block; width:4.2em; text-align:right' });
    const s = h('input', { type: 'range', min, max, step, value: st[key], style: 'width:8.5em', oninput: ev => { st[key] = +ev.target.value; draw(); } });
    rows.push(h('div', {}, h('label', { style: 'display:inline-block; width:5.6em' }, label), s, v));
    return () => { v.textContent = `${fmt(st[key], 2)} ${unit}`; };
  };
  const upd = [slider('frame', 'frame', 0.3, 3, 0.05, 'kg'), slider('batt', 'battery', 0.05, 4, 0.05, 'kg'), slider('prop', 'propeller', 5, 15, 0.5, '"')];
  const ctl = h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.2em; width:300px' }, rows);
  el.append(h('div', { style: 'display:flex; gap:1.2em; align-items:flex-start' }, svg, h('div', { style: 'font-size:0.82em; width:300px; flex:none' }, read, ctl)));

  function draw() {
    svg.replaceChildren();
    const w = W - 330 - 90, xmax = 4, ymax = 50;
    const h1 = Math.round((H - 120) * 0.36), y1 = 20, y0 = y1 + h1 + 42, ph = H - 60 - y0;   // power panel, time panel
    const xs = [], ys = [], ps = [];
    for (let i = 1; i <= 400; i++) { const mb = i / 100; xs.push(mb); ys.push(minutes(mb, st.frame, st.prop)); ps.push(power(st.frame + mb, st.prop)); }
    const pmax = Math.max(400, Math.ceil(Math.max(...ps) / 400) * 400);
    const lim = tw * st.frame / 2 - st.frame;             // battery mass at which T/W = 2
    const { Y: Yp } = plot(svg, { x0: 60, y0: y1, w, h: h1, xmin: 0, xmax, ymin: 0, ymax: pmax, yticks: [0, pmax / 2, pmax],
      ylabel: 'hover power (W)', series: [] });
    const { X, Y } = plot(svg, { x0: 60, y0, w, h: ph, xmin: 0, xmax, ymin: 0, ymax, xticks: [0, 1, 2, 3, 4], yticks: [0, 10, 20, 30, 40, 50],
      xlabel: 'battery mass (kg)', ylabel: 'hover time (min) = energy / power', series: [] });
    if (lim < xmax) {
      const x0 = X(Math.max(0, lim));
      for (const [yy, hh] of [[y1, h1], [y0, ph]]) {
        svg.append(S('rect', { x: x0, y: yy, width: X(xmax) - x0, height: hh, fill: C.red, 'fill-opacity': 0.1 }));
        svg.append(S('line', { x1: x0, x2: x0, y1: yy, y2: yy + hh, stroke: C.red, 'stroke-dasharray': '6 4' }));
      }
      svg.append(S('text', { x: x0 + 8, y: y0 + 22, fill: C.red, 'font-size': 15 }, 'thrust-to-weight below 2:1'));
    }
    plot(svg, { x0: 60, y0: y1, w, h: h1, xmin: 0, xmax, ymin: 0, ymax: pmax, bare: true, series: [{ xs, ys: ps, color: C.red, width: 3 }] });
    plot(svg, { x0: 60, y0, w, h: ph, xmin: 0, xmax, ymin: 0, ymax, bare: true, series: [{ xs, ys, color: C.blue, width: 3 }] });
    // optimum (m_batt = 2 m_frame) and the current battery
    const opt = 2 * st.frame;
    if (opt <= xmax) {
      const to = minutes(opt, st.frame, st.prop);
      svg.append(S('circle', { cx: X(opt), cy: Y(to), r: 7, fill: 'none', stroke: C.green, 'stroke-width': 2.5 }));
      svg.append(S('text', { x: X(opt), y: Y(to) - 14, fill: C.green, 'font-size': 15, 'text-anchor': 'middle' }, `longest: ${fmt(to, 0)} min at ${fmt(opt, 1)} kg`));
    }
    const t = minutes(st.batt, st.frame, st.prop);
    const m = st.frame + st.batt, P = power(m, st.prop), E = st.batt * e * dod, twr = tw * st.frame / m;
    svg.append(S('line', { x1: X(st.batt), x2: X(st.batt), y1: Yp(P), y2: y0 + ph, stroke: C.accent, 'stroke-dasharray': '4 4' }));
    svg.append(S('circle', { cx: X(st.batt), cy: Yp(P), r: 7, fill: C.accent }));
    svg.append(S('text', { x: X(st.batt) + 12, y: Yp(P) - 8, fill: C.accent, 'font-size': 15 }, `${fmt(P, 0)} W`));
    svg.append(S('circle', { cx: X(st.batt), cy: Y(t), r: 8, fill: C.accent }));
    read.innerHTML =
      `total mass <b>${fmt(m, 2)} kg</b><br>hover thrust ${fmt(m * 9.81, 1)} N<br>` +
      `hover power <b>${fmt(P, 0)} W</b><br>` +
      `usable energy ${fmt(E, 0)} Wh<br>` +
      `<span style="color:${C.accent}">hover time <b>${fmt(t, 1)} min</b></span><br>` +
      `<span style="color:${twr < 2 ? C.red : C.fg}">thrust-to-weight ${fmt(twr, 1)} : 1</span>` +
      `<div class="dim" style="font-size:0.85em; line-height:1.3; margin:0.3em 0">${e} Wh/kg, ${Math.round(dod * 100)}% used, FM ${fm}, η ${eta}; motors: max thrust ${tw}× the frame mass</div>`;
    upd.forEach(f => f());
  }
  draw();
}
