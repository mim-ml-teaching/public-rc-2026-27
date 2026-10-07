// Why a pinhole: side view of object → barrier with a hole → sensor, and what the sensor records.
// For every sensor point we integrate over the hole: the ray from the sensor point through a hole point
// hits the object at y_o = y_s + (y_h − y_s)·(a + d)/d. The record is the average colour (and brightness).
// config: { hole: 0.06, barrier: true, width, height }
import { C, h, fmt } from './util.js';

const BANDS = [[255, 107, 107], [242, 177, 52], [95, 211, 141], [90, 176, 255]]; // object top → bottom
const objColor = y => BANDS[Math.min(3, Math.max(0, Math.floor((1 - y) * 2)))]; // y in [−1, 1], top = +1

export function mount(el, cfg) {
  const W = cfg.width || 640, H = cfg.height || 340;
  const state = { hole: cfg.hole ?? 0.06, barrier: cfg.barrier ?? true };
  const canvas = h('canvas', { width: W, height: H });
  const g = canvas.getContext('2d');
  const rec = h('canvas', { width: 90, height: 260, style: 'width:90px; height:260px' });
  const gr = rec.getContext('2d');
  const caption = h('div', { class: 'dim', style: 'max-width:12em; min-height:3em' });

  // world: object at x = 0, barrier at x = A, sensor at x = A + D; y up, object spans [−1, 1]
  const A = 1.7, D = 1.7, S = 112;                  // S = px per unit; magnification D/A = 1
  const X0 = 50, Y0 = H / 2;
  const toC = (x, y) => [X0 + x * S, Y0 - y * S];
  const SENSOR = 1.3;                               // sensor half-height

  // colour recorded at sensor height ys: average of the object points seen through the hole,
  // scaled by the light gathered (∝ hole size; the default hole gives full brightness)
  const MEAN = [0, 1, 2].map(k => BANDS.reduce((a, c) => a + c[k], 0) / 4);
  function record(ys) {
    if (!state.barrier) return MEAN.map(Math.round);   // every sensor point sees the whole object
    const w = state.hole, N = 200;
    let r = 0, gg = 0, b = 0, n = 0;
    for (let i = 0; i < N; i++) {
      const yh = -w + 2 * w * (i + 0.5) / N;
      const yo = ys + (yh - ys) * (A + D) / D;
      if (yo < -1 || yo > 1) continue;
      const c = objColor(yo); r += c[0]; gg += c[1]; b += c[2]; n++;
    }
    if (!n) return [13, 15, 19];
    const light = state.barrier ? Math.min(1, (2 * w * n / N) / 0.1) : 1;
    return [r, gg, b].map(v => Math.round(13 + (v / n - 13) * light));
  }

  function draw() {
    g.fillStyle = C.bg2; g.fillRect(0, 0, W, H);
    // object: colour bands
    for (let i = 0; i < 4; i++) {
      const [x, y] = toC(0, 1 - i * 0.5);
      g.fillStyle = `rgb(${BANDS[i]})`; g.fillRect(x - 10, y, 20, 0.5 * S);
    }
    g.fillStyle = C.dim; g.font = '15px Inter, Arial';
    g.fillText('object', X0 - 24, Y0 + 1.2 * S);

    // rays from three object points (top, middle, bottom)
    const w = state.barrier ? state.hole : 3;
    for (const [yo, ci] of [[0.95, 0], [0, 2], [-0.95, 3]]) {
      const col = `rgba(${BANDS[ci]},0.35)`;
      const ys = yh => yo + (yh - yo) * (A + D) / A;  // where the ray through hole point yh hits the sensor
      const [ox, oy] = toC(0, yo);
      if (state.barrier) {
        g.fillStyle = col; g.beginPath();
        const [h1x, h1y] = toC(A, -w), [h2x, h2y] = toC(A, w);
        const [s1x, s1y] = toC(A + D, ys(-w)), [s2x, s2y] = toC(A + D, ys(w));
        g.moveTo(ox, oy); g.lineTo(h1x, h1y); g.lineTo(s1x, s1y); g.lineTo(s2x, s2y); g.lineTo(h2x, h2y); g.closePath(); g.fill();
      } else {
        // light from this point reaches the whole sensor
        g.fillStyle = `rgba(${BANDS[ci]},0.12)`; g.beginPath();
        const [s1x, s1y] = toC(A + D, -SENSOR), [s2x, s2y] = toC(A + D, SENSOR);
        g.moveTo(ox, oy); g.lineTo(s1x, s1y); g.lineTo(s2x, s2y); g.closePath(); g.fill();
      }
    }

    // barrier with a hole
    if (state.barrier) {
      g.fillStyle = '#5a606c';
      const [bx] = toC(A, 0), bt = 0, [, ht] = toC(A, state.hole), [, hb] = toC(A, -state.hole), bb = H;
      g.fillRect(bx - 4, bt, 8, ht - bt); g.fillRect(bx - 4, hb, 8, bb - hb);
      g.fillStyle = C.dim; g.fillText('barrier with a hole', bx + 10, 20);
    }
    // sensor: recorded colours along it
    const [sx] = toC(A + D, 0);
    for (let py = 0; py < 2 * SENSOR * S; py++) {
      const ys = SENSOR - py / S;
      g.fillStyle = `rgb(${record(ys)})`; g.fillRect(sx - 3, Y0 - SENSOR * S + py, 9, 1);
    }
    g.fillStyle = C.dim; g.fillText('sensor', sx + 12, Y0 + SENSOR * S - 4);

    // the record, enlarged
    gr.fillStyle = '#0d0f13'; gr.fillRect(0, 0, rec.width, rec.height);
    for (let py = 0; py < rec.height; py++) {
      const ys = SENSOR - py / rec.height * 2 * SENSOR;
      gr.fillStyle = `rgb(${record(ys)})`; gr.fillRect(10, py, rec.width - 20, 1);
    }
    caption.innerHTML = !state.barrier
      ? '<span style="color:#ff6b6b">no barrier</span>: every sensor point sees the whole object — just a uniform smear'
      : state.hole < 0.12
        ? '<span style="color:#5fd38d">small hole</span>: one ray per sensor point — a sharp image, <b>upside down</b>'
        : '<span style="color:#f2b134">large hole</span>: brighter, but each sensor point sees part of the object — blurred';
  }

  const slider = h('input', { type: 'range', min: 0.02, max: 0.9, step: 0.01, value: state.hole });
  slider.addEventListener('input', () => { state.hole = parseFloat(slider.value); draw(); });

  el.classList.add('widget');
  el.append(h('div', { style: 'display:flex; gap:0.8em; align-items:flex-start' },
    canvas,
    h('div', { style: 'display:flex; flex-direction:column; align-items:flex-start; gap:0.5em; width:14em; flex:none' },
      h('div', { class: 'dim' }, 'what the sensor records'), rec, caption,
      h('div', { class: 'wctl interactive-only', style: 'flex-direction:column; align-items:flex-start; gap:0.3em' },
        h('div', {}, h('label', {}, 'hole size '), slider)))));
  draw();
}
