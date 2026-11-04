// A travelling impulse becomes a response at the block's output.
// In sum mode, the lower trace is the pointwise sum of the coloured traces.
import { C, h, isPrint } from './util.js';

export function mount(el, cfg) {
  const cascade = cfg.mode === 'cascade';
  const multiple = cfg.mode === 'sum';
  const W = 1200, H = cascade ? 400 : multiple ? 380 : 390;
  const left = 460, right = 610, end = 1160, speed = 100;
  const baseline = multiple ? 165 : 255;
  const scale = multiple ? 50 : 125;
  const colours = [C.blue, C.green, C.accent, C.purple, C.red];
  const weights = multiple ? [0.45, 0.8, 1, 0.65, 0.35] : [1];
  const entry = multiple ? 1.3 : 3.2;
  const transit = (right - left) / speed;
  const impulses = weights.map((weight, i) => ({
    weight, colour: multiple ? colours[i] : C.blue,
    enter: entry + i * 0.65, exit: entry + i * 0.65 + transit,
  }));
  const finish = cascade ? 13 : impulses[0].exit + 5.3;
  const hold = 1.7;
  // A causal profile with a smooth rise and a decaying tail.
  const profile = t => t < 0 ? 0 : 1.6 * t * Math.exp(1 - 1.6 * t);
  const canvas = h('canvas', {
    width: W * 2, height: H * 2, style: `width:${W}px; height:${H}px`,
    role: 'img', 'aria-label': cascade
      ? 'Impulses pass through the first block. Their combined output travels into a second block, producing another set of response profiles and another convolution below.'
      : multiple
      ? 'Coloured impulses travel into a block. Their scaled response profiles emerge on the right, with their sum below.'
      : 'A Dirac impulse travels from left to right into a block and emerges as its response profile.',
  });
  const ctx = canvas.getContext('2d');
  ctx.scale(2, 2);

  function text(value, x, y, colour = C.fg, size = 21, align = 'left') {
    ctx.fillStyle = colour; ctx.font = `${size}px Inter, Arial, sans-serif`;
    ctx.textAlign = align; ctx.fillText(value, x, y);
  }
  function line(x1, y1, x2, y2, colour = C.dim, width = 1.5) {
    ctx.strokeStyle = colour; ctx.lineWidth = width;
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  }
  function arrow(x, y, height, colour) {
    line(x, y, x, y - height, colour, 3);
    line(x - 5, y - height + 8, x, y - height, colour, 3);
    line(x + 5, y - height + 8, x, y - height, colour, 3);
  }
  function trace(fn, y, colour, width = 3, xStart = right, xEnd = end, amplitudeScale = scale) {
    ctx.strokeStyle = colour; ctx.lineWidth = width; ctx.beginPath();
    for (let x = xStart; x <= xEnd; x += 1) {
      const value = fn(x);
      if (x === xStart) ctx.moveTo(x, y - amplitudeScale * value);
      else ctx.lineTo(x, y - amplitudeScale * value);
    }
    ctx.stroke();
  }

  // The middle waveform is the actual sum from block 1. Approximate its
  // convolution with g2 by a Riemann sum; every plotted contribution includes dt.
  const firstInputs = [0.45, 0.8, 1, 0.65, 0.35].map((weight, i) => ({
    weight, colour: colours[i], enter: 0.5 + i * 0.35, exit: 1.4 + i * 0.35,
  }));
  const firstOutput = t => firstInputs.reduce((sum, p) => sum + p.weight * profile(t - p.exit), 0);
  const sampleDt = 0.25;
  const secondInputs = cascade ? Array.from({ length: 32 }, (_, i) => {
    const sampledAt = 1.4 + (i + 0.5) * sampleDt;
    return { sampledAt, exit: sampledAt + 4.9, weight: firstOutput(sampledAt) * sampleDt, colour: colours[i % colours.length] };
  }) : [];
  const secondProfile = t => t < 0 ? 0 : 2.6 * t * Math.exp(1 - 2.6 * t);

  function drawCascade(time, printing) {
    const y = 180, lowerY = 370;
    text('Input u(t)', 30, 35);
    text('Output of block 1 → input to block 2', 315, 35, C.fg, 20);
    text('Shifted, scaled profiles again', 810, 35, C.fg, 20);
    line(25, y, 210, y); line(300, y, 700, y); line(790, y, 1170, y);
    for (const [i, p] of firstInputs.entries()) {
      const x = printing ? 175 - i * 33 : 210 + speed * (time - p.enter);
      if (x >= 25 && x < 210) arrow(x, y, 75 * p.weight, p.colour);
    }
    // Faint individual responses plus their sum, continuously entering block 2.
    ctx.globalAlpha = 0.35;
    for (const p of firstInputs) {
      trace(x => p.weight * profile(time - p.exit - (x - 300) / speed), y, p.colour, 1.5, 300, 700, 36);
    }
    ctx.globalAlpha = 1;
    trace(x => firstOutput(time - (x - 300) / speed), y, C.accent, 3, 300, 700, 36);
    text('u * g₁', 500, 217, C.accent, 22, 'center');
    // Coloured points identify pieces of the first output that become the
    // correspondingly coloured responses after passing through block 2.
    for (const p of secondInputs) {
      const x = 300 + speed * (time - p.sampledAt);
      if (x > 300 && x < 700) {
        ctx.fillStyle = p.colour; ctx.beginPath();
        ctx.arc(x, y - 36 * firstOutput(p.sampledAt), 3, 0, 2 * Math.PI); ctx.fill();
      }
    }
    const block = (x, label, activeColour) => {
      ctx.fillStyle = C.bg3; ctx.strokeStyle = activeColour || C.dim; ctx.lineWidth = activeColour ? 3 : 1.5;
      ctx.beginPath(); ctx.roundRect(x, 90, 90, 130, 10); ctx.fill(); ctx.stroke();
      text(label, x + 45, 158, C.fg, 28, 'center');
    };
    block(210, 'g₁(t)', firstInputs.find(p => time >= p.enter && time < p.exit)?.colour);
    block(700, 'g₂(t)', secondInputs.find(p => time >= p.exit - 0.9 && time < p.exit)?.colour);
    const contribution = (p, x) => p.weight * secondProfile(time - p.exit - (x - 790) / speed);
    for (const p of secondInputs) {
      if (time > p.exit) trace(x => contribution(p, x), y, p.colour, 1.8, 790, 1170, 36);
    }
    text('Add them: (u * g₁) * g₂', 810, 265, C.accent, 22);
    line(790, lowerY, 1170, lowerY);
    if (time > secondInputs[0].exit) {
      trace(x => secondInputs.reduce((sum, p) => sum + contribution(p, x), 0), lowerY, C.accent, 3.5, 790, 1170, 36);
    }
    line(980, 205, 980, 232); line(975, 224, 980, 232); line(985, 224, 980, 232);
    text('The first convolution becomes the next input.', 35, 310, C.dim, 21);
    text('Now shift, scale and sum all over again.', 35, 345, C.dim, 21);
  }

  function draw(time, printing = false) {
    ctx.fillStyle = C.bg2; ctx.fillRect(0, 0, W, H);
    if (cascade) { drawCascade(time, printing); return; }
    text(multiple ? 'Input: a sum of impulses' : 'Input: δ(t)', 35, 40);
    text(multiple ? 'Shifted, scaled profiles' : 'Output: the profile g(t)', right + 20, 40);
    line(35, baseline, left, baseline);
    line(right, baseline, end, baseline);
    // The diagram is a stream moving right, not a time axis: older samples
    // are farther from the output. Each new sample is emitted at x = right.
    const response = (impulse, x) => impulse.weight * profile(time - impulse.exit - (x - right) / speed);

    ctx.save(); ctx.beginPath(); ctx.rect(30, 60, left - 30, baseline - 35); ctx.clip();
    for (const impulse of impulses) {
      const x = printing ? left - 85 - impulses.indexOf(impulse) * 65 : left + speed * (time - impulse.enter);
      if (x < 30 || x > left) continue;
      arrow(x, baseline, scale * impulse.weight, impulse.colour);
      if (!multiple) text('δ(t)', x, baseline - scale - 18, impulse.colour, 24, 'center');
    }
    ctx.restore();

    const active = impulses.find(impulse => time >= impulse.enter && time < impulse.exit);
    ctx.fillStyle = C.bg3; ctx.strokeStyle = active?.colour || C.dim; ctx.lineWidth = active ? 3 : 1.5;
    ctx.beginPath(); ctx.roundRect(left, baseline - 105, right - left, 145, 10); ctx.fill(); ctx.stroke();
    text('g(t)', (left + right) / 2, baseline - 25, C.fg, 30, 'center');
    // Show the pulse crossing the block before the profile starts emerging.
    if (active && !printing) {
      const x = left + speed * (time - active.enter);
      ctx.globalAlpha = 0.6;
      arrow(x, baseline + 20, 23, active.colour);
      ctx.globalAlpha = 1;
    }

    if (multiple) {
      for (const impulse of impulses) {
        if (time > impulse.exit) trace(x => response(impulse, x), baseline, impulse.colour, 2.5);
      }
      text('Add the profiles: convolution', right + 20, 240);
      const sumBaseline = 350;
      line(right, sumBaseline, end, sumBaseline);
      if (time > impulses[0].exit) {
        trace(x => impulses.reduce((sum, impulse) => sum + response(impulse, x), 0), sumBaseline, C.accent, 3.5);
      }
    } else if (time > impulses[0].exit) {
      trace(x => response(impulses[0], x), baseline, C.green);
    }
    // A small flow arrow makes the direction of travel explicit.
    const arrowY = multiple ? 213 : 332;
    line(175, arrowY, 315, arrowY);
    line(307, arrowY - 5, 315, arrowY);
    line(307, arrowY + 5, 315, arrowY);
  }

  el.classList.add('widget'); el.style.width = W + 'px'; el.append(canvas);
  if (isPrint()) { draw(cascade ? 9 : finish, true); return; }

  let time = 0, playing = true, last = null, raf = 0;
  const section = el.closest('section');
  const shown = () => section?.classList.contains('present');
  const play = h('button', { type: 'button' }, 'Pause');
  const replay = h('button', { type: 'button' }, 'Replay');
  const scrub = h('input', {
    type: 'range', min: 0, max: finish, step: 0.01, value: 0,
    'aria-label': 'Animation progress', style: 'width:240px; margin-left:0.6em',
  });
  el.append(h('div', { class: 'wctl interactive-only', style: 'position:relative; z-index:1' }, play, replay, scrub));
  function sync() {
    const label = playing ? 'Pause' : 'Play';
    // Keep the button's text node stable between pointer-down and pointer-up.
    // Replacing it on every animation frame can suppress Safari's click.
    if (play.textContent !== label) play.textContent = label;
    scrub.value = Math.min(time, finish);
  }
  function tick(now) {
    raf = 0;
    if (!canvas.isConnected || !shown() || !playing) { last = null; return; }
    if (last !== null) time += Math.min((now - last) / 1000, 0.1);
    last = now;
    if (time > finish + hold) time = 0;
    draw(Math.min(time, finish)); sync();
    raf = requestAnimationFrame(tick);
  }
  function start() { if (!raf && playing && shown()) raf = requestAnimationFrame(tick); }
  function stop() { cancelAnimationFrame(raf); raf = 0; last = null; }
  play.addEventListener('click', ev => {
    ev.stopPropagation();
    playing = !playing; stop(); sync();
    if (playing) start();
  });
  replay.addEventListener('click', ev => { ev.stopPropagation(); stop(); time = 0; playing = true; draw(0); sync(); start(); });
  scrub.addEventListener('input', () => { stop(); playing = false; time = +scrub.value; draw(time); sync(); });
  let activeSlide = shown();
  const onSlide = () => {
    const active = shown();
    // Only entering this slide should restart it, not a repeated notification
    // while the lecturer has paused or scrubbed the animation.
    if (active === activeSlide) return;
    activeSlide = active; stop();
    if (active) { time = 0; playing = true; draw(0); sync(); start(); }
  };
  const deck = el.closest('.reveal');
  deck?.addEventListener('slidechanged', onSlide);
  // The shared bootstrap empties distant widgets; discard this mount's listeners.
  const observer = new MutationObserver(() => {
    if (!el.contains(canvas)) {
      stop(); deck?.removeEventListener('slidechanged', onSlide); observer.disconnect();
    }
  });
  observer.observe(el, { childList: true });
  draw(0); start();
}
