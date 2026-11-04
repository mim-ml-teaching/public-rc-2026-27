// Looping playback of a time parameter t ∈ [0, 1], for widgets that play by themselves: t runs from 0 to 1 in `dur` ms,
// holds at 1 for `hold` ms, jumps back to 0, holds there for `hold0` ms, and so on. step(t) applies t to the widget
// (and redraws). The loop ends with stop(), or when alive() turns false (the canvas was removed from the page).
// once: true plays a single run 0 → 1 and stops there (onEnd() is called), for widgets that play on demand.
export function loopPlayer({ dur, hold = 1500, hold0 = 500, alive = () => true, step, once = false, onEnd }) {
  let playing = false, raf = 0;
  const stop = () => { playing = false; cancelAnimationFrame(raf); };
  const start = (t0 = 0) => {
    stop(); playing = true;
    let t = t0, wait = t0 >= 1 ? hold : 0, last = performance.now();
    const tick = now => {
      if (!playing) return;
      if (!alive()) { playing = false; return; }
      const dt = Math.max(0, Math.min(100, now - last)); last = now;   // after a hidden tab: continue, do not jump
      if (wait > 0) { wait -= dt; if (wait <= 0 && t >= 1) { t = 0; wait = hold0; step(t); } }
      else { t = Math.min(1, t + dt / dur); step(t); if (t >= 1) { if (once) { playing = false; onEnd?.(); return; } wait = hold; } }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
  };
  return { start, stop, get playing() { return playing; } };
}
