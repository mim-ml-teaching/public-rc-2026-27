// Arrows from callout boxes to parts of a formula (or to any element), drawn from the rendered layout.
// Markup: a positioned container of a fixed pixel size (class "anno": position:relative), holding the formula(s),
// absolutely positioned callouts, and <div data-widget="annotate"></div>. Every element of the container with
// data-to="<selector>" gets an arrow to each element the selector matches inside the container; a part of a KaTeX
// formula is marked with \htmlClass{t-pose}{\mathbf P_j} and targeted with data-to=".t-pose". The arrows are
// computed in the container's own pixel coordinates from the rendered boxes, so they are right under reveal's
// scaling, in the print layout and after the fonts load; they are redrawn on resize and slide changes. A callout
// that is a hidden fragment has no arrow until it is shown.
// config: { color: "var(--fg-dim)", width: 1.6, gap: 4 }

export async function mount(el, cfg) {
  const root = el.parentElement;
  const color = 'currentColor', lw = cfg.width ?? 1.6, gap = cfg.gap ?? 4;   // the colour follows the theme
  const ns = 'http://www.w3.org/2000/svg';
  const id = 'anno-' + Math.random().toString(36).slice(2, 8);
  el.style.cssText = 'position:absolute; left:0; top:0; width:100%; height:100%; pointer-events:none; margin:0';
  const svg = document.createElementNS(ns, 'svg');
  svg.style.cssText = 'position:absolute; left:0; top:0; width:100%; height:100%; overflow:visible; color:' + (cfg.color || 'var(--fg-dim)');
  svg.innerHTML = `<defs><marker id="${id}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto"><path d="M0,0 L10,5 L0,10 z" fill="${color}"/></marker></defs><g></g>`;
  el.append(svg);
  const g = svg.lastElementChild;

  // the point where the ray from the centre of box b in direction (dx, dy) leaves b
  const exit = (b, dx, dy) => {
    const cx = b.x + b.w / 2, cy = b.y + b.h / 2;
    const t = Math.min(Math.abs(dx) > 1e-9 ? (b.w / 2) / Math.abs(dx) : 1e9, Math.abs(dy) > 1e-9 ? (b.h / 2) / Math.abs(dy) : 1e9);
    return [cx + dx * t, cy + dy * t];
  };
  function draw() {
    const R = root.getBoundingClientRect();
    if (!R.width || !root.offsetWidth) return;
    const sx = R.width / root.offsetWidth, sy = R.height / root.offsetHeight;
    svg.setAttribute('viewBox', `0 0 ${root.offsetWidth} ${root.offsetHeight}`);
    const box = e => { const r = e.getBoundingClientRect(); return { x: (r.left - R.left) / sx, y: (r.top - R.top) / sy, w: r.width / sx, h: r.height / sy }; };
    const lines = [];
    for (const from of root.querySelectorAll('[data-to]')) {
      if (from.classList.contains('fragment') && !from.classList.contains('visible')) continue;
      for (const to of root.querySelectorAll(from.dataset.to)) {
        const a = box(from), b = box(to);
        const dx = (b.x + b.w / 2) - (a.x + a.w / 2), dy = (b.y + b.h / 2) - (a.y + a.h / 2);
        const L = Math.hypot(dx, dy) || 1, ux = dx / L, uy = dy / L;
        const [x1, y1] = exit(a, ux, uy), [x2, y2] = exit(b, -ux, -uy);
        const f = v => v.toFixed(1);
        lines.push(`<line x1="${f(x1 + ux * gap)}" y1="${f(y1 + uy * gap)}" x2="${f(x2 - ux * gap)}" y2="${f(y2 - uy * gap)}" stroke="${color}" stroke-width="${lw}" marker-end="url(#${id})"/>`);
      }
    }
    g.innerHTML = lines.join('');
  }
  await document.fonts.ready;
  draw();
  const later = () => requestAnimationFrame(draw);
  new ResizeObserver(later).observe(root);
  const deck = window.deck;
  if (deck) ['resize', 'slidechanged', 'slidetransitionend', 'fragmentshown', 'fragmenthidden'].forEach(ev => deck.on(ev, later));
}
