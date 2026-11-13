// Minimal SVG line plot shared by the IMU widgets (imudrift, compfilter).
// bare: true skips the background (to draw a second layer on top of a first call)
// plot(svg, { x0, y0, w, h, xmin, xmax, ymin, ymax, xlabel, ylabel, xticks, yticks, series: [{ xs, ys, color, width, dash, opacity }] })
import { C } from './util.js';

const NS = 'http://www.w3.org/2000/svg';
export const S = (tag, a = {}, text) => {
  const e = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(a)) e.setAttribute(k, v);
  if (text !== undefined) e.textContent = text;
  return e;
};

export function plot(svg, o) {
  const { x0, y0, w, h, xmin, xmax, ymin, ymax } = o;
  const X = x => x0 + (x - xmin) / (xmax - xmin) * w;
  const Y = y => y0 + h - (Math.max(ymin, Math.min(ymax, y)) - ymin) / (ymax - ymin) * h;
  const g = S('g', { 'font-family': 'Inter, Arial, sans-serif' });
  if (!o.bare) g.append(S('rect', { x: x0, y: y0, width: w, height: h, fill: C.bg2, stroke: C.line }));
  for (const t of o.yticks || []) {
    g.append(S('line', { x1: x0, x2: x0 + w, y1: Y(t), y2: Y(t), stroke: C.line, 'stroke-width': t === 0 ? 1.5 : 1 }));
    g.append(S('text', { x: x0 - 6, y: Y(t) + 5, fill: C.dim, 'font-size': 14, 'text-anchor': 'end' }, String(t)));
  }
  for (const t of o.xticks || []) {
    g.append(S('line', { x1: X(t), x2: X(t), y1: y0 + h, y2: y0 + h + 5, stroke: C.dim }));
    g.append(S('text', { x: X(t), y: y0 + h + 20, fill: C.dim, 'font-size': 14, 'text-anchor': 'middle' }, String(t)));
  }
  if (o.xlabel) g.append(S('text', { x: x0 + w, y: y0 + h + 38, fill: C.dim, 'font-size': 14, 'text-anchor': 'end' }, o.xlabel));
  if (o.ylabel) g.append(S('text', { x: x0, y: y0 - 8, fill: C.dim, 'font-size': 14 }, o.ylabel));
  for (const s of o.series) {
    let d = '';
    const step = Math.max(1, Math.floor(s.xs.length / (w * 1.5)));
    for (let i = 0; i < s.xs.length; i += step) d += (d ? 'L' : 'M') + X(s.xs[i]).toFixed(1) + ',' + Y(s.ys[i]).toFixed(1);
    g.append(S('path', { d, fill: 'none', stroke: s.color, 'stroke-width': s.width || 2, 'stroke-dasharray': s.dash || 'none', opacity: s.opacity ?? 1 }));
  }
  svg.append(g);
  return { X, Y };
}

// Gaussian random numbers from a seeded generator (reproducible figures, also in the PDF)
export function rng(seed) {
  let s = seed >>> 0;
  const u = () => ((s = (s * 1664525 + 1013904223) >>> 0) + 0.5) / 4294967296;
  return () => Math.sqrt(-2 * Math.log(u())) * Math.cos(2 * Math.PI * u());
}

export function legend(svg, x, y, items) {
  items.forEach(([label, color, dash], i) => {
    const yy = y + i * 22;
    svg.append(S('line', { x1: x, x2: x + 26, y1: yy, y2: yy, stroke: color, 'stroke-width': 3, 'stroke-dasharray': dash || 'none' }));
    svg.append(S('text', { x: x + 34, y: yy + 5, fill: C.fg, 'font-size': 15, 'font-family': 'Inter, Arial, sans-serif' }, label));
  });
}
