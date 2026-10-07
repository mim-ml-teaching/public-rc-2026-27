// Small helpers shared by widgets: DOM, colors, 3x3 matrices, a procedural test image.

export const C = {
  bg: '#15171c', bg2: '#1e2128', bg3: '#272b34', fg: '#e7e9ee', dim: '#9aa1ae',
  accent: '#f2b134', blue: '#5ab0ff', red: '#ff6b6b', green: '#5fd38d', purple: '#b28dff', line: '#3a3f4b',
};

export function h(tag, attrs = {}, ...children) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'class') e.className = v;
    else if (k === 'style') e.style.cssText = v;
    else if (k.startsWith('on')) e.addEventListener(k.slice(2), v);
    else e.setAttribute(k, v);
  }
  for (const c of children.flat()) e.append(c instanceof Node ? c : document.createTextNode(c));
  return e;
}

export function isPrint() {
  return document.documentElement.classList.contains('reveal-print');
}

// ---- 3x3 matrices as flat row-major arrays of length 9 ----
export const I3 = () => [1, 0, 0, 0, 1, 0, 0, 0, 1];

export function mul3(a, b) {
  const r = new Array(9);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      r[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
  return r;
}

export function det3(m) {
  return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
}

export function inv3(m) {
  const d = det3(m);
  if (Math.abs(d) < 1e-12) return null;
  const r = [
    m[4] * m[8] - m[5] * m[7], m[2] * m[7] - m[1] * m[8], m[1] * m[5] - m[2] * m[4],
    m[5] * m[6] - m[3] * m[8], m[0] * m[8] - m[2] * m[6], m[2] * m[3] - m[0] * m[5],
    m[3] * m[7] - m[4] * m[6], m[1] * m[6] - m[0] * m[7], m[0] * m[4] - m[1] * m[3],
  ];
  return r.map(x => x / d);
}

export function apply3(m, x, y) {
  const w = m[6] * x + m[7] * y + m[8];
  return [(m[0] * x + m[1] * y + m[2]) / w, (m[3] * x + m[4] * y + m[5]) / w, w];
}

export const lerp = (a, b, t) => a + (b - a) * t;
export const lerp9 = (a, b, t) => a.map((x, i) => lerp(x, b[i], t));

export function fmt(x, digits = 2) {
  let s = x.toFixed(digits);
  if (s.includes('.')) s = s.replace(/\.?0+$/, '');
  return s === '-0' ? '0' : s;
}

// ---- Procedural test image: an asymmetric scene so flips/rotations are obvious. ----
export function testImage(W = 320, H = 240) {
  const c = document.createElement('canvas');
  c.width = W; c.height = H;
  const g = c.getContext('2d');
  const sky = g.createLinearGradient(0, 0, 0, H * 0.7);
  sky.addColorStop(0, '#3b6fb6'); sky.addColorStop(1, '#a9cbe8');
  g.fillStyle = sky; g.fillRect(0, 0, W, H);
  // sun (top-left, so flips are visible)
  g.fillStyle = '#ffd34d'; g.beginPath(); g.arc(W * 0.16, H * 0.2, H * 0.09, 0, 7); g.fill();
  // ground
  g.fillStyle = '#4f8a3c'; g.fillRect(0, H * 0.68, W, H * 0.32);
  // house (right of centre) with chimney on the left side of the roof
  const hx = W * 0.5, hy = H * 0.42, hw = W * 0.3, hh = H * 0.3;
  g.fillStyle = '#e9dcc0'; g.fillRect(hx, hy, hw, hh);
  g.fillStyle = '#7a3b2e'; g.fillRect(hx + hw * 0.12, hy - hh * 0.55, hw * 0.12, hh * 0.4);
  g.fillStyle = '#c0392b';
  g.beginPath(); g.moveTo(hx - hw * 0.08, hy); g.lineTo(hx + hw / 2, hy - hh * 0.6); g.lineTo(hx + hw * 1.08, hy); g.fill();
  g.fillStyle = '#6b4226'; g.fillRect(hx + hw * 0.65, hy + hh * 0.45, hw * 0.2, hh * 0.55);
  g.fillStyle = '#5ab0ff'; g.fillRect(hx + hw * 0.15, hy + hh * 0.25, hw * 0.25, hh * 0.28);
  // tree on the left
  g.fillStyle = '#5b3a1e'; g.fillRect(W * 0.2, H * 0.5, W * 0.03, H * 0.2);
  g.fillStyle = '#2f6b2a'; g.beginPath(); g.arc(W * 0.215, H * 0.46, H * 0.1, 0, 7); g.fill();
  // faint grid so that straight lines / parallelism are visible after warping
  g.strokeStyle = 'rgba(255,255,255,0.35)'; g.lineWidth = 1;
  for (let x = 0; x <= W; x += W / 8) { g.beginPath(); g.moveTo(x + 0.5, 0); g.lineTo(x + 0.5, H); g.stroke(); }
  for (let y = 0; y <= H; y += H / 6) { g.beginPath(); g.moveTo(0, y + 0.5); g.lineTo(W, y + 0.5); g.stroke(); }
  g.strokeStyle = '#fff'; g.lineWidth = 3; g.strokeRect(1.5, 1.5, W - 3, H - 3);
  return c;
}

export function arrow(g, x0, y0, x1, y1, color, width = 3) {
  const a = Math.atan2(y1 - y0, x1 - x0), L = Math.hypot(x1 - x0, y1 - y0), hd = Math.min(12, L * 0.4);
  g.strokeStyle = color; g.fillStyle = color; g.lineWidth = width;
  g.beginPath(); g.moveTo(x0, y0); g.lineTo(x1 - hd * 0.7 * Math.cos(a), y1 - hd * 0.7 * Math.sin(a)); g.stroke();
  g.beginPath(); g.moveTo(x1, y1);
  g.lineTo(x1 - hd * Math.cos(a - 0.4), y1 - hd * Math.sin(a - 0.4));
  g.lineTo(x1 - hd * Math.cos(a + 0.4), y1 - hd * Math.sin(a + 0.4));
  g.closePath(); g.fill();
}
