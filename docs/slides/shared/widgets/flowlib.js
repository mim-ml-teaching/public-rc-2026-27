// Shared code for the flow-matching widgets: seeded RNG, 2D toy distributions, a small MLP velocity field
// v_θ(t, x) with hand-written backprop and Adam, the conditional flow-matching loss, and ODE samplers.
// No DOM here: experiments/flow2d_train.mjs runs the same code in Node to pretrain the weights shipped with the deck.

export function rng(seed) { let s = seed >>> 0 || 1; return () => ((s = (s * 16807) % 2147483647) / 2147483647); }
export function gauss(r) { const u = Math.max(1e-9, r()), v = r(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v); }

// ---- distributions: samplers returning [x, y] in roughly [-3, 3]^2 (the scale of the standard Gaussian) ----
export const SOURCES = {
  gauss: r => [gauss(r), gauss(r)],
  gauss8: r => { const k = Math.floor(r() * 8), a = k * Math.PI / 4; return [2.4 * Math.cos(a) + 0.25 * gauss(r), 2.4 * Math.sin(a) + 0.25 * gauss(r)]; },
};
export const TARGETS = {
  moons: r => {
    const up = r() < 0.5, a = Math.PI * r();
    return up ? [2 * Math.cos(a) - 1, 2 * Math.sin(a) - 0.6 + 0.15 * gauss(r)] : [2 * Math.cos(a) + 1, -2 * Math.sin(a) + 0.6 + 0.15 * gauss(r)];
  },
  spiral: r => { const t = 0.4 + 2.6 * Math.sqrt(r()), a = 2.2 * t; return [t * Math.cos(a) + 0.12 * gauss(r), t * Math.sin(a) + 0.12 * gauss(r)]; },
  rings: r => { const k = r() < 0.4 ? 0.9 : 2.3, a = 2 * Math.PI * r(); return [k * Math.cos(a) + 0.1 * gauss(r), k * Math.sin(a) + 0.1 * gauss(r)]; },
  gauss8: SOURCES.gauss8,
};
export const sample = (f, n, r) => Array.from({ length: n }, () => f(r));

// ---- MLP velocity field: input (x, y, t) → (vx, vy); hidden layers with SiLU ----
const silu = z => z / (1 + Math.exp(-z));
const dsilu = z => { const s = 1 / (1 + Math.exp(-z)); return s * (1 + z * (1 - s)); };

export function makeNet(sizes, r) {
  return sizes.slice(1).map((nout, l) => {
    const nin = sizes[l], s = Math.sqrt(2 / nin) * (l === sizes.length - 2 ? 0.3 : 1);
    const W = new Float64Array(nout * nin);
    for (let i = 0; i < W.length; i++) W[i] = gauss(r) * s;
    return { nin, nout, W, b: new Float64Array(nout), gW: new Float64Array(nout * nin), gb: new Float64Array(nout),
      mW: new Float64Array(nout * nin), vW: new Float64Array(nout * nin), mb: new Float64Array(nout), vb: new Float64Array(nout),
      z: new Float64Array(nout), a: new Float64Array(nout), d: new Float64Array(nout) };
  });
}

export function nParams(net) { return net.reduce((s, L) => s + L.W.length + L.b.length, 0); }

// forward pass on one input; activations stay in the layers (for backward)
export function forward(net, inp) {
  let a = inp;
  for (let l = 0; l < net.length; l++) {
    const L = net[l], last = l === net.length - 1;
    for (let i = 0; i < L.nout; i++) {
      let z = L.b[i]; const off = i * L.nin;
      for (let j = 0; j < L.nin; j++) z += L.W[off + j] * a[j];
      L.z[i] = z; L.a[i] = last ? z : silu(z);
    }
    a = L.a;
  }
  return a;
}

export const velocity = (net, t, x, y) => { const o = forward(net, [x, y, t]); return [o[0], o[1]]; };

// one Adam step on the conditional flow-matching loss  E ||v(t, x_t) − (x1 − x0)||²,  x_t = (1−t) x0 + t x1
export function cfmStep(net, opt, r, source, target, batch = 128) {
  net.forEach(L => { L.gW.fill(0); L.gb.fill(0); });
  let loss = 0;
  for (let n = 0; n < batch; n++) {
    const x0 = source(r), x1 = target(r), t = r();
    const inp = [(1 - t) * x0[0] + t * x1[0], (1 - t) * x0[1] + t * x1[1], t];
    const out = forward(net, inp);
    const e0 = out[0] - (x1[0] - x0[0]), e1 = out[1] - (x1[1] - x0[1]);
    loss += e0 * e0 + e1 * e1;
    const last = net[net.length - 1];
    last.d[0] = 2 * e0 / batch; last.d[1] = 2 * e1 / batch;
    for (let l = net.length - 1; l >= 0; l--) {
      const L = net[l], prev = l ? net[l - 1].a : inp;
      for (let i = 0; i < L.nout; i++) {
        const d = L.d[i], off = i * L.nin; L.gb[i] += d;
        for (let j = 0; j < L.nin; j++) L.gW[off + j] += d * prev[j];
      }
      if (l) {
        const P = net[l - 1];
        for (let j = 0; j < P.nout; j++) {
          let s = 0;
          for (let i = 0; i < L.nout; i++) s += L.W[i * L.nin + j] * L.d[i];
          P.d[j] = s * dsilu(P.z[j]);
        }
      }
    }
  }
  opt.step++;
  const { lr, b1 = 0.9, b2 = 0.999, eps = 1e-8 } = opt, c1 = 1 - b1 ** opt.step, c2 = 1 - b2 ** opt.step;
  for (const L of net) {
    for (let i = 0; i < L.W.length; i++) {
      const g = L.gW[i]; L.mW[i] = b1 * L.mW[i] + (1 - b1) * g; L.vW[i] = b2 * L.vW[i] + (1 - b2) * g * g;
      L.W[i] -= lr * (L.mW[i] / c1) / (Math.sqrt(L.vW[i] / c2) + eps);
    }
    for (let i = 0; i < L.b.length; i++) {
      const g = L.gb[i]; L.mb[i] = b1 * L.mb[i] + (1 - b1) * g; L.vb[i] = b2 * L.vb[i] + (1 - b2) * g * g;
      L.b[i] -= lr * (L.mb[i] / c1) / (Math.sqrt(L.vb[i] / c2) + eps);
    }
  }
  return loss / batch;
}

// ---- ODE samplers: Euler steps along the learned field; returns the trajectory [[x,y], ...] of length n+1 ----
export function euler(net, x0, n) {
  const path = [x0.slice()];
  let [x, y] = x0;
  for (let k = 0; k < n; k++) {
    const [vx, vy] = velocity(net, k / n, x, y);
    x += vx / n; y += vy / n; path.push([x, y]);
  }
  return path;
}

// midpoint rule, as the "exact" reference
export function midpoint(net, x0, n) {
  const path = [x0.slice()];
  let [x, y] = x0;
  for (let k = 0; k < n; k++) {
    const h = 1 / n, [vx, vy] = velocity(net, k * h, x, y);
    const [mx, my] = velocity(net, (k + 0.5) * h, x + 0.5 * h * vx, y + 0.5 * h * vy);
    x += h * mx; y += h * my; path.push([x, y]);
  }
  return path;
}

// ---- weights ↔ JSON ----
export function exportNet(net) {
  const rd = v => +v.toPrecision(5);
  return { sizes: [net[0].nin, ...net.map(L => L.nout)], layers: net.map(L => ({ W: Array.from(L.W, rd), b: Array.from(L.b, rd) })) };
}
export function importNet(json) {
  const net = makeNet(json.sizes, rng(1));
  json.layers.forEach((L, l) => { net[l].W.set(L.W); net[l].b.set(L.b); });
  return net;
}
