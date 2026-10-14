// Small dense linear algebra for the calibration / PnP widgets (matrices as arrays of rows).
// symEig: Jacobi eigen-decomposition of a symmetric matrix; rq3: RQ decomposition of a 3x3 matrix;
// solve: Gaussian elimination with partial pivoting; rodrigues: rotation vector -> rotation matrix.

export const matmul = (A, B) => A.map(r => B[0].map((_, j) => r.reduce((s, a, k) => s + a * B[k][j], 0)));
export const transpose = A => A[0].map((_, j) => A.map(r => r[j]));
export const matvec = (A, x) => A.map(r => r.reduce((s, a, k) => s + a * x[k], 0));

// Eigenvalues (ascending) and unit eigenvectors (vecs[i] belongs to vals[i]) of a symmetric matrix.
export function symEig(S) {
  const n = S.length, A = S.map(r => r.slice());
  const V = A.map((_, i) => A.map((_, j) => (i === j ? 1 : 0)));
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += A[p][q] * A[p][q];
    if (off < 1e-30) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(A[p][q]) < 1e-300) continue;
      const th = (A[q][q] - A[p][p]) / (2 * A[p][q]);
      const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < n; k++) { // A <- A J
        const akp = A[k][p], akq = A[k][q];
        A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq;
      }
      for (let k = 0; k < n; k++) { // A <- Jᵀ A
        const apk = A[p][k], aqk = A[q][k];
        A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk;
      }
      for (let k = 0; k < n; k++) {
        const vkp = V[k][p], vkq = V[k][q];
        V[k][p] = c * vkp - s * vkq; V[k][q] = s * vkp + c * vkq;
      }
    }
  }
  const order = A.map((_, i) => i).sort((i, j) => A[i][i] - A[j][j]);
  return { vals: order.map(i => A[i][i]), vecs: order.map(i => V.map(r => r[i])) };
}

// M = K R with K upper triangular (positive diagonal) and R orthogonal.
export function rq3(M) {
  // Gram–Schmidt on the rows of M, from the last row up.
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const r3 = M[2].slice(), n3 = Math.hypot(...r3); const q3 = r3.map(v => v / n3);
  const k23 = dot(M[1], q3); const r2 = M[1].map((v, i) => v - k23 * q3[i]); const n2 = Math.hypot(...r2); const q2 = r2.map(v => v / n2);
  const k13 = dot(M[0], q3), k12 = dot(M[0], q2);
  const r1 = M[0].map((v, i) => v - k13 * q3[i] - k12 * q2[i]); const n1 = Math.hypot(...r1); const q1 = r1.map(v => v / n1);
  return { K: [[n1, k12, k13], [0, n2, k23], [0, 0, n3]], R: [q1, q2, q3] };
}

export function det3(M) {
  return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
    + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
}

// Solve A x = b (A square).
export function solve(A, b) {
  const n = A.length, M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    if (Math.abs(M[c][c]) < 1e-14) return null;
    for (let r = c + 1; r < n; r++) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) {
    let s = M[r][n];
    for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k];
    x[r] = s / M[r][r];
  }
  return x;
}

export function rodrigues(w) {
  const th = Math.hypot(...w);
  if (th < 1e-12) return [[1, -w[2], w[1]], [w[2], 1, -w[0]], [-w[1], w[0], 1]];
  const [x, y, z] = w.map(v => v / th), c = Math.cos(th), s = Math.sin(th), C = 1 - c;
  return [
    [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
    [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
    [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
  ];
}

// Rotation matrix -> rotation vector (axis * angle).
export function logR(R) {
  const c = Math.max(-1, Math.min(1, (R[0][0] + R[1][1] + R[2][2] - 1) / 2)), th = Math.acos(c);
  if (th < 1e-9) return [0, 0, 0];
  const v = [R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1]], s = 2 * Math.sin(th);
  if (Math.abs(s) < 1e-6) { // θ ≈ π: axis from (R + I) / 2 = k kᵀ
    const d = [0, 1, 2].map(i => Math.sqrt(Math.max(0, (R[i][i] + 1) / 2)));
    const i = d.indexOf(Math.max(...d)), k = [0, 1, 2].map(j => (j === i ? d[i] : (R[i][j] + R[j][i]) / (4 * d[i])));
    return k.map(v => v * th);
  }
  return v.map(x => x * th / s);
}

// Seeded random numbers (deterministic default states for printing).
export function rng(seed = 1) {
  let s = seed >>> 0 || 1;
  const u = () => { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return s / 4294967296; };
  u.normal = () => { const a = u() || 1e-12, b = u(); return Math.sqrt(-2 * Math.log(a)) * Math.cos(2 * Math.PI * b); };
  return u;
}
