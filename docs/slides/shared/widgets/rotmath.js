// Rotation math on flat row-major 3x3 arrays (math frame: right-handed, z up).
export const deg = Math.PI / 180;

export function Rx(a) { const c = Math.cos(a), s = Math.sin(a); return [1, 0, 0, 0, c, -s, 0, s, c]; }
export function Ry(a) { const c = Math.cos(a), s = Math.sin(a); return [c, 0, s, 0, 1, 0, -s, 0, c]; }
export function Rz(a) { const c = Math.cos(a), s = Math.sin(a); return [c, -s, 0, s, c, 0, 0, 0, 1]; }

export function mul(a, b) {
  const r = new Array(9);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      r[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
  return r;
}

export const I = () => [1, 0, 0, 0, 1, 0, 0, 0, 1];
export const transpose = m => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
export const apply = (m, v) => [m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2], m[6] * v[0] + m[7] * v[1] + m[8] * v[2]];

// Rodrigues: R = I + sinθ [k]x + (1 − cosθ) [k]x^2, k unit
export function axisAngle(k, t) {
  const [x, y, z] = k, c = Math.cos(t), s = Math.sin(t), C = 1 - c;
  return [
    c + x * x * C, x * y * C - z * s, x * z * C + y * s,
    y * x * C + z * s, c + y * y * C, y * z * C - x * s,
    z * x * C - y * s, z * y * C + x * s, c + z * z * C,
  ];
}

// R -> (axis, angle), angle in [0, π]
export function toAxisAngle(R) {
  const cos = Math.min(1, Math.max(-1, (R[0] + R[4] + R[8] - 1) / 2));
  const t = Math.acos(cos);
  if (t < 1e-9) return { axis: [0, 0, 1], angle: 0 };
  if (Math.PI - t < 1e-6) {
    // near π: axis from the diagonal of (R + I)/2 = k kᵀ
    const xx = (R[0] + 1) / 2, yy = (R[4] + 1) / 2, zz = (R[8] + 1) / 2;
    let k;
    if (xx >= yy && xx >= zz) { const x = Math.sqrt(xx); k = [x, R[1] / (2 * x), R[2] / (2 * x)]; }
    else if (yy >= zz) { const y = Math.sqrt(yy); k = [R[1] / (2 * y), y, R[5] / (2 * y)]; }
    else { const z = Math.sqrt(zz); k = [R[2] / (2 * z), R[5] / (2 * z), z]; }
    return { axis: k, angle: t };
  }
  const s = 2 * Math.sin(t);
  return { axis: [(R[7] - R[5]) / s, (R[2] - R[6]) / s, (R[3] - R[1]) / s], angle: t };
}

// unit quaternion (w, x, y, z) with w >= 0
export function toQuat(R) {
  const { axis, angle } = toAxisAngle(R);
  const s = Math.sin(angle / 2);
  return [Math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s];
}

// ZYX (yaw ψ, pitch θ, roll φ): R = Rz(ψ) Ry(θ) Rx(φ)
export function toEulerZYX(R) {
  const pitch = Math.asin(Math.max(-1, Math.min(1, -R[6])));
  if (Math.abs(Math.cos(pitch)) < 1e-6) {
    // gimbal lock: only ψ ∓ φ is defined; report φ = 0
    return { yaw: Math.atan2(-R[1], R[4]), pitch, roll: 0, locked: true };
  }
  return { yaw: Math.atan2(R[3], R[0]), pitch, roll: Math.atan2(R[7], R[8]), locked: false };
}

export const eulerZYX = (yaw, pitch, roll) => mul(Rz(yaw), mul(Ry(pitch), Rx(roll)));
