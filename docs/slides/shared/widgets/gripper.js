// Shared 3D helpers in the "math" world frame (z up, right-handed) and the parallel-jaw gripper.
import { THREE, arrow3 } from './three-util.js';

// math frame -> three.js frame (y up): (x, y, z) -> (x, z, -y)
export const T = (x, y, z) => [x, z, -y];

// rotation matrix (row-major, math frame) -> the same rotation in three.js coordinates: P R Pᵀ
export function toThree(R) {
  const [a, b, c, d, e, f, g, hh, i] = R;
  return [a, c, -b, g, i, -hh, -d, -f, e];
}

export function setRotation(obj, R, pos = [0, 0, 0]) {
  const m = toThree(R);
  obj.matrixAutoUpdate = false;
  obj.matrix.set(m[0], m[1], m[2], pos[0], m[3], m[4], m[5], pos[1], m[6], m[7], m[8], pos[2], 0, 0, 0, 1);
  obj.matrixWorldNeedsUpdate = true;
}

// three.js quaternion of a math-frame rotation matrix (for slerp-based animation)
export function quatFromMath(R) {
  const m = toThree(R);
  const M = new THREE.Matrix4().set(m[0], m[1], m[2], 0, m[3], m[4], m[5], 0, m[6], m[7], m[8], 0, 0, 0, 0, 1);
  return new THREE.Quaternion().setFromRotationMatrix(M);
}

// A parallel-jaw gripper with its tool frame, built in math coordinates:
// z = approach direction (fingers), y = closing direction, x = y × z (red connector on the +x side).
// The returned group has setOpening(d): finger centres at y = ±d (default 0.36, scaled).
export function makeGripper(scale = 1, { axes = true } = {}) {
  const g = new THREE.Group();
  const mat = c => new THREE.MeshStandardMaterial({ color: c, roughness: 0.55, metalness: 0.25 });
  const box = (parent, sx, sy, sz, p, c) => {
    const m = new THREE.Mesh(new THREE.BoxGeometry(...T(sx, sy, sz).map(v => Math.abs(v) * scale)), mat(c));
    m.position.set(...T(...p.map(v => v * scale))); parent.add(m);
  };
  // wrist flange (cylinder axis = three's Y = math z)
  const flange = new THREE.Mesh(new THREE.CylinderGeometry(0.22 * scale, 0.22 * scale, 0.22 * scale, 32), mat(0x4a505c));
  flange.position.set(...T(0, 0, -0.52 * scale)); g.add(flange);
  box(g, 0.12, 0.12, 0.1, [0.25, 0, -0.52], 0xff6b6b);          // connector: marks +x
  box(g, 0.3, 1.0, 0.16, [0, 0, -0.33], 0x8a93a6);               // palm
  const fingers = [-1, 1].map(side => {
    const f = new THREE.Group();
    box(f, 0.2, 0.1, 0.62, [0, 0, 0.06], 0xc9ccd4);              // finger
    box(f, 0.16, 0.03, 0.26, [0, -side * 0.065, 0.22], 0x2d313a); // pad on the inner side
    g.add(f);
    return { f, side };
  });
  g.setOpening = d => fingers.forEach(({ f, side }) => f.position.set(...T(0, side * d * scale, 0)));
  g.setOpening(0.36);
  if (axes) {
    const L = 1.1 * scale;
    g.add(arrow3([0, 0, 0], T(L, 0, 0), 0xff6b6b, 0.12));
    g.add(arrow3([0, 0, 0], T(0, L, 0), 0x5fd38d, 0.12));
    g.add(arrow3([0, 0, 0], T(0, 0, L), 0x5ab0ff, 0.12));
  }
  return g;
}
