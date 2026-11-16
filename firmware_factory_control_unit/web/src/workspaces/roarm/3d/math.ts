// Minimal vector/quaternion math — no external dependency. Just enough for
// forward/inverse kinematics and a simple 3D camera (position + orientation,
// no scale/shear, so plain vec3+quat composition is sufficient — no need for
// a general 4x4 matrix stack).

export type Vec3 = [number, number, number]
export type Quat = [number, number, number, number] // x, y, z, w

export function vec3(x = 0, y = 0, z = 0): Vec3 {
  return [x, y, z]
}

export function add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

export function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

export function scale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s]
}

export function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

export function cross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

export function length(a: Vec3): number {
  return Math.sqrt(dot(a, a))
}

export function normalize(a: Vec3): Vec3 {
  const len = length(a)
  return len < 1e-12 ? [0, 0, 0] : scale(a, 1 / len)
}

export function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v
}

export const QUAT_IDENTITY: Quat = [0, 0, 0, 1]

export function quatFromAxisAngle(axis: Vec3, angle: number): Quat {
  const a = normalize(axis)
  const s = Math.sin(angle / 2)
  return [a[0] * s, a[1] * s, a[2] * s, Math.cos(angle / 2)]
}

export function quatMultiply(a: Quat, b: Quat): Quat {
  const [ax, ay, az, aw] = a
  const [bx, by, bz, bw] = b
  return [
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ]
}

export function quatConjugate(q: Quat): Quat {
  return [-q[0], -q[1], -q[2], q[3]]
}

export function quatNormalize(q: Quat): Quat {
  const len = Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
  return len < 1e-12 ? [0, 0, 0, 1] : [q[0] / len, q[1] / len, q[2] / len, q[3] / len]
}

export function rotateVec3(q: Quat, v: Vec3): Vec3 {
  // v' = q * v * q^-1, expanded without allocating intermediate quaternions.
  const [qx, qy, qz, qw] = q
  const [vx, vy, vz] = v
  const tx = 2 * (qy * vz - qz * vy)
  const ty = 2 * (qz * vx - qx * vz)
  const tz = 2 * (qx * vy - qy * vx)
  return [
    vx + qw * tx + (qy * tz - qz * ty),
    vy + qw * ty + (qz * tx - qx * tz),
    vz + qw * tz + (qx * ty - qy * tx),
  ]
}

/** Roll/pitch/yaw about fixed X, Y, Z axes (URDF `rpy` convention: R = Rz * Ry * Rx). */
export function quatFromEulerXYZ(roll: number, pitch: number, yaw: number): Quat {
  const qx = quatFromAxisAngle([1, 0, 0], roll)
  const qy = quatFromAxisAngle([0, 1, 0], pitch)
  const qz = quatFromAxisAngle([0, 0, 1], yaw)
  return quatMultiply(quatMultiply(qz, qy), qx)
}

/** Builds a quaternion from an orthonormal right/up/forward basis (columns of a rotation matrix). */
export function quatFromBasis(right: Vec3, up: Vec3, forward: Vec3): Quat {
  // Local -Z maps to `forward` (OpenGL-style camera convention).
  const m00 = right[0], m10 = right[1], m20 = right[2]
  const m01 = up[0], m11 = up[1], m21 = up[2]
  const m02 = -forward[0], m12 = -forward[1], m22 = -forward[2]
  const trace = m00 + m11 + m22
  if (trace > 0) {
    const s = 0.5 / Math.sqrt(trace + 1)
    return quatNormalize([(m21 - m12) * s, (m02 - m20) * s, (m10 - m01) * s, 0.25 / s])
  } else if (m00 > m11 && m00 > m22) {
    const s = 2 * Math.sqrt(1 + m00 - m11 - m22)
    return quatNormalize([0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s])
  } else if (m11 > m22) {
    const s = 2 * Math.sqrt(1 + m11 - m00 - m22)
    return quatNormalize([(m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s])
  } else {
    const s = 2 * Math.sqrt(1 + m22 - m00 - m11)
    return quatNormalize([(m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s])
  }
}

/** Camera-style look-at quaternion: local -Z points from `eye` toward `target`. */
export function quatLookAt(eye: Vec3, target: Vec3, worldUp: Vec3): Quat {
  const forward = normalize(sub(target, eye))
  const right = normalize(cross(forward, worldUp))
  const up = cross(right, forward)
  return quatFromBasis(right, up, forward)
}

/** A rigid transform: world = position + quaternion * local. */
export interface Pose {
  pos: Vec3
  quat: Quat
}

export function composePose(parent: Pose, local: Pose): Pose {
  return {
    pos: add(parent.pos, rotateVec3(parent.quat, local.pos)),
    quat: quatNormalize(quatMultiply(parent.quat, local.quat)),
  }
}

export function transformPoint(pose: Pose, p: Vec3): Vec3 {
  return add(pose.pos, rotateVec3(pose.quat, p))
}
