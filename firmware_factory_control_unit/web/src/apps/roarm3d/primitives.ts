import type { Mesh } from './robot-data.js'

/** A cylindrical frustum (cone/cylinder) around the local Z axis, from z0..z1 with radius r0..r1. */
export function makeFrustum(segments: number, r0: number, z0: number, r1: number, z1: number, caps: boolean): Mesh {
  const positions: number[] = []
  const indices: number[] = []

  const ringBottom: number[] = []
  const ringTop: number[] = []
  for (let i = 0; i < segments; i++) {
    const a = (i / segments) * Math.PI * 2
    const cx = Math.cos(a)
    const cy = Math.sin(a)
    ringBottom.push(positions.length / 3)
    positions.push(cx * r0, cy * r0, z0)
    ringTop.push(positions.length / 3)
    positions.push(cx * r1, cy * r1, z1)
  }

  for (let i = 0; i < segments; i++) {
    const ni = (i + 1) % segments
    indices.push(ringBottom[i], ringBottom[ni], ringTop[i])
    indices.push(ringTop[i], ringBottom[ni], ringTop[ni])
  }

  if (caps) {
    const centerBottom = positions.length / 3
    positions.push(0, 0, z0)
    const centerTop = positions.length / 3
    positions.push(0, 0, z1)
    for (let i = 0; i < segments; i++) {
      const ni = (i + 1) % segments
      indices.push(centerBottom, ringBottom[ni], ringBottom[i])
      indices.push(centerTop, ringTop[i], ringTop[ni])
    }
  }

  return { positions: new Float32Array(positions), indices: new Uint16Array(indices) }
}

/**
 * An axis-aligned box centered at the origin, `sx`x`sy`x`sz`. Each face gets its own 4 vertices
 * (24 total, not the minimal 8) so it renders with crisp flat-shaded edges under the renderer's
 * smooth-normal-averaging -- sharing vertices between faces would incorrectly round the corners.
 */
export function makeBox(sx: number, sy: number, sz: number): Mesh {
  const x = sx / 2
  const y = sy / 2
  const z = sz / 2
  const positions: number[] = []
  const indices: number[] = []

  function face(a: [number, number, number], b: [number, number, number], c: [number, number, number], d: [number, number, number]) {
    const base = positions.length / 3
    for (const p of [a, b, c, d]) positions.push(p[0], p[1], p[2])
    indices.push(base, base + 1, base + 2, base, base + 2, base + 3)
  }

  face([x, -y, -z], [x, y, -z], [x, y, z], [x, -y, z]) // +X
  face([-x, y, -z], [-x, -y, -z], [-x, -y, z], [-x, y, z]) // -X
  face([x, y, -z], [-x, y, -z], [-x, y, z], [x, y, z]) // +Y
  face([-x, -y, -z], [x, -y, -z], [x, -y, z], [-x, -y, z]) // -Y
  face([-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]) // +Z
  face([x, -y, -z], [-x, -y, -z], [-x, y, -z], [x, y, -z]) // -Z

  return { positions: new Float32Array(positions), indices: new Uint16Array(indices) }
}

// Placeholder dimensions (meters) -- exact vacuum cup size TBD, adjust here once known. Single
// source of truth for both the mesh geometry below AND the gizmo/IK tool-tip offset in
// roarm-3d-view.ts (VACUUM_CUP_TIP_OFFSET_M there imports STEM_LEN_M+CUP_LEN_M from here), so the
// gizmo/IK target keeps tracking the cup's actual suction tip once these numbers change.
export const STEM_LEN_M = 0.018
export const CUP_LEN_M = 0.014 // stem end -> cup tip
export const VACUUM_CUP_TIP_OFFSET_M = STEM_LEN_M + CUP_LEN_M

/**
 * Simple placeholder geometry for a vacuum suction gripper, standing in for
 * the mechanical two-finger gripper: a mount stem plus a flared rubber cup,
 * extending along the tool frame's local +Z. Tip sits at local Z =
 * VACUUM_CUP_TIP_OFFSET_M -- keep in sync with that constant if these radii/lengths change.
 */
// Der Sauger transportiert 2,5cm-Holzwuerfel -- vor allem hilfreich, um den Effekt von "Roll"
// ueberhaupt sehen zu koennen (der Sauger selbst ist rotationssymmetrisch um seine eigene Achse,
// ein Wuerfel am Kopf macht die Drehung sichtbar).
export const TRANSPORTED_CUBE_SIZE_M = 0.025

export function makeVacuumCup(): Mesh {
  const segments = 16
  const stem = makeFrustum(segments, 0.006, 0, 0.006, STEM_LEN_M, false)
  const cup = makeFrustum(segments, 0.006, STEM_LEN_M, 0.02, VACUUM_CUP_TIP_OFFSET_M, true)

  const positions = new Float32Array(stem.positions.length + cup.positions.length)
  positions.set(stem.positions, 0)
  positions.set(cup.positions, stem.positions.length)

  const vertexOffset = stem.positions.length / 3
  const indices = new Uint16Array(stem.indices.length + cup.indices.length)
  indices.set(stem.indices, 0)
  for (let i = 0; i < cup.indices.length; i++) indices[stem.indices.length + i] = cup.indices[i] + vertexOffset

  return { positions, indices }
}
