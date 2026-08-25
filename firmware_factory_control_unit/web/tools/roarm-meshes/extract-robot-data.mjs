// One-off offline tool: reads the original RoArm-M3 STL meshes + hardcoded
// URDF joint data (transcribed from tools/source-assets/roarm_m3.urdf),
// decimates each mesh down to a small triangle budget with meshoptimizer,
// and writes the result as a plain-data TS module (src/robotData.ts) that
// the embedded-friendly app imports at build time — no STL/URDF parsing,
// no mesh loading, at runtime.
//
// Run with: node tools/extract-robot-data.mjs

import { readFileSync, writeFileSync } from 'node:fs'
import { fileURLToPath } from 'node:url'
import { dirname, join } from 'node:path'
import { MeshoptSimplifier } from 'meshoptimizer'

const __dirname = dirname(fileURLToPath(import.meta.url))
const MESH_DIR = join(__dirname, 'source-assets/meshes/roarm_m3')
const OUT_FILE = join(__dirname, '../../src/apps/roarm3d/robot-data.ts')

const MESH_SCALE = 0.001 // URDF `<mesh scale="0.001 0.001 0.001">`

// Full-fidelity by default (only losslessly welding duplicate STL vertices).
// Per-link, moderate targets here instead, using the topology-preserving
// `simplify()` (not `simplifySloppy`) — it needs an already-welded, indexed
// mesh as input, which is exactly what `weld()` below produces.
// TEMPORARILY EMPTIED (testweise Original-Meshes ohne Dekimierung, s. Absprache) — war
// { link2: 3000 } (9364 -> 3000 tris, die einzige tatsaechlich dekimierte der 4 aktuell
// gerenderten Meshes). Bei Bedarf wiederherstellen, falls die volle Aufloesung zu gross fuers
// Flash-Budget wird.
const DECIMATE_TARGETS = {}
const TARGET_ERROR = 0.02 // fraction of mesh extent; generous but not "sloppy"

// --- binary STL parsing -----------------------------------------------

function parseBinaryStl(buffer) {
  const triCount = buffer.readUInt32LE(80)
  const positions = new Float32Array(triCount * 3 * 3)
  let offset = 84
  let w = 0
  for (let i = 0; i < triCount; i++) {
    offset += 12 // skip facet normal, we recompute per-face normals at render time
    for (let v = 0; v < 3; v++) {
      positions[w++] = buffer.readFloatLE(offset) * MESH_SCALE
      positions[w++] = buffer.readFloatLE(offset + 4) * MESH_SCALE
      positions[w++] = buffer.readFloatLE(offset + 8) * MESH_SCALE
      offset += 12
    }
    offset += 2 // attribute byte count
  }
  return positions
}

// --- decimation ----------------------------------------------------------

await MeshoptSimplifier.ready

// Topology-preserving simplification of an already-welded, indexed mesh.
function simplifyTopology(positions, indices, targetTriangles, targetError) {
  const [simplified, resultError] = MeshoptSimplifier.simplify(
    new Uint32Array(indices),
    new Float32Array(positions),
    3,
    targetTriangles * 3,
    targetError,
    ['Prune'],
  )
  return { ...compact(simplified, positions), resultError }
}

// STL stores 3 independent vertices per triangle even along shared edges;
// merge byte-identical positions so adjacent triangles reference the same
// vertex, which is a lossless size reduction (no shape change at all).
function weld(positions) {
  const remap = new Map()
  const outPositions = []
  const outIndices = new Array(positions.length / 3)
  for (let i = 0; i < positions.length; i += 3) {
    const key = `${positions[i]},${positions[i + 1]},${positions[i + 2]}`
    let dstIdx = remap.get(key)
    if (dstIdx === undefined) {
      dstIdx = outPositions.length / 3
      remap.set(key, dstIdx)
      outPositions.push(positions[i], positions[i + 1], positions[i + 2])
    }
    outIndices[i / 3] = dstIdx
  }
  if (outPositions.length / 3 >= 65536) {
    throw new Error(`mesh has >= 65536 unique vertices after welding; Uint16Array indices would overflow`)
  }
  return { positions: outPositions, indices: outIndices }
}

// Drop unused vertices from the (large) original position buffer and remap
// indices to a small, tightly-packed vertex array. Used for the decimated
// path, where MeshoptSimplifier returns indices into the original buffer.
function compact(indices, positions) {
  const remap = new Map()
  const outPositions = []
  const outIndices = new Array(indices.length)
  for (let i = 0; i < indices.length; i++) {
    const srcIdx = indices[i]
    let dstIdx = remap.get(srcIdx)
    if (dstIdx === undefined) {
      dstIdx = outPositions.length / 3
      remap.set(srcIdx, dstIdx)
      outPositions.push(positions[srcIdx * 3], positions[srcIdx * 3 + 1], positions[srcIdx * 3 + 2])
    }
    outIndices[i] = dstIdx
  }
  return { positions: outPositions, indices: outIndices }
}

function round(arr, digits = 5) {
  const f = 10 ** digits
  return arr.map((v) => Math.round(v * f) / f)
}

// link5's own mesh (per Waveshare's CAD: a servo mount plate sized for the standard pinch
// gripper) is deliberately NOT rendered — this arm carries a vacuum cup instead, and link5.stl
// visibly shows gripper/servo-mount geometry that doesn't physically exist on this hardware.
// base_link is dropped too: this arm's turntable ring sits directly on the floor, not on
// Waveshare's own base housing/pedestal, so base_link.stl doesn't match this mounting either.
const LINKS = ['link1', 'link2', 'link3', 'link4']
const meshes = {}
for (const link of LINKS) {
  const raw = readFileSync(join(MESH_DIR, `${link}.stl`))
  const rawPositions = parseBinaryStl(raw)
  const originalTris = rawPositions.length / 9

  let { positions: pos, indices: idx } = weld(rawPositions)
  let resultError = null

  const target = DECIMATE_TARGETS[link]
  if (target && idx.length / 3 > target) {
    ;({ positions: pos, indices: idx, resultError } = simplifyTopology(pos, idx, target, TARGET_ERROR))
  }

  meshes[link] = { positions: round(pos), indices: idx }
  console.log(
    `${link}: ${originalTris} tris -> ${idx.length / 3} tris, ${pos.length / 3} verts` +
      (resultError !== null ? `, error ${resultError.toFixed(4)}` : ''),
  )
}

// --- kinematic chain (hand-transcribed from roarm_m3.urdf) ---------------
// Serial chain only: world -> base_link -> link1 -> ... -> link5 -> hand_tcp.
// The gripper_link branch is dropped (this arm carries a vacuum cup instead of
// Waveshare's pinch gripper); hand_tcp instead carries a procedural vacuum-cup
// placeholder, generated at runtime, bolted straight onto link5's servo output
// shaft: link5 has no offset/rotation of its own beyond the servo's spin, i.e.
// the tool frame is exactly that servo's own rotated frame.

function quatFromEulerXYZ(roll, pitch, yaw) {
  const half = (a) => [Math.sin(a / 2), Math.cos(a / 2)]
  const [sx, cx] = half(roll)
  const [sy, cy] = half(pitch)
  const [sz, cz] = half(yaw)
  const mul = (a, b) => [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  ]
  const qx = [sx, 0, 0, cx]
  const qy = [0, sy, 0, cy]
  const qz = [0, 0, sz, cz]
  return mul(mul(qz, qy), qx)
}

const segments = [
  {
    // URDF's world_to_base_link origin is [0,0,0.0701] -- that 70.1mm accounts for Waveshare's
    // own base housing, which isn't rendered (no mesh) or physically present on this hardware:
    // this arm's turntable ring is bolted directly to the floor/table, no pedestal underneath.
    // Zeroed here so link1 sits exactly on the floor grid (z=0) instead of visibly floating
    // 70mm above it. Purely a rendering-frame choice -- roarm-kinematics.ts (the backend-
    // authoritative FK/IK used for the actual Pose-mm readout) is untouched, still its own
    // reference frame.
    name: 'base_link',
    mesh: null,
    jointType: 'fixed',
    origin: [0, 0, 0],
    quat: quatFromEulerXYZ(0, 0, 0),
  },
  {
    name: 'link1',
    mesh: 'link1',
    jointType: 'revolute',
    origin: [0, 0, 0],
    quat: quatFromEulerXYZ(0, 0, 0),
    axis: [0, 0, 1],
    limit: [-3.1416, 3.1416],
  },
  {
    name: 'link2',
    mesh: 'link2',
    jointType: 'revolute',
    origin: [0, 0, 0.051959],
    quat: quatFromEulerXYZ(-1.5708, -1.5708, 0),
    axis: [0, 0, 1],
    limit: [-1.5708, 1.5708],
  },
  {
    name: 'link3',
    mesh: 'link3',
    jointType: 'revolute',
    origin: [0.236815, 0.030002, 0],
    quat: quatFromEulerXYZ(0, 0, 1.5708),
    axis: [0, 0, 1],
    limit: [-1, 2.95],
  },
  {
    name: 'link4',
    mesh: 'link4',
    jointType: 'revolute',
    origin: [0, -0.144586, 0],
    quat: quatFromEulerXYZ(0, 0, 0),
    axis: [0, 0, 1],
    limit: [-1.5708, 1.5708],
  },
  {
    name: 'link5',
    mesh: null,
    jointType: 'revolute',
    origin: [0.015147, -0.053653, 0],
    quat: quatFromEulerXYZ(1.5708, 1.5708, 0),
    axis: [0, 0, 1],
    limit: [-3.1416, 3.1416],
  },
]

// --- emit ------------------------------------------------------------

const meshEntries = LINKS.map(
  (link) =>
    `  ${link}: { positions: new Float32Array(${JSON.stringify(meshes[link].positions)}), indices: new Uint16Array(${JSON.stringify(meshes[link].indices)}) },`,
).join('\n')

const segmentEntries = segments
  .map((s) => {
    const fields = [
      `name: ${JSON.stringify(s.name)}`,
      `mesh: ${s.mesh ? JSON.stringify(s.mesh) : 'null'}`,
      `jointType: ${JSON.stringify(s.jointType)}`,
      `origin: ${JSON.stringify(round(s.origin, 6))}`,
      `quat: ${JSON.stringify(round(s.quat, 6))}`,
    ]
    if (s.jointType === 'revolute') {
      fields.push(`axis: ${JSON.stringify(s.axis)}`)
      fields.push(`limit: ${JSON.stringify(s.limit)}`)
    }
    return `  { ${fields.join(', ')} },`
  })
  .join('\n')

const out = `// GENERATED FILE — do not edit by hand.
// Produced by web/tools/roarm-meshes/extract-robot-data.mjs from source-assets/*
// (Waveshare RoArm-M3 URDF + STL meshes, full fidelity except: ${JSON.stringify(DECIMATE_TARGETS)}).
import type { Vec3, Quat } from './math.js'

export interface Mesh {
  positions: Float32Array
  indices: Uint16Array
}

export interface Segment {
  name: string
  mesh: string | null
  jointType: 'fixed' | 'revolute'
  origin: Vec3
  quat: Quat
  axis?: Vec3
  limit?: [number, number]
}

export const MESHES: Record<string, Mesh> = {
${meshEntries}
}

// Serial kinematic chain, base to tip. base_link's origin already folds in
// the URDF's fixed world_to_base_link joint.
export const CHAIN: Segment[] = [
${segmentEntries}
]
`

writeFileSync(OUT_FILE, out)
console.log(`\nWrote ${OUT_FILE}`)
