// One-off offline tool (derived from ../roarm-meshes/extract-robot-data.mjs): reads the Fusion 360 STL exports
// of the "Factory in a box" listed in factory-scene.json, moves them into the 3D view's robot frame, reduces
// them to a small triangle budget and writes src/workspaces/roarm/3d/factory-data.ts - plain data, no STL
// parsing at run time.
//
// Run with:  node tools/factory-meshes/extract-factory-data.mjs     (in web/)
// meshoptimizer is not a dependency of the web workspace (only this offline tool needs it). Either install it
// next to the repo, or point MESHOPTIMIZER at its index.js:
//   $env:MESHOPTIMIZER = "C:\somewhere\node_modules\meshoptimizer\index.js"
//
// Coordinate frames
// - Fusion: mm, Z up, assembly origin. robotBase.posMm is the seat of the lowest servo (where the RoArm's
//   turntable ring sits), robotBase.yawDeg the direction of base joint 0 deg about Fusion Z (0 = Fusion +X).
// - 3D view (roarm-3d-view.ts): m, Z up, origin = base_link of CHAIN in robot-data.ts. That file zeroes the
//   URDF world_to_base_link offset (70.1 mm, Waveshare's pedestal, not present on this hardware), so the
//   robot stands directly on z = 0; the lowest point of the rendered robot is link1's turntable ring at
//   ROBOT_SEAT_Z_M (= min z of link1's mesh, checked below against robot-data.ts).
// - Transformation: p_view = Rz(-yaw) * (p_fusion - base) * 0.001 + [0, 0, ROBOT_SEAT_Z_M]
//   => the servo seat measured in Fusion lands exactly under the turntable ring: neither stuck in the
//   plate nor floating.

import { readFileSync, writeFileSync } from 'node:fs'
import { fileURLToPath, pathToFileURL } from 'node:url'
import { dirname, join } from 'node:path'

const __dirname = dirname(fileURLToPath(import.meta.url))
const SCENE_FILE = join(__dirname, 'factory-scene.json')
const ROBOT_DATA_FILE = join(__dirname, '../../src/workspaces/roarm/3d/robot-data.ts')
const OUT_FILE = join(__dirname, '../../src/workspaces/roarm/3d/factory-data.ts')

const { MeshoptSimplifier } = await import(
  process.env.MESHOPTIMIZER ? pathToFileURL(process.env.MESHOPTIMIZER).href : 'meshoptimizer'
)
await MeshoptSimplifier.ready

const scene = JSON.parse(readFileSync(SCENE_FILE, 'utf8'))

// --- robot seat height (lowest point of the rendered robot) ---------------------------------------------
function link1MinZ() {
  const src = readFileSync(ROBOT_DATA_FILE, 'utf8')
  const m = src.match(/link1: \{ positions: new Float32Array\(\[([^\]]*)\]/)
  if (!m) throw new Error('link1 positions not found in robot-data.ts')
  const p = m[1].split(',').map(Number)
  let minZ = Infinity
  for (let i = 2; i < p.length; i += 3) minZ = Math.min(minZ, p[i])
  return minZ
}
// link1 hangs off base_link with origin [0,0,0] and rotates only about Z, so its mesh's min z is the robot's
// lowest point in every pose.
const ROBOT_SEAT_Z_M = link1MinZ()

// --- transformation ----------------------------------------------------------------------------------------
const base = scene.robotBase.posMm
const yawDeg = scene.robotBase.yawDeg
if (yawDeg % 90 !== 0) throw new Error(`robotBase.yawDeg must be a multiple of 90 (frame boxes stay axis-aligned), got ${yawDeg}`)
const yaw = (yawDeg * Math.PI) / 180
const cosY = Math.round(Math.cos(yaw))
const sinY = Math.round(Math.sin(yaw))

function toView(x, y, z) {
  const dx = x - base[0]
  const dy = y - base[1]
  // Rz(-yaw)
  return [(cosY * dx + sinY * dy) * 0.001, (-sinY * dx + cosY * dy) * 0.001, (z - base[2]) * 0.001 + ROBOT_SEAT_Z_M]
}

// --- binary STL --------------------------------------------------------------------------------------------
function readStl(file) {
  const buf = readFileSync(join(__dirname, file))
  const n = buf.readUInt32LE(80)
  if (84 + n * 50 !== buf.length) throw new Error(`${file}: not a binary STL`)
  const tri = new Float32Array(n * 9)
  for (let i = 0; i < n; i++) for (let k = 0; k < 9; k++) tri[i * 9 + k] = buf.readFloatLE(84 + i * 50 + 12 + k * 4)
  return tri
}

function weld(tri) {
  const remap = new Map()
  const positions = []
  const indices = new Uint32Array(tri.length / 3)
  for (let i = 0; i < tri.length; i += 3) {
    const key = `${tri[i]},${tri[i + 1]},${tri[i + 2]}`
    let id = remap.get(key)
    if (id === undefined) {
      id = positions.length / 3
      remap.set(key, id)
      positions.push(tri[i], tri[i + 1], tri[i + 2])
    }
    indices[i / 3] = id
  }
  return { positions: new Float32Array(positions), indices }
}

function compact(indices, positions) {
  const remap = new Map()
  const outPositions = []
  const outIndices = []
  for (const src of indices) {
    let id = remap.get(src)
    if (id === undefined) {
      id = outPositions.length / 3
      remap.set(src, id)
      outPositions.push(positions[src * 3], positions[src * 3 + 1], positions[src * 3 + 2])
    }
    outIndices.push(id)
  }
  if (outPositions.length / 3 >= 65536) throw new Error('>= 65536 vertices, Uint16Array indices would overflow')
  return { positions: outPositions, indices: outIndices }
}

// Topology-preserving simplify() first; CAD exports consist of many small closed bodies (screws, nuts) whose
// borders simplify() must keep, so if it stays clearly above budget, fall back to simplifySloppy().
function decimate(positions, indices, budget) {
  if (indices.length / 3 <= budget) return { indices, method: 'none', error: 0 }
  let [out, error] = MeshoptSimplifier.simplify(indices, positions, 3, budget * 3, 0.02, ['Prune'])
  let method = 'simplify'
  if (out.length / 3 > budget * 1.25) {
    let lo = 0.001
    let hi = 0.2
    let best = null
    for (let i = 0; i < 18; i++) {
      const err = Math.sqrt(lo * hi)
      const [o, e] = MeshoptSimplifier.simplifySloppy(indices, positions, 3, null, budget * 3, err)
      if (o.length / 3 > budget) lo = err
      else {
        hi = err
        best = [o, e]
      }
    }
    if (best) {
      ;[out, error] = best
      method = 'simplifySloppy'
    }
  }
  return { indices: out, method, error }
}

// --- Item frame -> axis-aligned profile boxes ----------------------------------------------------------------
// The frame is exported as one fused body (all profiles share vertices), so it can't be split into connected
// components. Instead: voxelize it (parity ray casting along Z, needs a watertight mesh), find long runs of
// solid voxels along each axis (a profile is solid along its whole length, crossing profiles only for their
// 20 mm width), group the runs' cross-section cells into profiles (dilated, so the slots don't split one
// profile into its corner/core pieces), snap the box faces to the frame's real face planes and trim boxes
// where a higher-priority profile (vertical posts first) runs through them. Pieces whose longest side is
// below MIN_PART_MM (slot nuts, brackets) are dropped.
const VOXEL_MM = 2
const MIN_RUN_MM = 40
const MIN_PART_MM = 15
const SNAP_MM = 3
const AXIS_PRIORITY = [2, 0, 1] // Z posts run through, then X, then Y profiles

function profileBoxes(tri) {
  const n = tri.length / 9
  const mn = [Infinity, Infinity, Infinity]
  const mx = [-Infinity, -Infinity, -Infinity]
  for (let i = 0; i < tri.length; i += 3)
    for (let j = 0; j < 3; j++) {
      mn[j] = Math.min(mn[j], tri[i + j])
      mx[j] = Math.max(mx[j], tri[i + j])
    }
  const R = VOXEL_MM
  const dim = mx.map((v, j) => Math.ceil((v - mn[j]) / R))
  const [NX, NY, NZ] = dim
  const vox = new Uint8Array(NX * NY * NZ)
  const at = (p) => (p[2] * NY + p[1]) * NX + p[0]

  // face planes (axis-aligned triangles), area-weighted, for snapping
  const planes = [new Map(), new Map(), new Map()]
  const bins = new Map()
  for (let t = 0; t < n; t++) {
    const o = t * 9
    const a = [tri[o], tri[o + 1], tri[o + 2]]
    const b = [tri[o + 3], tri[o + 4], tri[o + 5]]
    const c = [tri[o + 6], tri[o + 7], tri[o + 8]]
    const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]]
    const v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]]
    const cr = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
    const len = Math.hypot(...cr)
    if (len > 0)
      for (let j = 0; j < 3; j++)
        if (Math.abs(cr[j]) / len > 0.999) {
          const key = Math.round(a[j] * 100) / 100
          planes[j].set(key, (planes[j].get(key) ?? 0) + len / 2)
        }
    const x0 = Math.min(a[0], b[0], c[0])
    const x1 = Math.max(a[0], b[0], c[0])
    const y0 = Math.min(a[1], b[1], c[1])
    const y1 = Math.max(a[1], b[1], c[1])
    const i0 = Math.max(0, Math.ceil((x0 - mn[0]) / R - 0.5))
    const i1 = Math.min(NX - 1, Math.floor((x1 - mn[0]) / R - 0.5))
    const j0 = Math.max(0, Math.ceil((y0 - mn[1]) / R - 0.5))
    const j1 = Math.min(NY - 1, Math.floor((y1 - mn[1]) / R - 0.5))
    for (let i = i0; i <= i1; i++)
      for (let j = j0; j <= j1; j++) {
        const k = j * NX + i
        let list = bins.get(k)
        if (!list) bins.set(k, (list = []))
        list.push(t)
      }
  }
  let oddColumns = 0
  for (const [k, ts] of bins) {
    const i = k % NX
    const j = (k - i) / NX
    // tiny irrational offsets keep the ray off triangle edges
    const px = mn[0] + (i + 0.5) * R + 1.07e-4
    const py = mn[1] + (j + 0.5) * R + 2.31e-4
    const zs = []
    for (const t of ts) {
      const o = t * 9
      const [ax, ay, bx, by, cx, cy] = [tri[o], tri[o + 1], tri[o + 3], tri[o + 4], tri[o + 6], tri[o + 7]]
      const d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
      if (Math.abs(d) < 1e-12) continue
      const l1 = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / d
      const l2 = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / d
      const l3 = 1 - l1 - l2
      if (l1 < 0 || l2 < 0 || l3 < 0) continue
      zs.push(l1 * tri[o + 2] + l2 * tri[o + 5] + l3 * tri[o + 8])
    }
    zs.sort((p, q) => p - q)
    if (zs.length % 2) {
      oddColumns++
      continue
    }
    for (let q = 0; q < zs.length; q += 2) {
      const z0 = Math.max(0, Math.ceil((zs[q] - mn[2]) / R - 0.5))
      const z1 = Math.min(NZ - 1, Math.floor((zs[q + 1] - mn[2]) / R - 0.5))
      for (let z = z0; z <= z1; z++) vox[at([i, j, z])] = 1
    }
  }
  if (oddColumns > 0) throw new Error(`frame mesh not watertight (${oddColumns} odd ray columns)`)

  const planeList = planes.map((m) =>
    [...m.entries()].filter(([, area]) => area >= 200).map(([c]) => c), // >= 2 cm^2
  )
  const snap = (axis, v) => {
    let best = v
    let bestD = SNAP_MM
    for (const c of planeList[axis]) {
      const d = Math.abs(c - v)
      if (d < bestD) {
        bestD = d
        best = c
      }
    }
    return best
  }

  const minRun = Math.round(MIN_RUN_MM / R)
  const dil = 2 // voxels, bridges the slots inside one profile's cross-section
  const boxes = []
  for (const A of [0, 1, 2]) {
    const U = (A + 1) % 3
    const W = (A + 2) % 3
    const runs = new Map() // cell -> [[a0,a1],...]
    const p = [0, 0, 0]
    for (let u = 0; u < dim[U]; u++)
      for (let w = 0; w < dim[W]; w++) {
        p[U] = u
        p[W] = w
        let s = -1
        for (let a = 0; a <= dim[A]; a++) {
          p[A] = a
          const on = a < dim[A] && vox[at(p)]
          if (on && s < 0) s = a
          if (!on && s >= 0) {
            if (a - s >= minRun) {
              const key = u * 65536 + w
              const list = runs.get(key) ?? []
              list.push([s, a])
              runs.set(key, list)
            }
            s = -1
          }
        }
      }
    const seen = new Set()
    for (const start of runs.keys()) {
      if (seen.has(start)) continue
      seen.add(start)
      const stack = [start]
      const cells = []
      while (stack.length) {
        const c = stack.pop()
        cells.push(c)
        const u = Math.floor(c / 65536)
        const w = c % 65536
        for (let du = -dil; du <= dil; du++)
          for (let dw = -dil; dw <= dil; dw++) {
            const k2 = (u + du) * 65536 + (w + dw)
            if (runs.has(k2) && !seen.has(k2)) {
              seen.add(k2)
              stack.push(k2)
            }
          }
      }
      const iv = cells.flatMap((c) => runs.get(c)).sort((p1, p2) => p1[0] - p2[0])
      const segs = []
      for (const r of iv) {
        const last = segs[segs.length - 1]
        if (last && r[0] <= last[1]) last[1] = Math.max(last[1], r[1])
        else segs.push([...r])
      }
      for (const [a0, a1] of segs) {
        let u0 = Infinity
        let u1 = -Infinity
        let w0 = Infinity
        let w1 = -Infinity
        for (const c of cells) {
          if (!runs.get(c).some((r) => r[0] < a1 && r[1] > a0)) continue
          const u = Math.floor(c / 65536)
          const w = c % 65536
          u0 = Math.min(u0, u)
          u1 = Math.max(u1, u)
          w0 = Math.min(w0, w)
          w1 = Math.max(w1, w)
        }
        const lo = [0, 0, 0]
        const hi = [0, 0, 0]
        lo[A] = a0
        hi[A] = a1
        lo[U] = u0
        hi[U] = u1 + 1
        lo[W] = w0
        hi[W] = w1 + 1
        boxes.push({
          axis: A,
          min: lo.map((v, j) => snap(j, mn[j] + v * R)),
          max: hi.map((v, j) => snap(j, mn[j] + v * R)),
        })
      }
    }
  }

  // trim along the profile axis where a higher-priority profile fully covers the cross-section
  boxes.sort((p, q) => AXIS_PRIORITY.indexOf(p.axis) - AXIS_PRIORITY.indexOf(q.axis))
  for (let i = 0; i < boxes.length; i++) {
    const b = boxes[i]
    const A = b.axis
    for (let k = 0; k < i; k++) {
      const c = boxes[k]
      if (c.axis === A) continue
      const covers = [0, 1, 2].every((j) => j === A || (c.min[j] <= b.min[j] + 0.01 && c.max[j] >= b.max[j] - 0.01))
      if (!covers || c.max[A] <= b.min[A] || c.min[A] >= b.max[A]) continue
      if (c.min[A] <= b.min[A] + 0.01) b.min[A] = Math.max(b.min[A], c.max[A])
      else if (c.max[A] >= b.max[A] - 0.01) b.max[A] = Math.min(b.max[A], c.min[A])
    }
  }
  const kept = boxes.filter((b) => Math.max(...b.max.map((v, j) => v - b.min[j])) >= MIN_PART_MM && b.max.every((v, j) => v > b.min[j]))
  kept.sort((p, q) => p.axis - q.axis || p.min[2] - q.min[2] || p.min[1] - q.min[1] || p.min[0] - q.min[0])

  // how much of the solid was not covered by any box (dropped small parts, profile end caps)
  let solid = 0
  let uncovered = 0
  for (let z = 0; z < NZ; z++)
    for (let y = 0; y < NY; y++)
      for (let x = 0; x < NX; x++) {
        if (!vox[at([x, y, z])]) continue
        solid++
        const c = [x, y, z].map((v, j) => mn[j] + (v + 0.5) * R)
        if (!kept.some((b) => c.every((v, j) => v >= b.min[j] && v <= b.max[j]))) uncovered++
      }
  return { boxes: kept, solidCm3: (solid * R ** 3) / 1000, uncoveredCm3: (uncovered * R ** 3) / 1000 }
}

// --- process parts -------------------------------------------------------------------------------------------
const round = (v, digits) => Math.round(v * 10 ** digits) / 10 ** digits
const meshes = []
const profileParts = []
let totalTris = 0
for (const part of scene.parts) {
  if (!part.enabled) {
    console.log(`${part.name}: disabled`)
    continue
  }
  const tri = readStl(part.file)
  if (part.mode === 'profiles') {
    const { boxes, solidCm3, uncoveredCm3 } = profileBoxes(tri)
    const viewBoxes = boxes.map((b) => {
      const p0 = toView(...b.min)
      const p1 = toView(...b.max)
      return {
        center: p0.map((v, j) => round((v + p1[j]) / 2, 5)),
        size: p0.map((v, j) => round(Math.abs(p1[j] - v), 5)),
      }
    })
    profileParts.push({ part, boxes: viewBoxes })
    totalTris += viewBoxes.length * 12
    console.log(
      `${part.name}: ${tri.length / 9} tris -> ${viewBoxes.length} boxes (${viewBoxes.length * 12} tris); ` +
        `solid ${solidCm3.toFixed(0)} cm3, not covered by boxes ${uncoveredCm3.toFixed(1)} cm3`,
    )
    for (const b of boxes)
      console.log(`   axis ${'XYZ'[b.axis]}  min ${b.min.join('/')}  max ${b.max.join('/')}  mm`)
    continue
  }
  const { positions, indices } = weld(tri)
  const { indices: dec, method, error } = decimate(positions, indices, part.triangleBudget)
  const c = compact(dec, positions)
  const viewPos = []
  for (let i = 0; i < c.positions.length; i += 3) viewPos.push(...toView(c.positions[i], c.positions[i + 1], c.positions[i + 2]).map((v) => round(v, 4)))
  meshes.push({ part, positions: viewPos, indices: c.indices })
  totalTris += c.indices.length / 3
  console.log(
    `${part.name}: ${tri.length / 9} tris -> ${c.indices.length / 3} tris, ${c.positions.length / 3} verts (${method}` +
      (method !== 'none' ? `, error ${error.toFixed(4)}` : '') +
      ')',
  )
}
console.log(`total: ${totalTris} triangles, ROBOT_SEAT_Z_M = ${ROBOT_SEAT_Z_M}`)

// --- emit ------------------------------------------------------------------------------------------------------
const meta = (part) =>
  `name: ${JSON.stringify(part.name)}, color: ${JSON.stringify(part.color)}, material: ${JSON.stringify(part.material)}`
const out = `// GENERATED FILE — do not edit by hand.
// Produced by web/tools/factory-meshes/extract-factory-data.mjs from factory-scene.json and the Fusion 360 STL
// exports next to it. Frame: the 3D view's robot frame (m, Z up, origin = base_link, see robot-data.ts).
import type { Mesh } from './robot-data.ts'
import type { Vec3 } from './math.ts'

export interface FactoryMaterial {
  metallic: number
  roughness: number
}

export interface FactoryMeshPart {
  name: string
  color: [number, number, number]
  material: FactoryMaterial
  mesh: Mesh
}

/** Axis-aligned box (Item profile); also meant as the collision model of the frame. */
export interface FactoryBox {
  center: Vec3
  size: Vec3
}

export interface FactoryBoxPart {
  name: string
  color: [number, number, number]
  material: FactoryMaterial
  boxes: FactoryBox[]
}

/** Robot base in Fusion assembly coordinates (mm) and its yaw about Fusion Z (deg), from factory-scene.json. */
export const ROBOT_BASE_FUSION_MM: Vec3 = ${JSON.stringify(base)}
export const ROBOT_BASE_YAW_DEG = ${yawDeg}
/** Height of the servo seat (lowest point of link1) in the view frame - where robotBase.posMm lands. */
export const ROBOT_SEAT_Z_M = ${ROBOT_SEAT_Z_M}

export const FACTORY_MESH_PARTS: FactoryMeshPart[] = [
${meshes
  .map(
    (m) =>
      `  { ${meta(m.part)}, mesh: { positions: new Float32Array(${JSON.stringify(m.positions)}), indices: new Uint16Array(${JSON.stringify(m.indices)}) } },`,
  )
  .join('\n')}
]

export const FACTORY_BOX_PARTS: FactoryBoxPart[] = [
${profileParts
  .map((p) => `  { ${meta(p.part)}, boxes: [\n${p.boxes.map((b) => `    { center: ${JSON.stringify(b.center)}, size: ${JSON.stringify(b.size)} },`).join('\n')}\n  ] },`)
  .join('\n')}
]
`
writeFileSync(OUT_FILE, out)
console.log(`\nWrote ${OUT_FILE}`)
