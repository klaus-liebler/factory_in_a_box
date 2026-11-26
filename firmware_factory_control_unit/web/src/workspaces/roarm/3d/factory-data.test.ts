import { describe, expect, it } from 'vitest'
import { FACTORY_BOX_PARTS, FACTORY_MESH_PARTS, ROBOT_SEAT_Z_M } from './factory-data.ts'
import { MESHES } from './robot-data.ts'
import { makeBoxes } from './primitives.ts'

// The generated factory model must sit where the robot stands: the measured servo seat (factory-scene.json
// robotBase) is mapped onto the lowest point of the rendered robot, so the arm neither sticks in its mounting
// plate nor floats above it.
describe('factory-data', () => {
  it('maps the servo seat onto the lowest point of link1', () => {
    const p = MESHES.link1.positions
    let minZ = Infinity
    for (let i = 2; i < p.length; i += 3) minZ = Math.min(minZ, p[i])
    expect(ROBOT_SEAT_Z_M).toBeCloseTo(minZ, 6)
  })

  it('keeps every factory part out of the turntable volume of the robot', () => {
    const r = 0.03 // turntable ring radius is about 27 mm (link1 mesh)
    for (const part of FACTORY_MESH_PARTS) {
      const p = part.mesh.positions
      for (let i = 0; i < p.length; i += 3) {
        const inside = Math.hypot(p[i], p[i + 1]) < r && p[i + 2] > ROBOT_SEAT_Z_M + 0.0005 && p[i + 2] < 0.05
        expect(inside, `${part.name} vertex ${i / 3}`).toBe(false)
      }
    }
  })

  it('puts the RoArm mounting plate below the robot', () => {
    const plate = FACTORY_MESH_PARTS.find((m) => m.name === 'roarmPlatte')
    expect(plate).toBeDefined()
    let maxZ = -Infinity
    const p = plate!.mesh.positions
    for (let i = 2; i < p.length; i += 3) maxZ = Math.max(maxZ, p[i])
    expect(maxZ).toBeLessThanOrEqual(ROBOT_SEAT_Z_M)
    expect(maxZ).toBeGreaterThan(ROBOT_SEAT_Z_M - 0.01)
  })

  it('describes the Item frame as boxes of real profile size', () => {
    const frame = FACTORY_BOX_PARTS.find((b) => b.name === 'itemGestell')
    expect(frame).toBeDefined()
    expect(frame!.boxes.length).toBeGreaterThanOrEqual(12)
    for (const b of frame!.boxes) {
      const sorted = [...b.size].sort((x, y) => x - y)
      expect(sorted[0]).toBeGreaterThanOrEqual(0.015) // nothing smaller than a profile
      expect(sorted[2]).toBeGreaterThan(sorted[1]) // elongated
    }
  })

  it('merges boxes into one mesh', () => {
    const m = makeBoxes([
      { center: [1, 0, 0], size: [2, 2, 2] },
      { center: [0, 0, 5], size: [1, 1, 1] },
    ])
    expect(m.positions.length).toBe(2 * 24 * 3)
    expect(m.indices.length).toBe(72)
    expect(Math.max(...m.indices)).toBe(47)
    expect(m.positions[0]).toBe(2) // +X face of the first box
  })
})
