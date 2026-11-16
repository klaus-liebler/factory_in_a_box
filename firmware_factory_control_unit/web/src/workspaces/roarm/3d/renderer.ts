// The 2D overlay renderer: draws the reference grid and the gizmo on a
// transparent Canvas2D layer stacked on top of the WebGL mesh renderer
// (see oglRenderer.ts). It also owns `project()`, the screen-space math the
// gizmo needs for hit-testing and drag math — kept independent of the WebGL
// side so gizmo.ts doesn't need to know anything about OGL.

import type { Vec3, Quat, Pose } from './math.ts'
import { rotateVec3, sub } from './math.ts'

export interface Camera {
  pose: Pose // camera-to-world
  fovY: number // radians
}

export class Renderer {
  canvas: HTMLCanvasElement
  ctx: CanvasRenderingContext2D
  width = 0
  height = 0

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas
    this.ctx = canvas.getContext('2d')!
  }

  // Sets ONLY the drawing surface resolution (canvas.width/height, the bitmap pixel grid) - the CSS box size
  // (what actually takes up room on screen) deliberately comes ONLY from CSS/flexbox (position:absolute;
  // inset:0 in the container, see roarm-3d-view.ts), no style.width/height set by JS any more. Both used to
  // be set here - whenever `width`/`height` (from container.clientWidth/Height) were stale once (e.g. before
  // a layout-driven size change had triggered the ResizeObserver again), the CSS box itself drifted away from
  // the flexbox layout, independent of the actual container. Now the CSS box can never differ from what
  // flexbox dictates - only the internal bitmap resolution follows when needed.
  resize(width: number, height: number, dpr: number) {
    this.width = width
    this.height = height
    this.canvas.width = Math.round(width * dpr)
    this.canvas.height = Math.round(height * dpr)
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
  }

  // Projects a world-space point to screen pixels; returns null if behind camera.
  project(camera: Camera, world: Vec3): [number, number, number] | null {
    const camQuatInv: Quat = [
      -camera.pose.quat[0],
      -camera.pose.quat[1],
      -camera.pose.quat[2],
      camera.pose.quat[3],
    ]
    const local = rotateVec3(camQuatInv, sub(world, camera.pose.pos))
    // Camera looks down -Z in its local frame.
    const z = -local[2]
    if (z < 0.01) return null
    const f = this.height / 2 / Math.tan(camera.fovY / 2)
    const x = this.width / 2 + (local[0] * f) / z
    const y = this.height / 2 - (local[1] * f) / z
    return [x, y, z]
  }

  clear() {
    this.ctx.clearRect(0, 0, this.width, this.height)
  }
}

export function makeGrid(size: number, divisions: number): { a: Vec3; b: Vec3 }[] {
  const lines: { a: Vec3; b: Vec3 }[] = []
  const half = size / 2
  const step = size / divisions
  for (let i = 0; i <= divisions; i++) {
    const p = -half + i * step
    lines.push({ a: [p, -half, 0], b: [p, half, 0] })
    lines.push({ a: [-half, p, 0], b: [half, p, 0] })
  }
  return lines
}

export function drawLines(
  renderer: Renderer,
  camera: Camera,
  lines: { a: Vec3; b: Vec3 }[],
  color: string,
) {
  const { ctx } = renderer
  ctx.save()
  ctx.strokeStyle = color
  ctx.lineWidth = 1
  ctx.beginPath()
  for (const { a, b } of lines) {
    const pa = renderer.project(camera, a)
    const pb = renderer.project(camera, b)
    if (!pa || !pb) continue
    ctx.moveTo(pa[0], pa[1])
    ctx.lineTo(pb[0], pb[1])
  }
  ctx.stroke()
  ctx.restore()
}
