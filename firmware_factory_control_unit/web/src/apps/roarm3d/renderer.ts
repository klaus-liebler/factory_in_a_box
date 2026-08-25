// The 2D overlay renderer: draws the reference grid and the gizmo on a
// transparent Canvas2D layer stacked on top of the WebGL mesh renderer
// (see oglRenderer.ts). It also owns `project()`, the screen-space math the
// gizmo needs for hit-testing and drag math — kept independent of the WebGL
// side so gizmo.ts doesn't need to know anything about OGL.

import type { Vec3, Quat, Pose } from './math.js'
import { rotateVec3, sub } from './math.js'

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

  // Setzt NUR die Zeichenflaechen-Aufloesung (canvas.width/height, das Bitmap-Pixelraster) --
  // die CSS-Box-Groesse (was tatsaechlich auf dem Bildschirm Platz einnimmt) kommt bewusst
  // AUSSCHLIESSLICH aus CSS/Flexbox (position:absolute; inset:0 im Container, s.
  // roarm-3d-view.ts), kein per JS gesetztes style.width/height mehr. Frueher wurden beide Werte
  // hier gesetzt -- wenn `width`/`height` (aus container.clientWidth/Height) je einmal veraltet
  // waren (z.B. bevor ein Layout-getriebener Groessenwechsel den ResizeObserver erneut ausgeloest
  // hatte), driftete dadurch die CSS-Box selbst vom Flexbox-Layout weg, unabhaengig vom
  // eigentlichen Container. Jetzt kann die CSS-Box nie von dem abweichen, was Flexbox vorgibt --
  // nur die interne Bitmap-Aufloesung wird bei Bedarf nachgezogen.
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
