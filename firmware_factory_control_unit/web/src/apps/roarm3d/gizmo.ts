// Minimal hand-rolled translate/rotate gizmo drawn with the same Canvas2D
// projection as the renderer. Translate handles follow `space` (world axes by
// default, or the target's own local axes -- see the `space` field); rotate
// rings always use the target's own local axes (tied to the arm's kinematic
// tilt/roll DOF, "world" wouldn't be meaningful there).

import type { Camera, Renderer } from './renderer.js'
import type { Pose, Quat, Vec3 } from './math.js'
import {
  add,
  dot,
  normalize,
  quatFromAxisAngle,
  quatMultiply,
  quatNormalize,
  rotateVec3,
  scale,
  sub,
} from './math.js'

export type GizmoMode = 'translate' | 'rotate'

const AXES: Vec3[] = [
  [1, 0, 0],
  [0, 1, 0],
  [0, 0, 1],
]
const AXIS_COLORS = ['#e5484d', '#46a758', '#0091ff']
// Rotate-Ringe: nur Achse 0 (rot, Tilt) und 2 (blau, Roll) -- diese Kette hat genau EINEN echten
// Tilt-Freiheitsgrad (ueber Schulter/Ellbogen/Handgelenk-Summe) plus Roll, keine unabhaengige
// zweite Kippachse. Ein Ring fuer Achse 1 (gruen) wuerde eine Drehung anbieten, die die Kinematik
// gar nicht ausfuehren kann -- roarm-3d-view.ts bildete das bisher einfach auf denselben
// targetTiltSum wie Achse 0 ab, was beim Ziehen von Gruen zu einer Drehung um eine andere Achse
// fuehrte, als der Ring suggerierte. Translate-Pfeile (alle 3 Achsen) sind davon nicht betroffen --
// eine kartesische Zielposition in jede Richtung ist (innerhalb der Reichweite) tatsaechlich
// erreichbar, anders als eine freie 3D-Werkzeug-Orientierung.
const ROTATE_AXES = [0, 2] as const
const HANDLE_LENGTH = 0.09
const RING_RADIUS = 0.09
const HIT_RADIUS_PX = 10
// Sample point for estimating the local screen-pixels-per-meter rate along
// a translate axis. Must be small relative to the scene (this robot is
// ~0.4m across): sampling a full 1m away for the rate estimate put the
// second point at a meaningfully different depth whenever the axis had any
// component toward/away from the camera, which perspective-distorts the
// estimated rate — a tiny mouse move could then read as "few pixels per
// meter" and get amplified into a huge world-space jump. 2cm keeps both
// sample points close enough in depth that the local linear approximation
// is actually accurate.
const RATE_SAMPLE_DISTANCE = 0.02

interface DragState {
  axis: 0 | 1 | 2
  mode: GizmoMode
  // translate: fixed reference frame captured at drag *start* — everything
  // below is measured relative to this, never to the pose's current
  // (already-dragged) position, so the drag can't feed back on itself.
  startPos?: Vec3
  dir?: Vec3
  originScreen?: [number, number]
  axisDir2D?: [number, number]
  unitsPerPixel?: number
  // pixels-along-axis at the exact mousedown point, relative to
  // originScreen — subtracted out so wherever along the handle you grab,
  // the drag starts at zero offset instead of jumping to "as if you'd
  // dragged from the pivot to your cursor" on the very first move.
  grabOffsetPx?: number
  // rotate: last seen screen-space angle around the target's screen position
  lastAngle?: number
  signFlip?: number
}

export type GizmoSpace = 'local' | 'world'

export class Gizmo {
  mode: GizmoMode = 'translate'
  // Nur die Translate-Pfeile betrifft das (Rotate-Ringe sind an die kinematisch bedeutsamen
  // Tilt-/Roll-Achsen des Werkzeugs gekoppelt, s. main.ts' updateDrag()-Auswertung, "Welt" ergibt
  // dort keinen zusaetzlichen Sinn). 'world': Pfeile zeigen fix in Welt-X/Y/Z, unabhaengig von der
  // aktuellen Werkzeug-Ausrichtung -- Default, weil sich sonst nach jeder Drehung die Bedeutung
  // von "ziehe den roten Pfeil" aendert.
  space: GizmoSpace = 'world'
  private renderer: Renderer
  private hoverAxis: 0 | 1 | 2 | null = null
  private drag: DragState | null = null

  constructor(renderer: Renderer) {
    this.renderer = renderer
  }

  get dragging() {
    return this.drag !== null
  }

  private translateAxisDir(pose: Pose, i: number): Vec3 {
    return this.space === 'world' ? AXES[i] : rotateVec3(pose.quat, AXES[i])
  }

  // ringQuat: die Ausrichtung, gegen die die ROTATE-Ringe gezeichnet/getroffen-getestet werden --
  // per Default pose.quat, aber der Aufrufer kann eine davon abweichende Ausrichtung uebergeben
  // (s. roarm-3d-view.ts' tiltRingQuat()): der Tilt-Ring (Achse 0) soll NICHT mitdrehen, wenn nur
  // Roll (Achse 2) gezogen wird -- Roll ist kinematisch das LETZTE Kettenglied und aendert die
  // Tilt-Achse selbst nicht, mit dem vollen pose.quat (das Roll schon eingerechnet hat) drehte sich
  // der Tilt-Ring bislang optisch mit, obwohl seine tatsaechliche Bedeutung unveraendert blieb.
  // Betrifft nur die Ringe -- Translate-Pfeile nutzen weiterhin ausschliesslich pose.quat (ueber
  // translateAxisDir()/this.space).
  render(camera: Camera, pose: Pose, ringQuat: Quat = pose.quat) {
    const { ctx } = this.renderer
    if (this.mode === 'translate') {
      for (let i = 0; i < 3; i++) {
        const dir = this.translateAxisDir(pose, i)
        const tip = add(pose.pos, scale(dir, HANDLE_LENGTH))
        const p0 = this.renderer.project(camera, pose.pos)
        const p1 = this.renderer.project(camera, tip)
        if (!p0 || !p1) continue
        ctx.save()
        ctx.strokeStyle = AXIS_COLORS[i]
        ctx.fillStyle = AXIS_COLORS[i]
        ctx.lineWidth = i === this.hoverAxis ? 4 : 2.5
        ctx.beginPath()
        ctx.moveTo(p0[0], p0[1])
        ctx.lineTo(p1[0], p1[1])
        ctx.stroke()
        ctx.beginPath()
        ctx.arc(p1[0], p1[1], 5, 0, Math.PI * 2)
        ctx.fill()
        ctx.restore()
      }
    } else {
      for (const i of ROTATE_AXES) {
        const [u, v] = ringBasis(i)
        const worldU = rotateVec3(ringQuat, u)
        const worldV = rotateVec3(ringQuat, v)
        ctx.save()
        ctx.strokeStyle = AXIS_COLORS[i]
        ctx.lineWidth = i === this.hoverAxis ? 4 : 2.5
        ctx.beginPath()
        let started = false
        for (let s = 0; s <= 32; s++) {
          const a = (s / 32) * Math.PI * 2
          const p3 = add(pose.pos, add(scale(worldU, Math.cos(a) * RING_RADIUS), scale(worldV, Math.sin(a) * RING_RADIUS)))
          const p2 = this.renderer.project(camera, p3)
          if (!p2) continue
          if (!started) {
            ctx.moveTo(p2[0], p2[1])
            started = true
          } else {
            ctx.lineTo(p2[0], p2[1])
          }
        }
        ctx.stroke()
        ctx.restore()
      }
    }
  }

  hitTest(camera: Camera, pose: Pose, mx: number, my: number, ringQuat: Quat = pose.quat): 0 | 1 | 2 | null {
    let best: { axis: 0 | 1 | 2; dist: number } | null = null
    if (this.mode === 'translate') {
      const p0 = this.renderer.project(camera, pose.pos)
      if (!p0) return null
      for (let i = 0; i < 3; i++) {
        const dir = this.translateAxisDir(pose, i)
        const tip = add(pose.pos, scale(dir, HANDLE_LENGTH))
        const p1 = this.renderer.project(camera, tip)
        if (!p1) continue
        const d = distanceToSegment(mx, my, p0[0], p0[1], p1[0], p1[1])
        if (d < HIT_RADIUS_PX && (!best || d < best.dist)) best = { axis: i as 0 | 1 | 2, dist: d }
      }
    } else {
      for (const i of ROTATE_AXES) {
        const [u, v] = ringBasis(i)
        const worldU = rotateVec3(ringQuat, u)
        const worldV = rotateVec3(ringQuat, v)
        let prev: [number, number] | null = null
        for (let s = 0; s <= 32; s++) {
          const a = (s / 32) * Math.PI * 2
          const p3 = add(pose.pos, add(scale(worldU, Math.cos(a) * RING_RADIUS), scale(worldV, Math.sin(a) * RING_RADIUS)))
          const p2 = this.renderer.project(camera, p3)
          if (p2 && prev) {
            const d = distanceToSegment(mx, my, prev[0], prev[1], p2[0], p2[1])
            if (d < HIT_RADIUS_PX && (!best || d < best.dist)) best = { axis: i as 0 | 1 | 2, dist: d }
          }
          prev = p2 ? [p2[0], p2[1]] : prev
        }
      }
    }
    return best ? best.axis : null
  }

  setHover(axis: 0 | 1 | 2 | null) {
    this.hoverAxis = axis
  }

  beginDrag(camera: Camera, pose: Pose, axis: 0 | 1 | 2, mx: number, my: number, ringQuat: Quat = pose.quat) {
    if (this.mode === 'translate') {
      const dir = this.translateAxisDir(pose, axis)
      const origin = this.renderer.project(camera, pose.pos)
      const ratePoint = this.renderer.project(camera, add(pose.pos, scale(dir, RATE_SAMPLE_DISTANCE)))
      if (!origin || !ratePoint) return
      const dx = ratePoint[0] - origin[0]
      const dy = ratePoint[1] - origin[1]
      const pixelLen = Math.hypot(dx, dy) || 1
      const axisDir2D: [number, number] = [dx / pixelLen, dy / pixelLen]
      const unitsPerPixel = RATE_SAMPLE_DISTANCE / pixelLen
      const grabOffsetPx = (mx - origin[0]) * axisDir2D[0] + (my - origin[1]) * axisDir2D[1]
      this.drag = {
        axis,
        mode: 'translate',
        startPos: pose.pos,
        dir,
        originScreen: [origin[0], origin[1]],
        axisDir2D,
        unitsPerPixel,
        grabOffsetPx,
      }
    } else {
      const p0 = this.renderer.project(camera, pose.pos)
      if (!p0) return
      const worldAxis = rotateVec3(ringQuat, AXES[axis])
      const viewDir = normalize(sub(camera.pose.pos, pose.pos))
      const signFlip = dot(worldAxis, viewDir) < 0 ? -1 : 1
      this.drag = {
        axis,
        mode: 'rotate',
        lastAngle: Math.atan2(my - p0[1], mx - p0[0]),
        signFlip,
      }
    }
  }

  /**
   * Mutates `pose` in place while dragging. `rotateAxis`/`rotateAngle` are
   * only set for a rotate-mode drag (the applied incremental angle, about
   * the target's local X/Y/Z) — main.ts uses that to track a tilt-sum
   * target directly from the same delta the gizmo itself just applied,
   * rather than re-deriving it from a before/after orientation comparison.
   */
  updateDrag(
    camera: Camera,
    pose: Pose,
    mx: number,
    my: number,
  ): { changed: boolean; rotateAxis: 0 | 1 | 2 | null; rotateAngle: number } {
    const none = { changed: false, rotateAxis: null, rotateAngle: 0 } as const
    const d = this.drag
    if (!d) return none
    if (
      d.mode === 'translate' &&
      d.startPos &&
      d.dir &&
      d.originScreen &&
      d.axisDir2D &&
      d.unitsPerPixel !== undefined &&
      d.grabOffsetPx !== undefined
    ) {
      // Everything here is relative to the frame captured once in
      // beginDrag — not to `pose`'s current (already-moving) position —
      // and `grabOffsetPx` is subtracted so the drag starts at zero
      // regardless of where along the handle it was grabbed. Both matter:
      // without the fixed frame, re-deriving the axis from the *current*
      // pose each call turns this into a feedback loop; without the grab
      // offset, clicking anywhere but the exact pivot point applies a
      // one-time jump equal to the grab point's distance from the pivot.
      const px = mx - d.originScreen[0]
      const py = my - d.originScreen[1]
      const alongAxisPx = px * d.axisDir2D[0] + py * d.axisDir2D[1]
      const worldOffset = (alongAxisPx - d.grabOffsetPx) * d.unitsPerPixel
      pose.pos = add(d.startPos, scale(d.dir, worldOffset))
      return { changed: true, rotateAxis: null, rotateAngle: 0 }
    } else if (d.mode === 'rotate' && d.lastAngle !== undefined && d.signFlip !== undefined) {
      const p0 = this.renderer.project(camera, pose.pos)
      if (!p0) return none
      const angle = Math.atan2(my - p0[1], mx - p0[0])
      let delta = angle - d.lastAngle
      if (delta > Math.PI) delta -= Math.PI * 2
      if (delta < -Math.PI) delta += Math.PI * 2
      d.lastAngle = angle
      const appliedAngle = delta * d.signFlip
      const localAxis = AXES[d.axis]
      const spin = quatFromAxisAngle(localAxis, appliedAngle)
      pose.quat = quatNormalize(quatMultiply(pose.quat, spin))
      return { changed: true, rotateAxis: d.axis, rotateAngle: appliedAngle }
    }
    return none
  }

  endDrag() {
    this.drag = null
  }
}

function ringBasis(axis: number): [Vec3, Vec3] {
  if (axis === 0) return [[0, 1, 0], [0, 0, 1]]
  if (axis === 1) return [[1, 0, 0], [0, 0, 1]]
  return [[1, 0, 0], [0, 1, 0]]
}

function distanceToSegment(px: number, py: number, x0: number, y0: number, x1: number, y1: number): number {
  const dx = x1 - x0
  const dy = y1 - y0
  const lenSq = dx * dx + dy * dy
  let t = lenSq < 1e-9 ? 0 : ((px - x0) * dx + (py - y0) * dy) / lenSq
  t = Math.max(0, Math.min(1, t))
  const cx = x0 + t * dx
  const cy = y0 + t * dy
  return Math.hypot(px - cx, py - cy)
}
