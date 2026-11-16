// Navigation cube at the top right of the viewport, modelled on the ViewCube of Autodesk Fusion 360.
//
// Real SVG instead of the earlier WebGL solution (textured 3D quads in an orthographic overlay scene of their
// own). The WebGL approach needed hand-written hit-test math parallel to the render projection
// (projectOrtho() + point-in-polygon), which had to stay exactly in step with the actual projection matrix on
// every change of aspect ratio/container size. Despite several fixes (aspect-dependent positioning,
// ResizeObserver instead of only window resize, cumulative instead of per-event drag threshold) this stayed
// fragile in practice - the clickable area and the visible drawing still drifted apart at some window
// sizes/layout states.
//
// SVG elements are REAL DOM nodes - the browser does the hit testing itself, exactly based on what is
// actually drawn. So they cannot drift apart any more. The positioning of the whole widget is pure CSS (fixed
// screen corner, fixed pixel size, completely independent of the 3D viewport aspect ratio) - per frame only
// the ROTATION of the faces (as SVG polygon points) is recomputed, with the same quaternion math as before,
// only projected onto a simple fixed 2D area instead of an aspect-dependent camera projection.

import type { Vec3, Quat } from './math.ts'
import { add, dot, normalize, quatConjugate, quatFromAxisAngle, rotateVec3, scale } from './math.ts'
import type { Camera } from './renderer.ts'

export interface ViewPreset {
  /* null = keep the current yaw (for top/bottom - otherwise the view "tilts" arbitrarily). */
  yaw: number | null
  pitch: number
}

interface FaceDef {
  localNormal: Vec3
  label: string
  color: string // direkt als SVG-fill
  preset: ViewPreset
  /* Correction of how the letter sits on THIS face - quatFromZTo() does not pick a "readable" local X/Y
     orientation for each face, only some orthonormal basis perpendicular to the normal, which gives a
     different apparent rotation per face. Values set by eye. Degrees, clockwise. */
  textRotationDeg?: number
}

const NEAR_TOP = (Math.PI / 2) * 0.999 // Singularitaet bei exakt 90 Grad vermeiden

// World convention as in the main viewport: Z up, "front" arbitrarily but consistently along +Y.
const FACES: FaceDef[] = [
  { localNormal: [0, 1, 0], label: 'V', color: 'rgb(70,150,90)', preset: { yaw: Math.PI / 2, pitch: 0 }, textRotationDeg: 180 },
  { localNormal: [0, -1, 0], label: 'H', color: 'rgb(60,110,75)', preset: { yaw: -Math.PI / 2, pitch: 0 }, textRotationDeg: 0 },
  { localNormal: [1, 0, 0], label: 'R', color: 'rgb(190,80,80)', preset: { yaw: 0, pitch: 0 }, textRotationDeg: -90 },
  { localNormal: [-1, 0, 0], label: 'L', color: 'rgb(150,65,65)', preset: { yaw: Math.PI, pitch: 0 }, textRotationDeg: 90 },
  { localNormal: [0, 0, 1], label: 'O', color: 'rgb(70,110,190)', preset: { yaw: null, pitch: NEAR_TOP }, textRotationDeg: 0 },
  { localNormal: [0, 0, -1], label: 'U', color: 'rgb(60,90,150)', preset: { yaw: null, pitch: -NEAR_TOP }, textRotationDeg: 180 },
]

const SVG_PX = 84 // fixed CSS pixel size of the widget - no JS resize needed
const VIEWBOX_HALF = 50 // SVG-User-Units, viewBox="-50 -50 100 100"
const CUBE_HALF = 26 // Kantenlaenge/2 des Wuerfels in denselben User-Units
const DRAG_THRESHOLD_PX = 14 // measured cumulatively from pointerdown (not per move event) - otherwise
// a slightly shaky real click is wrongly taken as a drag and the view change does not happen.

/* Quaternion that rotates local +Z onto `normal` - the standard trick to "orient a face mesh (which faces +Z
   by default) towards its outward direction", see also primitives.ts. */
function quatFromZTo(normal: Vec3): Quat {
  const z: Vec3 = [0, 0, 1]
  if (dot(z, normal) > 1 - 1e-6) return [0, 0, 0, 1]
  if (dot(z, normal) < -1 + 1e-6) return quatFromAxisAngle([1, 0, 0], Math.PI)
  const axis = normalize([z[1] * normal[2] - z[2] * normal[1], z[2] * normal[0] - z[0] * normal[2], z[0] * normal[1] - z[1] * normal[0]])
  const angle = Math.acos(Math.max(-1, Math.min(1, dot(z, normal))))
  return quatFromAxisAngle(axis, angle)
}

const SVG_NS = 'http://www.w3.org/2000/svg'

interface Face {
  def: FaceDef
  group: SVGGElement
  polygon: SVGPolygonElement
  text: SVGTextElement
}

export interface ViewCubeCallbacks {
  onViewPreset(preset: ViewPreset): void
  /* Raw pixel deltas (not pre-scaled) - the caller applies the same formula as for its own main viewport
     orbit drag (see roarm-3d-view.ts), so that the cube rotates in the same direction as the main view for
     the same drag gesture. */
  onOrbitDrag(dx: number, dy: number): void
}

export interface ViewCubeHandle {
  /* Call every frame before rendering - updates the SVG face polygons relative to the camera. */
  updatePoses(camera: Camera): void
  dispose(): void
}

export function createViewCube(container: HTMLElement, callbacks: ViewCubeCallbacks): ViewCubeHandle {
  const svg = document.createElementNS(SVG_NS, 'svg')
  svg.setAttribute('viewBox', `${-VIEWBOX_HALF} ${-VIEWBOX_HALF} ${VIEWBOX_HALF * 2} ${VIEWBOX_HALF * 2}`)
  // Fixed screen corner, fixed pixel size - pure CSS, no JS ever sets a position/size here (see the module
  // comment). pointer-events:none on the root + auto on every face polygon: a click into the (invisible) gap
  // between the faces thus falls through to the canvas below automatically (normal orbit there), without any
  // manual hit testing.
  svg.style.cssText =
    `position:absolute;top:8px;right:8px;width:${SVG_PX}px;height:${SVG_PX}px;z-index:1;` +
    'touch-action:none;user-select:none;pointer-events:none;overflow:visible;'
  container.appendChild(svg)

  const faces: Face[] = FACES.map((def, i) => {
    const group = document.createElementNS(SVG_NS, 'g')
    group.dataset.faceIndex = String(i)
    group.style.cursor = 'pointer'

    const polygon = document.createElementNS(SVG_NS, 'polygon')
    polygon.setAttribute('fill', def.color)
    polygon.setAttribute('stroke', 'rgba(0,0,0,0.35)')
    polygon.setAttribute('stroke-width', '1.5')
    polygon.setAttribute('stroke-linejoin', 'round')
    polygon.style.pointerEvents = 'auto'
    group.appendChild(polygon)

    // x/y stay fixed at 0,0 (face centre in the local, unrotated coordinate system) - positioning AND
    // distortion happen completely via the "transform" set per frame (see updatePoses()), no longer via x/y.
    const text = document.createElementNS(SVG_NS, 'text')
    text.setAttribute('x', '0')
    text.setAttribute('y', '0')
    text.setAttribute('fill', '#ffffff')
    text.setAttribute('font-size', '24')
    text.setAttribute('font-weight', '700')
    text.setAttribute('font-family', 'system-ui, sans-serif')
    text.setAttribute('text-anchor', 'middle')
    text.setAttribute('dominant-baseline', 'central')
    text.style.pointerEvents = 'none' // Klicks immer ans Polygon durchreichen, nie am Text haengenbleiben
    text.textContent = def.label
    group.appendChild(text)

    svg.appendChild(group)
    return { def, group, polygon, text }
  })

  // --- Click vs. drag: measured cumulatively from the pointerdown position (not against the last move
  // position), see the DRAG_THRESHOLD_PX comment. ---------------------------------------
  let dragStart: [number, number] | null = null
  let lastPos: [number, number] | null = null
  let dragMoved = false
  let pressedFace: FaceDef | null = null

  function faceFromEvent(e: PointerEvent): FaceDef | null {
    const target = e.target as Element | null
    const groupEl = target?.closest('[data-face-index]') as SVGGElement | null
    if (!groupEl) return null
    const idx = Number(groupEl.dataset.faceIndex)
    return faces[idx]?.def ?? null
  }

  function onPointerDown(e: PointerEvent): void {
    if (e.button !== 0) return
    pressedFace = faceFromEvent(e)
    if (!pressedFace) return
    dragStart = [e.clientX, e.clientY]
    lastPos = dragStart
    dragMoved = false
    svg.setPointerCapture(e.pointerId)
  }

  function onPointerMove(e: PointerEvent): void {
    if (!dragStart || !lastPos) return
    const dx = e.clientX - lastPos[0]
    const dy = e.clientY - lastPos[1]
    const totalDx = e.clientX - dragStart[0]
    const totalDy = e.clientY - dragStart[1]
    if (Math.hypot(totalDx, totalDy) > DRAG_THRESHOLD_PX) dragMoved = true
    lastPos = [e.clientX, e.clientY]
    if (dragMoved) callbacks.onOrbitDrag(dx, dy)
  }

  function endDrag(): void {
    if (!dragMoved && pressedFace) callbacks.onViewPreset(pressedFace.preset)
    dragStart = null
    lastPos = null
    dragMoved = false
    pressedFace = null
  }

  svg.addEventListener('pointerdown', onPointerDown)
  svg.addEventListener('pointermove', onPointerMove)
  svg.addEventListener('pointerup', endDrag)
  svg.addEventListener('pointercancel', endDrag)

  // --- Projection: cube-local corners (via quatFromZTo() as before) rotated with the world-fixed conjugate
  // camera rotation, then simply projected orthographically to 2D (drop Z, Y mirrored - SVG grows downwards).
  // No aspect ratio dependency any more: the widget has a fixed pixel size, so the projection needs no
  // reference to the container. -----
  function faceWorldCorners(def: FaceDef, cubeQuat: Quat): Vec3[] {
    const localQuat = quatFromZTo(def.localNormal)
    const localCenter = scale(def.localNormal, CUBE_HALF)
    const localCorners: Vec3[] = [
      [-CUBE_HALF, -CUBE_HALF, 0],
      [CUBE_HALF, -CUBE_HALF, 0],
      [CUBE_HALF, CUBE_HALF, 0],
      [-CUBE_HALF, CUBE_HALF, 0],
    ]
    return localCorners.map((c) => rotateVec3(cubeQuat, add(localCenter, rotateVec3(localQuat, c))))
  }

  function project(p: Vec3): [number, number] {
    return [p[0], -p[1]]
  }

  function updatePoses(camera: Camera): void {
    // World-fixed orientation (conjugate of the main camera rotation): the cube thus shows exactly what a
    // world-fixed cube would show from the current main camera viewing direction, although this widget itself
    // never moves along (fixed screen corner).
    const cubeQuat = quatConjugate(camera.pose.quat)
    for (const face of faces) {
      const worldNormal = rotateVec3(cubeQuat, face.def.localNormal)
      const frontFacing = worldNormal[2] > 0.05
      face.group.style.display = frontFacing ? '' : 'none'
      if (!frontFacing) continue

      const corners = faceWorldCorners(face.def, cubeQuat)
      const projected = corners.map(project)
      face.polygon.setAttribute('points', projected.map(([x, y]) => `${x.toFixed(1)},${y.toFixed(1)}`).join(' '))
      const cx = projected.reduce((s, p) => s + p[0], 0) / 4
      const cy = projected.reduce((s, p) => s + p[1], 0) / 4

      // Real perspective (more precisely: affine - the orthographic projection of a planar face is always a
      // parallelogram, straight lines stay straight) distortion of the letter instead of a pure rotation:
      // derive the basis vectors "one local unit in X" (ex) and "one local unit in Y" (ey) of the face IN
      // SCREEN COORDINATES from the corners already computed - for a face seen at an angle, ex/ey differ in
      // length and are not perpendicular, which is exactly what makes the letter look squashed/sheared
      // instead of rigidly rotated (the earlier WebGL version got this automatically from the 3D mesh
      // transform; here it has to be rebuilt explicitly as an SVG matrix, since everything is projected flat
      // to 2D). The letter itself sits at the local origin (x=0,y=0, see its creation above) - the matrix
      // does position AND distortion. quatFromZTo() ALWAYS returns a right-handed local basis (right x up =
      // normal, for every face - checked by calculation). But project() mirrors Y (SVG grows downwards) - a
      // pure mirroring reverses the handedness, EQUALLY for EVERY face turned towards the camera (not just
      // some). Without a correction, letters therefore appear consistently mirrored (visible e.g. as an "R"
      // that looks like a backwards "Я"). So ey is negated here once, uniformly - this reverses the
      // handedness of the TEXT basis again (the polygon corners themselves stay unchanged, a square is
      // symmetric and does not show a mirroring anyway).
      const exX = (projected[1][0] - projected[0][0]) / (2 * CUBE_HALF)
      const exY = (projected[1][1] - projected[0][1]) / (2 * CUBE_HALF)
      const eyX = -(projected[3][0] - projected[0][0]) / (2 * CUBE_HALF)
      const eyY = -(projected[3][1] - projected[0][1]) / (2 * CUBE_HALF)

      // Fixed basis correction angle per face (textRotationDeg) - quatFromZTo() does not pick a "readable"
      // local axis basis for each face, only some basis perpendicular to the normal, which gives a different
      // but CONSTANT initial twist per face. Applied as a rotation of the basis vectors themselves (not as a
      // rotation of the result), so that it combines cleanly with the actual distortion instead of
      // overwriting it.
      const theta = ((face.def.textRotationDeg ?? 0) * Math.PI) / 180
      const cosT = Math.cos(theta)
      const sinT = Math.sin(theta)
      const rotExX = cosT * exX + sinT * eyX
      const rotExY = cosT * exY + sinT * eyY
      const rotEyX = -sinT * exX + cosT * eyX
      const rotEyY = -sinT * exY + cosT * eyY

      face.text.setAttribute(
        'transform',
        `matrix(${rotExX.toFixed(3)} ${rotExY.toFixed(3)} ${rotEyX.toFixed(3)} ${rotEyY.toFixed(3)} ${cx.toFixed(1)} ${cy.toFixed(1)})`,
      )
    }
  }

  return {
    updatePoses,
    dispose(): void {
      svg.removeEventListener('pointerdown', onPointerDown)
      svg.removeEventListener('pointermove', onPointerMove)
      svg.removeEventListener('pointerup', endDrag)
      svg.removeEventListener('pointercancel', endDrag)
      container.removeChild(svg)
    },
  }
}
