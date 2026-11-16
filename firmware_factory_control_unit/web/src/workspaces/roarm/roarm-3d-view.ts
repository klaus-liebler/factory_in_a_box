// 3D model of the RoArm-M3 based on the real Waveshare URDF meshes (see web/tools/roarm-meshes/;
// robot-data.ts is generated - origin and decimation are documented there) instead of a model built
// programmatically from primitives. IK, gizmo (Canvas2D overlay above the WebGL canvas) and the
// orbit/pan/dolly camera are a direct port of the prototype the user had already validated,
// C:\repos\playground\robot_ui_programmatic (kinematics/IK) and C:\repos\playground\robot_ui (mesh rendering,
// vacuum suction cup end effector - the real hardware carries a simple vacuum suction cup instead of the
// mechanical gripper, see makeVacuumCup() in primitives.ts).
//
// Unlike the old primitive-based model, the IK now runs on the client (roarm3d/ik.ts, closed-form,
// non-iterative 3R planar solution): the gizmo drags a target (position + tilt sum or roll), every frame
// solveIK() solves against it afresh and returns the full joint angles of the chain. The caller
// (roarm-teach-app.ts) only gets finished angles via onJointAnglesPreview() (rad, directly compatible with
// the convention of roarm-kinematics.ts - same link lengths, same Z-up axis, only m instead of mm) - no scene
// unit conversion needed any more (the old SCENE_UNITS_PER_MM/sceneDeltaToKinematicsMm approximation with its
// documented 10-15% error is gone completely). Actually executing the movement stays the job of the backend
// (firmware or mock) - the gizmo drag is still only an input method, not the authority over the exact
// position.
import { JOINT_COUNT } from './roarm-kinematics.ts'
import type { Vec3, Pose, Quat } from './3d/math.ts'
import { add, clamp, quatFromAxisAngle, quatLookAt, quatMultiply, rotateVec3, sub, transformPoint } from './3d/math.ts'
import { CHAIN, MESHES } from './3d/robot-data.ts'
import { defaultAngles, forwardKinematics, type JointAngles } from './3d/kinematics.ts'
import { solveIK, tiltSum, limitProximity } from './3d/ik.ts'
import { Renderer as Overlay2D, makeGrid, type Camera as AppCamera } from './3d/renderer.ts'
import { OglRenderer, type PartHandle } from './3d/oglRenderer.ts'
import { Gizmo } from './3d/gizmo.ts'
import { makeVacuumCup, VACUUM_CUP_TIP_OFFSET_M, makeBox, TRANSPORTED_CUBE_SIZE_M } from './3d/primitives.ts'
import { createViewCube, type ViewCubeHandle } from './3d/view-cube.ts'
import { createToggleSwitch } from './3d/toggle-switch.ts'

// While dragging freely, the camera may stand almost (but not exactly) vertical - at exactly 90 degrees the
// cross product of quatLookAt() with the world up vector becomes singular. The same limit as the top/bottom
// views of the ViewCube (view-cube.ts), so that a click on "top"/"bottom" reaches a pitch that could also be
// reached by mouse drag (and the following drag does not suddenly hang at the clamp).
const MAX_ORBIT_PITCH = (Math.PI / 2) * 0.999

// See releasedTarget in createRoArm3DView(). The tolerance covers the centiDeg rounding of the wire format;
// the timeout is the emergency exit in case the backend never reaches the target (e.g. clamped differently) -
// then the reported actual pose takes over as before.
const RELEASE_HOLD_TOLERANCE_RAD = (1 * Math.PI) / 180
const RELEASE_HOLD_TIMEOUT_MS = 5000

// Local offset from the origin of link5 (mounting point) to the suction cup tip, along link5's own Z axis -
// the gizmo/IK target must sit at the tip (the point that touches a workpiece), not at the shaft output.
// solveIK() still expects the origin position of link5, hence the conversion back and forth in
// syncTargetToRobot() and before every solveIK() call during a drag.
const TIP_OFFSET_LOCAL: Vec3 = [0, 0, VACUUM_CUP_TIP_OFFSET_M]

function tipTargetToLink5Origin(tipPos: Vec3, quat: Pose['quat']): Vec3 {
  return sub(tipPos, rotateVec3(quat, TIP_OFFSET_LOCAL))
}

// The suction cup "carries" a 2.5cm wooden cube (see primitives.ts) - mainly so that the effect of "roll" is
// visible at all (the cup itself is rotationally symmetric about its own axis). The cube centre sits right at
// the cup tip plus half the edge length (touches the tip instead of penetrating it).
const CARGO_CENTER_OFFSET_LOCAL: Vec3 = [0, 0, VACUUM_CUP_TIP_OFFSET_M + TRANSPORTED_CUBE_SIZE_M / 2]

// Chain segment names <-> wire format index (JOINT_NAMES/JointIndex of roarm-kinematics.ts): link1=Base(0),
// link2=Shoulder(1), link3=Elbow(2), link4=Wrist(3), link5=Roll(4). Gripper(5) has no entry in CHAIN (the
// vacuum cup has no joint axis of its own) and is neither shown nor changed by this view - the caller keeps
// its last known value itself.
const CHAIN_LINK_NAMES = ['link1', 'link2', 'link3', 'link4', 'link5'] as const

function armAnglesToChainAngles(armAnglesRad: readonly number[]): JointAngles {
  const angles: JointAngles = {}
  for (let i = 0; i < CHAIN_LINK_NAMES.length; i++) angles[CHAIN_LINK_NAMES[i]] = armAnglesRad[i] ?? 0
  return angles
}

function chainAnglesToArmAngles(angles: JointAngles): number[] {
  return CHAIN_LINK_NAMES.map((name) => angles[name] ?? 0)
}

export interface RoArm3DCallbacks {
  onDragStart?(): void
  /* Called continuously during a gizmo drag with the 5 IK-solved joint angles (rad, Base..Roll, see
     CHAIN_LINK_NAMES) - the caller merges them into its own 6-element jointAnglesCentiDeg store (the gripper
     channel stays untouched) and sends it to the backend. */
  onJointAnglesPreview?(armAnglesRad: readonly number[]): void
  onDragEnd?(): void
}

export interface RoArm3DHandles {
  /* Full 6-element angle array (rad) as reported by the backend - index 5 (gripper) is ignored (the vacuum
     cup has no animation of its own). */
  setJointAnglesRad(anglesRad: readonly number[]): void
  /* Shows/hides the ghost overlay (last recorded pose); null hides it. */
  setGhostAnglesRad(anglesRad: readonly number[] | null): void
  dispose(): void
}

// --- Material: black powder-coated aluminium (arm meshes) - dielectric/matte, no metallic shine. The vacuum
// cup on the other hand is matte rubber (a different physical material). The ghost deliberately stays neutral
// (visibility matters more than material realism, see GHOST_MATERIAL).
const ARM_COLOR: [number, number, number] = [24, 24, 26]
const ARM_MATERIAL = { metallic: 0.05, roughness: 0.58 }
const CUP_COLOR: [number, number, number] = [35, 35, 38]
const CUP_MATERIAL = { metallic: 0, roughness: 0.8 }
// Blue painted wood (the transported workpiece) - a material of its own, not metal, a bit less rough than
// rubber (painted surface).
const CARGO_COLOR: [number, number, number] = [45, 95, 175]
const CARGO_MATERIAL = { metallic: 0, roughness: 0.5 }
const GHOST_COLOR: [number, number, number] = [51, 148, 255]
const GHOST_OPACITY = 0.25
const GHOST_MATERIAL = { metallic: 0, roughness: 0.6 }

// The floor grid turns yellow, then red towards the motion limit - grey = far from every joint limit, red =
// at the limit or the dragged target is currently unreachable (see limitProximity() and the reachable flag of
// solveIK() in ik.ts). Feedback only, not a hard limit itself - solveIK() already keeps the pose from moving
// beyond the limit.
const GRID_COLOR_NORMAL: [number, number, number] = [90, 92, 98]
const GRID_COLOR_WARN: [number, number, number] = [235, 190, 40]
const GRID_COLOR_LIMIT: [number, number, number] = [220, 60, 50]

function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t
}

function gridWarningColor(proximity: number): [number, number, number] {
  const p = Math.max(0, Math.min(1, proximity))
  const [from, to, t] = p < 0.5 ? [GRID_COLOR_NORMAL, GRID_COLOR_WARN, p * 2] : [GRID_COLOR_WARN, GRID_COLOR_LIMIT, (p - 0.5) * 2]
  return [Math.round(lerp(from[0], to[0], t)), Math.round(lerp(from[1], to[1], t)), Math.round(lerp(from[2], to[2], t))]
}

interface PartSet {
  parts: { name: string; handle: PartHandle }[]
  cup: PartHandle
  cargo: PartHandle
}

function frameCameraToChain(): { center: Vec3; distance: number } {
  const { originPoses } = forwardKinematics(defaultAngles())
  let minZ = Infinity
  let maxZ = -Infinity
  let maxR = 0
  for (const p of originPoses) {
    minZ = Math.min(minZ, p.pos[2])
    maxZ = Math.max(maxZ, p.pos[2])
    maxR = Math.max(maxR, Math.hypot(p.pos[0], p.pos[1]))
  }
  const center: Vec3 = [0, 0, (minZ + maxZ) / 2]
  const distance = Math.max(maxZ - minZ, maxR * 2, 0.3) * 1.9
  return { center, distance }
}

export function createRoArm3DView(container: HTMLElement, callbacks: RoArm3DCallbacks = {}): RoArm3DHandles {
  container.style.position = 'relative'
  container.style.overflow = 'hidden'

  const canvasGl = document.createElement('canvas')
  canvasGl.style.position = 'absolute'
  canvasGl.style.inset = '0'
  canvasGl.style.pointerEvents = 'none'
  const canvas2d = document.createElement('canvas')
  canvas2d.style.position = 'absolute'
  canvas2d.style.inset = '0'
  canvas2d.style.touchAction = 'none'
  container.appendChild(canvasGl)
  container.appendChild(canvas2d)

  const glRenderer = new OglRenderer(canvasGl)
  const overlay = new Overlay2D(canvas2d)
  const gizmo = new Gizmo(overlay)
  const vacuumCupMesh = makeVacuumCup()
  const cargoCubeMesh = makeBox(TRANSPORTED_CUBE_SIZE_M, TRANSPORTED_CUBE_SIZE_M, TRANSPORTED_CUBE_SIZE_M)

  // Floor grid as real, depth-tested WebGL geometry (not on the 2D gizmo overlay) - otherwise it always lies
  // IN FRONT of the robot instead of being hidden by it when the robot stands in front of it from the
  // camera's view (confusing for a grid meant to show the floor). The gizmo deliberately stays on the 2D
  // overlay - it must always stay grabbable, even "behind" an arm part, the usual convention in 3D tools.
  const gridHandle = glRenderer.createLines(makeGrid(1.0, 20), GRID_COLOR_NORMAL, 0.9)

  const liveParts = buildLinkedPartSet(glRenderer, vacuumCupMesh, cargoCubeMesh, 1)
  const ghostParts = buildLinkedPartSet(glRenderer, vacuumCupMesh, cargoCubeMesh, GHOST_OPACITY, GHOST_COLOR)
  setPartSetVisible(ghostParts, false)

  let angles: JointAngles = defaultAngles()
  let target: Pose = { pos: [0, 0, 0], quat: [0, 0, 0, 1] }
  let targetTiltSum = 0
  let suppressExternalResync = false // true while a gizmo drag is in progress
  // After release: the pose last sent via onJointAnglesPreview(), until the backend feedback has caught up
  // with it (see setJointAnglesRad()) - otherwise the model jumped back on release to the intermediate pose
  // (following with limited speed) and only then moved to the dragged target again.
  let releasedTarget: number[] | null = null
  let releasedTargetDeadline = 0
  let dragUnreachable = false // letzter solveIK()-Aufruf waehrend eines Drags: reachable=false
  // During a rotate drag frozen to the tip position at drag start (see onPointerDown/onPointerMove) - the
  // pivot MUST be the suction cup tip, not "wherever the last frame ended up", see the comment there on the
  // actual bug.
  let rotateDragAnchorPos: Vec3 | null = null

  function syncTargetToRobot(): void {
    const { poses } = forwardKinematics(angles)
    const wrist = poses[poses.length - 1] // link5 origin (mounting point, not the cup tip)
    target = { pos: transformPoint(wrist, TIP_OFFSET_LOCAL), quat: wrist.quat }
    targetTiltSum = tiltSum(angles)
  }

  // Orientation for the tilt ring (red) of the gizmo: target.quat MINUS the current roll rotation
  // (angles.link5), removed by undoing the Z rotation applied last at the end of the chain. Roll is
  // kinematically the LAST link of the chain - it does not change the axis tilt actually tilts about, so it
  // must not rotate the tilt ring visually (without this correction the red ring visibly rotated along when
  // blue/roll was dragged, although its meaning stayed the same). The roll ring itself (blue) is not affected
  // by this correction - a circle about its own normal axis looks the same at every roll angle anyway.
  function tiltRingQuat(): Quat {
    return quatMultiply(target.quat, quatFromAxisAngle([0, 0, 1], -(angles.link5 ?? 0)))
  }
  syncTargetToRobot()

  // --- Orbit/Pan/Dolly-Kamera (Maus: links=Gizmo/Orbit, rechts=Pan, Mitte/Rad=Zoom) -------------
  const framing = frameCameraToChain()
  let orbitCenter = framing.center
  let orbitYaw = Math.PI * 0.28
  let orbitPitch = Math.PI * 0.22
  let orbitDistance = framing.distance

  function cameraPose(): Pose {
    const cp = Math.cos(orbitPitch)
    const eye: Vec3 = add(orbitCenter, [
      Math.cos(orbitYaw) * cp * orbitDistance,
      Math.sin(orbitYaw) * cp * orbitDistance,
      Math.sin(orbitPitch) * orbitDistance,
    ])
    return { pos: eye, quat: quatLookAt(eye, orbitCenter, [0, 0, 1]) }
  }
  function camera(): AppCamera {
    return { pose: cameraPose(), fovY: (50 * Math.PI) / 180 }
  }

  // --- Gizmo switches (two small segmented toggles in the viewport corner) ----------------------
  // A shared flex container instead of two elements positioned individually with fixed top values - their
  // actual height (button padding/line height) could not be predicted exactly enough by eye, and two fixed
  // top values led to overlap depending on rendering. The flex container with gap takes care of the spacing
  // automatically, and both switches align at the same left edge via align-items:flex-start (start instead of
  // stretch, otherwise the narrower "Welt/Lokal" switch would be stretched to the width of the wider one).
  const toggleStack = document.createElement('div')
  toggleStack.style.cssText = 'position:absolute;top:8px;left:8px;z-index:1;display:flex;flex-direction:column;align-items:flex-start;gap:6px;'
  container.appendChild(toggleStack)

  // "Bewegen" instead of "Verschieben" - the latter burst the width of the switch (see the fixed button width
  // of createToggleSwitch(), which keeps both switches the same size).
  const modeToggle = createToggleSwitch(['Bewegen', 'Drehen'], gizmo.mode === 'translate' ? 0 : 1, (i) => {
    gizmo.mode = i === 0 ? 'translate' : 'rotate'
  })
  toggleStack.appendChild(modeToggle.el)

  // Orientation of the translate arrows: world axes (default) or the vacuum cup's own coordinate system - see
  // the `space` field of gizmo.ts. Only affects translate mode (rotate always stays tied to tilt/roll of the
  // tool, see the comment there).
  const spaceToggle = createToggleSwitch(['Welt', 'Lokal'], gizmo.space === 'world' ? 0 : 1, (i) => {
    gizmo.space = i === 0 ? 'world' : 'local'
  })
  toggleStack.appendChild(spaceToggle.el)

  // --- ViewCube (top right) - an SVG widget of its own (see view-cube.ts), handles positioning AND
  // click/drag detection completely by itself (real DOM hit tests instead of hand-written
  // projection/point-in-polygon math). Calls onViewPreset on a click on a face, onOrbitDrag on dragging - the
  // same orbit formula as the main viewport itself (see its 'orbit' branch below), so that the cube rotates
  // in the same direction for the same drag gesture.
  function applyViewPreset(preset: { yaw: number | null; pitch: number }): void {
    if (preset.yaw !== null) orbitYaw = preset.yaw
    orbitPitch = clamp(preset.pitch, -MAX_ORBIT_PITCH, MAX_ORBIT_PITCH)
  }
  const viewCube: ViewCubeHandle = createViewCube(container, {
    onViewPreset: (preset) => applyViewPreset(preset),
    onOrbitDrag: (dx, dy) => {
      orbitYaw -= dx * 0.008
      orbitPitch = clamp(orbitPitch + dy * 0.008, -MAX_ORBIT_PITCH, MAX_ORBIT_PITCH)
    },
  })

  // --- Pointer handling --------------------------------------------------------------------------
  // No 'cube' branch any more: the ViewCube SVG (see above) sits as a DOM element of its own above this
  // canvas and already catches clicks/drags on its faces itself (real DOM hit testing) - a click next to it
  // (transparent gap in the SVG) falls through to this canvas automatically and is treated here as a normal
  // orbit, without roarm-3d-view.ts having to query the cube itself.
  let dragMode: 'orbit' | 'gizmo' | 'pan' | 'dolly' | null = null
  let lastMouse: [number, number] = [0, 0]

  canvas2d.addEventListener('contextmenu', (e) => e.preventDefault())

  function onPointerDown(e: PointerEvent): void {
    const rect = canvas2d.getBoundingClientRect()
    const mx = e.clientX - rect.left
    const my = e.clientY - rect.top

    if (e.button === 2) {
      dragMode = 'pan'
    } else if (e.button === 1) {
      dragMode = 'dolly'
    } else {
      const axis = gizmo.hitTest(camera(), target, mx, my, tiltRingQuat())
      if (axis !== null) {
        dragMode = 'gizmo'
        gizmo.beginDrag(camera(), target, axis, mx, my, tiltRingQuat())
        if (gizmo.mode === 'rotate') rotateDragAnchorPos = target.pos
        suppressExternalResync = true
        callbacks.onDragStart?.()
      } else {
        dragMode = 'orbit'
      }
    }
    lastMouse = [mx, my]
    canvas2d.setPointerCapture(e.pointerId)
  }

  function onPointerMove(e: PointerEvent): void {
    const rect = canvas2d.getBoundingClientRect()
    const mx = e.clientX - rect.left
    const my = e.clientY - rect.top
    const dx = mx - lastMouse[0]
    const dy = my - lastMouse[1]

    if (dragMode === 'gizmo') {
      const result = gizmo.updateDrag(camera(), target, mx, my)
      // Only axis 0 (red, tilt) and 2 (blue, roll) can be dragged at all (see ROTATE_AXES in gizmo.ts) -
      // there is no ring for axis 1 (green) any more, the chain has no independent second tilt axis.
      if (result.rotateAxis === 0) {
        targetTiltSum += result.rotateAngle
      } else if (result.rotateAxis === 2) {
        angles.link5 = wrapPi((angles.link5 ?? 0) + result.rotateAngle)
      }
      if (result.changed) {
        // During a rotate drag ALWAYS aim at the tip position frozen at drag start (rotateDragAnchorPos), NOT
        // the latest target.pos: solveIK() can reach the orientation freely accumulated by the gizmo
        // (target.quat) only approximately (the chain has only 1 real tilt degree of freedom plus roll, no
        // free 3D rotation) - the small deviation arising anew EVERY frame was previously taken over as the
        // new starting point for the NEXT frame and added up over a longer drag to several centimetres of
        // offset (measured: about 7mm/frame, already 3.6cm after 20 frames) - visible as a pivot that moved
        // step by step away from the cup towards the wrist. With the fixed anchor every frame stays
        // independently close to the REALLY wanted pivot, errors can no longer add up. For translate,
        // target.pos (the deliberately moved target) stays authoritative unchanged.
        const tipAnchor = rotateDragAnchorPos ?? target.pos
        const link5OriginTarget = tipTargetToLink5Origin(tipAnchor, target.quat)
        const solved = solveIK(angles, link5OriginTarget, targetTiltSum)
        angles = solved.angles
        dragUnreachable = !solved.reachable
        const trueWrist = forwardKinematics(angles).poses[CHAIN.length - 1]
        target = { pos: transformPoint(trueWrist, TIP_OFFSET_LOCAL), quat: trueWrist.quat }
        callbacks.onJointAnglesPreview?.(chainAnglesToArmAngles(angles))
      }
    } else if (dragMode === 'orbit') {
      orbitYaw -= dx * 0.008
      orbitPitch = clamp(orbitPitch + dy * 0.008, -MAX_ORBIT_PITCH, MAX_ORBIT_PITCH)
    } else if (dragMode === 'pan') {
      const pose = cameraPose()
      const right = rotateVec3(pose.quat, [1, 0, 0])
      const up = rotateVec3(pose.quat, [0, 1, 0])
      const worldPerPixel = (2 * orbitDistance * Math.tan(camera().fovY / 2)) / overlay.height
      orbitCenter = add(orbitCenter, [
        (-right[0] * dx + up[0] * dy) * worldPerPixel,
        (-right[1] * dx + up[1] * dy) * worldPerPixel,
        (-right[2] * dx + up[2] * dy) * worldPerPixel,
      ])
    } else if (dragMode === 'dolly') {
      orbitDistance = clamp(orbitDistance * (1 + dy * 0.005), 0.15, 4)
    } else {
      const axis = gizmo.hitTest(camera(), target, mx, my, tiltRingQuat())
      gizmo.setHover(axis)
    }
    lastMouse = [mx, my]
  }

  function endDrag(): void {
    const wasGizmo = dragMode === 'gizmo'
    dragMode = null
    if (wasGizmo) {
      gizmo.endDrag()
      rotateDragAnchorPos = null
      suppressExternalResync = false
      dragUnreachable = false
      releasedTarget = chainAnglesToArmAngles(angles)
      releasedTargetDeadline = performance.now() + RELEASE_HOLD_TIMEOUT_MS
      callbacks.onDragEnd?.()
    }
  }
  canvas2d.addEventListener('pointerdown', onPointerDown)
  canvas2d.addEventListener('pointermove', onPointerMove)
  canvas2d.addEventListener('pointerup', endDrag)
  canvas2d.addEventListener('pointercancel', endDrag)
  canvas2d.addEventListener(
    'wheel',
    (e) => {
      e.preventDefault()
      orbitDistance = clamp(orbitDistance * (1 + e.deltaY * 0.001), 0.15, 4)
    },
    { passive: false },
  )

  function wrapPi(a: number): number {
    return (((a + Math.PI) % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI) - Math.PI
  }

  // --- Render-Loop ---------------------------------------------------------------------------
  let disposed = false
  function draw(): void {
    if (disposed) return
    requestAnimationFrame(draw)
    const cam = camera()

    posePartSet(liveParts, angles)
    // Floor grid as a warning when approaching the motion limit: how close is the current pose to any joint
    // limit (limitProximity()), AND was the last requested gizmo target unreachable (dragUnreachable, see
    // onPointerMove) - the higher, the redder.
    glRenderer.setColor(gridHandle, gridWarningColor(Math.max(limitProximity(angles), dragUnreachable ? 1 : 0)))
    viewCube.updatePoses(cam) // SVG widget - only updates polygon points, no WebGL render pass needed any more
    glRenderer.render(cam)

    overlay.clear()
    gizmo.render(cam, target, tiltRingQuat())
  }
  requestAnimationFrame(draw)

  function resize(): void {
    const width = container.clientWidth || 1
    const height = container.clientHeight || 1
    const dpr = window.devicePixelRatio || 1
    glRenderer.resize(width, height, dpr)
    overlay.resize(width, height, dpr)
  }
  resize()

  // ResizeObserver instead of (only) window 'resize' - the container can also change its size WITHOUT a
  // browser window resize (sidebar panel expanded/collapsed, mission list grows/shrinks, a scrollbar
  // appears/disappears elsewhere on the page and shifts the available width) - without observing this, the
  // WebGL canvas resolution AND the orthographic overlay camera (used by the ViewCube hit testing, see
  // projectOrtho() in view-cube.ts) stayed at the state of the last REAL window resize, while
  // `canvas2d.getBoundingClientRect()` (for the mouse coordinates in onPointerDown/onPointerMove) always
  // returns the CURRENT value - exactly the drift between visible cube position and clickable area that only
  // showed at certain window sizes/states (depending on whether such a layout-driven size change had happened
  // since the last real window resize).
  const resizeObserver = new ResizeObserver(() => resize())
  resizeObserver.observe(container)

  return {
    setJointAnglesRad(anglesRad: readonly number[]): void {
      // During a gizmo drag, PoseFeedback updates keep coming in from the backend (the real/simulated
      // movement lags behind the drag, physically limited) - if they were taken over here unchecked, they
      // would overwrite the IK preview just solved locally with the old pose (not arrived yet) every frame,
      // and the mesh would not seem to follow the gizmo at all. So during the drag the local preview has
      // priority; as soon as suppressExternalResync (see onPointerDown/endDrag) is false again, the backend's
      // actual pose seamlessly takes the lead again.
      if (suppressExternalResync) return
      if (anglesRad.length < JOINT_COUNT) return
      if (releasedTarget) {
        const target = releasedTarget
        const caughtUp = target.every((t, i) => Math.abs(anglesRad[i] - t) <= RELEASE_HOLD_TOLERANCE_RAD)
        if (!caughtUp && performance.now() < releasedTargetDeadline) return
        releasedTarget = null
      }
      angles = armAnglesToChainAngles(anglesRad)
      posePartSet(liveParts, angles)
      syncTargetToRobot()
    },
    setGhostAnglesRad(anglesRad: readonly number[] | null): void {
      if (!anglesRad) {
        setPartSetVisible(ghostParts, false)
        return
      }
      posePartSet(ghostParts, armAnglesToChainAngles(anglesRad))
      setPartSetVisible(ghostParts, true)
    },
    dispose(): void {
      disposed = true
      resizeObserver.disconnect()
      canvas2d.removeEventListener('pointerdown', onPointerDown)
      canvas2d.removeEventListener('pointermove', onPointerMove)
      canvas2d.removeEventListener('pointerup', endDrag)
      canvas2d.removeEventListener('pointercancel', endDrag)
      viewCube.dispose()
      container.removeChild(toggleStack)
      container.removeChild(canvasGl)
      container.removeChild(canvas2d)
    },
  }

  // --- local helpers (closures over glRenderer/vacuumCupMesh/cargoCubeMesh) ----------------------
  function buildLinkedPartSet(
    gl: OglRenderer,
    cupMesh: ReturnType<typeof makeVacuumCup>,
    cargoMesh: ReturnType<typeof makeBox>,
    opacity: number,
    tint?: [number, number, number],
  ): PartSet {
    const isGhost = tint !== undefined
    const parts = CHAIN.filter((seg) => seg.mesh).map((seg) => ({
      name: seg.name,
      handle: gl.createPart(MESHES[seg.mesh!], tint ?? ARM_COLOR, opacity, isGhost ? GHOST_MATERIAL : ARM_MATERIAL),
    }))
    const cup = gl.createPart(cupMesh, tint ?? CUP_COLOR, opacity, isGhost ? GHOST_MATERIAL : CUP_MATERIAL)
    const cargo = gl.createPart(cargoMesh, tint ?? CARGO_COLOR, opacity, isGhost ? GHOST_MATERIAL : CARGO_MATERIAL)
    return { parts, cup, cargo }
  }

  function posePartSet(set: PartSet, forAngles: JointAngles): void {
    const { poses } = forwardKinematics(forAngles)
    for (let i = 0; i < CHAIN.length; i++) {
      const part = set.parts.find((p) => p.name === CHAIN[i].name)
      if (part) glRenderer.setPose(part.handle, poses[i])
    }
    const wrist = poses[poses.length - 1]
    glRenderer.setPose(set.cup, wrist)
    glRenderer.setPose(set.cargo, { pos: transformPoint(wrist, CARGO_CENTER_OFFSET_LOCAL), quat: wrist.quat })
  }

  function setPartSetVisible(set: PartSet, visible: boolean): void {
    for (const p of set.parts) glRenderer.setVisible(p.handle, visible)
    glRenderer.setVisible(set.cup, visible)
    glRenderer.setVisible(set.cargo, visible)
  }
}
