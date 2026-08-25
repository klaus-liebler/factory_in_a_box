// 3D-Modell des RoArm-M3 auf Basis der echten Waveshare-URDF-Meshes (s. web/tools/roarm-meshes/,
// robot-data.ts ist generiert -- Herkunft/Dekimierung dort dokumentiert) statt eines programmatisch
// aus Primitiven aufgebauten Modells. IK, Gizmo (Canvas2D-Overlay ueber dem WebGL-Canvas) und die
// Orbit/Pan/Dolly-Kamera sind ein direkter Port aus dem vom Nutzer bereits validierten Prototyp
// C:\repos\playground\robot_ui_programmatic (Kinematik/IK) bzw. C:\repos\playground\robot_ui
// (Mesh-Rendering, Vakuum-Sauggreifer-Endeffektor -- die reale Hardware traegt einen einfachen
// Vakuum-Sauger statt des mechanischen Zangen-Greifers, s. primitives.ts' makeVacuumCup()).
//
// Anders als das alte, primitivbasierte Modell laeuft die IK jetzt clientseitig (roarm3d/ik.ts,
// geschlossene, nicht-iterative 3R-Planar-Loesung): der Gizmo zieht ein Ziel (Position + Tilt-Summe
// bzw. Roll), jeder Frame loest solveIK() frisch dagegen und liefert die vollen Gelenkwinkel der
// Kette. Der Aufrufer (roarm-teach-app.ts) bekommt ueber onJointAnglesPreview() nur noch fertige
// Winkel (rad, direkt kompatibel mit roarm-kinematics.ts' Konvention -- selbe Linklaengen, selbe
// Z-up-Achse, nur m statt mm) -- keine Szeneneinheiten-Umrechnung mehr noetig (die alte
// SCENE_UNITS_PER_MM/sceneDeltaToKinematicsMm-Naeherung mit dokumentierter 10-15%-Abweichung faellt
// damit komplett weg). Die tatsaechliche Bewegungsausfuehrung bleibt Sache des Backends (Firmware
// bzw. Mock) -- der Gizmo-Drag ist weiterhin nur Eingabemethode, nicht Autoritaet ueber die exakte
// Position.
import { JOINT_COUNT } from './roarm-kinematics.js'
import type { Vec3, Pose, Quat } from './roarm3d/math.js'
import { add, clamp, quatFromAxisAngle, quatLookAt, quatMultiply, rotateVec3, sub, transformPoint } from './roarm3d/math.js'
import { CHAIN, MESHES } from './roarm3d/robot-data.js'
import { defaultAngles, forwardKinematics, type JointAngles } from './roarm3d/kinematics.js'
import { solveIK, tiltSum, limitProximity } from './roarm3d/ik.js'
import { Renderer as Overlay2D, makeGrid, type Camera as AppCamera } from './roarm3d/renderer.js'
import { OglRenderer, type PartHandle } from './roarm3d/oglRenderer.js'
import { Gizmo } from './roarm3d/gizmo.js'
import { makeVacuumCup, VACUUM_CUP_TIP_OFFSET_M, makeBox, TRANSPORTED_CUBE_SIZE_M } from './roarm3d/primitives.js'
import { createViewCube, type ViewCubeHandle } from './roarm3d/view-cube.js'
import { createToggleSwitch } from './roarm3d/toggle-switch.js'

// Kamera darf beim freien Ziehen fast (aber nicht exakt) senkrecht stehen -- bei exakt 90 Grad
// wird quatLookAt()s Kreuzprodukt mit dem Welt-Up-Vektor singulaer. Derselbe Grenzwert wie die
// Oben/Unten-Ansichten des ViewCube (view-cube.ts), damit ein Klick auf "Oben"/"Unten" einen
// Pitch erreicht, den man auch per Maus-Drag erreichen koennte (und der Folge-Drag danach nicht
// ploetzlich am Clamp haengt).
const MAX_ORBIT_PITCH = (Math.PI / 2) * 0.999

// Lokaler Versatz von link5s Ursprung (Montagepunkt) zur Saugerspitze, entlang link5s eigener
// Z-Achse -- Gizmo/IK-Ziel soll an der Spitze sitzen (das ist der Punkt, der ein Werkstueck
// beruehrt), nicht am Wellenausgang. solveIK() erwartet weiterhin link5s Ursprungsposition, daher
// Hin-/Rueckrechnung in syncTargetToRobot() bzw. vor jedem solveIK()-Aufruf waehrend eines Drags.
const TIP_OFFSET_LOCAL: Vec3 = [0, 0, VACUUM_CUP_TIP_OFFSET_M]

function tipTargetToLink5Origin(tipPos: Vec3, quat: Pose['quat']): Vec3 {
  return sub(tipPos, rotateVec3(quat, TIP_OFFSET_LOCAL))
}

// Der Sauger "traegt" einen 2,5cm-Holzwuerfel (s. primitives.ts) -- vor allem, damit man den
// Effekt von "Roll" ueberhaupt sieht (der Sauger selbst ist um seine eigene Achse
// rotationssymmetrisch). Wuerfelmittelpunkt sitzt direkt an der Saugerspitze plus halbe
// Kantenlaenge (beruehrt die Spitze, statt sie zu durchdringen).
const CARGO_CENTER_OFFSET_LOCAL: Vec3 = [0, 0, VACUUM_CUP_TIP_OFFSET_M + TRANSPORTED_CUBE_SIZE_M / 2]

// Kette-Segmentnamen <-> Wire-Format-Index (roarm-kinematics.ts' JOINT_NAMES/JointIndex):
// link1=Base(0), link2=Shoulder(1), link3=Elbow(2), link4=Wrist(3), link5=Roll(4). Gripper(5) hat
// keinen Eintrag in CHAIN (Vakuum-Sauger hat keine eigene Gelenkachse) und wird von dieser Ansicht
// weder dargestellt noch verstellt -- der Aufrufer haelt seinen zuletzt bekannten Wert selbst.
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
  /** Waehrend eines Gizmo-Drags kontinuierlich mit den 5 IK-geloesten Gelenkwinkeln (rad, Base..Roll,
   * s. CHAIN_LINK_NAMES) aufgerufen -- der Aufrufer mischt das in seine eigene 6-elementige
   * jointAnglesCentiDeg-Ablage (Gripper-Kanal bleibt unberuehrt) und schickt es ans Backend. */
  onJointAnglesPreview?(armAnglesRad: readonly number[]): void
  onDragEnd?(): void
}

export interface RoArm3DHandles {
  /** Volles 6-elementiges Winkel-Array (rad) wie vom Backend gemeldet -- Index 5 (Gripper) wird
   * ignoriert (Vakuum-Sauger hat keine eigene Animation). */
  setJointAnglesRad(anglesRad: readonly number[]): void
  /** Ghost-Overlay (letzte aufgezeichnete Pose) ein-/ausblenden; null blendet aus. */
  setGhostAnglesRad(anglesRad: readonly number[] | null): void
  dispose(): void
}

// --- Material: schwarz pulverbeschichtetes Aluminium (Arm-Meshes) -- dielektrisch/matt, kein
// Metallic-Glanz. Der Vakuum-Sauger ist dagegen mattes Gummi (anderes physisches Material). Ghost
// bleibt bewusst neutral (Sichtbarkeit wichtiger als Material-Realismus, s. GHOST_MATERIAL).
const ARM_COLOR: [number, number, number] = [24, 24, 26]
const ARM_MATERIAL = { metallic: 0.05, roughness: 0.58 }
const CUP_COLOR: [number, number, number] = [35, 35, 38]
const CUP_MATERIAL = { metallic: 0, roughness: 0.8 }
// Blau lackiertes Holz (das transportierte Werkstueck) -- eigenes Material, kein Metall, etwas
// weniger rau als Gummi (lackierte Oberflaeche).
const CARGO_COLOR: [number, number, number] = [45, 95, 175]
const CARGO_MATERIAL = { metallic: 0, roughness: 0.5 }
const GHOST_COLOR: [number, number, number] = [51, 148, 255]
const GHOST_OPACITY = 0.25
const GHOST_MATERIAL = { metallic: 0, roughness: 0.6 }

// Bodenraster faerbt sich zur Bewegungsgrenze hin gelb, dann rot ein -- grau=weit von jeder
// Gelenkgrenze entfernt, rot=an der Grenze bzw. das gezogene Ziel ist gerade unerreichbar (s.
// limitProximity()/solveIK()'s reachable-Flag in ik.ts). Reine Rueckmeldung, kein hartes Limit an
// sich -- solveIK() selbst verhindert schon, dass sich die Pose ueber die Grenze hinaus bewegt.
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

  // Bodenraster als echte, tiefengetestete WebGL-Geometrie (nicht auf dem 2D-Gizmo-Overlay) --
  // sonst liegt es immer VOR dem Roboter statt vom Roboter verdeckt zu werden, wenn dieser aus
  // Kamerasicht davor steht (verwirrend fuer ein Raster, das den Fussboden darstellen soll). Der
  // Gizmo bleibt bewusst auf dem 2D-Overlay -- soll immer greifbar bleiben, auch "hinter" einem
  // Armteil, uebliche Konvention in 3D-Tools.
  const gridHandle = glRenderer.createLines(makeGrid(1.0, 20), GRID_COLOR_NORMAL, 0.9)

  const liveParts = buildLinkedPartSet(glRenderer, vacuumCupMesh, cargoCubeMesh, 1)
  const ghostParts = buildLinkedPartSet(glRenderer, vacuumCupMesh, cargoCubeMesh, GHOST_OPACITY, GHOST_COLOR)
  setPartSetVisible(ghostParts, false)

  let angles: JointAngles = defaultAngles()
  let target: Pose = { pos: [0, 0, 0], quat: [0, 0, 0, 1] }
  let targetTiltSum = 0
  let suppressExternalResync = false // true while a gizmo drag is in progress
  let dragUnreachable = false // letzter solveIK()-Aufruf waehrend eines Drags: reachable=false
  // Waehrend eines Drehen-Drags fix auf die Spitzenposition beim Drag-Start eingefroren (s.
  // onPointerDown/onPointerMove) -- der Drehpunkt SOLL die Sauger-Spitze sein, nicht "wo auch immer
  // der letzte Frame gelandet ist", s. dortiger Kommentar zum eigentlichen Bug.
  let rotateDragAnchorPos: Vec3 | null = null

  function syncTargetToRobot(): void {
    const { poses } = forwardKinematics(angles)
    const wrist = poses[poses.length - 1] // link5 origin (Montagepunkt, nicht die Saugerspitze)
    target = { pos: transformPoint(wrist, TIP_OFFSET_LOCAL), quat: wrist.quat }
    targetTiltSum = tiltSum(angles)
  }

  // Ausrichtung fuer den Tilt-Ring (rot) des Gizmos: target.quat MINUS die aktuelle Roll-Drehung
  // (angles.link5), rausgerechnet durch Ruecknahme der zuletzt am Kettenende angewendeten
  // Z-Rotation. Roll ist kinematisch das LETZTE Kettenglied -- es aendert nichts an der Achse, um
  // die Tilt tatsaechlich kippt, sollte den Tilt-Ring also nicht optisch mitdrehen (ohne diese
  // Korrektur drehte sich der rote Ring beim Ziehen von Blau/Roll sichtbar mit, obwohl seine
  // Bedeutung unveraendert blieb). Der Roll-Ring selbst (blau) ist von dieser Korrektur unberuehrt
  // -- ein Kreis um seine eigene Normalenachse sieht bei jeder Roll-Drehung ohnehin gleich aus.
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

  // --- Gizmo-Umschalter (zwei kleine Segmented-Toggles in der Viewport-Ecke) ----------------------
  // Gemeinsamer Flex-Container statt zweier einzeln mit fixen top-Werten positionierter Elemente
  // -- deren tatsaechliche Hoehe (Button-Padding/Zeilenhoehe) liess sich per Augenmass nicht exakt
  // genug vorhersagen, zwei feste top-Werte fuehrten je nach Rendering zu Ueberlappung. Der
  // Flex-Container mit gap uebernimmt den Abstand automatisch, und beide Schalter richten sich
  // ueber align-items:flex-start an derselben linken Kante aus (start-align statt stretch, sonst
  // wuerde der schmalere "Welt/Lokal"-Schalter auf die Breite des breiteren gestreckt).
  const toggleStack = document.createElement('div')
  toggleStack.style.cssText = 'position:absolute;top:8px;left:8px;z-index:1;display:flex;flex-direction:column;align-items:flex-start;gap:6px;'
  container.appendChild(toggleStack)

  // "Bewegen" statt "Verschieben" -- Letzteres sprengte die Breite des Schalters (s.
  // createToggleSwitch()s feste Button-Breite, die beide Umschalter gleich gross haelt).
  const modeToggle = createToggleSwitch(['Bewegen', 'Drehen'], gizmo.mode === 'translate' ? 0 : 1, (i) => {
    gizmo.mode = i === 0 ? 'translate' : 'rotate'
  })
  toggleStack.appendChild(modeToggle.el)

  // Ausrichtung der Verschieben-Pfeile: Welt-Achsen (Default) oder das eigene Koordinatensystem
  // des Vakuumsaugers -- s. gizmo.ts' `space`-Feld. Betrifft nur den Verschieben-Modus (Drehen
  // bleibt immer an Tilt/Roll des Werkzeugs gekoppelt, s. dortiger Kommentar).
  const spaceToggle = createToggleSwitch(['Welt', 'Lokal'], gizmo.space === 'world' ? 0 : 1, (i) => {
    gizmo.space = i === 0 ? 'world' : 'local'
  })
  toggleStack.appendChild(spaceToggle.el)

  // --- ViewCube (oben rechts) -- eigenes SVG-Widget (s. view-cube.ts), verwaltet Positionierung
  // UND Klick-/Zug-Erkennung komplett selbst (echte DOM-Hit-Tests statt handgeschriebener
  // Projektions-/Punkt-in-Polygon-Mathematik). Ruft bei Klick auf eine Flaeche onViewPreset auf,
  // beim Ziehen onOrbitDrag -- dieselbe Orbit-Formel wie der Hauptviewport selbst (s. dessen
  // 'orbit'-Zweig unten), damit sich der Wuerfel bei gleicher Zuggeste in dieselbe Richtung dreht.
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

  // --- Pointer-Handling --------------------------------------------------------------------------
  // Kein 'cube'-Zweig mehr: das ViewCube-SVG (s. oben) sitzt als eigenes DOM-Element ueber diesem
  // Canvas und faengt Klicks/Zuege auf seinen Flaechen bereits selbst ab (echtes DOM-Hit-Testing) --
  // ein Klick daneben (transparenter Zwischenraum im SVG) faellt automatisch zu diesem Canvas durch
  // und wird hier ganz normal als Orbit behandelt, ohne dass roarm-3d-view.ts den Wuerfel selbst
  // abfragen muss.
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
      // Nur Achse 0 (rot, Tilt) und 2 (blau, Roll) sind ueberhaupt ziehbar (s.
      // gizmo.ts' ROTATE_AXES) -- Achse 1 (gruen) gibt es als Ring nicht mehr, die Kette hat keine
      // davon unabhaengige zweite Kippachse.
      if (result.rotateAxis === 0) {
        targetTiltSum += result.rotateAngle
      } else if (result.rotateAxis === 2) {
        angles.link5 = wrapPi((angles.link5 ?? 0) + result.rotateAngle)
      }
      if (result.changed) {
        // Bei einem Drehen-Drag IMMER die beim Drag-Start eingefrorene Spitzenposition anpeilen
        // (rotateDragAnchorPos), NICHT das jeweils letzte target.pos: solveIK() kann die vom Gizmo
        // frei akkumulierte Orientierung (target.quat) nur naeherungsweise erreichen (die Kette hat
        // nur 1 echten Tilt-Freiheitsgrad plus Roll, keine freie 3D-Drehung) -- die dadurch JEDEN
        // Frame neu entstehende kleine Abweichung wurde vorher als neuer Ausgangspunkt fuer den
        // NAECHSTEN Frame uebernommen und summierte sich ueber einen laengeren Zug zu mehreren
        // Zentimetern Versatz auf (gemessen: ca. 7mm/Frame, bei 20 Frames schon 3.6cm) -- sichtbar
        // als Drehpunkt, der sich sukzessive vom Sauger weg Richtung Handgelenk verschob. Mit dem
        // fixen Anker bleibt jeder Frame unabhaengig nah am WIRKLICH gewuenschten Drehpunkt, Fehler
        // koennen sich nicht mehr aufsummieren. Fuer Translate bleibt target.pos (bewusst bewegtes
        // Ziel) unveraendert massgeblich.
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
    // Bodenraster als Naeherungs-an-die-Bewegungsgrenze-Warnung: wie nah ist die aktuelle Pose an
    // irgendeiner Gelenkgrenze (limitProximity()), UND war das zuletzt angeforderte Gizmo-Ziel
    // gerade unerreichbar (dragUnreachable, s. onPointerMove) -- je hoeher, desto roeter.
    glRenderer.setColor(gridHandle, gridWarningColor(Math.max(limitProximity(angles), dragUnreachable ? 1 : 0)))
    viewCube.updatePoses(cam) // SVG-Widget -- aktualisiert nur Polygon-Punkte, kein WebGL-Renderpass mehr noetig
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

  // ResizeObserver statt (nur) window 'resize' -- der Container kann seine Groesse auch OHNE ein
  // Browserfenster-Resize aendern (Sidebar-Panel auf-/zugeklappt, Missionsliste waechst/schrumpft,
  // ein Scrollbalken erscheint/verschwindet anderswo auf der Seite und verschiebt die verfuegbare
  // Breite) -- ohne diese Beobachtung blieben WebGL-Canvas-Aufloesung UND die orthografische
  // Overlay-Kamera (die das ViewCube-Hit-Testing benutzt, s. view-cube.ts' projectOrtho()) auf dem
  // Stand des letzten ECHTEN Fenster-Resizes stehen, waehrend `canvas2d.getBoundingClientRect()`
  // (fuer die Maus-Koordinaten in onPointerDown/onPointerMove) immer den AKTUELLEN Wert liefert --
  // genau das Auseinanderlaufen von sichtbarer Wuerfelposition und klickbarem Bereich, das nur bei
  // bestimmten Fenstergroessen/Zustaenden auffiel (je nachdem, ob seit dem letzten echten
  // Fenster-Resize eine solche layoutgetriebene Groessenaenderung passiert war).
  const resizeObserver = new ResizeObserver(() => resize())
  resizeObserver.observe(container)

  return {
    setJointAnglesRad(anglesRad: readonly number[]): void {
      // Waehrend eines Gizmo-Drags kommen weiterhin laufend PoseFeedback-Updates vom Backend rein
      // (die reale/simulierte Bewegung hinkt dem Drag physikalisch begrenzt hinterher) -- wuerden
      // die hier ungebremst uebernommen, ueberschreiben sie die soeben lokal geloeste IK-Vorschau
      // jeden Frame wieder mit der (noch nicht angekommenen) alten Pose, und das Mesh scheint dem
      // Gizmo gar nicht zu folgen. Waehrend des Drags hat also die lokale Vorschau Vorrang; sobald
      // suppressExternalResync (s. onPointerDown/endDrag) wieder false ist, uebernimmt die Ist-Pose
      // des Backends nahtlos wieder die Fuehrung.
      if (suppressExternalResync) return
      if (anglesRad.length < JOINT_COUNT) return
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

  // --- lokale Hilfsfunktionen (schliessen ueber glRenderer/vacuumCupMesh/cargoCubeMesh) ----------
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
