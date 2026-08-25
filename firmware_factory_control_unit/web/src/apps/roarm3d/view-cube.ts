// Navigations-Wuerfel oben rechts im Viewport, angelehnt an Autodesk Fusion 360s ViewCube.
//
// Echtes SVG statt der fruoheren WebGL-Loesung (texturierte 3D-Quads in einer eigenen
// orthografischen Overlay-Szene). Der WebGL-Ansatz brauchte eine handgeschriebene, zur
// Render-Projektion parallele Hit-Test-Mathematik (projectOrtho() + point-in-polygon), die bei
// jeder Aenderung an Seitenverhaeltnis/Container-Groesse exakt im Gleichschritt mit der
// tatsaechlichen Projektionsmatrix bleiben musste. Trotz mehrerer Fixes (aspect-abhaengige
// Positionierung, ResizeObserver statt nur window-resize, kumulativer statt Pro-Event-
// Drag-Schwellwert) blieb das in der Praxis fragil -- klickbarer Bereich und sichtbare
// Darstellung liefen bei manchen Fenstergroessen/Layout-Zustaenden weiterhin auseinander.
//
// SVG-Elemente sind ECHTE DOM-Knoten -- der Browser macht das Hit-Testing selbst, exakt auf Basis
// dessen, was tatsaechlich gezeichnet wird. Es kann also gar nicht mehr auseinanderlaufen. Die
// Positionierung des ganzen Widgets ist reines CSS (fixe Bildschirmecke, feste Pixelgroesse,
// voellig unabhaengig vom 3D-Viewport-Seitenverhaeltnis) -- pro Frame wird nur noch die ROTATION
// der Flaechen (als SVG-Polygon-Punkte) neu berechnet, dieselbe Quaternion-Mathematik wie zuvor,
// nur projiziert auf eine simple, feste 2D-Flaeche statt eine aspect-abhaengige Kamera-Projektion.

import type { Vec3, Quat } from './math.js'
import { add, dot, normalize, quatConjugate, quatFromAxisAngle, rotateVec3, scale } from './math.js'
import type { Camera } from './renderer.js'

export interface ViewPreset {
  /** null = aktuellen Yaw beibehalten (fuer Oben/Unten -- sonst "kippt" die Ansicht willkuerlich). */
  yaw: number | null
  pitch: number
}

interface FaceDef {
  localNormal: Vec3
  label: string
  color: string // direkt als SVG-fill
  preset: ViewPreset
  /** Korrektur, wie der Buchstabe auf DIESER Flaeche steht -- quatFromZTo() waehlt die lokale
   * X/Y-Ausrichtung jeder Flaeche nicht "lesbar", sondern nur irgendeine orthonormale Basis
   * senkrecht zur Normalen, das ergibt je nach Flaeche eine andere scheinbare Drehung. Werte per
   * Augenmass festgelegt. Grad, im Uhrzeigersinn. */
  textRotationDeg?: number
}

const NEAR_TOP = (Math.PI / 2) * 0.999 // Singularitaet bei exakt 90 Grad vermeiden

// Weltkonvention wie im Hauptviewport: Z hoch, "vorne" willkuerlich aber konsistent auf +Y gelegt.
const FACES: FaceDef[] = [
  { localNormal: [0, 1, 0], label: 'V', color: 'rgb(70,150,90)', preset: { yaw: Math.PI / 2, pitch: 0 }, textRotationDeg: 180 },
  { localNormal: [0, -1, 0], label: 'H', color: 'rgb(60,110,75)', preset: { yaw: -Math.PI / 2, pitch: 0 }, textRotationDeg: 0 },
  { localNormal: [1, 0, 0], label: 'R', color: 'rgb(190,80,80)', preset: { yaw: 0, pitch: 0 }, textRotationDeg: -90 },
  { localNormal: [-1, 0, 0], label: 'L', color: 'rgb(150,65,65)', preset: { yaw: Math.PI, pitch: 0 }, textRotationDeg: 90 },
  { localNormal: [0, 0, 1], label: 'O', color: 'rgb(70,110,190)', preset: { yaw: null, pitch: NEAR_TOP }, textRotationDeg: 0 },
  { localNormal: [0, 0, -1], label: 'U', color: 'rgb(60,90,150)', preset: { yaw: null, pitch: -NEAR_TOP }, textRotationDeg: 180 },
]

const SVG_PX = 84 // feste CSS-Pixelgroesse des Widgets -- kein JS-Resize noetig
const VIEWBOX_HALF = 50 // SVG-User-Units, viewBox="-50 -50 100 100"
const CUBE_HALF = 26 // Kantenlaenge/2 des Wuerfels in denselben User-Units
const DRAG_THRESHOLD_PX = 14 // kumulativ ab Pointerdown gemessen (nicht pro Move-Event) -- sonst
// wird ein leicht zittriger echter Klick faelschlich als Drag gewertet und der Ansichtswechsel
// bleibt aus.

/** Quaternion, die lokal +Z auf `normal` dreht -- Standardtrick fuer "richte ein Flaechen-Mesh
 * (das per Default +Z zugewandt ist) auf seine Aussenrichtung aus", s. auch primitives.ts. */
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
  /** Rohe Pixel-Deltas (nicht vorskaliert) -- der Aufrufer wendet dieselbe Formel an wie fuer
   * seinen eigenen Haupt-Viewport-Orbit-Drag (s. roarm-3d-view.ts), damit sich der Wuerfel bei
   * gleicher Zuggeste in dieselbe Richtung dreht wie die Hauptansicht. */
  onOrbitDrag(dx: number, dy: number): void
}

export interface ViewCubeHandle {
  /** Jeden Frame vor dem Rendern aufrufen -- aktualisiert die SVG-Flaechen-Polygone kamerarelativ. */
  updatePoses(camera: Camera): void
  dispose(): void
}

export function createViewCube(container: HTMLElement, callbacks: ViewCubeCallbacks): ViewCubeHandle {
  const svg = document.createElementNS(SVG_NS, 'svg')
  svg.setAttribute('viewBox', `${-VIEWBOX_HALF} ${-VIEWBOX_HALF} ${VIEWBOX_HALF * 2} ${VIEWBOX_HALF * 2}`)
  // Feste Bildschirmecke, feste Pixelgroesse -- reines CSS, kein JS setzt hier jemals eine
  // Position/Groesse (s. Moduskommentar). pointer-events:none auf der Wurzel + auto auf jedem
  // Flaechen-Polygon: ein Klick in den (unsichtbaren) Zwischenraum zwischen den Flaechen faellt
  // dadurch automatisch zum darunterliegenden Canvas durch (normales Orbit dort), ganz ohne
  // manuelles Hit-Testing.
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

    // x/y bleiben fest bei 0,0 (Flaechenmitte im lokalen, unrotierten Koordinatensystem) -- die
    // Positionierung UND Verzerrung passiert komplett ueber das pro Frame gesetzte "transform"
    // (s. updatePoses()), nicht mehr ueber x/y.
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

  // --- Klick-vs-Zug: kumulativ ab der Pointerdown-Position gemessen (nicht gegen die jeweils
  // letzte Move-Position), s. DRAG_THRESHOLD_PX-Kommentar. ---------------------------------------
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

  // --- Projektion: Wuerfel-lokale Ecken (per quatFromZTo() wie zuvor) mit der weltfesten
  // Kamera-Konjugat-Rotation gedreht, dann simpel orthografisch auf 2D projiziert (Z fallen
  // lassen, Y gespiegelt -- SVG waechst nach unten). Keine Aspect-Ratio-Abhaengigkeit mehr: das
  // Widget hat eine feste Pixelgroesse, die Projektion braucht daher keinen Container-Bezug. -----
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
    // Weltfeste Ausrichtung (Konjugat der Hauptkamera-Rotation): der Wuerfel zeigt dadurch genau
    // das, was ein weltfester Wuerfel von der aktuellen Hauptkamera-Blickrichtung aus zeigen
    // wuerde, obwohl dieses Widget selbst nie mitschwenkt (feste Bildschirmecke).
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

      // Echte perspektivische (genauer: affine -- die orthografische Projektion einer ebenen
      // Flaeche ist immer ein Parallelogramm, Geradentreue bleibt erhalten) Verzerrung des
      // Buchstabens statt einer reinen Drehung: aus den bereits berechneten Eckpunkten die
      // Basisvektoren "eine lokale Einheit in X" (ex) und "eine lokale Einheit in Y" (ey) der
      // Flaeche IN BILDSCHIRMKOORDINATEN ableiten -- ex/ey sind bei einer schraeg angeschauten
      // Flaeche unterschiedlich lang/nicht senkrecht zueinander, genau das laesst den Buchstaben
      // gestaucht/geschert statt starr gedreht wirken (bei der frueheren WebGL-Fassung kam das
      // automatisch aus der 3D-Mesh-Transform, hier muss es explizit als SVG-Matrix nachgebaut
      // werden, da nur noch flach in 2D projiziert wird). Der Buchstabe selbst steht am lokalen
      // Ursprung (x=0,y=0, s. Erzeugung oben) -- die Matrix uebernimmt Position UND Verzerrung.
      // quatFromZTo() liefert IMMER eine rechtshaendige lokale Basis (right x up = normal, egal
      // fuer welche Flaeche -- rechnerisch nachgeprueft). project() spiegelt aber Y (SVG waechst
      // nach unten) -- eine reine Spiegelung kehrt die Haendigkeit um, und zwar fuer JEDE
      // kamerazugewandte Flaeche GLEICHERMASSEN (nicht nur fuer einzelne). Ohne Korrektur erscheinen
      // Buchstaben deshalb konsequent spiegelverkehrt (sichtbar u.a. an einem "R", das wie ein
      // rueckwaerts geschriebenes "Я" aussieht). ey wird deshalb hier einmal, einheitlich negiert
      // -- das dreht die Haendigkeit der TEXT-Basis wieder um (die Polygon-Eckpunkte selbst bleiben
      // unveraendert, ein Quadrat ist symmetrisch und zeigt eine Spiegelung ohnehin nicht).
      const exX = (projected[1][0] - projected[0][0]) / (2 * CUBE_HALF)
      const exY = (projected[1][1] - projected[0][1]) / (2 * CUBE_HALF)
      const eyX = -(projected[3][0] - projected[0][0]) / (2 * CUBE_HALF)
      const eyY = -(projected[3][1] - projected[0][1]) / (2 * CUBE_HALF)

      // Fester Basis-Korrekturwinkel je Flaeche (textRotationDeg) -- quatFromZTo() waehlt die
      // lokale Achsen-Basis jeder Flaeche nicht "lesbar", sondern nur irgendeine zur Normalen
      // senkrechte Basis, das ergibt je Flaeche eine andere, aber KONSTANTE Ausgangs-Verdrehung.
      // Als Rotation der Basisvektoren selbst angewendet (nicht als Rotation des Ergebnisses),
      // damit sie sich mit der eigentlichen Verzerrung sauber kombiniert statt sie zu ueberschreiben.
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
