import type { Vec3 } from './math.js'
import { CHAIN } from './robot-data.js'
import type { JointAngles } from './kinematics.js'

/**
 * Closed-form (non-iterative) inverse kinematics for the RoArm-M3's 3
 * co-planar "pitch" joints (link2/link3/link4) plus base yaw (link1).
 *
 * This replaces an earlier CCD-based solver (see the old robot_ui
 * prototype) that never fully converged well for this chain. The
 * replacement follows the same overall strategy as glumb/robot-gui's
 * InverseKinematic.js (github.com/glumb/robot-gui) — solve base yaw from
 * the target's azimuth, subtract the last bone to get a "wrist center",
 * solve a 2-link triangle via law of cosines, then take the remaining
 * angle to hit the desired end orientation — but that script's algebra is
 * hardcoded to a different (6-DOF, differently-shaped) arm and can't be
 * reused verbatim; the constants and branch structure below were re-derived
 * numerically from *this* chain's actual FK (see the derivation script that
 * produced them — verified against 2000 random poses, sub-micron error).
 *
 * Geometric facts this relies on (all verified numerically, not assumed):
 *  - The shoulder pivot (link2's origin) sits exactly on the base yaw axis
 *    for any link1 angle, so the whole 3-bone chain from shoulder onward
 *    always lies in a single plane through that axis.
 *  - link2, link3, link4 share an exactly parallel world rotation axis
 *    (three coplanar "pitch" joints), and their *cumulative* rotation in
 *    that plane is a plain sum of joint angles plus fixed per-bone "kink"
 *    offsets (KINK1, KINK3 below) — i.e. no iteration needed to place them.
 *  - 3 pitch joints exactly match the 3 unknowns of "reach this (x,z) at
 *    this in-plane tilt angle" — this is a fully-determined (not
 *    redundant) planar 3R IK problem.
 *
 * Two independent binary configuration choices exist, same idea as
 * `config` in the reference script:
 *  - elbow "up"/"down" (which way the shoulder/elbowA triangle bends)
 *  - "front"/"back" (the target can be reached with the yaw plane facing
 *    it directly, *or* facing away with the arm folded back past the yaw
 *    axis to the same world point — easy to miss since it only shows up
 *    for large joint excursions, which is exactly how it surfaced during
 *    verification here)
 * `solveIK` tries all 4 combinations and picks whichever lands closest (in
 * joint-space, wrapped) to the arm's current pose, so dragging the target
 * doesn't cause a sudden reconfiguration jump.
 */

// --- chain constants (see the derivation script; verified to ~1e-6 rad / ~1e-6 m) ---
// H_SHOULDER was originally 0.122059 (= 0.0701 base_link height + 0.051959 link1->link2 origin
// z), derived against Waveshare's own base_link origin. This app's base_link.origin.z was zeroed
// out in web/tools/roarm-meshes/extract-robot-data.mjs (no base housing rendered, the turntable
// ring sits directly on the floor) -- the shoulder pivot's height in *this* chain is therefore
// just the remaining 0.051959, not the original 0.122059. Keep this in sync with that origin.z
// if it ever changes again.
const H_SHOULDER = 0.051959 // height of the shoulder pivot on the yaw axis (floor-mounted base)
const L1 = 0.2387079056709225 // shoulder -> elbowA
const L2 = 0.1445858850103661 // elbowA -> elbowB
const L3 = 0.05575008017231955 // elbowB -> wrist-roll-axis pivot
const KINK1 = (7.22008543731122 * Math.PI) / 180 // cumAngle1 = theta2 + KINK1
const KINK3 = (15.765148797130701 * Math.PI) / 180 // cumAngle3 = (theta2+theta3+theta4) + KINK3
// (kink2 == -KINK1 exactly, i.e. cumAngle2 = theta2 + theta3 with no offset — a
// clean designed relationship, and exactly why "link2+link3+link4 = const"
// held tilt constant in the old CCD-based prototype.)

const TILT_JOINTS = ['link2', 'link3', 'link4'] as const

export function tiltSum(angles: JointAngles): number {
  return TILT_JOINTS.reduce((s, name) => s + (angles[name] ?? 0), 0)
}

interface Candidate {
  link1: number
  link2: number
  link3: number
  link4: number
}

function solveOneConfig(
  targetPos: Vec3,
  phi: number,
  elbowConfig: 1 | -1,
  frontBack: 1 | -1,
): Candidate | null {
  const [x, y, z] = targetPos
  const rawAzimuth = Math.atan2(y, x)
  const R0 = frontBack > 0 ? rawAzimuth : rawAzimuth + Math.PI
  const r = Math.hypot(x, y)

  const relX = frontBack * r
  const relZ = z - H_SHOULDER

  // Place elbowB by subtracting the last bone (elbowB -> wrist-roll-axis)
  // pointed at the desired tilt phi.
  const elbowBx = relX - L3 * Math.sin(phi)
  const elbowBz = relZ - L3 * Math.cos(phi)

  const d = Math.hypot(elbowBx, elbowBz)
  if (d > L1 + L2 || d < Math.abs(L1 - L2) || d < 1e-9) return null

  const baseAngle = Math.atan2(elbowBx, elbowBz)
  const alpha = Math.acos(clampCos((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d)))
  const cumAngle1 = baseAngle + elbowConfig * alpha
  const theta2 = cumAngle1 - KINK1

  const beta = Math.acos(clampCos((L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2)))
  const cumAngle2 = cumAngle1 - elbowConfig * (Math.PI - beta)
  const theta3 = cumAngle2 - cumAngle1 + KINK1 // == cumAngle2 - theta2

  const theta4 = phi - cumAngle2 - KINK3

  return { link1: wrapAngle(R0), link2: theta2, link3: theta3, link4: theta4 }
}

function clampCos(v: number): number {
  return v < -1 ? -1 : v > 1 ? 1 : v
}

function wrapAngle(a: number): number {
  return ((a + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI
}

function angleDiff(a: number, b: number): number {
  return Math.abs(wrapAngle(a - b))
}

function withinLimits(c: Candidate): boolean {
  for (const seg of CHAIN) {
    if (!seg.limit) continue
    const v = (c as unknown as Record<string, number>)[seg.name]
    if (v === undefined) continue
    if (v < seg.limit[0] - 1e-6 || v > seg.limit[1] + 1e-6) return false
  }
  return true
}

// Wie nah ist irgendein Gelenk an SEINER EIGENEN Grenze? 0 = alle Gelenke bequem in der Mitte
// ihres Bereichs, 1 = mindestens eines genau an (oder ueber) seiner Grenze. Nur das aeussere
// LIMIT_WARNING_ZONE des jeweiligen Bereichs ramped ueberhaupt hoch (sonst waere schon eine leicht
// asymmetrische Mittelstellung "gelb") -- fuers Bodenraster-Warnsignal in roarm-3d-view.ts, s. dort.
// War zuvor 0.25 mit linearem Anstieg -- das faerbte das Raster schon in der gefalteten
// Ruhepose ein (Wrist steht dort bei ca. 57% seines Bereichs, also < 25% vom Rand entfernt).
// Kleinere Zone + quadratischer statt linearer Anstieg: nahe der Zonengrenze fast unsichtbar,
// zieht erst wirklich nah an der Gelenkgrenze sichtbar an.
const LIMIT_WARNING_ZONE = 0.12 // Anteil des Gelenkbereichs, ab dem die Warnung einsetzt

export function limitProximity(angles: JointAngles): number {
  let maxProximity = 0
  for (const seg of CHAIN) {
    if (!seg.limit) continue
    const v = angles[seg.name] ?? 0
    const [min, max] = seg.limit
    const range = max - min
    if (range <= 1e-6) continue
    const distToNearestLimit = Math.min(v - min, max - v)
    const warningZoneWidth = range * LIMIT_WARNING_ZONE
    const linear = 1 - Math.max(0, Math.min(1, distToNearestLimit / warningZoneWidth))
    const proximity = linear * linear
    maxProximity = Math.max(maxProximity, proximity)
  }
  return maxProximity
}

export interface IKResult {
  angles: JointAngles
  reachable: boolean
}

/**
 * Solves link1..link4 for `targetPos` (the desired position of the
 * wrist-roll-axis pivot, i.e. link5's origin) and `targetTiltSum` (the
 * desired `link2+link3+link4`, same convention the old CCD prototype used
 * for "tilt from vertical" — see `tiltSum`). link5 (wrist roll) isn't
 * solved here: it doesn't affect position or tilt at all, so it's a free
 * parameter the caller sets directly (slider / roll gizmo ring), not part
 * of IK.
 *
 * Picks among the up-to-4 valid configurations (elbow up/down x
 * front/back) by minimizing joint-space movement from `currentAngles`, so
 * that dragging the target doesn't cause a sudden reconfiguration jump.
 */
export function solveIK(currentAngles: JointAngles, targetPos: Vec3, targetTiltSum: number): IKResult {
  const phi = targetTiltSum + KINK3

  const all: Candidate[] = []
  for (const elbowConfig of [1, -1] as const) {
    for (const frontBack of [1, -1] as const) {
      const c = solveOneConfig(targetPos, phi, elbowConfig, frontBack)
      if (c) all.push(c)
    }
  }
  if (all.length === 0) {
    return { angles: currentAngles, reachable: false }
  }

  function cost(c: Candidate): number {
    return (
      angleDiff(c.link1, currentAngles.link1 ?? 0) +
      angleDiff(c.link2, currentAngles.link2 ?? 0) +
      angleDiff(c.link3, currentAngles.link3 ?? 0) +
      angleDiff(c.link4, currentAngles.link4 ?? 0)
    )
  }

  // Nur Kandidaten, die OHNE Klemmen innerhalb aller Gelenkgrenzen liegen, kommen infrage. Frueher
  // fiel dies bei leerer withinLimit-Liste auf "irgendeinen" Kandidaten zurueck und klemmte dessen
  // Winkel EINZELN auf die Grenzen -- das Klemmen jedes Gelenks fuer sich ergibt aber keine in sich
  // konsistente (geometrisch gueltige) Pose mehr, sichtbar als "Ausrichtung kippt/springt weg",
  // sobald der Rand des erreichbaren Bereichs ueberschritten wird. Stattdessen jetzt: keine
  // gueltige Konfiguration -> einfrieren (wie der leere-all-Fall oben), der Arm bewegt sich dann
  // schlicht nicht weiter, als es die Kinematik zulaesst.
  const withinLimit = all.filter(withinLimits)
  if (withinLimit.length === 0) {
    return { angles: currentAngles, reachable: false }
  }
  withinLimit.sort((a, b) => cost(a) - cost(b))
  const best = withinLimit[0]

  return {
    angles: { ...currentAngles, link1: best.link1, link2: best.link2, link3: best.link3, link4: best.link4 },
    reachable: true,
  }
}
