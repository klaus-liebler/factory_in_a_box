import type { Pose, Quat } from './math.js'
import { composePose, quatFromAxisAngle, quatMultiply, quatNormalize } from './math.js'
import { CHAIN } from './robot-data.js'

export type JointAngles = Record<string, number>

// A "folded" starting pose (rather than fully extended) — easier to see the
// arm's shape at a glance and a more natural rest position to jog from.
const DEFAULT_ANGLES: JointAngles = {
  link2: 0.6,
  link3: 1.5,
  link4: 1,
}

export function defaultAngles(): JointAngles {
  const angles: JointAngles = {}
  for (const seg of CHAIN) if (seg.jointType === 'revolute') angles[seg.name] = DEFAULT_ANGLES[seg.name] ?? 0
  return angles
}

export interface ChainPoses {
  /** Pose of each segment's joint origin, *before* its own revolute rotation — this is the pivot/axis frame used by IK. */
  originPoses: Pose[]
  /** Pose of each segment after its own rotation is applied — where its mesh and children attach. */
  poses: Pose[]
}

const ROOT_POSE: Pose = { pos: [0, 0, 0], quat: [0, 0, 0, 1] }

export function forwardKinematics(angles: JointAngles): ChainPoses {
  const originPoses: Pose[] = []
  const poses: Pose[] = []
  let parent = ROOT_POSE
  for (const seg of CHAIN) {
    const origin: Pose = composePose(parent, { pos: seg.origin, quat: seg.quat })
    originPoses.push(origin)

    let pose = origin
    if (seg.jointType === 'revolute' && seg.axis) {
      const angle = angles[seg.name] ?? 0
      const spinQuat: Quat = quatFromAxisAngle(seg.axis, angle)
      pose = { pos: origin.pos, quat: quatNormalize(quatMultiply(origin.quat, spinQuat)) }
    }
    poses.push(pose)
    parent = pose
  }
  return { originPoses, poses }
}

export function segmentIndex(name: string): number {
  return CHAIN.findIndex((s) => s.name === name)
}

export function poseOf(chainPoses: ChainPoses, name: string): Pose {
  return chainPoses.poses[segmentIndex(name)]
}
