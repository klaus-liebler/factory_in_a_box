// Abstraction of the firmware communication for the RoArm teach workspace. The real implementation
// (WsRoArmBackend, roarm-ws-backend.ts; firmware side in src/web.cpp and src/roarm/roarm.cpp) sends/receives
// the generated BestBinaryBuffers message types imported here (roarm.* from generated/ws-protocol.ts)
// unchanged via ws-client.ts. MockRoArmBackend simulates the same interface purely on the client, so that the
// teach workspace can be used and tested on the PC without firmware/microcontroller (see roarm-teach-app.ts,
// which is programmed against this interface only).

import { roarm } from "../../../generated/ws-protocol.ts";
import { JOINT_COUNT, forwardKinematics, inverseKinematics, radToCentiDeg, centiDegToRad, type CartesianPose } from "./roarm-kinematics.ts";
import { MultiJointTracker } from "./roarm-motion-tracker.ts";

export type MissionStep = roarm.SaveMissionRequest.Payload["steps"][number];

export interface StoredMission {
	name: string;
	steps: MissionStep[];
}

export interface RoArmBackend {
	/* Starts teach mode (blocks mission playback in the real firmware). */
	startTeachMode(): Promise<boolean>;
	stopTeachMode(): Promise<boolean>;

	/* Fire-and-forget like the real JointJogTarget event - no request/response round trip. */
	setJointJogTargetCentiDeg(jointAnglesCentiDeg: readonly number[]): void;
	/* Like above, but cartesian - the real firmware does the IK, the mock does it locally (roarm-kinematics.ts). */
	setCartesianJogTarget(pose: CartesianPose): void;
	/* For mission playback (play): moves with at most maxSpeedDegPerSec instead of jumping at once, see
	   playMission() in roarm-teach-app.ts. The real hardware is speed-limited by the servo control anyway, so
	   WsRoArmBackend simply delegates to setJointJogTargetCentiDeg; the mock simulates the ramp on the client
	   via MultiJointTracker. */
	setJointMoveTargetCentiDeg(jointAnglesCentiDeg: readonly number[], maxSpeedDegPerSec: number): void;

	/* Returns the last known pose immediately (for the initial render, before the first PoseFeedback message). */
	getLastPoseFeedback(): roarm.PoseFeedback.Payload;
	/* Like the WS push - the return value is an unsubscribe function. */
	subscribePoseFeedback(cb: (feedback: roarm.PoseFeedback.Payload) => void): () => void;

	getMissionGpioNames(): Promise<string[]>;
	listMissions(): Promise<roarm.MissionSummary.Payload[]>;
	getMission(missionIndex: number): Promise<StoredMission | null>;
	saveMission(missionIndex: number, name: string, steps: MissionStep[]): Promise<{ success: boolean; errorCode: number }>;
	deleteMission(missionIndex: number): Promise<boolean>;
}

const MOCK_LATENCY_MS = 80;
function mockDelay<T>(value: T): Promise<T> {
	return new Promise((resolve) => setTimeout(() => resolve(value), MOCK_LATENCY_MS));
}

// Only one single mission GPIO on this arm - it carries a vacuum suction cup instead of the mechanical
// gripper, so there is only "vacuum on/off" to switch, no generic multi-GPIO selection (see addVacuumStep()
// in roarm-teach-app.ts).
export const VACUUM_GPIO_ID = 0;
const MOCK_MISSION_GPIO_NAMES = ["Vakuumsauger"];

// "Folded" rest pose instead of the fully stretched zero position (all angles 0) - the zero position is close
// to the maximum reach (almost singular for the closed-form gizmo IK, see roarm3d/ik.ts), where even small
// gizmo drags hardly move anything. The same values as DEFAULT_ANGLES of robot_ui_programmatic
// (link2=0.6/link3=1.5/link4=1 rad), mapped to shoulder/elbow/wrist.
export const FOLDED_REST_POSE_CENTIDEG: readonly number[] = [0, 3438, 8594, 5730, 0, 0];

// Deliberately generous (not taken from the reference firmware, which has no explicit servo
// speed/acceleration values for target tracking) - only for the local feel of motion in the mock; the actual
// values are tuned on the real arm during the firmware bring-up.
const MOCK_MAX_VELOCITY_RAD_PER_SEC = (90 * Math.PI) / 180;
const MOCK_MAX_ACCEL_RAD_PER_SEC2 = (180 * Math.PI) / 180;

export class MockRoArmBackend implements RoArmBackend {
	private readonly tracker: MultiJointTracker;
	private readonly missions = new Map<number, StoredMission>();
	private readonly poseListeners = new Set<(feedback: roarm.PoseFeedback.Payload) => void>();
	private lastPose: roarm.PoseFeedback.Payload;
	private teachModeActive = false;
	private lastTickAtMs = performance.now();
	private rafHandle = 0;

	constructor(initialJointsRad: readonly number[] = FOLDED_REST_POSE_CENTIDEG.map(centiDegToRad)) {
		this.tracker = new MultiJointTracker(initialJointsRad, MOCK_MAX_VELOCITY_RAD_PER_SEC, MOCK_MAX_ACCEL_RAD_PER_SEC2);
		this.lastPose = this.computeFeedback();
		this.seedDemoMissions();
		this.startLoop();
	}

	private seedDemoMissions(): void {
		this.missions.set(1, {
			name: "Demo: Pick & Place",
			steps: [
				{ classId: roarm.JointMoveStep.CLASS_ID, jointAnglesCentiDeg: [0, 0, 0, 0, 0, 0], maxSpeedDegPerSec: 60, isWaypoint: false },
				{ classId: roarm.JointMoveStep.CLASS_ID, jointAnglesCentiDeg: [3000, -2000, 4000, 0, 0, 0], maxSpeedDegPerSec: 60, isWaypoint: false },
				{ classId: roarm.GpioStep.CLASS_ID, gpioId: 0, state: true },
				{ classId: roarm.DelayStep.CLASS_ID, durationMs: 500 },
				{ classId: roarm.JointMoveStep.CLASS_ID, jointAnglesCentiDeg: [0, 0, 0, 0, 0, 0], maxSpeedDegPerSec: 60, isWaypoint: false },
				{ classId: roarm.GpioStep.CLASS_ID, gpioId: 0, state: false },
			],
		});
	}

	private startLoop(): void {
		const step = () => {
			const now = performance.now();
			const dt = (now - this.lastTickAtMs) / 1000;
			this.lastTickAtMs = now;
			this.tracker.tick(dt);
			this.lastPose = this.computeFeedback();
			if (this.teachModeActive) {
				for (const cb of this.poseListeners) cb(this.lastPose);
			}
			this.rafHandle = requestAnimationFrame(step);
		};
		this.rafHandle = requestAnimationFrame(step);
	}

	dispose(): void {
		cancelAnimationFrame(this.rafHandle);
	}

	private computeFeedback(): roarm.PoseFeedback.Payload {
		const jointsRad = this.tracker.currentRad;
		const pose = forwardKinematics(jointsRad);
		return {
			jointAnglesCentiDeg: jointsRad.map(radToCentiDeg),
			xMm: Math.round(pose.xMm),
			yMm: Math.round(pose.yMm),
			zMm: Math.round(pose.zMm),
			pitchCentiDeg: radToCentiDeg(pose.pitchRad),
			rollCentiDeg: radToCentiDeg(pose.rollRad),
			servoStatus: new Array(7).fill(roarm.ServoStatusBits.Ok),
		};
	}

	async startTeachMode(): Promise<boolean> {
		this.teachModeActive = true;
		return mockDelay(true);
	}

	async stopTeachMode(): Promise<boolean> {
		this.teachModeActive = false;
		return mockDelay(true);
	}

	// snapTo() instead of setTargets(): jogging (slider/gizmo/cartesian target) is direct hand guidance - the
	// user expects 1:1 following without the acceleration-limited profile in between, which otherwise feels like
	// "smoothing"/lagging of the movement. The tracker stays usable for a later mission playback simulation
	// (there via setTargets() + tick()).
	setJointJogTargetCentiDeg(jointAnglesCentiDeg: readonly number[]): void {
		this.tracker.snapTo(jointAnglesCentiDeg.map(centiDegToRad));
	}

	setCartesianJogTarget(pose: CartesianPose): void {
		const { jointsRad, isReachable } = inverseKinematics(pose);
		if (isReachable) this.tracker.snapTo(jointsRad);
	}

	setJointMoveTargetCentiDeg(jointAnglesCentiDeg: readonly number[], maxSpeedDegPerSec: number): void {
		const maxVelocityRadPerSec = centiDegToRad(maxSpeedDegPerSec * 100);
		this.tracker.setTargetsWithSpeed(jointAnglesCentiDeg.map(centiDegToRad), maxVelocityRadPerSec);
	}

	getLastPoseFeedback(): roarm.PoseFeedback.Payload {
		return this.lastPose;
	}

	subscribePoseFeedback(cb: (feedback: roarm.PoseFeedback.Payload) => void): () => void {
		this.poseListeners.add(cb);
		return () => this.poseListeners.delete(cb);
	}

	async getMissionGpioNames(): Promise<string[]> {
		return mockDelay([...MOCK_MISSION_GPIO_NAMES]);
	}

	async listMissions(): Promise<roarm.MissionSummary.Payload[]> {
		const list = [...this.missions.entries()]
			.map(([missionIndex, m]) => ({ missionIndex, name: m.name }))
			.sort((a, b) => a.missionIndex - b.missionIndex);
		return mockDelay(list);
	}

	async getMission(missionIndex: number): Promise<StoredMission | null> {
		const found = this.missions.get(missionIndex);
		return mockDelay(found ? { name: found.name, steps: [...found.steps] } : null);
	}

	async saveMission(missionIndex: number, name: string, steps: MissionStep[]): Promise<{ success: boolean; errorCode: number }> {
		if (missionIndex <= 0) return mockDelay({ success: false, errorCode: -1 });
		this.missions.set(missionIndex, { name, steps: [...steps] });
		return mockDelay({ success: true, errorCode: 0 });
	}

	async deleteMission(missionIndex: number): Promise<boolean> {
		const existed = this.missions.delete(missionIndex);
		return mockDelay(existed);
	}
}
