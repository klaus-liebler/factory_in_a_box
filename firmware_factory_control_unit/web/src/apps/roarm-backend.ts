// Abstraktion der Firmware-Kommunikation fuer die RoArm-Teach-Seite. Die echte Implementierung
// (WsRoArmBackend, folgt sobald die Firmware-Seite -- webserver.cpp/http_websocket_server.hpp --
// verdrahtet ist) sendet/empfaengt die hier importierten, bereits generierten BestBinaryBuffers-
// Nachrichtentypen (roarm.* aus generated/ws-protocol.ts) unveraendert ueber ws-client.ts.
// MockRoArmBackend simuliert dieselbe Schnittstelle rein clientseitig, damit die Teach-Seite schon
// jetzt -- ohne Firmware/Mikrocontroller -- am PC benutzt und getestet werden kann (s.
// roarm-teach-app.ts, das ausschliesslich gegen dieses Interface programmiert ist).

import { roarm } from "../../generated/ws-protocol.js";
import { JOINT_COUNT, forwardKinematics, inverseKinematics, radToCentiDeg, centiDegToRad, type CartesianPose } from "./roarm-kinematics.js";
import { MultiJointTracker } from "./roarm-motion-tracker.js";

export type MissionStep = roarm.SaveMissionRequest.Payload["steps"][number];

export interface StoredMission {
	name: string;
	steps: MissionStep[];
}

export interface RoArmBackend {
	/** Startet den Teach-Modus (sperrt in der echten Firmware das Mission-Playback). */
	startTeachMode(): Promise<boolean>;
	stopTeachMode(): Promise<boolean>;

	/** Feuert-und-vergisst wie das echte JointJogTarget-Event -- kein Request/Response-Umlauf. */
	setJointJogTargetCentiDeg(jointAnglesCentiDeg: readonly number[]): void;
	/** Wie oben, aber kartesisch -- die echte Firmware macht IK, der Mock macht es lokal (roarm-kinematics.ts). */
	setCartesianJogTarget(pose: CartesianPose): void;

	/** Liefert die zuletzt bekannte Pose sofort (fuer den initialen Render, vor der ersten PoseFeedback-Nachricht). */
	getLastPoseFeedback(): roarm.PoseFeedback.Payload;
	/** Analog zum WS-Push -- Rueckgabewert ist eine Unsubscribe-Funktion. */
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

// Placeholder-Namen exakt wie das geplante Firmware-Pendant (Core/Src/setup_and_loops/
// roarm_mission_gpio.hh) -- dort ohne zugewiesene Pins, bis feststeht, welche GPIOs frei sind.
const MOCK_MISSION_GPIO_NAMES = ["Mission GPIO 1", "Mission GPIO 2", "Mission GPIO 3", "Mission GPIO 4"];

// Bewusst grosszuegig (nicht aus der Referenzfirmware uebernommen, dort keine expliziten
// Servo-Geschwindigkeits-/Beschleunigungswerte fuer die Ziel-Verfolgung) -- rein fuers lokale
// Bewegungsgefuehl im Mock; die tatsaechlichen Werte werden beim Firmware-Bring-up am realen Arm
// abgestimmt.
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

	constructor(initialJointsRad: readonly number[] = new Array(JOINT_COUNT).fill(0)) {
		this.tracker = new MultiJointTracker(initialJointsRad, MOCK_MAX_VELOCITY_RAD_PER_SEC, MOCK_MAX_ACCEL_RAD_PER_SEC2);
		this.lastPose = this.computeFeedback();
		this.seedDemoMissions();
		this.startLoop();
	}

	private seedDemoMissions(): void {
		this.missions.set(1, {
			name: "Demo: Pick & Place",
			steps: [
				{ classId: roarm.JointMoveStep.CLASS_ID, jointAnglesCentiDeg: [0, 0, 0, 0, 0, 0], maxSpeedDegPerSec: 60 },
				{ classId: roarm.JointMoveStep.CLASS_ID, jointAnglesCentiDeg: [3000, -2000, 4000, 0, 0, 0], maxSpeedDegPerSec: 60 },
				{ classId: roarm.GpioStep.CLASS_ID, gpioId: 0, state: true },
				{ classId: roarm.DelayStep.CLASS_ID, durationMs: 500 },
				{ classId: roarm.JointMoveStep.CLASS_ID, jointAnglesCentiDeg: [0, 0, 0, 0, 0, 0], maxSpeedDegPerSec: 60 },
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

	setJointJogTargetCentiDeg(jointAnglesCentiDeg: readonly number[]): void {
		this.tracker.setTargets(jointAnglesCentiDeg.map(centiDegToRad));
	}

	setCartesianJogTarget(pose: CartesianPose): void {
		const { jointsRad, isReachable } = inverseKinematics(pose);
		if (isReachable) this.tracker.setTargets(jointsRad);
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
