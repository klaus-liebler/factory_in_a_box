// Echte Firmware-Anbindung fuers RoArmBackend-Interface (roarm-backend.ts) -- ersetzt
// MockRoArmBackend, sobald die Firmware-Seite existiert (WS-Handler in webserver.cpp,
// RoArmSetupAndLoop in Core/Src/setup_and_loops/roarm.hh). Uebersetzt 1:1 auf die generierten
// roarm.*-Nachrichtentypen (web/generated/ws-protocol.ts) ueber die generischen Sende-/
// Anfrage-Helfer aus ws-client.ts -- kein eigenes Wire-Format-Wissen hier. Besitzt den
// roarm-Namespace: als dessen WsProtocolListener empfaengt es alle roarm-Push-Nachrichten
// (aktuell PoseFeedback) und verteilt sie selbst an seine Abonnenten.
import { roarm } from "../../generated/ws-protocol.js";
import { registerWsProtocolListener, sendBinary, wsRequest, type WsProtocolListener } from "../ws-client.js";
import { JOINT_COUNT, radToCentiDeg, type CartesianPose } from "./roarm-kinematics.js";
import type { RoArmBackend, MissionStep, StoredMission } from "./roarm-backend.js";

function emptyPoseFeedback(): roarm.PoseFeedback.Payload {
	return {
		jointAnglesCentiDeg: new Array(JOINT_COUNT).fill(0),
		xMm: 0,
		yMm: 0,
		zMm: 0,
		pitchCentiDeg: 0,
		rollCentiDeg: 0,
		servoStatus: new Array(7).fill(roarm.ServoStatusBits.Ok),
	};
}

export class WsRoArmBackend implements RoArmBackend, WsProtocolListener {
	private lastPose: roarm.PoseFeedback.Payload = emptyPoseFeedback();
	private readonly poseListeners = new Set<(feedback: roarm.PoseFeedback.Payload) => void>();
	private readonly unregisterListener: () => void;

	constructor() {
		this.unregisterListener = registerWsProtocolListener(roarm.NAMESPACE_ID, this);
	}

	/** Meldet den roarm-Namespace-Listener wieder ab -- fuer den seltenen Fall, dass eine Seite dieses
	 * Backend nicht mehr braucht (aktuell lebt es fuer die gesamte Seitenlebensdauer, s. roarm-teach-app.ts). */
	dispose(): void {
		this.unregisterListener();
	}

	// Responses (StartTeachModeResponse usw.) landen hier nie -- die ordnet ws-client.ts direkt dem
	// wartenden wsRequest() zu. JointJogTarget/CartesianJogTarget gehen nur Client->Firmware.
	onWsMessage(messageTypeId: number, view: DataView): void {
		switch (messageTypeId) {
			case roarm.PoseFeedback.TYPE_ID:
				this.lastPose = roarm.PoseFeedback.decode(view, 0);
				for (const cb of this.poseListeners) cb(this.lastPose);
				return;
			default:
				console.debug(`WebSocket: unbekannte roarm-Nachricht typeId=${messageTypeId}`);
		}
	}

	async startTeachMode(): Promise<boolean> {
		try {
			const resp = await wsRequest(
				roarm.NAMESPACE_ID,
				(requestId) => roarm.StartTeachModeRequest.encode({ requestId }),
				roarm.StartTeachModeResponse,
			);
			return resp.success;
		} catch {
			return false;
		}
	}

	async stopTeachMode(): Promise<boolean> {
		try {
			const resp = await wsRequest(
				roarm.NAMESPACE_ID,
				(requestId) => roarm.StopTeachModeRequest.encode({ requestId }),
				roarm.StopTeachModeResponse,
			);
			return resp.success;
		} catch {
			return false;
		}
	}

	setJointJogTargetCentiDeg(jointAnglesCentiDeg: readonly number[]): void {
		sendBinary(roarm.JointJogTarget.encode({ jointAnglesCentiDeg: [...jointAnglesCentiDeg] }));
	}

	// Die Servo-Ansteuerung auf dem echten Board ist ohnehin geschwindigkeitsbegrenzt (kein
	// Sofort-Sprung wie im Mock ohne Rampe) -- maxSpeedDegPerSec ist daher hier nur fuer den Mock
	// relevant, s. Interface-Kommentar in roarm-backend.ts.
	setJointMoveTargetCentiDeg(jointAnglesCentiDeg: readonly number[]): void {
		this.setJointJogTargetCentiDeg(jointAnglesCentiDeg);
	}

	setCartesianJogTarget(pose: CartesianPose): void {
		sendBinary(
			roarm.CartesianJogTarget.encode({
				xMm: Math.round(pose.xMm),
				yMm: Math.round(pose.yMm),
				zMm: Math.round(pose.zMm),
				pitchCentiDeg: radToCentiDeg(pose.pitchRad),
				rollCentiDeg: radToCentiDeg(pose.rollRad),
				gripperCentiDeg: radToCentiDeg(pose.gripperRad),
			}),
		);
	}

	getLastPoseFeedback(): roarm.PoseFeedback.Payload {
		return this.lastPose;
	}

	subscribePoseFeedback(cb: (feedback: roarm.PoseFeedback.Payload) => void): () => void {
		this.poseListeners.add(cb);
		return () => this.poseListeners.delete(cb);
	}

	async getMissionGpioNames(): Promise<string[]> {
		try {
			const resp = await wsRequest(
				roarm.NAMESPACE_ID,
				(requestId) => roarm.GetMissionGpioListRequest.encode({ requestId }),
				roarm.GetMissionGpioListResponse,
			);
			return resp.names;
		} catch {
			return [];
		}
	}

	async listMissions(): Promise<roarm.MissionSummary.Payload[]> {
		try {
			const resp = await wsRequest(
				roarm.NAMESPACE_ID,
				(requestId) => roarm.ListMissionsRequest.encode({ requestId }),
				roarm.ListMissionsResponse,
			);
			return resp.missions.map((m) => ({ missionIndex: m.missionIndex, name: m.name }));
		} catch {
			return [];
		}
	}

	async getMission(missionIndex: number): Promise<StoredMission | null> {
		try {
			const resp = await wsRequest(
				roarm.NAMESPACE_ID,
				(requestId) => roarm.GetMissionRequest.encode({ requestId, missionIndex }),
				roarm.GetMissionResponse,
			);
			if (!resp.found) return null;
			return { name: resp.name, steps: resp.steps as MissionStep[] };
		} catch {
			return null;
		}
	}

	async saveMission(missionIndex: number, name: string, steps: MissionStep[]): Promise<{ success: boolean; errorCode: number }> {
		try {
			const resp = await wsRequest(
				roarm.NAMESPACE_ID,
				(requestId) => roarm.SaveMissionRequest.encode({ requestId, missionIndex, name, steps }),
				roarm.SaveMissionResponse,
			);
			return { success: resp.success, errorCode: resp.errorCode };
		} catch {
			return { success: false, errorCode: -1 };
		}
	}

	async deleteMission(missionIndex: number): Promise<boolean> {
		try {
			const resp = await wsRequest(
				roarm.NAMESPACE_ID,
				(requestId) => roarm.DeleteMissionRequest.encode({ requestId, missionIndex }),
				roarm.DeleteMissionResponse,
			);
			return resp.success;
		} catch {
			return false;
		}
	}
}
