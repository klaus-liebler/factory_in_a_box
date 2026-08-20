// Echte Firmware-Anbindung fuers RoArmBackend-Interface (roarm-backend.ts) -- ersetzt
// MockRoArmBackend, sobald die Firmware-Seite existiert (WS-Handler in webserver.cpp,
// RoArmSetupAndLoop in Core/Src/setup_and_loops/roarm.hh). Uebersetzt 1:1 auf die generierten
// roarm.*-Nachrichtentypen (web/generated/ws-protocol.ts) ueber die generischen Sende-/
// Anfrage-/Abonnement-Helfer aus ws-client.ts -- kein eigenes Wire-Format-Wissen hier.
import { roarm } from "../../generated/ws-protocol.js";
import { sendRoArmEvent, roarmRequest, subscribeRoArmEvent } from "../ws-client.js";
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

export class WsRoArmBackend implements RoArmBackend {
	private lastPose: roarm.PoseFeedback.Payload = emptyPoseFeedback();
	private readonly poseListeners = new Set<(feedback: roarm.PoseFeedback.Payload) => void>();
	private readonly unsubscribeInternal: () => void;

	constructor() {
		this.unsubscribeInternal = subscribeRoArmEvent(roarm.PoseFeedback.TYPE_ID, (view) => {
			this.lastPose = roarm.PoseFeedback.decode(view, 0);
			for (const cb of this.poseListeners) cb(this.lastPose);
		});
	}

	/** Loest das interne PoseFeedback-Abonnement -- fuer den seltenen Fall, dass eine Seite dieses
	 * Backend nicht mehr braucht (aktuell lebt es fuer die gesamte Seitenlebensdauer, s. roarm-teach-app.ts). */
	dispose(): void {
		this.unsubscribeInternal();
	}

	async startTeachMode(): Promise<boolean> {
		try {
			const resp = await roarmRequest(
				(requestId) => roarm.StartTeachModeRequest.encode({ requestId }),
				(view) => roarm.StartTeachModeResponse.decode(view, 0),
			);
			return resp.success;
		} catch {
			return false;
		}
	}

	async stopTeachMode(): Promise<boolean> {
		try {
			const resp = await roarmRequest(
				(requestId) => roarm.StopTeachModeRequest.encode({ requestId }),
				(view) => roarm.StopTeachModeResponse.decode(view, 0),
			);
			return resp.success;
		} catch {
			return false;
		}
	}

	setJointJogTargetCentiDeg(jointAnglesCentiDeg: readonly number[]): void {
		sendRoArmEvent(roarm.JointJogTarget.encode({ jointAnglesCentiDeg: [...jointAnglesCentiDeg] }));
	}

	setCartesianJogTarget(pose: CartesianPose): void {
		sendRoArmEvent(
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
			const resp = await roarmRequest(
				(requestId) => roarm.GetMissionGpioListRequest.encode({ requestId }),
				(view) => roarm.GetMissionGpioListResponse.decode(view, 0),
			);
			return resp.names;
		} catch {
			return [];
		}
	}

	async listMissions(): Promise<roarm.MissionSummary.Payload[]> {
		try {
			const resp = await roarmRequest(
				(requestId) => roarm.ListMissionsRequest.encode({ requestId }),
				(view) => roarm.ListMissionsResponse.decode(view, 0),
			);
			return resp.missions.map((m) => ({ missionIndex: m.missionIndex, name: m.name }));
		} catch {
			return [];
		}
	}

	async getMission(missionIndex: number): Promise<StoredMission | null> {
		try {
			const resp = await roarmRequest(
				(requestId) => roarm.GetMissionRequest.encode({ requestId, missionIndex }),
				(view) => roarm.GetMissionResponse.decode(view, 0),
			);
			if (!resp.found) return null;
			return { name: resp.name, steps: resp.steps as MissionStep[] };
		} catch {
			return null;
		}
	}

	async saveMission(missionIndex: number, name: string, steps: MissionStep[]): Promise<{ success: boolean; errorCode: number }> {
		try {
			const resp = await roarmRequest(
				(requestId) => roarm.SaveMissionRequest.encode({ requestId, missionIndex, name, steps }),
				(view) => roarm.SaveMissionResponse.decode(view, 0),
			);
			return { success: resp.success, errorCode: resp.errorCode };
		} catch {
			return { success: false, errorCode: -1 };
		}
	}

	async deleteMission(missionIndex: number): Promise<boolean> {
		try {
			const resp = await roarmRequest(
				(requestId) => roarm.DeleteMissionRequest.encode({ requestId, missionIndex }),
				(view) => roarm.DeleteMissionResponse.decode(view, 0),
			);
			return resp.success;
		} catch {
			return false;
		}
	}
}
