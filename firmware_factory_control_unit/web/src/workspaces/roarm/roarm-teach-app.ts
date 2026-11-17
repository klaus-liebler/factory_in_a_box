// RoArm-M3 teach workspace: simplified 3D model (roarm-3d-view.ts) + joint/cartesian jogging + step list
// (joint move/GPIO/delay) + save/load mission. Talks only to the RoArmBackend interface (roarm-backend.ts) -
// two implementations, see createBackend() below: WsRoArmBackend (real firmware, see src/roarm/roarm.cpp +
// src/web.cpp) in the embedded firmware build, MockRoArmBackend in the Vite dev server (no board needed, see
// the "import.meta.env.DEV" comment there) - so the workspace stays testable on the PC without hardware,
// while the real firmware automatically gets the real connection.
import { LitElement, html } from "lit";
import { customElement, state, query } from "lit/decorators.js";
import "./roarm-teach.css";
import type { WorkspaceApp } from "@klaus-liebler/ui-shell/shell/workspace-app.ts";
import { roarm } from "../../../generated/ws-protocol.ts";
import { createRoArm3DView, type RoArm3DHandles } from "./roarm-3d-view.ts";
import { MockRoArmBackend, type RoArmBackend, type MissionStep, VACUUM_GPIO_ID, FOLDED_REST_POSE_CENTIDEG } from "./roarm-backend.ts";
import { WsRoArmBackend } from "./roarm-ws-backend.ts";
import { JOINT_NAMES, JOINT_COUNT, JOINT_LIMITS_RAD, centiDegToRad, radToCentiDeg } from "./roarm-kinematics.ts";
import { kindOf, setAppStatus } from "@klaus-liebler/ui-shell/shell/status-bar.ts";
import { setAppRibbon } from "@klaus-liebler/ui-shell/shell/ribbon.ts";

// import.meta.env.DEV is Vite's built-in dev/prod flag (true under "npm run dev", false in the real "vite
// build" that is compiled into the firmware flash as index.html.br, see docs/build-process.md) - no manual
// switching needed, every kind of build automatically gets the backend that makes sense for it.
function createBackend(): RoArmBackend {
	return import.meta.env.DEV ? new MockRoArmBackend() : new WsRoArmBackend();
}

const DEFAULT_JOINT_MOVE_SPEED_DEG_PER_SEC = 60;
const DEFAULT_DELAY_MS = 500;
const GPIO_STEP_SIMULATED_DELAY_MS = 300; // play: no real hardware in teach mode, see playMission()
const JOINT_ARRIVAL_TOLERANCE_CENTIDEG = 50; // Play-Wiedergabe: "angekommen" (0.5 Grad), s. waitForArrival()
const JOINT_ARRIVAL_TIMEOUT_MS = 8000; // Play-Wiedergabe: Sicherheitsabbruch falls nie "angekommen", s. waitForArrival()

function sleep(ms: number): Promise<void> {
	return new Promise((resolve) => setTimeout(resolve, ms));
}

// No gripper on this arm (vacuum suction cup instead of a gripper) - the 6th channel (JOINT_NAMES[5]
// "Gripper") stays in the wire protocol/roarm-kinematics.ts (no protocol change in this round, as agreed),
// but is shown neither as a jogging slider nor as a cartesian target field here.
const JOGGABLE_JOINT_COUNT = 5;

// Joint angles (centiDeg) of the nearest JointMoveStep FROM fromIndex backwards - for the ghost overlay:
// shows where the arm last stood before the insert position (see insertAfterIndex). Backwards instead of
// simply "last step", because the marked insert point (click on a mission step) can lie on a delay or vacuum
// step WITHOUT a pose of its own - then the ghost must refer to the last pose BEFORE it, not to the marked
// step itself or the end of the list.
function lastJointMoveStepAngles(steps: readonly MissionStep[], fromIndex: number): number[] | null {
	for (let i = Math.min(fromIndex, steps.length - 1); i >= 0; i--) {
		const step = steps[i];
		if (step.classId === roarm.JointMoveStep.CLASS_ID) return [...step.jointAnglesCentiDeg];
	}
	return null;
}

function stepSummary(step: MissionStep, gpioNames: readonly string[]): string {
	switch (step.classId) {
		case roarm.JointMoveStep.CLASS_ID: {
			const degs = step.jointAnglesCentiDeg.map((c) => (c / 100).toFixed(0)).join(" / ");
			const kind = step.isWaypoint ? "Stützstelle" : "Exakte Pose";
			return `${kind}: ${degs}° @ ${step.maxSpeedDegPerSec}°/s`;
		}
		case roarm.GpioStep.CLASS_ID: {
			const name = gpioNames[step.gpioId] ?? `GPIO ${step.gpioId}`;
			return `${name} ${step.state ? "EIN" : "AUS"}`;
		}
		case roarm.DelayStep.CLASS_ID:
			return `Warten ${step.durationMs} ms`;
	}
}

@customElement("roarm-teach-app")
export class RoArmTeachApp extends LitElement implements WorkspaceApp {
	/** The state goes to the application's status bar (shell/status-bar.ts), not into the page. */
	protected willUpdate(): void {
		setAppStatus(this, { text: this.statusMessage, kind: kindOf(this.statusVariant) });
		const playing = this.isPlaying;
		setAppRibbon(this, [
			{ id: "mission", label: "Mission", groups: [
				{ label: "Datei", commands: [
					{ id: "open", label: "Laden", icon: "open", disabled: playing, title: "Eine gespeicherte Mission laden", run: () => this.openLoadDialog() },
					{ id: "save", label: "Speichern", icon: "save", disabled: playing || this.steps.length === 0, title: "Die Mission auf dem Gerät speichern", run: () => this.openSaveDialog() },
				] },
				{ label: "Ablauf", commands: [
					{ id: "play", label: "Abspielen", icon: "run", disabled: playing || this.steps.length === 0, run: () => void this.playMission() },
					{ id: "stop", label: "Stopp", icon: "stop", disabled: !playing, run: () => this.stopPlayback() },
				] },
				{ label: "Schritte", commands: [
					{ id: "pose", label: "Pose", icon: "pose", disabled: playing, title: "Die aktuelle Pose als Schritt einfügen", run: () => this.addCurrentPoseAsStep(false) },
					{ id: "waypoint", label: "Wegpunkt", icon: "pose", disabled: playing, title: "Die aktuelle Pose als Wegpunkt (durchfahren, nicht anhalten)", run: () => this.addCurrentPoseAsStep(true) },
					{ id: "vac-on", label: "Vakuum ein", icon: "vacuum", size: "small", disabled: playing, run: () => this.addVacuumStep(true) },
					{ id: "vac-off", label: "Vakuum aus", icon: "vacuum-off", size: "small", disabled: playing, run: () => this.addVacuumStep(false) },
					{ id: "delay", label: `Warten ${this.newDelayMs} ms`, icon: "delay", size: "small", disabled: playing, run: () => this.addDelayStep() },
				] },
			] },
		]);
	}

	protected createRenderRoot() {
		return this;
	}

	private readonly backend: RoArmBackend = createBackend();
	private view: RoArm3DHandles | null = null;
	private unsubscribePose: (() => void) | null = null;

	@query(".roarm-3d-container") private viewContainer!: HTMLDivElement;

	// Folded rest pose instead of fully stretched (0/0/.../0) - matches MockRoArmBackend's own initialJointsRad
	// default (see there), so that the view does not briefly flash the stretched pose before the first
	// PoseFeedback arrives.
	@state() private jointAnglesCentiDeg: number[] = [...FOLDED_REST_POSE_CENTIDEG];
	@state() private pose: roarm.PoseFeedback.Payload = this.backend.getLastPoseFeedback();
	@state() private steps: MissionStep[] = [];
	// New steps are inserted AFTER this index (-1 = at the very beginning) - movable by clicking a mission step
	// (see selectInsertPoint()), not necessarily the end of the list. Default: append at the end (kept up to
	// date on every change of `steps`, see insertStep()/removeStep()/moveStep()).
	@state() private insertAfterIndex = -1;
	@state() private missionIndex = 1;
	@state() private missionName = "";
	@state() private missionList: roarm.MissionSummary.Payload[] = [];
	@state() private gpioNames: string[] = [];
	@state() private newDelayMs = DEFAULT_DELAY_MS;
	@state() private statusMessage = "";
	@state() private statusVariant: "warning" | "success" | "error" = "warning";
	@state() private dialogMode: "closed" | "open" | "save" = "closed";
	@state() private dialogMissionIndex = 1;
	@state() private dialogMissionName = "";
	@state() private isPlaying = false;
	private playbackCancelled = false;

	onShow(): void {
		this.ensureView();
		this.unsubscribePose = this.backend.subscribePoseFeedback((feedback) => {
			this.pose = feedback;
			this.view?.setJointAnglesRad(feedback.jointAnglesCentiDeg.map(centiDegToRad));
		});
		void this.backend.startTeachMode();
		void this.refreshMissionGpioNames();
		void this.refreshMissionList();
	}

	onHide(): void {
		this.unsubscribePose?.();
		this.unsubscribePose = null;
		void this.backend.stopTeachMode();
	}

	private ensureView(): void {
		if (this.view || !this.viewContainer) return;
		this.view = createRoArm3DView(this.viewContainer, {
			onJointAnglesPreview: (armAnglesRad) => this.onGizmoJointAnglesPreview(armAnglesRad),
		});
		this.view.setJointAnglesRad(this.jointAnglesCentiDeg.map(centiDegToRad));
		this.view.setGhostAnglesRad(lastJointMoveStepAngles(this.steps, this.insertAfterIndex)?.map(centiDegToRad) ?? null);
	}

	protected firstUpdated(): void {
		this.ensureView();
	}

	// Keep the ghost overlay (last pose BEFORE the insert point, see lastJointMoveStepAngles()) reactively in
	// sync with `steps`/`insertAfterIndex` - covers "add step", load mission/delete step AND clicking a new
	// insert point, without every place that changes one of the two having to update the overlay itself.
	protected updated(changedProperties: Map<string, unknown>): void {
		if (changedProperties.has("steps") || changedProperties.has("insertAfterIndex")) {
			this.view?.setGhostAnglesRad(lastJointMoveStepAngles(this.steps, this.insertAfterIndex)?.map(centiDegToRad) ?? null);
		}
	}

	private setStatus(message: string, variant: "warning" | "success" | "error"): void {
		this.statusMessage = message;
		this.statusVariant = variant;
	}

	private onJointSliderInput(jointIndex: number, valueDeg: number): void {
		const next = [...this.jointAnglesCentiDeg];
		next[jointIndex] = Math.round(valueDeg * 100);
		this.jointAnglesCentiDeg = next;
		this.backend.setJointJogTargetCentiDeg(next);
	}

	// 3D gizmo (roarm-3d-view.ts) - now solves the IK on the client and reports finished joint angles here
	// (Base..Roll, rad) instead of raw deltas. Only index 5 (gripper) stays untouched - the vacuum cup end
	// effector has no joint axis of its own.
	private onGizmoJointAnglesPreview(armAnglesRad: readonly number[]): void {
		const next = [...this.jointAnglesCentiDeg];
		for (let i = 0; i < armAnglesRad.length; i++) next[i] = radToCentiDeg(armAnglesRad[i]);
		this.jointAnglesCentiDeg = next;
		this.backend.setJointJogTargetCentiDeg(next);
	}

	private onCartesianApply(form: HTMLFormElement): void {
		const data = new FormData(form);
		const num = (name: string) => Number(data.get(name) ?? 0);
		this.backend.setCartesianJogTarget({
			xMm: num("x"),
			yMm: num("y"),
			zMm: num("z"),
			pitchRad: centiDegToRad(num("pitch") * 100),
			rollRad: centiDegToRad(num("roll") * 100),
			gripperRad: 0, // no gripper on this arm (vacuum cup instead of a gripper), no form field for it
		});
	}

	// Inserts after insertAfterIndex (not necessarily at the end, see the field comment) and moves the insert
	// point on to the new step - further clicks on "+" buttons without a selection in between thus keep
	// appending one after another instead of piling up at the step clicked originally.
	private insertStep(step: MissionStep): void {
		const at = this.insertAfterIndex + 1;
		this.steps = [...this.steps.slice(0, at), step, ...this.steps.slice(at)];
		this.insertAfterIndex = at;
	}

	private addCurrentPoseAsStep(isWaypoint: boolean): void {
		this.insertStep({
			classId: roarm.JointMoveStep.CLASS_ID,
			jointAnglesCentiDeg: [...this.pose.jointAnglesCentiDeg],
			maxSpeedDegPerSec: DEFAULT_JOINT_MOVE_SPEED_DEG_PER_SEC,
			isWaypoint,
		});
	}

	// The only mission GPIO on this arm is the vacuum cup (VACUUM_GPIO_ID) - no generic multi-GPIO selection
	// needed, only on/off.
	private addVacuumStep(state: boolean): void {
		this.insertStep({ classId: roarm.GpioStep.CLASS_ID, gpioId: VACUUM_GPIO_ID, state });
	}

	private addDelayStep(): void {
		this.insertStep({ classId: roarm.DelayStep.CLASS_ID, durationMs: this.newDelayMs });
	}

	// Click on a mission step (or on the "Anfang" placeholder, index=-1): this step becomes the new insert point
	// for the next added step AND the arm moves there (the same pose shown as the ghost, see
	// lastJointMoveStepAngles()) - so one can continue from there with jogging/gizmo instead of only comparing
	// visually. At "Anfang" (index=-1) there is no previous pose, so the arm consistently does not move.
	private selectInsertPoint(index: number): void {
		this.insertAfterIndex = index;
		const angles = lastJointMoveStepAngles(this.steps, index);
		if (angles) {
			this.jointAnglesCentiDeg = angles;
			this.backend.setJointJogTargetCentiDeg(angles);
		}
	}

	private removeStep(index: number): void {
		this.steps = this.steps.filter((_, i) => i !== index);
		// If the insert point was on or after the deleted step, it moves up by one - afterwards it points to the
		// same (now moved up) neighbour as before.
		if (index <= this.insertAfterIndex) this.insertAfterIndex -= 1;
	}

	private moveStep(index: number, delta: number): void {
		const target = index + delta;
		if (target < 0 || target >= this.steps.length) return;
		const next = [...this.steps];
		[next[index], next[target]] = [next[target], next[index]];
		this.steps = next;
		// The insert point follows the same logical step through the swap, not the position.
		if (this.insertAfterIndex === index) this.insertAfterIndex = target;
		else if (this.insertAfterIndex === target) this.insertAfterIndex = index;
	}

	private async refreshMissionGpioNames(): Promise<void> {
		this.gpioNames = await this.backend.getMissionGpioNames();
	}

	private async refreshMissionList(): Promise<void> {
		this.missionList = await this.backend.listMissions();
	}

	private async saveMission(missionIndex: number, name: string): Promise<void> {
		if (missionIndex <= 0) {
			this.setStatus("Mission-Index muss > 0 sein", "error");
			return;
		}
		const result = await this.backend.saveMission(missionIndex, name || `Mission ${missionIndex}`, this.steps);
		if (result.success) {
			this.missionIndex = missionIndex;
			this.missionName = name;
			this.setStatus(`Mission ${missionIndex} gespeichert`, "success");
			void this.refreshMissionList();
		} else {
			this.setStatus(`Speichern fehlgeschlagen (Fehlercode ${result.errorCode})`, "error");
		}
	}

	private async loadMission(missionIndex: number): Promise<void> {
		const mission = await this.backend.getMission(missionIndex);
		if (!mission) {
			this.setStatus(`Mission ${missionIndex} nicht gefunden`, "error");
			return;
		}
		this.missionIndex = missionIndex;
		this.missionName = mission.name;
		this.steps = mission.steps;
		this.insertAfterIndex = mission.steps.length - 1; // frisch geladen: Einfuegepunkt ans Ende
		this.setStatus(`Mission ${missionIndex} geladen`, "success");
	}

	private async deleteMission(missionIndex: number): Promise<void> {
		await this.backend.deleteMission(missionIndex);
		void this.refreshMissionList();
	}

	// --- Speichern-/Oeffnen-Dialog (an klassische Datei-Dialoge angelehnt: Liste vorhandener
	// Missionen + Index/Name-Feld, s. render()) -------------------------------------------------
	private openSaveDialog(): void {
		void this.refreshMissionList();
		this.dialogMissionIndex = this.missionIndex;
		this.dialogMissionName = this.missionName;
		this.dialogMode = "save";
	}

	private openLoadDialog(): void {
		void this.refreshMissionList();
		this.dialogMissionIndex = this.missionList[0]?.missionIndex ?? this.missionIndex;
		this.dialogMode = "open";
	}

	private closeDialog(): void {
		this.dialogMode = "closed";
	}

	private selectDialogMission(m: roarm.MissionSummary.Payload): void {
		this.dialogMissionIndex = m.missionIndex;
		if (this.dialogMode === "save") this.dialogMissionName = m.name; // Ueberschreiben vorbereiten
	}

	private async deleteDialogMission(missionIndex: number): Promise<void> {
		await this.deleteMission(missionIndex);
	}

	private async confirmDialog(): Promise<void> {
		if (this.dialogMode === "save") {
			await this.saveMission(this.dialogMissionIndex, this.dialogMissionName);
		} else if (this.dialogMode === "open") {
			await this.loadMission(this.dialogMissionIndex);
		}
		this.dialogMode = "closed";
	}

	// --- Play: runs the current mission on the client, via the same RoArmBackend interface as interactive
	// jogging (no separate "play mission" channel needed in the wire protocol). GpioStep has NO live control of
	// its own (only meaningful within a mission stored and run on the board) - so it is only simulated here
	// (status text + short pause), while JointMoveStep/DelayStep are really executed (real jogging and real
	// waiting).
	private async playMission(): Promise<void> {
		if (this.steps.length === 0 || this.isPlaying) return;
		this.isPlaying = true;
		this.playbackCancelled = false;
		this.setStatus("Mission wird abgespielt …", "warning");
		for (const step of this.steps) {
			if (this.playbackCancelled) break;
			if (step.classId === roarm.JointMoveStep.CLASS_ID) {
				this.backend.setJointMoveTargetCentiDeg(step.jointAnglesCentiDeg, step.maxSpeedDegPerSec);
				await this.waitForArrival(step.jointAnglesCentiDeg);
			} else if (step.classId === roarm.GpioStep.CLASS_ID) {
				const name = this.gpioNames[step.gpioId] ?? `GPIO ${step.gpioId}`;
				this.setStatus(`${name} ${step.state ? "EIN" : "AUS"} (simuliert -- keine Live-GPIO-Ansteuerung im Teach-Modus)`, "warning");
				await sleep(GPIO_STEP_SIMULATED_DELAY_MS);
			} else if (step.classId === roarm.DelayStep.CLASS_ID) {
				await sleep(step.durationMs);
			}
		}
		this.isPlaying = false;
		if (!this.playbackCancelled) this.setStatus("Mission-Wiedergabe abgeschlossen", "success");
	}

	private stopPlayback(): void {
		this.playbackCancelled = true;
	}

	private waitForArrival(targetCentiDeg: readonly number[]): Promise<void> {
		return new Promise((resolve) => {
			const start = performance.now();
			const check = () => {
				if (this.playbackCancelled) return resolve();
				const current = this.pose.jointAnglesCentiDeg;
				const arrived = targetCentiDeg.every((t, i) => Math.abs((current[i] ?? 0) - t) <= JOINT_ARRIVAL_TOLERANCE_CENTIDEG);
				if (arrived || performance.now() - start > JOINT_ARRIVAL_TIMEOUT_MS) return resolve();
				requestAnimationFrame(check);
			};
			check();
		});
	}

	render() {
		return html`
			<div class="container roarm-container">

				<div class="roarm-layout">
					<div class="panel-section roarm-controls-panel">
						<div class="roarm-pose-readout">
							Pose: X=${this.pose.xMm}mm Y=${this.pose.yMm}mm Z=${this.pose.zMm}mm
							Pitch=${(this.pose.pitchCentiDeg / 100).toFixed(0)}° Roll=${(this.pose.rollCentiDeg / 100).toFixed(0)}°
						</div>

						<details class="roarm-collapsible" open>
							<summary class="roarm-collapsible-summary">Gelenke (Jogging)</summary>
							<div class="roarm-collapsible-body">
								${JOINT_NAMES.slice(0, JOGGABLE_JOINT_COUNT).map(
									(name, i) => html`
										<div class="roarm-slider-row">
											<span class="roarm-slider-label">${name}</span>
											<input
												type="range"
												min=${Math.ceil((JOINT_LIMITS_RAD[i][0] * 180) / Math.PI)}
												max=${Math.floor((JOINT_LIMITS_RAD[i][1] * 180) / Math.PI)}
												step="1"
												.value=${(this.jointAnglesCentiDeg[i] / 100).toString()}
												@input=${(e: Event) => this.onJointSliderInput(i, Number((e.target as HTMLInputElement).value))}
											/>
											<span class="register-slider-value">${(this.jointAnglesCentiDeg[i] / 100).toFixed(0)}°</span>
										</div>
									`,
								)}
							</div>
						</details>

						<details class="roarm-collapsible" open>
							<summary class="roarm-collapsible-summary">Kartesisches Ziel</summary>
							<div class="roarm-collapsible-body">
								<form
									class="roarm-cartesian-form"
									@submit=${(e: SubmitEvent) => {
										e.preventDefault();
										this.onCartesianApply(e.target as HTMLFormElement);
									}}
								>
									<label>X<input class="panel-input" type="number" name="x" value="200" /></label>
									<label>Y<input class="panel-input" type="number" name="y" value="0" /></label>
									<label>Z<input class="panel-input" type="number" name="z" value="150" /></label>
									<label>Pitch<input class="panel-input" type="number" name="pitch" value="0" /></label>
									<label>Roll<input class="panel-input" type="number" name="roll" value="0" /></label>
									<button type="submit">Anfahren</button>
								</form>
							</div>
						</details>

						<details class="roarm-collapsible roarm-mission-collapsible" open>
							<summary class="roarm-collapsible-summary">Missionsplanung</summary>
							<div class="roarm-collapsible-body">
								<div class="roarm-mission-toolbar">
									<button @click=${() => this.openSaveDialog()}>Speichern</button>
									<button @click=${() => this.openLoadDialog()}>Öffnen</button>
									${this.isPlaying
										? html`<button @click=${() => this.stopPlayback()}>Stop</button>`
										: html`<button ?disabled=${this.steps.length === 0} @click=${() => this.playMission()}>Play</button>`}
								</div>

								<div class="panel-label">Schritte hinzufügen</div>
								<div class="roarm-addstep-row">
									<button class="roarm-addstep-btn" @click=${() => this.addCurrentPoseAsStep(false)}>Exakte Pose</button>
									<span class="roarm-addstep-data">${this.pose.xMm} / ${this.pose.yMm} / ${this.pose.zMm} mm</span>
								</div>
								<div class="roarm-addstep-row">
									<button class="roarm-addstep-btn" @click=${() => this.addCurrentPoseAsStep(true)}>Stützstelle</button>
									<span class="roarm-addstep-data">${this.pose.xMm} / ${this.pose.yMm} / ${this.pose.zMm} mm</span>
								</div>
								<div class="roarm-addstep-row">
									<button class="roarm-addstep-btn" @click=${() => this.addVacuumStep(true)}>Vakuum ein</button>
									<span class="roarm-addstep-data">–</span>
								</div>
								<div class="roarm-addstep-row">
									<button class="roarm-addstep-btn" @click=${() => this.addVacuumStep(false)}>Vakuum aus</button>
									<span class="roarm-addstep-data">–</span>
								</div>
								<div class="roarm-addstep-row">
									<button class="roarm-addstep-btn" @click=${() => this.addDelayStep()}>Pause</button>
									<input
										class="panel-input roarm-addstep-data"
										type="number"
										min="0"
										.value=${this.newDelayMs.toString()}
										@input=${(e: Event) => (this.newDelayMs = Number((e.target as HTMLInputElement).value))}
									/>
								</div>

								<div class="panel-label">Aktuelle Mission</div>
								${this.steps.length === 0
									? html`<div class="roarm-empty-hint">Diese Mission enthält noch keine Schritte.</div>`
									: html`
											<ol class="roarm-mission-list">
												<li class="roarm-mission-cursor-slot ${this.insertAfterIndex === -1 ? "is-cursor" : ""}" @click=${() => this.selectInsertPoint(-1)}>▸ Anfang</li>
												${this.steps.map(
													(step, i) => html`
														<li class="roarm-mission-step ${this.insertAfterIndex === i ? "is-cursor" : ""}" @click=${() => this.selectInsertPoint(i)}>
															<span>${stepSummary(step, this.gpioNames)}</span>
															<span class="roarm-step-row-actions">
																<button @click=${(e: Event) => { e.stopPropagation(); this.moveStep(i, -1); }}>↑</button>
																<button @click=${(e: Event) => { e.stopPropagation(); this.moveStep(i, 1); }}>↓</button>
																<button @click=${(e: Event) => { e.stopPropagation(); this.removeStep(i); }}>✕</button>
															</span>
														</li>
													`,
												)}
											</ol>
										`}
							</div>
						</details>
					</div>

					<div class="roarm-view-panel">
						<div class="roarm-3d-container"></div>
					</div>
				</div>

				${this.dialogMode !== "closed"
					? html`
							<div class="roarm-modal-backdrop" @click=${() => this.closeDialog()}>
								<div class="roarm-modal" @click=${(e: Event) => e.stopPropagation()}>
									<div class="roarm-modal-title">${this.dialogMode === "save" ? "Mission speichern" : "Mission öffnen"}</div>
									<ul class="roarm-modal-list">
										${this.missionList.length === 0 ? html`<li class="roarm-modal-list-empty">Keine gespeicherten Missionen</li>` : ""}
										${this.missionList.map(
											(m) => html`
												<li
													class="roarm-modal-list-item ${this.dialogMissionIndex === m.missionIndex ? "is-selected" : ""}"
													@click=${() => this.selectDialogMission(m)}
													@dblclick=${() => this.confirmDialog()}
												>
													<span>#${m.missionIndex} — ${m.name}</span>
													<button @click=${(e: Event) => { e.stopPropagation(); this.deleteDialogMission(m.missionIndex); }}>Löschen</button>
												</li>
											`,
										)}
									</ul>
									<div class="roarm-modal-fields">
										<label
											>Index<input
												class="panel-input"
												type="number"
												min="1"
												.value=${this.dialogMissionIndex.toString()}
												@input=${(e: Event) => (this.dialogMissionIndex = Number((e.target as HTMLInputElement).value))}
										/></label>
										${this.dialogMode === "save"
											? html`<label
													>Name<input
														class="panel-input"
														type="text"
														placeholder="Name"
														.value=${this.dialogMissionName}
														@input=${(e: Event) => (this.dialogMissionName = (e.target as HTMLInputElement).value)}
											/></label>`
											: ""}
									</div>
									<div class="roarm-modal-actions">
										<button @click=${() => this.closeDialog()}>Abbrechen</button>
										<button @click=${() => this.confirmDialog()}>${this.dialogMode === "save" ? "Speichern" : "Öffnen"}</button>
									</div>
								</div>
							</div>
						`
					: ""}
			</div>
		`;
	}
}
