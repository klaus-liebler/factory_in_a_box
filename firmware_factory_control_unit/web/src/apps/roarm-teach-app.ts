// RoArm-M3 Teach-Seite: vereinfachtes 3D-Modell (roarm-3d-view.ts) + Gelenk-/Cartesian-Jogging +
// Schritt-Liste (Gelenkbewegung/GPIO/Delay) + Mission speichern/laden. Spricht ausschliesslich
// gegen das RoArmBackend-Interface (roarm-backend.ts) -- zwei Implementierungen, s.
// createBackend() unten: WsRoArmBackend (echte Firmware, s. Core/Src/setup_and_loops/roarm.hh +
// webserver.cpp) im eingebetteten Firmware-Build, MockRoArmBackend im Vite-Dev-Server (kein Board
// noetig, s. "import.meta.env.DEV"-Kommentar dort) -- damit bleibt die Seite weiterhin ohne
// Hardware am PC testbar, waehrend die echte Firmware automatisch die echte Anbindung bekommt.
import { LitElement, html } from "lit";
import { customElement, state, query } from "lit/decorators.js";
import "../styles.css";
import type { DashboardApp } from "../shell/dashboard-app.js";
import { roarm } from "../../generated/ws-protocol.js";
import { createRoArm3DView, type RoArm3DHandles } from "./roarm-3d-view.js";
import { MockRoArmBackend, type RoArmBackend, type MissionStep, VACUUM_GPIO_ID, FOLDED_REST_POSE_CENTIDEG } from "./roarm-backend.js";
import { WsRoArmBackend } from "./roarm-ws-backend.js";
import { JOINT_NAMES, JOINT_COUNT, centiDegToRad, radToCentiDeg } from "./roarm-kinematics.js";

// import.meta.env.DEV ist Vites eingebautes Dev/Prod-Flag (true unter "npm run dev", false im
// echten "vite build", der als index.html.br ins Firmware-Flash eincompiliert wird, s.
// docs/build-process.md) -- kein manuelles Umschalten noetig, jede Build-Art bekommt automatisch
// das fuer sie sinnvolle Backend.
function createBackend(): RoArmBackend {
	return import.meta.env.DEV ? new MockRoArmBackend() : new WsRoArmBackend();
}

const DEFAULT_JOINT_MOVE_SPEED_DEG_PER_SEC = 60;
const DEFAULT_DELAY_MS = 500;
const GPIO_STEP_SIMULATED_DELAY_MS = 300; // Play-Wiedergabe: keine echte Hardware im Teach-Modus, s. playMission()
const JOINT_ARRIVAL_TOLERANCE_CENTIDEG = 50; // Play-Wiedergabe: "angekommen" (0.5 Grad), s. waitForArrival()
const JOINT_ARRIVAL_TIMEOUT_MS = 8000; // Play-Wiedergabe: Sicherheitsabbruch falls nie "angekommen", s. waitForArrival()

function sleep(ms: number): Promise<void> {
	return new Promise((resolve) => setTimeout(resolve, ms));
}

// Kein Greifer an diesem Arm (Vakuumsauger statt Zangen-Greifer) -- der 6. Kanal (JOINT_NAMES[5]
// "Gripper") bleibt im Wire-Protokoll/roarm-kinematics.ts bestehen (keine Protokoll-Aenderung in
// diesem Durchgang, s. Absprache), wird hier aber weder als Jogging-Slider noch als
// Kartesisches-Ziel-Feld angezeigt.
const JOGGABLE_JOINT_COUNT = 5;

// Gelenkwinkel (centiDeg) des naechstgelegenen JointMoveStep AB fromIndex rueckwaerts -- fuers
// Ghost-Overlay: zeigt an, wo der Arm vor der Einfuegeposition (s. insertAfterIndex) zuletzt
// stand. Rueckwaerts statt einfach "letzter Schritt", weil der markierte Einfuegepunkt (Klick auf
// einen Missionsschritt) auf einem Pause- oder Vakuum-Schritt OHNE eigene Pose liegen kann --
// dann muss der Ghost sich auf die letzte Pose DAVOR beziehen, nicht auf den markierten Schritt
// selbst oder das Ende der Liste.
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
export class RoArmTeachApp extends LitElement implements DashboardApp {
	protected createRenderRoot() {
		return this;
	}

	private readonly backend: RoArmBackend = createBackend();
	private view: RoArm3DHandles | null = null;
	private unsubscribePose: (() => void) | null = null;

	@query(".roarm-3d-container") private viewContainer!: HTMLDivElement;

	// Eingeknickte Ruhepose statt voll gestreckt (0/0/.../0) -- deckt sich mit MockRoArmBackends
	// eigenem initialJointsRad-Default (s. dort), so dass die Ansicht nicht kurz die gestreckte
	// Pose aufblitzt, bevor das erste PoseFeedback eintrifft.
	@state() private jointAnglesCentiDeg: number[] = [...FOLDED_REST_POSE_CENTIDEG];
	@state() private pose: roarm.PoseFeedback.Payload = this.backend.getLastPoseFeedback();
	@state() private steps: MissionStep[] = [];
	// Neue Schritte werden HINTER diesem Index eingefuegt (-1 = ganz an den Anfang) -- per Klick auf
	// einen Missionsschritt verschiebbar (s. selectInsertPoint()), nicht zwingend das Ende der
	// Liste. Default: ans Ende anhaengen (wird bei jeder Aenderung von `steps` nachgezogen, s.
	// insertStep()/removeStep()/moveStep()).
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

	// Ghost-Overlay (letzte Pose VOR dem Einfuegepunkt, s. lastJointMoveStepAngles()) reaktiv mit
	// `steps`/`insertAfterIndex` synchron halten -- deckt "Schritt hinzufuegen", Mission laden/
	// Schritt loeschen UND einen neuen Einfuegepunkt anklicken ab, ohne dass jede Stelle, die eines
	// der beiden aendert, das Overlay einzeln nachfuehren muss.
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

	// 3D-Gizmo (roarm-3d-view.ts) -- loest die IK jetzt clientseitig und meldet hier direkt fertige
	// Gelenkwinkel (Base..Roll, rad) statt roher Deltas. Nur Index 5 (Gripper) bleibt unberuehrt --
	// der Vakuum-Sauger-Endeffektor hat keine eigene Gelenkachse.
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
			gripperRad: 0, // kein Greifer an diesem Arm (Vakuumsauger statt Zangen-Greifer), kein Formularfeld dafuer
		});
	}

	// Fuegt hinter insertAfterIndex ein (nicht zwingend am Ende, s. Feld-Kommentar) und ruecht den
	// Einfuegepunkt auf den neuen Schritt nach -- weitere Klicks auf "+"-Buttons ohne zwischenzeitliche
	// Auswahl haengen so weiterhin fortlaufend hintereinander an, statt sich am urspruenglich
	// angeklickten Schritt zu stapeln.
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

	// Einziger Mission-GPIO auf diesem Arm ist der Vakuumsauger (VACUUM_GPIO_ID) -- keine generische
	// Mehr-GPIO-Auswahl noetig, nur ein/aus.
	private addVacuumStep(state: boolean): void {
		this.insertStep({ classId: roarm.GpioStep.CLASS_ID, gpioId: VACUUM_GPIO_ID, state });
	}

	private addDelayStep(): void {
		this.insertStep({ classId: roarm.DelayStep.CLASS_ID, durationMs: this.newDelayMs });
	}

	// Klick auf einen Missionsschritt (oder auf den "Anfang"-Platzhalter, index=-1): dieser Schritt
	// wird zum neuen Einfuegepunkt fuer den naechsten hinzugefuegten Schritt UND der Arm faehrt dort
	// hin (dieselbe Pose, die als Ghost angezeigt wird, s. lastJointMoveStepAngles()) -- so kann man
	// von dieser Stelle aus mit Jogging/Gizmo weiterarbeiten, statt nur optisch zu vergleichen. Bei
	// "Anfang" (index=-1) gibt es keine vorherige Pose, dort bewegt sich der Arm folgerichtig nicht.
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
		// War der Einfuegepunkt auf oder hinter dem geloeschten Schritt, ruecht er um eins nach --
		// zeigt danach auf denselben (jetzt vorgerueckten) Nachbarn wie vorher.
		if (index <= this.insertAfterIndex) this.insertAfterIndex -= 1;
	}

	private moveStep(index: number, delta: number): void {
		const target = index + delta;
		if (target < 0 || target >= this.steps.length) return;
		const next = [...this.steps];
		[next[index], next[target]] = [next[target], next[index]];
		this.steps = next;
		// Einfuegepunkt folgt demselben logischen Schritt durch die Vertauschung, nicht der Position.
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

	// --- Play: fuehrt die aktuelle Mission clientseitig aus, ueber dasselbe RoArmBackend-Interface
	// wie das interaktive Jogging (kein separater "Mission abspielen"-Kanal im Wire-Protokoll noetig).
	// GpioStep hat KEINE eigene Live-Ansteuerung (nur innerhalb einer auf dem Board gespeicherten,
	// dort ausgefuehrten Mission bedeutungsvoll) -- hier daher nur simuliert (Status-Text + kurze
	// Pause), waehrend JointMoveStep/DelayStep echt ausgefuehrt werden (echtes Jogging bzw. echtes
	// Warten).
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
				<section class="header-section app-panel">
					<div class="panel-label">RoArm-M3 Teach</div>
					${this.statusMessage ? html`<div class="status-text status-${this.statusVariant}">${this.statusMessage}</div>` : ""}
				</section>

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
												min="-90"
												max="90"
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
