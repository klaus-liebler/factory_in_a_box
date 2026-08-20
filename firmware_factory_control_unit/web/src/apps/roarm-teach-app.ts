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
import { createRoArm3DView, SCENE_UNITS_PER_MM, type RoArm3DHandles, type HandleId } from "./roarm-3d-view.js";
import { MockRoArmBackend, type RoArmBackend, type MissionStep } from "./roarm-backend.js";
import { WsRoArmBackend } from "./roarm-ws-backend.js";
import { JOINT_NAMES, JOINT_COUNT, JOINT_LIMITS_RAD, centiDegToRad, radToCentiDeg } from "./roarm-kinematics.js";

// import.meta.env.DEV ist Vites eingebautes Dev/Prod-Flag (true unter "npm run dev", false im
// echten "vite build", der als index.html.br ins Firmware-Flash eincompiliert wird, s.
// docs/build-process.md) -- kein manuelles Umschalten noetig, jede Build-Art bekommt automatisch
// das fuer sie sinnvolle Backend.
function createBackend(): RoArmBackend {
	return import.meta.env.DEV ? new MockRoArmBackend() : new WsRoArmBackend();
}

const DEFAULT_JOINT_MOVE_SPEED_DEG_PER_SEC = 60;
const DEFAULT_DELAY_MS = 500;

// Umrechnung eines Kopf-Anfasser-Drags (Szeneneinheiten aus roarm-3d-view.ts, Y-up/Basis dreht um
// Y) in ein kartesisches mm-Delta im Koordinatensystem von roarm-kinematics.ts (Z-up/Basis dreht
// um den aequivalenten Z-Winkel). Achsen-Permutation+Vorzeichen empirisch ermittelt (isolierte
// Shoulder- bzw. Base-Rotationstests: rollJoint-Weltposition in Szeneneinheiten gegen
// forwardKinematics()-Ausgabe verglichen) -- OGL-Y (Hoehe) <-> kinematics Z (gleiches Vorzeichen),
// OGL-Z <-> kinematics Y (gleiches Vorzeichen), OGL-X <-> kinematics X ABER VORZEICHENVERKEHRT.
// Die Betraege stimmen dabei nur naeherungsweise (ca. 10-15% Abweichung): das vereinfachte 3D-
// Modell bildet die kleinen, konstanten Knick-Winkel T2RAD/T3RAD/TERAD der echten Mechanik
// (roarm-kinematics.ts) absichtlich NICHT nach (s. Kommentar in roarm-3d-view.ts) -- fuers
// Ziehen des Kopf-Anfassers ausreichend, da die tatsaechliche Zielposition ohnehin ueber IK in
// der (Mock-)Firmware entsteht und per PoseFeedback zurueckgemeldet wird; der Drag ist nur die
// Eingabemethode, nicht die Autoritaet ueber die exakte Position.
function sceneDeltaToKinematicsMm(d: { x: number; y: number; z: number }): { dxMm: number; dyMm: number; dzMm: number } {
	const mmPerSceneUnit = 1 / SCENE_UNITS_PER_MM;
	return { dxMm: -d.x * mmPerSceneUnit, dyMm: d.z * mmPerSceneUnit, dzMm: d.y * mmPerSceneUnit };
}

function stepSummary(step: MissionStep, gpioNames: readonly string[]): string {
	switch (step.classId) {
		case roarm.JointMoveStep.CLASS_ID: {
			const degs = step.jointAnglesCentiDeg.map((c) => (c / 100).toFixed(0)).join(" / ");
			return `Gelenkbewegung: ${degs}° @ ${step.maxSpeedDegPerSec}°/s`;
		}
		case roarm.GpioStep.CLASS_ID: {
			const name = gpioNames[step.gpioId] ?? `GPIO ${step.gpioId}`;
			return `GPIO "${name}" -> ${step.state ? "EIN" : "AUS"}`;
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
	private readonly onResize = () => this.view?.resize();

	// Schnappschuss bei Drag-Beginn (s. roarm-3d-view.ts' onDragStart) -- die Anfasser melden
	// Deltas KUMULIERT seit Drag-Start, damit hier ohne Rundungsdrift immer "Startwert + Delta"
	// gerechnet werden kann, statt viele kleine Deltas aufzuaddieren.
	private dragStartJointAnglesCentiDeg: number[] | null = null;
	private dragStartHeadPose: { xMm: number; yMm: number; zMm: number; pitchRad: number; rollRad: number; gripperRad: number } | null = null;

	@query(".roarm-3d-container") private viewContainer!: HTMLDivElement;

	@state() private jointAnglesCentiDeg: number[] = new Array(JOINT_COUNT).fill(0);
	@state() private pose: roarm.PoseFeedback.Payload = this.backend.getLastPoseFeedback();
	@state() private steps: MissionStep[] = [];
	@state() private missionIndex = 1;
	@state() private missionName = "";
	@state() private missionList: roarm.MissionSummary.Payload[] = [];
	@state() private gpioNames: string[] = [];
	@state() private newGpioId = 0;
	@state() private newGpioState = true;
	@state() private newDelayMs = DEFAULT_DELAY_MS;
	@state() private statusMessage = "";
	@state() private statusVariant: "warning" | "success" | "error" = "warning";

	onShow(): void {
		this.ensureView();
		window.addEventListener("resize", this.onResize);
		this.unsubscribePose = this.backend.subscribePoseFeedback((feedback) => {
			this.pose = feedback;
			this.view?.setJointAnglesRad(feedback.jointAnglesCentiDeg.map(centiDegToRad));
		});
		void this.backend.startTeachMode();
		void this.refreshMissionGpioNames();
		void this.refreshMissionList();
	}

	onHide(): void {
		window.removeEventListener("resize", this.onResize);
		this.unsubscribePose?.();
		this.unsubscribePose = null;
		void this.backend.stopTeachMode();
	}

	private ensureView(): void {
		if (this.view || !this.viewContainer) return;
		this.view = createRoArm3DView(this.viewContainer, {
			onDragStart: (handle: HandleId) => this.onHandleDragStart(handle),
			onJointDrag: (jointIndex, deltaRad) => this.onHandleJointDrag(jointIndex, deltaRad),
			onHeadDrag: (delta) => this.onHandleHeadDrag(delta),
			onHeadOrientationDrag: (axis, deltaRad) => this.onHandleHeadOrientationDrag(axis, deltaRad),
			onDragEnd: () => this.onHandleDragEnd(),
		});
		this.view.setJointAnglesRad(this.jointAnglesCentiDeg.map(centiDegToRad));
	}

	protected firstUpdated(): void {
		this.ensureView();
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

	// --- 3D-Anfasser (roarm-3d-view.ts) -- Gelenk-Drag dreht ein einzelnes Gelenk, Kopf-Drag
	// verschiebt das kartesische Ziel. Beide melden ihr Delta kumuliert seit Drag-Start (s. dortiger
	// Kommentar), daher hier ein Schnappschuss bei onDragStart statt fortlaufender Aufsummierung.
	private onHandleDragStart(handle: HandleId): void {
		// this.jointAnglesCentiDeg spiegelt normalerweise nur Slider-/Gelenk-Anfasser-Eingaben --
		// nach einem rein kartesischen Kopf-Drag (Positions-Pfeile) waere es sonst veraltet
		// (IK-bedingte Gelenkaenderungen laufen NICHT darueber). Vor jedem neuen Drag daher erst
		// mit der zuletzt gemeldeten Ist-Pose abgleichen, damit ein direkt darauf folgender
		// Gelenk-Ring-Drag nicht auf einen Stand VOR dem letzten Kopf-Drag zurueckfaellt.
		this.jointAnglesCentiDeg = [...this.pose.jointAnglesCentiDeg];
		this.dragStartJointAnglesCentiDeg = [...this.jointAnglesCentiDeg];
		this.dragStartHeadPose = {
			xMm: this.pose.xMm,
			yMm: this.pose.yMm,
			zMm: this.pose.zMm,
			pitchRad: centiDegToRad(this.pose.pitchCentiDeg),
			rollRad: centiDegToRad(this.pose.rollCentiDeg),
			gripperRad: centiDegToRad(this.jointAnglesCentiDeg[5]),
		};
		void handle;
	}

	private onHandleJointDrag(jointIndex: number, deltaRad: number): void {
		if (!this.dragStartJointAnglesCentiDeg) return;
		const startRad = centiDegToRad(this.dragStartJointAnglesCentiDeg[jointIndex]);
		const [min, max] = JOINT_LIMITS_RAD[jointIndex];
		const nextRad = Math.min(max, Math.max(min, startRad + deltaRad));
		const next = [...this.jointAnglesCentiDeg];
		next[jointIndex] = radToCentiDeg(nextRad);
		this.jointAnglesCentiDeg = next;
		this.backend.setJointJogTargetCentiDeg(next);
	}

	private onHandleHeadDrag(deltaSceneUnits: { x: number; y: number; z: number }): void {
		if (!this.dragStartHeadPose) return;
		const { dxMm, dyMm, dzMm } = sceneDeltaToKinematicsMm(deltaSceneUnits);
		const start = this.dragStartHeadPose;
		this.backend.setCartesianJogTarget({
			xMm: start.xMm + dxMm,
			yMm: start.yMm + dyMm,
			zMm: start.zMm + dzMm,
			pitchRad: start.pitchRad,
			rollRad: start.rollRad,
			gripperRad: start.gripperRad,
		});
	}

	// Pitch-Ring am Kopf -- "Roll" am Kopf kommt stattdessen ueber onHandleJointDrag(4, ...) rein
	// (derselbe Rollgelenk-Freiheitsgrad, nur ein zweiter Anfasser, s. roarm-3d-view.ts-Kommentar).
	// Nur X/Y/Z UND Roll bleiben beim Pitch-Ziehen auf dem Drag-Start-Wert fixiert -- sonst wuerde
	// z.B. ein waehrenddessen laufendes Roll-Jogging durch das hier live() gelesene this.pose
	// ueberschrieben.
	private onHandleHeadOrientationDrag(axis: "pitch", deltaRad: number): void {
		if (!this.dragStartHeadPose) return;
		const start = this.dragStartHeadPose;
		this.backend.setCartesianJogTarget({
			xMm: start.xMm,
			yMm: start.yMm,
			zMm: start.zMm,
			pitchRad: axis === "pitch" ? start.pitchRad + deltaRad : start.pitchRad,
			rollRad: start.rollRad,
			gripperRad: start.gripperRad,
		});
	}

	private onHandleDragEnd(): void {
		this.dragStartJointAnglesCentiDeg = null;
		this.dragStartHeadPose = null;
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
			gripperRad: centiDegToRad(num("gripper") * 100),
		});
	}

	private addCurrentPoseAsStep(): void {
		this.steps = [
			...this.steps,
			{
				classId: roarm.JointMoveStep.CLASS_ID,
				jointAnglesCentiDeg: [...this.pose.jointAnglesCentiDeg],
				maxSpeedDegPerSec: DEFAULT_JOINT_MOVE_SPEED_DEG_PER_SEC,
			},
		];
	}

	private addGpioStep(): void {
		this.steps = [...this.steps, { classId: roarm.GpioStep.CLASS_ID, gpioId: this.newGpioId, state: this.newGpioState }];
	}

	private addDelayStep(): void {
		this.steps = [...this.steps, { classId: roarm.DelayStep.CLASS_ID, durationMs: this.newDelayMs }];
	}

	private removeStep(index: number): void {
		this.steps = this.steps.filter((_, i) => i !== index);
	}

	private moveStep(index: number, delta: number): void {
		const target = index + delta;
		if (target < 0 || target >= this.steps.length) return;
		const next = [...this.steps];
		[next[index], next[target]] = [next[target], next[index]];
		this.steps = next;
	}

	private async refreshMissionGpioNames(): Promise<void> {
		this.gpioNames = await this.backend.getMissionGpioNames();
	}

	private async refreshMissionList(): Promise<void> {
		this.missionList = await this.backend.listMissions();
	}

	private async saveMission(): Promise<void> {
		if (this.missionIndex <= 0) {
			this.setStatus("Mission-Index muss > 0 sein", "error");
			return;
		}
		const result = await this.backend.saveMission(this.missionIndex, this.missionName || `Mission ${this.missionIndex}`, this.steps);
		if (result.success) {
			this.setStatus(`Mission ${this.missionIndex} gespeichert`, "success");
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
		this.setStatus(`Mission ${missionIndex} geladen`, "success");
	}

	private async deleteMission(missionIndex: number): Promise<void> {
		await this.backend.deleteMission(missionIndex);
		void this.refreshMissionList();
	}

	render() {
		return html`
			<div class="container roarm-container">
				<section class="header-section app-panel">
					<div class="panel-label">RoArm-M3 Teach</div>
					${this.statusMessage ? html`<div class="status-text status-${this.statusVariant}">${this.statusMessage}</div>` : ""}
				</section>

				<div class="roarm-layout">
					<div class="panel-section roarm-view-panel">
						<div class="panel-label">
						3D-Vorschau (Maus: drehen/zoomen) -- goldene Ringe: Gelenke drehen, RGB-Pfeile am Kopf: Position ziehen, oranger/goldener Ring am Kopf: Pitch/Roll drehen
					</div>
						<div class="roarm-3d-container"></div>

						<div class="roarm-pose-readout">
							Pose: X=${this.pose.xMm}mm Y=${this.pose.yMm}mm Z=${this.pose.zMm}mm
							Pitch=${(this.pose.pitchCentiDeg / 100).toFixed(0)}° Roll=${(this.pose.rollCentiDeg / 100).toFixed(0)}°
						</div>

						<div class="panel-label">Gelenke (Jogging)</div>
						${JOINT_NAMES.map(
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

						<div class="panel-label">Kartesisches Ziel</div>
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
							<label>Greifer<input class="panel-input" type="number" name="gripper" value="0" /></label>
							<button type="submit">Anfahren</button>
						</form>
					</div>

					<div class="panel-section roarm-steps-panel">
						<div class="panel-label">Schritte</div>
						<div class="roarm-step-toolbar">
							<button @click=${() => this.addCurrentPoseAsStep()}>+ Aktuelle Pose</button>
						</div>

						<div class="roarm-step-toolbar">
							<select class="panel-input" .value=${this.newGpioId.toString()} @change=${(e: Event) => (this.newGpioId = Number((e.target as HTMLSelectElement).value))}>
								${this.gpioNames.map((name, i) => html`<option value=${i}>${name}</option>`)}
							</select>
							<select class="panel-input" .value=${this.newGpioState ? "1" : "0"} @change=${(e: Event) => (this.newGpioState = (e.target as HTMLSelectElement).value === "1")}>
								<option value="1">EIN</option>
								<option value="0">AUS</option>
							</select>
							<button @click=${() => this.addGpioStep()}>+ GPIO-Schritt</button>
						</div>

						<div class="roarm-step-toolbar">
							<input class="panel-input" type="number" .value=${this.newDelayMs.toString()} @input=${(e: Event) => (this.newDelayMs = Number((e.target as HTMLInputElement).value))} />
							<button @click=${() => this.addDelayStep()}>+ Delay (ms)</button>
						</div>

						<ol class="roarm-step-list">
							${this.steps.map(
								(step, i) => html`
									<li class="roarm-step-row">
										<span>${stepSummary(step, this.gpioNames)}</span>
										<span class="roarm-step-row-actions">
											<button @click=${() => this.moveStep(i, -1)}>↑</button>
											<button @click=${() => this.moveStep(i, 1)}>↓</button>
											<button @click=${() => this.removeStep(i)}>✕</button>
										</span>
									</li>
								`,
							)}
						</ol>

						<div class="panel-label">Mission speichern</div>
						<div class="roarm-step-toolbar">
							<input class="panel-input" type="number" min="1" .value=${this.missionIndex.toString()} @input=${(e: Event) => (this.missionIndex = Number((e.target as HTMLInputElement).value))} />
							<input class="panel-input" type="text" placeholder="Name" .value=${this.missionName} @input=${(e: Event) => (this.missionName = (e.target as HTMLInputElement).value)} />
							<button @click=${() => this.saveMission()}>Speichern</button>
						</div>

						<div class="panel-label">Gespeicherte Missionen</div>
						<ul class="roarm-step-list">
							${this.missionList.map(
								(m) => html`
									<li class="roarm-step-row">
										<span>#${m.missionIndex} -- ${m.name}</span>
										<span class="roarm-step-row-actions">
											<button @click=${() => this.loadMission(m.missionIndex)}>Laden</button>
											<button @click=${() => this.deleteMission(m.missionIndex)}>Löschen</button>
										</span>
									</li>
								`,
							)}
						</ul>
					</div>
				</div>
			</div>
		`;
	}
}
