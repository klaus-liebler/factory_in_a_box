// Druckregelstrecke: Uebersichtsseite fuer den pneumatischen Versuchsaufbau (Kompressor ->
// Druckspeicher -> 3 Ventile -> Verbraucher, s. register_map_schema/Motor.cs, Pressure.cs,
// Valve.cs) MIT den vier Betriebsmodi aus docs/druckregelstrecke-modes.md (Freies Experiment,
// Kennlinie, Sprungantwort, Reglerbetrieb). Analog zu roarm-teach-app.ts (das die WebGL-Ansicht
// aus roarm-3d-view.ts einhaengt) bindet diese Datei die SVG-Ansicht aus
// druckregelstrecke-view.ts ein; die Anlagengrafik bleibt in JEDEM Modus sichtbar/bedienbar
// (Ventile per Klick, Kompressor ueber sein Bedientableau), darunter wechselt nur die
// modusspezifische Auswerte-/Bedien-UI. Aller Modus-State (Kennlinien-Punkte,
// Sprungantwort-Aufzeichnung/-Ergebnis, Reglereinstellungen) lebt bewusst HIER auf App-Ebene
// (s. docs/druckregelstrecke-modes.md, "Offene Punkte") statt in isolierten Unterkomponenten,
// damit z.B. der Sprungantwort-Entwurfsassistent seine Ergebnisse direkt in den
// Reglerbetrieb-Modus uebernehmen kann.
//
// Zwei parallele Datenquellen: die REST-Registerabfrage (fetchRegisters(), 1Hz, wie schon vorher)
// treibt weiterhin die SVG-Anlagengrafik; die NEUE pneumatics.PressureControlFeedback-WS-Nachricht
// (500ms, s. ws-client.ts/best_binary_buffers_schema/pneumatics.cs) speist die Trend-Ringpuffer,
// die Kennlinien-Laufzeit-Maximum-Verfolgung, die Sprungantwort-Aufzeichnung UND den
// clientseitigen Regler -- fuer all das waere 1Hz zu grob bzw. das volle Mehrpaket-
// "/api/registers" pro Regelzyklus unnoetig schwer (s. docs/druckregelstrecke-modes.md,
// "Offene Punkte").
import { LitElement, html } from "lit";
import { customElement, state, query } from "lit/decorators.js";
import "../styles.css";
import { REGIONS, type RegisterDef } from "../../generated/register-map.js";
import type { pneumatics } from "../../generated/ws-protocol.js";
import { fetchRegisters, writeHolding, type RegisterValues } from "../registers.js";
import { subscribePressureControlFeedback } from "../ws-client.js";
import type { DashboardApp } from "../shell/dashboard-app.js";
import { createDruckregelstreckeView, type DruckregelstreckeViewHandles, type ValveIndex } from "./druckregelstrecke-view.js";
import { drawTrendChart, drawCharacteristicChart, type TrendSample, type CharacteristicCurve } from "./druckregelstrecke-charts.js";
import {
	PidController,
	designController,
	type ControllerSettings,
	type ControllerType,
	type StepResponseParameters,
	type DesignMethod,
	type DesignMethodChoice,
	type TSummeVariant,
	type ChrOvershoot,
	type ChrBehavior,
} from "./druckregelstrecke-controller.js";

const POLL_INTERVAL_MS = 1000;
const CONTROLLER_CYCLE_MS = 500;
const TREND_WINDOW_SEC = 60;
const CURVE_PALETTE = ["#2a78d6", "#e08a1e", "#16a34a", "#b91c1c", "#7c3aed", "#0891b2", "#be185d", "#4d7c0f"];

type Mode = "free" | "kennlinie" | "sprung" | "regler";
type SprungPhase = "idle" | "before" | "stepped" | "analyzed";

function findRegister(name: string): RegisterDef {
	for (const region of REGIONS) {
		const reg = region.registers.find((r) => r.name === name);
		if (reg) return reg;
	}
	throw new Error(`Register ${name} nicht in register-map.ts gefunden`);
}

const PRESSURE_REG = findRegister("PRESSURE_RAW");
const COMPRESSOR_PWM_REG = findRegister("COMPRESSOR_PWM");
const VALVE_REGS: readonly [RegisterDef, RegisterDef, RegisterDef] = [findRegister("VALVE1"), findRegister("VALVE2"), findRegister("VALVE3")];

// Zuletzt gewaehlte Leistung beim erneuten Einschalten des Kompressors ueber den Bedientableau-
// Kippschalter -- ohne das wuerde ein "Aus"-Klick (Leistung -> 0) den Regler unbrauchbar machen,
// sobald man wieder einschaltet (Regler stuende dann wieder auf 0, "an" waere ohne PWM aber ein
// Widerspruch).
const DEFAULT_COMPRESSOR_PWM_ON = 500;

function readRegister(values: RegisterValues, reg: RegisterDef): number | undefined {
	const source = reg.bank === "holding" ? values.holding : values.input;
	return source[reg.address];
}

function valveComboKey(p: pneumatics.PressureControlFeedback.Payload): string {
	return `${p.valve1Open ? 1 : 0}${p.valve2Open ? 1 : 0}${p.valve3Open ? 1 : 0}`;
}

function valveComboLabel(key: string): string {
	const [v1, v2, v3] = key;
	const s = (c: string) => (c === "1" ? "auf" : "zu");
	return `V1 ${s(v1)} / V2 ${s(v2)} / V3 ${s(v3)}`;
}

@customElement("druckregelstrecke-app")
export class DruckregelstreckeApp extends LitElement implements DashboardApp {
	protected createRenderRoot() {
		return this;
	}

	@query(".druckregel-view-container") private viewContainer!: HTMLDivElement;
	private view: DruckregelstreckeViewHandles | null = null;

	@query(".trend-canvas") private trendCanvas?: HTMLCanvasElement;
	@query(".characteristic-canvas") private kennlinieCanvas?: HTMLCanvasElement;

	@state() private values: RegisterValues = { holding: [], input: [] };
	@state() private statusMessage = "Lade Messwerte...";
	@state() private statusVariant: "warning" | "success" | "error" = "warning";
	@state() private mode: Mode = "free";

	private pollTimeoutHandle?: ReturnType<typeof setTimeout>;
	private stopped = true;
	private lastNonZeroPwm = DEFAULT_COMPRESSOR_PWM_ON;

	// --- Gemeinsame Messwert-Quelle (pneumatics.PressureControlFeedback, s. Dateikommentar) ---
	private unsubscribeFeedback: (() => void) | null = null;
	private latestFeedback: pneumatics.PressureControlFeedback.Payload | null = null;
	private trendSamples: TrendSample[] = [];

	// --- Kennlinie ---
	@state() private kennlinieCurves: Map<string, { compressorPermille: number; maxPressureRaw: number }[]> = new Map();
	private kennlinieComboOrder: string[] = [];
	private kennlinieTrackingKey: string | null = null;
	@state() private kennlinieRunningMaxRaw = 0;

	// --- Sprungantwort ---
	@state() private sprungPhase: SprungPhase = "idle";
	@state() private sprungBeforePermille = 0;
	@state() private sprungAfterPermille = 500;
	private sprungRecording: { t: number; pressureRaw: number }[] | null = null;
	private sprungJumpAtMs = 0;
	@state() private sprungResult: StepResponseParameters | null = null;
	@state() private designChoice: DesignMethodChoice = {
		method: "t-summe",
		controllerType: "PID",
		tSummeVariant: "normal",
		chrOvershoot: "0",
		chrBehavior: "fuehrung",
	};

	// --- Reglerbetrieb --- Default Kp/Tn/Tv aus der T-Summen-Regel (normale Einstellung, PID)
	// mit den in docs/druckregelstrecke-modes.md genannten ERWARTETEN Streckenkennwerten
	// (Tu≈3s, T≈10s) und Ks=1 als neutralem Platzhalter -- nur ein Startpunkt, gedacht zum
	// Ueberschreiben, sobald eine echte Sprungantwort samt Ks gemessen und uebernommen wurde.
	@state() private sollwertRaw = 2048;
	@state() private reglerOn = false;
	@state() private reglerSettings: ControllerSettings = { type: "PID", arbeitspunktPermille: 300, kp: 1, tnSec: 8.58, tvSec: 2.17 };
	private controller: PidController | null = null;
	private reglerIntervalHandle?: ReturnType<typeof setInterval>;
	private lastControllerTickMs = 0;

	onShow(): void {
		this.stopped = false;
		this.ensureView();
		this.scheduleNextPoll(0);
		this.unsubscribeFeedback = subscribePressureControlFeedback((p) => this.onPressureFeedback(p));
	}

	onHide(): void {
		this.stopped = true;
		if (this.pollTimeoutHandle) clearTimeout(this.pollTimeoutHandle);
		this.unsubscribeFeedback?.();
		this.unsubscribeFeedback = null;
		// Sicherheitsabschaltung: ein laufender clientseitiger Regelkreis soll nicht unsichtbar
		// weiterlaufen, nur weil die Seite gerade nicht angezeigt wird.
		if (this.reglerOn) this.toggleRegler(false);
	}

	protected firstUpdated(): void {
		this.ensureView();
	}

	private ensureView(): void {
		if (this.view || !this.viewContainer) return;
		this.view = createDruckregelstreckeView(this.viewContainer, {
			onValveClick: (index) => this.toggleValve(index),
			onCompressorToggle: () => this.toggleCompressor(),
			onCompressorPowerChange: (promille) => this.setCompressorPower(promille),
		});
		this.pushStateToView();
	}

	private scheduleNextPoll(delayMs: number): void {
		if (this.stopped) return;
		this.pollTimeoutHandle = setTimeout(() => {
			void this.poll().finally(() => this.scheduleNextPoll(POLL_INTERVAL_MS));
		}, delayMs);
	}

	private async poll(): Promise<void> {
		try {
			this.values = await fetchRegisters();
			const pwm = readRegister(this.values, COMPRESSOR_PWM_REG);
			if (pwm) this.lastNonZeroPwm = pwm;
			const now = new Date().toLocaleTimeString("de-DE");
			this.statusMessage = `Verbunden -- zuletzt aktualisiert ${now}`;
			this.statusVariant = "success";
		} catch (error) {
			console.error("Druckregelstrecke-Abfrage fehlgeschlagen", error);
			this.statusMessage = "Verbindungsproblem zur Control-Unit";
			this.statusVariant = "error";
		}
	}

	private pushStateToView(): void {
		if (!this.view) return;
		this.view.setState({
			pressureRaw: readRegister(this.values, PRESSURE_REG) ?? 0,
			valveOpen: VALVE_REGS.map((reg) => (readRegister(this.values, reg) ?? 0) !== 0) as [boolean, boolean, boolean],
			compressorPwmPromille: readRegister(this.values, COMPRESSOR_PWM_REG) ?? 0,
		});
	}

	// Zentrale Senke fuer JEDEN eintreffenden pneumatics.PressureControlFeedback-Tick (500ms) --
	// speist Trend-Ringpuffer, Kennlinien-Laufzeit-Maximum, Sprungantwort-Aufzeichnung und (via
	// this.latestFeedback) den Reglerkreis, s. Dateikommentar.
	private onPressureFeedback(payload: pneumatics.PressureControlFeedback.Payload): void {
		this.latestFeedback = payload;
		const now = Date.now();
		this.trendSamples.push({ t: now, pressureRaw: payload.pressureRaw, compressorPermille: payload.compressorPwmPermille });
		const cutoff = now - TREND_WINDOW_SEC * 1000;
		while (this.trendSamples.length > 0 && this.trendSamples[0].t < cutoff) this.trendSamples.shift();

		if (this.mode === "kennlinie") {
			const trackingKey = `${valveComboKey(payload)}@${payload.compressorPwmPermille}`;
			if (this.kennlinieTrackingKey !== trackingKey) {
				this.kennlinieTrackingKey = trackingKey;
				this.kennlinieRunningMaxRaw = payload.pressureRaw;
			} else {
				this.kennlinieRunningMaxRaw = Math.max(this.kennlinieRunningMaxRaw, payload.pressureRaw);
			}
		}

		if (this.sprungRecording) {
			this.sprungRecording.push({ t: now - this.sprungJumpAtMs, pressureRaw: payload.pressureRaw });
		}

		this.requestUpdate();
	}

	protected updated(): void {
		this.pushStateToView();
		if (this.trendCanvas) {
			drawTrendChart(this.trendCanvas, this.trendSamples, Date.now(), TREND_WINDOW_SEC, this.mode === "regler" ? this.sollwertRaw : undefined);
		}
		if (this.kennlinieCanvas) {
			drawCharacteristicChart(this.kennlinieCanvas, this.kennlinieCurvesForChart());
		}
	}

	private toggleValve(index: ValveIndex): void {
		const reg = VALVE_REGS[index];
		const current = readRegister(this.values, reg) ?? 0;
		void writeHolding(reg.address, current !== 0 ? 0 : 1).then(() => this.poll());
	}

	private toggleCompressor(): void {
		const current = readRegister(this.values, COMPRESSOR_PWM_REG) ?? 0;
		const next = current !== 0 ? 0 : this.lastNonZeroPwm;
		void writeHolding(COMPRESSOR_PWM_REG.address, next).then(() => this.poll());
	}

	private setCompressorPower(promille: number): void {
		void writeHolding(COMPRESSOR_PWM_REG.address, promille).then(() => this.poll());
	}

	private selectMode(mode: Mode): void {
		this.mode = mode;
		this.kennlinieTrackingKey = null;
	}

	// --- Kennlinie -----------------------------------------------------------------------

	private kennlinieCurvesForChart(): CharacteristicCurve[] {
		return this.kennlinieComboOrder.map((key, i) => ({
			label: valveComboLabel(key),
			color: CURVE_PALETTE[i % CURVE_PALETTE.length],
			points: (this.kennlinieCurves.get(key) ?? []).map((p) => ({ compressorPermille: p.compressorPermille, maxPressureRaw: p.maxPressureRaw })),
		}));
	}

	private commitKennlinePoint(): void {
		if (!this.latestFeedback) return;
		const key = valveComboKey(this.latestFeedback);
		if (!this.kennlinieComboOrder.includes(key)) this.kennlinieComboOrder = [...this.kennlinieComboOrder, key];
		const pwm = this.latestFeedback.compressorPwmPermille;
		const points = [...(this.kennlinieCurves.get(key) ?? [])];
		const idx = points.findIndex((p) => p.compressorPermille === pwm);
		const point = { compressorPermille: pwm, maxPressureRaw: this.kennlinieRunningMaxRaw };
		if (idx >= 0) points[idx] = point;
		else points.push(point);
		const next = new Map(this.kennlinieCurves);
		next.set(key, points);
		this.kennlinieCurves = next;
	}

	// --- Sprungantwort ---------------------------------------------------------------------

	private sprungStart(): void {
		void writeHolding(COMPRESSOR_PWM_REG.address, this.sprungBeforePermille).then(() => this.poll());
		this.sprungPhase = "before";
		this.sprungRecording = null;
		this.sprungResult = null;
	}

	private sprungJump(): void {
		void writeHolding(COMPRESSOR_PWM_REG.address, this.sprungAfterPermille).then(() => this.poll());
		this.sprungJumpAtMs = Date.now();
		this.sprungRecording = [];
		this.sprungPhase = "stepped";
	}

	// Totzeit: erste merkliche Abweichung vom Vorher-Wert (Schwelle: 2% der Gesamtbewegung, min.
	// 20 Counts Rauschmarge). Zeitkonstante: 63%-Zeit ABZUEGLICH der Totzeit -- gemessen ab dem
	// Ende der Totzeit (Standardkonvention fuer die Zeitkonstante eines PT1-Glieds: bei einer
	// reinen Exponentialantwort y(t)=K(1-e^(-t/T)) sind nach genau T 63,2% des Endwerts erreicht,
	// t=0 ist hier der Moment, ab dem die Antwort ueberhaupt zu reagieren beginnt, also NACH der
	// Totzeit). S. auch StepResponseParameters-Kommentar in druckregelstrecke-controller.ts fuer
	// die daran anschliessende Annahme T≈Tg (Wendetangenten-Ausgleichszeit) fuer Chien-Hrones-
	// Reswick.
	private sprungEndAndAnalyze(): void {
		void writeHolding(COMPRESSOR_PWM_REG.address, 0).then(() => this.poll());
		const recording = this.sprungRecording;
		this.sprungRecording = null;
		this.sprungPhase = "analyzed";
		if (!recording || recording.length < 4) {
			this.sprungResult = null;
			return;
		}
		const before = recording[0].pressureRaw;
		const after = recording[recording.length - 1].pressureRaw;
		const totalMovement = after - before;
		const deltaPermille = this.sprungAfterPermille - this.sprungBeforePermille;
		if (totalMovement === 0 || deltaPermille === 0) {
			this.sprungResult = null;
			return;
		}
		const ks = totalMovement / deltaPermille;

		const threshold = Math.max(20, Math.abs(totalMovement) * 0.02);
		const deadTimeSample = recording.find((s) => Math.abs(s.pressureRaw - before) >= threshold);
		const tuSec = deadTimeSample ? deadTimeSample.t / 1000 : 0;

		const target63 = before + 0.63 * totalMovement;
		const sample63 = totalMovement >= 0 ? recording.find((s) => s.pressureRaw >= target63) : recording.find((s) => s.pressureRaw <= target63);
		const t63Sec = sample63 ? sample63.t / 1000 : 0;

		this.sprungResult = { ks, tuSec: Math.max(0.1, tuSec), tSec: Math.max(0.1, t63Sec - tuSec) };
	}

	private applyDesignResult(): void {
		if (!this.sprungResult) return;
		const result = designController(this.sprungResult, this.designChoice);
		if (!result) return;
		// Auf sinnvolle Anzeigegenauigkeit runden -- result.kp/tnSec/tvSec sind sonst volle
		// Fliesskomma-Praezision aus der Formel, was in den Zahlenfeldern nur unleserlich waere.
		this.reglerSettings = {
			type: this.designChoice.controllerType,
			arbeitspunktPermille: this.reglerSettings.arbeitspunktPermille,
			kp: Math.round(result.kp * 10000) / 10000,
			tnSec: Math.round(result.tnSec * 1000) / 1000,
			tvSec: Math.round(result.tvSec * 1000) / 1000,
		};
		this.mode = "regler";
	}

	// --- Reglerbetrieb ---------------------------------------------------------------------

	private toggleRegler(on: boolean): void {
		this.reglerOn = on;
		if (this.reglerIntervalHandle) {
			clearInterval(this.reglerIntervalHandle);
			this.reglerIntervalHandle = undefined;
		}
		if (on) {
			this.controller = new PidController(this.reglerSettings);
			this.controller.reset();
			this.lastControllerTickMs = Date.now();
			this.reglerIntervalHandle = setInterval(() => this.reglerTick(), CONTROLLER_CYCLE_MS);
		} else {
			this.controller = null;
			void writeHolding(COMPRESSOR_PWM_REG.address, 0).then(() => this.poll());
		}
	}

	private reglerTick(): void {
		if (!this.controller || !this.latestFeedback) return;
		const now = Date.now();
		const dtSec = (now - this.lastControllerTickMs) / 1000;
		this.lastControllerTickMs = now;
		const output = this.controller.step(this.sollwertRaw, this.latestFeedback.pressureRaw, dtSec);
		void writeHolding(COMPRESSOR_PWM_REG.address, output);
	}

	private updateReglerSettings(patch: Partial<ControllerSettings>): void {
		this.reglerSettings = { ...this.reglerSettings, ...patch };
		this.controller?.updateSettings(this.reglerSettings);
	}

	// --- Rendering ---------------------------------------------------------------------------

	private renderModeTabs() {
		const tabs: { mode: Mode; label: string }[] = [
			{ mode: "free", label: "Freies Experiment" },
			{ mode: "kennlinie", label: "Kennlinie" },
			{ mode: "sprung", label: "Sprungantwort" },
			{ mode: "regler", label: "Reglerbetrieb" },
		];
		return html`
			<div class="druckregel-mode-tabs">
				${tabs.map(
					(t) => html`
						<button class=${this.mode === t.mode ? "druckregel-mode-tab druckregel-mode-tab-active" : "druckregel-mode-tab"} @click=${() => this.selectMode(t.mode)}>
							${t.label}
						</button>
					`
				)}
			</div>
		`;
	}

	private renderFreeMode() {
		return html`
			<div class="panel-section">
				<div class="panel-label">Verlauf der letzten 60 Sekunden</div>
				<canvas class="trend-canvas druckregel-trend-canvas"></canvas>
			</div>
		`;
	}

	private renderKennlinieMode() {
		const combo = this.latestFeedback ? valveComboLabel(valveComboKey(this.latestFeedback)) : "–";
		const pwm = this.latestFeedback?.compressorPwmPermille ?? 0;
		return html`
			<div class="panel-section">
				<div class="panel-label">Aktueller Betriebspunkt</div>
				<div>${combo} — Kompressor ${pwm} ‰</div>
				<div class="panel-label">Beobachtetes Maximum seit letzter Änderung</div>
				<div class="druckregel-pressure-readout">${this.kennlinieRunningMaxRaw} / 4095</div>
				<button @click=${() => this.commitKennlinePoint()}>Wert in Kennliniendiagramm übernehmen</button>
			</div>
			<div class="panel-section">
				<div class="panel-label">Kennfeld (max. Druck über Kompressorleistung, je Ventilkombination)</div>
				<canvas class="characteristic-canvas druckregel-trend-canvas"></canvas>
			</div>
		`;
	}

	private renderSprungMode() {
		const r = this.sprungResult;
		const design = r ? designController(r, this.designChoice) : null;
		return html`
			<div class="panel-section">
				<div class="panel-label">Sprung definieren</div>
				<div class="druckregel-control-row">
					<label>Vorher (‰) <input class="panel-input" type="number" min="0" max="1000" .value=${String(this.sprungBeforePermille)}
						@change=${(e: Event) => (this.sprungBeforePermille = Number((e.target as HTMLInputElement).value))} /></label>
					<label>Nachher (‰) <input class="panel-input" type="number" min="0" max="1000" .value=${String(this.sprungAfterPermille)}
						@change=${(e: Event) => (this.sprungAfterPermille = Number((e.target as HTMLInputElement).value))} /></label>
				</div>
				<div class="druckregel-control-row">
					<button @click=${() => this.sprungStart()} ?disabled=${this.sprungPhase === "stepped"}>Start</button>
					<button @click=${() => this.sprungJump()} ?disabled=${this.sprungPhase !== "before"}>Sprung!</button>
					<button @click=${() => this.sprungEndAndAnalyze()} ?disabled=${this.sprungPhase !== "stepped"}>Ende und Analyse</button>
				</div>
			</div>

			<div class="panel-section">
				<div class="panel-label">Verlauf der letzten 60 Sekunden</div>
				<canvas class="trend-canvas druckregel-trend-canvas"></canvas>
			</div>

			${r
				? html`
						<div class="panel-section">
							<div class="panel-label">Analyseergebnis</div>
							<div>Totzeit T_t ≈ ${r.tuSec.toFixed(2)} s -- Zeitkonstante T ≈ ${r.tSec.toFixed(2)} s -- K_S ≈ ${r.ks.toFixed(3)} Counts/‰</div>

							<div class="panel-label">Regler-Entwurfs-Assistent</div>
							<div class="druckregel-control-row">
								<label>Verfahren
									<select @change=${(e: Event) => (this.designChoice = { ...this.designChoice, method: (e.target as HTMLSelectElement).value as DesignMethod })}>
										<option value="t-summe" ?selected=${this.designChoice.method === "t-summe"}>T-Summen-Regel</option>
										<option value="chr" ?selected=${this.designChoice.method === "chr"}>Chien-Hrones-Reswick</option>
									</select>
								</label>
								<label>Reglertyp
									<select @change=${(e: Event) => (this.designChoice = { ...this.designChoice, controllerType: (e.target as HTMLSelectElement).value as ControllerType })}>
										<option value="P" ?selected=${this.designChoice.controllerType === "P"}>P</option>
										<option value="PI" ?selected=${this.designChoice.controllerType === "PI"}>PI</option>
										<option value="PID" ?selected=${this.designChoice.controllerType === "PID"}>PID</option>
									</select>
								</label>
								${this.designChoice.method === "t-summe"
									? html`
											<label>Einstellung
												<select @change=${(e: Event) => (this.designChoice = { ...this.designChoice, tSummeVariant: (e.target as HTMLSelectElement).value as TSummeVariant })}>
													<option value="normal" ?selected=${this.designChoice.tSummeVariant === "normal"}>normal</option>
													<option value="schnell" ?selected=${this.designChoice.tSummeVariant === "schnell"}>schnell</option>
												</select>
											</label>
										`
									: html`
											<label>Überschwingen
												<select @change=${(e: Event) => (this.designChoice = { ...this.designChoice, chrOvershoot: (e.target as HTMLSelectElement).value as ChrOvershoot })}>
													<option value="0" ?selected=${this.designChoice.chrOvershoot === "0"}>0% (aperiodisch)</option>
													<option value="20" ?selected=${this.designChoice.chrOvershoot === "20"}>20%</option>
												</select>
											</label>
											<label>Verhalten
												<select @change=${(e: Event) => (this.designChoice = { ...this.designChoice, chrBehavior: (e.target as HTMLSelectElement).value as ChrBehavior })}>
													<option value="fuehrung" ?selected=${this.designChoice.chrBehavior === "fuehrung"}>Führungsverhalten</option>
													<option value="stoerung" ?selected=${this.designChoice.chrBehavior === "stoerung"}>Störverhalten</option>
												</select>
											</label>
										`}
							</div>
							${design
								? html`
										<div>Ergebnis: K_P = ${design.kp.toFixed(3)}${design.tnSec > 0 ? html`, T_N = ${design.tnSec.toFixed(2)} s` : ""}${design.tvSec > 0 ? html`, T_V = ${design.tvSec.toFixed(2)} s` : ""}</div>
										<button @click=${() => this.applyDesignResult()}>Werte übernehmen → Reglerbetrieb</button>
									`
								: html`<div class="status-text status-warning">Entwurfsverfahren für diese Reglertyp-Kombination nicht anwendbar.</div>`}
						</div>
					`
				: this.sprungPhase === "analyzed"
					? html`<div class="panel-section status-text status-error">Aufzeichnung zu kurz/unbrauchbar -- bitte Sprung wiederholen.</div>`
					: ""}
		`;
	}

	private renderReglerMode() {
		const s = this.reglerSettings;
		return html`
			<div class="panel-section">
				<div class="panel-label">Regler</div>
				<div class="druckregel-control-row">
					<label class="toggle-switch">
						<input id="regler-onoff-toggle" type="checkbox" .checked=${this.reglerOn} @change=${(e: Event) => this.toggleRegler((e.target as HTMLInputElement).checked)} />
						<span class="toggle-slider-track"></span>
					</label>
					<span>${this.reglerOn ? "Regler ein" : "Regler aus"}</span>
				</div>

				<div class="druckregel-control-row">
					<label>Sollwert (Rohwert) <input class="panel-input" type="number" min="0" max="4095" .value=${String(this.sollwertRaw)}
						@change=${(e: Event) => (this.sollwertRaw = Number((e.target as HTMLInputElement).value))} /></label>
					<label>Reglertyp
						<select @change=${(e: Event) => this.updateReglerSettings({ type: (e.target as HTMLSelectElement).value as ControllerType })}>
							<option value="P" ?selected=${s.type === "P"}>P</option>
							<option value="PI" ?selected=${s.type === "PI"}>PI</option>
							<option value="PID" ?selected=${s.type === "PID"}>PID</option>
						</select>
					</label>
				</div>
				<div class="druckregel-control-row">
					<label>Arbeitspunkt (‰) <input class="panel-input" type="number" min="0" max="1000" .value=${String(s.arbeitspunktPermille)}
						@change=${(e: Event) => this.updateReglerSettings({ arbeitspunktPermille: Number((e.target as HTMLInputElement).value) })} /></label>
					<label>K_P <input class="panel-input" type="number" step="0.01" .value=${String(s.kp)}
						@change=${(e: Event) => this.updateReglerSettings({ kp: Number((e.target as HTMLInputElement).value) })} /></label>
					${s.type !== "P"
						? html`<label>T_N (s) <input class="panel-input" type="number" step="0.1" .value=${String(s.tnSec)}
								@change=${(e: Event) => this.updateReglerSettings({ tnSec: Number((e.target as HTMLInputElement).value) })} /></label>`
						: ""}
					${s.type === "PID"
						? html`<label>T_V (s) <input class="panel-input" type="number" step="0.1" .value=${String(s.tvSec)}
								@change=${(e: Event) => this.updateReglerSettings({ tvSec: Number((e.target as HTMLInputElement).value) })} /></label>`
						: ""}
				</div>
				<div class="panel-text register-comment">Anti-Windup-Verfahren: Begrenzung des Integrators auf 100% (fest, einzige Option).</div>
			</div>

			<div class="panel-section">
				<div class="panel-label">Verlauf der letzten 60 Sekunden (gestrichelt: Sollwert)</div>
				<canvas class="trend-canvas druckregel-trend-canvas"></canvas>
			</div>
		`;
	}

	render() {
		return html`
			<div class="container druckregel-container">
				<section class="header-section app-panel">
					<div class="panel-label">Druckregelstrecke</div>
					<div class="status-text status-${this.statusVariant}">${this.statusMessage}</div>
				</section>

				<div class="panel-section druckregel-view-card">
					<div class="druckregel-view-container"></div>
				</div>

				${this.renderModeTabs()}

				${this.mode === "free" ? this.renderFreeMode() : ""}
				${this.mode === "kennlinie" ? this.renderKennlinieMode() : ""}
				${this.mode === "sprung" ? this.renderSprungMode() : ""}
				${this.mode === "regler" ? this.renderReglerMode() : ""}
			</div>
		`;
	}
}
