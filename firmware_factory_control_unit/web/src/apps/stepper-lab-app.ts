// Schrittmotoren: test bench of both TMC2209 axes (ws::stepper, src/io.cpp). Manual control (the
// register targets rest), moves of N full steps with a ramp or a sigmoid, sensorless homing with
// its own parameters, every TMC2209 setting, and the driver's live view - current, StallGuard,
// StealthChop amplitude, speed - as curves, to see what a setting does.
import { LitElement, html, nothing } from "lit";
import { customElement, query, state } from "lit/decorators.js";
import "../styles.css";
import "./stepper-lab.css";
import { stepper } from "../../generated/ws-protocol.js";
import { wsRequest } from "../ws-client.js";
import type { DashboardApp } from "../shell/dashboard-app.js";
import { setAppStatus } from "../shell/status-bar.js";
import { drawLineChart } from "./line-chart.js";
import {
	currentShare, decodeDrvStatus, decodePwmScale, delayUnitMs, fullStepsPerSecond, moveCurve, profileProblem, tstepOf, type Shape,
} from "./stepper-lab-model.js";

const NS = stepper.NAMESPACE_ID;
const POLL_MS = 200;
const HISTORY_S = 60;
const MICROSTEPS = [1, 2, 4, 8, 16, 32, 64, 128, 256];

interface Sample {
	t: number;
	cs: number;
	sg: number;
	pwm: number;
	speed: number;
}

/** The move form in full steps (the firmware counts micro steps). */
interface MoveForm {
	steps: number;
	startFs: number;
	cruiseFs: number;
	accelFs: number;
	shape: Shape;
}

@customElement("stepper-lab-app")
export class StepperLabApp extends LitElement implements DashboardApp {
	@state() private axis = 0;
	@state() private settings: stepper.TmcSettings | null = null;
	@state() private status: stepper.StatusResponse.Payload | null = null;
	@state() private move: MoveForm = { steps: 400, startFs: 20, cruiseFs: 200, accelFs: 400, shape: "sigmoid" };
	@state() private home = { speed: -200, sgThrs: 100, timeoutMs: 10000 };
	@state() private message = "";
	@state() private busy = false;
	private history: Sample[][] = [[], []];
	private timer: ReturnType<typeof setTimeout> | null = null;
	private shown = false;
	@query("#c-cs") private cCs?: HTMLCanvasElement;
	@query("#c-sg") private cSg?: HTMLCanvasElement;
	@query("#c-pwm") private cPwm?: HTMLCanvasElement;
	@query("#c-speed") private cSpeed?: HTMLCanvasElement;
	@query("#c-profile") private cProfile?: HTMLCanvasElement;

	protected createRenderRoot() {
		return this;
	}

	/** The state and the last result go to the application's status bar (shell/status-bar.ts). */
	protected willUpdate(): void {
		const s = this.status;
		const name = `Stepper ${this.axis + 1}`;
		setAppStatus(this, {
			text: !s ? `${name}: keine Antwort` : s.homing ? `${name}: Referenzfahrt` : s.moving ? `${name}: fährt` : s.manual ? `${name}: Handbetrieb` : `${name}: Registerbetrieb`,
			kind: !s ? "error" : s.homing || s.moving ? "busy" : s.tmcOk ? "ok" : "warning",
			extra: this.message ? html`<span class="sb-chip">${this.message}</span>` : null,
		});
	}

	onShow(): void {
		this.shown = true;
		void this.loadSettings();
		void this.poll();
	}

	onHide(): void {
		this.shown = false;
		if (this.timer) clearTimeout(this.timer);
		this.timer = null;
	}

	private get micro(): number {
		return this.settings?.microsteps ?? 16;
	}

	private say(text: string): void {
		this.message = text;
	}

	// ---- device ----------------------------------------------------------------------------------
	private async loadSettings(): Promise<void> {
		try {
			const r = await wsRequest(NS, (requestId) => stepper.SettingsRequest.encode({ requestId, axis: this.axis, write: false,
				settings: this.settings ?? DEFAULT_SETTINGS, profile: this.profile() }), stepper.SettingsResponse);
			this.settings = r.settings;
			const m = r.settings.microsteps || 16;
			this.move = { ...this.move, startFs: r.profile.startHz / m, cruiseFs: r.profile.cruiseHz / m, accelFs: r.profile.accel / m,
				shape: r.profile.shape === stepper.RampShape.Linear ? "linear" : "sigmoid" };
			this.home = { ...this.home, sgThrs: r.settings.sgThrs };
		} catch (e) {
			this.say(`Einstellungen nicht gelesen: ${(e as Error).message}`);
		}
	}

	private profile(): stepper.Profile {
		const m = this.micro;
		return { startHz: Math.round(this.move.startFs * m), cruiseHz: Math.round(this.move.cruiseFs * m), accel: Math.round(this.move.accelFs * m),
			shape: this.move.shape === "linear" ? stepper.RampShape.Linear : stepper.RampShape.Sigmoid };
	}

	private async apply(): Promise<void> {
		if (!this.settings) return;
		const p = this.profile();
		const problem = profileProblem({ startHz: p.startHz, cruiseHz: p.cruiseHz, accel: p.accel, shape: this.move.shape });
		if (problem) return this.say(problem);
		this.busy = true;
		try {
			const r = await wsRequest(NS, (requestId) => stepper.SettingsRequest.encode({ requestId, axis: this.axis, write: true, settings: this.settings!, profile: p }),
				stepper.SettingsResponse);
			this.settings = r.settings;
			this.say(r.ok ? "Einstellungen und Profil übernommen" : `Nicht übernommen: ${r.message}`);
		} catch (e) {
			this.say((e as Error).message);
		}
		this.busy = false;
	}

	private async command(what: string, run: () => Promise<{ ok: boolean; message?: string }>): Promise<void> {
		try {
			const r = await run();
			this.say(r.ok ? what : `${what}: ${r.message ?? "abgelehnt"}`);
		} catch (e) {
			this.say(`${what}: ${(e as Error).message}`);
		}
	}

	private setManual(on: boolean): Promise<void> {
		return this.command(on ? "Handbetrieb an" : "Handbetrieb aus - die Register steuern wieder",
			() => wsRequest(NS, (requestId) => stepper.ManualRequest.encode({ requestId, axis: this.axis, on }), stepper.ManualResponse));
	}

	private async drive(sign: number): Promise<void> {
		// the profile first (it may have changed), then the move
		if (!this.settings) return;
		const p = this.profile();
		const problem = profileProblem({ startHz: p.startHz, cruiseHz: p.cruiseHz, accel: p.accel, shape: this.move.shape });
		if (problem) return this.say(problem);
		const s = await wsRequest(NS, (requestId) => stepper.SettingsRequest.encode({ requestId, axis: this.axis, write: true, settings: this.settings!, profile: p }),
			stepper.SettingsResponse).catch((e: Error) => ({ ok: false, message: e.message }));
		if (!s.ok && !("settings" in s && s.message.includes("does not answer"))) return this.say(`Profil: ${s.message}`);
		const steps = Math.round(sign * this.move.steps * this.micro);
		return this.command(`Fahrt ${sign * this.move.steps} Vollschritte`,
			() => wsRequest(NS, (requestId) => stepper.MoveRequest.encode({ requestId, axis: this.axis, steps }), stepper.MoveResponse));
	}

	private stop(): Promise<void> {
		return this.command("Gestoppt", () => wsRequest(NS, (requestId) => stepper.StopRequest.encode({ requestId, axis: this.axis }), stepper.StopResponse));
	}

	private startHoming(): Promise<void> {
		const h = this.home;
		return this.command("Referenzfahrt gestartet", () => wsRequest(NS, (requestId) => stepper.HomeRequest.encode({ requestId, axis: this.axis,
			speed: h.speed, sgThrs: h.sgThrs, timeoutMs: h.timeoutMs }), stepper.HomeResponse));
	}

	private async poll(): Promise<void> {
		if (!this.shown) return;
		const axis = this.axis;
		try {
			const s = await wsRequest(NS, (requestId) => stepper.StatusRequest.encode({ requestId, axis }), stepper.StatusResponse);
			if (axis === this.axis) {
				this.status = s;
				const drv = decodeDrvStatus(s.drvStatus);
				const h = this.history[axis];
				h.push({ t: Date.now() / 1000, cs: s.tmcOk ? currentShare(drv.csActual) * 100 : 0, sg: s.sgResult, pwm: decodePwmScale(s.pwmScale).sum,
					speed: fullStepsPerSecond(s.tstep) });
				while (h.length && h[0].t < Date.now() / 1000 - HISTORY_S) h.shift();
			}
		} catch {
			this.status = null;
		}
		if (this.shown) this.timer = setTimeout(() => void this.poll(), POLL_MS);
	}

	protected updated(): void {
		const h = this.history[this.axis];
		const series = (f: (s: Sample) => number) => h.map((s) => ({ t: s.t * 1000, v: f(s) }));  // wall-clock ms
		if (this.cCs) drawLineChart(this.cCs, series((s) => s.cs), { color: "#dc2626", unit: "%", decimals: 0 });
		if (this.cSg) drawLineChart(this.cSg, series((s) => s.sg), { color: "#2563eb", unit: "", decimals: 0 });
		if (this.cPwm) drawLineChart(this.cPwm, series((s) => s.pwm), { color: "#7c3aed", unit: "", decimals: 0 });
		if (this.cSpeed) drawLineChart(this.cSpeed, series((s) => s.speed), { color: "#16a34a", unit: "VS/s", decimals: 0 });
		if (this.cProfile) {
			const p = this.profile();
			const ok = profileProblem({ startHz: p.startHz, cruiseHz: p.cruiseHz, accel: p.accel, shape: this.move.shape }) === null;
			const curve = ok ? moveCurve({ startHz: p.startHz, cruiseHz: p.cruiseHz, accel: p.accel, shape: this.move.shape }, Math.max(1, this.move.steps * this.micro)) : [];
			drawLineChart(this.cProfile, curve.map((c) => ({ t: c.t, v: c.v / this.micro })),
				{ color: "#0f766e", unit: "VS/s", decimals: 0, formatTime: (t) => `${t.toFixed(2)} s`, showLast: false });
		}
	}

	// ---- view --------------------------------------------------------------------------------------
	render() {
		const s = this.status;
		const drv = s?.tmcOk ? decodeDrvStatus(s.drvStatus) : null;  // without the chip: no values
		const pwm = s?.tmcOk ? decodePwmScale(s.pwmScale) : null;
		return html`
			<div class="container stepper-lab">
				<div class="stepper-head">
					<div class="stp-seg" role="group" aria-label="Achse">
						${[0, 1].map((a) => html`<button class=${this.axis === a ? "on" : ""} @click=${() => {
							this.axis = a;
							this.status = null;
							void this.loadSettings();
						}}>Stepper ${a + 1}</button>`)}
					</div>
					<label class="stepper-toggle" title="Die Register-Ziele (Modbus, OPC UA, SPS) ruhen, diese Seite fährt die Achse - auch ohne Referenzfahrt">
						<input type="checkbox" .checked=${s?.manual ?? false} ?disabled=${!s} @change=${(e: Event) => this.setManual((e.target as HTMLInputElement).checked)} /> Handbetrieb
					</label>
					${s ? html`
						<span class="chip ${s.tmcOk ? "chip-ok" : "chip-bad"}">${s.tmcOk ? "TMC2209 antwortet" : "TMC2209 antwortet nicht"}</span>
						<span class="chip ${s.enabled ? "chip-ok" : ""}">${s.enabled ? "Treiber an" : "Treiber aus"}</span>
						<span class="chip ${s.homed ? "chip-ok" : "chip-warn"}">${s.homed ? "referenziert" : "nicht referenziert"}</span>
						${s.homing ? html`<span class="chip chip-warn">Referenzfahrt läuft</span>` : nothing}
						${s.moving ? html`<span class="chip chip-warn">fährt</span>` : nothing}
						<span class="chip" title="Stall-Erkennung: DIAG-Leitung (Interrupt, sofort) oder SG_RESULT über UART (alle 5 ms)">${s.hasDiag ? "DIAG verdrahtet" : "ohne DIAG: Stall über UART"}</span>` : nothing}
				</div>

				<section class="panel-section stepper-live">
					<h3>Live</h3>
					<div class="stepper-facts">
						<span>Position <b>${s ? (s.position / this.micro).toFixed(this.micro > 1 ? 2 : 0) : "-"}</b> VS <span class="muted">(${s?.position ?? "-"} µS)</span></span>
						<span>Geschwindigkeit <b>${s?.tmcOk ? fullStepsPerSecond(s.tstep).toFixed(0) : "-"}</b> VS/s</span>
						<span title="CS_ACTUAL: der Strom, den der Treiber gerade einstellt (nach IHOLD/IRUN, CoolStep)">Strom <b>${drv ? `${drv.csActual}/31 (${(currentShare(drv.csActual) * 100).toFixed(0)} %)` : "-"}</b></span>
						<span title="SG_RESULT: Last-Maß; klein = hohe Last (Stall bei ≤ 2·SGTHRS)">StallGuard <b>${s?.tmcOk ? s.sgResult : "-"}</b></span>
						<span title="PWM_SCALE_SUM: Amplitude im StealthChop (Autoscale regelt sie)">PWM <b>${pwm ? `${pwm.sum} (auto ${pwm.auto})` : "-"}</b></span>
						${drv ? html`
							<span class="flags">
								${drv.standstill ? html`<span class="chip">Stillstand</span>` : nothing}
								<span class="chip">${drv.stealth ? "StealthChop" : "SpreadCycle"}</span>
								<span class="chip">${drv.temperature}</span>
								${drv.overtempWarning ? html`<span class="chip chip-warn">Übertemperatur-Warnung</span>` : nothing}
								${drv.overtemp ? html`<span class="chip chip-bad">Übertemperatur: abgeschaltet</span>` : nothing}
								${drv.shortToGround || drv.shortLowSide ? html`<span class="chip chip-bad">Kurzschluss</span>` : nothing}
								${drv.openLoad ? html`<span class="chip chip-warn" title="auch bei schneller Fahrt oder im Stillstand möglich">offene Last?</span>` : nothing}
								${s && s.homingResult ? html`<span class="chip ${s.homingResult === 1 ? "chip-ok" : "chip-bad"}">Referenz: ${["", "gefunden", "Timeout", "Fehler"][s.homingResult]}${s.homingResult === 1 ? (s.stallByDiag ? " (DIAG)" : " (UART)") : ""}</span>` : nothing}
							</span>` : nothing}
					</div>
					<div class="stepper-charts">
						<figure><figcaption>Strom CS_ACTUAL [% Vollausschlag]</figcaption><canvas id="c-cs"></canvas></figure>
						<figure><figcaption>StallGuard SG_RESULT</figcaption><canvas id="c-sg"></canvas></figure>
						<figure><figcaption>StealthChop-Amplitude PWM_SCALE_SUM</figcaption><canvas id="c-pwm"></canvas></figure>
						<figure><figcaption>Geschwindigkeit [Vollschritte/s]</figcaption><canvas id="c-speed"></canvas></figure>
					</div>
				</section>

				<div class="stepper-cols">
					<section class="panel-section">
						<h3>Fahren</h3>
						${this.renderMove(s)}
					</section>
					<section class="panel-section">
						<h3>Referenzfahrt (sensorlos, StallGuard)</h3>
						${this.renderHoming(s)}
					</section>
				</div>

				<section class="panel-section">
					<h3>TMC2209-Einstellungen (Stepper ${this.axis + 1})</h3>
					${this.settings ? this.renderSettings(this.settings) : html`<p class="muted">werden gelesen …</p>`}
				</section>
			</div>`;
	}

	private num(label: string, value: number, set: (v: number) => void, opts: { min?: number; max?: number; step?: number; title?: string; unit?: string } = {}) {
		return html`<label class="stepper-field" title=${opts.title ?? ""}><span>${label}</span>
			<input type="number" .value=${String(value)} min=${opts.min ?? nothing} max=${opts.max ?? nothing} step=${opts.step ?? 1}
				@change=${(e: Event) => set(Number((e.target as HTMLInputElement).value))} />${opts.unit ? html`<span class="muted">${opts.unit}</span>` : nothing}</label>`;
	}

	private renderMove(s: stepper.StatusResponse.Payload | null) {
		const m = this.move;
		const set = (c: Partial<MoveForm>) => (this.move = { ...m, ...c });
		const manual = s?.manual ?? false;
		const p = this.profile();
		const problem = profileProblem({ startHz: p.startHz, cruiseHz: p.cruiseHz, accel: p.accel, shape: m.shape });
		const curve = problem ? null : moveCurve({ startHz: p.startHz, cruiseHz: p.cruiseHz, accel: p.accel, shape: m.shape }, Math.max(1, m.steps * this.micro));
		return html`
			<div class="stepper-form">
				${this.num("Schritte", m.steps, (v) => set({ steps: Math.max(1, Math.round(v)) }), { min: 1, unit: "Vollschritte" })}
				${this.num("Start", m.startFs, (v) => set({ startFs: v }), { min: 0, step: 0.1, unit: "VS/s", title: "Geschwindigkeit beim Anfahren und Anhalten" })}
				${this.num("Ziel", m.cruiseFs, (v) => set({ cruiseFs: v }), { min: 0, unit: "VS/s" })}
				${this.num("Beschleunigung", m.accelFs, (v) => set({ accelFs: v }), { min: 1, unit: "VS/s²", title: "der Rampe; das Sigmoid legt dieselbe Strecke zurück, beschleunigt aber in der Mitte stärker und an den Enden sanft" })}
				<label class="stepper-field"><span>Form</span>
					<select @change=${(e: Event) => set({ shape: (e.target as HTMLSelectElement).value as Shape })}>
						<option value="linear" ?selected=${m.shape === "linear"}>Rampe (konstante Beschleunigung)</option>
						<option value="sigmoid" ?selected=${m.shape === "sigmoid"}>Sigmoid (ruckarm)</option>
					</select></label>
			</div>
			<figure class="stepper-profile"><figcaption>Geschwindigkeitsprofil der Fahrt${curve ? html` - Dauer ${curve.at(-1)!.t.toFixed(2)} s` : nothing}</figcaption><canvas id="c-profile"></canvas></figure>
			${problem ? html`<p class="stp-bad">${problem}</p>` : nothing}
			<div class="stepper-actions">
				<button class="stp-primary" ?disabled=${!manual || s?.moving || s?.homing || problem !== null} @click=${() => this.drive(1)}>▶ ${m.steps} VS vorwärts</button>
				<button ?disabled=${!manual || s?.moving || s?.homing || problem !== null} @click=${() => this.drive(-1)}>◀ ${m.steps} VS zurück</button>
				<button class="stepper-stop" ?disabled=${!s} @click=${() => this.stop()}>■ Stopp</button>
				${manual ? nothing : html`<span class="muted">Fahren nur im Handbetrieb</span>`}
			</div>`;
	}

	private renderHoming(s: stepper.StatusResponse.Payload | null) {
		const h = this.home;
		const set = (c: Partial<typeof h>) => (this.home = { ...h, ...c });
		return html`
			<p class="muted">Der Motor fährt mit dem internen Schrittgenerator (VACTUAL) gegen den Anschlag, bis StallGuard eine Blockade meldet: SG_RESULT ≤ 2·SGTHRS.
				${s?.hasDiag === false ? "Diese Platine hat keine DIAG-Leitung: die Blockade wird alle 5 ms über UART abgefragt." : "DIAG meldet die Blockade sofort, UART-Abfrage alle 5 ms als Rückfall."}</p>
			<div class="stepper-form">
				${this.num("Geschwindigkeit", h.speed, (v) => set({ speed: Math.round(v) }), { unit: "VS/s", title: "Vorzeichen = Richtung zum Anschlag; StallGuard braucht etwa ≥ 100 VS/s" })}
				${this.num("SGTHRS", h.sgThrs, (v) => set({ sgThrs: Math.max(0, Math.min(255, Math.round(v))) }), { min: 0, max: 255, title: "höher = empfindlicher" })}
				${this.num("Timeout", h.timeoutMs, (v) => set({ timeoutMs: Math.max(500, Math.round(v)) }), { min: 500, step: 500, unit: "ms" })}
			</div>
			<div class="stepper-actions">
				<button class="stp-primary" ?disabled=${!s || s.moving || s.homing} @click=${() => this.startHoming()}>⌂ Referenzfahrt starten</button>
				<button class="stepper-stop" ?disabled=${!s?.homing} @click=${() => this.stop()}>■ Abbrechen</button>
			</div>`;
	}

	private renderSettings(t: stepper.TmcSettings) {
		const set = (c: Partial<stepper.TmcSettings>) => (this.settings = { ...t, ...c });
		const pct = (v: number) => `${(currentShare(v) * 100).toFixed(0)} %`;
		const slider = (label: string, v: number, max: number, fmt: (v: number) => string, change: (v: number) => void, title: string) => html`
			<label class="stepper-slider" title=${title}><span>${label}</span>
				<input type="range" min="0" max=${max} .value=${String(v)} @input=${(e: Event) => change(Number((e.target as HTMLInputElement).value))} />
				<b>${v}</b><span class="muted">${fmt(v)}</span></label>`;
		const check = (label: string, v: boolean, change: (v: boolean) => void, title = "") => html`
			<label class="stepper-check" title=${title}><input type="checkbox" .checked=${v} @change=${(e: Event) => change((e.target as HTMLInputElement).checked)} /> ${label}</label>`;
		return html`
			<div class="stepper-settings">
				<fieldset><legend>Strom</legend>
					${slider("IRUN", t.irun, 31, pct, (v) => set({ irun: v }), "Fahrstrom: (IRUN+1)/32 des Vollausschlags")}
					${slider("IHOLD", t.ihold, 31, pct, (v) => set({ ihold: v }), "Haltestrom im Stillstand")}
					${slider("IHOLDDELAY", t.iholdDelay, 15, (v) => `${(v * delayUnitMs).toFixed(0)} ms/Stufe`, (v) => set({ iholdDelay: v }), "wie schnell der Strom auf IHOLD absinkt")}
					${slider("TPOWERDOWN", t.tpowerdown, 255, (v) => `${(v * delayUnitMs).toFixed(0)} ms`, (v) => set({ tpowerdown: v }), "Wartezeit nach dem Stillstand, bevor der Strom auf IHOLD absinkt (automatische Haltestromabsenkung)")}
				</fieldset>
				<fieldset><legend>Chopper</legend>
					<label class="stepper-check"><input type="radio" name="chop" .checked=${!t.spreadCycle} @change=${() => set({ spreadCycle: false })} /> StealthChop (leise)</label>
					<label class="stepper-check"><input type="radio" name="chop" .checked=${t.spreadCycle} @change=${() => set({ spreadCycle: true })} /> SpreadCycle (kräftig, hörbar)</label>
					${this.num("TPWMTHRS", t.tpwmThrs, (v) => set({ tpwmThrs: Math.max(0, Math.round(v)) }), { min: 0, title: `StealthChop nur unterhalb dieser Geschwindigkeit (TSTEP-Wert; 0 = immer). ${tstepOf(100)} ≈ 100 VS/s` })}
					${check("pwm_autoscale", t.pwmAutoscale, (v) => set({ pwmAutoscale: v }), "StealthChop regelt den Strom selbst")}
					${check("pwm_autograd", t.pwmAutograd, (v) => set({ pwmAutograd: v }), "StealthChop passt die Steigung an die Geschwindigkeit an")}
					<label class="stepper-field"><span>Mikroschritte</span><select @change=${(e: Event) => set({ microsteps: Number((e.target as HTMLSelectElement).value) })}>
						${MICROSTEPS.map((m) => html`<option value=${m} ?selected=${m === t.microsteps}>${m}</option>`)}</select></label>
				</fieldset>
				<fieldset><legend>StallGuard / CoolStep</legend>
					${this.num("SGTHRS", t.sgThrs, (v) => set({ sgThrs: Math.max(0, Math.min(255, Math.round(v))) }), { min: 0, max: 255, title: "Stall-Schwelle der Referenzfahrt" })}
					${this.num("TCOOLTHRS", t.tcoolThrs, (v) => set({ tcoolThrs: Math.max(0, Math.round(v)) }), { min: 0, title: "CoolStep und Stall-Ausgang oberhalb dieser Geschwindigkeit (TSTEP ≤ TCOOLTHRS)" })}
					${check("CoolStep (lastabhängiger Strom)", t.coolStep, (v) => set({ coolStep: v }))}
					${t.coolStep ? html`
						${slider("SEMIN", t.semin, 15, (v) => `< ${v * 32}`, (v) => set({ semin: Math.max(1, v) }), "unterhalb SEMIN·32 steigt der Strom")}
						${slider("SEMAX", t.semax, 15, (v) => `> ${(t.semin + v + 1) * 32}`, (v) => set({ semax: v }), "oberhalb (SEMIN+SEMAX+1)·32 sinkt er")}
						${slider("SEUP", t.seup, 3, (v) => `+${2 ** v}`, (v) => set({ seup: v }), "Stromschritt nach oben")}
						${slider("SEDN", t.sedn, 3, (v) => `${[32, 8, 2, 1][v]} SG/Schritt`, (v) => set({ sedn: v }), "Tempo nach unten")}
						${check("SEIMIN: Minimum 1/4 statt 1/2 IRUN", t.seimin, (v) => set({ seimin: v }))}` : nothing}
				</fieldset>
			</div>
			<div class="stepper-actions">
				<button class="stp-primary" ?disabled=${this.busy} @click=${() => this.apply()}>Übernehmen</button>
				<span class="muted">Mikroschritte ändern verschiebt den Maßstab der Position: danach neu referenzieren.</span>
			</div>`;
	}
}

const DEFAULT_SETTINGS: stepper.TmcSettings = {
	irun: 12, ihold: 4, iholdDelay: 6, tpowerdown: 20, spreadCycle: false, tpwmThrs: 0, pwmAutoscale: true, pwmAutograd: true, microsteps: 16,
	sgThrs: 100, tcoolThrs: 0, coolStep: false, semin: 5, semax: 2, seup: 0, sedn: 0, seimin: false,
};

declare global {
	interface HTMLElementTagNameMap {
		"stepper-lab-app": StepperLabApp;
	}
}
