// Aktorentest: direct operation of the plant's actuators - conveyor, compressor, valves (with vacuum) and the RGB
// lighting - with the feedback that belongs to them (light barriers, pressure). A direct-operation page
// (core/direct-operation.ts): it writes the output registers itself, which the device only accepts while the
// fieldbus is the control source; the bar on top says who switches the outputs and takes them over.
import { LitElement, html, nothing } from "lit";
import { customElement, state } from "lit/decorators.js";
import "./actuator-test.css";
import type { WorkspaceApp } from "@klaus-liebler/ui-shell/shell/workspace-app.ts";
import { setAppStatus } from "@klaus-liebler/ui-shell/shell/status-bar.ts";
import { openWorkspace, setAppRibbon } from "@klaus-liebler/ui-shell/shell/ribbon.ts";
import { Poller, pollingGroup } from "../../core/poller.ts";
import { DirectOperation } from "../../core/direct-operation.ts";
import { fetchRegisters, register, regBool, regValue, type RegisterValues } from "../../core/registers.ts";

/** The PWM outputs (per mille). */
const PWMS = [
	{ reg: "CONVEYOR_PWM", name: "Förderband", hint: "Motor des Förderbands" },
	{ reg: "COMPRESSOR_PWM", name: "Kompressor", hint: "Kompressor des Druckspeichers" },
] as const;

/** The switching outputs. */
const SWITCHES = [
	{ reg: "VALVE1", name: "Ventil 1" },
	{ reg: "VALVE2", name: "Ventil 2" },
	{ reg: "VALVE3", name: "Ventil 3" },
	{ reg: "VACUUM_VALVE", name: "Vakuumventil" },
	{ reg: "VACUUM_PUMP", name: "Vakuumpumpe" },
] as const;

const PULSE_MS = 500;

// The colours of the WS2812 outputs as the firmware shows them (src/io.cpp, leds()): channel 1 is a strip of
// 8 LEDs with 5 patterns (pattern % 5), channel 2 one LED with an 8-colour palette (pattern % 8).
const STRIP_PATTERNS: { name: string; leds: string[] }[] = [
	{ name: "Aus", leds: Array(8).fill("#000000") },
	{ name: "Rot", leds: Array(8).fill("#ff0000") },
	{ name: "Gelb", leds: Array(8).fill("#ffff00") },
	{ name: "Grün", leds: Array(8).fill("#00ff00") },
	{ name: "Blau/Weiß", leds: Array.from({ length: 8 }, (_, i) => (i % 2 === 0 ? "#0000ff" : "#ffffff")) },
];
const SINGLE_PALETTE: { name: string; color: string }[] = [
	{ name: "Aus", color: "#000000" }, { name: "Rot", color: "#ff0000" }, { name: "Grün", color: "#00ff00" }, { name: "Blau", color: "#0000ff" },
	{ name: "Weiß", color: "#ffffff" }, { name: "Gelb", color: "#ffff00" }, { name: "Cyan", color: "#00ffff" }, { name: "Magenta", color: "#ff00ff" },
];

@customElement("actuator-test-app")
export class ActuatorTestApp extends LitElement implements WorkspaceApp {
	@state() private values: RegisterValues | null = null;
	/** A PWM slider being dragged: its value until released. */
	@state() private dragging: { reg: string; value: number } | null = null;
	private readonly poller = new Poller(this, () => this.poll(), 1000);
	private readonly direct = new DirectOperation(this, () => this.poller.refresh());

	protected createRenderRoot() {
		return this;
	}

	onShow(): void {
		this.poller.start();
	}

	onHide(): void {
		this.poller.stop();
	}

	private async poll(): Promise<void> {
		this.values = await fetchRegisters();
		await this.direct.refresh();
	}

	/** No direct operation now: the PLC switches the outputs, or the values or the control source are unknown. */
	private get blocked(): boolean {
		return this.direct.locked || !this.values || !this.direct.plc;
	}

	private max(reg: string): number {
		return register(reg).max ?? 1000;
	}

	private setPwm(reg: string, permille: number): void {
		this.dragging = null;
		void this.direct.write(reg, Math.round(Math.min(this.max(reg), Math.max(0, permille))));
	}

	private toggle(reg: string): void {
		void this.direct.write(reg, regBool(this.values, reg) ? 0 : 1);
	}

	/** Switches the output on for PULSE_MS, then off (a valve test). */
	private async pulse(reg: string): Promise<void> {
		if (!(await this.direct.write(reg, 1, false))) return;
		setTimeout(() => void this.direct.write(reg, 0), PULSE_MS);
	}

	/** Every actuator of the page to its rest state. */
	private allOff(): void {
		for (const p of PWMS) void this.direct.write(p.reg, 0, false);
		for (const s of SWITCHES) void this.direct.write(s.reg, 0, false);
		void this.direct.write("WS2812_CH1_PATTERN", 0, false);
		void this.direct.write("WS2812_CH2_PATTERN", 0);
	}

	protected willUpdate(): void {
		setAppStatus(this, this.direct.error ? { text: this.direct.error, kind: "error" } : this.poller.status("Lade Ausgänge …"));
		const locked = this.direct.locked;
		const blocked = this.blocked;
		setAppRibbon(this, [
			{ id: "actuators", label: "Aktoren", groups: [
				{ label: "Steuerquelle", commands: [
					{ id: "take", label: "Ausgänge übernehmen", icon: "fieldbus", disabled: !locked, title: "Steuerquelle auf Feldbus schalten (das SPS-Programm stoppt)", run: () => void this.direct.takeOver() },
					{ id: "plc", label: "SPS-Entwicklung", icon: "plc", size: "small", run: () => openWorkspace("/plc") },
				] },
				{ label: "Sicherheit", commands: [
					{ id: "off", label: "Alles aus", icon: "stop", disabled: blocked, title: "Förderband, Kompressor, Ventile, Vakuum und Beleuchtung aus", run: () => this.allOff() },
				] },
				{ label: "Förderband", commands: [
					{ id: "conv-on", label: "100 %", icon: "run", disabled: blocked, run: () => this.setPwm("CONVEYOR_PWM", 1000) },
					{ id: "conv-off", label: "Aus", icon: "stop", size: "small", disabled: blocked, run: () => this.setPwm("CONVEYOR_PWM", 0) },
				] },
				pollingGroup(this.poller, "die Ausgänge"),
			] },
		]);
	}

	render() {
		const locked = this.blocked;
		return html`
			<div class="container act">
				${this.direct.bar()}
				<div class="act-grid">
					${PWMS.map((p) => this.renderPwm(p.reg, p.name, p.hint, locked))}
					${this.renderSwitches(locked)}
					${this.renderLighting(locked)}
				</div>
			</div>`;
	}

	private renderPwm(reg: string, name: string, hint: string, locked: boolean) {
		const max = this.max(reg);
		const actual = regValue(this.values, reg);
		const shown = this.dragging?.reg === reg ? this.dragging.value : actual ?? 0;
		const lb = (n: 1 | 2 | 3) => regBool(this.values, `LIGHTBARRIER${n}`);
		const pressure = regValue(this.values, "PRESSURE_RAW");
		return html`
			<section class="panel-section act-card">
				<div class="act-head"><b>${name}</b><span class="act-hint">${hint}</span>
					<span class="act-value ${shown ? "act-on" : ""}">${actual === undefined ? "–" : `${(shown / 10).toFixed(1)} %`}</span></div>
				<input type="range" min="0" max=${max} step="10" .value=${String(shown)} ?disabled=${locked || actual === undefined}
					@input=${(e: Event) => (this.dragging = { reg, value: Number((e.target as HTMLInputElement).value) })}
					@change=${(e: Event) => this.setPwm(reg, Number((e.target as HTMLInputElement).value))} />
				<div class="act-buttons">
					${[0, 250, 500, 750, 1000].map((v) => html`<button class=${actual === v ? "selected" : ""} ?disabled=${locked} @click=${() => this.setPwm(reg, v)}>${v === 0 ? "Aus" : `${v / 10} %`}</button>`)}
				</div>
				${reg === "CONVEYOR_PWM" ? html`<div class="act-feedback">
					${([1, 2, 3] as const).map((n) => html`<span class="led-item"><span class="led ${lb(n) ? "on" : ""}"></span>Lichtschranke ${n}</span>`)}
				</div>` : html`<div class="act-feedback">Druck (roh): <b>${pressure ?? "–"}</b> / 4095
					<span class="meter"><span style="width:${((pressure ?? 0) / 4095) * 100}%"></span></span></div>`}
			</section>`;
	}

	private renderSwitches(locked: boolean) {
		return html`
			<section class="panel-section act-card">
				<div class="act-head"><b>Ventile &amp; Vakuum</b><span class="act-hint">Ein/Aus oder Impuls ${PULSE_MS} ms</span></div>
				${SWITCHES.map((s) => {
					const on = regBool(this.values, s.reg);
					return html`<div class="act-switch">
						<span class="led ${on ? "on amber" : ""}"></span>
						<span class="act-switch-name">${s.name}</span>
						<button class="act-toggle ${on ? "selected" : ""}" ?disabled=${locked || on === undefined} @click=${() => this.toggle(s.reg)}>${on ? "Ein" : "Aus"}</button>
						<button ?disabled=${locked || on === undefined || on} title="${PULSE_MS} ms einschalten" @click=${() => void this.pulse(s.reg)}>Impuls</button>
					</div>`;
				})}
			</section>`;
	}

	private renderLighting(locked: boolean) {
		const p1 = regValue(this.values, "WS2812_CH1_PATTERN");
		const p2 = regValue(this.values, "WS2812_CH2_PATTERN");
		const strip = p1 === undefined ? undefined : p1 % STRIP_PATTERNS.length;
		const single = p2 === undefined ? undefined : p2 % SINGLE_PALETTE.length;
		return html`
			<section class="panel-section act-card act-wide">
				<div class="act-head"><b>RGB-Beleuchtung</b><span class="act-hint">WS2812: Kanal 1 Streifen (8 LEDs, Muster), Kanal 2 Einzel-LED (Farbe)</span></div>
				<div class="act-sub">Kanal 1 - Streifen ${strip !== undefined ? html`<span class="act-hint">aktiv: ${STRIP_PATTERNS[strip].name}</span>` : nothing}</div>
				<div class="act-swatches">
					${STRIP_PATTERNS.map((pat, i) => html`<button class="act-pattern ${strip === i ? "selected" : ""}" ?disabled=${locked || p1 === undefined}
						title=${pat.name} @click=${() => void this.direct.write("WS2812_CH1_PATTERN", i)}>
						<span class="act-strip">${pat.leds.map((c) => html`<i style="background:${c}"></i>`)}</span><span>${pat.name}</span></button>`)}
				</div>
				<div class="act-sub">Kanal 2 - Einzel-LED ${single !== undefined ? html`<span class="act-hint">aktiv: ${SINGLE_PALETTE[single].name}</span>` : nothing}</div>
				<div class="act-swatches">
					${SINGLE_PALETTE.map((c, i) => html`<button class="act-color ${single === i ? "selected" : ""}" ?disabled=${locked || p2 === undefined}
						title=${c.name} @click=${() => void this.direct.write("WS2812_CH2_PATTERN", i)}><i style="background:${c.color}"></i><span>${c.name}</span></button>`)}
				</div>
			</section>`;
	}
}

declare global {
	interface HTMLElementTagNameMap {
		"actuator-test-app": ActuatorTestApp;
	}
}
