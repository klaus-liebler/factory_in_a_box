import { LitElement, html } from "lit";
import { customElement, state, query } from "lit/decorators.js";
import "../styles.css";
import { REGIONS, type RegisterDef } from "../../generated/register-map.js";
import { fetchRegisters, type RegisterValues } from "../registers.js";
import type { DashboardApp } from "../shell/dashboard-app.js";
import { drawLineChart } from "./line-chart.js";

const POLL_INTERVAL_MS = 1000;

// Laengste waehlbare Zeitspanne bestimmt zugleich die Ringpuffer-Groesse: der Puffer haelt immer
// bis zu 60 Minuten vor, auch wenn gerade ein kuerzeres Zeitfenster angezeigt wird -- Umschalten
// auf ein laengeres Fenster zeigt so sofort mehr Historie, ohne dass neu aufgezeichnet werden
// muss.
const DURATION_OPTIONS: { label: string; seconds: number }[] = [
	{ label: "1 min", seconds: 60 },
	{ label: "5 min", seconds: 300 },
	{ label: "15 min", seconds: 900 },
	{ label: "60 min", seconds: 3600 },
];
const MAX_BUFFER_SECONDS = DURATION_OPTIONS[DURATION_OPTIONS.length - 1].seconds;

interface Sample {
	t: number;
	voltage: number;
	current: number;
	power: number;
}

// Adressen kommen aus register-map.ts (generiert aus register-map.json) statt als Magic Numbers
// hier zu stehen -- bleiben so automatisch in Sync, falls sich die Register-Map je aendert.
function findRegister(name: string): RegisterDef {
	for (const region of REGIONS) {
		const reg = region.registers.find((r) => r.name === name);
		if (reg) return reg;
	}
	throw new Error(`Register ${name} nicht in register-map.ts gefunden`);
}

const VOLTAGE_REG = findRegister("PWR_BUS_VOLTAGE_MV");
const CURRENT_REG = findRegister("PWR_CURRENT_MA");
const POWER_REG = findRegister("PWR_POWER_MW");

function readRegister(values: RegisterValues, reg: RegisterDef): number | undefined {
	const source = reg.bank === "holding" ? values.holding : values.input;
	const raw = source[reg.address];
	if (raw === undefined) return undefined;
	if (reg.signed && raw > 0x7fff) return raw - 0x10000;
	return raw;
}

@customElement("power-management-app")
export class PowerManagementApp extends LitElement implements DashboardApp {
	protected createRenderRoot() {
		return this;
	}

	@state() private statusMessage = "Lade Messwerte...";
	@state() private statusVariant: "warning" | "success" | "error" = "warning";
	@state() private durationSeconds = 300;
	@state() private latest: Sample | null = null;

	@query(".chart-voltage") private voltageCanvas!: HTMLCanvasElement;
	@query(".chart-current") private currentCanvas!: HTMLCanvasElement;
	@query(".chart-power") private powerCanvas!: HTMLCanvasElement;

	private samples: Sample[] = [];
	private pollTimeoutHandle?: ReturnType<typeof setTimeout>;
	private stopped = true;
	private readonly onResize = () => this.redraw();

	onShow(): void {
		this.stopped = false;
		window.addEventListener("resize", this.onResize);
		this.scheduleNextPoll(0);
		this.updateComplete.then(() => this.redraw());
	}

	onHide(): void {
		this.stopped = true;
		window.removeEventListener("resize", this.onResize);
		if (this.pollTimeoutHandle) {
			clearTimeout(this.pollTimeoutHandle);
		}
	}

	private scheduleNextPoll(delayMs: number) {
		if (this.stopped) return;
		this.pollTimeoutHandle = setTimeout(() => {
			void this.poll().finally(() => this.scheduleNextPoll(POLL_INTERVAL_MS));
		}, delayMs);
	}

	private async poll() {
		try {
			const values = await fetchRegisters();
			const voltageRaw = readRegister(values, VOLTAGE_REG);
			const currentRaw = readRegister(values, CURRENT_REG);
			const powerRaw = readRegister(values, POWER_REG);
			if (voltageRaw === undefined || currentRaw === undefined || powerRaw === undefined) {
				throw new Error("Power-Register fehlen in der Antwort");
			}

			const sample: Sample = {
				t: Date.now(),
				voltage: voltageRaw / 1000,
				current: currentRaw / 1000,
				power: powerRaw / 1000,
			};
			this.samples.push(sample);
			const cutoff = sample.t - MAX_BUFFER_SECONDS * 1000;
			while (this.samples.length > 0 && this.samples[0].t < cutoff) {
				this.samples.shift();
			}
			this.latest = sample;

			const now = new Date(sample.t).toLocaleTimeString("de-DE");
			this.statusMessage = `Verbunden -- zuletzt aktualisiert ${now}`;
			this.statusVariant = "success";
		} catch (error) {
			console.error("Power-Abfrage fehlgeschlagen", error);
			this.statusMessage = "Verbindungsproblem zur Control-Unit";
			this.statusVariant = "error";
		}
		// Kein manueller redraw() hier noetig: "latest" ist @state, die dadurch ausgeloeste
		// Re-Render-Runde ruft updated() -> redraw() ohnehin auf.
	}

	private visibleSamples(): Sample[] {
		const cutoff = Date.now() - this.durationSeconds * 1000;
		return this.samples.filter((s) => s.t >= cutoff);
	}

	private redraw(): void {
		const visible = this.visibleSamples();
		if (this.voltageCanvas) {
			drawLineChart(this.voltageCanvas, visible.map((s) => ({ t: s.t, v: s.voltage })), { color: "#2a78d6", unit: "V", decimals: 2 });
		}
		if (this.currentCanvas) {
			drawLineChart(this.currentCanvas, visible.map((s) => ({ t: s.t, v: s.current })), { color: "#e34948", unit: "A", decimals: 3 });
		}
		if (this.powerCanvas) {
			drawLineChart(this.powerCanvas, visible.map((s) => ({ t: s.t, v: s.power })), { color: "#008300", unit: "W", decimals: 2 });
		}
	}

	private selectDuration(seconds: number) {
		this.durationSeconds = seconds;
	}

	protected updated(): void {
		this.redraw();
	}

	render() {
		return html`
			<div class="container">
				<section class="header-section app-panel">
					<div class="panel-label">Power Management</div>
					<div class="status-text status-${this.statusVariant}">${this.statusMessage}</div>
				</section>

				<div class="panel-section power-toolbar">
					<span class="panel-label">Zeitfenster</span>
					<div class="duration-buttons">
						${DURATION_OPTIONS.map(
							(opt) => html`
								<button
									class=${this.durationSeconds === opt.seconds ? "duration-btn duration-btn-active" : "duration-btn"}
									@click=${() => this.selectDuration(opt.seconds)}
								>${opt.label}</button>
							`
						)}
					</div>
				</div>

				<div class="power-chart-grid">
					<div class="panel-section power-chart-card">
						<div class="panel-label">Spannung ${this.latest ? html`<span class="power-readout">${this.latest.voltage.toFixed(2)} V</span>` : ""}</div>
						<canvas class="chart-voltage power-chart-canvas"></canvas>
					</div>
					<div class="panel-section power-chart-card">
						<div class="panel-label">Strom ${this.latest ? html`<span class="power-readout">${this.latest.current.toFixed(3)} A</span>` : ""}</div>
						<canvas class="chart-current power-chart-canvas"></canvas>
					</div>
					<div class="panel-section power-chart-card">
						<div class="panel-label">Leistung ${this.latest ? html`<span class="power-readout">${this.latest.power.toFixed(2)} W</span>` : ""}</div>
						<canvas class="chart-power power-chart-canvas"></canvas>
					</div>
				</div>
			</div>
		`;
	}
}
