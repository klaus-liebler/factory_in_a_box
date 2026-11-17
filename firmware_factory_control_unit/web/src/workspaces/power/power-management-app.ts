// Power management: bus voltage, current and power of the INA226 as time series (canvas charts drawn by
// line-chart.ts, no chart library), sampled every second while the workspace is shown.
import { LitElement, html } from "lit";
import { customElement, state, query } from "lit/decorators.js";
import "./power-management.css";
import { fetchRegisters, regValue } from "../../core/registers.ts";
import { Poller } from "../../core/poller.ts";
import type { WorkspaceApp } from "@fiab/ui-shell/shell/workspace-app.ts";
import { drawLineChart } from "@fiab/ui-shell/ui/line-chart.ts";
import { setAppStatus } from "@fiab/ui-shell/shell/status-bar.ts";
import { openWorkspace, setAppRibbon } from "@fiab/ui-shell/shell/ribbon.ts";

// The longest selectable window is also the buffer size: up to 60 minutes are always kept, so switching to a
// longer window shows more history at once.
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

@customElement("power-management-app")
export class PowerManagementApp extends LitElement implements WorkspaceApp {
	protected willUpdate(): void {
		setAppStatus(this, this.poller.status("Lade Messwerte …"));
		setAppRibbon(this, [
			{ id: "power", label: "Messung", groups: [
				{ label: "Zeitfenster", commands: DURATION_OPTIONS.map((o) => ({
					id: `d${o.seconds}`, label: o.label, icon: "chart" as const, active: this.durationSeconds === o.seconds,
					title: `Die letzten ${o.label} zeigen (aufgezeichnet werden immer bis zu 60 min)`, run: () => this.selectDuration(o.seconds),
				})) },
				{ label: "Versorgung", commands: [
					{ id: "dash", label: "Dashboard", icon: "dashboard", title: "USB-PD und Gesamtzustand", run: () => openWorkspace("/") },
				] },
			] },
		]);
	}

	protected createRenderRoot() {
		return this;
	}

	@state() private durationSeconds = 300;
	@state() private latest: Sample | null = null;

	@query(".chart-voltage") private voltageCanvas!: HTMLCanvasElement;
	@query(".chart-current") private currentCanvas!: HTMLCanvasElement;
	@query(".chart-power") private powerCanvas!: HTMLCanvasElement;

	private samples: Sample[] = [];
	private readonly poller = new Poller(this, () => this.poll(), 1000);
	private readonly onResize = () => this.redraw();

	onShow(): void {
		window.addEventListener("resize", this.onResize);
		this.poller.start();
		this.updateComplete.then(() => this.redraw());
	}

	onHide(): void {
		window.removeEventListener("resize", this.onResize);
		this.poller.stop();
	}

	private async poll(): Promise<void> {
		const values = await fetchRegisters();
		const u = regValue(values, "PWR_BUS_VOLTAGE_MV"), i = regValue(values, "PWR_CURRENT_MA"), p = regValue(values, "PWR_POWER_MW");
		if (u === undefined || i === undefined || p === undefined) throw new Error("power registers missing in the answer");
		const sample: Sample = { t: Date.now(), voltage: u / 1000, current: i / 1000, power: p / 1000 };
		this.samples.push(sample);
		const cutoff = sample.t - MAX_BUFFER_SECONDS * 1000;
		while (this.samples.length > 0 && this.samples[0].t < cutoff) this.samples.shift();
		// "latest" is a state: the render it causes calls updated() -> redraw()
		this.latest = sample;
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
