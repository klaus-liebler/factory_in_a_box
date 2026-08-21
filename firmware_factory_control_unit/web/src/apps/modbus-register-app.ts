import { LitElement, html } from "lit";
import { customElement, state } from "lit/decorators.js";
import "../styles.css";
import "../register-panel.js";
import { REGIONS } from "../../generated/register-map.js";
import { fetchRegisters, writeHolding, type RegisterValues } from "../registers.js";
import type { DashboardApp } from "../shell/dashboard-app.js";

const POLL_INTERVAL_MS = 1000;

@customElement("modbus-register-app")
export class ModbusRegisterApp extends LitElement implements DashboardApp {
	protected createRenderRoot() {
		return this;
	}

	@state() private values: RegisterValues = { holding: [], input: [] };
	@state() private statusMessage = "Lade Register...";
	@state() private statusVariant: "warning" | "success" | "error" = "warning";

	private pollTimeoutHandle?: ReturnType<typeof setTimeout>;
	private stopped = true;

	// Ersetzt connectedCallback()/disconnectedCallback() als Start/Stop-Signal: das Element
	// bleibt jetzt dauerhaft im DOM (s. app-shell.ts), Polling laeuft daher nur, waehrend diese
	// App auch tatsaechlich sichtbar/aktiv ist.
	onShow(): void {
		this.stopped = false;
		this.scheduleNextPoll(0);
	}

	onHide(): void {
		this.stopped = true;
		if (this.pollTimeoutHandle) {
			clearTimeout(this.pollTimeoutHandle);
		}
	}

	private scheduleNextPoll(delayMs: number) {
		if (this.stopped) {
			return;
		}
		this.pollTimeoutHandle = setTimeout(() => {
			void this.poll().finally(() => this.scheduleNextPoll(POLL_INTERVAL_MS));
		}, delayMs);
	}

	// Bewusst NIE ueberlappend: der naechste Poll wird erst geplant, NACHDEM dieser
	// abgeschlossen ist (siehe scheduleNextPoll), statt per setInterval fest im Takt zu
	// feuern -- vermeidet mehrere gleichzeitig ausstehende Requests auf derselben (einzigen)
	// WebSocket-Verbindung.
	private async poll() {
		try {
			this.values = await fetchRegisters();
			const now = new Date().toLocaleTimeString("de-DE");
			this.statusMessage = `Verbunden -- zuletzt aktualisiert ${now}`;
			this.statusVariant = "success";
		} catch (error) {
			console.error("Register-Abfrage fehlgeschlagen", error);
			this.statusMessage = "Verbindungsproblem zur Control-Unit";
			this.statusVariant = "error";
		}
	}

	private async handleWriteRegister(ev: Event) {
		const { address, value } = (ev as CustomEvent<{ address: number; value: number }>).detail;
		try {
			await writeHolding(address, value);
			await this.poll();
		} catch (error) {
			console.error("Schreiben fehlgeschlagen", error);
			this.statusMessage = "Schreiben fehlgeschlagen";
			this.statusVariant = "error";
		}
	}

	render() {
		return html`
			<div class="container">
				<section class="header-section app-panel">
					<div class="panel-label">Modbus Register Console</div>
					<div class="status-text status-${this.statusVariant}">${this.statusMessage}</div>
				</section>

				<div class="region-grid" @write-register=${this.handleWriteRegister}>
					${REGIONS.map(
						(region) => html`
							<register-panel
								.region=${region}
								.holdingValues=${this.values.holding}
								.inputValues=${this.values.input}
							></register-panel>
						`
					)}
				</div>
			</div>
		`;
	}
}
