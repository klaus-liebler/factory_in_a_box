import { LitElement, html } from "lit";
import { customElement, state } from "lit/decorators.js";
import "../styles.css";
import "../register-panel.js";
import { REGIONS } from "../../generated/register-map.js";
import { fetchRegisters, writeHolding, type RegisterValues } from "../registers.js";
import type { DashboardApp } from "../shell/dashboard-app.js";
import { kindOf, setAppStatus } from "../shell/status-bar.js";
import { plc } from "../../generated/ws-protocol.js";
import { wsRequest } from "../ws-client.js";

const POLL_INTERVAL_MS = 1000;

@customElement("modbus-register-app")
export class ModbusRegisterApp extends LitElement implements DashboardApp {
	/** The state goes to the application's status bar (shell/status-bar.ts), not into the page. */
	protected willUpdate(): void {
		setAppStatus(this, {
			text: this.statusMessage, kind: kindOf(this.statusVariant),
			extra: this.iec ? html`<span title="Das SPS-Programm schreibt die Ausgänge; schreibbar sind nur Parameter. Umschalten auf Feldbus in der SPS-Seite.">Steuerquelle IEC 61131: Ausgänge gesperrt</span>` : null,
		});
	}

	/** The IEC 61131 program is the control source: only parameter registers are writable. */
	@state() private iec = false;

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
			const status = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.StatusRequest.encode({ requestId }), plc.StatusResponse);
			this.iec = status.source === plc.ControlSource.Iec;
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
			// refused although offered: the control source changed to IEC since the last poll
			this.statusMessage = this.iec || (await this.iecNow())
				? `Register ${address} nicht geschrieben: das SPS-Programm ist Steuerquelle (nur Parameter schreibbar)`
				: `Register ${address} nicht geschrieben`;
			this.statusVariant = "error";
		}
	}

	private async iecNow(): Promise<boolean> {
		try {
			const s = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.StatusRequest.encode({ requestId }), plc.StatusResponse);
			this.iec = s.source === plc.ControlSource.Iec;
		} catch {
			/* keep what we knew */
		}
		return this.iec;
	}

	render() {
		return html`
			<div class="container">
				<section class="header-section app-panel">
					<div class="panel-label">Register-Konsole</div>
				</section>

				<div class="region-grid" @write-register=${this.handleWriteRegister}>
					${REGIONS.map(
						(region) => html`
							<register-panel
								.region=${region}
								.holdingValues=${this.values.holding}
								.inputValues=${this.values.input}
								.iecControlled=${this.iec}
							></register-panel>
						`
					)}
				</div>
			</div>
		`;
	}
}
