import { LitElement, html } from "lit";
import { customElement, state } from "lit/decorators.js";
import "../styles.css";
import { fetchSystemInfo, type SystemInfo } from "../api.js";
import type { DashboardApp } from "../shell/dashboard-app.js";
import * as buildInfo from "../generated/build-info.js";

// Muss zu ResetCause in Core/Src/app.hh passen.
const RESET_CAUSE_LABELS: Record<number, string> = {
	0: "Unbekannt",
	1: "Independent Watchdog",
	2: "Window Watchdog",
	3: "Software",
	4: "Low-Power",
	5: "Brown-Out",
	6: "Reset-Pin / Power-On",
};

function formatUptime(totalSeconds: number): string {
	const days = Math.floor(totalSeconds / 86400);
	const hours = Math.floor((totalSeconds % 86400) / 3600);
	const minutes = Math.floor((totalSeconds % 3600) / 60);
	const seconds = totalSeconds % 60;
	const parts: string[] = [];
	if (days > 0) parts.push(`${days} d`);
	if (days > 0 || hours > 0) parts.push(`${hours} h`);
	if (days > 0 || hours > 0 || minutes > 0) parts.push(`${minutes} min`);
	parts.push(`${seconds} s`);
	return parts.join(" ");
}

@customElement("system-info-app")
export class SystemInfoApp extends LitElement implements DashboardApp {
	protected createRenderRoot() {
		return this;
	}

	@state() private info: SystemInfo | null = null;
	@state() private statusMessage = "Lade Systemdaten...";
	@state() private statusVariant: "warning" | "success" | "error" = "warning";
	@state() private loading = false;

	private shown = false;

	// Kein Polling wie bei den anderen beiden Apps: Systemdaten aendern sich kaum, ein einmaliger
	// Fetch beim Oeffnen (+ manueller "Aktualisieren"-Button) genuegt.
	onShow(): void {
		if (this.shown) return;
		this.shown = true;
		void this.refresh();
	}

	onHide(): void {
		// Nichts zu stoppen -- kein Poll-Timer.
	}

	private async refresh() {
		this.loading = true;
		try {
			this.info = await fetchSystemInfo();
			this.statusMessage = `Verbunden -- zuletzt aktualisiert ${new Date().toLocaleTimeString("de-DE")}`;
			this.statusVariant = "success";
		} catch (error) {
			console.error("Systemdaten-Abfrage fehlgeschlagen", error);
			this.statusMessage = "Verbindungsproblem zur Control-Unit";
			this.statusVariant = "error";
		} finally {
			this.loading = false;
		}
	}

	private row(name: string, value: string | number) {
		return html`<tr><td>${name}</td><td>${value}</td></tr>`;
	}

	render() {
		const info = this.info;
		return html`
			<div class="container">
				<section class="header-section app-panel">
					<div class="panel-label">System</div>
					<div class="status-text status-${this.statusVariant}">${this.statusMessage}</div>
				</section>

				<div class="region-grid">
					<div class="panel-section">
						<div class="panel-label">Build (Web-UI)</div>
						<table class="register-table">
							<tbody>
								${this.row("Git-Commit", buildInfo.GIT_COMMIT_HASH)}
								${this.row("Branch", buildInfo.GIT_BRANCH)}
								${this.row("Tag", buildInfo.GIT_TAG)}
								${this.row("Arbeitsverzeichnis", buildInfo.GIT_IS_DIRTY ? "geaendert (dirty)" : "sauber")}
								${this.row("Letzter Commit", buildInfo.GIT_COMMIT_DATE)}
								${this.row("Web-UI gebaut am", new Date(buildInfo.BUILD_TIMESTAMP).toLocaleString("de-DE"))}
							</tbody>
						</table>
					</div>

					<div class="panel-section">
						<div class="panel-label">
							Laufzeit (Control-Unit)
							<button class="system-refresh-btn" @click=${() => this.refresh()} ?disabled=${this.loading}>
								${this.loading ? "Lädt..." : "Aktualisieren"}
							</button>
						</div>
						${info
							? html`
									<table class="register-table">
										<tbody>
											${this.row("Board", info.boardName)}
											${this.row("Hostname", `${info.hostname}.local`)}
											${this.row("Firmware-Version", `${info.fwVersionMajor}.${info.fwVersionMinor}.${info.fwVersionPatch}`)}
											${this.row("Uptime", formatUptime(info.uptimeSeconds))}
											${this.row("Freier Heap", `${(info.freeHeapBytes / 1024).toFixed(1)} KiB`)}
											${this.row("IP-Adresse", info.ipAddress)}
											${this.row("Netzmaske", info.netMask)}
											${this.row("MAC-Adresse", info.macAddress)}
											${this.row("Chip-UID", info.chipUid)}
											${this.row("Letzter Reset", RESET_CAUSE_LABELS[info.resetCauseCode] ?? "Unbekannt")}
										</tbody>
									</table>
								`
							: html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>
				</div>
			</div>
		`;
	}
}
