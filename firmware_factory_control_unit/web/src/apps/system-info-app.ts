import { LitElement, html } from "lit";
import { customElement, state } from "lit/decorators.js";
import "../styles.css";
import { fetchSystemInfo, type SystemInfo } from "../api.js";
import type { DashboardApp } from "../shell/dashboard-app.js";
import * as buildInfo from "../../generated/build-info.js";
import { I2C_ADDRESS_CANDIDATES } from "../i2c-device-names.js";

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

// Alle Zeitpunkte kommen als Unix-Epoch-Sekunden (int64-tauglich, s. builder/GitInfo.cs) --
// hier und NUR hier, am Anzeigeort, in die lokale Zeitzone des Browsers umgerechnet
// (toLocaleString() nutzt implizit Intl mit der System-/Browser-Zeitzone).
function formatEpochSeconds(epochSeconds: number): string {
	return new Date(epochSeconds * 1000).toLocaleString("de-DE");
}

// LAN8742-Bitmasken (s. Drivers/BSP/Components/lan8742/lan8742.h) -- Firmware liefert die
// Register nur noch roh (s. Core/Src/webserver.cpp PhyRegisters), das Parsen/Interpretieren
// passiert bewusst hier, client-seitig.
const LAN8742_BSR_LINK_STATUS = 0x0004;
const LAN8742_PHYSCSR_AUTONEGO_DONE = 0x1000;
const LAN8742_PHYSCSR_HCDSPEEDMASK = 0x001c;
const LAN8742_PHYSCSR_10BT_HD = 0x0004;
const LAN8742_PHYSCSR_10BT_FD = 0x0014;
const LAN8742_PHYSCSR_100BTX_HD = 0x0008;
const LAN8742_PHYSCSR_100BTX_FD = 0x0018;
const LAN8742_MCSR_ENERGYON = 0x0002;
const LAN8742_SCSIR_AUTO_MDIX_ENABLE = 0x8000;
const LAN8742_SCSIR_XPOLALITY = 0x0010;
const LAN8742_TCSR_TDR_CH_CABLE_TYPE = 0x0600;
const LAN8742_TCSR_TDR_CH_CABLE_DEFAULT = 0x0000;
const LAN8742_TCSR_TDR_CH_CABLE_SHORTED = 0x0200;
const LAN8742_TCSR_TDR_CH_CABLE_OPEN = 0x0400;
const LAN8742_TCSR_TDR_CH_LENGTH = 0x00ff;
const LAN8742_CLR_CABLE_LENGTH = 0xf000;

const SPEED_DUPLEX_LABELS: Record<number, string> = {
	[LAN8742_PHYSCSR_10BT_HD]: "10 MBit/s Halbduplex",
	[LAN8742_PHYSCSR_10BT_FD]: "10 MBit/s Vollduplex",
	[LAN8742_PHYSCSR_100BTX_HD]: "100 MBit/s Halbduplex",
	[LAN8742_PHYSCSR_100BTX_FD]: "100 MBit/s Vollduplex",
};

function cableTypeLabel(tcsr: number): string {
	switch (tcsr & LAN8742_TCSR_TDR_CH_CABLE_TYPE) {
		case LAN8742_TCSR_TDR_CH_CABLE_DEFAULT:
			return "unklar (noch nicht gemessen)";
		case LAN8742_TCSR_TDR_CH_CABLE_SHORTED:
			return "Kurzschluss";
		case LAN8742_TCSR_TDR_CH_CABLE_OPEN:
			return "Leitungsunterbrechung";
		default:
			return "angepasst (kein Fehler erkannt)";
	}
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

	private renderPhySection(phy: SystemInfo["phy"]) {
		if (!phy.readOk) {
			return html`<p class="panel-text status-error">PHY nicht erreichbar (MDIO-Lesefehler).</p>`;
		}
		const linkUp = (phy.bsr & LAN8742_BSR_LINK_STATUS) !== 0;
		const autonegDone = (phy.physcsr & LAN8742_PHYSCSR_AUTONEGO_DONE) !== 0;
		const speedDuplex = SPEED_DUPLEX_LABELS[phy.physcsr & LAN8742_PHYSCSR_HCDSPEEDMASK] ?? "unbekannt";
		const energyDetected = (phy.mcsr & LAN8742_MCSR_ENERGYON) !== 0;
		const autoMdix = (phy.scsir & LAN8742_SCSIR_AUTO_MDIX_ENABLE) !== 0;
		const polarityReversed = (phy.scsir & LAN8742_SCSIR_XPOLALITY) !== 0;
		const cableLengthClass = (phy.clr & LAN8742_CLR_CABLE_LENGTH) >> 12;
		return html`
			<table class="register-table">
				<tbody>
					${this.row("Link", linkUp ? "up" : "down")}
					${this.row("Auto-Negotiation", autonegDone ? "abgeschlossen" : "läuft/nicht abgeschlossen")}
					${this.row("Geschwindigkeit/Duplex", speedDuplex)}
					${this.row("Energie am Kabel (ENERGYON)", energyDetected ? "ja" : "nein")}
					${this.row("Auto-MDIX", autoMdix ? "aktiv" : "inaktiv")}
					${this.row("Polarität", polarityReversed ? "invertiert (korrigiert)" : "normal")}
					${phy.tdrAvailable
						? html`
								${this.row("Kabeldiagnose (TDR)", cableTypeLabel(phy.tcsr))}
								${this.row("TDR-Fehlerstelle (roh, unkalibriert)", phy.tcsr & LAN8742_TCSR_TDR_CH_LENGTH)}
								${this.row("Kabellängen-Klasse (0-15, nur bei Link)", cableLengthClass)}
							`
						: this.row("Kabeldiagnose (TDR)", "nicht verfügbar")}
				</tbody>
			</table>
		`;
	}

	private renderI2cBusSection(busLabel: string, addresses: boolean[]) {
		const found = addresses
			.map((present, addr) => ({ present, addr }))
			.filter((entry) => entry.present);
		return html`
			<div class="panel-text"><strong>${busLabel}</strong></div>
			${found.length === 0
				? html`<p class="panel-text">Keine Geräte gefunden.</p>`
				: html`
						<table class="register-table">
							<tbody>
								${found.map(
									(entry) => html`
										<tr>
											<td>0x${entry.addr.toString(16).padStart(2, "0")}</td>
											<td>${I2C_ADDRESS_CANDIDATES[entry.addr] ?? "unbekannt"}</td>
										</tr>
									`
								)}
							</tbody>
						</table>
					`}
		`;
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
					<!-- Board/Hostname/Chip-UID/MAC/Zertifikat sind Compile-Zeit-Konstanten
					     (s. builder/Phases/ReadGitStatus.cs) -- kommen direkt aus build-info.ts, keine
					     Laufzeit-Abfrage noetig, deshalb ohne "Aktualisieren"-Button/Ladezustand. -->
					<div class="panel-section">
						<div class="panel-label">Board Info</div>
						<table class="register-table">
							<tbody>
								${this.row("Board", buildInfo.BOARD_NAME)}
								${this.row("Hostname", `${buildInfo.DEVICE_HOSTNAME}.local`)}
								${this.row("Chip-UID", buildInfo.DEVICE_CHIP_UID)}
								${this.row("Ethernet-MAC", buildInfo.DEVICE_ETH_MAC)}
								${this.row("USB-NCM-MAC", buildInfo.DEVICE_USB_NCM_MAC)}
								${this.row("TLS-Zertifikat-Aussteller", buildInfo.DEVICE_CERT_ISSUER)}
								${this.row("Zertifikat ausgestellt am", formatEpochSeconds(buildInfo.DEVICE_CERT_ISSUED_AT_EPOCH))}
								${this.row("Zertifikat gültig bis", formatEpochSeconds(buildInfo.DEVICE_CERT_VALID_UNTIL_EPOCH))}
							</tbody>
						</table>
					</div>

					<!-- Firmware und Web-UI werden immer gemeinsam aus demselben Git-Stand gebaut
					     (s. docs/build-process.md) -- deshalb ein einziger Build-Info-Block statt einer
					     getrennten Firmware-/Web-UI-Ansicht. -->
					<div class="panel-section">
						<div class="panel-label">Build Info</div>
						<table class="register-table">
							<tbody>
								${this.row(
									"Firmware-Version",
									`${buildInfo.FW_VERSION_MAJOR}.${buildInfo.FW_VERSION_MINOR}.${buildInfo.FW_VERSION_PATCH}`
								)}
								${this.row("Git-Commit", buildInfo.GIT_COMMIT_HASH)}
								${this.row("Commit-Titel", buildInfo.GIT_COMMIT_MESSAGE)}
								${this.row("Branch", buildInfo.GIT_BRANCH)}
								${this.row("Tag", buildInfo.GIT_TAG)}
								${this.row("Arbeitsverzeichnis", buildInfo.GIT_IS_DIRTY ? "geaendert (dirty)" : "sauber")}
								${this.row("Letzter Commit", formatEpochSeconds(buildInfo.GIT_COMMIT_DATE_EPOCH))}
								${this.row("Gebaut am", formatEpochSeconds(buildInfo.BUILD_TIMESTAMP_EPOCH))}
							</tbody>
						</table>
					</div>

					<div class="panel-section">
						<div class="panel-label">
							Live Info - System
							<button class="system-refresh-btn" @click=${() => this.refresh()} ?disabled=${this.loading}>
								${this.loading ? "Lädt..." : "Aktualisieren"}
							</button>
						</div>
						${info
							? html`
									<table class="register-table">
										<tbody>
											${this.row("Uptime", formatUptime(info.uptimeSeconds))}
											${this.row("Freier Heap", `${(info.freeHeapBytes / 1024).toFixed(1)} KiB`)}
											${this.row("Letzter Reset", RESET_CAUSE_LABELS[info.resetCauseCode] ?? "Unbekannt")}
										</tbody>
									</table>
								`
							: html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>

					<div class="panel-section">
						<div class="panel-label">Live Info - Network</div>
						${info
							? html`
									<table class="register-table">
										<tbody>
											${this.row("IP-Adresse", info.ipAddress)}
											${this.row("Netzmaske", info.netMask)}
										</tbody>
									</table>
									${this.renderPhySection(info.phy)}
								`
							: html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>

					<div class="panel-section">
						<div class="panel-label">I2C Diagnostics</div>
						${info
							? html`
									${this.renderI2cBusSection("I2C1", info.i2c.I2C_1)}
									${this.renderI2cBusSection("I2C2", info.i2c.I2C_2)}
									${this.renderI2cBusSection("I2C4", info.i2c.I2C_4)}
								`
							: html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>
				</div>
			</div>
		`;
	}
}
