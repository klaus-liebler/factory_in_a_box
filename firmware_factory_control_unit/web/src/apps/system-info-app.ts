import { LitElement, html } from "lit";
import { customElement, state } from "lit/decorators.js";
import "../styles.css";
import { fetchI2cScan, fetchSystemInfo, type I2cScanResult, type SystemInfo } from "../api.js";
import type { DashboardApp } from "../shell/dashboard-app.js";
import { I2C_ADDRESS_CANDIDATES } from "../i2c-device-names.js";
import * as buildInfo from "../../generated/build-info.js";

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

// Alle Zeitpunkte kommen als Unix-Epoch-Sekunden (int64-tauglich, s. builder/src/git-info.ts) --
// hier und NUR hier, am Anzeigeort, in die lokale Zeitzone des Browsers umgerechnet
// (toLocaleString() nutzt implizit Intl mit der System-/Browser-Zeitzone).
function formatEpochSeconds(epochSeconds: number): string {
	return new Date(epochSeconds * 1000).toLocaleString("de-DE");
}

const I2C_BUSES: readonly { label: string; key: keyof I2cScanResult }[] = [
	{ label: "I2C1", key: "I2C_1" },
	{ label: "I2C2", key: "I2C_2" },
	{ label: "I2C4", key: "I2C_4" },
];

@customElement("system-info-app")
export class SystemInfoApp extends LitElement implements DashboardApp {
	protected createRenderRoot() {
		return this;
	}

	@state() private info: SystemInfo | null = null;
	@state() private statusMessage = "Lade Systemdaten...";
	@state() private statusVariant: "warning" | "success" | "error" = "warning";
	@state() private loading = false;

	@state() private i2cResult: I2cScanResult | null = null;
	@state() private i2cScanning = false;
	@state() private i2cError: string | null = null;

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

	// I2C-Scan ist eine bewusste, seltene Aktion (mehrere 100ms Laufzeit auf der Firmware, s.
	// Core/Src/webserver.cpp perform_i2c_scan()) -- deshalb ein eigener Button statt automatisch
	// bei jedem "Aktualisieren".
	private async scanI2c() {
		this.i2cScanning = true;
		this.i2cError = null;
		try {
			this.i2cResult = await fetchI2cScan();
		} catch (error) {
			console.error("I2C-Scan fehlgeschlagen", error);
			this.i2cError = "I2C-Scan fehlgeschlagen (Verbindungsproblem zur Control-Unit).";
		} finally {
			this.i2cScanning = false;
		}
	}

	private row(name: string, value: string | number) {
		return html`<tr><td>${name}</td><td>${value}</td></tr>`;
	}

	private static readonly CABLE_TYPE_LABELS: Record<SystemInfo["phy"]["cableType"], string> = {
		default: "unklar (noch nicht gemessen)",
		shorted: "Kurzschluss",
		open: "Leitungsunterbrechung",
		match: "angepasst (kein Fehler erkannt)",
	};

	// LAN8742-PHY-Diagnose (s. Core/Src/webserver.cpp read_phy_diagnostics()): passive
	// Register-Reads plus eine bei jedem Abruf aktiv ausgelöste TDR-Kabeldiagnose.
	private renderPhySection(phy: SystemInfo["phy"]) {
		if (!phy.readOk) {
			return html`<p class="panel-text status-error">PHY nicht erreichbar (MDIO-Lesefehler).</p>`;
		}
		return html`
			<table class="register-table">
				<tbody>
					${this.row("Link", phy.linkUp ? "up" : "down")}
					${this.row("Auto-Negotiation", phy.autonegDone ? "abgeschlossen" : "läuft/nicht abgeschlossen")}
					${this.row("Geschwindigkeit/Duplex", phy.speedDuplex)}
					${this.row("Energie am Kabel (ENERGYON)", phy.energyDetected ? "ja" : "nein")}
					${this.row("Auto-MDIX", phy.autoMdixEnabled ? "aktiv" : "inaktiv")}
					${this.row("Polarität", phy.polarityReversed ? "invertiert (korrigiert)" : "normal")}
					${this.row("Symbolfehler seit letztem Abruf", phy.symbolErrorCount)}
					${phy.tdrAvailable
						? html`
								${this.row("Kabeldiagnose (TDR)", SystemInfoApp.CABLE_TYPE_LABELS[phy.cableType])}
								${this.row("TDR-Fehlerstelle (roh, unkalibriert)", phy.cableFaultDistanceRaw)}
								${this.row("Kabellängen-Klasse (0-15, nur bei Link)", phy.cableLengthClass)}
							`
						: this.row("Kabeldiagnose (TDR)", "nicht verfügbar")}
				</tbody>
			</table>
		`;
	}

	// Nur gefundene Adressen werden aufgelistet (statt 3x128 Zeilen) -- I2C_ADDRESS_CANDIDATES
	// liefert dazu die ueblichen Verdaechtigen fuer diese Adresse (mehrere ICs teilen sich oft
	// dieselbe Standardadresse, ein Treffer ist ein Hinweis, keine sichere Identifikation).
	private renderI2cSection() {
		if (this.i2cError) {
			return html`<p class="panel-text status-error">${this.i2cError}</p>`;
		}
		if (!this.i2cResult) {
			return html`<p class="panel-text">Noch kein Scan durchgeführt.</p>`;
		}

		const result = this.i2cResult;
		const rows = I2C_BUSES.flatMap(({ label, key }) =>
			result[key]
				.map((found, address) => ({ bus: label, address, found }))
				.filter((entry) => entry.found)
		);

		if (rows.length === 0) {
			return html`<p class="panel-text">Keine Geräte gefunden.</p>`;
		}

		return html`
			<table class="register-table">
				<thead>
					<tr>
						<th>Bus</th>
						<th>Adresse</th>
						<th>Mögliche ICs</th>
					</tr>
				</thead>
				<tbody>
					${rows.map(
						(entry) => html`
							<tr>
								<td>${entry.bus}</td>
								<td>0x${entry.address.toString(16).toUpperCase().padStart(2, "0")}</td>
								<td>${I2C_ADDRESS_CANDIDATES[entry.address]}</td>
							</tr>
						`
					)}
				</tbody>
			</table>
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
					<div class="panel-section">
						<div class="panel-label">Build (Web-UI)</div>
						<table class="register-table">
							<tbody>
								${this.row("Git-Commit", buildInfo.GIT_COMMIT_HASH)}
								${this.row("Branch", buildInfo.GIT_BRANCH)}
								${this.row("Tag", buildInfo.GIT_TAG)}
								${this.row("Arbeitsverzeichnis", buildInfo.GIT_IS_DIRTY ? "geaendert (dirty)" : "sauber")}
								${this.row("Letzter Commit", formatEpochSeconds(buildInfo.GIT_COMMIT_DATE_EPOCH))}
								${this.row("Web-UI gebaut am", formatEpochSeconds(buildInfo.BUILD_TIMESTAMP_EPOCH))}
							</tbody>
						</table>
					</div>

					<!-- Board/Hostname/Firmware-Version/Chip-UID/MAC/Zertifikat sind Compile-Zeit-Konstanten
					     (s. builder/src/phases/read-git-status.ts) -- kommen direkt aus build-info.ts, keine
					     Laufzeit-Abfrage noetig, deshalb ohne "Aktualisieren"-Button/Ladezustand. -->
					<div class="panel-section">
						<div class="panel-label">Board-Identität</div>
						<table class="register-table">
							<tbody>
								${this.row("Board", buildInfo.BOARD_NAME)}
								${this.row("Hostname", `${buildInfo.DEVICE_HOSTNAME}.local`)}
								${this.row(
									"Firmware-Version",
									`${buildInfo.FW_VERSION_MAJOR}.${buildInfo.FW_VERSION_MINOR}.${buildInfo.FW_VERSION_PATCH}`
								)}
								${this.row("Chip-UID", buildInfo.DEVICE_CHIP_UID)}
								${this.row("Ethernet-MAC", buildInfo.DEVICE_ETH_MAC)}
								${this.row("USB-NCM-MAC", buildInfo.DEVICE_USB_NCM_MAC)}
								${this.row("TLS-Zertifikat-Aussteller", buildInfo.DEVICE_CERT_ISSUER)}
								${this.row("Zertifikat ausgestellt am", formatEpochSeconds(buildInfo.DEVICE_CERT_ISSUED_AT_EPOCH))}
								${this.row("Zertifikat gültig bis", formatEpochSeconds(buildInfo.DEVICE_CERT_VALID_UNTIL_EPOCH))}
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
											${this.row("Uptime", formatUptime(info.uptimeSeconds))}
											${this.row("Freier Heap", `${(info.freeHeapBytes / 1024).toFixed(1)} KiB`)}
											${this.row("IP-Adresse", info.ipAddress)}
											${this.row("Netzmaske", info.netMask)}
											${this.row("Letzter Reset", RESET_CAUSE_LABELS[info.resetCauseCode] ?? "Unbekannt")}
										</tbody>
									</table>
								`
							: html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>

					<div class="panel-section">
						<div class="panel-label">Ethernet-PHY (LAN8742)</div>
						${info ? this.renderPhySection(info.phy) : html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>

					<div class="panel-section">
						<div class="panel-label">
							I2C-Geräte-Discovery
							<button class="system-refresh-btn" @click=${() => this.scanI2c()} ?disabled=${this.i2cScanning}>
								${this.i2cScanning ? "Scanne..." : "Scan starten"}
							</button>
						</div>
						${this.renderI2cSection()}
					</div>
				</div>
			</div>
		`;
	}
}
