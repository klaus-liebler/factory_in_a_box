// System: board identity and build (compile-time constants from build-info.ts), and the live system
// information - uptime, network, Ethernet PHY diagnostics, EtherCAT (ESC, ports, SyncManagers), I2C scans -
// refreshed every 10 s while shown.
import { LitElement, html } from "lit";
import { customElement, state } from "lit/decorators.js";
import type { system } from "../../../generated/ws-protocol.ts";
import type { WorkspaceApp } from "@klaus-liebler/ui-shell/shell/workspace-app.ts";
import * as buildInfo from "../../../generated/build-info.ts";
import { I2C_ADDRESS_CANDIDATES } from "../../core/i2c-device-names.ts";
import { i2cBusSections } from "../../core/i2c-scan.ts";
import { setAppStatus } from "@klaus-liebler/ui-shell/shell/status-bar.ts";
import { openWorkspace, setAppRibbon } from "@klaus-liebler/ui-shell/shell/ribbon.ts";
import { Poller, pollingGroup } from "../../core/poller.ts";
import { dateTimeOfEpoch, formatUptime } from "@klaus-liebler/ui-shell/ui/format.ts";
import { fetchSystemInfo, RESET_CAUSE_NAMES, type SystemInfo } from "../../core/system.ts";
import {
	alCodeText, alStateText, dlStatus, eepromText, fetchEthercatStatus, fmmuKind, MODE_NAMES, sdoAbortText, stateName, syncManagerKind,
	systemTimeText, type EthercatStatus,
} from "../../core/ethercat.ts";

function formatOctets(bytes: readonly number[]): string {
	return bytes.join(".");
}

// LAN8742 bit masks (LAN8742A data sheet, registers 1/17/31): the firmware sends the raw registers
// (net::phy_diagnostics() in src/network.cpp), they are interpreted here.
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
export class SystemInfoApp extends LitElement implements WorkspaceApp {
	protected willUpdate(): void {
		setAppStatus(this, this.poller.status("Lade Systemdaten …"));
		setAppRibbon(this, [
			{ id: "system", label: "System", groups: [
				pollingGroup(this.poller, "Uptime, Netzwerk, PHY, EtherCAT und I²C"),
				{ label: "Weitere", commands: [
					{ id: "tasks", label: "Task Manager", icon: "tasks", run: () => openWorkspace("/tasks") },
					{ id: "power", label: "Power", icon: "power", run: () => openWorkspace("/power") },
					{ id: "terminal", label: "Terminal", icon: "terminal", run: () => openWorkspace("/terminal") },
				] },
			] },
		]);
	}

	protected createRenderRoot() {
		return this;
	}

	@state() private info: SystemInfo | null = null;
	@state() private ecat: EthercatStatus | null = null;
	private readonly poller = new Poller(this, async () => {
		this.info = await fetchSystemInfo();
		this.ecat = await fetchEthercatStatus();
	}, 10_000);

	onShow(): void {
		this.poller.start();
	}

	onHide(): void {
		this.poller.stop();
	}

	private row(name: string, value: string | number) {
		return html`<tr><td>${name}</td><td>${value}</td></tr>`;
	}

	private renderPhySection(phy: system.PhyRegisters) {
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
			<table class="register-table kv-table">
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

	private hex(v: number, digits: number): string {
		return "0x" + v.toString(16).toUpperCase().padStart(digits, "0");
	}

	/** The slave: AL state, identity of the ESC, station address, counters. */
	private renderEthercatState(e: EthercatStatus) {
		if (e.mode !== 2) {
			return html`<p class="panel-text ${e.mode === 0 ? "" : "status-error"}">${MODE_NAMES[e.mode] ?? "?"}</p>`;
		}
		const errorState = (e.alStatus & 0x10) !== 0;
		return html`
			<table class="register-table kv-table">
				<tbody>
					${this.row("Anbindung", `${MODE_NAMES[e.mode]}, ${e.irq ? "IRQ" : "gepollt (keine IRQ-Leitung)"}`)}
					${this.row("Konfiguration geladen (READY)", e.ready ? "ja" : "nein - wartet auf den EEPROM-Reload")}
					${e.ready ? html`
						<tr><td>AL-Zustand</td><td><span class="bool-badge ${errorState ? "bool-badge-error" : "bool-badge-on"}">${alStateText(e.alStatus)}</span></td></tr>
						${this.row("AL-Statuscode", alCodeText(e.alStatusCode))}
						${this.row("Stationsadresse / Alias", `${this.hex(e.stationAddress, 4)} / ${this.hex(e.stationAlias, 4)}`)}
						${this.row("ESC", `Typ ${this.hex(e.escType, 2)}, Rev. ${e.escRevision}, Build ${this.hex(e.escBuild, 4)}`)}
						${this.row("Ressourcen", `${e.fmmuCount} FMMUs, ${e.syncManagerCount} SyncManager, ${e.ramKb} KB RAM`)}
						${this.row("Features / Port-Deskriptor", `${this.hex(e.features, 4)} / ${this.hex(e.portDescriptor, 2)}`)}
						${this.row("PDI / ESC-Konfiguration", `${this.hex(e.pdiControl, 2)} / ${this.hex(e.escConfig, 2)}`)}
						${this.row("EEPROM", `${eepromText(e.eepromStatus)} - ${e.eepromReads} gelesen, ${e.eepromWrites} geschrieben`)}
						${this.row("Zustandswechsel", `${e.alTransitions} ausgeführt, ${e.alRefused} abgelehnt`)}
						${this.row("Mailbox", `${e.mailboxRequests} Anfragen, ${e.mailboxResponses} Antworten, ${e.mailboxErrors} Fehler, ${e.mailboxRepeats} Wiederholungen`)}
						${this.row("SDO (CoE)", `${e.sdoUploads} gelesen, ${e.sdoDownloads} geschrieben, ${e.sdoAborts} abgebrochen, ${e.sdoInfoRequests} Info-Anfragen`)}
						${e.sdoAborts > 0 ? this.row("Letzter SDO-Abbruch", `${this.hex(e.lastAbortIndex, 4)}:${e.lastAbortSubindex} - ${sdoAbortText(e.lastAbortCode)}`) : null}
						${this.row("Prozessdaten (PDO-Zuordnung)", `${e.outputsBytes} Byte Ausgänge (SM2), ${e.inputsBytes} Byte Eingänge (SM3)`)}
						${this.row("Ausgänge (RxPDO)", `${e.outputsReceived} Abbilder empfangen, ${e.outputWrites} Werte geschrieben (nur Änderungen, nur OP), ${e.outputRefused} abgelehnt (SPS)`)}
						${this.row("Eingänge (TxPDO)", `${e.inputsWritten} Abbilder geschrieben`)}
						${this.row("Sicherer Zustand", `${e.safeStates}× (OP verlassen), davon Watchdog ${e.watchdogExpired}×`)}
						${e.alRefused > 0 ? this.row("Zuletzt abgelehnt", `${stateName(e.lastRefusedFrom)} → ${stateName(e.lastRefusedTo)}: ${alCodeText(e.lastRefusedCode)}`) : null}
						${this.row("Fehlerzähler Verarbeitung / PDI", `${e.processingUnitErrors} / ${e.pdiErrors}`)}
						${this.row("Watchdog Prozessdaten", `Zeit ${e.watchdogTimeProcessData} (Teiler ${e.watchdogDivider}), abgelaufen ${e.watchdogCounterProcessData}×; PDI ${e.watchdogCounterPdi}×`)}
						${this.row("Systemzeit (DC)", systemTimeText(e.systemTimeNs))}
					` : null}
					${this.row("ID_REV / HW_CFG", `${this.hex(e.idRev, 8)} / ${this.hex(e.hwCfg, 8)}`)}
					${this.row("Stand", `vor ${(e.statusAgeMs / 1000).toFixed(1)} s`)}
				</tbody>
			</table>
		`;
	}

	/** Both ports: DL status, the internal PHY, the error counters. */
	private renderEthercatPorts(e: EthercatStatus) {
		if (e.mode !== 2 || !e.ready) return html`<p class="panel-text">Keine Daten (ESC nicht bereit).</p>`;
		const dl = dlStatus(e.dlStatus);
		const yesNo = (b: boolean) => (b ? "ja" : "nein");
		return html`
			<p class="panel-text">PDI betriebsbereit: ${yesNo(dl.pdiOperational)}</p>
			<table class="register-table">
				<thead><tr><th></th>${e.ports.map((_, i) => html`<th>Port ${i} (${i === 0 ? "IN" : "OUT"})</th>`)}</tr></thead>
				<tbody>
					<tr><td>Link (ESC)</td>${dl.ports.map((p) => html`<td>${p.link ? "up" : "down"}</td>`)}</tr>
					<tr><td>Kommunikation / Loop geschlossen</td>${dl.ports.map((p) => html`<td>${yesNo(p.communication)} / ${yesNo(p.loopClosed)}</td>`)}</tr>
					<tr><td>PHY-Adresse</td>${e.ports.map((p) => html`<td>${p.phyPresent ? p.phyAddress : "–"}</td>`)}</tr>
					<tr><td>PHY Link / Auto-Negotiation</td>${e.ports.map((p) => html`<td>${p.phyPresent ? `${p.phyLink ? "up" : "down"} / ${p.phyAutonegotiationComplete ? "fertig" : "läuft"}` : "–"}</td>`)}</tr>
					<tr><td>Energie am Kabel / Auto-MDIX</td>${e.ports.map((p) => html`<td>${p.phyPresent ? `${yesNo(p.phyEnergy)} / ${p.phyAutoMdix ? "aktiv" : "aus"}` : "–"}</td>`)}</tr>
					<tr><td>PHY BMCR / BMSR</td>${e.ports.map((p) => html`<td>${p.phyPresent ? `${this.hex(p.phyBasicControl, 4)} / ${this.hex(p.phyBasicStatus, 4)}` : "–"}</td>`)}</tr>
					<tr><td>Ungültige Frames / RX-Fehler</td>${e.ports.map((p) => html`<td>${p.invalidFrames} / ${p.rxErrors}</td>`)}</tr>
					<tr><td>Weitergeleitete Fehler</td>${e.ports.map((p) => html`<td>${p.forwardedErrors}</td>`)}</tr>
					<tr><td>Link-Verluste</td>${e.ports.map((p) => html`<td>${p.lostLinks}</td>`)}</tr>
				</tbody>
			</table>
		`;
	}

	/** What the master configured: SyncManagers and FMMUs. */
	private renderEthercatMemory(e: EthercatStatus) {
		if (e.mode !== 2 || !e.ready) return html`<p class="panel-text">Keine Daten (ESC nicht bereit).</p>`;
		return html`
			<table class="register-table">
				<thead><tr><th>SM</th><th>Art</th><th>Start</th><th>Länge</th><th>Control / Status</th></tr></thead>
				<tbody>
					${e.syncManagers.map((sm, i) => html`<tr>
						<td>${i}</td><td>${syncManagerKind(sm.control, sm.length, (sm.activate & 1) !== 0)}</td>
						<td>${this.hex(sm.start, 4)}</td><td>${sm.length}</td><td>${this.hex(sm.control, 2)} / ${this.hex(sm.status, 2)}</td>
					</tr>`)}
				</tbody>
			</table>
			<table class="register-table">
				<thead><tr><th>FMMU</th><th>Art</th><th>logisch</th><th>Länge</th><th>physisch</th></tr></thead>
				<tbody>
					${e.fmmus.map((f, i) => html`<tr>
						<td>${i}</td><td>${f.active ? fmmuKind(f.type) : "inaktiv"}</td>
						<td>${this.hex(f.logicalStart, 8)}</td><td>${f.length}</td><td>${this.hex(f.physicalStart, 4)}</td>
					</tr>`)}
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
						<table class="register-table kv-table">
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

				<div class="region-grid">
					<!-- Board/Hostname/Chip-UID/MAC/Zertifikat sind Compile-Zeit-Konstanten
					     (s. builder/Phases/ReadGitStatus.cs) -- kommen direkt aus build-info.ts, keine
					     Laufzeit-Abfrage noetig, deshalb ohne "Aktualisieren"-Button/Ladezustand. -->
					<div class="panel-section">
						<div class="panel-label">Board Info</div>
						<table class="register-table kv-table">
							<tbody>
								${this.row("Board", buildInfo.BOARD_NAME)}
								${this.row("Hostname", `${buildInfo.DEVICE_HOSTNAME}.local`)}
								${this.row("Chip-UID", buildInfo.DEVICE_CHIP_UID)}
								${this.row("Ethernet-MAC", buildInfo.DEVICE_ETH_MAC)}
								${this.row("USB-NCM-MAC", buildInfo.DEVICE_USB_NCM_MAC)}
								${this.row("TLS-Zertifikat-Aussteller", buildInfo.DEVICE_CERT_ISSUER)}
								${this.row("Zertifikat ausgestellt am", dateTimeOfEpoch(buildInfo.DEVICE_CERT_ISSUED_AT_EPOCH))}
								${this.row("Zertifikat gültig bis", dateTimeOfEpoch(buildInfo.DEVICE_CERT_VALID_UNTIL_EPOCH))}
							</tbody>
						</table>
					</div>

					<!-- Firmware und Web-UI werden immer gemeinsam aus demselben Git-Stand gebaut
					     (s. docs/build-process.md) -- deshalb ein einziger Build-Info-Block statt einer
					     getrennten Firmware-/Web-UI-Ansicht. -->
					<div class="panel-section">
						<div class="panel-label">Build Info</div>
						<table class="register-table kv-table">
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
								${this.row("Letzter Commit", dateTimeOfEpoch(buildInfo.GIT_COMMIT_DATE_EPOCH))}
								${this.row("Gebaut am", dateTimeOfEpoch(buildInfo.BUILD_TIMESTAMP_EPOCH))}
							</tbody>
						</table>
					</div>

					<div class="panel-section">
						<div class="panel-label">Live Info - System</div>
						${info
							? html`
									<table class="register-table kv-table">
										<tbody>
											${this.row("Uptime", formatUptime(info.uptimeSeconds))}
											${this.row("Freier Heap", `${(info.freeHeapBytes / 1024).toFixed(1)} KiB`)}
											${this.row("Letzter Reset", RESET_CAUSE_NAMES[info.resetCauseCode] ?? "Unbekannt")}
										</tbody>
									</table>
								`
							: html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>

					<div class="panel-section">
						<div class="panel-label">Live Info - Network</div>
						${info
							? html`
									<table class="register-table kv-table">
										<tbody>
											${this.row("IP-Adresse", formatOctets(info.ipAddress))}
											${this.row("Netzmaske", formatOctets(info.netMask))}
										</tbody>
									</table>
									${this.renderPhySection(info.phy)}
								`
							: html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>

					<div class="panel-section">
						<div class="panel-label">EtherCAT - Slave</div>
						${this.ecat ? this.renderEthercatState(this.ecat) : html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>

					<div class="panel-section">
						<div class="panel-label">EtherCAT - Ports</div>
						${this.ecat ? this.renderEthercatPorts(this.ecat) : html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>

					<div class="panel-section">
						<div class="panel-label">EtherCAT - SyncManager und FMMUs</div>
						${this.ecat ? this.renderEthercatMemory(this.ecat) : html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>

					<div class="panel-section">
						<div class="panel-label">I2C Diagnostics</div>
						${info
							? html`
									${i2cBusSections(info.i2cScans).map((s) => this.renderI2cBusSection(s.label, s.addresses))}
								`
							: html`<p class="panel-text">Noch keine Daten geladen.</p>`}
					</div>
				</div>
			</div>
		`;
	}
}
