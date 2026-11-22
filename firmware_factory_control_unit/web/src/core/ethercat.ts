// The EtherCAT slave's status (schema/best_binary_buffers/ethercat.cs, ethercat_status() in src/web.cpp):
// the firmware sends the ESC's registers raw, they are interpreted here (ETG.1000.4/.6, LAN9253 DS00003421B 11.16).
import { ethercat } from "../../generated/ws-protocol.ts";
import { wsRequest } from "./ws-client.ts";

export type EthercatStatus = ethercat.StatusResponse.Payload;

export function fetchEthercatStatus(): Promise<EthercatStatus> {
	return wsRequest(ethercat.NAMESPACE_ID, (correlationId) => ethercat.StatusRequest.encode({}, correlationId), ethercat.StatusResponse);
}

export const MODE_NAMES: Record<number, string> = {
	0: "kein ESC gefunden (oder keiner auf dieser Revision)",
	1: "LAN9252-kompatibel (PDI 0x80) - nicht unterstützt, Straps prüfen",
	2: "SPI, EtherCAT Direct Mapped (PDI 0x82)",
};

const STATES: Record<number, string> = { 1: "INIT", 2: "PRE-OP", 3: "BOOT", 4: "SAFE-OP", 8: "OP" };

/** "OP", "SAFE-OP + Fehler", ... of an AL status (bits 3:0 the state, bit 4 an indicated error). */
export function alStateText(alStatus: number): string {
	const name = STATES[alStatus & 0x0f] ?? `? (0x${(alStatus & 0x0f).toString(16)})`;
	return (alStatus & 0x10) !== 0 ? `${name} + Fehler` : name;
}

export function stateName(state: number): string {
	return STATES[state & 0x0f] ?? "?";
}

const AL_CODES: Record<number, string> = {
	0x0000: "kein Fehler",
	0x0001: "unspezifizierter Fehler",
	0x0011: "ungültiger Zustandswechsel angefordert",
	0x0012: "unbekannter Zustand angefordert",
	0x0013: "Bootstrap nicht unterstützt",
	0x0016: "ungültige Mailbox-Konfiguration",
	0x0017: "ungültige SyncManager-Konfiguration",
	0x001b: "SyncManager-Watchdog abgelaufen",
	0x001d: "ungültige Ausgangskonfiguration",
	0x001e: "ungültige Eingangskonfiguration",
};

/** The AL status code as text (ETG.1000.6 table 11), with the hex code. */
export function alCodeText(code: number): string {
	const hex = `0x${code.toString(16).padStart(4, "0")}`;
	return `${hex} ${AL_CODES[code] ?? "unbekannt"}`;
}

const SDO_ABORTS: Record<number, string> = {
	0x05030000: "Toggle-Bit nicht gewechselt",
	0x05040001: "Kommando ungültig/nicht unterstützt",
	0x06010000: "Zugriff nicht unterstützt",
	0x06010001: "nur schreibbar",
	0x06010002: "nur lesbar",
	0x06020000: "Objekt existiert nicht",
	0x06070010: "Datentyp-Länge passt nicht",
	0x06070012: "zu lang",
	0x06070013: "zu kurz",
	0x06090011: "Subindex existiert nicht",
	0x06090030: "Wertebereich überschritten",
	0x08000000: "allgemeiner Fehler",
	0x08000020: "nicht gespeichert",
	0x08000021: "lokale Steuerung (SPS ist Steuerquelle)",
	0x08000022: "im aktuellen Zustand nicht erlaubt",
};

/** An SDO abort code as text (ETG.1000.6 table 41), with the hex code. */
export function sdoAbortText(code: number): string {
	return `0x${code.toString(16).padStart(8, "0")} ${SDO_ABORTS[code] ?? "unbekannt"}`;
}

export interface DlPort {
	link: boolean;
	loopClosed: boolean;
	communication: boolean;
}

/** DL status (0110h): bit 0 PDI operational, bits 4+n physical link of port n, bits 8+2n loop closed and
 *  9+2n communication of port n. */
export function dlStatus(dl: number): { pdiOperational: boolean; ports: DlPort[] } {
	const ports = [0, 1].map((n) => ({
		link: (dl & (1 << (4 + n))) !== 0,
		loopClosed: (dl & (1 << (8 + 2 * n))) !== 0,
		communication: (dl & (1 << (9 + 2 * n))) !== 0,
	}));
	return { pdiOperational: (dl & 1) !== 0, ports };
}

/** A SyncManager's role from its control byte: mailbox or buffered (bits 1:0), master writes or reads (3:2). */
export function syncManagerKind(control: number, length: number, enabled: boolean): string {
	if (!enabled || length === 0) return "inaktiv";
	const mode = (control & 0x03) === 0x02 ? "Mailbox" : (control & 0x03) === 0x00 ? "3-Puffer" : "?";
	const dir = (control & 0x0c) === 0x04 ? "Master → Slave" : (control & 0x0c) === 0x00 ? "Slave → Master" : "?";
	return `${mode}, ${dir}`;
}

/** EEPROM control/status (0502h). */
export function eepromText(status: number): string {
	const parts = [(status & 0x20) !== 0 ? "emuliert" : "I²C-EEPROM", (status & 0x1000) !== 0 ? "nicht geladen" : "geladen"];
	if ((status & 0x0800) !== 0) parts.push("Prüfsummenfehler");
	if ((status & 0x8000) !== 0) parts.push("beschäftigt");
	return parts.join(", ");
}

/** FMMU type: bit 0 the master reads, bit 1 it writes. */
export function fmmuKind(type: number): string {
	return ["–", "lesen (Eingänge)", "schreiben (Ausgänge)", "lesen + schreiben"][type & 3]!;
}

/** The ESC's system time (DC, ns since 2000-01-01; the wire's 64-bit value as a number, ms precision is
 *  enough): as a date when it runs, else "steht". */
export function systemTimeText(ns: number): string {
	if (ns === 0) return "steht (keine Distributed Clocks)";
	const epoch2000 = Date.UTC(2000, 0, 1);
	return new Date(epoch2000 + Math.floor(ns / 1_000_000)).toISOString().replace("T", " ").replace(".000Z", " UTC").replace("Z", " UTC");
}
