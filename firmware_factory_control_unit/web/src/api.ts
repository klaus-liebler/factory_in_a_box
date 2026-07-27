// Schlanker Zugriff auf die Firmware-Endpunkte -- bewusst kein JSON: /api/registers liefert
// alle Register voll binaer (2 Byte Little-Endian je Register, erst alle Holding- dann alle
// Input-Register, s. Core/Src/webserver.cpp register_binary_total_length()/
// write_register_binary_chunk()), /api/system liefert ein festes kleines Binaer-Struct
// (s. webserver.cpp fill_system_info() fuer das exakte Byte-Layout), /api/write-holding ist ein
// simples GET mit Query-Parametern statt POST+JSON-Body. Das erspart der Firmware jeglichen
// JSON-Parser (siehe Core/Src/webserver.cpp), passend zum "Prinzip"-Charakter dieser Demo-UI.

import { HOLDING_REGISTER_COUNT, INPUT_REGISTER_COUNT } from "../generated/register-map.js";

export interface RegisterValues {
	holding: number[];
	input: number[];
}

// LAN8742-PHY-Diagnose (s. Core/Src/webserver.cpp read_phy_diagnostics()): passive MDIO-Reads plus
// eine aktiv ausgeloeste TDR-Kabeldiagnose (Kurzschluss/Unterbrechung/Angepasst). readOk === false,
// wenn schon die passiven Registerzugriffe fehlgeschlagen sind (z.B. falsche PHY-Adresse) -- alle
// anderen Felder sind dann bedeutungslos. tdrAvailable === false analog fuer die TDR-Messung.
export interface PhyDiagnostics {
	readOk: boolean;
	linkUp: boolean;
	autonegDone: boolean;
	energyDetected: boolean;
	autoMdixEnabled: boolean;
	polarityReversed: boolean;
	speedDuplex: "10-HD" | "10-FD" | "100-HD" | "100-FD" | "unknown";
	// Symbol-Error-Counter -- loescht sich beim Lesen selbst (Datenblatt), ist deshalb der
	// Zaehlerstand SEIT DEM LETZTEN GET /api/system, nicht seit Systemstart.
	symbolErrorCount: number;
	tdrAvailable: boolean;
	// TCSR.TDR_CH_STATUS -- laut LAN8742-Registerbeschreibung "Messung gueltig".
	tdrStatus: boolean;
	// TCSR.TDR_CH_CABLE_TYPE. "default" bedeutet unklar/nicht (neu) gemessen, "match" bedeutet
	// angepasst/kein Fehler erkannt.
	cableType: "default" | "shorted" | "open" | "match";
	// TCSR.TDR_CH_LENGTH, roh (0-255) -- kein Meterwert, das Datenblatt dokumentiert keine direkte
	// Umrechnungskonstante, nur als relativer Indikator zur Fehlerstelle zu verstehen.
	cableFaultDistanceRaw: number;
	// CLR.CABLE_LENGTH, grobe 4-Bit-Klasse (0-15) -- nur bei bestehendem Link aussagekraeftig
	// (anders als die TDR-Werte oben).
	cableLengthClass: number;
}

// Nur echte Laufzeitwerte -- Firmware-Version, Board-Name, Hostname, Chip-UID und MAC-Adressen
// sind Compile-Zeit-Konstanten und kommen stattdessen aus ../generated/build-info.ts (s. dortiger
// Kommentar), nicht mehr per HTTP.
export interface SystemInfo {
	uptimeSeconds: number;
	freeHeapBytes: number;
	ipAddress: string;
	netMask: string;
	resetCauseCode: number;
	phy: PhyDiagnostics;
}

// Der Server-Paket-Pool der Firmware ist bewusst winzig (siehe Core/Src/app.cpp), eine
// Mehr-Paket-Antwort kann dadurch spuerbar dauern. Ein harter Timeout stellt sicher, dass ein
// haengender Request den Poll-Zyklus nicht fuer immer blockiert (die Apps planen den naechsten
// Poll ohnehin erst NACH Abschluss dieses Aufrufs).
const FETCH_TIMEOUT_MS = 5000;

async function fetchWithTimeout(url: string, timeoutMs: number = FETCH_TIMEOUT_MS): Promise<Response> {
	const controller = new AbortController();
	const timeoutHandle = setTimeout(() => controller.abort(), timeoutMs);
	try {
		return await fetch(url, { cache: "no-store", signal: controller.signal });
	} finally {
		clearTimeout(timeoutHandle);
	}
}

export async function fetchRegisters(): Promise<RegisterValues> {
	const response = await fetchWithTimeout("/api/registers");
	if (!response.ok) {
		throw new Error(`GET /api/registers fehlgeschlagen: HTTP ${response.status}`);
	}
	const buffer = await response.arrayBuffer();
	const view = new DataView(buffer);

	const holding: number[] = new Array(HOLDING_REGISTER_COUNT);
	let offset = 0;
	for (let i = 0; i < HOLDING_REGISTER_COUNT; i++, offset += 2) {
		holding[i] = view.getUint16(offset, true);
	}
	const input: number[] = new Array(INPUT_REGISTER_COUNT);
	for (let i = 0; i < INPUT_REGISTER_COUNT; i++, offset += 2) {
		input[i] = view.getUint16(offset, true);
	}
	return { holding, input };
}

export async function writeHolding(address: number, value: number): Promise<void> {
	const response = await fetchWithTimeout(`/api/write-holding?address=${address}&value=${value}`);
	if (!response.ok) {
		throw new Error(`Schreiben fehlgeschlagen: HTTP ${response.status}`);
	}
}

function formatOctets(view: DataView, offset: number, count: number, sep: string, radix: number, pad: number): string {
	const parts: string[] = [];
	for (let i = 0; i < count; i++) {
		const v = view.getUint8(offset + i);
		parts.push(radix === 16 ? v.toString(16).padStart(pad, "0") : String(v));
	}
	return parts.join(sep);
}

const SPEED_DUPLEX_BY_CODE: Record<number, PhyDiagnostics["speedDuplex"]> = {
	0: "10-HD",
	1: "10-FD",
	2: "100-HD",
	3: "100-FD",
};

const CABLE_TYPE_BY_CODE: Record<number, PhyDiagnostics["cableType"]> = {
	0: "default",
	1: "shorted",
	2: "open",
	3: "match",
};

// phyFlagsOffset ist der Offset des ersten PHY-Feldes (phy_flags) -- alle weiteren PHY-Felder
// folgen ab dort in fester Reihenfolge, s. Layout-Tabelle in Core/Src/webserver.cpp.
function decodePhyDiagnostics(view: DataView, phyFlagsOffset: number): PhyDiagnostics {
	const flags = view.getUint8(phyFlagsOffset);
	return {
		linkUp: (flags & 0x01) !== 0,
		autonegDone: (flags & 0x02) !== 0,
		energyDetected: (flags & 0x04) !== 0,
		autoMdixEnabled: (flags & 0x08) !== 0,
		polarityReversed: (flags & 0x10) !== 0,
		readOk: (flags & 0x20) !== 0,
		tdrAvailable: (flags & 0x40) !== 0,
		tdrStatus: (flags & 0x80) !== 0,
		speedDuplex: SPEED_DUPLEX_BY_CODE[view.getUint8(phyFlagsOffset + 1)] ?? "unknown",
		symbolErrorCount: view.getUint16(phyFlagsOffset + 2, true),
		cableType: CABLE_TYPE_BY_CODE[view.getUint8(phyFlagsOffset + 4)] ?? "default",
		cableFaultDistanceRaw: view.getUint8(phyFlagsOffset + 5),
		cableLengthClass: view.getUint8(phyFlagsOffset + 6),
	};
}

// Byte-Layout muss exakt zu Core/Src/webserver.cpp fill_system_info() passen (dort auch als
// Tabelle dokumentiert).
export async function fetchSystemInfo(): Promise<SystemInfo> {
	const response = await fetchWithTimeout("/api/system");
	if (!response.ok) {
		throw new Error(`GET /api/system fehlgeschlagen: HTTP ${response.status}`);
	}
	const buffer = await response.arrayBuffer();
	const view = new DataView(buffer);

	return {
		uptimeSeconds: view.getUint32(0, true),
		freeHeapBytes: view.getUint32(4, true),
		ipAddress: formatOctets(view, 8, 4, ".", 10, 0),
		netMask: formatOctets(view, 12, 4, ".", 10, 0),
		resetCauseCode: view.getUint8(16),
		phy: decodePhyDiagnostics(view, 17),
	};
}

// Ergebnis von GET /api/i2c (s. Core/Src/webserver.cpp perform_i2c_scan()) -- je Bus ein
// 128-Eintrag-Array, Index = 7-Bit-I2C-Adresse, true = Geraet hat auf HAL_I2C_IsDeviceReady()
// geantwortet. Busnamen wie in register-map.json (reg.i2c.bus).
export interface I2cScanResult {
	I2C_1: boolean[];
	I2C_2: boolean[];
	I2C_4: boolean[];
}

// bitfieldOffset zeigt auf ein 16-Byte-Bitfeld (128 Bit, LSB von Byte 0 = Adresse 0), s.
// Core/Src/webserver.cpp scan_i2c_bus().
function decodeI2cBitfield(view: DataView, bitfieldOffset: number): boolean[] {
	const result: boolean[] = new Array(128);
	for (let addr = 0; addr < 128; addr++) {
		const byte = view.getUint8(bitfieldOffset + Math.floor(addr / 8));
		result[addr] = (byte & (1 << addr % 8)) !== 0;
	}
	return result;
}

// Ein voller 3-Bus-Scan (128 Adressen je Bus, s. Core/Src/webserver.cpp scan_i2c_bus()) dauert
// spuerbar laenger als die anderen Endpunkte -- eigener, grosszuegigerer Timeout statt des
// FETCH_TIMEOUT_MS-Standardwerts, der fuer die gepollten Endpunkte ausgelegt ist.
const I2C_SCAN_TIMEOUT_MS = 15000;

export async function fetchI2cScan(): Promise<I2cScanResult> {
	const response = await fetchWithTimeout("/api/i2c", I2C_SCAN_TIMEOUT_MS);
	if (!response.ok) {
		throw new Error(`GET /api/i2c fehlgeschlagen: HTTP ${response.status}`);
	}
	const buffer = await response.arrayBuffer();
	const view = new DataView(buffer);

	return {
		I2C_1: decodeI2cBitfield(view, 0),
		I2C_2: decodeI2cBitfield(view, 16),
		I2C_4: decodeI2cBitfield(view, 32),
	};
}
