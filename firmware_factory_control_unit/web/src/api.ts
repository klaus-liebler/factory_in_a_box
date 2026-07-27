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

// LAN8742-PHY-Register, unveraendert/roh wie vom Chip gelesen (s. Core/Src/webserver.cpp
// PhyRegisters/read_phy_registers() -- auf Anwenderwunsch keinerlei Bit-Interpretation mehr in
// der Firmware, das Parsen/Interpretieren passiert stattdessen client-seitig, s.
// apps/system-info-app.ts). readOk === false, wenn schon die passiven Registerzugriffe
// fehlgeschlagen sind (z.B. falsche PHY-Adresse) -- alle anderen Felder sind dann bedeutungslos.
// tdrAvailable === false analog fuer die TDR-Messung (bsr/physcsr/mcsr/secr/scsir bleiben in dem
// Fall trotzdem gueltig).
export interface PhyRegisters {
	readOk: boolean;
	tdrAvailable: boolean;
	bsr: number;
	physcsr: number;
	mcsr: number;
	secr: number;
	scsir: number;
	tcsr: number;
	clr: number;
}

// Ergebnis des einmaligen I2C-Geraete-Scans beim Boot (s. Core/Src/webserver.hpp
// PerformBootI2cScans(), aufgerufen aus Io::Setup()) -- je Bus 128 Eintraege (Index = 7-Bit-
// Adresse), true = Geraet antwortet. I2C3 ist auf dieser Platine aktuell nicht per CubeMX
// aktiviert (keine freien SCL/SDA-Pins mit passender Alternate Function ohne Konflikt).
export interface I2cScanResult {
	I2C_1: boolean[];
	I2C_2: boolean[];
	I2C_4: boolean[];
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
	phy: PhyRegisters;
	i2c: I2cScanResult;
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

// bitfieldOffset zeigt auf ein 16-Byte-Bitfeld (128 Bit, LSB von Byte 0 = Adresse 0), s.
// Core/Src/webserver.cpp PerformBootI2cScans().
function decodeI2cBitfield(view: DataView, bitfieldOffset: number): boolean[] {
	const result: boolean[] = new Array(128);
	for (let addr = 0; addr < 128; addr++) {
		const byte = view.getUint8(bitfieldOffset + Math.floor(addr / 8));
		result[addr] = (byte & (1 << addr % 8)) !== 0;
	}
	return result;
}

// Byte-Layout muss exakt zu Core/Src/webserver.cpp fill_system_info() passen (dort auch als
// Tabelle dokumentiert): PHY-Register roh ab Offset 17, I2C-Bitfelder ab Offset 33/49/65.
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
		phy: {
			readOk: view.getUint8(17) !== 0,
			tdrAvailable: view.getUint8(18) !== 0,
			bsr: view.getUint16(19, true),
			physcsr: view.getUint16(21, true),
			mcsr: view.getUint16(23, true),
			secr: view.getUint16(25, true),
			scsir: view.getUint16(27, true),
			tcsr: view.getUint16(29, true),
			clr: view.getUint16(31, true),
		},
		i2c: {
			I2C_1: decodeI2cBitfield(view, 33),
			I2C_2: decodeI2cBitfield(view, 49),
			I2C_4: decodeI2cBitfield(view, 65),
		},
	};
}
