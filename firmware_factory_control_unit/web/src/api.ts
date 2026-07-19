// Schlanker Zugriff auf die Firmware-Endpunkte -- bewusst kein JSON: /api/registers liefert
// alle Register voll binaer (2 Byte Little-Endian je Register, erst alle Holding- dann alle
// Input-Register, s. Core/Src/webserver.cpp register_binary_total_length()/
// write_register_binary_chunk()), /api/system liefert ein festes kleines Binaer-Struct
// (s. webserver.cpp fill_system_info() fuer das exakte Byte-Layout), /api/write-holding ist ein
// simples GET mit Query-Parametern statt POST+JSON-Body. Das erspart der Firmware jeglichen
// JSON-Parser (siehe Core/Src/webserver.cpp), passend zum "Prinzip"-Charakter dieser Demo-UI.

import { HOLDING_REGISTER_COUNT, INPUT_REGISTER_COUNT } from "./register-map.js";

export interface RegisterValues {
	holding: number[];
	input: number[];
}

export interface SystemInfo {
	fwVersionMajor: number;
	fwVersionMinor: number;
	fwVersionPatch: number;
	uptimeSeconds: number;
	freeHeapBytes: number;
	ipAddress: string;
	netMask: string;
	macAddress: string;
	chipUid: string;
	resetCauseCode: number;
	hostname: string;
	boardName: string;
}

// Der Server-Paket-Pool der Firmware ist bewusst winzig (siehe Core/Src/app.cpp), eine
// Mehr-Paket-Antwort kann dadurch spuerbar dauern. Ein harter Timeout stellt sicher, dass ein
// haengender Request den Poll-Zyklus nicht fuer immer blockiert (die Apps planen den naechsten
// Poll ohnehin erst NACH Abschluss dieses Aufrufs).
const FETCH_TIMEOUT_MS = 5000;

async function fetchWithTimeout(url: string): Promise<Response> {
	const controller = new AbortController();
	const timeoutHandle = setTimeout(() => controller.abort(), FETCH_TIMEOUT_MS);
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

function decodePaddedString(view: DataView, offset: number, length: number): string {
	const bytes = new Uint8Array(view.buffer, view.byteOffset + offset, length);
	const nullIndex = bytes.indexOf(0);
	const trimmed = nullIndex >= 0 ? bytes.subarray(0, nullIndex) : bytes;
	return new TextDecoder("ascii").decode(trimmed);
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
		fwVersionMajor: view.getUint16(0, true),
		fwVersionMinor: view.getUint16(2, true),
		fwVersionPatch: view.getUint16(4, true),
		uptimeSeconds: view.getUint32(6, true),
		freeHeapBytes: view.getUint32(10, true),
		ipAddress: formatOctets(view, 14, 4, ".", 10, 0),
		netMask: formatOctets(view, 18, 4, ".", 10, 0),
		macAddress: formatOctets(view, 22, 6, ":", 16, 2),
		chipUid: formatOctets(view, 28, 12, "", 16, 2).toUpperCase(),
		resetCauseCode: view.getUint8(40),
		hostname: decodePaddedString(view, 41, 32),
		boardName: decodePaddedString(view, 73, 48),
	};
}
