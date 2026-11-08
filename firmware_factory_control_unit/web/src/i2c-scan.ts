// I2C-Bootscans der Firmware fuer die Systemseite (apps/system-info-app.ts): jeder Bus mit seiner
// Nummer auf dieser Hardware-Revision (Revision 1/2: I2C1/I2C2/I2C4, Revision 3: I2C1/I2C4/I2C3),
// s. system_info() in src/web.cpp und scan() in src/io.cpp.
import type { system } from "../generated/ws-protocol.js";

// bytes ist ein 16-Byte-Bitfeld (128 Bit, LSB von Byte 0 = Adresse 0).
export function decodeI2cBitfield(bytes: readonly number[]): boolean[] {
	const result: boolean[] = new Array(128);
	for (let addr = 0; addr < 128; addr++) {
		result[addr] = (bytes[Math.floor(addr / 8)] & (1 << addr % 8)) !== 0;
	}
	return result;
}

export interface I2cBusSection {
	label: string;
	addresses: boolean[];
}

// Die Abschnitte nach Busnummer sortiert, beschriftet mit der Nummer, die die Firmware meldet.
export function i2cBusSections(scans: readonly system.I2cBusScan[]): I2cBusSection[] {
	return [...scans]
		.sort((a, b) => a.bus - b.bus)
		.map((s) => ({ label: `I2C${s.bus}`, addresses: decodeI2cBitfield(s.devices) }));
}
