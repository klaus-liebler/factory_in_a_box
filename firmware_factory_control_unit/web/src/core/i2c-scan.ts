// The firmware's I2C boot scans for the system workspace: every bus with its number on this hardware revision
// (revision 1: I2C1/I2C2/I2C4, revision 3: I2C1/I2C4/I2C3), see system_info() in src/web.cpp and scan() in
// src/io.cpp.
import type { system } from "../../generated/ws-protocol.ts";

// bytes is a 16-byte bit field (128 bits, LSB of byte 0 = address 0).
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

// The sections sorted by bus number, labelled with the number the firmware reports.
export function i2cBusSections(scans: readonly system.I2cBusScan[]): I2cBusSection[] {
	return [...scans]
		.sort((a, b) => a.bus - b.bus)
		.map((s) => ({ label: `I2C${s.bus}`, addresses: decodeI2cBitfield(s.devices) }));
}
