import { describe, expect, it } from "vitest";
import { decodeI2cBitfield, i2cBusSections } from "./i2c-scan.js";

const bits = (...addresses: number[]) => {
	const b = new Array<number>(16).fill(0);
	for (const a of addresses) b[Math.floor(a / 8)] |= 1 << a % 8;
	return b;
};

describe("decodeI2cBitfield", () => {
	it("maps bit n to address n", () => {
		const d = decodeI2cBitfield(bits(0x29, 0x40));
		expect(d.length).toBe(128);
		expect(d.flatMap((p, a) => (p ? [a] : []))).toEqual([0x29, 0x40]);
	});
});

// Regression: the system page labelled the scans "I2C1/I2C2/I2C4" whatever the board had - on
// hardware revision 3 the ToF 2 bus is I2C4 and the supply bus (INA226) I2C3. The label now comes
// from the bus number the firmware sends.
describe("i2cBusSections", () => {
	it("labels each scan with the bus number the firmware reports (revision 3)", () => {
		const sections = i2cBusSections([
			{ bus: 1, devices: bits(0x29, 0x30) },
			{ bus: 4, devices: bits(0x29) },
			{ bus: 3, devices: bits(0x29, 0x40) },
		]);
		expect(sections.map((s) => s.label)).toEqual(["I2C1", "I2C3", "I2C4"]);
		expect(sections[1].addresses[0x40]).toBe(true);
		expect(sections[2].addresses[0x40]).toBe(false);
	});

	it("labels revision 2's buses I2C1, I2C2, I2C4", () => {
		const sections = i2cBusSections([
			{ bus: 1, devices: bits() },
			{ bus: 2, devices: bits() },
			{ bus: 4, devices: bits(0x40) },
		]);
		expect(sections.map((s) => s.label)).toEqual(["I2C1", "I2C2", "I2C4"]);
	});
});
