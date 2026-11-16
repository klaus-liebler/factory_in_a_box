// The stepper test bench's logic: TMC2209 status decoding and the move profile (the firmware's formulas).
import { describe, expect, it } from "vitest";
import { decodeDrvStatus, decodePwmScale, fullStepsPerSecond, moveCurve, moveRate, profileProblem, rampRate, rampSteps, tstepOf, type MoveProfile } from "./stepper-lab-model.ts";

describe("stepper test bench: TMC2209 status", () => {
	it("DRV_STATUS: current, standstill, StealthChop, errors", () => {
		const s = decodeDrvStatus((1 << 31) | (1 << 30) | (12 << 16) | (1 << 9) | (1 << 0) | (1 << 3) | (1 << 7));
		expect(s).toMatchObject({ csActual: 12, standstill: true, stealth: true, overtempWarning: true, overtemp: false,
			shortToGround: true, shortLowSide: false, openLoad: true, temperature: "> 143 °C" });
		expect(decodeDrvStatus(0).temperature).toBe("< 120 °C");
	});

	it("PWM_SCALE and TSTEP", () => {
		expect(decodePwmScale((0x1f0 << 16) | 0x64)).toEqual({ sum: 100, auto: -16 });
		expect(fullStepsPerSecond(0xfffff)).toBe(0);
		expect(fullStepsPerSecond(469)).toBeCloseTo(100, 0);  // 12 MHz / (469 * 256)
		expect(tstepOf(100)).toBe(469);
	});
});

describe("stepper test bench: the move profile (rtoos ProfileStepper)", () => {
	const lin: MoveProfile = { startHz: 200, cruiseHz: 4000, accel: 20000, shape: "linear" };
	const sig: MoveProfile = { ...lin, shape: "sigmoid" };

	it("the ramp length and the linear law, as the firmware test", () => {
		expect(rampSteps(lin)).toBe(399);
		expect(rampRate(lin, 0)).toBe(200);
		expect(rampRate(lin, 200)).toBeCloseTo(Math.sqrt(200 ** 2 + 2 * 20000 * 200), 6);
		expect(rampRate(lin, 1000)).toBe(4000);
		expect(rampRate(sig, 0)).toBeCloseTo(200, 3);
		expect(rampRate(sig, 20)).toBeLessThan(rampRate(lin, 20));  // gentle start
	});

	it("a move goes up, cruises, comes down the same way; a short one is a triangle", () => {
		for (const p of [lin, sig]) {
			for (let k = 0; k < 2000; k++) expect(moveRate(p, 2000, k)).toBe(moveRate(p, 2000, 1999 - k));
			expect(moveRate(p, 2000, 1000)).toBe(4000);
			expect(Math.max(...Array.from({ length: 101 }, (_, k) => moveRate(p, 101, k)))).toBeLessThan(4000);
		}
		const c = moveCurve(lin, 2000);
		expect(c[0].v).toBe(200);
		expect(c.at(-1)!.v).toBe(0);
		// 2 ramps of 0.19 s and (2000 - 798) steps at 4000/s = 0.3 s
		expect(c.at(-1)!.t).toBeCloseTo(0.19 * 2 + 1202 / 4000, 1);
	});

	it("refuses what the step timer cannot make", () => {
		expect(profileProblem(lin)).toBeNull();
		expect(profileProblem({ ...lin, startHz: 10 })).toMatch(/Startfrequenz zu niedrig/);
		expect(profileProblem({ ...lin, cruiseHz: 200_000 })).toMatch(/Zielfrequenz zu hoch/);
		expect(profileProblem({ ...lin, accel: 0 })).toMatch(/Beschleunigung/);
	});
});
