// The shared core of the web UI: the poller's schedule and states, register access by name, formatting.
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { Poller } from "./poller.ts";
import { healthProblems, regBool, register, regValue, type RegisterValues } from "./registers.ts";
import { cString, formatBytes, formatUptime } from "./format.ts";

const host = () => ({ addController: vi.fn(), removeController: vi.fn(), requestUpdate: vi.fn(), updateComplete: Promise.resolve(true) });

describe("Poller", () => {
	beforeEach(() => vi.useFakeTimers());
	afterEach(() => vi.useRealTimers());

	it("runs at once and then every interval, only between start and stop", async () => {
		let n = 0;
		const p = new Poller(host(), () => void n++, 1000);
		await vi.advanceTimersByTimeAsync(3000);
		expect(n).toBe(0);
		p.start();
		await vi.advanceTimersByTimeAsync(2500);
		expect(n).toBe(3);
		p.stop();
		await vi.advanceTimersByTimeAsync(5000);
		expect(n).toBe(3);
	});

	it("never overlaps: a slow task delays the next run", async () => {
		let running = 0, max = 0, n = 0;
		const p = new Poller(host(), async () => {
			running++;
			max = Math.max(max, running);
			n++;
			await new Promise((r) => setTimeout(r, 2500));
			running--;
		}, 1000);
		p.start();
		await vi.advanceTimersByTimeAsync(6900);
		expect(max).toBe(1);
		expect(n).toBe(2);  // 0..2.5 s, 3.5..6 s
	});

	it("a run that ends after stop() schedules nothing", async () => {
		let n = 0;
		const p = new Poller(host(), async () => {
			n++;
			await new Promise((r) => setTimeout(r, 500));
		}, 1000);
		p.start();
		await vi.advanceTimersByTimeAsync(100);
		p.stop();
		await vi.advanceTimersByTimeAsync(5000);
		expect(n).toBe(1);
	});

	it("pauses and resumes; refresh runs once while paused", async () => {
		let n = 0;
		const h = host();
		const p = new Poller(h, () => void n++, 1000);
		p.start();
		await vi.advanceTimersByTimeAsync(10);
		p.setPaused(true);
		expect(h.requestUpdate).toHaveBeenCalled();
		await vi.advanceTimersByTimeAsync(5000);
		expect(n).toBe(1);
		p.refresh();
		await vi.advanceTimersByTimeAsync(5000);
		expect(n).toBe(2);
		p.setPaused(false);
		await vi.advanceTimersByTimeAsync(1500);
		expect(n).toBe(4);
		// starting while paused does not run
		p.stop();
		p.setPaused(true);
		p.start();
		await vi.advanceTimersByTimeAsync(3000);
		expect(n).toBe(4);
	});

	it("status for the status bar: loading, ok, error, paused", async () => {
		let fail = false;
		const p = new Poller(host(), () => { if (fail) throw new Error("x"); }, 1000);
		expect(p.status().kind).toBe("busy");
		p.start();
		await vi.advanceTimersByTimeAsync(10);
		expect(p.status()).toMatchObject({ kind: "ok" });
		expect(p.status().text).toMatch(/^Verbunden – Stand /);
		fail = true;
		await vi.advanceTimersByTimeAsync(1000);
		expect(p.status()).toEqual({ text: "Keine Verbindung zur Control Unit", kind: "error" });
		p.setPaused(true);
		expect(p.status()).toEqual({ text: "Angehalten", kind: "idle" });
	});
});

describe("registers by name", () => {
	const input: number[] = [];
	const holding: number[] = [];
	const at = (name: string) => register(name).address;
	input[at("CPU_TEMPERATURE_C")] = 0xffd8;          // signed 16 bit: -40
	input[at("STEPPER1_POSITION")] = 0xffff;          // signed 32 bit, high word first: -2
	input[at("STEPPER1_POSITION") + 1] = 0xfffe;
	input[at("ETH_RX_COUNT")] = 0x0001;               // unsigned 32 bit
	input[at("ETH_RX_COUNT") + 1] = 0x0002;
	holding[at("VALVE1")] = 1;
	const v: RegisterValues = { input, holding };

	it("decodes signed, 32-bit pairs and the bank", () => {
		expect(regValue(v, "CPU_TEMPERATURE_C")).toBe(-40);
		expect(regValue(v, "STEPPER1_POSITION")).toBe(-2);
		expect(regValue(v, "ETH_RX_COUNT")).toBe(65538);
		expect(regBool(v, "VALVE1")).toBe(true);
		expect(regValue(v, "VALVE2")).toBeUndefined();
		expect(regValue(null, "VALVE1")).toBeUndefined();
	});
	it("an unknown name is a programming error", () => {
		expect(() => register("NO_SUCH_REGISTER")).toThrow(/not in the register map/);
	});
	it("health: bit 0 is ok, every other bit a problem", () => {
		expect(healthProblems(1)).toEqual([]);
		expect(healthProblems((1 << 1) | (1 << 4))).toEqual(["Übertemperatur", "Ethernet ohne Link"]);
	});
});

it("formats durations, sizes, names", () => {
	expect([formatUptime(12), formatUptime(247), formatUptime(7500), formatUptime(3 * 86400 + 4 * 3600 + 5)]).toEqual(["12 s", "4 min 7 s", "2 h 5 min", "3 d 4 h"]);
	expect([formatBytes(512), formatBytes(3482), formatBytes(1258291)]).toEqual(["512 B", "3.4 KiB", "1.20 MiB"]);
	expect(cString([0x69, 0x64, 0x6c, 0x65, 0, 0x78])).toBe("idle");
});
