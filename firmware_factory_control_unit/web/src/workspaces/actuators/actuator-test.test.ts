// @vitest-environment happy-dom
// Aktorentest and the direct-operation controller: writes only while the fieldbus switches the outputs, the
// take-over asks first, refusals are reported, a pulse switches off again, "Alles aus" reaches every output.
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

const dev = vi.hoisted(() => ({
	iec: false,
	state: 2,
	values: { input: [] as number[], holding: [] as number[] },
	writes: [] as [string, number][],
	refuse: false,
	sources: [] as boolean[],
	confirm: true,
	offline: false,
}));

vi.mock("../../core/registers.ts", async (importOriginal) => ({
	...(await importOriginal<object>()),
	fetchRegisters: async () => dev.values,
	writeRegister: async (reg: string | { name: string }, value: number) => {
		const name = typeof reg === "string" ? reg : reg.name;
		if (dev.refuse || dev.iec) throw new Error("refused");
		dev.writes.push([name, value]);
	},
}));
vi.mock("../../core/plc.ts", async (importOriginal) => ({
	...(await importOriginal<object>()),
	fetchPlcStatus: async () => {
		if (dev.offline) throw new Error("offline");
		return { source: dev.iec ? 1 : 0, state: dev.state };
	},
	isIec: (s: { source: number } | null) => s?.source === 1,
	setControlSource: async (iec: boolean) => {
		dev.sources.push(iec);
		dev.iec = iec;
	},
}));
vi.mock("../../shell/dialogs.ts", () => ({ confirmDialog: async () => dev.confirm }));

import { register } from "../../core/registers.ts";
import { appStatusOf } from "../../shell/status-bar.ts";
import "./actuator-test-app.ts";
import type { ActuatorTestApp } from "./actuator-test-app.ts";

const flush = async (app: ActuatorTestApp, ms = 0) => {
	await vi.advanceTimersByTimeAsync(ms);
	await app.updateComplete;
};

async function open(): Promise<ActuatorTestApp> {
	const app = document.createElement("actuator-test-app") as ActuatorTestApp;
	document.body.appendChild(app);
	await app.updateComplete;
	app.onShow();
	await flush(app, 10);
	return app;
}

const buttons = (app: ActuatorTestApp, text: string) => [...app.querySelectorAll("button")].filter((b) => b.textContent?.trim() === text) as HTMLButtonElement[];

beforeEach(() => {
	vi.useFakeTimers();
	Object.assign(dev, { iec: false, state: 2, writes: [], refuse: false, sources: [], confirm: true, offline: false });
	dev.values = { input: [], holding: [] };
	for (const r of ["CONVEYOR_PWM", "COMPRESSOR_PWM", "VALVE1", "VALVE2", "VALVE3", "VACUUM_VALVE", "VACUUM_PUMP", "WS2812_CH1_PATTERN", "WS2812_CH2_PATTERN"]) {
		dev.values.holding[register(r).address] = 0;
	}
	document.body.innerHTML = "";
});
afterEach(() => vi.useRealTimers());

describe("Aktorentest", () => {
	it("fieldbus: switches a valve, sets the conveyor, selects a strip pattern", async () => {
		const app = await open();
		expect(app.querySelector(".direct-free")).not.toBeNull();
		buttons(app, "Aus").find((b) => b.classList.contains("act-toggle"))!.click();
		buttons(app, "50 %")[0].click();
		(app.querySelector(".act-pattern[title='Grün']") as HTMLButtonElement).click();
		await flush(app);
		expect(dev.writes).toEqual([["VALVE1", 1], ["CONVEYOR_PWM", 500], ["WS2812_CH1_PATTERN", 3]]);
	});

	it("IEC: everything locked; the take-over asks and switches to the fieldbus", async () => {
		dev.iec = true;
		const app = await open();
		expect(app.querySelector(".direct-locked")).not.toBeNull();
		expect(buttons(app, "Impuls").every((b) => b.disabled)).toBe(true);
		expect(buttons(app, "Aus").every((b) => b.disabled)).toBe(true);
		dev.confirm = false;
		(app.querySelector(".direct-take") as HTMLButtonElement).click();
		await flush(app);
		expect(dev.sources).toEqual([]);
		dev.confirm = true;
		(app.querySelector(".direct-take") as HTMLButtonElement).click();
		await flush(app);
		expect(dev.sources).toEqual([false]);
		expect(app.querySelector(".direct-free")).not.toBeNull();
	});

	it("control source unknown (no answer): every control is locked", async () => {
		dev.offline = true;
		const app = await open();
		expect(app.querySelector(".direct-unknown")).not.toBeNull();
		const controls = [...app.querySelectorAll(".act button, .act input")] as (HTMLButtonElement | HTMLInputElement)[];
		expect(controls.length).toBeGreaterThan(20);
		expect(controls.every((c) => c.disabled)).toBe(true);
	});

	it("a refused write is reported in the status bar", async () => {
		const app = await open();
		dev.refuse = true;
		buttons(app, "Aus").find((b) => b.classList.contains("act-toggle"))!.click();
		await flush(app);
		expect(appStatusOf(app)).toMatchObject({ kind: "error", text: "VALVE1 nicht geschrieben" });
		dev.iec = true;
		buttons(app, "Impuls")[1].click();
		await flush(app);
		expect(appStatusOf(app)?.text).toMatch(/das SPS-Programm schaltet die Ausgänge/);
	});

	it("a pulse switches on and off again after 500 ms", async () => {
		const app = await open();
		buttons(app, "Impuls")[0].click();
		await flush(app, 10);
		expect(dev.writes).toEqual([["VALVE1", 1]]);
		await flush(app, 500);
		expect(dev.writes).toEqual([["VALVE1", 1], ["VALVE1", 0]]);
	});

	it("Alles aus writes 0 to every output of the page", async () => {
		const app = await open();
		(app as unknown as { allOff(): void }).allOff();
		await flush(app);
		expect(dev.writes.map(([r]) => r).sort()).toEqual(["COMPRESSOR_PWM", "CONVEYOR_PWM", "VACUUM_PUMP", "VACUUM_VALVE", "VALVE1", "VALVE2", "VALVE3", "WS2812_CH1_PATTERN", "WS2812_CH2_PATTERN"]);
		expect(dev.writes.every(([, v]) => v === 0)).toBe(true);
	});
});
