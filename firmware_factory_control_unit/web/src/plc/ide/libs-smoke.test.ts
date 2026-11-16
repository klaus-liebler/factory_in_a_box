// Every block of the VDI3814 and SIM libraries compiles, appears as an FBD box with a title and runs
// 200 cycles with its default inputs without a runtime error.
import { describe, expect, it } from "vitest";
import { LibRun } from "./lib-harness.ts";
import { boxesFor, exampleProject } from "./project.ts";

const p = exampleProject();
const boxes = boxesFor(p, p.objects[0]).filter((b) => b.type.startsWith("VDI3814.") || b.type.startsWith("SIM."));

describe("library smoke test", () => {
	it("offers the blocks as boxes in their groups, titled, without the helpers", () => {
		expect(boxes.length).toBeGreaterThan(55);
		expect(boxes.some((b) => b.type.includes("__"))).toBe(false);
		for (const b of boxes) expect(b.title, b.type).not.toMatch(/^Funktion/);
		expect(boxes.find((b) => b.type === "VDI3814.PID")?.group).toBe("VDI 3814 · 2.3 Regelung");
		expect(boxes.find((b) => b.type === "SIM.TANK")?.group).toBe("Simulation · Pneumatik/Hydraulik");
	});

	it("runs every function block with its default inputs", () => {
		const fbs = boxes.filter((b) => b.kind === "fb");
		const vars = fbs.map((b, i) => `b${i} : ${b.type};`).join("\n");
		const body = fbs.map((b, i) => `b${i}();`).join("\n");
		const r = new LibRun(vars, body);
		r.cycle(200);
	});

	it("calls the functions", () => {
		const r = new LibRun("y : REAL;", "y := VDI3814.SCALE(5.0, 0.0, 10.0, 0.0, 100.0);");
		r.cycle();
		expect(r.num("y")).toBeCloseTo(50);
	});
});
