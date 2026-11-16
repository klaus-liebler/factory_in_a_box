// The HMI model: placing on the grid, values for writing, the variable list, the alarm list's states, the
// trend buffer, and pages in a stored project.
import { describe, expect, it } from "vitest";
import { T } from "../compiler/types.ts";
import { build, exampleProject, parseProject, projectHash } from "../ide/project.ts";
import {
	AlarmList, duplicate, examplePages, fraction, hmiProblem, newElement, newPage, place, quantize, removeElements, reorder, resize, scalarPaths,
	toValue, TrendBuffer, type HmiPage,
} from "./hmi.ts";

describe("pages and elements", () => {
	it("new elements snap to the grid and stay on the page", () => {
		const p = newPage("A");
		const e = newElement("knob", 1273, -14, p);
		expect([e.x, e.y]).toEqual([p.width - e.w, 0]);
		expect(place(e, 33, 47, p)).toMatchObject({ x: 30, y: 50 });
		expect(resize({ ...e, x: 1200 }, 400, 3, p)).toMatchObject({ w: 80, h: 20 });
	});
	it("duplicates with new ids, reorders, removes", () => {
		let p: HmiPage = { ...newPage("A"), elements: [newElement("led", 0, 0), newElement("switch", 100, 0)] };
		const [a, b] = p.elements;
		const d = duplicate(p, new Set([a.id]));
		expect(d.page.elements).toHaveLength(3);
		expect(d.ids.has(a.id)).toBe(false);
		expect(d.page.elements[2]).toMatchObject({ x: 20, y: 20, kind: "led" });
		p = reorder(d.page, new Set([a.id]), true);
		expect(p.elements[2].id).toBe(a.id);
		expect(removeElements(p, new Set([b.id])).elements.map((e) => e.id)).not.toContain(b.id);
	});
});

describe("values", () => {
	it("operator values become the variable's type", () => {
		expect(toValue(T.BOOL, 1)).toBe(true);
		expect(toValue(T.INT, 12.6)).toBe(13);
		expect(toValue(T.INT, 99999)).toBe(32767);
		expect(toValue(T.UINT, -5)).toBe(0);
		expect(toValue(T.REAL, 2.25)).toBe(2.25);
		expect(toValue(T.TIME, 1500.4)).toBe(1500);
		expect(toValue(T.LINT, 1e20)).toBe(9223372036854775807n);
	});
	it("knob and slider positions follow the step grid within the range; reversed ranges work", () => {
		expect(quantize(47.4, 0, 100, 5)).toBe(45);
		expect(quantize(-3, 0, 100, 5)).toBe(0);
		expect(quantize(0.3, 0, 1, 0.1)).toBe(0.3);
		expect(quantize(103, 0, 100, 0)).toBe(100);
		expect(quantize(5, 10, 0, 1)).toBe(5);
		expect(fraction(5, 0, 0)).toBe(0);
		expect(fraction(150, 0, 100)).toBe(1);
	});
});

describe("variables of a build", () => {
	it("lists globals, I/O and program variables, FB members, not the libraries' internals", () => {
		const b = build(exampleProject());
		const paths = scalarPaths(b.result.debug!).map((c) => c.path.toUpperCase());
		expect(paths).toContain("ENABLE");
		expect(paths).toContain("CONVEYORPWM");
		expect(paths).toContain("PLC_PRG.PIECES.CV");
		expect(paths).toContain("PLC_PRG.EJECTOR.VALVE");
		expect(paths.some((p) => p.includes("__"))).toBe(false);
		expect(paths.indexOf("ENABLE")).toBeLessThan(paths.indexOf("PLC_PRG.EJECTED"));
	});
	it("every variable of the example page exists in the example program", () => {
		const debug = build(exampleProject()).result.debug!;
		for (const e of examplePages()[0].elements) {
			for (const v of [e.var, ...(e.pens ?? []).map((p) => p.var)].filter(Boolean)) expect(debug.resolve(v), v).not.toBeNull();
		}
	});
});

describe("alarm list", () => {
	const page: HmiPage = { ...newPage("P"), elements: [{ ...newElement("alarm", 0, 0), id: "a", text: "Druck", prio: 3, ackVar: "Ack" }] };
	it("come, acknowledge, go (states as VDI3814.NC)", () => {
		const l = new AlarmList();
		let on = true;
		l.update([page], () => on, 1000);
		expect(l.list()[0]).toMatchObject({ active: true, acked: false, came: 1000 });
		expect(l.unacked).toBe(1);
		expect(l.ack(null, 2000)).toEqual(["Ack"]);
		expect(l.list()[0].acked).toBe(true);
		on = false;
		l.update([page], () => on, 3000);
		expect(l.list()).toHaveLength(0);  // gone and acknowledged
		expect(l.history.map((h) => h.what)).toEqual(["geht", "quittiert", "kommt"]);
	});
	it("gone unacknowledged stays until acknowledged; no value changes nothing", () => {
		const l = new AlarmList();
		l.update([page], () => true, 1);
		l.update([page], () => null, 2);
		expect(l.list()[0].active).toBe(true);
		l.update([page], () => false, 3);
		expect(l.list()[0]).toMatchObject({ active: false, acked: false, went: 3 });
		l.ack("P".length ? l.list()[0].key : null, 4);
		expect(l.list()).toHaveLength(0);
		l.update([page], () => true, 5);
		l.update([], () => true, 6);  // the element was removed from the project
		expect(l.list()).toHaveLength(0);
	});
});

it("trend buffer drops samples older than the span and keeps time order", () => {
	const b = new TrendBuffer(2);
	for (let t = 0; t <= 20_000; t += 100) b.push(t, [t, null], 10_000);
	b.push(5, [1, 1], 10_000);
	expect(b.t[0]).toBeGreaterThanOrEqual(20_000 - 10_500);
	expect(b.t[b.t.length - 1]).toBe(20_000);
	expect(b.v[1].every((v) => v === null)).toBe(true);
});

describe("pages in the project", () => {
	it("are stored and loaded with the project and do not change the project hash", () => {
		const p = exampleProject();
		const q = parseProject(JSON.stringify(p));
		expect(q.hmi?.[0].elements.length).toBe(p.hmi![0].elements.length);
		expect(projectHash({ ...p, hmi: [] })).toBe(projectHash(p));
		expect(parseProject(JSON.stringify({ ...p, hmi: undefined })).hmi).toBeUndefined();
	});
	it("broken pages are refused with a reason", () => {
		expect(hmiProblem([{ id: "x", name: "A", width: 10, height: 10, elements: [{ kind: "rocket" }] }])).toMatch(/Element ungültig/);
		expect(hmiProblem({})).toMatch(/keine Liste/);
		const p = exampleProject();
		expect(() => parseProject(JSON.stringify({ ...p, hmi: [{ ...p.hmi![0], elements: [{ ...p.hmi![0].elements[0], x: "1" }] }] }))).toThrow(/Element ungültig/);
	});
});
