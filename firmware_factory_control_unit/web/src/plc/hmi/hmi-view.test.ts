// @vitest-environment happy-dom
// The HMI components against the simulator: values shown, operating elements writing to the PLC, the
// runtime's alarm acknowledgement pulse, the editor's palette and keys.
import { describe, expect, it } from "vitest";
import { Simulator } from "../ide/simulator.ts";
import { build, type Project } from "../ide/project.ts";
import { newElement, type HmiElement, type HmiPage } from "./hmi.ts";
import type { HmiView } from "./hmi-view.ts";
import type { HmiEditor } from "./hmi-editor.ts";
import "./hmi-view.ts";
import "./hmi-editor.ts";

const project: Project = {
	format: 1, name: "T",
	objects: [
		{ id: "g", name: "GVL", kind: "GVL", lang: "ST", decl: "VAR_GLOBAL\n    Btn : BOOL;\n    Sw : BOOL;\n    Sp : INT := 10;\n    Ack : BOOL;\n    Acks : INT;\n    Temp : REAL := 21.5;\nEND_VAR", body: "" },
		{ id: "p", name: "PLC_PRG", kind: "PRG", lang: "ST", decl: "PROGRAM PLC_PRG\nVAR\n    Edge : R_TRIG;\nEND_VAR", body: "Edge(CLK := Ack);\nIF Edge.Q THEN Acks := Acks + 1; END_IF;\n" },
	],
	tasks: [{ name: "MainTask", intervalCycles: 1, programs: ["PLC_PRG"] }],
};

async function setup(elements: HmiElement[]) {
	const sim = new Simulator();
	expect(sim.download(build(project).result)).toBe(true);
	sim.singleCycle();
	const page: HmiPage = { id: "p1", name: "P", width: 800, height: 600, elements };
	const v = document.createElement("hmi-view") as HmiView;
	v.page = page;
	v.device = sim;
	v.mode = "run";
	document.body.appendChild(v);
	await v.updateComplete;
	const rerender = async () => {
		v.tick++;
		await v.updateComplete;
	};
	return { sim, v, rerender };
}

const el = (kind: HmiElement["kind"], over: Partial<HmiElement>): HmiElement => ({ ...newElement(kind, 0, 0), ...over });

describe("hmi-view in run mode", () => {
	it("shows values: digital display, LED, switch state", async () => {
		const { v, sim, rerender } = await setup([el("digital", { id: "d", var: "Temp", decimals: 2, unit: "°C" }), el("led", { id: "l", var: "Sw", color: "green" })]);
		expect(v.querySelector(".hmi-digits")?.textContent).toBe("21.50");
		expect(v.querySelector(".hmi-led-body")?.getAttribute("fill")).toBe("#123a1d");
		sim.write("Sw", true);
		await rerender();
		expect(v.querySelector(".hmi-led-body")?.getAttribute("fill")).toBe("#30d158");
		v.remove();
	});
	it("a button writes TRUE while pressed and FALSE on release; inverted the other way round", async () => {
		const { v, sim } = await setup([el("button", { id: "b", var: "Btn" })]);
		const press = v.querySelector(".hmi-press")!;
		press.dispatchEvent(new PointerEvent("pointerdown", { bubbles: true, pointerId: 1 }));
		await new Promise((r) => setTimeout(r));
		expect(sim.read("Btn").value).toBe(true);
		press.dispatchEvent(new PointerEvent("pointerup", { bubbles: true, pointerId: 1 }));
		await new Promise((r) => setTimeout(r));
		expect(sim.read("Btn").value).toBe(false);
		v.remove();
	});
	it("a switch toggles on a click; read-only elements do not write", async () => {
		const { v, sim, rerender } = await setup([el("switch", { id: "s", var: "Sw" }), el("switch", { id: "r", var: "Btn", readonly: true, x: 300 })]);
		const [sw, ro] = [...v.querySelectorAll(".hmi-switch g")];
		(sw as SVGGElement).dispatchEvent(new MouseEvent("click", { bubbles: true }));
		await new Promise((r) => setTimeout(r));
		expect(sim.read("Sw").value).toBe(true);
		await rerender();
		(sw as SVGGElement).dispatchEvent(new MouseEvent("click", { bubbles: true }));
		await new Promise((r) => setTimeout(r));
		expect(sim.read("Sw").value).toBe(false);
		(ro as SVGGElement).dispatchEvent(new MouseEvent("click", { bubbles: true }));
		await new Promise((r) => setTimeout(r));
		expect(sim.read("Btn").value).toBe(false);
		v.remove();
	});
	it("a knob steps with the arrow keys within its range and the variable's type", async () => {
		const { v, sim } = await setup([el("knob", { id: "k", var: "Sp", min: 0, max: 12, step: 1 })]);
		const g = v.querySelector(".hmi-knob g[tabindex]")!;
		for (let i = 0; i < 5; i++) g.dispatchEvent(new KeyboardEvent("keydown", { key: "ArrowUp", bubbles: true }));
		await new Promise((r) => setTimeout(r));
		expect(sim.read("Sp").value).toBe(12);
		g.dispatchEvent(new KeyboardEvent("keydown", { key: "PageDown", bubbles: true }));
		await new Promise((r) => setTimeout(r));
		expect(sim.read("Sp").value).toBe(2);
		v.remove();
	});
	it("an element bound to an unknown variable is marked and writes nothing", async () => {
		const notices: string[] = [];
		const { v } = await setup([el("switch", { id: "s", var: "NoSuchVar" })]);
		v.addEventListener("notice", (e) => notices.push((e as CustomEvent<string>).detail));
		expect(v.querySelector(".hmi-missing")).not.toBeNull();
		v.querySelector(".hmi-switch g")!.dispatchEvent(new MouseEvent("click", { bubbles: true }));
		await new Promise((r) => setTimeout(r));
		expect(notices[0]).toMatch(/NoSuchVar/);
		v.remove();
	});
});

describe("runtime acknowledgement", () => {
	it("acknowledging pulses the acknowledge variable: the program sees one rising edge", async () => {
		const { HmiRuntimeApp } = await import("./hmi-runtime-app.ts");
		const { sim } = await setup([]);
		const app = new HmiRuntimeApp();
		(app as unknown as { dev: Simulator }).dev = sim;
		(app as unknown as { project: Project }).project = { ...project, hmi: [{ id: "p", name: "P", width: 800, height: 600, elements: [el("alarm", { id: "a", var: "Sw", ackVar: "Ack" })] }] };
		(app as unknown as { shown: boolean }).shown = true;
		sim.write("Sw", true);
		sim.singleCycle();
		(app as unknown as { onUpdate(d: Simulator): void }).onUpdate(sim);
		(app as unknown as { ack(k: string | null): void }).ack(null);
		await new Promise((r) => setTimeout(r));
		sim.singleCycle();
		expect(sim.read("Acks").value).toBe(1);
		await new Promise((r) => setTimeout(r, 350));
		expect(sim.read("Ack").value).toBe(false);
	});
});

describe("hmi-editor", () => {
	it("adds from the palette, deletes with the Delete key, undoes", async () => {
		const ed = document.createElement("hmi-editor") as HmiEditor;
		let pages: HmiPage[] = [{ id: "p1", name: "P", width: 800, height: 600, elements: [] }];
		ed.pages = pages;
		ed.addEventListener("change", (e) => {
			pages = (e as CustomEvent<HmiPage[]>).detail;
			ed.pages = pages;
		});
		document.body.appendChild(ed);
		await ed.updateComplete;
		const knob = [...ed.querySelectorAll(".hmi-pal-item")].find((b) => b.textContent?.includes("Drehpotentiometer")) as HTMLButtonElement;
		knob.click();
		await ed.updateComplete;
		expect(pages[0].elements.map((e) => e.kind)).toEqual(["knob"]);
		window.dispatchEvent(new KeyboardEvent("keydown", { key: "Delete" }));
		await ed.updateComplete;
		expect(pages[0].elements).toHaveLength(0);
		window.dispatchEvent(new KeyboardEvent("keydown", { key: "z", ctrlKey: true }));
		await ed.updateComplete;
		expect(pages[0].elements).toHaveLength(1);
		ed.remove();
	});
});
