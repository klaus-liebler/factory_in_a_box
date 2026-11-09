// FBD and SFC lower to ST that the compiler takes and the VM runs (docs/plc-iec61131.md 5.3, 5.4).
import { describe, expect, it } from "vitest";
import { compile, type CompileResult } from "../compiler/compiler.ts";
import { formatDiagnostic } from "../compiler/diagnostics.ts";
import { Vm } from "../vm.ts";
import { exampleFbd, fbdSource, literalType, lowerFbd, networkTypes, pinKey, typeClass, type FbdPou } from "./fbd.ts";
import { ioBindings, ioMapHash } from "./io-list.ts";
import { deviceSources } from "./project.ts";
import { exampleSfc, sfcGraph, sfcSource, type SfcPou } from "./sfc.ts";
import { readValue, writeValue, type Value } from "./values.ts";

function build(name: string, text: string): CompileResult {
	const r = compile(deviceSources(name, text), [{ name: "MainTask", intervalCycles: 1, programs: [name] }], { io: ioBindings(), ioMapHash: ioMapHash() });
	if (!r.ok) throw new Error(`${r.diagnostics.map(formatDiagnostic).join("\n")}\n---\n${text}`);
	return r;
}

/** The program in the VM, cycle by cycle (50 ms), its variables by path. */
function machine(r: CompileResult) {
	const data = new Uint8Array(32 * 1024);
	const vm = new Vm(data);
	expect(vm.load(r.image!)).toBe("none");
	let t = 0;
	return {
		get: (p: string) => readValue(data, r.debug!.resolve(p)!),
		set: (p: string, v: Value) => writeValue(data, r.debug!.resolve(p)!, v),
		cycle: (n = 1) => {
			for (let i = 0; i < n; i++) expect(vm.runCycle(0, (t += 50)).state).toBe("done");
		},
	};
}

describe("FBD", () => {
	it("lowers the example: instances, calls in dependency order, expressions inlined", () => {
		const l = lowerFbd(exampleFbd());
		expect(l.problems).toEqual([]);
		expect(l.instances).toContain("Pieces : CTU;");
		expect(l.body).toContain("Pieces(CU := Lightbarrier1, R := (NOT Enable), PV := 10);");
		expect(l.body).toContain("ConveyorPwm := SEL((Enable AND (NOT Pieces.Q)), 0, 600);");
		expect(l.body).toContain("Eject(IN := (AtGate.Q AND Pieces.CV MOD 2 = 0), PT := T#300ms);");
		// AtGate is called before Eject, which reads its output
		expect(l.body.indexOf("AtGate(")).toBeLessThan(l.body.indexOf("Eject("));
	});

	it("the example compiles and runs: conveyor on, counts, stops at 10", () => {
		const p = exampleFbd();
		const m = machine(build(p.name, fbdSource(p).text));
		m.cycle();
		expect(m.get("ConveyorPwm")).toBe(600);
		for (let i = 0; i < 10; i++) {
			m.set("Lightbarrier1", true);
			m.cycle();
			m.set("Lightbarrier1", false);
			m.cycle();
		}
		expect(m.get("FBD_PRG.Pieces.CV")).toBe(10);
		expect(m.get("ConveyorPwm")).toBe(0);
	});

	it("pin types: fixed pins, variables, constants, generic pins through wires and boxes; mismatches", () => {
		const vars: Record<string, string> = { ENABLE: "BOOL", "PIECES.Q": "BOOL", CONVEYORPWM: "UINT" };
		const varType = (n: string) => vars[n.toUpperCase()] ?? null;
		const net2 = exampleFbd().networks[1];
		const t = networkTypes(net2, varType);
		expect(t.pins.get(pinKey("b2", "in", "IN1"))).toBe("bool");  // AND from Enable
		expect(t.pins.get(pinKey("b2", "out", "OUT"))).toBe("bool");  // the AND's generic group
		expect(t.pins.get(pinKey("b3", "in", "G"))).toBe("bool");  // fixed
		expect(t.pins.get(pinKey("i3", "out", "OUT"))).toBe("int");  // the constant 0
		expect(t.pins.get(pinKey("b3", "in", "IN1"))).toBe("int");  // over the wire from 600
		expect(t.pins.get(pinKey("b3", "out", "OUT"))).toBe("int");  // SEL's generic group
		expect(t.mismatches.size).toBe(0);

		const net3 = networkTypes(exampleFbd().networks[2], varType);
		expect(net3.pins.get(pinKey("i3", "out", "OUT"))).toBe("time");  // T#300ms
		expect(net3.pins.get(pinKey("b3", "in", "PT"))).toBe("time");
		expect(net3.pins.get(pinKey("i2", "out", "OUT"))).toBe("bool");  // a comparison

		const bad: FbdPou["networks"][number] = { id: "n", title: "", nodes: [
			{ kind: "input", id: "i", x: 0, y: 0, expr: "1.5" },
			{ kind: "box", id: "c", x: 100, y: 0, type: "CTU", instance: "C" },
		], wires: [{ from: { node: "i", pin: "OUT" }, to: { node: "c", pin: "PV" } }] };
		const tb = networkTypes(bad, varType);
		expect(tb.pins.get(pinKey("i", "out", "OUT"))).toBe("real");
		expect([...tb.mismatches]).toEqual([0]);  // REAL into INT
		expect(literalType("TIME#1s")).toBe("TIME");
		expect(literalType("16#FF")).toBe("DINT");
		expect(literalType("Valve1")).toBeNull();
		expect(typeClass("LREAL")).toBe("real");
		expect(typeClass("DT")).toBe("time");
	});

	it("reports an open input and an operator loop; a feedback through an instance is fine", () => {
		const p: FbdPou = { name: "P", decl: "PROGRAM P", networks: [{ id: "n", title: "", wires: [], nodes: [
			{ kind: "box", id: "a", x: 0, y: 0, type: "AND" },
			{ kind: "output", id: "o", x: 100, y: 0, target: "Valve1" },
		] }] };
		p.networks[0].wires.push({ from: { node: "a", pin: "OUT" }, to: { node: "o", pin: "IN" } });
		expect(lowerFbd(p).problems.map((x) => x.message)).toEqual(["Eingang IN1 ist nicht verbunden", "Eingang IN2 ist nicht verbunden"]);
		p.networks[0].wires.push({ from: { node: "a", pin: "OUT" }, to: { node: "a", pin: "IN1" } });
		expect(lowerFbd(p).problems.some((x) => x.message.includes("Kreis"))).toBe(true);

		const fb: FbdPou = { name: "P", decl: "PROGRAM P", networks: [{ id: "n", title: "", nodes: [
			{ kind: "box", id: "t", x: 0, y: 0, type: "TON", instance: "Blink" },
			{ kind: "box", id: "n", x: 0, y: 0, type: "NOT" },
			{ kind: "input", id: "p", x: 0, y: 0, expr: "T#100ms" },
		], wires: [
			{ from: { node: "t", pin: "Q" }, to: { node: "n", pin: "IN" } },
			{ from: { node: "n", pin: "OUT" }, to: { node: "t", pin: "IN" } },
			{ from: { node: "p", pin: "OUT" }, to: { node: "t", pin: "PT" } },
		] }] };
		const l = lowerFbd(fb);
		expect(l.problems).toEqual([]);
		expect(l.body).toContain("Blink(IN := (NOT Blink.Q), PT := T#100ms);");
	});
});

describe("SFC", () => {
	it("the example's graph: alternative with priority, parallel split and join, the jump", () => {
		const g = sfcGraph(exampleSfc());
		expect(g.problems).toEqual([]);
		const t = (from: string) => g.transitions.filter((x) => x.sources.join() === from).map((x) => x.targets.join());
		expect(t("Start")).toEqual(["Detect"]);
		expect(t("Detect")).toEqual(["Eject", "Pass"]);
		expect(t("Eject")).toEqual(["Count"]);
		expect(t("Count")).toEqual(["Pump,Hold"]);  // parallel split
		expect(t("PumpDone,Hold")).toEqual(["Release"]);  // the join waits for both
		expect(t("Release")).toEqual(["Start"]);  // the jump
		const alts = g.transitions.filter((x) => x.sources.join() === "Detect");
		expect(alts[1].before).toEqual([alts[0].id]);  // the right one only when the left one is false
	});

	it("the example runs: steps, step times, both alternatives, the join, actions N/S/R/P", () => {
		const p = exampleSfc();
		const m = machine(build(p.name, sfcSource(p).text));
		m.cycle();
		expect(m.get("SFC_PRG.Start.X")).toBe(true);
		expect(m.get("ConveyorPwm")).toBe(600);
		m.set("Lightbarrier1", true);
		m.cycle();
		expect(m.get("SFC_PRG.Detect.X")).toBe(true);
		expect(m.get("SFC_PRG.Start.X")).toBe(false);
		m.set("Lightbarrier2", true);
		m.cycle();
		expect(m.get("SFC_PRG.Eject.X")).toBe(true);
		expect(m.get("Valve1")).toBe(true);  // N with a variable
		m.cycle(7);  // Eject.T reaches 300 ms
		expect(m.get("SFC_PRG.Count.X")).toBe(true);
		expect(m.get("Valve1")).toBe(false);
		expect(m.get("SFC_PRG.Pieces")).toBe(1);  // P: once on entering
		m.cycle();
		expect(m.get("SFC_PRG.Pump.X") && m.get("SFC_PRG.Hold.X")).toBe(true);  // parallel
		expect(m.get("SFC_PRG.Pieces")).toBe(1);
		expect(m.get("Valve3")).toBe(true);  // S
		m.cycle(12);
		expect(m.get("SFC_PRG.PumpDone.X")).toBe(true);
		m.cycle();
		expect(m.get("SFC_PRG.Release.X")).toBe(true);  // joined
		expect(m.get("SFC_PRG.Hold.X")).toBe(false);
		expect(m.get("Valve3")).toBe(false);  // R
		m.cycle(6);
		expect(m.get("SFC_PRG.Start.X")).toBe(true);  // the jump back

		// the other alternative: no piece at light barrier 2 within 2 s
		m.set("Lightbarrier2", false);
		m.cycle();
		expect(m.get("SFC_PRG.Detect.X")).toBe(true);
		let pass = 0, eject = false;
		for (let i = 0; i < 60 && !pass; i++) {
			m.cycle();
			if (m.get("SFC_PRG.Pass.X")) pass = i + 1;
			if (m.get("SFC_PRG.Eject.X")) eject = true;
		}
		expect(eject).toBe(false);
		expect(pass).toBeGreaterThanOrEqual(40);  // after Detect.T > 2 s
		expect(pass).toBeLessThanOrEqual(42);
	});

	it("reports a broken structure", () => {
		const p: SfcPou = { name: "P", decl: "PROGRAM P", chart: [
			{ kind: "step", id: "a", name: "A", actions: [] },
			{ kind: "step", id: "b", name: "A", actions: [] },
			{ kind: "trans", id: "t", condition: "" },
			{ kind: "jump", id: "j", target: "Nowhere" },
		] };
		const msgs = sfcGraph(p).problems.map((x) => x.message);
		expect(msgs).toContain("zwischen zwei Schritten fehlt eine Transition (vor A)");
		expect(msgs).toContain("Schritt A doppelt");
		expect(msgs).toContain("Transition ohne Bedingung");
		expect(msgs).toContain("Sprungziel Nowhere gibt es nicht");
	});
});
