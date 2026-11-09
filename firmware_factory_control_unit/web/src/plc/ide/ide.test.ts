// The PLC page's logic without the DOM: example project, simulator with the plant model,
// declaration table <-> text, value parsing.
import { describe, expect, it } from "vitest";
import { addRow, deleteRow, parseRows, updateRow } from "./decl-table.ts";
import { COMMAND_VARS, IO_VARS, ioSource } from "./io-list.ts";
import { addObject, build, declLines, exampleProject, nameProblem, removeObject, renameIn, renameObject, type Project } from "./project.ts";
import { Simulator } from "./simulator.ts";
import { formatTime, parseValue } from "./values.ts";
import { T } from "../compiler/types.ts";

/** Runs n cycles of the simulator synchronously (instead of its timer). */
const prgOf = (p: Project) => p.objects.find((o) => o.name === "PLC_PRG")!;

function cycles(sim: Simulator, n: number) {
	for (let i = 0; i < n; i++) (sim as unknown as { cycle(): void }).cycle();
}

describe("PLC IDE", () => {
	it("the I/O list compiles and maps the register names", () => {
		expect(IO_VARS.find((v) => v.name === "ConveyorPwm")).toMatchObject({ type: "UINT", direction: "output", address: "%QW40" });
		expect(IO_VARS.find((v) => v.name === "Lightbarrier1")).toMatchObject({ type: "BOOL", direction: "input", address: "%IX180" });
		expect(ioSource()).toContain("Stepper1Target : DINT;");
	});

	it("the example project compiles without diagnostics", () => {
		const b = build(exampleProject());
		expect(b.located.map((l) => l.diagnostic.message)).toEqual([]);
		expect(b.result.ok).toBe(true);
	});

	it("the example sorts pieces on the simulated conveyor", () => {
		const sim = new Simulator();
		expect(sim.download(build(exampleProject()).result)).toBe(true);
		sim.state = "RUN"; // without the timer
		cycles(sim, 600); // 30 s
		expect(sim.state).toBe("RUN");
		expect(sim.read("PLC_PRG.Pieces.CV").value).toBe(10); // stops after 10 pieces
		expect(sim.read("ConveyorPwm").value).toBe(0);
		expect(sim.plant.ejected).toBeGreaterThanOrEqual(4); // every second one
		expect(sim.read("PLC_PRG.Ejected").value).toBe(sim.plant.ejected);
	});

	it("maps diagnostics to declaration and implementation lines", () => {
		const p = exampleProject();
		prgOf(p).body = "ConveyorPwm := 600;\nFoo := 1;\n";
		const b = build(p);
		expect(b.located.map((l) => [l.object, l.part, l.line, l.diagnostic.message])).toEqual([["plc_prg", "body", 2, "unknown variable 'Foo'"]]);
	});

	it("reports a runtime error with its line", () => {
		const p = exampleProject();
		prgOf(p).body = "Ejected := Ejected + 1;\nConveyorPwm := INT_TO_UINT(10 / (3 - Ejected));\n"; // cycle 3: / 0
		const sim = new Simulator();
		sim.download(build(p).result);
		sim.state = "RUN";
		cycles(sim, 5);
		expect(sim.state).toBe("ERROR");
		expect(sim.error?.trap).toBe("DIV_BY_ZERO");
		expect(sim.error!.line! - declLines(prgOf(p))).toBe(2);
	});

	it("forced values win against the program and the plant", () => {
		const sim = new Simulator();
		sim.download(build(exampleProject()).result);
		sim.state = "RUN";
		sim.force("ConveyorPwm", 123);
		sim.force("Enable", false);
		cycles(sim, 3);
		expect(sim.read("ConveyorPwm").value).toBe(123);
		expect(sim.read("Enable").value).toBe(false);
	});

	it("declaration table edits the text", () => {
		const decl = "PROGRAM P\nVAR\n    a : INT := 1; (* first *)\nEND_VAR\nVAR_INPUT\n    b : BOOL;\nEND_VAR";
		const rows = parseRows(decl);
		expect(rows.map((r) => [r.cls, r.name, r.type, r.init, r.comment])).toEqual([["VAR", "a", "INT", "1", "first"], ["VAR_INPUT", "b", "BOOL", "", ""]]);
		const renamed = updateRow(decl, rows[0], { name: "count", type: "DINT" });
		expect(parseRows(renamed)[0]).toMatchObject({ name: "count", type: "DINT", init: "1", comment: "first" });
		const moved = updateRow(decl, rows[0], { cls: "VAR_INPUT" });
		expect(parseRows(moved).map((r) => `${r.cls}:${r.name}`)).toEqual(["VAR_INPUT:b", "VAR_INPUT:a"]);
		const added = addRow(decl, { cls: "VAR_OUTPUT", qualifier: "", name: "q", type: "BOOL", init: "", comment: "" });
		expect(added.endsWith("VAR_OUTPUT\n    q : BOOL;\nEND_VAR")).toBe(true);
		expect(parseRows(deleteRow(decl, rows[1])).map((r) => r.name)).toEqual(["a"]);
	});

	it("parses and formats values", () => {
		expect(parseValue(T.BOOL, "true")).toBe(true);
		expect(parseValue(T.INT, "70000")).toBe(null);
		expect(parseValue(T.BYTE, "16#ff")).toBe(255);
		expect(parseValue(T.TIME, "T#1m2s5ms")).toBe(62005);
		expect(parseValue(T.LINT, "-5")).toBe(-5n);
		expect(formatTime(62005)).toBe("T#1m2s5ms");
	});
});

describe("PLC IDE: several POUs (P4)", () => {
	it("the example is split into GVL, FB and PRG; the FB's instance is visible online", () => {
		const p = exampleProject();
		expect(p.objects.map((o) => [o.name, o.kind])).toEqual([["GVL", "GVL"], ["FB_Ejector", "FB"], ["PLC_PRG", "PRG"]]);
		const sim = new Simulator();
		expect(sim.download(build(p).result)).toBe(true);
		sim.state = "RUN";
		cycles(sim, 600);
		expect(sim.read("PLC_PRG.Ejector.Count").value).toBe(sim.plant.ejected);
		expect(sim.read("PiecesMax").value).toBe(10);
	});

	it("diagnostics belong to their object (FB, GVL, DUT)", () => {
		let p = exampleProject();
		const fb = p.objects.find((o) => o.kind === "FB")!;
		fb.body = "Valve := Arrive;\nCount := Missing;\n";
		p = addObject(p, "DUT", "ST_Part").project;
		p.objects.find((o) => o.name === "ST_Part")!.decl = "TYPE ST_Part :\nSTRUCT\n    Size : NoType;\nEND_STRUCT\nEND_TYPE";
		const b = build(p);
		expect(b.located.map((l) => [l.object, l.part, l.line]).sort()).toEqual([["fb_ejector", "body", 2], ["st_part", "decl", 3]]);
	});

	it("adds, renames (with every reference) and removes objects", () => {
		let p = exampleProject();
		expect(nameProblem(p, "fb_ejector")).toBe("Name schon vergeben");
		expect(nameProblem(p, "2x")).toBe("kein gültiger Bezeichner");
		const added = addObject(p, "PRG", "PRG_Lamp");
		p = added.project;
		expect(p.tasks[0].programs).toEqual(["PLC_PRG", "PRG_Lamp"]);
		p = renameObject(p, "fb_ejector", "FB_Pusher");
		const prg = p.objects.find((o) => o.name === "PLC_PRG")!;
		expect(prg.decl).toContain("Ejector : FB_Pusher;");
		expect(p.objects.find((o) => o.id === "fb_ejector")!.decl.startsWith("FUNCTION_BLOCK FB_Pusher")).toBe(true);
		p = renameObject(p, added.object.id, "PRG_Light");
		expect(p.tasks[0].programs).toEqual(["PLC_PRG", "PRG_Light"]);
		expect(build(p).result.ok).toBe(true);
		p = removeObject(p, added.object.id);
		expect(p.tasks[0].programs).toEqual(["PLC_PRG"]);
		expect(build(p).result.ok).toBe(true);
	});

	it("renaming leaves comments, strings and fields of the same name alone; enum literals follow", () => {
		const t = "x : Mode := Mode#Auto; (* Mode *) s := 'Mode'; y := a.Mode; z := MODE;";
		expect(renameIn(t, "Mode", "E_Mode")).toBe("x : E_Mode := E_Mode#Auto; (* Mode *) s := 'Mode'; y := a.Mode; z := E_Mode;");
	});
});

describe("PLC IDE: I/O bindings", () => {
	it("the image binds every I/O variable to its register; command registers are not I/O", async () => {
		const { Vm } = await import("../vm.ts");
		const r = build(exampleProject()).result;
		const vm = new Vm(new Uint8Array(32 * 1024));
		expect(vm.load(r.image!)).toBe("none");
		expect(vm.io.length).toBe(IO_VARS.length + COMMAND_VARS.length);
		const pwm = vm.io.find((b) => b.reg === 40)!;
		expect(pwm).toMatchObject({ area: 1, kind: 1, bits: 16, signed: false, addr: r.debug!.resolve("ConveyorPwm")!.addr });
		const target = vm.io.find((b) => b.reg === 61)!;
		expect(target).toMatchObject({ bits: 32, signed: true });
		expect(IO_VARS.some((v) => v.name === "Stepper1StartHoming")).toBe(false);
		expect(vm.io.find((b) => b.reg === 69)).toMatchObject({ area: 1, kind: 2, bits: 16 }); // command
	});
});
