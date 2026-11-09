// Online values of FBD operator outputs (fbd-live.ts): computed in the browser from the input values.
// The differential test compares them with what the VM computed for the same network.
import { describe, expect, it } from "vitest";
import { compile } from "../compiler/compiler.ts";
import { formatDiagnostic } from "../compiler/diagnostics.ts";
import { Vm } from "../vm.ts";
import { boxDef, fbdSource, type FbdNetwork, type FbdNode, type FbdWire } from "./fbd.ts";
import { evalBox, formatLive, parseLive } from "./fbd-live.ts";
import { ioBindings, ioMapHash } from "./io-list.ts";
import { deviceSources } from "./project.ts";
import { formatValue, readValue, writeValue, type Value } from "./values.ts";

describe("FBD live values of operators", () => {
	it("parses and formats what values.ts shows", () => {
		for (const t of ["TRUE", "FALSE", "42", "-7", "16#FF", "1.5", "2.0", "T#1s500ms", "T#0ms"]) expect(formatLive(parseLive(t)!)).toBe(t);
		expect(parseLive("INT#21")).toMatchObject({ k: "int", v: 21n });
		expect(parseLive("Variable")).toBeNull();
		expect(parseLive("D#2024-01-01")).toBeNull();
	});

	it("evaluates the standard operators; unknown or unfitting inputs give no value", () => {
		const ev = (type: string, ...ins: (string | null)[]) => evalBox(boxDef(type)!, ins);
		expect(ev("AND", "TRUE", "FALSE")).toBe("FALSE");
		expect(ev("XOR", "TRUE", "FALSE")).toBe("TRUE");
		expect(ev("AND", "16#F0", "16#3C")).toBe("16#30");
		expect(ev("NOT", "TRUE")).toBe("FALSE");
		expect(ev("NOT", "5")).toBeNull();  // a bit string's width is not known
		expect(ev("ADD", "40", "2")).toBe("42");
		expect(ev("DIV", "-7", "2")).toBe("-3");  // truncated toward zero
		expect(ev("DIV", "7", "0")).toBeNull();
		expect(ev("MOD", "-7", "3")).toBe("-1");
		expect(ev("MOD", "7", "0")).toBe("0");
		expect(ev("DIV", "1.0", "4")).toBe("0.25");
		expect(ev("ADD", "T#1s", "T#500ms")).toBe("T#1s500ms");
		expect(ev("MUL", "T#300ms", "3")).toBe("T#900ms");
		expect(ev("GT", "T#1s", "T#2s")).toBe("FALSE");
		expect(ev("GT", "T#1s", "2")).toBeNull();
		expect(ev("SEL", "TRUE", "1", "2")).toBe("2");
		expect(ev("LIMIT", "0", "15", "10")).toBe("10");
		expect(ev("MAX", "3", "2.5")).toBe("3");
		expect(ev("ADD", "1", null)).toBeNull();
		expect(ev("AND", "TRUE", "1")).toBeNull();
	});

	it("matches the VM on a network of operators (differential)", () => {
		// inputs a, b (INT), x (REAL), t (TIME), f (BOOL); every operator writes its own output variable
		const nodes: FbdNode[] = [];
		const wires: FbdWire[] = [];
		const vars = ["a", "b", "x", "t", "f"];
		for (const v of vars) nodes.push({ kind: "input", id: v, x: 0, y: 0, expr: v });
		const ops: [string, string[], string][] = [
			["ADD", ["a", "b"], "INT"], ["SUB", ["a", "b"], "INT"], ["MUL", ["a", "b"], "INT"], ["DIV", ["a", "b"], "INT"], ["MOD", ["a", "b"], "INT"],
			["GT", ["a", "b"], "BOOL"], ["LE", ["a", "b"], "BOOL"], ["EQ", ["a", "b"], "BOOL"], ["MAX", ["a", "b"], "INT"], ["MIN", ["a", "b"], "INT"],
			["SEL", ["f", "a", "b"], "INT"], ["LIMIT", ["b", "a", "c10"], "INT"], ["AND", ["f", "g"], "BOOL"], ["XOR", ["f", "g"], "BOOL"],
			["ADD", ["x", "x"], "REAL"], ["DIV", ["x", "x2"], "REAL"], ["GT", ["x", "x2"], "BOOL"], ["ADD", ["t", "t"], "TIME"], ["GT", ["t", "t2"], "BOOL"],
		];
		nodes.push({ kind: "input", id: "c10", x: 0, y: 0, expr: "INT#10" }, { kind: "input", id: "g", x: 0, y: 0, expr: "a > 0" },
			{ kind: "input", id: "x2", x: 0, y: 0, expr: "REAL#2.5" }, { kind: "input", id: "t2", x: 0, y: 0, expr: "T#250ms" });
		const decl: string[] = ["    a : INT;", "    b : INT;", "    x : REAL;", "    t : TIME;", "    f : BOOL;"];
		ops.forEach(([type, ins, out], i) => {
			const box = `op${i}`;
			nodes.push({ kind: "box", id: box, x: 100, y: i * 50, type }, { kind: "output", id: `o${i}`, x: 300, y: i * 50, target: `r${i}` });
			boxDef(type)!.inputs.forEach((pin, k) => wires.push({ from: { node: ins[k], pin: "OUT" }, to: { node: box, pin } }));
			wires.push({ from: { node: box, pin: "OUT" }, to: { node: `o${i}`, pin: "IN" } });
			decl.push(`    r${i} : ${out};`);
		});
		const net: FbdNetwork = { id: "n", title: "", nodes, wires };
		const src = fbdSource({ name: "P", decl: `PROGRAM P\nVAR\n${decl.join("\n")}\nEND_VAR`, networks: [net] });
		expect(src.problems).toEqual([]);
		const r = compile(deviceSources("P", src.text), [{ name: "MainTask", intervalCycles: 1, programs: ["P"] }], { io: ioBindings(), ioMapHash: ioMapHash() });
		if (!r.ok) throw new Error(r.diagnostics.map(formatDiagnostic).join("\n"));
		const data = new Uint8Array(32 * 1024);
		const vm = new Vm(data);
		expect(vm.load(r.image!)).toBe("none");
		const sym = (p: string) => r.debug!.resolve(`P.${p}`)!;
		const show = (p: string) => formatValue(sym(p).type, readValue(data, sym(p)));
		// the editor's lookup: a variable's online value; constants and expressions are not variables
		const lookup = (e: string) => (/^[a-z]\w*$/i.test(e) && r.debug!.resolve(`P.${e}`) ? show(e) : null);
		const valueOf = (id: string): string | null => {
			const n = nodes.find((x) => x.id === id)!;
			if (n.kind === "input") return lookup(n.expr) ?? (parseLive(n.expr) ? n.expr : null);
			return null;
		};
		let time = 0;
		const cases: Record<string, Value>[] = [
			{ a: 7, b: 2, x: 1.5, t: 300, f: true }, { a: -7, b: 3, x: -0.5, t: 0, f: false }, { a: 0, b: 0, x: 2.5, t: 250, f: true },
			{ a: 12, b: -5, x: 100.25, t: 1000, f: false },
		];
		let compared = 0;
		for (const c of cases) {
			for (const [k, v] of Object.entries(c)) writeValue(data, sym(k), v);
			const st = vm.runCycle(0, (time += 50)).state;
			const divByZero = c.b === 0;
			if (divByZero) {
				expect(st).not.toBe("done");  // the VM traps on a / 0: no comparison for this case
				expect(evalBox(boxDef("DIV")!, [show("a"), show("b")])).toBeNull();
				vm.load(r.image!);
				continue;
			}
			expect(st).toBe("done");
			ops.forEach(([type, ins], i) => {
				// g (a > 0) is an expression: the editor has no value for it - feed the VM's view of it
				const inputs = ins.map((id) => (id === "g" ? (Number(c.a) > 0 ? "TRUE" : "FALSE") : valueOf(id)));
				const ours = evalBox(boxDef(type)!, inputs);
				expect(`${type}(${inputs.join(", ")}) = ${ours}`).toBe(`${type}(${inputs.join(", ")}) = ${show(`r${i}`)}`);
				compared++;
			});
		}
		expect(compared).toBe(ops.length * 3);
	});
});
