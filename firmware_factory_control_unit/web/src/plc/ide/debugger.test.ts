// Breakpoints and single steps: the VM halts before an instruction and resumes there; the debugger maps
// lines to statement offsets and halts to frame chains (docs/plc-iec61131.md 10.3).
import { describe, expect, it } from "vitest";
import { compile, type CompileResult } from "../compiler/compiler.ts";
import { formatDiagnostic } from "../compiler/diagnostics.ts";
import { Vm } from "../vm.ts";
import { Debugger } from "./debugger.ts";
import { readValue } from "./values.ts";

const SOURCE = `FUNCTION_BLOCK Counter
VAR_INPUT inc : INT; END_VAR
VAR_OUTPUT value : INT; END_VAR
value := value + inc;
value := value + 0;
END_FUNCTION_BLOCK

PROGRAM PLC_PRG
VAR c : Counter; n : INT; m : INT; END_VAR
n := n + 1;
c(inc := 2);
m := c.value;
END_PROGRAM
`;
const FILE = "PLC_PRG";
const line = (text: string) => SOURCE.split("\n").findIndex((l) => l.startsWith(text)) + 1;
const IN_FB = line("value := value + inc"), N = line("n := n + 1"), CALL = line("c(inc := 2)"), M = line("m := c.value");

function setup() {
	const r: CompileResult = compile([{ file: FILE, text: SOURCE }]);
	if (!r.ok) throw new Error(r.diagnostics.map(formatDiagnostic).join("\n"));
	const data = new Uint8Array(32 * 1024);
	const vm = new Vm(data);
	expect(vm.load(r.image!)).toBe("none");
	const dbg = new Debugger(r.debug!);
	const get = (p: string) => readValue(data, r.debug!.resolve(p)!);
	/** Arms the VM with the debugger's predicate (after every change, as the simulator does). */
	const arm = () => {
		vm.stop = dbg.predicate();
	};
	const where = () => dbg.frames(vm.contexts[0].pc, vm.contexts[0].calls);
	return { vm, dbg, get, arm, where };
}

describe("PLC debugging", () => {
	it("knows the lines a breakpoint can go on", () => {
		const { dbg } = setup();
		const lines = dbg.breakableLines(FILE);
		for (const l of [IN_FB, N, CALL, M]) expect(lines.has(l)).toBe(true);
		expect(lines.has(line("VAR c : Counter"))).toBe(false);
		expect(dbg.setBreakpoints(FILE, [line("END_PROGRAM"), CALL])).toEqual([CALL]);  // no statement: no breakpoint
	});

	it("halts at a breakpoint before the statement, resumes there, the cycle ends as without it", () => {
		const { vm, dbg, get, arm, where } = setup();
		dbg.setBreakpoints(FILE, [CALL]);
		arm();
		const r = vm.runCycle(0, 0);
		expect(r.state).toBe("halted");
		expect(vm.halted(0)).toBe(true);
		expect(get("PLC_PRG.n")).toBe(1);  // the line before ran
		expect(get("PLC_PRG.m")).toBe(0);  // the call not yet
		expect(where()).toMatchObject([{ file: FILE, line: CALL, pou: "PLC_PRG" }]);
		expect(() => vm.runCycle(0, 0)).toThrow(/halted/);
		dbg.resume();
		arm();
		expect(vm.resume(0).state).toBe("done");  // the rest of the cycle
		expect(get("PLC_PRG.m")).toBe(2);
		expect(vm.runCycle(0, 50).state).toBe("halted");  // the breakpoint stays: the next cycle halts there again
		expect(where()[0].line).toBe(CALL);
		expect(get("PLC_PRG.n")).toBe(2);
	});

	it("resume runs the halted instruction without stopping on it again", () => {
		const { vm, dbg, get, arm } = setup();
		dbg.setBreakpoints(FILE, [CALL]);
		arm();
		expect(vm.runCycle(0, 0).state).toBe("halted");
		dbg.setBreakpoints(FILE, []);
		dbg.resume();
		arm();
		expect(vm.resume(0).state).toBe("done");
		expect(get("PLC_PRG.m")).toBe(2);
		expect(vm.halted(0)).toBe(false);
	});

	it("step into enters the FB, the call stack shows the caller", () => {
		const { vm, dbg, arm, where } = setup();
		dbg.setBreakpoints(FILE, [CALL]);
		arm();
		vm.runCycle(0, 0);
		dbg.stepFrom("into", vm.contexts[0].calls.length);
		arm();
		expect(vm.resume(0).state).toBe("halted");
		expect(where()).toMatchObject([{ line: IN_FB, pou: "COUNTER" }, { line: CALL, pou: "PLC_PRG" }]);
	});

	it("step over runs the call through, step out returns to the caller's next statement", () => {
		const { vm, dbg, get, arm, where } = setup();
		dbg.setBreakpoints(FILE, [CALL]);
		arm();
		vm.runCycle(0, 0);
		dbg.setBreakpoints(FILE, []);
		dbg.stepFrom("over", vm.contexts[0].calls.length);
		arm();
		expect(vm.resume(0).state).toBe("halted");
		expect(where()).toMatchObject([{ line: M }]);
		expect(get("PLC_PRG.c.value")).toBe(2);  // the FB ran

		const s = setup();
		s.dbg.setBreakpoints(FILE, [IN_FB]);
		s.arm();
		expect(s.vm.runCycle(0, 0).state).toBe("halted");
		expect(s.where()[0].line).toBe(IN_FB);
		s.dbg.setBreakpoints(FILE, []);
		s.dbg.stepFrom("out", s.vm.contexts[0].calls.length);
		s.arm();
		expect(s.vm.resume(0).state).toBe("halted");
		expect(s.where()).toMatchObject([{ line: M, pou: "PLC_PRG" }]);
		expect(s.vm.contexts[0].calls.length).toBe(1);
	});

	it("a step past the end of the cycle halts at the first statement of the next one", () => {
		const { vm, dbg, get, arm, where } = setup();
		dbg.setBreakpoints(FILE, [M]);
		arm();
		vm.runCycle(0, 0);
		dbg.setBreakpoints(FILE, []);
		dbg.stepFrom("over", vm.contexts[0].calls.length);
		arm();
		expect(vm.resume(0).state).toBe("done");  // nothing left in this cycle
		dbg.cycleEnded();
		arm();
		expect(vm.runCycle(0, 50).state).toBe("halted");
		expect(where()).toMatchObject([{ line: N }]);
		expect(get("PLC_PRG.n")).toBe(1);  // the second cycle's n := n + 1 not yet
	});

	it("the simulator: HALTED at a breakpoint, waits, steps, continues, STOP from HALTED", async () => {
		const { Simulator } = await import("./simulator.ts");
		const { build, exampleProject, sourceOf } = await import("./project.ts");
		const p = exampleProject();
		const text = sourceOf(p.objects[0]);
		const at = (s: string) => text.split("\n").findIndex((l) => l.startsWith(s)) + 1;
		const gate = at("AtGate(CLK"), ifLine = at("IF Enable AND");
		const sim = new Simulator();
		const cycle = () => (sim as unknown as { cycle(): void }).cycle();
		expect(sim.download(build(p).result)).toBe(true);
		expect(sim.setBreakpoints("PLC_PRG", [gate])).toEqual([gate]);
		sim.singleCycle();
		expect(sim.state).toBe("HALTED");
		expect(sim.halt).toMatchObject({ reason: "breakpoint", frames: [{ file: "PLC_PRG", line: gate, pou: "PLC_PRG" }] });
		const cycles = sim.cycles;
		cycle();
		cycle();  // the timer goes on: the program does not
		expect(sim.state).toBe("HALTED");
		expect(sim.cycles).toBe(cycles);
		sim.step("over");
		expect(sim.halt?.frames[0].line).toBe(ifLine);
		expect(sim.halt?.reason).toBe("step");
		sim.continueRun();
		expect(sim.state).toBe("STOP");  // a single cycle: back to STOP when it ends
		expect(sim.cycles).toBe(cycles + 1);
		sim.start();
		cycle();
		expect(sim.state).toBe("HALTED");
		sim.stop();
		expect(sim.state).toBe("STOP");
		expect(sim.halt).toBeNull();
		sim.setBreakpoints("PLC_PRG", []);
		sim.singleCycle();
		expect(sim.state).toBe("STOP");  // no breakpoint: the cycle runs through
	});

	it("without breakpoint or step the VM gets no predicate (full speed)", () => {
		const { dbg } = setup();
		expect(dbg.predicate()).toBeNull();
		dbg.setBreakpoints(FILE, [N]);
		expect(dbg.predicate()).not.toBeNull();
		dbg.setBreakpoints(FILE, []);
		expect(dbg.predicate()).toBeNull();
	});
});
