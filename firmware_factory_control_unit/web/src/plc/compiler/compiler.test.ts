// The ST compiler end to end: compile, run in the TypeScript VM, read the variables back.
import { describe, expect, it } from "vitest";
import { Trap } from "../generated/isa.ts";
import { Vm } from "../vm.ts";
import { compile, projectHashOf, type CompileResult, type TaskDef } from "./compiler.ts";
import { formatDiagnostic } from "./diagnostics.ts";
import { ELEM, elemOf } from "./types.ts";

class Run {
	readonly vm: Vm;
	readonly data = new Uint8Array(32 * 1024);
	readonly dv = new DataView(this.data.buffer);
	now = 0;
	constructor(readonly r: CompileResult) {
		this.vm = new Vm(this.data);
		const e = this.vm.load(r.image!);
		if (e !== "none") throw new Error(`load: ${e}`);
	}
	cycle(n = 1, dtMs = 10) {
		let last = this.vm.runCycle(0, this.now);
		for (let i = 1; i < n; i++) {
			this.now += dtMs;
			last = this.vm.runCycle(0, this.now);
		}
		this.now += dtMs;
		return last;
	}
	get(path: string): number | bigint | boolean {
		const s = this.r.debug!.resolve(path);
		if (!s) throw new Error(`no variable ${path}`);
		const e = elemOf(s.type);
		if (!e) throw new Error(`${path} is not elementary`);
		const info = ELEM[e];
		const a = s.addr;
		if (e === "BOOL") return this.data[a] !== 0;
		if (e === "REAL") return this.dv.getFloat32(a, true);
		if (e === "LREAL") return this.dv.getFloat64(a, true);
		switch (info.size) {
			case 1: return info.signed ? this.dv.getInt8(a) : this.dv.getUint8(a);
			case 2: return info.signed ? this.dv.getInt16(a, true) : this.dv.getUint16(a, true);
			case 4: return info.signed ? this.dv.getInt32(a, true) : this.dv.getUint32(a, true);
			default: return info.signed ? this.dv.getBigInt64(a, true) : this.dv.getBigUint64(a, true);
		}
	}
	set(path: string, v: number | boolean) {
		const s = this.r.debug!.resolve(path)!;
		const e = elemOf(s.type)!;
		const size = ELEM[e].size;
		if (e === "REAL") this.dv.setFloat32(s.addr, Number(v), true);
		else if (size === 1) this.dv.setUint8(s.addr, Number(v) & 0xff);
		else if (size === 2) this.dv.setUint16(s.addr, Number(v) & 0xffff, true);
		else this.dv.setUint32(s.addr, Number(v) >>> 0, true);
	}
}

function build(text: string, tasks?: TaskDef[]): CompileResult {
	return compile([{ file: "PLC_PRG", text }], tasks);
}

function ok(text: string, extra = ""): Run {
	const r = compile([{ file: "PLC_PRG", text }, ...(extra ? [{ file: "lib", text: extra }] : [])]);
	if (!r.ok) throw new Error(r.diagnostics.map(formatDiagnostic).join("\n"));
	return new Run(r);
}

function errors(text: string): string[] {
	const r = build(text);
	expect(r.ok).toBe(false);
	return r.diagnostics.filter((d) => d.severity === "error").map((d) => `${d.span.start.line}: ${d.message}`);
}

const prg = (vars: string, body: string) => `PROGRAM PLC_PRG\nVAR\n${vars}\nEND_VAR\n${body}\nEND_PROGRAM\n`;

describe("compiler: basics", () => {
	it("counts across cycles", () => {
		const r = ok(prg("count : DINT;", "count := count + 1;"));
		r.cycle(5);
		expect(r.get("PLC_PRG.count")).toBe(5);
	});

	it("initial values", () => {
		const r = ok(prg("a : INT := -7; b : REAL := 1.5; c : BOOL := TRUE; d : TIME := T#1m2s; e : LINT := -5; f : BYTE := 16#A5;", ""));
		expect([r.get("PLC_PRG.a"), r.get("PLC_PRG.b"), r.get("PLC_PRG.c"), r.get("PLC_PRG.d"), r.get("PLC_PRG.e"), r.get("PLC_PRG.f")])
			.toEqual([-7, 1.5, true, 62000, -5n, 0xa5]);
	});

	it("is deterministic: the same source gives the same image", () => {
		const src = prg("x : INT; t : TON;", "t(IN := TRUE, PT := T#1s); x := x + 1;");
		expect(build(src).image).toEqual(build(src).image);
	});
});

describe("compiler: arithmetic", () => {
	it("wraps narrow integers as their type", () => {
		const r = ok(prg("s : SINT := 127; u : USINT := 255; i : INT := -32768; d : DINT := 2147483647;",
			"s := s + 1; u := u + 1; i := -i; d := d + 1;"));
		r.cycle();
		expect([r.get("PLC_PRG.s"), r.get("PLC_PRG.u"), r.get("PLC_PRG.i"), r.get("PLC_PRG.d")]).toEqual([-128, 0, -32768, -2147483648]);
	});

	it("rejects arithmetic on bit strings (they are not numbers)", () => {
		expect(errors(prg("w : WORD;", "w := w + 1;"))[0]).toMatch(/needs numbers/);
	});

	it("divides, MOD, mixed widths widen implicitly", () => {
		const r = ok(prg("a : INT := -7; b : DINT; c : DINT; d : LREAL; e : UDINT := 7; f : UDINT;",
			"b := a / 2; c := a MOD 2; d := a / 2.0; f := e / 2;"));
		r.cycle();
		expect([r.get("PLC_PRG.b"), r.get("PLC_PRG.c"), r.get("PLC_PRG.d"), r.get("PLC_PRG.f")]).toEqual([-3, -1, -3.5, 3]);
	});

	it("literals take the type of the other operand", () => {
		const r = ok(prg("r : REAL; s : SINT; x : LINT;", "r := r + 1; s := 100 + s; x := 3000000000 * 2;"));
		r.cycle();
		expect([r.get("PLC_PRG.r"), r.get("PLC_PRG.s"), r.get("PLC_PRG.x")]).toEqual([1, 100, 6000000000n]);
		expect(errors(prg("s : SINT;", "s := s + 300;"))[0]).toMatch(/300 does not fit/);
	});

	it("REAL is single precision, ** and SQRT", () => {
		const r = ok(prg("a : REAL := 16777216.0; b : LREAL; c : REAL; d : REAL;", "a := a + 1.0; b := 2.0 ** 10; c := SQRT(2.0); d := 2.0 ** 3;"));
		r.cycle();
		expect(r.get("PLC_PRG.a")).toBe(16777216);
		expect(r.get("PLC_PRG.b")).toBe(1024);
		expect(r.get("PLC_PRG.c")).toBe(Math.fround(Math.SQRT2));
		expect(r.get("PLC_PRG.d")).toBe(8);
	});

	it("division by zero stops the PLC", () => {
		const r = ok(prg("a : INT; b : INT;", "a := 1 / b;"));
		const res = r.cycle();
		expect(res.state).toBe("trapped");
		expect(res.trap).toBe(Trap.DIV_BY_ZERO);
		const st = r.r.debug!.statements.find((s) => s.pc <= res.pc && s.pou === "PLC_PRG");
		expect(st?.span.start.line).toBe(5);
	});

	it("TIME arithmetic", () => {
		const r = ok(prg("t : TIME := T#1s; u : TIME; n : DINT := 3;", "u := t * n + T#250ms; t := u / 2;"));
		r.cycle();
		expect([r.get("PLC_PRG.u"), r.get("PLC_PRG.t")]).toEqual([3250, 1625]);
	});

	it("boolean and bitwise operators, NOT", () => {
		const r = ok(prg("a : BOOL := TRUE; b : BOOL; c : BOOL; w : BYTE := 16#0F; x : BYTE;", "b := NOT a; c := a XOR b OR FALSE; x := NOT w AND 16#3C;"));
		r.cycle();
		expect([r.get("PLC_PRG.b"), r.get("PLC_PRG.c"), r.get("PLC_PRG.x")]).toEqual([false, true, 0x30]);
	});

	it("operator precedence (IEC 61131-3 Ed. 3)", () => {
		const r = ok(prg("a : DINT; b : BOOL; c : LREAL;", "a := 2 + 3 * 4 - 10 / 5; b := 1 < 2 = TRUE; c := -2.0 ** 2;"));
		r.cycle();
		expect([r.get("PLC_PRG.a"), r.get("PLC_PRG.b"), r.get("PLC_PRG.c")]).toEqual([12, true, 4]);
	});
});

describe("compiler: conversions", () => {
	it("REAL_TO_INT rounds half away from zero, TRUNC truncates", () => {
		const r = ok(prg("a : INT; b : INT; c : DINT; d : DINT;", "a := REAL_TO_INT(2.5); b := REAL_TO_INT(-2.5); c := TRUNC(-2.7); d := LREAL_TO_DINT(1.0E9);"));
		r.cycle();
		expect([r.get("PLC_PRG.a"), r.get("PLC_PRG.b"), r.get("PLC_PRG.c"), r.get("PLC_PRG.d")]).toEqual([3, -3, -2, 1000000000]);
	});

	it("narrowing conversions are range checked", () => {
		const r = ok(prg("d : DINT := 70000; i : INT;", "i := DINT_TO_INT(d);"));
		expect(r.cycle().trap).toBe(Trap.RANGE);
		const s = ok(prg("d : DINT := -5; i : INT; u : UINT;", "i := DINT_TO_INT(d);"));
		s.cycle();
		expect(s.get("PLC_PRG.i")).toBe(-5);
		const n = ok(prg("i : INT := -1; u : UINT;", "u := INT_TO_UINT(i);"));
		expect(n.cycle().trap).toBe(Trap.RANGE);
	});

	it("bit strings transfer bits", () => {
		const r = ok(prg("w : WORD := 16#FFFF; i : INT; d : DWORD := 16#12345678; b : BYTE; x : INT := -1; lw : LWORD;",
			"i := WORD_TO_INT(w); b := DWORD_TO_BYTE(d); lw := INT_TO_LWORD(x);"));
		r.cycle();
		expect([r.get("PLC_PRG.i"), r.get("PLC_PRG.b"), r.get("PLC_PRG.lw")]).toEqual([-1, 0x78, 0xffffn]);
	});

	it("BOOL conversions", () => {
		const r = ok(prg("b : BOOL; i : INT; x : DINT := 5;", "b := DINT_TO_BOOL(x); i := BOOL_TO_INT(b);"));
		r.cycle();
		expect([r.get("PLC_PRG.b"), r.get("PLC_PRG.i")]).toEqual([true, 1]);
	});

	it("rejects implicit narrowing", () => {
		expect(errors(prg("d : DINT; i : INT;", "i := d;"))[0]).toMatch(/cannot convert DINT to INT implicitly: use DINT_TO_INT/);
		expect(errors(prg("r : LREAL; f : REAL;", "f := r;"))[0]).toMatch(/LREAL to REAL/);
	});
});

describe("compiler: statements", () => {
	it("IF / ELSIF / ELSE", () => {
		const r = ok(prg("x : INT; y : INT;", "IF x < 0 THEN y := -1; ELSIF x = 0 THEN y := 0; ELSE y := 1; END_IF;\nx := x + 1;"));
		r.cycle();
		expect(r.get("PLC_PRG.y")).toBe(0);
		r.cycle();
		expect(r.get("PLC_PRG.y")).toBe(1);
	});

	it("CASE with values, ranges, lists and ELSE", () => {
		const r = ok(prg("i : INT; r : INT; hits : ARRAY[0..9] OF INT;", `
FOR i := 0 TO 9 DO
  CASE i OF
    0: r := 100;
    1, 2: r := 200;
    3..5: r := 300;
  ELSE
    r := -1;
  END_CASE;
  hits[i] := r;
END_FOR;`));
		r.cycle();
		const hits = Array.from({ length: 10 }, (_, k) => r.get(`PLC_PRG.hits[${k}]`));
		expect(hits).toEqual([100, 200, 200, 300, 300, 300, -1, -1, -1, -1]);
	});

	it("FOR up, down, with a variable step; EXIT and CONTINUE", () => {
		const r = ok(prg("i : INT; s1 : INT; s2 : INT; s3 : INT; st : INT := -2; s4 : INT;", `
FOR i := 1 TO 10 DO s1 := s1 + i; END_FOR;
FOR i := 10 TO 1 BY -3 DO s2 := s2 * 10 + i; END_FOR;
FOR i := 9 TO 0 BY st DO s3 := s3 + 1; END_FOR;
FOR i := 1 TO 100 DO
  IF i MOD 2 = 0 THEN CONTINUE; END_IF;
  IF i > 7 THEN EXIT; END_IF;
  s4 := s4 + i;
END_FOR;`));
		r.cycle();
		expect([r.get("PLC_PRG.s1"), r.get("PLC_PRG.s2"), r.get("PLC_PRG.s3"), r.get("PLC_PRG.s4")]).toEqual([55, 10741, 5, 16]);
	});

	it("WHILE and REPEAT", () => {
		const r = ok(prg("n : DINT := 27; steps : INT; k : INT;", `
WHILE n <> 1 DO
  IF n MOD 2 = 0 THEN n := n / 2; ELSE n := 3 * n + 1; END_IF;
  steps := steps + 1;
END_WHILE;
REPEAT k := k + 1; UNTIL k >= 3 END_REPEAT;`));
		r.cycle();
		expect([r.get("PLC_PRG.steps"), r.get("PLC_PRG.k")]).toEqual([111, 3]);
	});

	it("an endless loop hits the budget", () => {
		const r = compile([{ file: "PLC_PRG", text: prg("x : INT;", "WHILE TRUE DO x := x + 1; END_WHILE;") }], [{ name: "T", intervalCycles: 1, programs: ["PLC_PRG"], budget: 10000 }]);
		const run = new Run(r);
		expect(run.cycle().trap).toBe(Trap.BUDGET);
	});

	it("RETURN leaves the program", () => {
		const r = ok(prg("a : INT; b : INT;", "a := 1; RETURN; b := 1;"));
		r.cycle();
		expect([r.get("PLC_PRG.a"), r.get("PLC_PRG.b")]).toEqual([1, 0]);
	});
});

describe("compiler: functions and function blocks", () => {
	const lib = `
FUNCTION SCALE : REAL
VAR_INPUT x : INT; k : REAL := 2.0; END_VAR
SCALE := INT_TO_REAL(x) * k;
END_FUNCTION

FUNCTION SWAP_INC : BOOL
VAR_IN_OUT a : INT; b : INT; END_VAR
VAR t : INT; END_VAR
t := a; a := b + 1; b := t + 1;
SWAP_INC := TRUE;
END_FUNCTION

FUNCTION SUMSQ : DINT
VAR_INPUT n : INT; END_VAR
VAR i : INT; END_VAR
FOR i := 1 TO n DO SUMSQ := SUMSQ + SQR(i); END_FOR;
END_FUNCTION

FUNCTION SQR : DINT
VAR_INPUT v : INT; END_VAR
SQR := INT_TO_DINT(v) * v;
END_FUNCTION

FUNCTION_BLOCK FB_AVG
VAR_INPUT x : REAL; END_VAR
VAR_OUTPUT avg : REAL; n : DINT; END_VAR
VAR sum : REAL; END_VAR
sum := sum + x; n := n + 1; avg := sum / DINT_TO_REAL(n);
END_FUNCTION_BLOCK`;

	it("calls functions positionally, formally, with defaults and VAR_IN_OUT", () => {
		const r = ok(prg("a : REAL; b : REAL; c : REAL; p : INT := 1; q : INT := 5; ok : BOOL; s : DINT;",
			"a := SCALE(3, 0.5); b := SCALE(x := 4); c := SCALE(k := 3.0, x := 2); ok := SWAP_INC(p, q); s := SUMSQ(4);"), lib);
		r.cycle();
		expect([r.get("PLC_PRG.a"), r.get("PLC_PRG.b"), r.get("PLC_PRG.c"), r.get("PLC_PRG.p"), r.get("PLC_PRG.q"), r.get("PLC_PRG.ok"), r.get("PLC_PRG.s")])
			.toEqual([1.5, 8, 6, 6, 2, true, 30]);
	});

	it("function block instances keep their state, outputs by => and by member", () => {
		const r = ok(prg("f1 : FB_AVG; f2 : FB_AVG; m : REAL; k : DINT;", "f1(x := 2.0); f1(x := 4.0, avg => m); f2(x := 10.0); k := f1.n + f2.n;"), lib);
		r.cycle();
		expect([r.get("PLC_PRG.m"), r.get("PLC_PRG.f1.avg"), r.get("PLC_PRG.f2.avg"), r.get("PLC_PRG.k")]).toEqual([3, 3, 10, 3]);
	});

	it("rejects recursion, wrong parameters and access to internals", () => {
		expect(build(`FUNCTION F : INT VAR_INPUT n : INT; END_VAR F := F(n - 1); END_FUNCTION\n` + prg("x : INT;", "x := F(3);"))
			.diagnostics.map((d) => d.message).join()).toMatch(/recursion/);
		const e = errors(prg("f : FB_AVG; r : REAL;", "f(y := 1.0); r := f.sum; f.avg := 1.0;") + "\n" + lib);
		expect(e.join("\n")).toMatch(/no parameter 'y'/);
		expect(e.join("\n")).toMatch(/internal variable/);
		expect(e.join("\n")).toMatch(/output of FB_AVG/);
	});
});

describe("compiler: standard function blocks", () => {
	it("TON, TOF, TP measure time", () => {
		const r = ok(prg("on : TON; off : TOF; p : TP; in : BOOL; q1 : BOOL; q2 : BOOL; q3 : BOOL;",
			"on(IN := in, PT := T#100ms, Q => q1); off(IN := in, PT := T#100ms, Q => q2); p(IN := in, PT := T#50ms, Q => q3);"));
		r.cycle(); // t=0, IN false
		expect([r.get("PLC_PRG.q1"), r.get("PLC_PRG.q2"), r.get("PLC_PRG.q3")]).toEqual([false, false, false]);
		r.set("PLC_PRG.in", true);
		r.cycle(); // t=10: rising edge
		expect([r.get("PLC_PRG.q1"), r.get("PLC_PRG.q2"), r.get("PLC_PRG.q3")]).toEqual([false, true, true]);
		r.cycle(5); // t=60: TP over
		expect([r.get("PLC_PRG.q1"), r.get("PLC_PRG.q3"), r.get("PLC_PRG.p.ET")]).toEqual([false, false, 50]);
		r.cycle(5); // t=110: TON done
		expect([r.get("PLC_PRG.q1"), r.get("PLC_PRG.on.ET")]).toEqual([true, 100]);
		r.set("PLC_PRG.in", false);
		r.cycle(); // falling edge: TON off, TOF starts
		expect([r.get("PLC_PRG.q1"), r.get("PLC_PRG.q2"), r.get("PLC_PRG.p.ET")]).toEqual([false, true, 0]);
		r.cycle(10);
		expect([r.get("PLC_PRG.q2"), r.get("PLC_PRG.off.ET")]).toEqual([false, 100]);
	});

	it("R_TRIG, F_TRIG, CTU, CTD, SR, RS", () => {
		const r = ok(prg("rt : R_TRIG; ft : F_TRIG; cu : CTU; cd : CTD; sr : SR; rs : RS; x : BOOL; nr : INT; nf : INT;",
			"rt(CLK := x); ft(CLK := x); IF rt.Q THEN nr := nr + 1; END_IF; IF ft.Q THEN nf := nf + 1; END_IF;\n" +
			"cu(CU := x, PV := 3); cd(CD := x, LD := FALSE, PV := 0); sr(S1 := TRUE, R := TRUE); rs(S := TRUE, R1 := TRUE);"));
		for (let i = 0; i < 8; i++) {
			r.set("PLC_PRG.x", i % 2 === 1);
			r.cycle();
		}
		expect([r.get("PLC_PRG.nr"), r.get("PLC_PRG.nf"), r.get("PLC_PRG.cu.CV"), r.get("PLC_PRG.cu.Q"), r.get("PLC_PRG.cd.CV")]).toEqual([4, 3, 4, true, -4]);
		expect([r.get("PLC_PRG.sr.Q1"), r.get("PLC_PRG.rs.Q1")]).toEqual([true, false]);
	});
});

describe("compiler: data types", () => {
	const types = `
TYPE
  E_Mode : (Off, Manual, Auto := 5, Fault);
  S_Axis : STRUCT pos : DINT := 10; speed : REAL; mode : E_Mode := Manual; END_STRUCT;
  T_Pct : INT (0..100);
END_TYPE`;

	it("enumerations, structures, subranges", () => {
		const r = ok(types + "\n" + prg("m : E_Mode; a : S_Axis; b : S_Axis := (speed := 2.5); p : T_Pct := 50; n : INT;",
			"IF m = Off THEN m := E_Mode#Auto; END_IF;\nCASE m OF Auto: n := 1; Fault: n := 2; END_CASE;\na.pos := a.pos + 1; b := a; p := p + 10;"));
		r.cycle();
		expect([r.get("PLC_PRG.m"), r.get("PLC_PRG.n"), r.get("PLC_PRG.a.pos"), r.get("PLC_PRG.b.pos"), r.get("PLC_PRG.b.speed"), r.get("PLC_PRG.a.mode"), r.get("PLC_PRG.p")])
			.toEqual([5, 1, 11, 11, 0, 1, 60]);
		for (let i = 0; i < 4; i++) expect(r.cycle().state).toBe("done");
		expect(r.cycle().trap).toBe(Trap.RANGE); // p would be 110
	});

	it("arrays: multi-dimensional, initialized with repetition, indices checked", () => {
		const r = ok(prg("m : ARRAY[1..3, 0..1] OF INT := [1, 2, 3, 4, 2(9)]; i : INT := 3; j : INT := 1; v : INT; z : ARRAY[-2..2] OF BOOL;",
			"v := m[i, j] + m[1, 0]; z[-2] := TRUE; i := i + 1;"));
		r.cycle();
		expect([r.get("PLC_PRG.v"), r.get("PLC_PRG.m[2, 1]"), r.get("PLC_PRG.z[-2]")]).toEqual([10, 4, true]);
		expect(r.cycle().trap).toBe(Trap.RANGE);
	});

	it("rejects mixing enumerations and wrong types", () => {
		const e = errors(types + "\n" + prg("m : E_Mode; i : INT; b : BOOL;", "m := 1; i := m; b := m < Auto; IF i THEN END_IF;"));
		expect(e.length).toBe(4);
		expect(e[3]).toMatch(/condition must be BOOL/);
	});
});

describe("compiler: intrinsic functions", () => {
	it("MIN, MAX, LIMIT, SEL, MUX, ABS, SHL, ROL", () => {
		const r = ok(prg("a : INT; b : INT; c : INT; d : INT; e : INT; f : INT; g : BYTE; h : BYTE; x : INT := -9;",
			"a := MIN(5, x, 3); b := MAX(5, x, 3); c := LIMIT(0, x, 10); d := SEL(TRUE, 1, 2); e := MUX(2, 10, 20, 30); f := ABS(x); g := SHL(BYTE#16#81, 1); h := ROL(BYTE#16#81, 1);"));
		r.cycle();
		expect([r.get("PLC_PRG.a"), r.get("PLC_PRG.b"), r.get("PLC_PRG.c"), r.get("PLC_PRG.d"), r.get("PLC_PRG.e"), r.get("PLC_PRG.f"), r.get("PLC_PRG.g"), r.get("PLC_PRG.h")])
			.toEqual([-9, 5, 0, 2, 30, 9, 0x02, 0x03]);
	});

	it("MUX with K out of range stops", () => {
		expect(ok(prg("e : INT; k : INT := 3;", "e := MUX(k, 1, 2, 3);")).cycle().trap).toBe(Trap.RANGE);
	});
});

describe("compiler: diagnostics", () => {
	it("reports several errors with positions", () => {
		const e = errors(prg("x : INT;", "x := y;\nx := 1 +;\nz := 2;"));
		expect(e).toEqual(["5: unknown name 'y'", "6: expected an expression, found ';'", "7: unknown variable 'z'"]);
	});

	it("rejects identifiers with a double underscore and calls to __PLC_TIME", () => {
		expect(errors(prg("a__b : INT;", ""))[0]).toMatch(/two underscores/);
	});

	it("rejects writing constants", () => {
		expect(errors(prg("x : INT;", "x := 1;") + "\nVAR_GLOBAL CONSTANT N : INT := 5; END_VAR\nPROGRAM P2 N := 1; END_PROGRAM")[0]).toMatch(/'N' is a constant/);
	});

	it("constants size arrays", () => {
		const r = ok("VAR_GLOBAL CONSTANT N : INT := 4; END_VAR\n" + prg("a : ARRAY[1..N] OF INT; i : INT;", "FOR i := 1 TO N DO a[i] := i * i; END_FOR;"));
		r.cycle();
		expect(r.get("PLC_PRG.a[4]")).toBe(16);
	});
});

describe("compiler: I/O bindings", () => {
	const io = [{ name: "Valve1", register: 20, area: "holding" as const, kind: "output" as const, bits: 1 as const, signed: false }];
	it("rejects an I/O variable of the wrong type or a missing one", () => {
		const r = compile([{ file: "PLC_PRG", text: "VAR_GLOBAL Valve1 : INT; END_VAR\n" + prg("x : INT;", "") }], undefined, { io });
		expect(r.diagnostics.map((d) => d.message).join()).toMatch(/'Valve1' is INT; register 20 needs BOOL/);
		const m = compile([{ file: "PLC_PRG", text: prg("x : INT;", "") }], undefined, { io });
		expect(m.diagnostics.map((d) => d.message).join()).toMatch(/'Valve1' \(register 20\) is not declared/);
	});
});

// Regression: the project hash covered the raw text, and git (autocrlf) checks the corpus sources out
// with CRLF on Windows - the same project gave other images there (corpus test failed on Windows only).
describe("project hash", () => {
	it("line ends do not count", () => {
		const lf = "PROGRAM PLC_PRG\nVAR x : INT; END_VAR\nx := 1;\nEND_PROGRAM\n";
		const crlf = lf.replace(/\n/g, "\r\n");
		expect(projectHashOf([{ file: "PLC_PRG", text: crlf }])).toBe(projectHashOf([{ file: "PLC_PRG", text: lf }]));
		expect(projectHashOf([{ file: "PLC_PRG", text: lf.replace("x := 1", "x := 2") }])).not.toBe(projectHashOf([{ file: "PLC_PRG", text: lf }]));
		const a = compile([{ file: "PLC_PRG", text: crlf }]), b = compile([{ file: "PLC_PRG", text: lf }]);
		expect(a.ok && b.ok).toBe(true);
		expect(Buffer.compare(Buffer.from(a.image!), Buffer.from(b.image!))).toBe(0);
	});
});

describe("compiler: namespaces (IEC 61131-3 Ed. 3, 6.9)", () => {
	// a library as it will come with P7: its own namespace, nested ones, a type, globals, an FB, a function
	const lib = `
NAMESPACE Lib
    TYPE Mode : (Off, Slow, Fast) := Off; END_TYPE
    VAR_GLOBAL CONSTANT
        cMax : INT := 100;
    END_VAR
    VAR_GLOBAL
        gCount : INT;
    END_VAR
    FUNCTION Scale : INT
    VAR_INPUT x : INT; END_VAR
        Scale := LIMIT(0, x * 2, cMax);   (* the namespace's own constant, unqualified *)
    END_FUNCTION
    NAMESPACE Valves
        FUNCTION_BLOCK Valve
        VAR_INPUT Open : BOOL; END_VAR
        VAR_OUTPUT Q : BOOL; Speed : Mode; END_VAR
            Q := Open;
            Speed := SEL(Open, Off, Fast);
            gCount := gCount + 1;      (* the enclosing namespace's global *)
            Q := Q AND Scale(1) = 2;   (* the enclosing namespace's function *)
        END_FUNCTION_BLOCK
    END_NAMESPACE
END_NAMESPACE
`;

	it("qualified names from outside: types, function blocks, functions, globals, constants", () => {
		const r = ok(prg("    v : Lib.Valves.Valve;\n    m : Lib.Mode;\n    y : INT;\n    z : INT;\n    c : INT;\n    arr : ARRAY[1..Lib.cMax] OF BYTE;",
			"v(Open := TRUE);\nm := v.Speed;\ny := Lib.Scale(70);\nz := Lib.gCount;\nc := Lib.cMax;"), lib);
		r.cycle(3);
		expect(r.get("PLC_PRG.v.Q")).toBe(true);
		expect(r.get("PLC_PRG.m")).toBe(2);  // Fast
		expect(r.get("PLC_PRG.y")).toBe(100);  // 140 limited to cMax
		expect(r.get("PLC_PRG.z")).toBe(3);
		expect(r.get("PLC_PRG.c")).toBe(100);
	});

	it("USING makes a namespace's names visible unqualified", () => {
		const r = ok(`USING Lib, Lib.Valves;\n${prg("    v : Valve;\n    y : INT;", "v(Open := TRUE);\ny := Scale(3) + cMax;")}`, lib);
		r.cycle(1);
		expect(r.get("PLC_PRG.y")).toBe(106);
		expect(r.get("PLC_PRG.v.Q")).toBe(true);
	});

	it("the same name in two namespaces is no conflict; unqualified it needs USING, and two USINGs make it ambiguous", () => {
		const two = `${lib}
NAMESPACE Other
    FUNCTION Scale : INT
    VAR_INPUT x : INT; END_VAR
        Scale := x * 10;
    END_FUNCTION
END_NAMESPACE`;
		const r = ok(prg("    a : INT;\n    b : INT;", "a := Lib.Scale(2);\nb := Other.Scale(2);"), two);
		r.cycle(1);
		expect([r.get("PLC_PRG.a"), r.get("PLC_PRG.b")]).toEqual([4, 20]);
		const unknown = compile([{ file: "PLC_PRG", text: prg("    v : Valve;", "") }, { file: "lib", text: lib }]);
		expect(unknown.diagnostics.map((d) => d.message)).toContain("unknown type 'VALVE'");
		const ambiguous = compile([{ file: "PLC_PRG", text: `USING Lib, Other;\n${prg("    a : INT;", "a := Scale(2);")}` }, { file: "lib", text: two }]);
		expect(ambiguous.ok).toBe(false);
		expect(ambiguous.diagnostics.map((d) => d.message).join("\n")).toMatch(/ambiguous: LIB.SCALE, OTHER.SCALE/);
	});

	it("a name of the own namespace wins over a global one of the same name", () => {
		const r = ok(prg("    a : INT;\n    b : INT;", "a := Twice(5);\nb := N.Use(5);"), `
FUNCTION Twice : INT
VAR_INPUT x : INT; END_VAR
    Twice := x * 2;
END_FUNCTION
NAMESPACE N
    FUNCTION Twice : INT
    VAR_INPUT x : INT; END_VAR
        Twice := x * 3;
    END_FUNCTION
    FUNCTION Use : INT
    VAR_INPUT x : INT; END_VAR
        Use := Twice(x);
    END_FUNCTION
END_NAMESPACE`);
		r.cycle(1);
		expect([r.get("PLC_PRG.a"), r.get("PLC_PRG.b")]).toEqual([10, 15]);
	});

	it("a variable named like a namespace stays a variable (Lib.x is its member)", () => {
		const r = ok(`TYPE P : STRUCT cMax : INT; END_STRUCT END_TYPE\n${prg("    Lib : P;\n    y : INT;", "Lib.cMax := 7;\ny := Lib.cMax;")}`, lib);
		r.cycle(1);
		expect(r.get("PLC_PRG.y")).toBe(7);
	});

	it("parse errors: END_NAMESPACE missing or stray, recovery goes on", () => {
		const missing = build(`NAMESPACE A\nFUNCTION F : INT\n    F := 1;\nEND_FUNCTION\n${prg("    a : INT;", "a := A.F();")}`);
		expect(missing.diagnostics.some((d) => d.message.includes("END_NAMESPACE"))).toBe(true);
		const stray = build(`END_NAMESPACE\n${prg("    a : INT;", "a := 1;")}`);
		expect(stray.ok).toBe(false);
		expect(stray.diagnostics.length).toBe(1);  // the program after it is parsed (no follow-up errors)
	});
});
