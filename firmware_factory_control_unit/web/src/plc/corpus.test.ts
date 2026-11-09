// Differential tests (rtoos/docs/plc.md "Differential tests"): every corpus program is compiled and
// run in the TypeScript VM; its image, run plan and trace (state and the whole data segment after
// every cycle) are the files rtoos/tests/plc_corpus_test.cpp replays in the C++ VM, which must give
// byte-identical traces. The corpus: the hand-written programs in corpus/*.st and 40 random programs
// from a seeded generator (every type, operator, conversion; traps included).
//
// This test fails when the files in rtoos/tests/plc_corpus are not what the current compiler and VM
// produce; `npm run plc:corpus` rewrites them.
import fs from "node:fs";
import path from "node:path";
import { describe, expect, it } from "vitest";
import { compile, type TaskDef } from "./compiler/compiler.ts";
import { formatDiagnostic } from "./compiler/diagnostics.ts";
import { ELEM, elemOf, type ElemName } from "./compiler/types.ts";
import { NATIVES } from "./generated/isa.ts";
import { Hdr, NATIVE_RECORD_SIZE } from "./image.ts";
import { Vm } from "./vm.ts";

const CORPUS_DIR = path.join(import.meta.dirname, "corpus");
const OUT_DIR = path.resolve(import.meta.dirname, "../../../rtoos/tests/plc_corpus");
const UPDATE = process.env.PLC_CORPUS_UPDATE === "1";
const DATA_CAPACITY = 32 * 1024;

interface Case {
	name: string;
	source: string;
	cycles: number;
	dt: number;
	budget?: number;
	programs: string[];
	sets: { cycle: number; path: string; value: bigint }[];
	expects: { path: string; value: string }[];
}

// ---- directives in the hand-written programs ------------------------------------------------------
function parseCase(name: string, source: string): Case {
	const c: Case = { name, source, cycles: 1, dt: 10, programs: ["PLC_PRG"], sets: [], expects: [] };
	for (const m of source.matchAll(/\(\*\s*@(\w+)\s+(.*?)\s*\*\)/g)) {
		const [, key, arg] = m;
		if (key === "cycles") c.cycles = Number(arg);
		else if (key === "dt") c.dt = Number(arg);
		else if (key === "budget") c.budget = Number(arg);
		else if (key === "programs") c.programs = arg.split(/\s+/);
		else if (key === "set") {
			const s = /^(\d+)\s+(\S+)\s*:=\s*(-?\d+)$/.exec(arg);
			if (!s) throw new Error(`${name}: bad @set ${arg}`);
			c.sets.push({ cycle: Number(s[1]), path: s[2], value: BigInt(s[3]) });
		} else if (key === "expect") {
			const s = /^(\S+)\s*=\s*(\S+)$/.exec(arg);
			if (!s) throw new Error(`${name}: bad @expect ${arg}`);
			c.expects.push({ path: s[1], value: s[2] });
		} else throw new Error(`${name}: unknown directive @${key}`);
	}
	return c;
}

// ---- the random program generator --------------------------------------------------------------
function rng(seed: number) {
	let a = seed >>> 0;
	const next = () => {
		a = (a + 0x6d2b79f5) >>> 0;
		let t = a;
		t = Math.imul(t ^ (t >>> 15), t | 1);
		t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
	return {
		next,
		int: (n: number) => Math.floor(next() * n),
		pick: <T>(xs: readonly T[]): T => xs[Math.floor(next() * xs.length)],
		chance: (p: number) => next() < p,
	};
}
type Rng = ReturnType<typeof rng>;

const INTS: ElemName[] = ["SINT", "INT", "DINT", "LINT", "USINT", "UINT", "UDINT", "ULINT"];
const BITS: ElemName[] = ["BYTE", "WORD", "DWORD", "LWORD"];
const REALS: ElemName[] = ["REAL", "LREAL"];
const ALL: ElemName[] = [...INTS, ...BITS, ...REALS, "BOOL", "TIME"];

function literal(r: Rng, t: ElemName): string {
	const info = ELEM[t];
	switch (info.category) {
		case "bool": return r.chance(0.5) ? "TRUE" : "FALSE";
		case "real": {
			const v = r.pick([0, 1, -1, 0.5, 0.1, 3.75, -1234.5, 1e10, -2.5e-7, r.next() * 1000 - 500]);
			return `${t}#${v.toExponential(6)}`;
		}
		case "time": return `T#${r.int(200000) - 100000}ms`;
		default: {
			const span = info.max - info.min + 1n;
			const special = [info.min, info.max, 0n, 1n, info.signed ? -1n : 2n];
			let v = r.chance(0.3) ? r.pick(special) : info.min + (BigInt(r.int(2 ** 30)) * BigInt(r.int(2 ** 30)) * BigInt(r.int(2 ** 4) + 1)) % span;
			if (v < info.min) v = info.min;
			return v < 0n ? `${t}#-${-v}` : `${t}#${v}`;
		}
	}
}

class Gen {
	vars = new Map<ElemName, string[]>();
	constructor(private readonly r: Rng) {}

	expr(t: ElemName, depth: number): string {
		const r = this.r;
		const vs = this.vars.get(t) ?? [];
		if (depth <= 0 || r.chance(0.25)) return vs.length > 0 && r.chance(0.7) ? r.pick(vs) : literal(r, t);
		const d = depth - 1;
		const info = ELEM[t];
		const conv = (): string => {
			// from another type, kept in range so that most conversions do not trap
			const cat = info.category;
			const from = r.pick(cat === "bit" ? [...INTS, ...BITS, "BOOL"] as ElemName[] : cat === "real" ? [...INTS, ...REALS, "TIME"] as ElemName[] :
				cat === "bool" ? [...INTS, ...BITS] as ElemName[] : cat === "time" ? ["DINT", "INT", "UINT", "LINT"] as ElemName[] : [...INTS, ...BITS, ...REALS, "BOOL"] as ElemName[]);
			if (from === t) return this.expr(t, d);
			const fc = ELEM[from].category;
			let src = this.expr(from, d);
			if ((cat === "int" || cat === "uint" || cat === "time") && (fc === "int" || fc === "uint") && r.chance(0.85)) src = `ABS(${src} MOD ${from}#100)`;
			if ((cat === "int" || cat === "uint" || cat === "time") && fc === "real") src = `ABS(LIMIT(${from}#-100.0, ${src}, ${from}#100.0))`;
			return `${from}_TO_${t}(${src})`;
		};
		switch (info.category) {
			case "int":
			case "uint": {
				const k = r.int(9);
				const a = () => this.expr(t, d);
				const divisor = () => (info.category === "int" ? `MAX(ABS(${a()}), ${t}#1)` : `MAX(${a()}, ${t}#1)`);
				switch (k) {
					case 0: return `(${a()} + ${a()})`;
					case 1: return `(${a()} - ${a()})`;
					case 2: return `(${a()} * ${a()})`;
					case 3: return `(${a()} / ${divisor()})`;
					case 4: return `(${a()} MOD ${divisor()})`;
					case 5: return info.category === "int" ? `(-${a()})` : `ABS(${a()})`;
					case 6: return `${r.pick(["MIN", "MAX"])}(${a()}, ${a()}${r.chance(0.3) ? `, ${a()}` : ""})`;
					case 7: return `SEL(${this.expr("BOOL", d)}, ${a()}, ${a()})`;
					default: return conv();
				}
			}
			case "bit": {
				const a = () => this.expr(t, d);
				switch (r.int(6)) {
					case 0: return `(${a()} ${r.pick(["AND", "OR", "XOR"])} ${a()})`;
					case 1: return `(NOT ${a()})`;
					case 2: return `${r.pick(["SHL", "SHR", "ROL", "ROR"])}(${a()}, ${r.int(70)})`;
					case 3: return `MUX(${r.int(3)}, ${a()}, ${a()}, ${a()})`;
					default: return conv();
				}
			}
			case "real": {
				const a = () => this.expr(t, d);
				switch (r.int(7)) {
					case 0: return `(${a()} + ${a()})`;
					case 1: return `(${a()} - ${a()})`;
					case 2: return `(${a()} * ${a()})`;
					case 3: return `(${a()} / MAX(ABS(${a()}), ${t}#1.0))`;
					case 4: return `${r.pick(["ABS", "SQRT"])}(${a()})`;
					case 5: return `(-${a()})`;
					default: return conv();
				}
			}
			case "bool": {
				switch (r.int(5)) {
					case 0: {
						const ct = r.pick(ALL.filter((x) => x !== "BOOL"));
						const ops = ELEM[ct].category === "real" || ELEM[ct].category === "int" || ELEM[ct].category === "uint" || ct === "TIME" || ELEM[ct].category === "bit"
							? ["<", ">", "<=", ">=", "=", "<>"] : ["=", "<>"];
						return `(${this.expr(ct, d)} ${r.pick(ops)} ${this.expr(ct, d)})`;
					}
					case 1: return `(${this.expr("BOOL", d)} ${r.pick(["AND", "OR", "XOR"])} ${this.expr("BOOL", d)})`;
					case 2: return `(NOT ${this.expr("BOOL", d)})`;
					default: return conv();
				}
			}
			case "time": {
				switch (r.int(4)) {
					case 0: return `(${this.expr("TIME", d)} + ${this.expr("TIME", d)})`;
					case 1: return `(${this.expr("TIME", d)} - ${this.expr("TIME", d)})`;
					case 2: return `(${this.expr("TIME", d)} * ${r.int(7) - 3})`;
					default: return conv();
				}
			}
		}
	}
}

function randomCase(seed: number): Case {
	const r = rng(seed * 7919 + 17);
	const g = new Gen(r);
	const decls: string[] = [];
	const n = 8 + r.int(8);
	for (let i = 0; i < n; i++) {
		const t = i < ALL.length && r.chance(0.6) ? ALL[(seed + i) % ALL.length] : r.pick(ALL);
		const name = `v${i}`;
		decls.push(`${name} : ${t} := ${literal(r, t)};`);
		g.vars.set(t, [...(g.vars.get(t) ?? []), name]);
	}
	const body: string[] = [];
	const names = [...g.vars.entries()].flatMap(([t, vs]) => vs.map((v) => ({ t, v })));
	for (let i = 0; i < 6 + r.int(6); i++) {
		const { t, v } = r.pick(names);
		const assign = `${v} := ${g.expr(t, 3)};`;
		if (r.chance(0.2)) {
			const o = r.pick(names);
			body.push(`IF ${g.expr("BOOL", 2)} THEN ${assign} ELSE ${o.v} := ${g.expr(o.t, 2)}; END_IF;`);
		} else body.push(assign);
	}
	const source = `PROGRAM PLC_PRG\nVAR\n${decls.map((d) => "  " + d).join("\n")}\nEND_VAR\n${body.join("\n")}\nEND_PROGRAM\n`;
	return { name: `random_${String(seed).padStart(2, "0")}`, source, cycles: 6, dt: 10, programs: ["PLC_PRG"], sets: [], expects: [] };
}

// ---- running a case ---------------------------------------------------------------------------------
interface Artifacts {
	image: Uint8Array;
	run: string;
	trace: Uint8Array;
	final: (path: string) => string;
}

function runCase(c: Case): Artifacts {
	const tasks: TaskDef[] = [{ name: "MainTask", intervalCycles: 1, programs: c.programs, budget: c.budget }];
	const r = compile([{ file: c.name, text: c.source }], tasks, { maxData: DATA_CAPACITY });
	if (!r.ok) throw new Error(`${c.name}:\n${r.diagnostics.map(formatDiagnostic).join("\n")}\n${c.source}`);
	const image = r.image!;
	const debug = r.debug!;
	const hv = new DataView(image.buffer, image.byteOffset);
	for (let i = 0, at = hv.getUint32(Hdr.nativeOffset, true); i < hv.getUint32(Hdr.nativeCount, true); i++, at += NATIVE_RECORD_SIZE) {
		const n = NATIVES[hv.getUint16(at, true)];
		if (n.transcendental) throw new Error(`${c.name}: uses ${n.name}, which is not byte-identical across the VMs`);
	}
	const data = new Uint8Array(DATA_CAPACITY);
	const vm = new Vm(data);
	const err = vm.load(image);
	if (err !== "none") throw new Error(`${c.name}: load ${err}`);

	const sets = c.sets.map((s) => {
		const sym = debug.resolve(s.path);
		const e = sym && elemOf(sym.type);
		if (!sym || !e) throw new Error(`${c.name}: @set of unknown ${s.path}`);
		return { cycle: s.cycle, addr: sym.addr, size: ELEM[e].size, value: BigInt.asUintN(ELEM[e].size * 8, s.value) };
	});
	const runLines = [`cycles ${c.cycles}`, `dt ${c.dt}`];
	for (const s of sets) {
		const bytes: string[] = [];
		for (let i = 0; i < s.size; i++) bytes.push(Number((s.value >> BigInt(8 * i)) & 0xffn).toString(16).padStart(2, "0"));
		runLines.push(`set ${s.cycle} ${s.addr} ${bytes.join("")}`);
	}

	const dataSize = vm.dataSize;
	const records: Uint8Array[] = [];
	const head = new Uint8Array(8);
	new DataView(head.buffer).setUint32(0, dataSize, true);
	let cycles = 0;
	for (let k = 0; k < c.cycles; k++) {
		for (const s of sets) {
			if (s.cycle !== k) continue;
			for (let i = 0; i < s.size; i++) data[s.addr + i] = Number((s.value >> BigInt(8 * i)) & 0xffn);
		}
		const res = vm.runCycle(0, (k * c.dt) >>> 0);
		const rec = new Uint8Array(12 + dataSize);
		const dv = new DataView(rec.buffer);
		dv.setUint8(0, res.state === "done" ? 0 : 1);
		dv.setUint8(1, res.trap);
		dv.setUint16(2, res.trapCode, true);
		dv.setUint32(4, res.pc, true);
		dv.setUint32(8, res.instructions, true);
		rec.set(data.subarray(0, dataSize), 12);
		records.push(rec);
		cycles++;
		if (res.state === "trapped") break;
	}
	new DataView(head.buffer).setUint32(4, cycles, true);
	const trace = new Uint8Array(8 + records.reduce((n, x) => n + x.length, 0));
	trace.set(head, 0);
	let at = 8;
	for (const x of records) {
		trace.set(x, at);
		at += x.length;
	}
	const final = (p: string) => {
		const sym = debug.resolve(p);
		const e = sym && elemOf(sym.type);
		if (!sym || !e) throw new Error(`${c.name}: unknown ${p}`);
		const dv = new DataView(data.buffer);
		const info = ELEM[e];
		if (e === "REAL") return String(dv.getFloat32(sym.addr, true));
		if (e === "LREAL") return String(dv.getFloat64(sym.addr, true));
		if (e === "BOOL") return data[sym.addr] ? "TRUE" : "FALSE";
		let v = 0n;
		for (let i = info.size - 1; i >= 0; i--) v = (v << 8n) | BigInt(data[sym.addr + i]);
		return String(info.signed ? BigInt.asIntN(info.size * 8, v) : v);
	};
	return { image, run: runLines.join("\n") + "\n", trace, final };
}

// ---- the cases ------------------------------------------------------------------------------------
const cases: Case[] = [
	...fs.readdirSync(CORPUS_DIR).filter((f) => f.endsWith(".st")).sort().map((f) =>
		parseCase(f.slice(0, -3), fs.readFileSync(path.join(CORPUS_DIR, f), "utf-8"))),
	...Array.from({ length: 40 }, (_, i) => randomCase(i)),
];

describe("PLC differential corpus", () => {
	const written = new Set<string>();
	for (const c of cases) {
		it(c.name, () => {
			const a = runCase(c);
			for (const e of c.expects) expect(`${e.path} = ${a.final(e.path)}`).toBe(`${e.path} = ${e.value}`);
			const files: [string, Uint8Array | string][] = [[`${c.name}.img`, a.image], [`${c.name}.run`, a.run], [`${c.name}.trace`, a.trace]];
			for (const [f, content] of files) {
				written.add(f);
				const p = path.join(OUT_DIR, f);
				const bytes = typeof content === "string" ? new TextEncoder().encode(content) : content;
				if (UPDATE) {
					fs.mkdirSync(OUT_DIR, { recursive: true });
					fs.writeFileSync(p, bytes);
				} else {
					expect(fs.existsSync(p), `${p} is missing: npm run plc:corpus`).toBe(true);
					expect(Buffer.compare(fs.readFileSync(p), Buffer.from(bytes)) === 0, `${p} is out of date: npm run plc:corpus`).toBe(true);
				}
			}
		});
	}

	it("has no stale files", () => {
		const stale = fs.existsSync(OUT_DIR) ? fs.readdirSync(OUT_DIR).filter((f) => !written.has(f) && /\.(img|run|trace)$/.test(f)) : [];
		if (UPDATE) for (const f of stale) fs.rmSync(path.join(OUT_DIR, f));
		else expect(stale, "stale files in rtoos/tests/plc_corpus: npm run plc:corpus").toEqual([]);
	});

	it("covers at least 50 programs, traps included", () => {
		expect(cases.length).toBeGreaterThanOrEqual(50);
	});
});
