// SPDX-License-Identifier: MIT
// IEC 61131-3 compiler: declarations, types, layout, type checking and code generation to the
// PLC bytecode (rtoos/docs/plc.md). check*() turns expressions into a typed tree with the implicit
// conversions made explicit; emit*() writes bytecode. DOM-free.
import { Native, Op } from "../generated/isa.ts";
import { writeImage, type ImageContents, type TaskInfo } from "../image.ts";
import type * as A from "./ast.ts";
import { Diagnostics, type Diagnostic, type Span } from "./diagnostics.ts";
import { Emitter, type Label } from "./emitter.ts";
import { parse } from "./parser.ts";
import { STDLIB_FILE, STDLIB_SOURCE } from "./stdlib.ts";
import {
	ELEM, ERROR, T, alignOf, alignUp, assignable, elemOf, implicitElem, isBitLike, isElemName, isInteger, isNumeric,
	isReal, sameType, sizeOf, typeName, type ElemName, type FbLayout, type Field, type Type,
} from "./types.ts";

export const COMPILER_VERSION = 0x000100; // 0.1.0

// ---- input and output ---------------------------------------------------------------------------
export interface SourceUnit {
	file: string;
	text: string;
}

export interface TaskDef {
	name: string;
	/** In I/O cycles. */
	intervalCycles: number;
	priority?: number;
	budget?: number;
	maxOverruns?: number;
	programs: string[];
}

export interface CompileOptions {
	/** Data segment capacity of the target (bytes). */
	maxData?: number;
	/** Code segment capacity of the target (bytes). */
	maxCode?: number;
	ioMapHash?: number;
}

export interface StatementInfo {
	pc: number;
	pou: string;
	span: Span;
}

export interface Symbol {
	addr: number;
	type: Type;
}

export interface DebugInfo {
	statements: StatementInfo[];
	/** Programs (instances) and global variables by upper-case name. */
	symbols: Map<string, Symbol>;
	/** "PLC_PRG.timer.Q", "PLC_PRG.values[3]", "G_COUNT": address and type, or null. */
	resolve(path: string): Symbol | null;
	dataSize: number;
}

export interface CompileResult {
	ok: boolean;
	diagnostics: Diagnostic[];
	image: Uint8Array | null;
	debug: DebugInfo | null;
}

// ---- internal model ---------------------------------------------------------------------------
type Storage = "global" | "this" | "frame";

interface VarInfo {
	name: string;
	key: string;
	type: Type;
	storage: Storage;
	offset: number;
	cls: A.VarClass | "RESULT";
	constant: boolean;
	/** VAR_IN_OUT: the variable holds the address of the actual parameter. */
	byRef: boolean;
	init: A.Init | null;
	span: Span;
}

interface PouInfo {
	ast: A.Pou;
	key: string;
	name: string;
	kind: A.PouKind;
	vars: Map<string, VarInfo>;
	/** Inputs, in-outs (and for FBs outputs) in declaration order. */
	params: VarInfo[];
	layout: FbLayout | null; // FB and PROGRAM: the instance
	frameSize: number;
	returnType: Type | null;
	result: VarInfo | null;
	callees: Set<string>;
	library: boolean;
	state: "new" | "laying" | "laid" | "emitted";
}

interface Dyn {
	index: TExpr;
	lo: number;
	hi: number;
	stride: number;
}

interface LV {
	type: Type;
	base: { k: "global"; addr: number } | { k: "this"; off: number } | { k: "frame"; off: number } |
		{ k: "ptrThis"; off: number } | { k: "ptrFrame"; off: number };
	offset: number;
	dyn: Dyn[];
	writable: boolean;
	/** Why it is not writable (for the message). */
	why?: string;
}

type TExpr =
	| { k: "const"; type: Type; raw: number | bigint }
	| { k: "load"; type: Type; lv: LV }
	| { k: "conv"; type: Type; from: Type; e: TExpr; checked: boolean; trunc?: boolean }
	| { k: "unary"; type: Type; op: "-" | "NOT"; e: TExpr }
	| { k: "binary"; type: Type; op: A.BinaryOp; l: TExpr; r: TExpr; opType: Type }
	| { k: "call"; type: Type; fn: PouInfo; args: { param: VarInfo; value: TExpr | null; lv: LV | null }[] }
	| { k: "intrinsic"; type: Type; name: string; args: TExpr[]; opType: Type }
	| { k: "lvalue"; type: Type; lv: LV } // a composite value (structure, array): only copied by address
	| { k: "error"; type: Type };

type Const = { k: "int"; v: bigint } | { k: "real"; v: number } | { k: "bool"; v: boolean } | { k: "time"; v: number } |
	{ k: "enum"; type: Type; v: bigint };

class CompileError {}

const ERR: TExpr = { k: "error", type: ERROR };

// conversion function names: X_TO_Y
const CONV_RE = /^([A-Z]+)_TO_([A-Z]+)$/;

const REAL_NATIVES: Record<string, Native> = {
	LN: Native.LN_F64, LOG: Native.LOG_F64, EXP: Native.EXP_F64, SIN: Native.SIN_F64, COS: Native.COS_F64,
	TAN: Native.TAN_F64, ASIN: Native.ASIN_F64, ACOS: Native.ACOS_F64, ATAN: Native.ATAN_F64,
};

const MAX_FRAME_OFFSET = 0xffff;

export function compile(sources: SourceUnit[], tasks?: TaskDef[], options: CompileOptions = {}): CompileResult {
	return new Compiler(options).run(sources, tasks ?? [{ name: "MainTask", intervalCycles: 1, programs: ["PLC_PRG"] }]);
}

class Compiler {
	private diag = new Diagnostics();
	private types = new Map<string, A.TypeDecl>();
	private resolvedTypes = new Map<string, Type>();
	private resolving = new Set<string>();
	private pous = new Map<string, PouInfo>();
	private globals = new Map<string, VarInfo>();
	private enumValues = new Map<string, Type[]>();
	private emitter = new Emitter();
	private natives = new Set<Native>();
	private statements: StatementInfo[] = [];
	private dataTop = 0;
	/** Bytes of composite constants (initial values of function locals), copied with MEMCPY. */
	private constPool: { addr: number; bytes: Uint8Array }[] = [];
	private dataInit = new Map<number, Uint8Array>(); // addr -> bytes, written into the data image

	// per body
	private pou: PouInfo | null = null;
	private loops: { exit: Label; cont: Label }[] = [];
	private epilogue: Label | null = null;

	constructor(private readonly options: CompileOptions) {}

	run(sources: SourceUnit[], taskDefs: TaskDef[]): CompileResult {
		const units: A.Unit[] = [];
		units.push(...parse(STDLIB_SOURCE, STDLIB_FILE, this.diag));
		for (const s of sources) units.push(...parse(s.text, s.file, this.diag));
		try {
			this.declare(units);
			for (const t of this.types.values()) this.resolveNamedType(t.key, t.span);
			this.layoutGlobals(units);
			for (const p of this.pous.values()) if (!p.library) this.layoutPou(p);
			const programs = this.placePrograms(taskDefs);
			// bodies: every user POU (diagnostics), the library POUs that are used
			const queue = [...this.pous.values()].filter((p) => !p.library);
			for (let i = 0; i < queue.length; i++) {
				const p = queue[i];
				if (p.state === "emitted") continue;
				this.layoutPou(p);
				this.emitBody(p);
				for (const c of p.callees) {
					const q = this.pous.get(c);
					if (q && q.state !== "emitted") queue.push(q);
				}
			}
			if (this.diag.hasErrors) return this.result(null, null);
			return this.link(programs, taskDefs, sources);
		} catch (e) {
			if (!(e instanceof CompileError)) throw e;
			return this.result(null, null);
		}
	}

	private result(image: Uint8Array | null, debug: DebugInfo | null): CompileResult {
		// by position: parse and semantic errors interleave as in the text
		const files = [...new Set(this.diag.list.map((d) => d.span.start.file))];
		const diagnostics = this.diag.list
			.filter((d) => !d.span.start.file.startsWith("<") || d.severity === "error")
			.sort((a, b) => files.indexOf(a.span.start.file) - files.indexOf(b.span.start.file) || a.span.start.offset - b.span.start.offset);
		return { ok: !this.diag.hasErrors && image !== null, diagnostics, image, debug };
	}

	private fatal(span: Span, message: string): never {
		this.diag.error(span, message);
		throw new CompileError();
	}

	// ---- declarations -----------------------------------------------------------------------------
	private declare(units: A.Unit[]): void {
		const taken = (key: string) => this.types.has(key) || this.pous.has(key);
		for (const u of units) {
			if (u.kind === "type") {
				if (taken(u.key) || isElemName(u.key)) this.diag.error(u.span, `'${u.name}' is already declared`);
				else this.types.set(u.key, u);
			} else if (u.kind === "pou") {
				if (taken(u.key) || isElemName(u.key)) {
					this.diag.error(u.span, `'${u.name}' is already declared`);
					continue;
				}
				this.pous.set(u.key, {
					ast: u, key: u.key, name: u.name, kind: u.pouKind, vars: new Map(), params: [], layout: null, frameSize: 0,
					returnType: null, result: null, callees: new Set(), library: u.file.startsWith("<"), state: "new",
				});
			}
		}
	}

	private resolveNamedType(key: string, span: Span): Type {
		const done = this.resolvedTypes.get(key);
		if (done) return done;
		if (isElemName(key)) return T[key];
		const decl = this.types.get(key);
		if (!decl) {
			const pou = this.pous.get(key);
			if (pou && pou.kind === "FUNCTION_BLOCK") {
				this.layoutPou(pou);
				return pou.layout ? { k: "fb", layout: pou.layout } : ERROR;
			}
			if (pou) {
				this.diag.error(span, `'${pou.name}' is a ${pou.kind}, not a type`);
				return ERROR;
			}
			if (key === "STRING" || key === "WSTRING") this.diag.error(span, "STRING is not supported yet");
			else this.diag.error(span, `unknown type '${key}'`);
			return ERROR;
		}
		if (this.resolving.has(key)) {
			this.diag.error(decl.span, `type '${decl.name}' contains itself`);
			return ERROR;
		}
		this.resolving.add(key);
		let t: Type = ERROR;
		const d = decl.def;
		if (d.kind === "alias") {
			t = this.resolveTypeRef(d.type);
			if (t.k === "subrange" && t.name === null) t = { ...t, name: decl.name };
		} else if (d.kind === "struct") {
			const fields: Field[] = [];
			let off = 0;
			let align = 1;
			for (const f of d.fields) {
				if (fields.some((x) => x.key === f.key)) this.diag.error(f.span, `'${f.name}' is declared twice`);
				const ft = this.resolveTypeRef(f.type);
				if (ft.k === "fb") this.diag.error(f.span, "a structure cannot contain a function block instance");
				const a = alignOf(ft);
				off = alignUp(off, a);
				fields.push({ name: f.name, key: f.key, type: ft, offset: off, init: f.init });
				off += sizeOf(ft);
				align = Math.max(align, a);
			}
			t = { k: "struct", name: decl.name, key: decl.key, fields, size: alignUp(Math.max(off, 1), align), align };
		} else {
			const base = d.base ? this.resolveTypeRef(d.base) : T.INT;
			const be = elemOf(base);
			if (base.k !== "elem" || be === null || !isInteger(be)) {
				this.diag.error(decl.span, "the base type of an enumeration must be an integer type");
			}
			const baseName: ElemName = base.k === "elem" && isInteger(base.name) ? base.name : "INT";
			const values = new Map<string, { name: string; value: bigint }>();
			let next = 0n;
			for (const v of d.values) {
				let value = next;
				if (v.value) {
					const c = this.constEval(v.value, null);
					if (c?.k === "int") value = c.v;
					else this.diag.error(v.span, "the value of an enumeration element must be an integer constant");
				}
				if (value < ELEM[baseName].min || value > ELEM[baseName].max) this.diag.error(v.span, `${value} does not fit ${baseName}`);
				if (values.has(v.key)) this.diag.error(v.span, `'${v.name}' is declared twice`);
				values.set(v.key, { name: v.name, value });
				next = value + 1n;
			}
			let init = d.values.length > 0 ? values.get(d.values[0].key)!.value : 0n;
			if (d.init) {
				const iv = values.get(d.init);
				if (iv) init = iv.value;
				else this.diag.error(decl.span, `'${d.init}' is not an element of ${decl.name}`);
			}
			t = { k: "enum", name: decl.name, key: decl.key, base: baseName, values, init };
			for (const k of values.keys()) {
				const list = this.enumValues.get(k) ?? [];
				list.push(t);
				this.enumValues.set(k, list);
			}
		}
		this.resolving.delete(key);
		this.resolvedTypes.set(key, t);
		return t;
	}

	private resolveTypeRef(r: A.TypeRef): Type {
		switch (r.kind) {
			case "named": return this.resolveNamedType(r.key, r.span);
			case "string":
				this.diag.error(r.span, "STRING is not supported yet");
				return ERROR;
			case "subrange": {
				const base = this.resolveTypeRef(r.base);
				const lo = this.constEval(r.lo, null);
				const hi = this.constEval(r.hi, null);
				if (base.k !== "elem" || !isInteger(base.name) || lo?.k !== "int" || hi?.k !== "int") {
					this.diag.error(r.span, "a subrange needs an integer base type and integer constant bounds");
					return ERROR;
				}
				if (lo.v > hi.v || lo.v < ELEM[base.name].min || hi.v > ELEM[base.name].max) {
					this.diag.error(r.span, `the subrange ${lo.v}..${hi.v} does not fit ${base.name}`);
					return ERROR;
				}
				return { k: "subrange", name: null, base: base.name, lo: lo.v, hi: hi.v };
			}
			case "array": {
				const elem = this.resolveTypeRef(r.of);
				const dims: { lo: number; hi: number }[] = [];
				for (const d of r.dims) {
					const lo = this.constEval(d.lo, null);
					const hi = this.constEval(d.hi, null);
					if (lo?.k !== "int" || hi?.k !== "int" || lo.v > hi.v) {
						this.diag.error(r.span, "array bounds must be integer constants with lower <= upper");
						return ERROR;
					}
					if (lo.v < -(1n << 31n) || hi.v >= 1n << 31n) {
						this.diag.error(r.span, "array bounds must be DINT values");
						return ERROR;
					}
					dims.push({ lo: Number(lo.v), hi: Number(hi.v) });
				}
				const count = dims.reduce((n, d) => n * (d.hi - d.lo + 1), 1);
				const size = count * sizeOf(elem);
				if (size > 0x7fffffff) {
					this.diag.error(r.span, "the array is too large");
					return ERROR;
				}
				return { k: "array", dims, elem, size, align: alignOf(elem) };
			}
		}
	}

	// ---- variables and layout ---------------------------------------------------------------------
	private allocGlobal(size: number, align: number): number {
		this.dataTop = alignUp(this.dataTop, Math.min(align, 8));
		const a = this.dataTop;
		this.dataTop += Math.max(size, 1);
		return a;
	}

	private layoutGlobals(units: A.Unit[]): void {
		for (const u of units) {
			if (u.kind !== "globals") continue;
			for (const v of u.vars) {
				if (this.globals.has(v.key)) {
					this.diag.error(v.span, `global variable '${v.name}' is declared twice`);
					continue;
				}
				if (v.at) this.diag.error(v.span, "AT addresses are not supported yet: the I/O variables come from the register map");
				if (v.retain) this.diag.warning(v.span, "RETAIN is not supported yet: the variable starts with its initial value");
				const type = this.resolveTypeRef(v.type);
				const addr = this.allocGlobal(sizeOf(type), alignOf(type));
				const info: VarInfo = {
					name: v.name, key: v.key, type, storage: "global", offset: addr, cls: "VAR_GLOBAL", constant: v.constant,
					byRef: false, init: v.init, span: v.span,
				};
				this.globals.set(v.key, info);
				this.initValue(addr, type, v.init, v.span);
			}
		}
	}

	/** Variables, parameters and (FB, PROGRAM) the instance layout of a POU. */
	private layoutPou(p: PouInfo): void {
		if (p.state !== "new") {
			if (p.state === "laying") this.fatal(p.ast.span, `'${p.name}' contains an instance of itself`);
			return;
		}
		p.state = "laying";
		const instance = p.kind !== "FUNCTION";
		const fields: Field[] = [];
		let instSize = 0;
		let frame = 0;
		const place = (v: A.VarDecl, type: Type, byRef: boolean): VarInfo => {
			const size = byRef ? 4 : sizeOf(type);
			const align = byRef ? 4 : alignOf(type);
			const inFrame = !instance || v.cls === "VAR_TEMP";
			let offset: number;
			if (inFrame) {
				frame = alignUp(frame, align);
				offset = frame;
				frame += Math.max(size, 1);
			} else {
				instSize = alignUp(instSize, align);
				offset = instSize;
				instSize += Math.max(size, 1);
				fields.push({ name: v.name, key: v.key, type, offset, init: v.init });
			}
			return {
				name: v.name, key: v.key, type, storage: inFrame ? "frame" : "this", offset, cls: v.cls, constant: v.constant,
				byRef, init: v.init, span: v.span,
			};
		};
		for (const v of p.ast.vars) {
			if (p.vars.has(v.key)) {
				this.diag.error(v.span, `'${v.name}' is declared twice`);
				continue;
			}
			if (v.at) this.diag.error(v.span, "AT addresses are not supported yet: the I/O variables come from the register map");
			if (v.retain) this.diag.warning(v.span, "RETAIN is not supported yet: the variable starts with its initial value");
			if (v.cls === "VAR_GLOBAL") {
				this.diag.error(v.span, "VAR_GLOBAL belongs into a global variable list");
				continue;
			}
			if (v.cls === "VAR_EXTERNAL") {
				const g = this.globals.get(v.key);
				if (!g) this.diag.error(v.span, `no global variable '${v.name}'`);
				else if (!sameType(this.resolveTypeRef(v.type), g.type)) this.diag.error(v.span, `'${v.name}' is declared as ${typeName(g.type)} in the global variable list`);
				else p.vars.set(v.key, { ...g, constant: g.constant || v.constant });
				continue;
			}
			if (p.kind === "FUNCTION" && v.cls === "VAR_OUTPUT") {
				this.diag.error(v.span, "VAR_OUTPUT in functions is not supported yet");
				continue;
			}
			const type = this.resolveTypeRef(v.type);
			if (type.k === "fb" && (p.kind === "FUNCTION" || v.cls !== "VAR")) {
				this.diag.error(v.span, p.kind === "FUNCTION" ? "a function cannot contain function block instances" :
					"a function block instance must be declared in VAR");
			}
			if (v.constant && v.init === null && type.k === "elem") this.diag.warning(v.span, `constant '${v.name}' has no value (0)`);
			const info = place(v, type, v.cls === "VAR_IN_OUT");
			p.vars.set(v.key, info);
			if (v.cls === "VAR_INPUT" || v.cls === "VAR_IN_OUT" || (p.kind !== "FUNCTION" && v.cls === "VAR_OUTPUT")) p.params.push(info);
		}
		if (p.kind === "FUNCTION") {
			const rt = this.resolveTypeRef(p.ast.returnType!);
			if (rt.k !== "elem" && rt.k !== "enum" && rt.k !== "subrange" && rt.k !== "error") {
				this.diag.error(p.ast.returnType!.span, "a function returns an elementary value (structures and arrays: not yet)");
			}
			p.returnType = rt;
			frame = alignUp(frame, 8);
			p.result = {
				name: p.name, key: p.key, type: rt, storage: "frame", offset: frame, cls: "RESULT", constant: false, byRef: false,
				init: null, span: p.ast.span,
			};
			p.vars.set(p.key, p.result);
			frame += 8;
		} else {
			p.layout = { key: p.key, name: p.name, size: alignUp(Math.max(instSize, 1), 8), fields };
		}
		p.frameSize = frame;
		p.state = "laid";
	}

	private placePrograms(taskDefs: TaskDef[]): Map<string, number> {
		const placed = new Map<string, number>();
		for (const t of taskDefs) {
			for (const name of t.programs) {
				const p = this.pous.get(name.toUpperCase());
				const span = { start: { file: "<tasks>", line: 1, col: 1, offset: 0 }, end: { file: "<tasks>", line: 1, col: 1, offset: 0 } };
				if (!p || p.kind !== "PROGRAM") this.fatal(p?.ast.span ?? span, `task '${t.name}': no PROGRAM '${name}'`);
				if (placed.has(p.key)) this.fatal(p.ast.span, `PROGRAM '${p.name}' is called by two tasks`);
				const addr = this.allocGlobal(p.layout!.size, 8);
				placed.set(p.key, addr);
				this.initInstance(addr, p.layout!);
			}
		}
		return placed;
	}

	// ---- initial values -------------------------------------------------------------------------------
	private put(addr: number, bytes: Uint8Array): void {
		this.dataInit.set(addr, bytes);
	}

	private initInstance(addr: number, layout: FbLayout): void {
		for (const f of layout.fields) this.initValue(addr + f.offset, f.type, f.init ?? null, null);
	}

	/** Writes the initial value of a variable of type `t` at `addr` (declared init, else the type's default). */
	private initValue(addr: number, t: Type, init: A.Init | null, span: Span | null): void {
		switch (t.k) {
			case "fb":
				if (init) this.diag.error(span ?? spanOfInit(init), "a function block instance has no initial value");
				this.initInstance(addr, t.layout);
				return;
			case "struct": {
				const given = new Map<string, A.Init>();
				if (init) {
					if (init.kind !== "struct") {
						this.diag.error(span ?? spanOfInit(init), `the initial value of a ${t.name} is (field := value, ...)`);
					} else {
						for (const f of init.fields) {
							if (!t.fields.some((x) => x.key === f.key)) this.diag.error(f.span, `${t.name} has no field '${f.name}'`);
							given.set(f.key, f.value);
						}
					}
				}
				for (const f of t.fields) this.initValue(addr + f.offset, f.type, given.get(f.key) ?? f.init ?? null, null);
				return;
			}
			case "array": {
				const count = t.dims.reduce((n, d) => n * (d.hi - d.lo + 1), 1);
				const es = sizeOf(t.elem);
				const values: (A.Init | null)[] = [];
				if (init) {
					if (init.kind !== "array") {
						this.diag.error(span ?? spanOfInit(init), "the initial value of an array is [v1, v2, n(v), ...]");
					} else {
						for (const item of init.items) {
							let n = 1;
							if (item.repeat) {
								const c = this.constEval(item.repeat, null);
								if (c?.k !== "int" || c.v < 0n) this.diag.error(item.span, "a repetition count is a non-negative integer constant");
								else n = Number(c.v);
							}
							for (let i = 0; i < n; i++) values.push(item.value);
						}
						if (values.length > count) this.diag.error(init.span, `${values.length} initial values for ${count} elements`);
					}
				}
				for (let i = 0; i < count; i++) this.initValue(addr + i * es, t.elem, values[i] ?? null, null);
				return;
			}
			case "error":
				return;
			default: {
				const e = elemOf(t)!;
				let raw: number | bigint = t.k === "enum" ? toRaw(e, t.init) : 0;
				if (t.k === "subrange") raw = toRaw(e, t.lo > 0n ? t.lo : t.hi < 0n ? t.hi : 0n);
				if (init) {
					if (init.kind !== "expr") this.diag.error(spanOfInit(init), `the initial value of a ${typeName(t)} is a constant`);
					else {
						const r = this.constFor(init.expr, t);
						if (r !== null) raw = r;
					}
				}
				const b = new Uint8Array(ELEM[e].size);
				writeRaw(b, 0, e, raw);
				if (b.some((x) => x !== 0)) this.put(addr, b);
			}
		}
	}

	/** A constant expression as the raw value of type `t`, null after an error. */
	private constFor(e: A.Expr, t: Type): number | bigint | null {
		const c = this.constEval(e, t);
		if (c === null) {
			this.diag.error(e.span, "an initial value must be a constant");
			return null;
		}
		const el = elemOf(t);
		if (el === null) return null;
		const ok = (raw: number | bigint) => raw;
		switch (c.k) {
			case "int":
				if (isInteger(el) || ELEM[el].category === "bit") {
					if (!this.fits(c.v, t)) {
						this.diag.error(e.span, `${c.v} does not fit ${typeName(t)}`);
						return null;
					}
					return ok(toRaw(el, c.v));
				}
				if (isReal(el)) return ok(realRaw(el, Number(c.v)));
				break;
			case "real":
				if (isReal(el)) return ok(realRaw(el, c.v));
				break;
			case "bool":
				if (el === "BOOL") return c.v ? 1 : 0;
				break;
			case "time":
				if (el === "TIME") return c.v >>> 0;
				break;
			case "enum":
				if (sameType(c.type, t)) return ok(toRaw(el, c.v));
				break;
		}
		this.diag.error(e.span, `a ${constName(c)} is not a ${typeName(t)}`);
		return null;
	}

	private fits(v: bigint, t: Type): boolean {
		if (t.k === "subrange") return v >= t.lo && v <= t.hi;
		const e = elemOf(t)!;
		return v >= ELEM[e].min && v <= ELEM[e].max;
	}

	/** Compile-time value of a constant expression; `hint`: the type an enum value is looked up in. */
	private constEval(e: A.Expr, hint: Type | null): Const | null {
		switch (e.kind) {
			case "int": return { k: "int", v: e.value };
			case "real": return { k: "real", v: e.value };
			case "bool": return { k: "bool", v: e.value };
			case "time": return { k: "time", v: e.ms };
			case "enumLit": {
				const t = this.resolvedTypes.get(e.typeKey) ?? (this.types.has(e.typeKey) ? this.resolveNamedType(e.typeKey, e.span) : null);
				if (!t || t.k !== "enum") return null;
				const v = t.values.get(e.key);
				return v ? { k: "enum", type: t, v: v.value } : null;
			}
			case "name": {
				if (hint?.k === "enum") {
					const v = hint.values.get(e.key);
					if (v) return { k: "enum", type: hint, v: v.value };
				}
				const local = this.pou?.vars.get(e.key);
				const g = local ?? this.globals.get(e.key);
				if (g) {
					if (g.constant && g.init?.kind === "expr") return this.constEval(g.init.expr, g.type);
					return null;
				}
				const enums = this.enumValues.get(e.key);
				if (enums?.length === 1) return { k: "enum", type: enums[0], v: (enums[0] as { values: Map<string, { value: bigint }> }).values.get(e.key)!.value };
				return null;
			}
			case "unary": {
				const c = this.constEval(e.operand, hint);
				if (e.op === "-" && c?.k === "int") return { k: "int", v: -c.v };
				if (e.op === "-" && c?.k === "real") return { k: "real", v: -c.v };
				if (e.op === "-" && c?.k === "time") return { k: "time", v: -c.v };
				if (e.op === "+") return c;
				if (e.op === "NOT" && c?.k === "bool") return { k: "bool", v: !c.v };
				return null;
			}
			case "binary": {
				const l = this.constEval(e.left, hint);
				const r = this.constEval(e.right, hint);
				if (l?.k === "int" && r?.k === "int") {
					switch (e.op) {
						case "+": return { k: "int", v: l.v + r.v };
						case "-": return { k: "int", v: l.v - r.v };
						case "*": return { k: "int", v: l.v * r.v };
						case "/": return r.v === 0n ? null : { k: "int", v: l.v / r.v };
						case "MOD": return r.v === 0n ? null : { k: "int", v: l.v % r.v };
					}
				}
				if (l?.k === "real" && r?.k === "real") {
					switch (e.op) {
						case "+": return { k: "real", v: l.v + r.v };
						case "-": return { k: "real", v: l.v - r.v };
						case "*": return { k: "real", v: l.v * r.v };
						case "/": return { k: "real", v: l.v / r.v };
					}
				}
				return null;
			}
			default: return null;
		}
	}

	// ---- bodies -------------------------------------------------------------------------------------
	private emitBody(p: PouInfo): void {
		this.pou = p;
		this.loops = [];
		const em = this.emitter;
		this.entries.set(p.key, em.here);
		em.op(Op.ENTER);
		const frameOperand = em.here;
		em.u32(0);
		this.epilogue = em.label();
		try {
			if (p.kind === "FUNCTION") {
				// the caller pushed the arguments in declaration order: pop the last first
				for (let i = p.params.length - 1; i >= 0; i--) {
					const v = p.params[i];
					if (v.byRef) this.storeFrame(v.offset, "DWORD");
					else if (isComposite(v.type)) {
						em.opArg(Op.ADDR_L, v.offset);
						em.op(Op.SWAP);
						em.opArg(Op.MEMCPY, sizeOf(v.type));
					} else this.storeFrame(v.offset, elemOf(v.type) ?? "DWORD");
				}
			}
			// frame variables with an initial value (function locals, VAR_TEMP): at every call
			for (const v of p.vars.values()) {
				if (v.storage !== "frame" || v.byRef || v.cls === "VAR_INPUT" || v.cls === "RESULT") continue;
				this.emitFrameInit(v);
			}
			this.statementsOf(p.ast.body);
		} catch (e) {
			if (!(e instanceof CompileError)) throw e;
		}
		em.bind(this.epilogue);
		if (p.kind === "FUNCTION" && p.result) {
			const e = elemOf(p.result.type);
			if (e) this.loadDirect(Op.LDL_U8, p.result.offset, e);
		}
		em.op(Op.RET);
		if (p.frameSize > MAX_FRAME_OFFSET) this.diag.error(p.ast.span, `the local variables of '${p.name}' need more than 64 KB`);
		this.patchFrame(frameOperand);
		p.state = "emitted";
		this.pou = null;
	}

	private entries = new Map<string, number>();
	/** ENTER operands: the frame size is final only after the body (hidden temporaries). */
	private framePatches: { at: number; pou: PouInfo }[] = [];
	private patchFrame(at: number): void {
		this.framePatches.push({ at, pou: this.pou! });
	}

	private emitFrameInit(v: VarInfo): void {
		const em = this.emitter;
		const t = v.type;
		const e = elemOf(t);
		if (e !== null) {
			let raw: number | bigint | null = null;
			if (v.init?.kind === "expr") raw = this.constFor(v.init.expr, t);
			else if (t.k === "enum" && t.init !== 0n) raw = toRaw(e, t.init);
			else if (t.k === "subrange" && (t.lo > 0n || t.hi < 0n)) raw = toRaw(e, t.lo > 0n ? t.lo : t.hi);
			if (raw === null || raw === 0 || raw === 0n) return;
			this.pushRaw(e, raw);
			this.storeFrame(v.offset, e);
			return;
		}
		if (t.k === "struct" || t.k === "array") {
			// build the initial bytes once, keep them in the constant pool, copy at every call
			const saveInit = this.dataInit;
			this.dataInit = new Map();
			this.initValue(0, t, v.init, v.span);
			const image = new Uint8Array(sizeOf(t));
			for (const [a, b] of this.dataInit) image.set(b, a);
			this.dataInit = saveInit;
			if (image.every((x) => x === 0)) return;
			const addr = this.allocGlobal(image.length, alignOf(t));
			this.constPool.push({ addr, bytes: image });
			this.put(addr, image);
			em.opArg(Op.ADDR_L, v.offset);
			em.opArg(Op.ADDR_G, addr);
			em.opArg(Op.MEMCPY, image.length);
		}
	}

	private hiddenTemp(size: number): number {
		const p = this.pou!;
		p.frameSize = alignUp(p.frameSize, size);
		const off = p.frameSize;
		p.frameSize += size;
		return off;
	}

	// ---- statements -----------------------------------------------------------------------------
	private statementsOf(list: A.Stmt[]): void {
		for (const s of list) {
			this.statements.push({ pc: this.emitter.here, pou: this.pou!.key, span: s.span });
			try {
				this.statement(s);
			} catch (e) {
				if (!(e instanceof CompileError)) throw e;
			}
		}
	}

	private statement(s: A.Stmt): void {
		const em = this.emitter;
		switch (s.kind) {
			case "empty": return;
			case "assign": return this.assign(s.target, s.value);
			case "call": return this.callStatement(s.call);
			case "if": {
				const end = em.label();
				for (const b of s.branches) {
					const next = em.label();
					this.emitExpr(this.checkCond(b.cond));
					em.jump(Op.JZ, next);
					this.statementsOf(b.body);
					em.jump(Op.JMP, end);
					em.bind(next);
				}
				if (s.otherwise) this.statementsOf(s.otherwise);
				em.bind(end);
				return;
			}
			case "while": {
				const top = em.label();
				const end = em.label();
				em.bind(top);
				this.emitExpr(this.checkCond(s.cond));
				em.jump(Op.JZ, end);
				this.loops.push({ exit: end, cont: top });
				this.statementsOf(s.body);
				this.loops.pop();
				em.jump(Op.JMP, top);
				em.bind(end);
				return;
			}
			case "repeat": {
				const top = em.label();
				const cont = em.label();
				const end = em.label();
				em.bind(top);
				this.loops.push({ exit: end, cont });
				this.statementsOf(s.body);
				this.loops.pop();
				em.bind(cont);
				this.emitExpr(this.checkCond(s.cond));
				em.jump(Op.JZ, top);
				em.bind(end);
				return;
			}
			case "for": return this.forStatement(s);
			case "case": return this.caseStatement(s);
			case "exit":
			case "continue": {
				const l = this.loops.at(-1);
				if (!l) this.fatal(s.span, `${s.kind.toUpperCase()} outside a loop`);
				em.jump(Op.JMP, s.kind === "exit" ? l.exit : l.cont);
				return;
			}
			case "return":
				em.jump(Op.JMP, this.epilogue!);
				return;
		}
	}

	private checkCond(e: A.Expr): TExpr {
		const c = this.check(e, T.BOOL);
		if (c.type.k !== "error" && !(c.type.k === "elem" && c.type.name === "BOOL")) {
			this.diag.error(e.span, `a condition must be BOOL, not ${typeName(c.type)}`);
		}
		return c;
	}

	private assign(targetExpr: A.Expr, valueExpr: A.Expr): void {
		const target = this.lvalue(targetExpr);
		if (!target) return;
		if (!target.writable) this.fatal(targetExpr.span, target.why ?? "this cannot be assigned");
		if (target.type.k === "fb") this.fatal(targetExpr.span, "a function block instance cannot be assigned");
		if (isComposite(target.type)) {
			const v = this.lvalue(valueExpr);
			if (!v) return;
			if (!sameType(v.type, target.type)) {
				this.fatal(valueExpr.span, `cannot assign ${typeName(v.type)} to ${typeName(target.type)}`);
			}
			this.emitAddr(target);
			this.emitAddr(v);
			this.emitter.opArg(Op.MEMCPY, sizeOf(target.type));
			return;
		}
		const value = this.convertTo(this.check(valueExpr, target.type), target.type, valueExpr.span);
		this.emitStore(target, () => this.emitExpr(value));
	}

	private forStatement(s: Extract<A.Stmt, { kind: "for" }>): void {
		const em = this.emitter;
		const v = this.lvalue(s.variable);
		if (!v) return;
		const e = elemOf(v.type);
		if (v.type.k !== "elem" || e === null || !isInteger(e) || v.dyn.length > 0 || v.base.k.startsWith("ptr")) {
			this.fatal(s.variable.span, "the control variable of FOR is an integer variable");
		}
		if (!v.writable) this.fatal(s.variable.span, v.why ?? "the control variable cannot be assigned");
		const t = v.type;
		const slot = ELEM[e].slot;
		const size = slot === "I64" ? 8 : 4;
		const from = this.convertTo(this.check(s.from, t), t, s.from.span);
		const to = this.convertTo(this.check(s.to, t), t, s.to.span);
		const byConst = s.by ? this.constEval(s.by, t) : { k: "int" as const, v: 1n };
		const by = s.by ? this.convertTo(this.check(s.by, t), t, s.by.span) : null;
		// end and step: evaluated once (hidden temporaries)
		const endOff = this.hiddenTemp(size);
		const stepOff = this.hiddenTemp(size);
		this.emitStore(v, () => this.emitExpr(from));
		this.emitExpr(to);
		this.storeFrame(endOff, e);
		if (by) this.emitExpr(by);
		else this.pushRaw(e, toRaw(e, 1n));
		this.storeFrame(stepOff, e);
		const top = em.label();
		const cont = em.label();
		const end = em.label();
		em.bind(top);
		// continue while (step >= 0 ? v <= end : v >= end); a constant step decides at compile time
		const cmp = (op: "<=" | ">=") => {
			this.emitLoad(v);
			this.loadDirect(Op.LDL_U8, endOff, e);
			em.op(compareOp(op, e));
		};
		if (byConst?.k === "int") cmp(byConst.v >= 0n ? "<=" : ">=");
		else {
			const neg = em.label();
			const done = em.label();
			this.loadDirect(Op.LDL_U8, stepOff, e);
			this.pushRaw(e, toRaw(e, 0n));
			em.op(compareOp("<", e));
			em.jump(Op.JNZ, neg);
			cmp("<=");
			em.jump(Op.JMP, done);
			em.bind(neg);
			cmp(">=");
			em.bind(done);
		}
		em.jump(Op.JZ, end);
		this.loops.push({ exit: end, cont });
		this.statementsOf(s.body);
		this.loops.pop();
		em.bind(cont);
		this.emitStore(v, () => {
			this.emitLoad(v);
			this.loadDirect(Op.LDL_U8, stepOff, e);
			em.op(slot === "I64" ? Op.ADD_I64 : Op.ADD_I32);
			this.normalize(e);
		});
		em.jump(Op.JMP, top);
		em.bind(end);
	}

	private caseStatement(s: Extract<A.Stmt, { kind: "case" }>): void {
		const em = this.emitter;
		const sel = this.check(s.selector, null);
		const st = sel.type;
		const e = elemOf(st);
		if (st.k !== "error" && (e === null || !(isInteger(e) || st.k === "enum" || ELEM[e].category === "bit"))) {
			this.fatal(s.selector.span, `the selector of CASE is an integer or an enumeration, not ${typeName(st)}`);
		}
		const el = e ?? "DINT";
		const size = ELEM[el].slot === "I64" ? 8 : 4;
		const tmp = this.hiddenTemp(size);
		this.emitExpr(sel);
		this.storeFrame(tmp, el);
		const end = em.label();
		const seen: { lo: bigint; hi: bigint; span: Span }[] = [];
		const labelValue = (x: A.Expr): bigint | null => {
			const c = this.constEval(x, st);
			if (c?.k === "int" && st.k !== "enum") {
				if (!this.fits(c.v, st.k === "subrange" ? T[st.base] : st)) {
					this.diag.error(x.span, `${c.v} does not fit ${typeName(st)}`);
					return null;
				}
				return c.v;
			}
			if (c?.k === "enum" && sameType(c.type, st)) return c.v;
			this.diag.error(x.span, `a CASE label is a constant of the selector's type ${typeName(st)}`);
			return null;
		};
		for (const b of s.branches) {
			const body = em.label();
			const next = em.label();
			for (const l of b.labels) {
				const lo = labelValue(l.kind === "value" ? l.value : l.lo);
				const hi = l.kind === "value" ? lo : labelValue(l.hi);
				if (lo === null || hi === null) continue;
				const span = l.kind === "value" ? l.value.span : l.lo.span;
				if (lo > hi) this.diag.error(span, "the range is empty");
				if (seen.some((x) => lo <= x.hi && x.lo <= hi)) this.diag.warning(span, "this CASE label is covered by an earlier one");
				seen.push({ lo, hi, span });
				if (lo === hi) {
					this.loadDirect(Op.LDL_U8, tmp, el);
					this.pushRaw(el, toRaw(el, lo));
					em.op(compareOp("=", el));
					em.jump(Op.JNZ, body);
				} else {
					const skip = em.label();
					this.loadDirect(Op.LDL_U8, tmp, el);
					this.pushRaw(el, toRaw(el, lo));
					em.op(compareOp("<", el));
					em.jump(Op.JNZ, skip);
					this.loadDirect(Op.LDL_U8, tmp, el);
					this.pushRaw(el, toRaw(el, hi));
					em.op(compareOp("<=", el));
					em.jump(Op.JNZ, body);
					em.bind(skip);
				}
			}
			em.jump(Op.JMP, next);
			em.bind(body);
			this.statementsOf(b.body);
			em.jump(Op.JMP, end);
			em.bind(next);
		}
		if (s.otherwise) this.statementsOf(s.otherwise);
		em.bind(end);
	}

	// ---- function block calls --------------------------------------------------------------------
	private callStatement(c: A.CallExpr): void {
		const callee = c.callee;
		// a function call as a statement: its result is dropped
		if (callee.kind === "name" && !this.lookupVar(callee.key)) {
			const r = this.check(c, null);
			this.emitExpr(r);
			if (r.type.k !== "error") this.emitter.op(Op.DROP);
			return;
		}
		const inst = this.lvalue(callee);
		if (!inst) return;
		if (inst.type.k !== "fb") this.fatal(callee.span, `'${exprText(callee)}' is not a function block instance`);
		const layout = inst.type.layout;
		const fb = this.pous.get(layout.key)!;
		this.pou!.callees.add(fb.key);
		const outputs: { field: Field; target: A.Expr; negated: boolean; span: Span }[] = [];
		const given = new Set<string>();
		for (const a of c.args) {
			if (a.key === null) this.fatal(a.span, "a function block is called with formal parameters: name := value");
			const v = fb.vars.get(a.key);
			const field = layout.fields.find((f) => f.key === a.key);
			if (!v || !field || (v.cls !== "VAR_INPUT" && v.cls !== "VAR_OUTPUT" && v.cls !== "VAR_IN_OUT")) {
				this.fatal(a.span, `${layout.name} has no parameter '${a.name}'`);
			}
			if (given.has(a.key)) this.fatal(a.span, `'${a.name}' is given twice`);
			given.add(a.key);
			const member: LV = { ...inst, type: field.type, offset: inst.offset + field.offset, writable: true };
			if (a.output) {
				if (v.cls !== "VAR_OUTPUT") this.fatal(a.span, `'${a.name}' is not an output: use :=`);
				outputs.push({ field, target: a.value, negated: a.negated, span: a.span });
			} else if (v.cls === "VAR_OUTPUT") {
				this.fatal(a.span, `'${a.name}' is an output: use =>`);
			} else if (v.cls === "VAR_IN_OUT") {
				const actual = this.lvalue(a.value);
				if (!actual) continue;
				if (!sameType(actual.type, v.type)) this.fatal(a.value.span, `'${a.name}' needs a ${typeName(v.type)}, not ${typeName(actual.type)}`);
				if (!actual.writable) this.fatal(a.value.span, "a VAR_IN_OUT parameter needs a writable variable");
				this.emitStore({ ...member, type: T.DWORD }, () => this.emitAddr(actual));
			} else if (isComposite(v.type)) {
				const actual = this.lvalue(a.value);
				if (!actual) continue;
				if (!sameType(actual.type, v.type)) this.fatal(a.value.span, `'${a.name}' needs a ${typeName(v.type)}`);
				this.emitAddr(member);
				this.emitAddr(actual);
				this.emitter.opArg(Op.MEMCPY, sizeOf(v.type));
			} else {
				const value = this.convertTo(this.check(a.value, v.type), v.type, a.value.span);
				this.emitStore(member, () => this.emitExpr(value));
			}
		}
		for (const p of fb.params) {
			if (p.cls === "VAR_IN_OUT" && !given.has(p.key)) this.diag.error(c.span, `the VAR_IN_OUT parameter '${p.name}' must be given`);
		}
		this.emitAddr(inst);
		this.emitter.call(Op.CALL_FB, fb.key);
		for (const o of outputs) {
			const target = this.lvalue(o.target);
			if (!target) continue;
			if (!target.writable) this.fatal(o.target.span, target.why ?? "this cannot be assigned");
			const src: LV = { ...inst, type: o.field.type, offset: inst.offset + o.field.offset, writable: true };
			if (isComposite(o.field.type)) {
				if (!sameType(o.field.type, target.type)) this.fatal(o.span, `cannot assign ${typeName(o.field.type)} to ${typeName(target.type)}`);
				this.emitAddr(target);
				this.emitAddr(src);
				this.emitter.opArg(Op.MEMCPY, sizeOf(target.type));
				continue;
			}
			let value: TExpr = { k: "load", type: o.field.type, lv: src };
			if (o.negated) value = this.unaryNot(value, o.span);
			value = this.convertTo(value, target.type, o.span);
			this.emitStore(target, () => this.emitExpr(value));
		}
	}

	// ---- names and lvalues --------------------------------------------------------------------------
	private lookupVar(key: string): VarInfo | null {
		return this.pou?.vars.get(key) ?? this.globals.get(key) ?? null;
	}

	private varLV(v: VarInfo): LV {
		let base: LV["base"];
		if (v.storage === "global") base = { k: "global", addr: v.offset };
		else if (v.byRef) base = v.storage === "this" ? { k: "ptrThis", off: v.offset } : { k: "ptrFrame", off: v.offset };
		else base = v.storage === "this" ? { k: "this", off: v.offset } : { k: "frame", off: v.offset };
		const writable = !v.constant;
		return { type: v.type, base, offset: v.storage === "global" ? 0 : 0, dyn: [], writable, why: v.constant ? `'${v.name}' is a constant` : undefined };
	}

	/** An lvalue (a variable, a field, an element), or null after an error. */
	private lvalue(e: A.Expr): LV | null {
		try {
			return this.lvalueOrThrow(e);
		} catch (x) {
			if (!(x instanceof CompileError)) throw x;
			return null;
		}
	}

	private lvalueOrThrow(e: A.Expr): LV {
		switch (e.kind) {
			case "name": {
				const v = this.lookupVar(e.key);
				if (!v) {
					if (this.pous.has(e.key)) this.fatal(e.span, `'${e.name}' is a POU, not a variable`);
					this.fatal(e.span, `unknown variable '${e.name}'`);
				}
				return this.varLV(v);
			}
			case "member": {
				const o = this.lvalueOrThrow(e.object);
				const t = o.type;
				if (t.k === "struct") {
					const f = t.fields.find((x) => x.key === e.key);
					if (!f) this.fatal(e.span, `${t.name} has no field '${e.name}'`);
					return { ...o, type: f.type, offset: o.offset + f.offset };
				}
				if (t.k === "fb") {
					const f = t.layout.fields.find((x) => x.key === e.key);
					const fb = this.pous.get(t.layout.key)!;
					const v = fb.vars.get(e.key);
					if (!f || !v) this.fatal(e.span, `${t.layout.name} has no parameter '${e.name}'`);
					if (v.cls === "VAR") this.fatal(e.span, `'${e.name}' is an internal variable of ${t.layout.name}`);
					if (v.cls === "VAR_IN_OUT") this.fatal(e.span, `the VAR_IN_OUT '${e.name}' cannot be accessed from outside`);
					const writable = o.writable && v.cls === "VAR_INPUT";
					return { ...o, type: f.type, offset: o.offset + f.offset, writable, why: v.cls === "VAR_OUTPUT" ? `'${e.name}' is an output of ${t.layout.name}` : o.why };
				}
				this.fatal(e.span, `${typeName(t)} has no fields`);
			}
			case "index": {
				const o = this.lvalueOrThrow(e.object);
				const t = o.type;
				if (t.k !== "array") this.fatal(e.span, `${typeName(t)} is not an array`);
				if (e.indices.length !== t.dims.length) this.fatal(e.span, `the array has ${t.dims.length} dimension(s)`);
				let offset = o.offset;
				const dyn = [...o.dyn];
				let stride = sizeOf(t.elem);
				const strides: number[] = [];
				for (let i = t.dims.length - 1; i >= 0; i--) {
					strides[i] = stride;
					stride *= t.dims[i].hi - t.dims[i].lo + 1;
				}
				e.indices.forEach((ix, i) => {
					const d = t.dims[i];
					const c = this.constEval(ix, null);
					if (c?.k === "int") {
						if (c.v < BigInt(d.lo) || c.v > BigInt(d.hi)) this.fatal(ix.span, `index ${c.v} outside ${d.lo}..${d.hi}`);
						offset += (Number(c.v) - d.lo) * strides[i];
						return;
					}
					const x = this.check(ix, T.DINT);
					const xe = elemOf(x.type);
					if (x.type.k !== "error" && (xe === null || !isInteger(xe) || x.type.k === "enum")) {
						this.fatal(ix.span, `an array index is an integer, not ${typeName(x.type)}`);
					}
					dyn.push({ index: x, lo: d.lo, hi: d.hi, stride: strides[i] });
				});
				return { ...o, type: t.elem, offset, dyn };
			}
			default:
				this.fatal(e.span, "expected a variable");
		}
	}

	// ---- expressions: checking ----------------------------------------------------------------------
	private isUntypedLiteral(e: A.Expr): boolean {
		return (e.kind === "int" || e.kind === "real") && e.type === null;
	}

	/** Type-checks `e`; `expected` guides untyped literals and enum value names. */
	private check(e: A.Expr, expected: Type | null): TExpr {
		try {
			return this.checkOrThrow(e, expected);
		} catch (x) {
			if (!(x instanceof CompileError)) throw x;
			return ERR;
		}
	}

	private checkOrThrow(e: A.Expr, expected: Type | null): TExpr {
		switch (e.kind) {
			case "int": {
				let t: Type | null = e.type ? T[e.type as ElemName] : null;
				if (!t) {
					const ex = expected && elemOf(expected);
					if (expected && ex && expected.k !== "enum" && (isInteger(ex) || ELEM[ex].category === "bit")) {
						if (!this.fits(e.value, T[ex])) this.fatal(e.span, `${e.value} does not fit ${ex}`);
						t = T[ex];
					} else if (ex && isReal(ex)) {
						return { k: "const", type: T[ex], raw: realRaw(ex, Number(e.value)) };
					} else t = e.value >= -(1n << 31n) && e.value < 1n << 31n ? T.DINT : e.value >= -(1n << 63n) && e.value < 1n << 63n ? T.LINT : T.ULINT;
				}
				const el = elemOf(t)!;
				if (isReal(el)) return { k: "const", type: t, raw: realRaw(el, Number(e.value)) };
				if (!this.fits(e.value, t)) this.fatal(e.span, `${e.value} does not fit ${typeName(t)}`);
				return { k: "const", type: t, raw: toRaw(el, e.value) };
			}
			case "real": {
				const ex = expected && elemOf(expected);
				const el: ElemName = (e.type as ElemName | null) ?? (ex && isReal(ex) ? ex : "LREAL");
				if (el === "REAL" && Number.isFinite(e.value) && !Number.isFinite(Math.fround(e.value))) this.fatal(e.span, `${e.text} does not fit REAL`);
				return { k: "const", type: T[el], raw: realRaw(el, e.value) };
			}
			case "bool": return { k: "const", type: T.BOOL, raw: e.value ? 1 : 0 };
			case "time":
				if (e.ms < -(2 ** 31) || e.ms >= 2 ** 31) this.fatal(e.span, "TIME is limited to +/-24d20h31m23s647ms");
				return { k: "const", type: T.TIME, raw: e.ms >>> 0 };
			case "string": this.fatal(e.span, "STRING is not supported yet");
			case "enumLit": {
				const c = this.constEval(e, null);
				if (c?.k !== "enum") this.fatal(e.span, `unknown enumeration value '${e.type}#${e.name}'`);
				return { k: "const", type: c.type, raw: toRaw(elemOf(c.type)!, c.v) };
			}
			case "name": {
				const v = this.lookupVar(e.key);
				if (v) {
					const lv = this.varLV(v);
					return isComposite(v.type) || v.type.k === "fb" ? { k: "lvalue", type: v.type, lv } : { k: "load", type: v.type, lv };
				}
				if (expected?.k === "enum" && expected.values.has(e.key)) {
					return { k: "const", type: expected, raw: toRaw(expected.base, expected.values.get(e.key)!.value) };
				}
				const enums = this.enumValues.get(e.key);
				if (enums && enums.length === 1) {
					const t = enums[0] as Extract<Type, { k: "enum" }>;
					return { k: "const", type: t, raw: toRaw(t.base, t.values.get(e.key)!.value) };
				}
				if (enums && enums.length > 1) this.fatal(e.span, `'${e.name}' is an element of several enumerations: write Type#${e.name}`);
				if (this.pous.has(e.key)) this.fatal(e.span, `'${e.name}' is a POU: call it with ()`);
				this.fatal(e.span, `unknown name '${e.name}'`);
			}
			case "member":
			case "index": {
				const lv = this.lvalueOrThrow(e);
				return isComposite(lv.type) || lv.type.k === "fb" ? { k: "lvalue", type: lv.type, lv } : { k: "load", type: lv.type, lv };
			}
			case "unary": {
				if (e.op === "+") return this.check(e.operand, expected);
				const x = this.scalar(this.check(e.operand, expected), e.operand.span);
				if (x.type.k === "error") return ERR;
				if (e.op === "NOT") return this.unaryNot(x, e.span);
				const el = elemOf(x.type)!;
				const cat = ELEM[el].category;
				if (!((cat === "int" || cat === "real" || cat === "time") && x.type.k !== "enum")) {
					this.fatal(e.span, `'-' needs a signed number or TIME, not ${typeName(x.type)}`);
				}
				return { k: "unary", type: T[el], op: "-", e: x };
			}
			case "binary": return this.checkBinary(e, expected);
			case "call": return this.checkCall(e, expected);
		}
	}

	private unaryNot(x: TExpr, span: Span): TExpr {
		const el = elemOf(x.type);
		if (x.type.k === "error") return ERR;
		if (el === null || x.type.k === "enum" || !isBitLike(el)) this.fatal(span, `NOT needs BOOL or a bit string, not ${typeName(x.type)}`);
		return { k: "unary", type: T[el], op: "NOT", e: x };
	}

	/** Only elementary values take part in expressions. */
	private scalar(x: TExpr, span: Span): TExpr {
		if (x.k === "lvalue") this.fatal(span, `a ${typeName(x.type)} cannot be used in an expression`);
		return x;
	}

	private checkBinary(e: Extract<A.Expr, { kind: "binary" }>, expected: Type | null): TExpr {
		const op = e.op;
		const compare = ["<", ">", "<=", ">=", "=", "<>"].includes(op);
		const expectOperand = compare ? null : expected;
		let l: TExpr;
		let r: TExpr;
		if (this.isUntypedLiteral(e.left) && !this.isUntypedLiteral(e.right)) {
			r = this.scalar(this.check(e.right, expectOperand), e.right.span);
			l = this.scalar(this.check(e.left, op === "**" ? T.LREAL : r.type.k === "error" ? expectOperand : r.type), e.left.span);
		} else {
			l = this.scalar(this.check(e.left, expectOperand), e.left.span);
			const rightExpected = op === "**" ? T.LREAL : (l.type.k === "error" ? expectOperand : (l.type.k === "elem" && l.type.name === "TIME" && (op === "*" || op === "/") ? T.DINT : l.type));
			r = this.scalar(this.check(e.right, rightExpected), e.right.span);
		}
		if (l.type.k === "error" || r.type.k === "error") return ERR;
		const le = elemOf(l.type);
		const re = elemOf(r.type);
		if (le === null || re === null) this.fatal(e.span, `'${op}' cannot be applied to ${typeName(l.type)} and ${typeName(r.type)}`);

		// TIME arithmetic
		if (le === "TIME" || re === "TIME") {
			if ((op === "+" || op === "-") && le === "TIME" && re === "TIME") return { k: "binary", type: T.TIME, op, l, r, opType: T.TIME };
			if ((op === "*" || op === "/") && le === "TIME" && isInteger(re)) {
				return { k: "binary", type: T.TIME, op, l, r: this.convertTo(r, T.DINT, e.right.span), opType: T.DINT };
			}
			if (op === "*" && re === "TIME" && isInteger(le)) {
				return { k: "binary", type: T.TIME, op, l: this.convertTo(l, T.DINT, e.left.span), r, opType: T.DINT };
			}
		}
		if (op === "**") {
			if (!isReal(le)) this.fatal(e.left.span, `the base of ** is REAL or LREAL, not ${typeName(l.type)}`);
			if (!isNumeric(re)) this.fatal(e.right.span, `the exponent of ** is a number, not ${typeName(r.type)}`);
			this.natives.add(Native.EXPT_F64);
			return { k: "intrinsic", type: T[le], name: "EXPT", args: [l, r], opType: T[le] };
		}

		// the operands get a common type: the one the other converts to implicitly
		let common: Type;
		if (sameType(l.type, r.type)) common = l.type;
		else if (l.type.k !== "enum" && r.type.k !== "enum" && implicitElem(le, re)) common = T[re];
		else if (l.type.k !== "enum" && r.type.k !== "enum" && implicitElem(re, le)) common = T[le];
		else this.fatal(e.span, `'${op}': ${typeName(l.type)} and ${typeName(r.type)} do not fit together`);
		const ce = elemOf(common)!;
		const cat = ELEM[ce].category;
		const L = sameType(l.type, common) ? l : this.convertTo(l, T[ce], e.left.span);
		const R = sameType(r.type, common) ? r : this.convertTo(r, T[ce], e.right.span);
		const opType = T[ce];
		if (compare) {
			if (common.k === "enum" && op !== "=" && op !== "<>") this.fatal(e.span, "enumeration values are compared with = and <> only");
			return { k: "binary", type: T.BOOL, op, l: L, r: R, opType };
		}
		if (common.k === "enum") this.fatal(e.span, `'${op}' cannot be applied to an enumeration`);
		switch (op) {
			case "AND":
			case "OR":
			case "XOR":
				if (!isBitLike(ce)) this.fatal(e.span, `${op} needs BOOL or bit strings, not ${typeName(common)}`);
				return { k: "binary", type: opType, op, l: L, r: R, opType };
			case "MOD":
				if (!isInteger(ce)) this.fatal(e.span, `MOD needs integers, not ${typeName(common)}`);
				return { k: "binary", type: opType, op, l: L, r: R, opType };
			default:
				if (!(cat === "int" || cat === "uint" || cat === "real")) this.fatal(e.span, `'${op}' needs numbers, not ${typeName(common)}`);
				return { k: "binary", type: opType, op, l: L, r: R, opType };
		}
	}

	/** Implicit conversion of `x` to `to` (assignment, parameter); error if not allowed. */
	private convertTo(x: TExpr, to: Type, span: Span): TExpr {
		if (x.type.k === "error" || to.k === "error") return x;
		if (x.k === "lvalue") this.fatal(span, `a ${typeName(x.type)} cannot be used here`);
		if (sameType(x.type, to)) return x;
		if (!assignable(x.type, to)) {
			const hint = x.type.k === "elem" && to.k === "elem" ? `: use ${x.type.name}_TO_${to.name}` : "";
			this.fatal(span, `cannot convert ${typeName(x.type)} to ${typeName(to)} implicitly${hint}`);
		}
		// constant into a subrange: checked now
		if (to.k === "subrange" && x.k === "const") {
			const v = rawToBig(elemOf(x.type)!, x.raw);
			if (v < to.lo || v > to.hi) this.fatal(span, `${v} is outside ${typeName(to)}`);
			return { ...x, type: to };
		}
		return { k: "conv", type: to, from: x.type, e: x, checked: to.k === "subrange" };
	}

	private checkCall(c: A.CallExpr, expected: Type | null): TExpr {
		if (c.callee.kind !== "name") this.fatal(c.callee.span, "only functions can be called in an expression");
		const key = c.callee.key;
		const fn = this.pous.get(key);
		if (fn) {
			if (fn.kind !== "FUNCTION") this.fatal(c.span, `${fn.kind} '${fn.name}' cannot be called in an expression`);
			return this.checkFunctionCall(fn, c);
		}
		const v = this.lookupVar(key);
		if (v?.type.k === "fb") this.fatal(c.span, "a function block call is a statement, not an expression");
		return this.checkIntrinsic(key, c, expected);
	}

	private checkFunctionCall(fn: PouInfo, c: A.CallExpr): TExpr {
		this.layoutPou(fn);
		this.pou!.callees.add(fn.key);
		const formal = c.args.some((a) => a.key !== null);
		if (formal && c.args.some((a) => a.key === null)) this.fatal(c.span, "either all parameters are formal (name := value) or none");
		const args: { param: VarInfo; value: TExpr | null; lv: LV | null }[] = [];
		const given = new Map<string, A.CallArg>();
		if (formal) {
			for (const a of c.args) {
				if (a.output) this.fatal(a.span, "functions have no outputs (yet)");
				if (!fn.params.some((p) => p.key === a.key)) this.fatal(a.span, `${fn.name} has no input '${a.name}'`);
				if (given.has(a.key!)) this.fatal(a.span, `'${a.name}' is given twice`);
				given.set(a.key!, a);
			}
		} else {
			if (c.args.length !== fn.params.length) this.fatal(c.span, `${fn.name} has ${fn.params.length} parameter(s), not ${c.args.length}`);
			fn.params.forEach((p, i) => given.set(p.key, c.args[i]));
		}
		for (const p of fn.params) {
			const a = given.get(p.key);
			if (p.byRef) {
				if (!a) this.fatal(c.span, `the VAR_IN_OUT parameter '${p.name}' must be given`);
				const lv = this.lvalue(a.value);
				if (!lv) return ERR;
				if (!sameType(lv.type, p.type)) this.fatal(a.value.span, `'${p.name}' needs a ${typeName(p.type)}, not ${typeName(lv.type)}`);
				if (!lv.writable) this.fatal(a.value.span, "a VAR_IN_OUT parameter needs a writable variable");
				args.push({ param: p, value: null, lv });
			} else if (isComposite(p.type)) {
				if (!a) this.fatal(c.span, `the parameter '${p.name}' must be given`);
				const lv = this.lvalue(a.value);
				if (!lv) return ERR;
				if (!sameType(lv.type, p.type)) this.fatal(a.value.span, `'${p.name}' needs a ${typeName(p.type)}, not ${typeName(lv.type)}`);
				args.push({ param: p, value: null, lv });
			} else if (a) {
				args.push({ param: p, value: this.convertTo(this.check(a.value, p.type), p.type, a.value.span), lv: null });
			} else {
				// an omitted input: its initial value
				const el = elemOf(p.type)!;
				let raw: number | bigint = 0;
				if (p.init?.kind === "expr") raw = this.constFor(p.init.expr, p.type) ?? 0;
				else if (p.type.k === "enum") raw = toRaw(el, p.type.init);
				args.push({ param: p, value: { k: "const", type: p.type, raw }, lv: null });
			}
		}
		return { k: "call", type: fn.returnType ?? ERROR, fn, args };
	}

	private checkIntrinsic(key: string, c: A.CallExpr, expected: Type | null): TExpr {
		const span = c.span;
		const positional = () => {
			if (c.args.some((a) => a.key !== null && !/^IN\d*$|^G$|^K$|^MN$|^MX$|^N$/.test(a.key))) this.fatal(span, `${key} takes positional parameters`);
			return c.args.map((a) => a.value);
		};
		const argc = (n: number) => {
			if (c.args.length !== n) this.fatal(span, `${key} takes ${n} parameter(s)`);
		};
		const conv = CONV_RE.exec(key);
		if (conv) {
			const [, fromName, toName] = conv;
			if (!isElemName(fromName) || !isElemName(toName)) this.fatal(c.callee.span, `unknown function '${key}'`);
			argc(1);
			const x = this.scalar(this.check(positional()[0], T[fromName]), span);
			if (x.type.k === "error") return ERR;
			if (elemOf(x.type) !== fromName || x.type.k === "enum") this.fatal(c.args[0].span, `${key} needs a ${fromName}, not ${typeName(x.type)}`);
			const fc = ELEM[fromName].category;
			const tc = ELEM[toName].category;
			if ((fc === "bit" && tc === "real") || (fc === "real" && tc === "bit")) this.fatal(span, `${key}: conversions between bit strings and REAL are not supported`);
			if ((fc === "real" || tc === "real") && (fc === "bool" || tc === "bool")) this.fatal(span, `${key} is not a valid conversion`);
			return { k: "conv", type: T[toName], from: T[fromName], e: { ...x, type: T[fromName] }, checked: true };
		}
		if (key === "TRUNC") {
			argc(1);
			const x = this.scalar(this.check(positional()[0], null), span);
			const el = elemOf(x.type);
			if (x.type.k !== "error" && (el === null || !isReal(el))) this.fatal(span, "TRUNC needs REAL or LREAL");
			return { k: "conv", type: T.DINT, from: x.type, e: x, checked: true, trunc: true };
		}
		if (key === "__PLC_TIME") {
			argc(0);
			this.natives.add(Native.TIME_MS);
			return { k: "intrinsic", type: T.TIME, name: key, args: [], opType: T.TIME };
		}
		if (key === "ABS" || key === "SQRT" || key in REAL_NATIVES) {
			argc(1);
			const x = this.scalar(this.check(positional()[0], expected), span);
			const el = elemOf(x.type);
			if (x.type.k === "error") return ERR;
			if (key === "ABS") {
				if (el === null || x.type.k === "enum" || !isNumeric(el)) this.fatal(span, `ABS needs a number, not ${typeName(x.type)}`);
				const n = ELEM[el].slot === "I64" ? Native.ABS_I64 : ELEM[el].slot === "I32" ? Native.ABS_I32 : ELEM[el].slot === "F32" ? Native.ABS_F32 : Native.ABS_F64;
				if (ELEM[el].category !== "uint") this.natives.add(n);
				return { k: "intrinsic", type: T[el], name: key, args: [x], opType: T[el] };
			}
			let arg = x;
			let rt: ElemName = el !== null && isReal(el) ? el : "LREAL";
			if (el === null || !isReal(el)) {
				if (el !== null && isInteger(el) && x.type.k !== "enum") arg = this.convertTo(x, T.LREAL, span);
				else this.fatal(span, `${key} needs REAL or LREAL, not ${typeName(x.type)}`);
				rt = "LREAL";
			}
			this.natives.add(key === "SQRT" ? (rt === "REAL" ? Native.SQRT_F32 : Native.SQRT_F64) : REAL_NATIVES[key]);
			return { k: "intrinsic", type: T[rt], name: key, args: [arg], opType: T[rt] };
		}
		if (key === "EXPT") {
			argc(2);
			const [a, b] = positional();
			return this.checkBinary({ kind: "binary", op: "**", left: a, right: b, span }, expected);
		}
		if (key === "MIN" || key === "MAX" || key === "LIMIT") {
			const xs = positional();
			if (key === "LIMIT") argc(3);
			else if (xs.length < 2) this.fatal(span, `${key} takes at least 2 parameters`);
			const typed = this.unify(xs, expected, span);
			const t = typed[0].type;
			const el = elemOf(t);
			if (t.k === "error") return ERR;
			if (el === null || t.k === "enum" || el === "BOOL") this.fatal(span, `${key} needs numbers, bit strings or TIME, not ${typeName(t)}`);
			if (key === "LIMIT") {
				const [mn, x, mx] = typed;
				const inner: TExpr = { k: "intrinsic", type: T[el], name: "MAX", args: [x, mn], opType: T[el] };
				return { k: "intrinsic", type: T[el], name: "MIN", args: [inner, mx], opType: T[el] };
			}
			return { k: "intrinsic", type: T[el], name: key, args: typed, opType: T[el] };
		}
		if (key === "SEL") {
			argc(3);
			const [g, a, b] = positional();
			const G = this.convertTo(this.check(g, T.BOOL), T.BOOL, g.span);
			const typed = this.unify([a, b], expected, span);
			if (typed[0].type.k === "error") return ERR;
			return { k: "intrinsic", type: typed[0].type, name: key, args: [G, ...typed], opType: typed[0].type };
		}
		if (key === "MUX") {
			const xs = positional();
			if (xs.length < 2) this.fatal(span, "MUX takes K and at least one input");
			const k = this.scalar(this.check(xs[0], T.DINT), xs[0].span);
			const ke = elemOf(k.type);
			if (k.type.k !== "error" && (ke === null || !isInteger(ke))) this.fatal(xs[0].span, "the selector K of MUX is an integer");
			const K = k.type.k === "error" ? k : this.convertTo(k, isInteger(ke!) && ELEM[ke!].slot === "I64" ? T.LINT : T.DINT, xs[0].span);
			const typed = this.unify(xs.slice(1), expected, span);
			if (typed[0].type.k === "error") return ERR;
			return { k: "intrinsic", type: typed[0].type, name: key, args: [K, ...typed], opType: typed[0].type };
		}
		if (key === "MOVE") {
			argc(1);
			return this.scalar(this.check(positional()[0], expected), span);
		}
		if (key === "SHL" || key === "SHR" || key === "ROL" || key === "ROR") {
			argc(2);
			const [a, n] = positional();
			const x = this.scalar(this.check(a, expected), a.span);
			const el = elemOf(x.type);
			if (x.type.k === "error") return ERR;
			if (el === null || x.type.k === "enum" || ELEM[el].category !== "bit") this.fatal(a.span, `${key} needs a bit string (BYTE, WORD, DWORD, LWORD), not ${typeName(x.type)}`);
			const N = this.scalar(this.check(n, T.UDINT), n.span);
			const ne = elemOf(N.type);
			if (N.type.k !== "error" && (ne === null || !isInteger(ne))) this.fatal(n.span, "the shift count N is an integer");
			if (key === "ROL" || key === "ROR") {
				const bits = ELEM[el].size * 8;
				const native = ({ ROL: { 8: Native.ROL_8, 16: Native.ROL_16, 32: Native.ROL_32, 64: Native.ROL_64 }, ROR: { 8: Native.ROR_8, 16: Native.ROR_16, 32: Native.ROR_32, 64: Native.ROR_64 } } as const)[key][bits as 8 | 16 | 32 | 64];
				this.natives.add(native);
			}
			return { k: "intrinsic", type: T[el], name: key, args: [x, N], opType: T[el] };
		}
		const arith: Record<string, A.BinaryOp> = { ADD: "+", SUB: "-", MUL: "*", DIV: "/", MOD: "MOD", GT: ">", GE: ">=", EQ: "=", LE: "<=", LT: "<", NE: "<>" };
		if (key in arith) {
			const xs = positional();
			const nary = key === "ADD" || key === "MUL";
			if (nary ? xs.length < 2 : xs.length !== 2) this.fatal(span, `${key} takes ${nary ? "at least 2" : "2"} parameters`);
			let e: A.Expr = xs[0];
			for (const x of xs.slice(1)) e = { kind: "binary", op: arith[key], left: e, right: x, span };
			return this.check(e, expected);
		}
		this.fatal(c.callee.span, `unknown function '${(c.callee as { name: string }).name}'`);
	}

	/** Several values of one common type (MIN, MAX, SEL, MUX). */
	private unify(xs: A.Expr[], expected: Type | null, span: Span): TExpr[] {
		const typed: TExpr[] = [];
		let common: Type | null = null;
		for (const x of xs) {
			if (this.isUntypedLiteral(x)) continue;
			const t = this.scalar(this.check(x, expected), x.span);
			if (t.type.k === "error") return [ERR];
			if (common === null || assignable(common, t.type)) common = t.type.k === "subrange" ? T[t.type.base] : t.type;
			else if (!assignable(t.type, common)) this.fatal(x.span, `${typeName(t.type)} does not fit ${typeName(common)}`);
		}
		const target = common ?? (expected && elemOf(expected) ? T[elemOf(expected)!] : T.DINT);
		for (const x of xs) typed.push(this.convertTo(this.scalar(this.check(x, target), x.span), target, x.span ?? span));
		return typed;
	}

	// ---- emission ------------------------------------------------------------------------------------
	private pushRaw(e: ElemName, raw: number | bigint): void {
		if (ELEM[e].slot === "I64" || ELEM[e].slot === "F64") this.emitter.pushI64(BigInt(raw));
		else this.emitter.pushI32(Number(raw));
	}

	/** LD{G,L,T}_x by the element type; `base`: the U8 opcode of the mode (LDG_U8, LDL_U8, LDT_U8). */
	private loadDirect(base: Op, off: number, e: ElemName): void {
		this.emitter.opArg(base + loadIndex(e), off);
	}

	private storeFrame(off: number, e: ElemName): void {
		this.emitter.opArg(Op.STL_8 + storeIndex(e), off);
	}

	private emitAddr(lv: LV): void {
		const em = this.emitter;
		const b = lv.base;
		switch (b.k) {
			case "global": em.opArg(Op.ADDR_G, b.addr + lv.offset); break;
			case "this":
			case "frame": {
				const off = b.off + lv.offset;
				if (off <= MAX_FRAME_OFFSET) em.opArg(b.k === "this" ? Op.ADDR_T : Op.ADDR_L, off);
				else {
					em.opArg(b.k === "this" ? Op.ADDR_T : Op.ADDR_L, 0);
					em.pushI32(off);
					em.op(Op.ADD_I32);
				}
				break;
			}
			case "ptrThis":
			case "ptrFrame":
				em.opArg(b.k === "ptrThis" ? Op.LDT_32 : Op.LDL_32, b.off);
				if (lv.offset !== 0) {
					em.pushI32(lv.offset);
					em.op(Op.ADD_I32);
				}
				break;
		}
		for (const d of lv.dyn) {
			this.emitIndex(d.index, d.lo, d.hi);
			if (d.lo !== 0) {
				em.pushI32(d.lo);
				em.op(Op.SUB_I32);
			}
			if (d.stride !== 1) {
				em.pushI32(d.stride);
				em.op(Op.MUL_I32);
			}
			em.op(Op.ADD_I32);
		}
	}

	/** An array index as a DINT, checked against the bounds. */
	private emitIndex(x: TExpr, lo: number, hi: number): void {
		this.emitExpr(x);
		const e = elemOf(x.type) ?? "DINT";
		const em = this.emitter;
		if (ELEM[e].slot === "I64") {
			em.op(ELEM[e].signed ? Op.CHK_I64 : Op.CHK_U64);
			em.u64(BigInt(ELEM[e].signed ? lo : Math.max(lo, 0)));
			em.u64(BigInt(hi));
			em.op(Op.I64_TO_I32);
			return;
		}
		if (ELEM[e].signed) {
			em.op(Op.CHK_I32);
			em.u32(lo >>> 0);
			em.u32(hi >>> 0);
		} else {
			if (hi < 0) {
				em.op(Op.TRAP);
				em.u16(0);
				return;
			}
			em.op(Op.CHK_U32);
			em.u32(Math.max(lo, 0));
			em.u32(hi);
		}
	}

	private direct(lv: LV): { mode: "G" | "L" | "T"; off: number } | null {
		if (lv.dyn.length > 0) return null;
		const b = lv.base;
		if (b.k === "global") return { mode: "G", off: b.addr + lv.offset };
		if ((b.k === "this" || b.k === "frame") && b.off + lv.offset <= MAX_FRAME_OFFSET) return { mode: b.k === "this" ? "T" : "L", off: b.off + lv.offset };
		return null;
	}

	private emitLoad(lv: LV): void {
		const e = elemOf(lv.type) ?? "DWORD";
		const d = this.direct(lv);
		if (d) this.loadDirect(d.mode === "G" ? Op.LDG_U8 : d.mode === "L" ? Op.LDL_U8 : Op.LDT_U8, d.off, e);
		else {
			this.emitAddr(lv);
			this.emitter.op(Op.LDI_U8 + loadIndex(e));
		}
	}

	private emitStore(lv: LV, value: () => void): void {
		const e = elemOf(lv.type) ?? "DWORD";
		const d = this.direct(lv);
		if (d) {
			value();
			this.emitter.opArg((d.mode === "G" ? Op.STG_8 : d.mode === "L" ? Op.STL_8 : Op.STT_8) + storeIndex(e), d.off);
		} else {
			this.emitAddr(lv);
			value();
			this.emitter.op(Op.STI_8 + storeIndex(e));
		}
	}

	/** After arithmetic on a narrow type: back into its range (wraps). */
	private normalize(e: ElemName): void {
		const em = this.emitter;
		const info = ELEM[e];
		if (info.size === 1 && e !== "BOOL") em.op(info.signed ? Op.SEXT8 : Op.ZEXT8);
		else if (info.size === 2) em.op(info.signed ? Op.SEXT16 : Op.ZEXT16);
	}

	private emitExpr(x: TExpr): void {
		const em = this.emitter;
		switch (x.k) {
			case "error": return;
			case "lvalue": throw new Error("internal: composite value in an expression");
			case "const": this.pushRaw(elemOf(x.type)!, x.raw); return;
			case "load": this.emitLoad(x.lv); return;
			case "conv":
				this.emitExpr(x.e);
				if (x.type.k === "subrange") {
					this.emitConv(elemOf(x.from)!, x.type.base, true, false);
					this.emitCheck(x.type.base, x.type.lo, x.type.hi);
				} else this.emitConv(elemOf(x.from)!, elemOf(x.type)!, x.checked, x.trunc ?? false);
				return;
			case "unary": {
				this.emitExpr(x.e);
				const e = elemOf(x.type)!;
				const s = ELEM[e].slot;
				if (x.op === "NOT") {
					if (e === "BOOL") {
						em.pushI32(1);
						em.op(Op.XOR_I32);
					} else {
						em.op(s === "I64" ? Op.NOT_I64 : Op.NOT_I32);
						this.normalize(e);
					}
				} else {
					em.op(s === "I64" ? Op.NEG_I64 : s === "F32" ? Op.NEG_F32 : s === "F64" ? Op.NEG_F64 : Op.NEG_I32);
					this.normalize(e);
				}
				return;
			}
			case "binary": {
				this.emitExpr(x.l);
				this.emitExpr(x.r);
				const e = elemOf(x.opType)!;
				em.op(binaryOp(x.op, e));
				if (!["<", ">", "<=", ">=", "=", "<>"].includes(x.op)) this.normalize(elemOf(x.type)!);
				return;
			}
			case "call": {
				for (const a of x.args) {
					if (a.lv) this.emitAddr(a.lv);
					else this.emitExpr(a.value!);
				}
				em.call(Op.CALL, x.fn.key);
				return;
			}
			case "intrinsic": return this.emitIntrinsic(x);
		}
	}

	private emitCheck(e: ElemName, lo: bigint, hi: bigint): void {
		const em = this.emitter;
		const info = ELEM[e];
		if (info.slot === "I64") {
			em.op(info.signed ? Op.CHK_I64 : Op.CHK_U64);
			em.u64(lo);
			em.u64(hi);
		} else {
			em.op(info.signed ? Op.CHK_I32 : Op.CHK_U32);
			em.u32(Number(BigInt.asUintN(32, lo)));
			em.u32(Number(BigInt.asUintN(32, hi)));
		}
	}

	/** Converts the slot on the stack from `from` to `to` (rtoos/docs/plc.md "Values and types"). */
	private emitConv(from: ElemName, to: ElemName, checked: boolean, trunc: boolean): void {
		if (from === to) return;
		const em = this.emitter;
		const f = ELEM[from];
		const t = ELEM[to];
		if (to === "BOOL") {
			if (f.slot === "I32") {
				em.pushI32(0);
				em.op(Op.NE_I32);
			} else if (f.slot === "I64") {
				em.pushI64(0n);
				em.op(Op.NE_I64);
			} else if (f.slot === "F32") {
				em.pushI32(0);
				em.op(Op.NE_F32);
			} else {
				em.pushI64(0n);
				em.op(Op.NE_F64);
			}
			return;
		}
		if (t.category === "real") {
			if (f.category === "real") {
				em.op(f.slot === "F32" ? Op.F32_TO_F64 : Op.F64_TO_F32);
				return;
			}
			const signed = f.signed && f.category !== "bit";
			const op = f.slot === "I64" ? (signed ? (to === "REAL" ? Op.I64S_TO_F32 : Op.I64S_TO_F64) : (to === "REAL" ? Op.I64U_TO_F32 : Op.I64U_TO_F64))
				: (signed ? (to === "REAL" ? Op.I32S_TO_F32 : Op.I32S_TO_F64) : (to === "REAL" ? Op.I32U_TO_F32 : Op.I32U_TO_F64));
			em.op(op);
			return;
		}
		if (f.category === "real") {
			em.op(trunc ? (f.slot === "F32" ? Op.F32_TRUNC_I64 : Op.F64_TRUNC_I64) : (f.slot === "F32" ? Op.F32_ROUND_I64 : Op.F64_ROUND_I64));
			// now an i64; LINT needs no check (the conversion traps outside i64)
			if (to !== "LINT" && to !== "LWORD") this.emitCheck("LINT", t.category === "bit" ? 0n : t.min, t.max > (1n << 63n) - 1n ? (1n << 63n) - 1n : t.max);
			if (t.slot === "I32") em.op(Op.I64_TO_I32);
			return;
		}
		const binary = f.category === "bit" || t.category === "bit";
		if (binary) {
			// bit pattern transfer: the source's bits, cut or zero-extended to the target
			if (f.size < 4 && f.signed) em.op(f.size === 1 ? Op.ZEXT8 : Op.ZEXT16);
			if (f.slot === "I32" && t.slot === "I64") em.op(Op.I32U_TO_I64);
			if (f.slot === "I64" && t.slot === "I32") em.op(Op.I64_TO_I32);
			if (t.size < 4 && t.size < f.size) {
				if (t.size === 1) em.op(t.signed ? Op.SEXT8 : Op.ZEXT8);
				else em.op(t.signed ? Op.SEXT16 : Op.ZEXT16);
			} else if (t.size < 4 && t.signed && t.size <= f.size) {
				em.op(t.size === 1 ? Op.SEXT8 : Op.SEXT16);
			}
			return;
		}
		// numeric (int, uint, time, bool as 0/1): checked when the target cannot hold every source value
		const fmin = from === "BOOL" ? 0n : f.min;
		const fmax = from === "BOOL" ? 1n : f.max;
		if (checked && (fmin < t.min || fmax > t.max)) {
			this.emitCheck(from, fmin > t.min ? fmin : t.min, fmax < t.max ? fmax : t.max);
		}
		if (f.slot === "I32" && t.slot === "I64") em.op(f.signed ? Op.I32S_TO_I64 : Op.I32U_TO_I64);
		else if (f.slot === "I64" && t.slot === "I32") em.op(Op.I64_TO_I32);
		if (!checked) this.normalize(to);
	}

	private emitIntrinsic(x: Extract<TExpr, { k: "intrinsic" }>): void {
		const em = this.emitter;
		const e = elemOf(x.opType)!;
		const slot = ELEM[e].slot;
		const tempSize = slot === "I64" || slot === "F64" ? 8 : 4;
		switch (x.name) {
			case "__PLC_TIME":
				em.opArg(Op.SYSCALL, Native.TIME_MS);
				return;
			case "ABS": {
				this.emitExpr(x.args[0]);
				if (ELEM[e].category === "uint") return;
				em.opArg(Op.SYSCALL, slot === "I64" ? Native.ABS_I64 : slot === "I32" ? Native.ABS_I32 : slot === "F32" ? Native.ABS_F32 : Native.ABS_F64);
				this.normalize(e);
				return;
			}
			case "SQRT":
				this.emitExpr(x.args[0]);
				em.opArg(Op.SYSCALL, e === "REAL" ? Native.SQRT_F32 : Native.SQRT_F64);
				return;
			case "EXPT": {
				const [a, b] = x.args;
				this.emitExpr(a);
				if (e === "REAL") em.op(Op.F32_TO_F64);
				this.emitExpr(b);
				this.emitConv(elemOf(b.type)!, "LREAL", false, false);
				em.opArg(Op.SYSCALL, Native.EXPT_F64);
				if (e === "REAL") em.op(Op.F64_TO_F32);
				return;
			}
			case "MIN":
			case "MAX": {
				const acc = this.hiddenTemp(tempSize);
				const cur = this.hiddenTemp(tempSize);
				this.emitExpr(x.args[0]);
				this.storeFrame(acc, e);
				for (const a of x.args.slice(1)) {
					const skip = em.label();
					this.emitExpr(a);
					this.storeFrame(cur, e);
					this.loadDirect(Op.LDL_U8, cur, e);
					this.loadDirect(Op.LDL_U8, acc, e);
					em.op(compareOp(x.name === "MIN" ? "<" : ">", e));
					em.jump(Op.JZ, skip);
					this.loadDirect(Op.LDL_U8, cur, e);
					this.storeFrame(acc, e);
					em.bind(skip);
				}
				this.loadDirect(Op.LDL_U8, acc, e);
				return;
			}
			case "SEL": {
				const one = em.label();
				const end = em.label();
				this.emitExpr(x.args[0]);
				em.jump(Op.JNZ, one);
				this.emitExpr(x.args[1]);
				em.jump(Op.JMP, end);
				em.bind(one);
				this.emitExpr(x.args[2]);
				em.bind(end);
				return;
			}
			case "MUX": {
				const k = x.args[0];
				const ke = elemOf(k.type)!;
				const inputs = x.args.slice(1);
				const tmp = this.hiddenTemp(ELEM[ke].slot === "I64" ? 8 : 4);
				this.emitExpr(k);
				this.emitCheck(ke, 0n, BigInt(inputs.length - 1));
				this.storeFrame(tmp, ke);
				const end = em.label();
				inputs.forEach((input, i) => {
					const next = em.label();
					if (i < inputs.length - 1) {
						this.loadDirect(Op.LDL_U8, tmp, ke);
						this.pushRaw(ke, toRaw(ke, BigInt(i)));
						em.op(compareOp("=", ke));
						em.jump(Op.JZ, next);
					}
					this.emitExpr(input);
					em.jump(Op.JMP, end);
					em.bind(next);
				});
				em.bind(end);
				return;
			}
			case "SHL":
			case "SHR": {
				this.emitExpr(x.args[0]);
				const n = x.args[1];
				this.emitExpr(n);
				const ne = elemOf(n.type)!;
				// the count as an unsigned value of the operand's slot width (negative: huge, gives 0)
				if (slot === "I64" && ELEM[ne].slot === "I32") em.op(ELEM[ne].signed ? Op.I32S_TO_I64 : Op.I32U_TO_I64);
				if (slot === "I32" && ELEM[ne].slot === "I64") {
					// a count >= 2^32 must still give 0: clamp to 255
					em.op(Op.DUP);
					em.pushI64(255n);
					em.op(Op.GTU_I64);
					const ok = em.label();
					em.jump(Op.JZ, ok);
					em.op(Op.DROP);
					em.pushI64(255n);
					em.bind(ok);
					em.op(Op.I64_TO_I32);
				}
				em.op(x.name === "SHL" ? (slot === "I64" ? Op.SHL_I64 : Op.SHL_I32) : (slot === "I64" ? Op.SHR_I64 : Op.SHR_I32));
				this.normalize(e);
				return;
			}
			case "ROL":
			case "ROR": {
				this.emitExpr(x.args[0]);
				const n = x.args[1];
				this.emitExpr(n);
				if (ELEM[elemOf(n.type)!].slot === "I64") em.op(Op.I64_TO_I32);
				const bits = ELEM[e].size * 8;
				const table = { ROL: { 8: Native.ROL_8, 16: Native.ROL_16, 32: Native.ROL_32, 64: Native.ROL_64 }, ROR: { 8: Native.ROR_8, 16: Native.ROR_16, 32: Native.ROR_32, 64: Native.ROR_64 } } as const;
				em.opArg(Op.SYSCALL, table[x.name][bits as 8 | 16 | 32 | 64]);
				return;
			}
			default: {
				const n = REAL_NATIVES[x.name];
				if (n === undefined) throw new Error(`internal: intrinsic ${x.name}`);
				this.emitExpr(x.args[0]);
				if (e === "REAL") em.op(Op.F32_TO_F64);
				em.opArg(Op.SYSCALL, n);
				if (e === "REAL") em.op(Op.F64_TO_F32);
			}
		}
	}

	// ---- linking ------------------------------------------------------------------------------------
	private link(programs: Map<string, number>, taskDefs: TaskDef[], sources: SourceUnit[]): CompileResult {
		// frame sizes are final now
		for (const fp of this.framePatches) this.emitter.patch32(fp.at, fp.pou.frameSize);
		// frame area per task: the deepest call chain (IEC forbids recursion)
		const depth = new Map<string, number>();
		const visiting = new Set<string>();
		const frameNeed = (key: string): number => {
			const d = depth.get(key);
			if (d !== undefined) return d;
			const p = this.pous.get(key)!;
			if (visiting.has(key)) this.fatal(p.ast.span, `'${p.name}' calls itself (recursion is not allowed)`);
			visiting.add(key);
			let max = 0;
			for (const c of p.callees) max = Math.max(max, frameNeed(c));
			visiting.delete(key);
			const need = alignUp(p.frameSize, 8) + max;
			depth.set(key, need);
			return need;
		};
		const tasks: TaskInfo[] = [];
		for (const t of taskDefs) {
			const progs = t.programs.map((n) => this.pous.get(n.toUpperCase())!);
			const need = Math.max(8, ...progs.map((p) => frameNeed(p.key)));
			const frameBase = this.allocGlobal(need, 8);
			tasks.push({
				interval: t.intervalCycles, priority: t.priority ?? 0, maxOverruns: t.maxOverruns ?? 3, budget: t.budget ?? 2_000_000,
				frameBase, frameSize: need, programs: progs.map((p) => ({ entry: this.entries.get(p.key)!, instance: programs.get(p.key)! })),
				ioFirst: 0, ioCount: 0,
			});
		}
		if (this.diag.hasErrors) return this.result(null, null);
		const dataSize = alignUp(this.dataTop, 8);
		const maxData = this.options.maxData ?? 32 * 1024;
		const nowhere = { start: { file: "<link>", line: 1, col: 1, offset: 0 }, end: { file: "<link>", line: 1, col: 1, offset: 0 } };
		if (dataSize > maxData) this.fatal(nowhere, `the program needs ${dataSize} bytes of data, the device has ${maxData}`);
		const code = this.emitter.finish(this.entries);
		const maxCode = this.options.maxCode ?? 64 * 1024;
		if (code.length > maxCode) this.fatal(nowhere, `the program needs ${code.length} bytes of code, the device has ${maxCode}`);

		// initial values: the non-zero runs of the data image
		const data = new Uint8Array(dataSize);
		for (const [a, b] of this.dataInit) data.set(b, a);
		const init: { addr: number; bytes: Uint8Array }[] = [];
		for (let i = 0; i < data.length;) {
			if (data[i] === 0) {
				i++;
				continue;
			}
			let j = i;
			let zeros = 0;
			while (j < data.length && zeros < 8) {
				zeros = data[j] === 0 ? zeros + 1 : 0;
				j++;
			}
			const end = j - zeros;
			init.push({ addr: i, bytes: data.slice(i, end) });
			i = j;
		}

		const projectHash = crc32Text(sources.map((s) => `${s.file}\n${s.text}`).join("\n\0"));
		const contents: ImageContents = {
			code, dataSize, init, tasks, io: [], natives: [...this.natives].sort((a, b) => a - b),
			ioMapHash: this.options.ioMapHash ?? 0, projectHash, compilerVersion: COMPILER_VERSION,
		};
		const image = writeImage(contents);

		const symbols = new Map<string, Symbol>();
		for (const [k, v] of this.globals) symbols.set(k, { addr: v.offset, type: v.type });
		for (const [k, addr] of programs) symbols.set(k, { addr, type: { k: "fb", layout: this.pous.get(k)!.layout! } });
		const debug: DebugInfo = {
			statements: this.statements,
			symbols,
			dataSize,
			resolve: (path) => resolvePath(symbols, path),
		};
		return this.result(image, debug);
	}
}

// ---- helpers ------------------------------------------------------------------------------------------
const isComposite = (t: Type) => t.k === "struct" || t.k === "array";

function spanOfInit(i: A.Init): Span {
	return i.kind === "expr" ? i.expr.span : i.span;
}

function exprText(e: A.Expr): string {
	switch (e.kind) {
		case "name": return e.name;
		case "member": return `${exprText(e.object)}.${e.name}`;
		default: return "expression";
	}
}

function constName(c: Const): string {
	return c.k === "int" ? "integer" : c.k === "real" ? "real number" : c.k === "bool" ? "BOOL" : c.k === "time" ? "TIME" : typeName(c.type);
}

/** The slot bits of an integer value of type e. */
function toRaw(e: ElemName, v: bigint): number | bigint {
	const s = ELEM[e].slot;
	if (s === "I64") return BigInt.asUintN(64, v);
	return Number(BigInt.asUintN(32, v));
}

function rawToBig(e: ElemName, raw: number | bigint): bigint {
	const info = ELEM[e];
	if (info.slot === "I64") return info.signed ? BigInt.asIntN(64, BigInt(raw)) : BigInt.asUintN(64, BigInt(raw));
	return info.signed ? BigInt(Number(raw) | 0) : BigInt(Number(raw) >>> 0);
}

const f32Scratch = new DataView(new ArrayBuffer(8));
function realRaw(e: ElemName, v: number): number | bigint {
	if (e === "REAL") {
		f32Scratch.setFloat32(0, v, true);
		return f32Scratch.getUint32(0, true);
	}
	f32Scratch.setFloat64(0, v, true);
	return f32Scratch.getBigUint64(0, true);
}

function writeRaw(b: Uint8Array, at: number, e: ElemName, raw: number | bigint): void {
	const dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
	switch (ELEM[e].size) {
		case 1: dv.setUint8(at, Number(raw) & 0xff); break;
		case 2: dv.setUint16(at, Number(raw) & 0xffff, true); break;
		case 4: dv.setUint32(at, Number(raw) >>> 0, true); break;
		default: dv.setBigUint64(at, BigInt.asUintN(64, BigInt(raw)), true);
	}
}

/** Index into the 6 loads of a mode: U8 S8 U16 S16 32 64. */
function loadIndex(e: ElemName): number {
	const i = ELEM[e];
	if (i.size === 1) return i.signed && i.category === "int" ? 1 : 0;
	if (i.size === 2) return i.signed && i.category === "int" ? 3 : 2;
	return i.size === 4 ? 4 : 5;
}

/** Index into the 4 stores of a mode: 8 16 32 64. */
function storeIndex(e: ElemName): number {
	return [0, 0, 1, 1, 2, 2, 2, 2, 3][ELEM[e].size];
}

function compareOp(op: "<" | ">" | "<=" | ">=" | "=" | "<>", e: ElemName): Op {
	const i = ELEM[e];
	const sfx = i.slot;
	const signed = i.signed && i.category !== "bit";
	const pick = (s32: Op, u32: Op, s64: Op, u64: Op, f32: Op, f64: Op) =>
		sfx === "F32" ? f32 : sfx === "F64" ? f64 : sfx === "I64" ? (signed ? s64 : u64) : signed ? s32 : u32;
	switch (op) {
		case "=": return pick(Op.EQ_I32, Op.EQ_I32, Op.EQ_I64, Op.EQ_I64, Op.EQ_F32, Op.EQ_F64);
		case "<>": return pick(Op.NE_I32, Op.NE_I32, Op.NE_I64, Op.NE_I64, Op.NE_F32, Op.NE_F64);
		case "<": return pick(Op.LTS_I32, Op.LTU_I32, Op.LTS_I64, Op.LTU_I64, Op.LT_F32, Op.LT_F64);
		case "<=": return pick(Op.LES_I32, Op.LEU_I32, Op.LES_I64, Op.LEU_I64, Op.LE_F32, Op.LE_F64);
		case ">": return pick(Op.GTS_I32, Op.GTU_I32, Op.GTS_I64, Op.GTU_I64, Op.GT_F32, Op.GT_F64);
		case ">=": return pick(Op.GES_I32, Op.GEU_I32, Op.GES_I64, Op.GEU_I64, Op.GE_F32, Op.GE_F64);
	}
}

function binaryOp(op: A.BinaryOp, e: ElemName): Op {
	if (["<", ">", "<=", ">=", "=", "<>"].includes(op)) return compareOp(op as "<", e);
	const i = ELEM[e];
	const s = i.slot;
	const signed = i.signed && i.category !== "bit" && i.category !== "uint";
	const by = (i32: Op, i64: Op, f32: Op | null, f64: Op | null): Op => {
		const r = s === "I32" ? i32 : s === "I64" ? i64 : s === "F32" ? f32 : f64;
		if (r === null) throw new Error(`internal: ${op} on ${e}`);
		return r;
	};
	switch (op) {
		case "+": return by(Op.ADD_I32, Op.ADD_I64, Op.ADD_F32, Op.ADD_F64);
		case "-": return by(Op.SUB_I32, Op.SUB_I64, Op.SUB_F32, Op.SUB_F64);
		case "*": return by(Op.MUL_I32, Op.MUL_I64, Op.MUL_F32, Op.MUL_F64);
		case "/": return by(signed ? Op.DIVS_I32 : Op.DIVU_I32, signed ? Op.DIVS_I64 : Op.DIVU_I64, Op.DIV_F32, Op.DIV_F64);
		case "MOD": return by(signed ? Op.MODS_I32 : Op.MODU_I32, signed ? Op.MODS_I64 : Op.MODU_I64, null, null);
		case "AND": return by(Op.AND_I32, Op.AND_I64, null, null);
		case "OR": return by(Op.OR_I32, Op.OR_I64, null, null);
		case "XOR": return by(Op.XOR_I32, Op.XOR_I64, null, null);
		default: throw new Error(`internal: operator ${op}`);
	}
}

function crc32Text(s: string): number {
	let crc = 0xffffffff;
	const bytes = new TextEncoder().encode(s);
	for (const b of bytes) {
		crc ^= b;
		for (let k = 0; k < 8; k++) crc = crc & 1 ? 0xedb88320 ^ (crc >>> 1) : crc >>> 1;
	}
	return ~crc >>> 0;
}

function resolvePath(symbols: Map<string, Symbol>, path: string): Symbol | null {
	const parts = path.toUpperCase().match(/[A-Z_][A-Z0-9_]*|\[[^\]]*\]/g);
	if (!parts) return null;
	let cur: Symbol | undefined = symbols.get(parts[0]);
	if (!cur) return null;
	for (const p of parts.slice(1)) {
		const t: Type = cur.type;
		if (p.startsWith("[")) {
			if (t.k !== "array") return null;
			const idx = p.slice(1, -1).split(",").map((s) => Number(s.trim()));
			if (idx.length !== t.dims.length || idx.some((v, i) => !Number.isInteger(v) || v < t.dims[i].lo || v > t.dims[i].hi)) return null;
			let off = 0;
			let stride = sizeOf(t.elem);
			for (let i = t.dims.length - 1; i >= 0; i--) {
				off += (idx[i] - t.dims[i].lo) * stride;
				stride *= t.dims[i].hi - t.dims[i].lo + 1;
			}
			cur = { addr: cur.addr + off, type: t.elem };
			continue;
		}
		const fields: Field[] | null = t.k === "struct" ? t.fields : t.k === "fb" ? t.layout.fields : null;
		const f: Field | undefined = fields?.find((x) => x.key === p);
		if (!f) return null;
		cur = { addr: cur.addr + f.offset, type: f.type };
	}
	return cur;
}
