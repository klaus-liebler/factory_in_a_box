// SPDX-License-Identifier: MIT
// The PLC bytecode VM in TypeScript: the simulator of the IDE and the reference for the
// differential tests. It is the exact counterpart of rtoos/src/plc/vm.cpp (rtoos/docs/plc.md):
// same verifier, same traps, bit-identical results. DOM-free.
//
// Slots hold raw bits like the C++ VM: a 32-bit value (i32, f32 bits) is a number, zero-extended
// in the C++ slot; a 64-bit value (i64, f64 bits) is a bigint in [0, 2^64). Every pop converts, so
// even bytecode that mixes the kinds behaves as in C++.
import { NATIVES, Native, OPS, Op, Trap, ISA_HASH, ISA_VERSION } from "./generated/isa.ts";
import {
	HEADER_SIZE, Hdr, IMAGE_MAGIC, IO_RECORD_SIZE, NATIVE_RECORD_SIZE, PROGRAM_RECORD_SIZE, TASK_RECORD_SIZE,
	crc32, type IoBinding, type LoadError, type TaskInfo,
} from "./image.ts";

export const OPERAND_STACK_SLOTS = 64;
export const CALL_DEPTH = 32;
export const MAX_TASKS = 8;
const NO_RETURN = 0xffffffff;

export type RunState = "done" | "trapped";
export interface RunResult {
	state: RunState;
	trap: Trap;
	trapCode: number;
	pc: number;
	instructions: number;
}

type Slot = number | bigint;
const M64 = (1n << 64n) - 1n;

// ---- slot conversions --------------------------------------------------------------------------
const lo32 = (v: Slot): number => (typeof v === "number" ? v | 0 : Number(BigInt.asIntN(32, v)));
const u32 = (v: Slot): number => lo32(v) >>> 0;
const u64 = (v: Slot): bigint => (typeof v === "bigint" ? v : BigInt(v >>> 0));
const s64 = (v: Slot): bigint => BigInt.asIntN(64, u64(v));
const ofU64 = (v: bigint): bigint => v & M64;
const ofB = (b: boolean): number => (b ? 1 : 0);

const scratch = new DataView(new ArrayBuffer(8));
const f32 = (v: Slot): number => {
	scratch.setInt32(0, lo32(v), true);
	return scratch.getFloat32(0, true);
};
const f64 = (v: Slot): number => {
	scratch.setBigUint64(0, u64(v), true);
	return scratch.getFloat64(0, true);
};
/** Float results: a NaN becomes the canonical quiet NaN. */
const ofF32 = (x: number): number => {
	if (Number.isNaN(x)) return 0x7fc00000;
	scratch.setFloat32(0, x, true);
	return scratch.getInt32(0, true);
};
const ofF64 = (x: number): bigint => {
	if (Number.isNaN(x)) return 0x7ff8000000000000n;
	scratch.setFloat64(0, x, true);
	return scratch.getBigUint64(0, true);
};

/** A 64-bit integer to binary32 with a single rounding (Number() then fround would round twice). */
function bigintToF32(v: bigint): number {
	const neg = v < 0n;
	let a = neg ? -v : v;
	if (a < 1n << 53n) return Math.fround(Number(v));
	// round to odd at 53 bits, then the rounding to 24 bits is correct
	const shift = BigInt(a.toString(2).length - 53);
	let m = a >> shift;
	if ((a & ((1n << shift) - 1n)) !== 0n) m |= 1n;
	a = m;
	const r = Math.fround(Number(a) * 2 ** Number(shift));
	return neg ? -r : r;
}

/** Round half away from zero (REAL_TO_INT); x is finite. */
function roundAway(x: number): number {
	const t = Math.trunc(x);
	return Math.abs(x - t) >= 0.5 ? t + Math.sign(x) : t;
}

function rotl(x: number, n: number, width: number): number {
	const mask = width === 32 ? 0xffffffff : (1 << width) - 1;
	x = (x & mask) >>> 0;
	n = (n >>> 0) % width;
	if (n === 0) return x;
	return (((x << n) | (x >>> (width - n))) & mask) >>> 0;
}
const rotr = (x: number, n: number, width: number): number => {
	n = (n >>> 0) % width;
	return rotl(x, n === 0 ? 0 : width - n, width);
};
const rotl64 = (x: bigint, n: number): bigint => {
	const k = BigInt((n >>> 0) % 64);
	return k === 0n ? x : ((x << k) | (x >> (64n - k))) & M64;
};

const G_WIDTH = new Map<Op, number>([
	[Op.LDG_U8, 1], [Op.LDG_S8, 1], [Op.LDG_U16, 2], [Op.LDG_S16, 2], [Op.LDG_32, 4], [Op.LDG_64, 8],
	[Op.STG_8, 1], [Op.STG_16, 2], [Op.STG_32, 4], [Op.STG_64, 8],
]);

class TrapError {
	constructor(readonly trap: Trap, readonly code = 0) {}
}

export class Context {
	stack: Slot[] = new Array<Slot>(OPERAND_STACK_SLOTS).fill(0);
	calls: { returnPc: number; fp: number; self: number }[] = [];
	sp = 0;
	pc = 0;
	fp = 0;
	self = 0;
	frameTop = 0;
	frameEnd = 0;
	nowMs = 0;
}

export class Vm {
	private image: Uint8Array = new Uint8Array(0);
	private img: DataView = new DataView(new ArrayBuffer(0));
	private codeOffset = 0;
	private codeSize = 0;
	private dataSize_ = 0;
	private tasks_: TaskInfo[] = [];
	private io_: IoBinding[] = [];
	private nativesListed = new Set<number>();
	private loaded_ = false;
	readonly contexts: Context[];
	private readonly dv: DataView;

	/** `data`: the data segment's memory (its capacity), `contexts`: one per task. */
	constructor(readonly data: Uint8Array, contextCount = MAX_TASKS) {
		this.dv = new DataView(data.buffer, data.byteOffset, data.byteLength);
		this.contexts = Array.from({ length: contextCount }, () => new Context());
	}

	get loaded(): boolean {
		return this.loaded_;
	}
	get tasks(): readonly TaskInfo[] {
		return this.tasks_;
	}
	get io(): readonly IoBinding[] {
		return this.io_;
	}
	get dataSize(): number {
		return this.dataSize_;
	}
	unload(): void {
		this.loaded_ = false;
	}

	// ---- loader and verifier (same checks and order as Vm::load in C++) -------------------------
	load(image: Uint8Array): LoadError {
		this.loaded_ = false;
		if (image.length < HEADER_SIZE) return "too_short";
		const h = new DataView(image.buffer, image.byteOffset, image.byteLength);
		for (let i = 0; i < IMAGE_MAGIC.length; i++) if (image[i] !== IMAGE_MAGIC[i]) return "magic";
		if (h.getUint16(Hdr.headerSize, true) !== HEADER_SIZE) return "header_size";
		const length = h.getUint32(Hdr.length, true);
		if (length < HEADER_SIZE || length > image.length) return "too_short";
		image = image.subarray(0, length);
		if (crc32(image.subarray(Hdr.crc + 4)) !== h.getUint32(Hdr.crc, true)) return "crc";
		if (h.getUint16(Hdr.isaVersion, true) !== ISA_VERSION) return "isa_version";
		if (h.getUint32(Hdr.isaHash, true) !== ISA_HASH >>> 0) return "isa_hash";

		const r32 = (at: number) => h.getUint32(at, true);
		const inside = (lo: number, size: number, limit: number) => lo + size <= limit;
		const section = (offField: number, bytes: number) => {
			const off = r32(offField);
			return off % 4 === 0 && off >= HEADER_SIZE && inside(off, bytes, length);
		};
		const codeSize = r32(Hdr.codeSize);
		const dataSize = r32(Hdr.dataSize);
		const initSize = r32(Hdr.initSize);
		const taskCount = r32(Hdr.taskCount);
		const ioCount = r32(Hdr.ioCount);
		const nativeCount = r32(Hdr.nativeCount);
		if (!section(Hdr.codeOffset, codeSize) || !section(Hdr.initOffset, initSize) ||
			!section(Hdr.taskOffset, taskCount * TASK_RECORD_SIZE) || !section(Hdr.ioOffset, ioCount * IO_RECORD_SIZE) ||
			!section(Hdr.nativeOffset, nativeCount * NATIVE_RECORD_SIZE)) {
			return "section";
		}
		if (dataSize > this.data.length) return "data_too_large";
		if (codeSize === 0) return "truncated";

		this.image = image;
		this.img = h;
		this.codeOffset = r32(Hdr.codeOffset);
		this.codeSize = codeSize;
		this.dataSize_ = dataSize;

		this.nativesListed = new Set();
		for (let i = 0, at = r32(Hdr.nativeOffset); i < nativeCount; i++, at += NATIVE_RECORD_SIZE) {
			const id = h.getUint16(at, true);
			if (id >= NATIVES.length) return "native_unknown";
			if (r32(at + 4) !== NATIVES[id].signature >>> 0) return "native_signature";
			this.nativesListed.add(id);
		}

		const initAt = r32(Hdr.initOffset);
		for (let pos = 0; pos < initSize;) {
			if (initSize - pos < 8) return "init_range";
			const addr = r32(initAt + pos);
			const n = r32(initAt + pos + 4);
			const padded = (n + 3) & ~3;
			if (!inside(addr, n, dataSize) || n + 3 > 0xffffffff || padded > initSize - pos - 8) return "init_range";
			pos += 8 + padded;
		}

		if (taskCount === 0 || taskCount > MAX_TASKS || taskCount > this.contexts.length) return "task_range";
		const tasks: TaskInfo[] = [];
		const taskAt = r32(Hdr.taskOffset);
		for (let i = 0; i < taskCount; i++) {
			const p = taskAt + i * TASK_RECORD_SIZE;
			const programsAt = r32(p + 16);
			const programCount = r32(p + 20);
			const t: TaskInfo = {
				interval: h.getUint16(p, true), priority: image[p + 2], maxOverruns: image[p + 3], budget: r32(p + 4),
				frameBase: r32(p + 8), frameSize: r32(p + 12), programs: [], ioFirst: r32(p + 24), ioCount: r32(p + 28),
			};
			if (t.budget === 0 || t.interval === 0 || !inside(t.frameBase, t.frameSize, dataSize)) return "task_range";
			for (const o of tasks) {
				if (t.frameBase < o.frameBase + o.frameSize && o.frameBase < t.frameBase + t.frameSize) return "task_range";
			}
			if (programCount === 0 || programsAt % 4 !== 0 || programsAt < HEADER_SIZE ||
				!inside(programsAt, programCount * PROGRAM_RECORD_SIZE, length)) {
				return "program_range";
			}
			for (let k = 0; k < programCount; k++) {
				const entry = r32(programsAt + k * PROGRAM_RECORD_SIZE);
				const instance = r32(programsAt + k * PROGRAM_RECORD_SIZE + 4);
				if (instance > dataSize) return "program_range";
				t.programs.push({ entry, instance });
			}
			if (!inside(t.ioFirst, t.ioCount, ioCount)) return "io_range";
			tasks.push(t);
		}
		this.tasks_ = tasks;

		const io: IoBinding[] = [];
		for (let i = 0, at = r32(Hdr.ioOffset); i < ioCount; i++, at += IO_RECORD_SIZE) {
			const bits = image[at + 4];
			const b = {
				reg: h.getUint16(at, true), area: image[at + 2], kind: image[at + 3], bits, signed: (image[at + 5] & 1) !== 0,
				addr: r32(at + 8),
			} as IoBinding;
			const bytes = bits === 1 ? 1 : bits / 8;
			if ((bits !== 1 && bits !== 16 && bits !== 32) || b.area > 1 || b.kind > 3 || !inside(b.addr, bytes, dataSize)) {
				return "io_range";
			}
			io.push(b);
		}
		this.io_ = io;

		const e = this.verifyCode();
		if (e !== "none") return e;

		this.data.fill(0, 0, dataSize);
		for (let pos = 0; pos < initSize;) {
			const addr = r32(initAt + pos);
			const n = r32(initAt + pos + 4);
			this.data.set(image.subarray(initAt + pos + 8, initAt + pos + 8 + n), addr);
			pos += 8 + ((n + 3) & ~3);
		}
		this.loaded_ = true;
		return "none";
	}

	private verifyCode(): LoadError {
		const code = this.image.subarray(this.codeOffset, this.codeOffset + this.codeSize);
		const cv = new DataView(code.buffer, code.byteOffset, code.byteLength);
		const size = code.length;
		// The C++ VM keeps this bitmap in the data segment; it needs (size + 7) / 8 bytes of it.
		if ((size + 7) >>> 3 > this.data.length) return "data_too_large";
		const starts = new Uint8Array(size);
		let last = 0;
		for (let pc = 0; pc < size;) {
			const b = code[pc];
			const info = OPS[b];
			if (info === undefined) return "bad_opcode";
			if (info.size > size - pc) return "truncated";
			starts[pc] = 1;
			const w = G_WIDTH.get(b);
			if (w !== undefined && cv.getUint32(pc + 1, true) + w > this.dataSize_) return "data_address";
			if (b === Op.ADDR_G && cv.getUint32(pc + 1, true) > this.dataSize_) return "data_address";
			if (b === Op.SYSCALL) {
				const id = cv.getUint16(pc + 1, true);
				if (id >= NATIVES.length || !this.nativesListed.has(id)) return "native_not_listed";
			}
			last = pc;
			pc += info.size;
		}
		const end = code[last];
		if (end !== Op.RET && end !== Op.JMP && end !== Op.TRAP) return "truncated";
		const isEnter = (pc: number) => pc >= 0 && pc < size && starts[pc] === 1 && code[pc] === Op.ENTER;
		for (let pc = 0; pc < size; pc += OPS[code[pc]].size) {
			const o = code[pc];
			if (o === Op.JMP || o === Op.JZ || o === Op.JNZ) {
				const target = pc + OPS[o].size + cv.getInt32(pc + 1, true);
				if (target < 0 || target >= size || starts[target] !== 1) return "jump_target";
			} else if ((o === Op.CALL || o === Op.CALL_FB) && !isEnter(cv.getUint32(pc + 1, true))) {
				return "call_target";
			}
		}
		for (const t of this.tasks_) for (const p of t.programs) if (!isEnter(p.entry)) return "call_target";
		return "none";
	}

	// ---- execution ----------------------------------------------------------------------------------
	runCycle(task: number, nowMs: number): RunResult {
		if (!this.loaded_ || task >= this.tasks_.length) throw new Error("PLC VM: no such task");
		const t = this.tasks_[task];
		const c = this.contexts[task];
		c.nowMs = nowMs >>> 0;
		c.frameTop = t.frameBase;
		c.frameEnd = t.frameBase + t.frameSize;
		const total: RunResult = { state: "done", trap: Trap.NONE, trapCode: 0, pc: 0, instructions: 0 };
		let remaining = t.budget;
		for (const p of t.programs) {
			c.sp = 0;
			c.pc = p.entry;
			c.self = p.instance;
			c.fp = c.frameTop;
			c.calls = [{ returnPc: NO_RETURN, fp: c.fp, self: c.self }];
			const r = this.execute(c, remaining);
			total.instructions += r.instructions;
			if (r.state === "trapped") return { ...r, instructions: total.instructions };
			remaining -= r.instructions;
		}
		return total;
	}

	private execute(c: Context, budget: number): RunResult {
		const code = this.image.subarray(this.codeOffset, this.codeOffset + this.codeSize);
		const cv = new DataView(code.buffer, code.byteOffset, code.byteLength);
		const data = this.data;
		const dv = this.dv;
		const dsize = this.dataSize_;
		const st = c.stack;
		let sp = c.sp;
		let pc = c.pc;
		let at = pc;
		let count = 0;

		const need = (k: number) => {
			if (sp < k) throw new TrapError(Trap.STACK_UNDERFLOW);
		};
		const room = (k: number) => {
			if (sp + k > OPERAND_STACK_SLOTS) throw new TrapError(Trap.STACK_OVERFLOW);
		};
		const addr = (a: number, w: number) => {
			if (a + w > dsize) throw new TrapError(Trap.MEMORY);
			return a;
		};
		const push = (v: Slot) => {
			room(1);
			st[sp++] = v;
		};
		const pop = (): Slot => {
			need(1);
			return st[--sp];
		};
		const binary = (f: (a: Slot, b: Slot) => Slot) => {
			need(2);
			const b = st[--sp];
			st[sp - 1] = f(st[sp - 1], b);
		};
		const unary = (f: (a: Slot) => Slot) => {
			need(1);
			st[sp - 1] = f(st[sp - 1]);
		};
		const load = (a: number, kind: number): Slot => {
			switch (kind) {
				case 0: return data[addr(a, 1)];
				case 1: return (data[addr(a, 1)] << 24) >> 24 >>> 0;
				case 2: return dv.getUint16(addr(a, 2), true);
				case 3: return dv.getInt16(addr(a, 2), true) >>> 0;
				case 4: return dv.getUint32(addr(a, 4), true);
				default: return dv.getBigUint64(addr(a, 8), true);
			}
		};
		const store = (a: number, width: number, v: Slot) => {
			switch (width) {
				case 0: data[addr(a, 1)] = u32(v) & 0xff; break;
				case 1: dv.setUint16(addr(a, 2), u32(v) & 0xffff, true); break;
				case 2: dv.setUint32(addr(a, 4), u32(v), true); break;
				default: dv.setBigUint64(addr(a, 8), u64(v), true);
			}
		};
		const jump = () => {
			pc = pc + cv.getInt32(at + 1, true);
		};

		try {
			for (;;) {
				at = pc;
				if (count === budget) throw new TrapError(Trap.BUDGET);
				count++;
				const o: Op = code[pc];
				pc += OPS[o].size;
				const a1 = at + 1;
				// the families of loads and stores: LD{G,L,T}_{U8,S8,U16,S16,32,64}, ST{G,L,T}_{8,16,32,64}
				if (o >= Op.LDG_U8 && o <= Op.LDT_64) {
					const k = o - Op.LDG_U8;
					const mode = Math.floor(k / 6);
					const a = mode === 0 ? cv.getUint32(a1, true) : (mode === 1 ? c.fp : c.self) + cv.getUint16(a1, true);
					push(load(a, k % 6));
					continue;
				}
				if (o >= Op.STG_8 && o <= Op.STT_64) {
					const k = o - Op.STG_8;
					const mode = Math.floor(k / 4);
					const a = mode === 0 ? cv.getUint32(a1, true) : (mode === 1 ? c.fp : c.self) + cv.getUint16(a1, true);
					need(1);
					store(a, k % 4, st[sp - 1]);
					sp--;
					continue;
				}
				if (o >= Op.LDI_U8 && o <= Op.LDI_64) {
					need(1);
					st[sp - 1] = load(u32(st[sp - 1]), o - Op.LDI_U8);
					continue;
				}
				if (o >= Op.STI_8 && o <= Op.STI_64) {
					need(2);
					store(u32(st[sp - 2]), o - Op.STI_8, st[sp - 1]);
					sp -= 2;
					continue;
				}
				switch (o) {
					case Op.NOP: break;
					case Op.PUSH_I8: push((cv.getInt8(a1)) >>> 0); break;
					case Op.PUSH_I32: push(cv.getUint32(a1, true)); break;
					case Op.PUSH_I64: push(cv.getBigUint64(a1, true)); break;
					case Op.DUP: need(1); push(st[sp - 1]); break;
					case Op.DROP: pop(); break;
					case Op.SWAP: { need(2); const t = st[sp - 1]; st[sp - 1] = st[sp - 2]; st[sp - 2] = t; break; }
					case Op.ADDR_G: push(cv.getUint32(a1, true)); break;
					case Op.ADDR_L: push((c.fp + cv.getUint16(a1, true)) >>> 0); break;
					case Op.ADDR_T: push((c.self + cv.getUint16(a1, true)) >>> 0); break;
					case Op.MEMCPY: {
						need(2);
						const n = cv.getUint32(a1, true);
						const src = addr(u32(st[sp - 1]), n);
						const dst = addr(u32(st[sp - 2]), n);
						data.copyWithin(dst, src, src + n);
						sp -= 2;
						break;
					}

					// ---- i32 ----
					case Op.ADD_I32: binary((a, b) => (u32(a) + u32(b)) >>> 0); break;
					case Op.SUB_I32: binary((a, b) => (u32(a) - u32(b)) >>> 0); break;
					case Op.MUL_I32: binary((a, b) => Math.imul(u32(a), u32(b)) >>> 0); break;
					case Op.DIVS_I32: binary((a, b) => {
						const x = lo32(a), y = lo32(b);
						if (y === 0) throw new TrapError(Trap.DIV_BY_ZERO);
						return (y === -1 ? -x : Math.trunc(x / y)) >>> 0;
					}); break;
					case Op.DIVU_I32: binary((a, b) => {
						if (u32(b) === 0) throw new TrapError(Trap.DIV_BY_ZERO);
						return Math.floor(u32(a) / u32(b)) >>> 0;
					}); break;
					case Op.MODS_I32: binary((a, b) => {
						const x = lo32(a), y = lo32(b);
						if (y === 0) throw new TrapError(Trap.DIV_BY_ZERO);
						return (y === -1 ? 0 : x % y) >>> 0;
					}); break;
					case Op.MODU_I32: binary((a, b) => {
						if (u32(b) === 0) throw new TrapError(Trap.DIV_BY_ZERO);
						return (u32(a) % u32(b)) >>> 0;
					}); break;
					case Op.NEG_I32: unary((a) => (0 - u32(a)) >>> 0); break;
					case Op.AND_I32: binary((a, b) => (u32(a) & u32(b)) >>> 0); break;
					case Op.OR_I32: binary((a, b) => (u32(a) | u32(b)) >>> 0); break;
					case Op.XOR_I32: binary((a, b) => (u32(a) ^ u32(b)) >>> 0); break;
					case Op.NOT_I32: unary((a) => ~u32(a) >>> 0); break;
					case Op.SHL_I32: binary((a, b) => (u32(b) >= 32 ? 0 : (u32(a) << u32(b)) >>> 0)); break;
					case Op.SHR_I32: binary((a, b) => (u32(b) >= 32 ? 0 : u32(a) >>> u32(b))); break;

					// ---- i64 ----
					case Op.ADD_I64: binary((a, b) => ofU64(u64(a) + u64(b))); break;
					case Op.SUB_I64: binary((a, b) => ofU64(u64(a) - u64(b))); break;
					case Op.MUL_I64: binary((a, b) => ofU64(u64(a) * u64(b))); break;
					case Op.DIVS_I64: binary((a, b) => {
						const x = s64(a), y = s64(b);
						if (y === 0n) throw new TrapError(Trap.DIV_BY_ZERO);
						return ofU64(y === -1n ? -x : x / y); // BigInt division truncates toward zero
					}); break;
					case Op.DIVU_I64: binary((a, b) => {
						if (u64(b) === 0n) throw new TrapError(Trap.DIV_BY_ZERO);
						return u64(a) / u64(b);
					}); break;
					case Op.MODS_I64: binary((a, b) => {
						const x = s64(a), y = s64(b);
						if (y === 0n) throw new TrapError(Trap.DIV_BY_ZERO);
						return y === -1n ? 0n : ofU64(x % y);
					}); break;
					case Op.MODU_I64: binary((a, b) => {
						if (u64(b) === 0n) throw new TrapError(Trap.DIV_BY_ZERO);
						return u64(a) % u64(b);
					}); break;
					case Op.NEG_I64: unary((a) => ofU64(-u64(a))); break;
					case Op.AND_I64: binary((a, b) => u64(a) & u64(b)); break;
					case Op.OR_I64: binary((a, b) => u64(a) | u64(b)); break;
					case Op.XOR_I64: binary((a, b) => u64(a) ^ u64(b)); break;
					case Op.NOT_I64: unary((a) => ofU64(~u64(a))); break;
					case Op.SHL_I64: binary((a, b) => (u64(b) >= 64n ? 0n : ofU64(u64(a) << u64(b)))); break;
					case Op.SHR_I64: binary((a, b) => (u64(b) >= 64n ? 0n : u64(a) >> u64(b))); break;

					// ---- f32, f64 (f32: in binary64, rounded once: exact for + - * /) ----
					case Op.ADD_F32: binary((a, b) => ofF32(Math.fround(f32(a) + f32(b)))); break;
					case Op.SUB_F32: binary((a, b) => ofF32(Math.fround(f32(a) - f32(b)))); break;
					case Op.MUL_F32: binary((a, b) => ofF32(Math.fround(f32(a) * f32(b)))); break;
					case Op.DIV_F32: binary((a, b) => {
						if (f32(b) === 0) throw new TrapError(Trap.DIV_BY_ZERO);
						return ofF32(Math.fround(f32(a) / f32(b)));
					}); break;
					case Op.NEG_F32: unary((a) => ofF32(-f32(a))); break;
					case Op.ADD_F64: binary((a, b) => ofF64(f64(a) + f64(b))); break;
					case Op.SUB_F64: binary((a, b) => ofF64(f64(a) - f64(b))); break;
					case Op.MUL_F64: binary((a, b) => ofF64(f64(a) * f64(b))); break;
					case Op.DIV_F64: binary((a, b) => {
						if (f64(b) === 0) throw new TrapError(Trap.DIV_BY_ZERO);
						return ofF64(f64(a) / f64(b));
					}); break;
					case Op.NEG_F64: unary((a) => ofF64(-f64(a))); break;

					// ---- comparisons ----
					case Op.EQ_I32: binary((a, b) => ofB(u32(a) === u32(b))); break;
					case Op.NE_I32: binary((a, b) => ofB(u32(a) !== u32(b))); break;
					case Op.LTS_I32: binary((a, b) => ofB(lo32(a) < lo32(b))); break;
					case Op.LTU_I32: binary((a, b) => ofB(u32(a) < u32(b))); break;
					case Op.LES_I32: binary((a, b) => ofB(lo32(a) <= lo32(b))); break;
					case Op.LEU_I32: binary((a, b) => ofB(u32(a) <= u32(b))); break;
					case Op.GTS_I32: binary((a, b) => ofB(lo32(a) > lo32(b))); break;
					case Op.GTU_I32: binary((a, b) => ofB(u32(a) > u32(b))); break;
					case Op.GES_I32: binary((a, b) => ofB(lo32(a) >= lo32(b))); break;
					case Op.GEU_I32: binary((a, b) => ofB(u32(a) >= u32(b))); break;
					case Op.EQ_I64: binary((a, b) => ofB(u64(a) === u64(b))); break;
					case Op.NE_I64: binary((a, b) => ofB(u64(a) !== u64(b))); break;
					case Op.LTS_I64: binary((a, b) => ofB(s64(a) < s64(b))); break;
					case Op.LTU_I64: binary((a, b) => ofB(u64(a) < u64(b))); break;
					case Op.LES_I64: binary((a, b) => ofB(s64(a) <= s64(b))); break;
					case Op.LEU_I64: binary((a, b) => ofB(u64(a) <= u64(b))); break;
					case Op.GTS_I64: binary((a, b) => ofB(s64(a) > s64(b))); break;
					case Op.GTU_I64: binary((a, b) => ofB(u64(a) > u64(b))); break;
					case Op.GES_I64: binary((a, b) => ofB(s64(a) >= s64(b))); break;
					case Op.GEU_I64: binary((a, b) => ofB(u64(a) >= u64(b))); break;
					case Op.EQ_F32: binary((a, b) => ofB(f32(a) === f32(b))); break;
					case Op.NE_F32: binary((a, b) => ofB(f32(a) !== f32(b))); break;
					case Op.LT_F32: binary((a, b) => ofB(f32(a) < f32(b))); break;
					case Op.LE_F32: binary((a, b) => ofB(f32(a) <= f32(b))); break;
					case Op.GT_F32: binary((a, b) => ofB(f32(a) > f32(b))); break;
					case Op.GE_F32: binary((a, b) => ofB(f32(a) >= f32(b))); break;
					case Op.EQ_F64: binary((a, b) => ofB(f64(a) === f64(b))); break;
					case Op.NE_F64: binary((a, b) => ofB(f64(a) !== f64(b))); break;
					case Op.LT_F64: binary((a, b) => ofB(f64(a) < f64(b))); break;
					case Op.LE_F64: binary((a, b) => ofB(f64(a) <= f64(b))); break;
					case Op.GT_F64: binary((a, b) => ofB(f64(a) > f64(b))); break;
					case Op.GE_F64: binary((a, b) => ofB(f64(a) >= f64(b))); break;

					// ---- normalization, conversion ----
					case Op.SEXT8: unary((a) => ((u32(a) << 24) >> 24) >>> 0); break;
					case Op.SEXT16: unary((a) => ((u32(a) << 16) >> 16) >>> 0); break;
					case Op.ZEXT8: unary((a) => (typeof a === "bigint" ? a & 0xffn : u32(a) & 0xff)); break;
					case Op.ZEXT16: unary((a) => (typeof a === "bigint" ? a & 0xffffn : u32(a) & 0xffff)); break;
					case Op.I32S_TO_I64: unary((a) => ofU64(BigInt(lo32(a)))); break;
					case Op.I32U_TO_I64: unary((a) => BigInt(u32(a))); break;
					case Op.I64_TO_I32: unary((a) => u32(a)); break;
					case Op.I32S_TO_F32: unary((a) => ofF32(Math.fround(lo32(a)))); break;
					case Op.I32S_TO_F64: unary((a) => ofF64(lo32(a))); break;
					case Op.I32U_TO_F32: unary((a) => ofF32(Math.fround(u32(a)))); break;
					case Op.I32U_TO_F64: unary((a) => ofF64(u32(a))); break;
					case Op.I64S_TO_F32: unary((a) => ofF32(bigintToF32(s64(a)))); break;
					case Op.I64S_TO_F64: unary((a) => ofF64(Number(s64(a)))); break;
					case Op.I64U_TO_F32: unary((a) => ofF32(bigintToF32(u64(a)))); break;
					case Op.I64U_TO_F64: unary((a) => ofF64(Number(u64(a)))); break;
					case Op.F32_TO_F64: unary((a) => ofF64(f32(a))); break;
					case Op.F64_TO_F32: unary((a) => ofF32(Math.fround(f64(a)))); break;
					case Op.F32_ROUND_I64:
					case Op.F32_TRUNC_I64:
					case Op.F64_ROUND_I64:
					case Op.F64_TRUNC_I64: unary((a) => {
						const x = o === Op.F32_ROUND_I64 || o === Op.F32_TRUNC_I64 ? f32(a) : f64(a);
						if (!(x >= -(2 ** 63) && x < 2 ** 63)) throw new TrapError(Trap.CONVERSION);
						const v = o === Op.F32_ROUND_I64 || o === Op.F64_ROUND_I64 ? roundAway(x) : Math.trunc(x);
						return ofU64(BigInt(v));
					}); break;
					case Op.CHK_I32: {
						need(1);
						const v = lo32(st[sp - 1]);
						if (v < cv.getInt32(a1, true) || v > cv.getInt32(a1 + 4, true)) throw new TrapError(Trap.RANGE);
						break;
					}
					case Op.CHK_U32: {
						need(1);
						const v = u32(st[sp - 1]);
						if (v < cv.getUint32(a1, true) || v > cv.getUint32(a1 + 4, true)) throw new TrapError(Trap.RANGE);
						break;
					}
					case Op.CHK_I64: {
						need(1);
						const v = s64(st[sp - 1]);
						if (v < cv.getBigInt64(a1, true) || v > cv.getBigInt64(a1 + 8, true)) throw new TrapError(Trap.RANGE);
						break;
					}
					case Op.CHK_U64: {
						need(1);
						const v = u64(st[sp - 1]);
						if (v < cv.getBigUint64(a1, true) || v > cv.getBigUint64(a1 + 8, true)) throw new TrapError(Trap.RANGE);
						break;
					}

					// ---- control ----
					case Op.JMP: jump(); break;
					case Op.JZ: if (u32(pop()) === 0) jump(); break;
					case Op.JNZ: if (u32(pop()) !== 0) jump(); break;
					case Op.CALL:
					case Op.CALL_FB: {
						if (c.calls.length === CALL_DEPTH) throw new TrapError(Trap.CALL_DEPTH);
						let self = c.self;
						if (o === Op.CALL_FB) self = u32(pop());
						c.calls.push({ returnPc: pc, fp: c.fp, self: c.self });
						c.self = self;
						pc = cv.getUint32(a1, true);
						break;
					}
					case Op.ENTER: {
						const frame = cv.getUint32(a1, true);
						if (c.frameTop + frame > c.frameEnd) throw new TrapError(Trap.FRAME_OVERFLOW);
						data.fill(0, c.frameTop, c.frameTop + frame);
						c.fp = c.frameTop;
						c.frameTop += frame;
						break;
					}
					case Op.RET: {
						const k = c.calls.pop();
						if (k === undefined) throw new TrapError(Trap.STACK_UNDERFLOW);
						c.frameTop = c.fp;
						c.fp = k.fp;
						c.self = k.self;
						if (k.returnPc === NO_RETURN) {
							c.sp = sp;
							c.pc = pc;
							return { state: "done", trap: Trap.NONE, trapCode: 0, pc: 0, instructions: count };
						}
						pc = k.returnPc;
						break;
					}
					case Op.TRAP: throw new TrapError(Trap.USER, cv.getUint16(a1, true));
					case Op.SYSCALL: {
						const id: Native = cv.getUint16(a1, true);
						const info = NATIVES[id];
						need(info.args.length);
						if (info.results.length > info.args.length) room(info.results.length - info.args.length);
						const p = sp - info.args.length;
						const x = st[p], y = st[p + 1];
						let r: Slot;
						switch (id) {
							case Native.TIME_MS: r = c.nowMs; break;
							case Native.ABS_I32: r = lo32(x) < 0 ? (0 - u32(x)) >>> 0 : u32(x); break;
							case Native.ABS_I64: r = s64(x) < 0n ? ofU64(-u64(x)) : u64(x); break;
							case Native.ABS_F32: r = ofF32(Math.abs(f32(x))); break;
							case Native.ABS_F64: r = ofF64(Math.abs(f64(x))); break;
							case Native.SQRT_F32: r = ofF32(Math.fround(Math.sqrt(f32(x)))); break;
							case Native.SQRT_F64: r = ofF64(Math.sqrt(f64(x))); break;
							case Native.ROL_8: r = rotl(u32(x), u32(y), 8); break;
							case Native.ROL_16: r = rotl(u32(x), u32(y), 16); break;
							case Native.ROL_32: r = rotl(u32(x), u32(y), 32); break;
							case Native.ROL_64: r = rotl64(u64(x), u32(y)); break;
							case Native.ROR_8: r = rotr(u32(x), u32(y), 8); break;
							case Native.ROR_16: r = rotr(u32(x), u32(y), 16); break;
							case Native.ROR_32: r = rotr(u32(x), u32(y), 32); break;
							case Native.ROR_64: { const k = u32(y) % 64; r = rotl64(u64(x), k === 0 ? 0 : 64 - k); break; }
							case Native.LN_F64: r = ofF64(Math.log(f64(x))); break;
							case Native.LOG_F64: r = ofF64(Math.log10(f64(x))); break;
							case Native.EXP_F64: r = ofF64(Math.exp(f64(x))); break;
							case Native.EXPT_F64: r = ofF64(Math.pow(f64(x), f64(y))); break;
							case Native.SIN_F64: r = ofF64(Math.sin(f64(x))); break;
							case Native.COS_F64: r = ofF64(Math.cos(f64(x))); break;
							case Native.TAN_F64: r = ofF64(Math.tan(f64(x))); break;
							case Native.ASIN_F64: r = ofF64(Math.asin(f64(x))); break;
							case Native.ACOS_F64: r = ofF64(Math.acos(f64(x))); break;
							case Native.ATAN_F64: r = ofF64(Math.atan(f64(x))); break;
							default: throw new TrapError(Trap.NATIVE);
						}
						st[p] = r;
						sp = p + info.results.length;
						break;
					}
					default: throw new TrapError(Trap.USER); // unreachable: the verifier accepts known opcodes only
				}
			}
		} catch (e) {
			if (!(e instanceof TrapError)) throw e;
			c.sp = sp;
			c.pc = at;
			return { state: "trapped", trap: e.trap, trapCode: e.code, pc: at, instructions: count };
		}
	}
}
