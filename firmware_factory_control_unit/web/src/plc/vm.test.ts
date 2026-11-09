// The TypeScript PLC VM (vm.ts): the same cases and expected values as rtoos/tests/plc_vm_test.cpp,
// so both VMs are held to one specification (rtoos/docs/plc.md).
import { describe, expect, it } from "vitest";
import { ISA_HASH, ISA_VERSION, Native, Op, Trap } from "./generated/isa.ts";
import { Hdr, crc32, writeImage, type ImageContents, type ProgramInfo } from "./image.ts";
import { Vm } from "./vm.ts";

class Asm {
	bytes: number[] = [];
	get here(): number {
		return this.bytes.length;
	}
	op(o: Op): this {
		this.bytes.push(o);
		return this;
	}
	u16(v: number): this {
		this.bytes.push(v & 0xff, (v >>> 8) & 0xff);
		return this;
	}
	u32(v: number): this {
		return this.u16(v & 0xffff).u16(v >>> 16);
	}
	u64(v: bigint): this {
		const x = BigInt.asUintN(64, v);
		return this.u32(Number(x & 0xffffffffn)).u32(Number(x >> 32n));
	}
	push(v: number): this {
		return this.op(Op.PUSH_I32).u32(v >>> 0);
	}
	push64(v: bigint): this {
		return this.op(Op.PUSH_I64).u64(v);
	}
	pushf(f: number): this {
		const dv = new DataView(new ArrayBuffer(4));
		dv.setFloat32(0, f, true);
		return this.op(Op.PUSH_I32).u32(dv.getUint32(0, true));
	}
	pushd(d: number): this {
		const dv = new DataView(new ArrayBuffer(8));
		dv.setFloat64(0, d, true);
		return this.op(Op.PUSH_I64).u64(dv.getBigUint64(0, true));
	}
	enter(frame: number): this {
		return this.op(Op.ENTER).u32(frame);
	}
	stg32(a: number): this {
		return this.op(Op.STG_32).u32(a);
	}
	stg64(a: number): this {
		return this.op(Op.STG_64).u32(a);
	}
	ldg32(a: number): this {
		return this.op(Op.LDG_32).u32(a);
	}
	sys(n: Native): this {
		return this.op(Op.SYSCALL).u16(n);
	}
	patch32(at: number, v: number): void {
		this.bytes[at] = v & 0xff;
		this.bytes[at + 1] = (v >>> 8) & 0xff;
		this.bytes[at + 2] = (v >>> 16) & 0xff;
		this.bytes[at + 3] = (v >>> 24) & 0xff;
	}
}

function contents(code: number[], natives: Native[] = [], programs: ProgramInfo[] = [{ entry: 0, instance: 0 }]): ImageContents {
	return {
		code: Uint8Array.from(code),
		dataSize: 256,
		init: [],
		tasks: [{ interval: 1, priority: 0, maxOverruns: 3, budget: 10_000, frameBase: 128, frameSize: 128, programs, ioFirst: 0, ioCount: 0 }],
		io: [],
		natives,
		ioMapHash: 0,
		projectHash: 0,
		compilerVersion: 0,
	};
}

/** ENTER 0, body, RET. */
function program(body: Asm, natives: Native[] = []): ImageContents {
	const a = new Asm().enter(0);
	a.bytes.push(...body.bytes);
	a.op(Op.RET);
	return contents(a.bytes, natives);
}

class Machine {
	data = new Uint8Array(1024);
	vm = new Vm(this.data, 2);
	dv = new DataView(this.data.buffer);
	load(c: ImageContents) {
		return this.vm.load(writeImage(c));
	}
	g32(a: number) {
		return this.dv.getUint32(a, true);
	}
	s32(a: number) {
		return this.dv.getInt32(a, true);
	}
	g64(a: number) {
		return this.dv.getBigUint64(a, true);
	}
	s64(a: number) {
		return this.dv.getBigInt64(a, true);
	}
	gf(a: number) {
		return this.dv.getFloat32(a, true);
	}
	gd(a: number) {
		return this.dv.getFloat64(a, true);
	}
}

function run(m: Machine, body: Asm, natives: Native[] = []) {
	expect(m.load(program(body, natives))).toBe("none");
	return m.vm.runCycle(0, 1234);
}

describe("PLC VM: integers", () => {
	it("keeps globals across cycles", () => {
		const m = new Machine();
		expect(m.load(program(new Asm().ldg32(0).push(1).op(Op.ADD_I32).stg32(0)))).toBe("none");
		for (let i = 0; i < 5; i++) expect(m.vm.runCycle(0, 0).state).toBe("done");
		expect(m.g32(0)).toBe(5);
	});

	it("initializes the data segment", () => {
		const m = new Machine();
		m.data.fill(0xaa);
		const c = program(new Asm());
		c.init = [{ addr: 8, bytes: Uint8Array.of(1, 2, 3) }, { addr: 16, bytes: Uint8Array.of(0x78, 0x56, 0x34, 0x12) }];
		expect(m.load(c)).toBe("none");
		expect([m.data[0], m.data[8], m.data[10], m.data[11], m.g32(16), m.data[255], m.data[256]]).toEqual([0, 1, 3, 0, 0x12345678, 0, 0xaa]);
	});

	it("wraps and normalizes narrow types", () => {
		const m = new Machine();
		const a = new Asm();
		a.push(0x7fffffff).push(1).op(Op.ADD_I32).stg32(0);
		a.push(127).push(1).op(Op.ADD_I32).op(Op.SEXT8).stg32(4);
		a.push(255).push(1).op(Op.ADD_I32).op(Op.ZEXT8).stg32(8);
		a.push(-32768).op(Op.NEG_I32).op(Op.SEXT16).stg32(12);
		a.push64(0x7fffffffffffffffn).push64(1n).op(Op.ADD_I64).stg64(16);
		expect(run(m, a).state).toBe("done");
		expect([m.g32(0), m.s32(4), m.g32(8), m.s32(12), m.g64(16)]).toEqual([0x80000000, -128, 0, -32768, 0x8000000000000000n]);
	});

	it("divides toward zero, MIN / -1 = MIN", () => {
		const m = new Machine();
		const a = new Asm();
		a.push(-7).push(2).op(Op.DIVS_I32).stg32(0);
		a.push(-7).push(2).op(Op.MODS_I32).stg32(4);
		a.push(-2147483648).push(-1).op(Op.DIVS_I32).stg32(8);
		a.push(-2147483648).push(-1).op(Op.MODS_I32).stg32(12);
		a.push(-1).push(2).op(Op.DIVU_I32).stg32(16);
		a.push64(0x8000000000000000n).push64(-1n).op(Op.DIVS_I64).stg64(24);
		expect(run(m, a).state).toBe("done");
		expect([m.s32(0), m.s32(4), m.g32(8), m.g32(12), m.g32(16), m.g64(24)]).toEqual([-3, -1, 0x80000000, 0, 0x7fffffff, 0x8000000000000000n]);
	});

	it("traps division by zero at the instruction", () => {
		const m = new Machine();
		const a = new Asm().push(1).push(0);
		const at = a.here + 5;
		a.op(Op.DIVS_I32).stg32(0);
		const r = run(m, a);
		expect([r.state, r.trap, r.pc]).toEqual(["trapped", Trap.DIV_BY_ZERO, at]);
		expect(run(new Machine(), new Asm().pushf(1).pushf(-0).op(Op.DIV_F32).stg32(0)).trap).toBe(Trap.DIV_BY_ZERO);
	});

	it("shifts by the width or more give 0", () => {
		const m = new Machine();
		const a = new Asm();
		a.push(1).push(31).op(Op.SHL_I32).stg32(0);
		a.push(1).push(32).op(Op.SHL_I32).stg32(4);
		a.push(-1).push(40).op(Op.SHR_I32).stg32(8);
		a.push(-1).push(4).op(Op.SHR_I32).stg32(12);
		a.push64(1n).push64(64n).op(Op.SHL_I64).stg64(16);
		expect(run(m, a).state).toBe("done");
		expect([m.g32(0), m.g32(4), m.g32(8), m.g32(12), m.g64(16)]).toEqual([0x80000000, 0, 0, 0x0fffffff, 0n]);
	});

	it("compares signed and unsigned", () => {
		const m = new Machine();
		const a = new Asm();
		a.push(-1).push(1).op(Op.LTS_I32).stg32(0);
		a.push(-1).push(1).op(Op.LTU_I32).stg32(4);
		a.push64(-1n).push64(0n).op(Op.GTS_I64).stg32(8);
		a.push64(-1n).push64(0n).op(Op.GTU_I64).stg32(12);
		expect(run(m, a).state).toBe("done");
		expect([m.g32(0), m.g32(4), m.g32(8), m.g32(12)]).toEqual([1, 0, 0, 1]);
	});
});

describe("PLC VM: floating point", () => {
	it("makes NaN canonical", () => {
		const m = new Machine();
		const a = new Asm();
		a.pushf(-1).sys(Native.SQRT_F32).stg32(0);
		a.pushd(-1).sys(Native.SQRT_F64).stg64(8);
		a.pushf(Infinity).pushf(Infinity).op(Op.SUB_F32).stg32(16);
		a.op(Op.PUSH_I32).u32(0xffc00001).op(Op.NEG_F32).stg32(20);
		expect(run(m, a, [Native.SQRT_F32, Native.SQRT_F64]).state).toBe("done");
		expect([m.g32(0), m.g64(8), m.g32(16), m.g32(20)]).toEqual([0x7fc00000, 0x7ff8000000000000n, 0x7fc00000, 0x7fc00000]);
	});

	it("rounds single precision once per operation", () => {
		const m = new Machine();
		const a = new Asm();
		a.pushf(16777216).pushf(1).op(Op.ADD_F32).stg32(0);
		a.pushf(0.1).op(Op.F32_TO_F64).stg64(8);
		expect(run(m, a).state).toBe("done");
		expect(m.gf(0)).toBe(16777216);
		expect(m.gd(8)).toBe(Math.fround(0.1));
	});

	it("converts 64-bit integers to REAL with a single rounding", () => {
		// 2^60 + 2^36 + 1: exactly halfway between two floats after the double rounding of Number() -
		// Math.fround(Number(v)) would round to even downward, correct is upward (the +1 decides)
		const v = (1n << 60n) + (1n << 36n) + 1n;
		const m = new Machine();
		expect(run(m, new Asm().push64(v).op(Op.I64S_TO_F32).stg32(0)).state).toBe("done");
		expect(m.gf(0)).toBe(2 ** 60 + 2 ** 37);
	});

	it("rounds half away from zero and truncates", () => {
		const m = new Machine();
		const a = new Asm();
		a.pushf(2.5).op(Op.F32_ROUND_I64).stg64(0);
		a.pushf(-2.5).op(Op.F32_ROUND_I64).stg64(8);
		a.pushd(-2.7).op(Op.F64_TRUNC_I64).stg64(16);
		a.pushd(0.49999999999999994).op(Op.F64_ROUND_I64).stg64(24);
		expect(run(m, a).state).toBe("done");
		expect([m.s64(0), m.s64(8), m.s64(16), m.s64(24)]).toEqual([3n, -3n, -2n, 0n]);
	});

	it("traps conversions out of range or of NaN", () => {
		for (const x of [9.3e18, -9.3e18, NaN]) {
			expect(run(new Machine(), new Asm().pushd(x).op(Op.F64_TRUNC_I64).stg64(0)).trap).toBe(Trap.CONVERSION);
		}
		const m = new Machine();
		expect(run(m, new Asm().pushd(-(2 ** 63)).op(Op.F64_TRUNC_I64).stg64(0)).state).toBe("done");
		expect(m.g64(0)).toBe(0x8000000000000000n);
	});

	it("range checks keep the value", () => {
		const m = new Machine();
		expect(run(m, new Asm().push(5).op(Op.CHK_I32).u32(-10 >>> 0).u32(10).stg32(0)).state).toBe("done");
		expect(m.g32(0)).toBe(5);
		expect(run(new Machine(), new Asm().push(70000).op(Op.CHK_I32).u32(-32768 >>> 0).u32(32767).stg32(0)).trap).toBe(Trap.RANGE);
	});
});

describe("PLC VM: memory and calls", () => {
	it("loads and stores every width and sign", () => {
		const m = new Machine();
		const a = new Asm();
		a.push(0x1ff).op(Op.STG_8).u32(0);
		a.op(Op.LDG_S8).u32(0).stg32(4);
		a.op(Op.LDG_U8).u32(0).stg32(8);
		a.push(0x18000).op(Op.STG_16).u32(12);
		a.op(Op.LDG_S16).u32(12).stg32(16);
		a.op(Op.ADDR_G).u32(20).push(77).op(Op.STI_32);
		a.op(Op.ADDR_G).u32(20).op(Op.LDI_32).stg32(24);
		expect(run(m, a).state).toBe("done");
		expect([m.s32(4), m.g32(8), m.data[14], m.s32(16), m.g32(24)]).toEqual([-1, 255, 0, -32768, 77]);
	});

	it("traps indirect accesses outside the data segment", () => {
		for (const addr of [253, 256, 0xfffffffc]) {
			expect(run(new Machine(), new Asm().push(addr).op(Op.LDI_32).stg32(0)).trap).toBe(Trap.MEMORY);
		}
		expect(run(new Machine(), new Asm().push(0).push(250).op(Op.MEMCPY).u32(8)).trap).toBe(Trap.MEMORY);
	});

	it("calls a function with arguments and a result", () => {
		const a = new Asm().enter(0).push(10).push(3);
		const call = a.here + 1;
		a.op(Op.CALL).u32(0).stg32(0).op(Op.RET);
		const fn = a.here;
		a.enter(8).op(Op.STL_32).u16(4).op(Op.STL_32).u16(0);
		a.op(Op.LDL_32).u16(0).op(Op.LDL_32).u16(4).op(Op.SUB_I32).op(Op.RET);
		a.patch32(call, fn);
		const m = new Machine();
		expect(m.load(contents(a.bytes))).toBe("none");
		expect(m.vm.runCycle(0, 0).state).toBe("done");
		expect(m.g32(0)).toBe(7);
	});

	it("gives a function block its instance as THIS", () => {
		const a = new Asm().enter(0);
		const calls: number[] = [];
		for (const inst of [32, 48, 48]) {
			a.op(Op.ADDR_G).u32(inst);
			calls.push(a.here + 1);
			a.op(Op.CALL_FB).u32(0);
		}
		a.op(Op.RET);
		const fb = a.here;
		a.enter(0).op(Op.LDT_32).u16(4).push(1).op(Op.ADD_I32).op(Op.STT_32).u16(4).op(Op.RET);
		for (const c of calls) a.patch32(c, fb);
		const m = new Machine();
		expect(m.load(contents(a.bytes))).toBe("none");
		expect(m.vm.runCycle(0, 0).state).toBe("done");
		expect([m.g32(36), m.g32(52)]).toEqual([1, 2]);
	});

	it("traps endless recursion", () => {
		const m = new Machine();
		expect(m.load(contents(new Asm().enter(0).op(Op.CALL).u32(0).op(Op.RET).bytes))).toBe("none");
		expect(m.vm.runCycle(0, 0).trap).toBe(Trap.CALL_DEPTH);
		const n = new Machine();
		expect(n.load(contents(new Asm().enter(16).op(Op.CALL).u32(0).op(Op.RET).bytes))).toBe("none");
		expect(n.vm.runCycle(0, 0).trap).toBe(Trap.FRAME_OVERFLOW);
	});
});

describe("PLC VM: budget, natives", () => {
	it("stops an endless loop at the budget", () => {
		const a = new Asm().enter(0);
		a.op(Op.JMP).u32(-5 >>> 0).op(Op.RET);
		const c = contents(a.bytes);
		c.tasks[0].budget = 1000;
		const m = new Machine();
		expect(m.load(c)).toBe("none");
		const r = m.vm.runCycle(0, 0);
		expect([r.trap, r.instructions]).toEqual([Trap.BUDGET, 1000]);
	});

	it("time and rotation", () => {
		const m = new Machine();
		const a = new Asm();
		a.sys(Native.TIME_MS).stg32(0);
		a.push(0x81).push(1).sys(Native.ROL_8).stg32(4);
		a.push(0x81).push(9).sys(Native.ROR_8).stg32(8);
		a.push(1).push(0).sys(Native.ROL_32).stg32(12);
		a.push(-2147483648).sys(Native.ABS_I32).stg32(16);
		expect(run(m, a, [Native.TIME_MS, Native.ROL_8, Native.ROR_8, Native.ROL_32, Native.ABS_I32]).state).toBe("done");
		expect([m.g32(0), m.g32(4), m.g32(8), m.g32(12), m.g32(16)]).toEqual([1234, 0x03, 0xc0, 1, 0x80000000]);
	});

	it("TRAP carries its code, the operand stack overflows", () => {
		const r = run(new Machine(), new Asm().op(Op.TRAP).u16(42));
		expect([r.trap, r.trapCode]).toEqual([Trap.USER, 42]);
		const a = new Asm();
		for (let i = 0; i <= 64; i++) a.push(0);
		expect(run(new Machine(), a).trap).toBe(Trap.STACK_OVERFLOW);
	});
});

describe("PLC verifier", () => {
	it("accepts only intact images for this instruction set", () => {
		const m = new Machine();
		const img = writeImage(program(new Asm()));
		expect(m.vm.load(img)).toBe("none");
		const bad = img.slice();
		bad[bad.length - 1] ^= 1;
		expect(m.vm.load(bad)).toBe("crc");
		expect(m.vm.loaded).toBe(false);
		expect(m.vm.load(img.subarray(0, 40))).toBe("too_short");
		expect(m.load({ ...program(new Asm()), isaHash: (ISA_HASH ^ 1) >>> 0 })).toBe("isa_hash");
		expect(m.load({ ...program(new Asm()), isaVersion: ISA_VERSION + 1 })).toBe("isa_version");
	});

	it("rejects bad code", () => {
		const m = new Machine();
		expect(m.load(contents([Op.ENTER, 0, 0, 0, 0, 0xff]))).toBe("bad_opcode");
		expect(m.load(contents([Op.ENTER, 0, 0, 0, 0, Op.PUSH_I32, 1, 2]))).toBe("truncated");
		expect(m.load(contents([Op.ENTER, 0, 0, 0, 0, Op.NOP]))).toBe("truncated");
	});

	it("jumps hit instruction starts, calls an ENTER", () => {
		const m = new Machine();
		expect(m.load(contents(new Asm().enter(0).op(Op.JMP).u32(-3 >>> 0).op(Op.RET).bytes))).toBe("jump_target");
		expect(m.load(contents(new Asm().enter(0).op(Op.CALL).u32(5).op(Op.RET).bytes))).toBe("call_target");
		expect(m.load(contents(new Asm().op(Op.NOP).enter(0).op(Op.RET).bytes))).toBe("call_target");
	});

	it("checks global addresses and natives", () => {
		const m = new Machine();
		expect(m.load(program(new Asm().ldg32(253).op(Op.DROP)))).toBe("data_address");
		const s = new Asm().sys(Native.TIME_MS).op(Op.DROP);
		expect(m.load(program(s))).toBe("native_not_listed");
		expect(m.load(program(s, [Native.TIME_MS]))).toBe("none");
		const img = writeImage(program(new Asm(), [Native.TIME_MS]));
		const dv = new DataView(img.buffer);
		const nat = dv.getUint32(Hdr.nativeOffset, true);
		img[nat + 4] ^= 1;
		dv.setUint32(Hdr.crc, crc32(img.subarray(Hdr.crc + 4)), true);
		expect(m.vm.load(img)).toBe("native_signature");
		img[nat] = 0xff;
		dv.setUint32(Hdr.crc, crc32(img.subarray(Hdr.crc + 4)), true);
		expect(m.vm.load(img)).toBe("native_unknown");
	});

	it("checks the memory layout", () => {
		const m = new Machine();
		expect(m.load({ ...program(new Asm()), dataSize: 2048 })).toBe("data_too_large");
		expect(m.load({ ...program(new Asm()), init: [{ addr: 250, bytes: Uint8Array.of(1, 2, 3, 4, 5, 6, 7) }] })).toBe("init_range");
		const c = program(new Asm());
		c.tasks.push({ ...c.tasks[0] });
		expect(m.load(c)).toBe("task_range");
		c.tasks[1].frameBase = 0;
		expect(m.load(c)).toBe("none");
		c.tasks.push({ ...c.tasks[1], frameBase: 64, frameSize: 32 });
		c.tasks[1].frameSize = 64;
		expect(m.load(c)).toBe("task_range");
	});
});
