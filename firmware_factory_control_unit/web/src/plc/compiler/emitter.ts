// SPDX-License-Identifier: MIT
// Bytecode assembler: instructions, labels, jump and call fixups. DOM-free.
import { OPS, Op } from "../generated/isa.ts";

export interface Label {
	id: number;
	at: number | null;
}

export class Emitter {
	private bytes: number[] = [];
	private jumpFixups: { operand: number; label: Label }[] = [];
	private callFixups: { operand: number; target: string }[] = [];
	private nextLabel = 0;

	get here(): number {
		return this.bytes.length;
	}

	label(): Label {
		return { id: this.nextLabel++, at: null };
	}
	bind(l: Label): void {
		l.at = this.here;
	}

	op(o: Op): void {
		this.bytes.push(o);
	}
	u8(v: number): void {
		this.bytes.push(v & 0xff);
	}
	u16(v: number): void {
		this.bytes.push(v & 0xff, (v >>> 8) & 0xff);
	}
	u32(v: number): void {
		this.u16(v & 0xffff);
		this.u16((v >>> 16) & 0xffff);
	}
	u64(v: bigint): void {
		const x = BigInt.asUintN(64, v);
		this.u32(Number(x & 0xffffffffn));
		this.u32(Number(x >> 32n));
	}

	/** op with one u16/u32 operand, by the opcode's operand table. */
	opArg(o: Op, v: number): void {
		this.op(o);
		const kind = OPS[o].operands[0];
		if (kind === "u16") this.u16(v);
		else if (kind === "i8") this.u8(v);
		else this.u32(v);
	}

	pushI32(raw: number): void {
		const s = raw | 0;
		if (s >= -128 && s <= 127) {
			this.op(Op.PUSH_I8);
			this.u8(s);
		} else {
			this.op(Op.PUSH_I32);
			this.u32(raw >>> 0);
		}
	}
	pushI64(raw: bigint): void {
		this.op(Op.PUSH_I64);
		this.u64(raw);
	}

	jump(o: Op.JMP | Op.JZ | Op.JNZ, l: Label): void {
		this.op(o);
		this.jumpFixups.push({ operand: this.here, label: l });
		this.u32(0);
	}

	/** CALL or CALL_FB of the body named `target` (resolved by finish()). */
	call(o: Op.CALL | Op.CALL_FB, target: string): void {
		this.op(o);
		this.callFixups.push({ operand: this.here, target });
		this.u32(0);
	}

	/** Overwrites the u32 at `at` (an operand written earlier). */
	patch32(at: number, v: number): void {
		this.bytes[at] = v & 0xff;
		this.bytes[at + 1] = (v >>> 8) & 0xff;
		this.bytes[at + 2] = (v >>> 16) & 0xff;
		this.bytes[at + 3] = (v >>> 24) & 0xff;
	}

	/** Resolves the fixups; `entries`: code offset of every body. */
	finish(entries: Map<string, number>): Uint8Array {
		for (const f of this.jumpFixups) {
			if (f.label.at === null) throw new Error(`internal: label ${f.label.id} not bound`);
			this.patch32(f.operand, f.label.at - (f.operand + 4));
		}
		for (const f of this.callFixups) {
			const e = entries.get(f.target);
			if (e === undefined) throw new Error(`internal: no body for ${f.target}`);
			this.patch32(f.operand, e);
		}
		return Uint8Array.from(this.bytes);
	}
}
