// SPDX-License-Identifier: MIT
// The PLC image (rtoos/docs/plc.md "Image format"): constants, CRC-32 and the writer the compiler
// uses. The C++ counterpart is rtoos/include/rtos/plc/image.hpp. DOM-free.
import { ISA_HASH, ISA_VERSION, NATIVES, type Native } from "./generated/isa.ts";

export const IMAGE_MAGIC = [0x46, 0x43, 0x55, 0x50, 0x4c, 0x43, 0, 1] as const; // "FCUPLC\0\1"
export const HEADER_SIZE = 88;
export const TASK_RECORD_SIZE = 32;
export const PROGRAM_RECORD_SIZE = 8;
export const IO_RECORD_SIZE = 12;
export const NATIVE_RECORD_SIZE = 8;

export const Hdr = {
	isaVersion: 8,
	headerSize: 10,
	length: 12,
	crc: 16,
	isaHash: 20,
	ioMapHash: 24,
	projectHash: 28,
	compilerVersion: 32,
	codeOffset: 36,
	codeSize: 40,
	dataSize: 44,
	initOffset: 48,
	initSize: 52,
	taskOffset: 56,
	taskCount: 60,
	ioOffset: 64,
	ioCount: 68,
	nativeOffset: 72,
	nativeCount: 76,
	statementOffset: 80,
	statementCount: 84,
} as const;

/** Same names and order as rtos::plc::LoadError. */
export const LOAD_ERRORS = [
	"none", "too_short", "magic", "header_size", "crc", "isa_version", "isa_hash", "section",
	"code_too_large", "data_too_large", "init_range", "task_range", "program_range", "io_range",
	"bad_opcode", "truncated", "jump_target", "call_target", "data_address", "native_unknown",
	"native_signature", "native_not_listed", "statement_range",
] as const;
export type LoadError = (typeof LOAD_ERRORS)[number];

export interface ProgramInfo {
	entry: number; // code offset of an ENTER
	instance: number; // data address
}

export interface TaskInfo {
	interval: number; // in I/O cycles
	priority: number;
	maxOverruns: number;
	budget: number; // instructions per cycle
	frameBase: number;
	frameSize: number;
	programs: ProgramInfo[];
	ioFirst: number;
	ioCount: number;
}

export const enum IoArea { Input = 0, Holding = 1 }
export const enum IoKind { Input = 0, Output = 1, Command = 2, Parameter = 3 }

export interface IoBinding {
	reg: number;
	area: IoArea;
	kind: IoKind;
	bits: 1 | 16 | 32;
	signed: boolean;
	addr: number;
}

export interface ImageContents {
	code: Uint8Array;
	dataSize: number;
	init: { addr: number; bytes: Uint8Array }[];
	tasks: TaskInfo[];
	io: IoBinding[];
	natives: Native[];
	/** Code offsets where a source statement begins (single step); sorted and unique by the writer. */
	statements?: number[];
	ioMapHash: number;
	projectHash: number;
	compilerVersion: number;
	/** Only for tests of the verifier. */
	isaVersion?: number;
	isaHash?: number;
}

// ---- CRC-32 (IEEE 802.3, reflected, as zlib) --------------------------------------------------
const CRC_TABLE = (() => {
	const t = new Uint32Array(256);
	for (let i = 0; i < 256; i++) {
		let c = i;
		for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
		t[i] = c >>> 0;
	}
	return t;
})();

export function crc32(bytes: Uint8Array, crc = 0): number {
	crc = ~crc >>> 0;
	for (let i = 0; i < bytes.length; i++) crc = CRC_TABLE[(crc ^ bytes[i]) & 0xff] ^ (crc >>> 8);
	return ~crc >>> 0;
}

// ---- writer -----------------------------------------------------------------------------------
class ByteWriter {
	private buf = new Uint8Array(1024);
	length = 0;
	private reserve(n: number): void {
		if (this.length + n <= this.buf.length) return;
		let size = this.buf.length * 2;
		while (size < this.length + n) size *= 2;
		const b = new Uint8Array(size);
		b.set(this.buf.subarray(0, this.length));
		this.buf = b;
	}
	u8(v: number): void {
		this.reserve(1);
		this.buf[this.length++] = v & 0xff;
	}
	u16(v: number): void {
		this.u8(v);
		this.u8(v >>> 8);
	}
	u32(v: number): void {
		this.u16(v & 0xffff);
		this.u16(v >>> 16);
	}
	bytes(b: Uint8Array): void {
		this.reserve(b.length);
		this.buf.set(b, this.length);
		this.length += b.length;
	}
	align4(): void {
		while (this.length % 4 !== 0) this.u8(0);
	}
	put32(at: number, v: number): void {
		new DataView(this.buf.buffer).setUint32(at, v >>> 0, true);
	}
	result(): Uint8Array {
		return this.buf.slice(0, this.length);
	}
}

/** Writes an image; the same contents always give the same bytes. */
export function writeImage(c: ImageContents): Uint8Array {
	const w = new ByteWriter();
	for (let i = 0; i < HEADER_SIZE; i++) w.u8(0);

	w.put32(Hdr.codeOffset, w.length);
	w.put32(Hdr.codeSize, c.code.length);
	w.bytes(c.code);
	w.align4();
	w.put32(Hdr.dataSize, c.dataSize);

	const initStart = w.length;
	w.put32(Hdr.initOffset, initStart);
	for (const r of c.init) {
		w.u32(r.addr);
		w.u32(r.bytes.length);
		w.bytes(r.bytes);
		w.align4();
	}
	w.put32(Hdr.initSize, w.length - initStart);

	const lists: number[] = [];
	for (const t of c.tasks) {
		lists.push(w.length);
		for (const p of t.programs) {
			w.u32(p.entry);
			w.u32(p.instance);
		}
	}
	w.put32(Hdr.taskOffset, w.length);
	w.put32(Hdr.taskCount, c.tasks.length);
	c.tasks.forEach((t, i) => {
		w.u16(t.interval);
		w.u8(t.priority);
		w.u8(t.maxOverruns);
		w.u32(t.budget);
		w.u32(t.frameBase);
		w.u32(t.frameSize);
		w.u32(lists[i]);
		w.u32(t.programs.length);
		w.u32(t.ioFirst);
		w.u32(t.ioCount);
	});

	w.put32(Hdr.ioOffset, w.length);
	w.put32(Hdr.ioCount, c.io.length);
	for (const b of c.io) {
		w.u16(b.reg);
		w.u8(b.area);
		w.u8(b.kind);
		w.u8(b.bits);
		w.u8(b.signed ? 1 : 0);
		w.u16(0);
		w.u32(b.addr);
	}

	w.put32(Hdr.nativeOffset, w.length);
	w.put32(Hdr.nativeCount, c.natives.length);
	for (const n of c.natives) {
		w.u16(n);
		w.u16(0);
		w.u32(NATIVES[n].signature);
	}

	const statements = [...new Set(c.statements ?? [])].sort((a, b) => a - b);
	w.put32(Hdr.statementOffset, w.length);
	w.put32(Hdr.statementCount, statements.length);
	for (const s of statements) w.u32(s);

	const img = w.result();
	const dv = new DataView(img.buffer);
	img.set(IMAGE_MAGIC, 0);
	dv.setUint16(Hdr.isaVersion, c.isaVersion ?? ISA_VERSION, true);
	dv.setUint16(Hdr.headerSize, HEADER_SIZE, true);
	dv.setUint32(Hdr.length, img.length, true);
	dv.setUint32(Hdr.isaHash, (c.isaHash ?? ISA_HASH) >>> 0, true);
	dv.setUint32(Hdr.ioMapHash, c.ioMapHash >>> 0, true);
	dv.setUint32(Hdr.projectHash, c.projectHash >>> 0, true);
	dv.setUint32(Hdr.compilerVersion, c.compilerVersion >>> 0, true);
	dv.setUint32(Hdr.crc, crc32(img.subarray(Hdr.crc + 4)), true);
	return img;
}
