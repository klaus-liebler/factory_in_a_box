// Reading, writing and formatting PLC variables in a VM's data segment (simulator, watch lists,
// inline values).
import type { Symbol } from "../compiler/compiler.ts";
import { ELEM, elemOf, typeName, type ElemName, type Type } from "../compiler/types.ts";

export type Value = number | bigint | boolean;

export function isScalar(t: Type): boolean {
	return elemOf(t) !== null;
}

export function readValue(data: Uint8Array, s: Symbol): Value | null {
	const e = elemOf(s.type);
	if (e === null) return null;
	const dv = new DataView(data.buffer, data.byteOffset, data.byteLength);
	const a = s.addr;
	const info = ELEM[e];
	if (e === "BOOL") return data[a] !== 0;
	if (e === "REAL") return dv.getFloat32(a, true);
	if (e === "LREAL") return dv.getFloat64(a, true);
	switch (info.size) {
		case 1: return info.signed && info.category !== "bit" ? dv.getInt8(a) : dv.getUint8(a);
		case 2: return info.signed && info.category !== "bit" ? dv.getInt16(a, true) : dv.getUint16(a, true);
		case 4: return info.signed && info.category !== "bit" ? dv.getInt32(a, true) : dv.getUint32(a, true);
		default: return info.signed && info.category !== "bit" ? dv.getBigInt64(a, true) : dv.getBigUint64(a, true);
	}
}

export function writeValue(data: Uint8Array, s: Symbol, v: Value): void {
	const e = elemOf(s.type);
	if (e === null) return;
	const dv = new DataView(data.buffer, data.byteOffset, data.byteLength);
	const a = s.addr;
	const n = typeof v === "boolean" ? (v ? 1 : 0) : v;
	if (e === "REAL") return dv.setFloat32(a, Number(n), true);
	if (e === "LREAL") return dv.setFloat64(a, Number(n), true);
	switch (ELEM[e].size) {
		case 1: return dv.setUint8(a, Number(BigInt.asUintN(8, BigInt(Math.trunc(Number(n))))));
		case 2: return dv.setUint16(a, Number(BigInt.asUintN(16, BigInt(Math.trunc(Number(n))))), true);
		case 4: return dv.setUint32(a, Number(BigInt.asUintN(32, BigInt(Math.trunc(Number(n))))), true);
		default: return dv.setBigUint64(a, BigInt.asUintN(64, typeof n === "bigint" ? n : BigInt(Math.trunc(n))), true);
	}
}

export function formatTime(ms: number): string {
	if (ms === 0) return "T#0ms";
	const sign = ms < 0 ? "-" : "";
	let r = Math.abs(ms);
	const parts: string[] = [];
	for (const [unit, size] of [["d", 86_400_000], ["h", 3_600_000], ["m", 60_000], ["s", 1000]] as const) {
		if (r >= size) {
			parts.push(`${Math.floor(r / size)}${unit}`);
			r %= size;
		}
	}
	if (r > 0) parts.push(`${r}ms`);
	return `T#${sign}${parts.join("")}`;
}

export function formatValue(t: Type, v: Value | null): string {
	if (v === null) return "";
	const e = elemOf(t);
	if (t.k === "enum") {
		for (const x of t.values.values()) if (x.value === BigInt(v as number)) return x.name;
		return String(v);
	}
	if (e === "BOOL") return v ? "TRUE" : "FALSE";
	if (e === "TIME") return formatTime(Number(v));
	if (e === "REAL" || e === "LREAL") {
		const x = Number(v);
		if (Number.isNaN(x)) return "NaN";
		if (!Number.isFinite(x)) return x > 0 ? "+Inf" : "-Inf";
		return Number.isInteger(x) && Math.abs(x) < 1e15 ? x.toFixed(1) : Number(x.toPrecision(e === "REAL" ? 7 : 15)).toString();
	}
	if (e && ELEM[e].category === "bit") return `16#${BigInt(v as number).toString(16).toUpperCase().padStart(ELEM[e].size * 2, "0")}`;
	return String(v);
}

/** Parses what the user typed for a variable of type t (TRUE, 12, 3.5, T#2s, 16#FF). */
export function parseValue(t: Type, text: string): Value | null {
	const s = text.trim().toUpperCase();
	const e: ElemName | null = elemOf(t);
	if (e === null) return null;
	if (t.k === "enum") {
		for (const [k, x] of t.values) if (k === s) return Number(x.value);
	}
	if (e === "BOOL") {
		if (s === "TRUE" || s === "1") return true;
		if (s === "FALSE" || s === "0") return false;
		return null;
	}
	if (e === "TIME") {
		const m = /^(?:T|TIME)#(-?)(.*)$/.exec(s);
		if (!m) return /^-?\d+$/.test(s) ? Number(s) : null;
		let total = 0;
		const re = /(\d+(?:\.\d+)?)(MS|D|H|M|S)/gy;
		let at = 0;
		const body = m[2].replace(/_/g, "");
		while (at < body.length) {
			re.lastIndex = at;
			const x = re.exec(body);
			if (!x) return null;
			total += Number(x[1]) * { D: 86_400_000, H: 3_600_000, M: 60_000, S: 1000, MS: 1 }[x[2] as "D"];
			at = re.lastIndex;
		}
		return (m[1] ? -1 : 1) * Math.round(total);
	}
	if (e === "REAL" || e === "LREAL") {
		const x = Number(s);
		return Number.isNaN(x) ? null : x;
	}
	const based = /^(2|8|16)#([0-9A-F_]+)$/.exec(s);
	let v: bigint;
	try {
		v = based ? BigInt((based[1] === "2" ? "0b" : based[1] === "8" ? "0o" : "0x") + based[2].replace(/_/g, "")) : BigInt(s.replace(/_/g, ""));
	} catch {
		return null;
	}
	const info = ELEM[e];
	if (v < info.min || v > info.max) return null;
	return info.size === 8 ? v : Number(v);
}

export { typeName };
