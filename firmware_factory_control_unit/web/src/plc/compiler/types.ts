// SPDX-License-Identifier: MIT
// The type model of the IEC 61131-3 compiler: elementary types (rtoos/docs/plc.md "Values and
// types"), derived types, the implicit conversions of IEC 61131-3 Ed. 3 (table 11) and the memory
// layout. DOM-free.
import type { Slot } from "../generated/isa.ts";
import type { Init } from "./ast.ts";

export type ElemName =
	| "BOOL" | "SINT" | "INT" | "DINT" | "LINT" | "USINT" | "UINT" | "UDINT" | "ULINT"
	| "BYTE" | "WORD" | "DWORD" | "LWORD" | "REAL" | "LREAL" | "TIME";

export type Category = "bool" | "int" | "uint" | "bit" | "real" | "time";

export interface ElemInfo {
	size: 1 | 2 | 4 | 8;
	slot: Slot;
	category: Category;
	signed: boolean;
	min: bigint;
	max: bigint;
}

const I = (size: 1 | 2 | 4 | 8, signed: boolean, category: Category): ElemInfo => {
	const bits = BigInt(size * 8);
	return {
		size, slot: size === 8 ? "I64" : "I32", category, signed,
		min: signed ? -(1n << (bits - 1n)) : 0n, max: signed ? (1n << (bits - 1n)) - 1n : (1n << bits) - 1n,
	};
};

export const ELEM: Record<ElemName, ElemInfo> = {
	BOOL: { ...I(1, false, "bool"), max: 1n },
	SINT: I(1, true, "int"), INT: I(2, true, "int"), DINT: I(4, true, "int"), LINT: I(8, true, "int"),
	USINT: I(1, false, "uint"), UINT: I(2, false, "uint"), UDINT: I(4, false, "uint"), ULINT: I(8, false, "uint"),
	BYTE: I(1, false, "bit"), WORD: I(2, false, "bit"), DWORD: I(4, false, "bit"), LWORD: I(8, false, "bit"),
	REAL: { size: 4, slot: "F32", category: "real", signed: true, min: 0n, max: 0n },
	LREAL: { size: 8, slot: "F64", category: "real", signed: true, min: 0n, max: 0n },
	TIME: I(4, true, "time"),
};

export const isElemName = (s: string): s is ElemName => s in ELEM;

// ---- types --------------------------------------------------------------------------------------
export interface Field {
	name: string;
	key: string;
	type: Type;
	offset: number;
	/** The declared initial value (instances and structures are initialized from it). */
	init?: Init | null;
}

export interface FbLayout {
	/** Upper-case name of the FUNCTION_BLOCK or PROGRAM. */
	key: string;
	name: string;
	size: number;
	fields: Field[];
}

export type Type =
	| { k: "elem"; name: ElemName }
	| { k: "enum"; name: string; key: string; base: ElemName; values: Map<string, { name: string; value: bigint }>; init: bigint }
	| { k: "subrange"; name: string | null; base: ElemName; lo: bigint; hi: bigint }
	| { k: "struct"; name: string; key: string; fields: Field[]; size: number; align: number }
	| { k: "array"; dims: { lo: number; hi: number }[]; elem: Type; size: number; align: number }
	| { k: "fb"; layout: FbLayout }
	| { k: "error" };

export const T = Object.fromEntries(Object.keys(ELEM).map((n) => [n, { k: "elem", name: n } as Type])) as Record<ElemName, Type>;
export const ERROR: Type = { k: "error" };

/** The elementary type a value of `t` is held as (enum and subrange: their base). */
export function elemOf(t: Type): ElemName | null {
	switch (t.k) {
		case "elem": return t.name;
		case "enum":
		case "subrange": return t.base;
		default: return null;
	}
}

export function sizeOf(t: Type): number {
	switch (t.k) {
		case "elem": return ELEM[t.name].size;
		case "enum":
		case "subrange": return ELEM[t.base].size;
		case "struct":
		case "array": return t.size;
		case "fb": return t.layout.size;
		case "error": return 4;
	}
}

export function alignOf(t: Type): number {
	switch (t.k) {
		case "struct":
		case "array": return t.align;
		case "fb": return 8;
		default: return Math.min(sizeOf(t), 8);
	}
}

export function typeName(t: Type): string {
	switch (t.k) {
		case "elem": return t.name;
		case "enum": return t.name;
		case "subrange": return t.name ?? `${t.base} (${t.lo}..${t.hi})`;
		case "struct": return t.name;
		case "array": return `ARRAY[${t.dims.map((d) => `${d.lo}..${d.hi}`).join(", ")}] OF ${typeName(t.elem)}`;
		case "fb": return t.layout.name;
		case "error": return "<error>";
	}
}

export function sameType(a: Type, b: Type): boolean {
	if (a === b) return true;
	if (a.k !== b.k) return false;
	switch (a.k) {
		case "elem": return a.name === (b as typeof a).name;
		case "enum": return a.key === (b as typeof a).key;
		case "struct": return a.key === (b as typeof a).key;
		case "fb": return a.layout.key === (b as typeof a).layout.key;
		case "subrange": {
			const o = b as typeof a;
			return a.base === o.base && a.lo === o.lo && a.hi === o.hi;
		}
		case "array": {
			const o = b as typeof a;
			return a.dims.length === o.dims.length && a.dims.every((d, i) => d.lo === o.dims[i].lo && d.hi === o.dims[i].hi) && sameType(a.elem, o.elem);
		}
		case "error": return true;
	}
}

// ---- implicit conversions (IEC 61131-3 Ed. 3, table 11: widening without loss) -------------------
const IMPLICIT: Partial<Record<ElemName, ElemName[]>> = {
	SINT: ["INT", "DINT", "LINT", "REAL", "LREAL"],
	INT: ["DINT", "LINT", "REAL", "LREAL"],
	DINT: ["LINT", "LREAL"],
	USINT: ["UINT", "UDINT", "ULINT", "INT", "DINT", "LINT", "REAL", "LREAL"],
	UINT: ["UDINT", "ULINT", "DINT", "LINT", "REAL", "LREAL"],
	UDINT: ["ULINT", "LINT", "LREAL"],
	REAL: ["LREAL"],
	BOOL: ["BYTE", "WORD", "DWORD", "LWORD"],
	BYTE: ["WORD", "DWORD", "LWORD"],
	WORD: ["DWORD", "LWORD"],
	DWORD: ["LWORD"],
};

export function implicitElem(from: ElemName, to: ElemName): boolean {
	return from === to || (IMPLICIT[from]?.includes(to) ?? false);
}

/** May a value of `from` be assigned to `to` without an explicit conversion? */
export function assignable(from: Type, to: Type): boolean {
	if (from.k === "error" || to.k === "error") return true;
	if (sameType(from, to)) return true;
	// a subrange value is a value of its base; a value of the base goes into a subrange with a check
	if (to.k === "subrange") {
		const f = elemOf(from);
		return from.k !== "enum" && f !== null && implicitElem(f, to.base);
	}
	if (from.k === "subrange" && to.k === "elem") return implicitElem(from.base, to.name);
	if (from.k === "elem" && to.k === "elem") return implicitElem(from.name, to.name);
	return false;
}

export const isNumeric = (e: ElemName) => ["int", "uint", "real"].includes(ELEM[e].category);
export const isInteger = (e: ElemName) => ["int", "uint"].includes(ELEM[e].category);
export const isBitLike = (e: ElemName) => ["bool", "bit"].includes(ELEM[e].category);
export const isReal = (e: ElemName) => ELEM[e].category === "real";

export const alignUp = (n: number, a: number) => Math.ceil(n / a) * a;
