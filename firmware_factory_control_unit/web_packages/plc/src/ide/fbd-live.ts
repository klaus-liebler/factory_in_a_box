// Online values of FBD operator outputs (docs/plc-iec61131.md 5.3): the lowering inlines operators and
// functions into expressions, so no variable holds their result. The editor computes it from the
// online values of the box's inputs (variables, instance outputs, other operators), as formatted by
// values.ts: TRUE/FALSE, integers, REAL with a '.', 16#.. bit strings, T#.. times. A display aid: the
// inputs are the values read after the cycle (a variable written later in the cycle than the
// operator ran shows its new value), and integer results are not wrapped to the (unknown) type's
// width. Unknown inputs or user functions give no value. DOM-free.
import { T } from "../compiler/types.ts";
import type { BoxDef } from "./fbd.ts";
import { formatTime, parseValue } from "./values.ts";

type Live =
	| { k: "bool"; v: boolean }
	| { k: "int"; v: bigint; hex: boolean }
	| { k: "real"; v: number }
	| { k: "time"; v: number };

export function parseLive(text: string | null | undefined): Live | null {
	if (text === null || text === undefined) return null;
	const t = text.trim();
	if (t === "TRUE" || t === "FALSE") return { k: "bool", v: t === "TRUE" };
	if (/^T#/i.test(t)) {
		const ms = parseValue(T.TIME, t);
		return typeof ms === "number" ? { k: "time", v: ms } : null;
	}
	if (/^16#[0-9A-F]+$/i.test(t)) return { k: "int", v: BigInt(`0x${t.slice(3)}`), hex: true };
	// a typed constant of an input variable: INT#21, REAL#1.5 (dates and enums: not known)
	const typed = /^([A-Za-z_]+)#(.+)$/.exec(t);
	if (typed) return /^(D|DATE|TOD|TIME_OF_DAY|DT|DATE_AND_TIME)$/i.test(typed[1]) ? null : parseLive(typed[2]);
	if (/^[+-]?\d+$/.test(t)) return { k: "int", v: BigInt(t), hex: false };
	if (/^[+-]?(\d+\.\d*|\d*\.\d+|\d+(\.\d*)?[eE][+-]?\d+)$/.test(t)) return { k: "real", v: Number(t) };
	return null;
}

export function formatLive(v: Live): string {
	switch (v.k) {
		case "bool": return v.v ? "TRUE" : "FALSE";
		case "time": return formatTime(v.v);
		case "int": return v.hex && v.v >= 0n ? `16#${v.v.toString(16).toUpperCase()}` : v.v.toString();
		case "real": {
			if (Number.isNaN(v.v)) return "NaN";
			if (!Number.isFinite(v.v)) return v.v > 0 ? "+Inf" : "-Inf";
			return Number.isInteger(v.v) && Math.abs(v.v) < 1e15 ? v.v.toFixed(1) : Number(v.v.toPrecision(15)).toString();
		}
	}
}

const num = (v: Live): number | null => (v.k === "int" ? Number(v.v) : v.k === "real" || v.k === "time" ? v.v : null);

/** Compares two values of the same class (int and real mix); null: not comparable. */
function compare(a: Live, b: Live): number | null {
	if (a.k === "bool" && b.k === "bool") return Number(a.v) - Number(b.v);
	if (a.k === "int" && b.k === "int") return a.v < b.v ? -1 : a.v > b.v ? 1 : 0;
	if (a.k === "time" !== (b.k === "time")) return null;
	const x = num(a), y = num(b);
	return x === null || y === null ? null : x - y;
}

function arith(op: string, a: Live, b: Live): Live | null {
	if (a.k === "int" && b.k === "int") {
		const hex = a.hex && b.hex;
		switch (op) {
			case "+": return { k: "int", v: a.v + b.v, hex };
			case "-": return { k: "int", v: a.v - b.v, hex };
			case "*": return { k: "int", v: a.v * b.v, hex };
			case "/": return b.v === 0n ? null : { k: "int", v: a.v / b.v, hex };  // BigInt division truncates toward zero, as ST
			case "MOD": return b.v === 0n ? { k: "int", v: 0n, hex } : { k: "int", v: a.v % b.v, hex };  // x MOD 0 = 0
		}
		return null;
	}
	if (a.k === "time" || b.k === "time") {
		if (a.k === "time" && b.k === "time" && (op === "+" || op === "-")) return { k: "time", v: op === "+" ? a.v + b.v : a.v - b.v };
		// TIME * number, number * TIME, TIME / number
		const ms = a.k === "time" ? a.v : b.k === "time" ? b.v : 0;
		const n = a.k === "time" ? b : a;
		const f = num(n);
		if (f === null || n.k === "time") return null;
		if (op === "*") return { k: "time", v: Math.trunc(ms * f) };
		if (op === "/" && a.k === "time" && f !== 0) return { k: "time", v: Math.trunc(ms / f) };
		return null;
	}
	if (a.k === "bool" || b.k === "bool" || op === "MOD") return null;
	const x = num(a)!, y = num(b)!;
	switch (op) {
		case "+": return { k: "real", v: x + y };
		case "-": return { k: "real", v: x - y };
		case "*": return { k: "real", v: x * y };
		case "/": return { k: "real", v: x / y };
	}
	return null;
}

function logic(op: string, a: Live, b: Live): Live | null {
	if (a.k === "bool" && b.k === "bool") return { k: "bool", v: op === "AND" ? a.v && b.v : op === "OR" ? a.v || b.v : a.v !== b.v };
	if (a.k === "int" && b.k === "int") {
		const v = op === "AND" ? a.v & b.v : op === "OR" ? a.v | b.v : a.v ^ b.v;
		return { k: "int", v, hex: a.hex || b.hex };
	}
	return null;
}

/** The output value of a standard operator or function box from its input values (in the order of
 *  `def.inputs`); null: not known (an input unknown, a type that does not fit, a user function). */
export function evalBox(def: BoxDef, inputs: (string | null | undefined)[]): string | null {
	const v = inputs.map(parseLive);
	if (def.kind === "move") return v[0] ? formatLive(v[0]) : null;
	if (def.kind === "fb") return null;
	const all = (xs: (Live | null)[]): xs is Live[] => xs.every((x) => x !== null);
	switch (def.type) {
		case "NOT": {
			const a = v[0];
			return a?.k === "bool" ? formatLive({ k: "bool", v: !a.v }) : null;  // a bit string's NOT needs its width
		}
		case "AND": case "OR": case "XOR": {
			if (!all(v)) return null;
			const r = logic(def.type, v[0], v[1]);
			return r ? formatLive(r) : null;
		}
		case "GT": case "GE": case "EQ": case "NE": case "LE": case "LT": {
			if (!all(v)) return null;
			const c = compare(v[0], v[1]);
			if (c === null) return null;
			const r = { GT: c > 0, GE: c >= 0, EQ: c === 0, NE: c !== 0, LE: c <= 0, LT: c < 0 }[def.type];
			return formatLive({ k: "bool", v: r });
		}
		case "ADD": case "SUB": case "MUL": case "DIV": case "MOD": {
			if (!all(v)) return null;
			const r = arith({ ADD: "+", SUB: "-", MUL: "*", DIV: "/", MOD: "MOD" }[def.type], v[0], v[1]);
			return r ? formatLive(r) : null;
		}
		case "SEL": {
			const g = v[0];
			if (g?.k !== "bool") return null;
			const r = g.v ? v[2] : v[1];
			return r ? formatLive(r) : null;
		}
		case "MAX": case "MIN": {
			if (!all(v)) return null;
			const c = compare(v[0], v[1]);
			if (c === null) return null;
			return formatLive((def.type === "MAX") === c >= 0 ? v[0] : v[1]);
		}
		case "LIMIT": {
			if (!all(v)) return null;
			const [mn, x, mx] = v;
			const lo = compare(x, mn), hi = compare(x, mx);
			if (lo === null || hi === null) return null;
			return formatLive(lo < 0 ? mn : hi > 0 ? mx : x);
		}
	}
	return null;
}
