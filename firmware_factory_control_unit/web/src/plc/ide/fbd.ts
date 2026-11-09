// FBD (function block diagram, IEC 61131-3 Ed. 3, 8.2) - the model of the graphical editor and its
// lowering to ST (docs/plc-iec61131.md 5.3): networks of boxes, input variables (any ST expression)
// and output variables, joined by wires from an output pin to an input pin. The ST that comes out
// is compiled like any other source, so an FBD POU runs and debugs on the same VM. DOM-free.

/** A box type: its pins and how it lowers. op: an infix operator over its inputs; func: a standard
 *  function with positional arguments; fb: a function block (an instance, called with its inputs). */
export interface BoxDef {
	type: string;
	kind: "op" | "unary" | "func" | "move" | "fb";
	inputs: string[];
	outputs: string[];
	/** op: the ST operator; unary: the prefix. */
	op?: string;
	group: "Logik" | "Vergleich" | "Arithmetik" | "Auswahl" | "Zeit" | "Zähler" | "Flanke/Speicher";
	title: string;
}

const op = (type: string, opText: string, group: BoxDef["group"], title: string, inputs = ["IN1", "IN2"]): BoxDef =>
	({ type, kind: "op", op: opText, inputs, outputs: ["OUT"], group, title });

export const BOXES: readonly BoxDef[] = [
	op("AND", "AND", "Logik", "UND"),
	op("OR", "OR", "Logik", "ODER"),
	op("XOR", "XOR", "Logik", "Exklusiv-ODER"),
	{ type: "NOT", kind: "unary", op: "NOT ", inputs: ["IN"], outputs: ["OUT"], group: "Logik", title: "Negation" },
	op("GT", ">", "Vergleich", "größer"),
	op("GE", ">=", "Vergleich", "größer gleich"),
	op("EQ", "=", "Vergleich", "gleich"),
	op("NE", "<>", "Vergleich", "ungleich"),
	op("LE", "<=", "Vergleich", "kleiner gleich"),
	op("LT", "<", "Vergleich", "kleiner"),
	op("ADD", "+", "Arithmetik", "Addition"),
	op("SUB", "-", "Arithmetik", "Subtraktion"),
	op("MUL", "*", "Arithmetik", "Multiplikation"),
	op("DIV", "/", "Arithmetik", "Division"),
	op("MOD", "MOD", "Arithmetik", "Divisionsrest"),
	{ type: "SEL", kind: "func", inputs: ["G", "IN0", "IN1"], outputs: ["OUT"], group: "Auswahl", title: "Auswahl: G ? IN1 : IN0" },
	{ type: "MAX", kind: "func", inputs: ["IN1", "IN2"], outputs: ["OUT"], group: "Auswahl", title: "Maximum" },
	{ type: "MIN", kind: "func", inputs: ["IN1", "IN2"], outputs: ["OUT"], group: "Auswahl", title: "Minimum" },
	{ type: "LIMIT", kind: "func", inputs: ["MN", "IN", "MX"], outputs: ["OUT"], group: "Auswahl", title: "Begrenzung" },
	{ type: "MOVE", kind: "move", inputs: ["IN"], outputs: ["OUT"], group: "Auswahl", title: "Zuweisung" },
	{ type: "TON", kind: "fb", inputs: ["IN", "PT"], outputs: ["Q", "ET"], group: "Zeit", title: "Einschaltverzögerung" },
	{ type: "TOF", kind: "fb", inputs: ["IN", "PT"], outputs: ["Q", "ET"], group: "Zeit", title: "Ausschaltverzögerung" },
	{ type: "TP", kind: "fb", inputs: ["IN", "PT"], outputs: ["Q", "ET"], group: "Zeit", title: "Impuls" },
	{ type: "CTU", kind: "fb", inputs: ["CU", "R", "PV"], outputs: ["Q", "CV"], group: "Zähler", title: "Vorwärtszähler" },
	{ type: "CTD", kind: "fb", inputs: ["CD", "LD", "PV"], outputs: ["Q", "CV"], group: "Zähler", title: "Rückwärtszähler" },
	{ type: "R_TRIG", kind: "fb", inputs: ["CLK"], outputs: ["Q"], group: "Flanke/Speicher", title: "steigende Flanke" },
	{ type: "F_TRIG", kind: "fb", inputs: ["CLK"], outputs: ["Q"], group: "Flanke/Speicher", title: "fallende Flanke" },
	{ type: "SR", kind: "fb", inputs: ["S1", "R"], outputs: ["Q1"], group: "Flanke/Speicher", title: "Setzen dominant" },
	{ type: "RS", kind: "fb", inputs: ["S", "R1"], outputs: ["Q1"], group: "Flanke/Speicher", title: "Rücksetzen dominant" },
];
const BOX = new Map(BOXES.map((b) => [b.type, b]));
export const boxDef = (type: string): BoxDef | undefined => BOX.get(type);

// ---- pin types (for the colours of the editor) --------------------------------------------------------
/** The pins with a fixed type; every other pin of a box is generic, and the generic pins of one box
 *  share one type (ADD: IN1, IN2, OUT; SEL: IN0, IN1, OUT; GT: IN1, IN2 - OUT is BOOL). */
const FIXED: Record<string, Record<string, string>> = {
	GT: { OUT: "BOOL" }, GE: { OUT: "BOOL" }, EQ: { OUT: "BOOL" }, NE: { OUT: "BOOL" }, LE: { OUT: "BOOL" }, LT: { OUT: "BOOL" },
	SEL: { G: "BOOL" },
	TON: { IN: "BOOL", PT: "TIME", Q: "BOOL", ET: "TIME" },
	TOF: { IN: "BOOL", PT: "TIME", Q: "BOOL", ET: "TIME" },
	TP: { IN: "BOOL", PT: "TIME", Q: "BOOL", ET: "TIME" },
	CTU: { CU: "BOOL", R: "BOOL", PV: "INT", Q: "BOOL", CV: "INT" },
	CTD: { CD: "BOOL", LD: "BOOL", PV: "INT", Q: "BOOL", CV: "INT" },
	R_TRIG: { CLK: "BOOL", Q: "BOOL" }, F_TRIG: { CLK: "BOOL", Q: "BOOL" },
	SR: { S1: "BOOL", R: "BOOL", Q1: "BOOL" }, RS: { S: "BOOL", R1: "BOOL", Q1: "BOOL" },
};

/** The type a box declares for a pin; null: generic. */
export function declaredPinType(boxType: string, pin: string): string | null {
	return FIXED[boxType]?.[pin] ?? null;
}

/** The colour classes of the editor: BOOL, integers (and bit strings), REAL/LREAL, times and dates,
 *  strings; any: not known (yet). IEC 61131-3 prescribes no colours - the palette is the FCU's. */
export type TypeClass = "bool" | "int" | "real" | "time" | "string" | "any";

export function typeClass(elem: string | null | undefined): TypeClass {
	switch ((elem ?? "").toUpperCase()) {
		case "BOOL": return "bool";
		case "SINT": case "INT": case "DINT": case "LINT": case "USINT": case "UINT": case "UDINT": case "ULINT":
		case "BYTE": case "WORD": case "DWORD": case "LWORD":
			return "int";
		case "REAL": case "LREAL": return "real";
		case "TIME": case "LTIME": case "DATE": case "LDATE": case "TOD": case "TIME_OF_DAY": case "LTOD": case "DT": case "DATE_AND_TIME": case "LDT":
			return "time";
		case "STRING": case "WSTRING": return "string";
		default: return "any";
	}
}

/** The type of a constant or a simple expression of an input variable, as far as it shows; null: not known. */
export function literalType(expr: string): string | null {
	const e = expr.trim();
	if (/^(TRUE|FALSE)$/i.test(e)) return "BOOL";
	const typed = /^([A-Z_0-9]+)#/i.exec(e);
	if (typed) {
		const p = typed[1].toUpperCase();
		if (p === "T" || p === "TIME" || p === "LT" || p === "LTIME") return "TIME";
		if (p === "D" || p === "DATE") return "DATE";
		if (p === "TOD" || p === "TIME_OF_DAY") return "TOD";
		if (p === "DT" || p === "DATE_AND_TIME") return "DT";
		if (/^\d+$/.test(p)) return "DINT";  // 16#FF, 2#1010
		return p;  // INT#5, REAL#1.5, BOOL#1
	}
	if (/^[+-]?\d+(_\d+)*$/.test(e)) return "DINT";
	if (/^[+-]?\d+(_\d+)*\.\d+([eE][+-]?\d+)?$/.test(e) || /^[+-]?\d+[eE][+-]?\d+$/.test(e)) return "LREAL";
	if (/^'.*'$/.test(e)) return "STRING";
	// an expression: a comparison or a Boolean operator gives BOOL
	if (/<>|<=|>=|[=<>]/.test(e) || /\b(AND|OR|XOR|NOT)\b/i.test(e)) return "BOOL";
	return null;
}

export interface NetworkTypes {
	/** "node:in|out:pin" -> its type class. */
	pins: Map<string, TypeClass>;
	/** Indexes of wires that join two known, different types. */
	mismatches: Set<number>;
}

export const pinKey = (node: string, side: "in" | "out", pin: string) => `${node}:${side}:${pin}`;

/** The type class of every pin of a network: fixed pins of the boxes, variables (`varType`: a name ->
 *  its elementary type, or null) and constants; then generic pins take the type over their wires and
 *  within their box, until nothing changes. */
export function networkTypes(net: FbdNetwork, varType: (name: string) => string | null): NetworkTypes {
	const pins = new Map<string, TypeClass>();
	const flexible = new Set<string>();  // may still get a type
	const groups: string[][] = [];  // the generic pins of a box
	for (const n of net.nodes) {
		if (n.kind === "input") {
			const k = pinKey(n.id, "out", "OUT");
			const t = typeClass(varType(n.expr.trim()) ?? literalType(n.expr));
			pins.set(k, t);
			if (t === "any") flexible.add(k);
		} else if (n.kind === "output") {
			const k = pinKey(n.id, "in", "IN");
			const t = typeClass(varType(n.target.trim()));
			pins.set(k, t);
			if (t === "any") flexible.add(k);
		} else {
			const p = pinsOf(n);
			const group: string[] = [];
			for (const [side, list] of [["in", p.inputs], ["out", p.outputs]] as const) {
				for (const pin of list) {
					const k = pinKey(n.id, side, pin);
					const fixed = declaredPinType(n.type, pin);
					pins.set(k, fixed ? typeClass(fixed) : "any");
					if (!fixed) {
						flexible.add(k);
						group.push(k);
					}
				}
			}
			if (group.length > 1) groups.push(group);
		}
	}
	const known = (k: string) => (pins.get(k) ?? "any") !== "any";
	for (let round = 0, changed = true; changed && round < 32; round++) {
		changed = false;
		for (const w of net.wires) {
			const a = pinKey(w.from.node, "out", w.from.pin), b = pinKey(w.to.node, "in", w.to.pin);
			if (known(a) && !known(b) && flexible.has(b)) {
				pins.set(b, pins.get(a)!);
				changed = true;
			} else if (known(b) && !known(a) && flexible.has(a)) {
				pins.set(a, pins.get(b)!);
				changed = true;
			}
		}
		for (const g of groups) {
			const t = g.map((k) => pins.get(k)).find((x) => x && x !== "any");
			if (!t) continue;
			for (const k of g) {
				if (!known(k)) {
					pins.set(k, t);
					changed = true;
				}
			}
		}
	}
	const mismatches = new Set<number>();
	net.wires.forEach((w, i) => {
		const a = pins.get(pinKey(w.from.node, "out", w.from.pin)) ?? "any", b = pins.get(pinKey(w.to.node, "in", w.to.pin)) ?? "any";
		if (a !== "any" && b !== "any" && a !== b) mismatches.add(i);
	});
	return { pins, mismatches };
}

export type FbdNode =
	| { kind: "box"; id: string; x: number; y: number; type: string; instance?: string }
	| { kind: "input"; id: string; x: number; y: number; expr: string }
	| { kind: "output"; id: string; x: number; y: number; target: string };

export interface PinRef {
	node: string;
	pin: string;
}

export interface FbdWire {
	from: PinRef; // an output pin (a box output, or an input variable's "OUT")
	to: PinRef; // an input pin (a box input, or an output variable's "IN")
}

export interface FbdNetwork {
	id: string;
	title: string;
	nodes: FbdNode[];
	wires: FbdWire[];
}

export interface FbdPou {
	name: string;
	/** "PROGRAM <name>" and the VAR sections of the user's variables (the instances are added). */
	decl: string;
	networks: FbdNetwork[];
}

/** The pins of a node: inputs on the left, outputs on the right. */
export function pinsOf(n: FbdNode): { inputs: string[]; outputs: string[] } {
	if (n.kind === "input") return { inputs: [], outputs: ["OUT"] };
	if (n.kind === "output") return { inputs: ["IN"], outputs: [] };
	const d = boxDef(n.type);
	return { inputs: d?.inputs ?? [], outputs: d?.outputs ?? [] };
}

export interface FbdProblem {
	network: string;
	node?: string;
	message: string;
}

export interface FbdLowered {
	/** The FB instances of the boxes, one VAR section. */
	instances: string;
	/** The statements of all networks. */
	body: string;
	problems: FbdProblem[];
}

/** FBD -> ST. Per network: every function block box becomes a call of its instance (in dependency
 *  order), every output variable an assignment; operators and functions are expressions inlined into
 *  their consumers. An input pin without a wire is an error (except a counter's or timer's optional
 *  ones: R, LD, PV, PT get FALSE / 0 / T#0s - the editor shows them unconnected). */
export function lowerFbd(pou: FbdPou): FbdLowered {
	const problems: FbdProblem[] = [];
	const instances: string[] = [];
	const lines: string[] = [];
	const usedNames = new Set<string>();
	for (const net of pou.networks) {
		lines.push(`(* ${net.title.replace(/\*\)/g, "* )") || "Netzwerk"} *)`);
		const byId = new Map(net.nodes.map((n) => [n.id, n]));
		const into = new Map<string, PinRef>(); // "node.pin" -> source
		for (const w of net.wires) into.set(`${w.to.node}.${w.to.pin}`, w.from);
		const problem = (node: string | undefined, message: string) => problems.push({ network: net.id, node, message });

		const instanceOf = (n: FbdNode & { kind: "box" }) => n.instance?.trim() || `${n.type}_${n.id}`;
		for (const n of net.nodes) {
			if (n.kind !== "box") continue;
			const d = boxDef(n.type);
			if (!d) {
				problem(n.id, `unbekannter Baustein ${n.type}`);
				continue;
			}
			if (d.kind === "fb") {
				const name = instanceOf(n);
				if (usedNames.has(name.toUpperCase())) problem(n.id, `Instanzname ${name} doppelt`);
				usedNames.add(name.toUpperCase());
				instances.push(`    ${name} : ${n.type};`);
			}
		}

		const visiting = new Set<string>();
		const called = new Set<string>();
		const calls: string[] = [];
		const optional: Record<string, string> = { R: "FALSE", LD: "FALSE", PV: "0", PT: "T#0s" };

		const exprOfPin = (node: FbdNode, pin: string): string => {
			const src = into.get(`${node.id}.${pin}`);
			if (!src) {
				if (node.kind === "box" && boxDef(node.type)?.kind === "fb" && optional[pin]) return optional[pin];
				problem(node.id, `Eingang ${pin} ist nicht verbunden`);
				return "FALSE";
			}
			return exprOfOutput(src);
		};
		const exprOfOutput = (ref: PinRef): string => {
			const n = byId.get(ref.node);
			if (!n) return "FALSE";
			if (n.kind === "input") return n.expr.trim() || (problem(n.id, "Eingangsvariable ohne Ausdruck"), "FALSE");
			if (n.kind !== "box") return "FALSE";
			const d = boxDef(n.type);
			if (!d) return "FALSE";
			if (d.kind === "fb") {
				// a feedback through an instance is allowed: it reads the value of the last cycle
				if (!visiting.has(n.id)) {
					visiting.add(n.id);
					ensureCall(n);
					visiting.delete(n.id);
				}
				return `${instanceOf(n)}.${ref.pin}`;
			}
			if (visiting.has(n.id)) {
				problem(n.id, "Rückkopplung ohne Speicher: ein Kreis aus Operatoren im Netzwerk");
				return "FALSE";
			}
			visiting.add(n.id);
			try {
				switch (d.kind) {
					case "op":
						return `(${d.inputs.map((p) => exprOfPin(n, p)).join(` ${d.op} `)})`;
					case "unary":
						return `(${d.op}${exprOfPin(n, d.inputs[0])})`;
					case "func":
						return `${d.type}(${d.inputs.map((p) => exprOfPin(n, p)).join(", ")})`;
					case "move":
						return exprOfPin(n, "IN");
				}
			} finally {
				visiting.delete(n.id);
			}
			return "FALSE";
		};
		const ensureCall = (n: FbdNode & { kind: "box" }) => {
			if (called.has(n.id)) return;
			called.add(n.id);
			const d = boxDef(n.type)!;
			const args = d.inputs.map((p) => `${p} := ${exprOfPin(n, p)}`).join(", ");
			calls.push(`${instanceOf(n)}(${args});`);
		};

		// in the order of the drawing: boxes left to right, then the outputs top to bottom
		const ordered = [...net.nodes].sort((a, b) => a.x - b.x || a.y - b.y);
		for (const n of ordered) if (n.kind === "box" && boxDef(n.type)?.kind === "fb") {
			visiting.add(n.id);
			ensureCall(n);
			visiting.delete(n.id);
		}
		const assigns: string[] = [];
		for (const n of [...net.nodes].filter((x) => x.kind === "output").sort((a, b) => a.y - b.y)) {
			if (n.kind !== "output") continue;
			if (!n.target.trim()) {
				problem(n.id, "Ausgangsvariable ohne Namen");
				continue;
			}
			assigns.push(`${n.target.trim()} := ${exprOfPin(n, "IN")};`);
		}
		lines.push(...calls, ...assigns, "");
	}
	return { instances: instances.length ? `VAR (* FBD-Instanzen *)\n${instances.join("\n")}\nEND_VAR` : "", body: lines.join("\n"), problems };
}

/** The whole POU as ST: the user's declaration, the instances, the networks, END_PROGRAM. */
export function fbdSource(pou: FbdPou): { text: string; problems: FbdProblem[] } {
	const l = lowerFbd(pou);
	return { text: `${pou.decl.trim()}\n${l.instances}\n${l.body}END_PROGRAM\n`, problems: l.problems };
}

let counter = 0;
/** A fresh node id within a network. */
export function newId(net: FbdNetwork): string {
	const used = new Set(net.nodes.map((n) => n.id));
	let id: string;
	do id = `n${++counter}`; while (used.has(id));
	return id;
}

/** The example: the sorting conveyor of the ST example, as FBD. */
export function exampleFbd(): FbdPou {
	return {
		name: "FBD_PRG",
		decl: "PROGRAM FBD_PRG\nVAR\n    Enable : BOOL := TRUE;   (* conveyor on *)\nEND_VAR",
		networks: [
			{
				id: "net1", title: "Teile an Lichtschranke 1 zählen",
				nodes: [
					{ kind: "input", id: "i1", x: 20, y: 40, expr: "Lightbarrier1" },
					{ kind: "input", id: "i2", x: 20, y: 70, expr: "Enable" },
					{ kind: "box", id: "b1", x: 180, y: 70, type: "NOT" },
					{ kind: "input", id: "i3", x: 20, y: 110, expr: "10" },
					{ kind: "box", id: "b2", x: 330, y: 20, type: "CTU", instance: "Pieces" },
				],
				wires: [
					{ from: { node: "i1", pin: "OUT" }, to: { node: "b2", pin: "CU" } },
					{ from: { node: "i2", pin: "OUT" }, to: { node: "b1", pin: "IN" } },
					{ from: { node: "b1", pin: "OUT" }, to: { node: "b2", pin: "R" } },
					{ from: { node: "i3", pin: "OUT" }, to: { node: "b2", pin: "PV" } },
				],
			},
			{
				id: "net2", title: "Förderband: an, bis 10 Teile gezählt sind",
				nodes: [
					{ kind: "input", id: "i1", x: 20, y: 30, expr: "Enable" },
					{ kind: "input", id: "i2", x: 20, y: 70, expr: "Pieces.Q" },
					{ kind: "box", id: "b1", x: 170, y: 70, type: "NOT" },
					{ kind: "box", id: "b2", x: 300, y: 20, type: "AND" },
					{ kind: "input", id: "i3", x: 300, y: 110, expr: "0" },
					{ kind: "input", id: "i4", x: 300, y: 140, expr: "600" },
					{ kind: "box", id: "b3", x: 450, y: 30, type: "SEL" },
					{ kind: "output", id: "o1", x: 620, y: 40, target: "ConveyorPwm" },
				],
				wires: [
					{ from: { node: "i1", pin: "OUT" }, to: { node: "b2", pin: "IN1" } },
					{ from: { node: "i2", pin: "OUT" }, to: { node: "b1", pin: "IN" } },
					{ from: { node: "b1", pin: "OUT" }, to: { node: "b2", pin: "IN2" } },
					{ from: { node: "b2", pin: "OUT" }, to: { node: "b3", pin: "G" } },
					{ from: { node: "i3", pin: "OUT" }, to: { node: "b3", pin: "IN0" } },
					{ from: { node: "i4", pin: "OUT" }, to: { node: "b3", pin: "IN1" } },
					{ from: { node: "b3", pin: "OUT" }, to: { node: "o1", pin: "IN" } },
				],
			},
			{
				id: "net3", title: "Jedes zweite Teil an Lichtschranke 2 ausschleusen",
				nodes: [
					{ kind: "input", id: "i1", x: 20, y: 30, expr: "Lightbarrier2" },
					{ kind: "box", id: "b1", x: 170, y: 20, type: "R_TRIG", instance: "AtGate" },
					{ kind: "input", id: "i2", x: 20, y: 100, expr: "Pieces.CV MOD 2 = 0" },
					{ kind: "box", id: "b2", x: 330, y: 30, type: "AND" },
					{ kind: "input", id: "i3", x: 330, y: 120, expr: "T#300ms" },
					{ kind: "box", id: "b3", x: 480, y: 30, type: "TP", instance: "Eject" },
					{ kind: "output", id: "o1", x: 650, y: 40, target: "Valve1" },
				],
				wires: [
					{ from: { node: "i1", pin: "OUT" }, to: { node: "b1", pin: "CLK" } },
					{ from: { node: "b1", pin: "Q" }, to: { node: "b2", pin: "IN1" } },
					{ from: { node: "i2", pin: "OUT" }, to: { node: "b2", pin: "IN2" } },
					{ from: { node: "b2", pin: "OUT" }, to: { node: "b3", pin: "IN" } },
					{ from: { node: "i3", pin: "OUT" }, to: { node: "b3", pin: "PT" } },
					{ from: { node: "b3", pin: "Q" }, to: { node: "o1", pin: "IN" } },
				],
			},
		],
	};
}
