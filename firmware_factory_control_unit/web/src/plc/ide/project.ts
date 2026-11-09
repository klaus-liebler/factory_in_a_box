// The PLC project (docs/plc-iec61131.md sections 5.8 and 11): from P4 any number of objects - programs,
// function blocks, functions, data types, global variable lists - each its own source file for the
// compiler (the file name is the object's name). Declaration and implementation are edited
// separately; for the compiler they are one text. The implementation is ST, or an FBD or SFC that is
// lowered to ST (P5/P6); the declaration is always ST.
import { compile, projectHashOf, type CompileResult } from "../compiler/compiler.ts";
import { Diagnostics, type Diagnostic } from "../compiler/diagnostics.ts";
import { lex } from "../compiler/lexer.ts";
import { parse } from "../compiler/parser.ts";
import type * as A from "../compiler/ast.ts";
import { FCU_LIB_FILE, fcuLibSource } from "./fcu-lib.ts";
import { IO_FILE, ioBindings, ioMapHash, ioSource } from "./io-list.ts";
import { exampleFbd, fbdSource, type BoxDef, type FbdNetwork, type FbdProblem } from "./fbd.ts";
import { exampleSfc, SFC_FILE, SFC_STEP_SOURCE, sfcId, sfcSource, type SfcItem, type SfcProblem } from "./sfc.ts";

export type ObjectKind = "PRG" | "FB" | "FUN" | "DUT" | "GVL";
export const OBJECT_KINDS: readonly ObjectKind[] = ["PRG", "FB", "FUN", "DUT", "GVL"];
export const KIND_NAMES: Record<ObjectKind, string> = {
	PRG: "Programm", FB: "Funktionsbaustein", FUN: "Funktion", DUT: "Datentyp", GVL: "Globale Variablenliste",
};

export type Lang = "ST" | "FBD" | "SFC";
/** The implementation languages of each kind (DUT and GVL: a declaration only). */
export const LANGS_OF: Record<ObjectKind, readonly Lang[]> = { PRG: ["ST", "FBD", "SFC"], FB: ["ST", "FBD", "SFC"], FUN: ["ST", "FBD"], DUT: ["ST"], GVL: ["ST"] };
export const LANG_NAMES: Record<Lang, string> = { ST: "Strukturierter Text", FBD: "Funktionsbausteinsprache", SFC: "Ablaufsprache" };

export interface PlcObject {
	id: string;
	name: string;
	kind: ObjectKind;
	/** The implementation's language, chosen when the object is created. */
	lang: Lang;
	/** PRG/FB/FUN: the header ("PROGRAM PLC_PRG") and the VAR sections; DUT: the whole TYPE ... END_TYPE;
	 *  GVL: the VAR_GLOBAL sections. */
	decl: string;
	/** ST: the statements (without END_...); otherwise empty. */
	body: string;
	/** FBD: the networks. */
	networks?: FbdNetwork[];
	/** SFC: the chart (its first step is the initial step). */
	chart?: SfcItem[];
	/** The folder in the project tree ("" or absent: top level); only for the overview. */
	folder?: string;
}

export interface Project {
	format: 1;
	name: string;
	objects: PlcObject[];
	tasks: { name: string; intervalCycles: number; programs: string[] }[];
}

/** Objects with an implementation part. */
export function hasBody(kind: ObjectKind): boolean {
	return kind === "PRG" || kind === "FB" || kind === "FUN";
}

const END: Record<ObjectKind, string> = { PRG: "END_PROGRAM", FB: "END_FUNCTION_BLOCK", FUN: "END_FUNCTION", DUT: "", GVL: "" };

// ---- the example: the sorting conveyor, split into GVL + FB + PRG (P4) --------------------------------------
export const EXAMPLE_GVL = `VAR_GLOBAL
    Enable : BOOL := TRUE;      (* conveyor on *)
    PiecesMax : INT := 10;      (* the conveyor stops after this many pieces *)
END_VAR`;

export const EXAMPLE_FB_DECL = `FUNCTION_BLOCK FB_Ejector
VAR_INPUT
    Arrive : BOOL;              (* light barrier at the ejector *)
    Select : BOOL;              (* eject this piece *)
END_VAR
VAR_OUTPUT
    Valve : BOOL;               (* ejector valve *)
    Count : INT;                (* pieces ejected *)
END_VAR
VAR
    AtGate : R_TRIG;            (* a piece reaches the light barrier *)
    Pulse : TP;                 (* pulse for the valve *)
END_VAR`;

export const EXAMPLE_FB_BODY = `AtGate(CLK := Arrive);
Pulse(IN := AtGate.Q AND Select, PT := T#300ms);
Valve := Pulse.Q;
IF AtGate.Q AND Select THEN
    Count := Count + 1;
END_IF;
`;

export const EXAMPLE_DECL = `PROGRAM PLC_PRG
VAR
    Pieces : CTU;               (* pieces seen at light barrier 1 *)
    Ejector : FB_Ejector;       (* pushes every second piece off *)
    Ejected : INT;
END_VAR`;

export const EXAMPLE_BODY = `(* Sorting conveyor: stops after PiecesMax pieces, every second piece is
   pushed off at light barrier 2 *)
Pieces(CU := Lightbarrier1, R := NOT Enable, PV := PiecesMax);

IF Enable AND NOT Pieces.Q THEN
    ConveyorPwm := 600;
ELSE
    ConveyorPwm := 0;
END_IF;

Ejector(Arrive := Lightbarrier2, Select := Pieces.CV MOD 2 = 0);
Valve1 := Ejector.Valve;
Ejected := Ejector.Count;
`;

export function exampleProject(): Project {
	return {
		format: 1,
		name: "Sortieranlage",
		objects: [
			{ id: "gvl", name: "GVL", kind: "GVL", lang: "ST", decl: EXAMPLE_GVL, body: "" },
			{ id: "fb_ejector", name: "FB_Ejector", kind: "FB", lang: "ST", decl: EXAMPLE_FB_DECL, body: EXAMPLE_FB_BODY },
			{ id: "plc_prg", name: "PLC_PRG", kind: "PRG", lang: "ST", decl: EXAMPLE_DECL, body: EXAMPLE_BODY },
			// the same conveyor in FBD and a sequence in SFC: not in MainTask (they would fight PLC_PRG for the
			// outputs); the task configuration or "allein ausführen" runs them
			{ id: "fbd_prg", name: "FBD_PRG", kind: "PRG", lang: "FBD", decl: "PROGRAM FBD_PRG\nVAR\nEND_VAR", body: "", networks: exampleFbd().networks },
			{ id: "sfc_prg", name: "SFC_PRG", kind: "PRG", lang: "SFC", ...(({ decl, chart }) => ({ decl, chart }))(exampleSfc()), body: "" },
		],
		tasks: [{ name: "MainTask", intervalCycles: 1, programs: ["PLC_PRG"] }],
	};
}

// ---- new objects -----------------------------------------------------------------------------------------
const IDENT = /^[A-Za-z_][A-Za-z0-9_]*$/;

/** A new object's declaration and implementation. */
export function template(kind: ObjectKind, name: string, returnType = "INT", lang: Lang = "ST"): Pick<PlcObject, "decl" | "body" | "networks" | "chart"> {
	const st = templateSt(kind, name, returnType);
	if (lang === "FBD") return { decl: st.decl, body: "", networks: [{ id: "n1", title: "", nodes: [], wires: [] }] };
	if (lang === "SFC") {
		return {
			decl: st.decl, body: "",
			chart: [{ kind: "step", id: sfcId(), name: "Init", actions: [] }, { kind: "trans", id: sfcId(), condition: "FALSE" }],
		};
	}
	return st;
}

function templateSt(kind: ObjectKind, name: string, returnType: string): { decl: string; body: string } {
	switch (kind) {
		case "PRG": return { decl: `PROGRAM ${name}\nVAR\nEND_VAR`, body: "" };
		case "FB": return { decl: `FUNCTION_BLOCK ${name}\nVAR_INPUT\nEND_VAR\nVAR_OUTPUT\nEND_VAR\nVAR\nEND_VAR`, body: "" };
		case "FUN": return { decl: `FUNCTION ${name} : ${returnType}\nVAR_INPUT\nEND_VAR\nVAR\nEND_VAR`, body: `${name} := 0;\n` };
		case "DUT": return { decl: `TYPE ${name} :\nSTRUCT\n    Value : INT;\nEND_STRUCT\nEND_TYPE`, body: "" };
		case "GVL": return { decl: "VAR_GLOBAL\nEND_VAR", body: "" };
	}
}

/** Why a name cannot be an object's new name (null: it can). */
export function nameProblem(p: Project, name: string, except?: string): string | null {
	if (!IDENT.test(name) || name.includes("__")) return "kein gültiger Bezeichner";
	if (p.objects.some((o) => o.id !== except && o.name.toUpperCase() === name.toUpperCase())) return "Name schon vergeben";
	return null;
}

export function addObject(p: Project, kind: ObjectKind, name: string, returnType?: string, lang: Lang = "ST"): { project: Project; object: PlcObject } {
	let id = name.toLowerCase();
	for (let i = 2; p.objects.some((o) => o.id === id); i++) id = `${name.toLowerCase()}_${i}`;
	const l = LANGS_OF[kind].includes(lang) ? lang : "ST";
	const object: PlcObject = { id, name, kind, lang: l, ...template(kind, name, returnType, l) };
	const tasks = kind === "PRG" ? p.tasks.map((t, i) => (i === 0 ? { ...t, programs: [...t.programs, name] } : t)) : p.tasks;
	return { project: { ...p, objects: [...p.objects, object], tasks }, object };
}

export function removeObject(p: Project, id: string): Project {
	const o = p.objects.find((x) => x.id === id);
	if (!o) return p;
	const up = o.name.toUpperCase();
	return {
		...p,
		objects: p.objects.filter((x) => x.id !== id),
		tasks: p.tasks.map((t) => ({ ...t, programs: t.programs.filter((n) => n.toUpperCase() !== up) })),
	};
}

/** Replaces the identifier `from` by `to` in ST text (case-insensitive; not in comments and strings,
 *  not after a '.': a field of the same name stays). Enum literals `From#Value` follow. */
export function renameIn(text: string, from: string, to: string): string {
	const key = from.toUpperCase();
	const toks = lex(text, "rename", new Diagnostics());
	const cuts: { at: number; len: number }[] = [];
	toks.forEach((t, i) => {
		const prev = toks[i - 1];
		if (prev && prev.kind === "op" && prev.text === ".") return;
		if ((t.kind === "ident" && t.key === key) || (t.kind === "typed" && t.key === key)) cuts.push({ at: t.span.start.offset, len: from.length });
	});
	let out = text;
	for (const c of cuts.reverse()) out = out.slice(0, c.at) + to + out.slice(c.at + c.len);
	return out;
}

/** Renames an object and every reference to it in the project (all declarations and implementations,
 *  the task configuration). */
export function renameObject(p: Project, id: string, to: string): Project {
	const o = p.objects.find((x) => x.id === id);
	if (!o) return p;
	const from = o.name;
	const up = from.toUpperCase();
	const refs = o.kind !== "GVL";  // a GVL's name is no identifier of the program
	const r = (t: string) => (refs ? renameIn(t, from, to) : t);
	return {
		...p,
		objects: p.objects.map((x) => ({
			...x,
			name: x.id === id ? to : x.name,
			decl: r(x.decl),
			body: r(x.body),
			...(x.networks ? { networks: x.networks.map((n) => renameInNetwork(n, r, refs ? from : null, to)) } : {}),
			...(x.chart ? { chart: renameInChart(x.chart, r) } : {}),
		})),
		tasks: p.tasks.map((t) => ({ ...t, programs: t.programs.map((n) => (n.toUpperCase() === up ? to : n)) })),
	};
}

/** The ST texts of an FBD network (input expressions, output targets) through `r`; boxes of the type
 *  `from` (a renamed FB or function) become `to`. */
function renameInNetwork(n: FbdNetwork, r: (t: string) => string, from: string | null, to: string): FbdNetwork {
	const up = from?.toUpperCase();
	return {
		...n,
		nodes: n.nodes.map((x) => (x.kind === "input" ? { ...x, expr: r(x.expr) } : x.kind === "output" ? { ...x, target: r(x.target) }
			: x.type.toUpperCase() === up ? { ...x, type: to } : x)),
	};
}

/** The ST texts of an SFC chart (conditions, action bodies) through `r`. */
function renameInChart(chart: SfcItem[], r: (t: string) => string): SfcItem[] {
	return chart.map((it): SfcItem => {
		if (it.kind === "trans") return { ...it, condition: r(it.condition) };
		if (it.kind === "step") return { ...it, actions: it.actions.map((a) => ({ ...a, body: r(a.body) })) };
		if (it.kind === "branch") return { ...it, branches: it.branches.map((b) => renameInChart(b, r)) };
		return it;
	});
}

/** The name in an object's header (PROGRAM x / FUNCTION_BLOCK x / FUNCTION x / TYPE x); null: none. */
export function headerName(o: PlcObject): string | null {
	if (o.kind === "GVL") return null;
	const m = /^\s*(?:PROGRAM|FUNCTION_BLOCK|FUNCTION|TYPE)\s+([A-Za-z_][A-Za-z0-9_]*)/i.exec(o.decl.replace(/\(\*[\s\S]*?\*\)/g, " "));
	return m ? m[1] : null;
}

// ---- folders ----------------------------------------------------------------------------------------------
/** A folder name as stored: trimmed, inner whitespace collapsed, no "/" (one level); "" = top level. */
export function cleanFolder(name: string): string {
	return name.replace(/\//g, " ").replace(/\s+/g, " ").trim();
}

export function setFolder(p: Project, id: string, folder: string): Project {
	const f = cleanFolder(folder);
	return { ...p, objects: p.objects.map((o) => (o.id === id ? { ...o, folder: f || undefined } : o)) };
}

/** The tree: the top-level objects first (project order), then the folders by name with their objects. */
export function objectTree(p: Project): { folder: string; objects: PlcObject[] }[] {
	const top = p.objects.filter((o) => !o.folder);
	const names = [...new Set(p.objects.map((o) => o.folder).filter((f): f is string => !!f))].sort((a, b) => a.localeCompare(b, "de"));
	return [{ folder: "", objects: top }, ...names.map((folder) => ({ folder, objects: p.objects.filter((o) => o.folder === folder) }))];
}

// ---- cross references -------------------------------------------------------------------------------------
export interface CrossRef {
	object: string;
	/** decl/body: a line of the declaration or the ST implementation; fbd: network `line` (1-based); sfc: the chart. */
	where: "decl" | "body" | "fbd" | "sfc";
	line: number;
	col: number;
	/** The line (decl/body) or the element's text (fbd/sfc). */
	text: string;
	/** The name is declared here (a variable's row, a POU header, an FBD instance, an SFC step). */
	declares: boolean;
}

/** Positions (1-based line, column) of the identifier `key` (upper case) in ST text: not in comments
 *  or strings, not as a field after '.'; enum literals `Name#Value` count. */
function occurrences(text: string, key: string): { line: number; col: number }[] {
	const toks = lex(text, "xref", new Diagnostics());
	const out: { line: number; col: number }[] = [];
	toks.forEach((t, i) => {
		const prev = toks[i - 1];
		if (prev && prev.kind === "op" && prev.text === ".") return;
		if ((t.kind === "ident" || t.kind === "typed") && t.key === key) out.push({ line: t.span.start.line, col: t.span.start.col });
	});
	return out;
}

/** Every use of a name in the project (variable, POU, type, instance, step): all objects, their
 *  declarations, implementations and graphics. By name, not by scope: a local variable of the same
 *  name in another POU is listed too (its object says which). */
export function crossReferences(p: Project, name: string): CrossRef[] {
	const key = name.trim().toUpperCase();
	if (!/^[A-Z_][A-Z0-9_]*$/.test(key)) return [];
	const declLine = new RegExp(`^\\s*(?:(?:PROGRAM|FUNCTION_BLOCK|FUNCTION|TYPE)\\s+)?${key}\\s*(?::|$)`, "i");
	const out: CrossRef[] = [];
	for (const o of p.objects) {
		const lines = o.decl.split("\n");
		for (const at of occurrences(o.decl, key)) {
			const text = lines[at.line - 1] ?? "";
			out.push({ object: o.id, where: "decl", ...at, text: text.trim(), declares: declLine.test(text) });
		}
		if (o.lang === "ST" && hasBody(o.kind)) {
			const body = o.body.split("\n");
			for (const at of occurrences(o.body, key)) out.push({ object: o.id, where: "body", ...at, text: (body[at.line - 1] ?? "").trim(), declares: false });
		}
		(o.networks ?? []).forEach((net, i) => {
			for (const n of net.nodes) {
				if (n.kind === "box") {
					if (n.instance?.toUpperCase() === key) out.push({ object: o.id, where: "fbd", line: i + 1, col: 0, text: `${n.instance} : ${n.type}`, declares: true });
					if (n.type.toUpperCase() === key) out.push({ object: o.id, where: "fbd", line: i + 1, col: 0, text: `Baustein ${n.type}`, declares: false });
					continue;
				}
				const t = n.kind === "input" ? n.expr : n.target;
				if (occurrences(t, key).length) out.push({ object: o.id, where: "fbd", line: i + 1, col: 0, text: n.kind === "input" ? `Eingang ${t}` : `Ausgang ${t}`, declares: false });
			}
		});
		const walk = (items: SfcItem[]) => {
			for (const it of items) {
				if (it.kind === "step") {
					if (it.name.toUpperCase() === key) out.push({ object: o.id, where: "sfc", line: 0, col: 0, text: `Schritt ${it.name}`, declares: true });
					for (const a of it.actions) if (occurrences(a.body, key).length) out.push({ object: o.id, where: "sfc", line: 0, col: 0, text: `${it.name}: ${a.qualifier} ${a.body}`, declares: false });
				} else if (it.kind === "trans") {
					if (occurrences(it.condition, key).length) out.push({ object: o.id, where: "sfc", line: 0, col: 0, text: `Transition ${it.condition}`, declares: false });
				} else if (it.kind === "jump") {
					if (it.target.toUpperCase() === key) out.push({ object: o.id, where: "sfc", line: 0, col: 0, text: `Sprung zu ${it.target}`, declares: false });
				} else walk(it.branches.flat());
			}
		};
		walk(o.chart ?? []);
	}
	return out;
}

// ---- storage ---------------------------------------------------------------------------------------------
const STORAGE_KEY = "fcu.plc.project";

/** Why the parsed JSON is no project (null: it is one). */
function projectProblem(p: Project): string | null {
	if (p?.format !== 1 || !Array.isArray(p.objects) || p.objects.length === 0) return "kein FCU-SPS-Projekt (Format 1)";
	const names = new Set<string>();
	for (const o of p.objects) {
		if (!OBJECT_KINDS.includes(o.kind) || typeof o.decl !== "string" || typeof o.name !== "string" || !IDENT.test(o.name)) return `Objekt '${o.name}' ungültig`;
		const lang = o.lang ?? "ST";
		if (!LANGS_OF[o.kind].includes(lang)) return `Objekt '${o.name}': Sprache ${lang} geht für ${o.kind} nicht`;
		if (lang === "FBD" && !Array.isArray(o.networks)) return `Objekt '${o.name}': FBD ohne Netzwerke`;
		if (lang === "SFC" && !Array.isArray(o.chart)) return `Objekt '${o.name}': SFC ohne Ablauf`;
		if (o.folder !== undefined && typeof o.folder !== "string") return `Objekt '${o.name}': Ordner ungültig`;
		if (names.has(o.name.toUpperCase())) return `Objektname '${o.name}' doppelt`;
		names.add(o.name.toUpperCase());
	}
	if (!Array.isArray(p.tasks) || p.tasks.length === 0) return "keine Task";
	return null;
}

function normalize(p: Project): Project {
	return { ...p, objects: p.objects.map((o, i) => ({ ...o, id: o.id || `obj${i}`, lang: o.lang ?? "ST", body: o.body ?? "" })) };
}

export function loadProject(): Project {
	try {
		const raw = localStorage.getItem(STORAGE_KEY);
		if (raw) {
			const p = JSON.parse(raw) as Project;
			if (projectProblem(p) === null) return normalize(p);
		}
	} catch {
		// private mode, broken data: start with the example
	}
	return exampleProject();
}

/** A project file's text (export, the device's SD card) as a project; throws why not. */
export function parseProject(text: string): Project {
	const p = JSON.parse(text) as Project;
	const problem = projectProblem(p);
	if (problem) throw new Error(problem);
	return normalize(p);
}

export function saveProject(p: Project): void {
	try {
		localStorage.setItem(STORAGE_KEY, JSON.stringify(p));
	} catch {
		// not persistent then
	}
}

// ---- build ------------------------------------------------------------------------------------------------
/** Where a diagnostic belongs in the IDE. */
export interface Located {
	/** The object (its id) for decl/body; null for io/other. */
	object: string | null;
	part: "decl" | "body" | "io" | "other";
	line: number; // 1-based within the part
	col: number;
	diagnostic: Diagnostic;
}

/** A graphical object's lowering: its ST and what the lowering could not take. */
export interface Generated {
	text: string;
	problems: (FbdProblem | SfcProblem)[];
}

export interface Build {
	result: CompileResult;
	located: Located[];
	/** The ST of the FBD/SFC objects (by id). */
	generated: Map<string, Generated>;
	/** Compiled, and no graphical object has a problem. */
	ok: boolean;
}

/** Line n of an object's compiled text is line n - declLines(o) of its implementation. */
export function declLines(o: PlcObject): number {
	return o.decl.split("\n").length;
}

/** A line of an object's compiled text as a part and a line of it. */
export function partLine(o: PlcObject, line: number): { part: "decl" | "body"; line: number } {
	const d = declLines(o);
	return hasBody(o.kind) && line > d ? { part: "body", line: line - d } : { part: "decl", line };
}

// ---- FBD boxes beyond the standard ones: the project's FBs and functions, the FCU library -------------
/** The boxes of the function blocks and functions in an ST text (their interface: VAR_INPUT and
 *  VAR_IN_OUT are inputs, VAR_OUTPUT outputs, a function's result is OUT). */
function boxesOf(text: string, file: string, group: string, title: (kind: "FB" | "FUN") => string): BoxDef[] {
	const typeOf = (t: A.TypeRef) => (t.kind === "named" ? t.name.toUpperCase() : t.kind === "string" ? "STRING" : "");
	const out: BoxDef[] = [];
	for (const u of parse(text, file, new Diagnostics())) {
		if (u.kind !== "pou" || u.pouKind === "PROGRAM") continue;
		const ins = u.vars.filter((v) => v.cls === "VAR_INPUT" || v.cls === "VAR_IN_OUT");
		const outs = u.vars.filter((v) => v.cls === "VAR_OUTPUT");
		const types: Record<string, string> = {};
		for (const v of [...ins, ...outs]) types[v.name] = typeOf(v.type);
		if (u.pouKind === "FUNCTION_BLOCK") {
			out.push({ type: u.name, kind: "fb", inputs: ins.map((v) => v.name), outputs: outs.map((v) => v.name), group, title: title("FB"), types, omit: true });
		} else {
			if (u.returnType) types.OUT = typeOf(u.returnType);
			out.push({ type: u.name, kind: "func", inputs: ins.map((v) => v.name), outputs: ["OUT"], group, title: title("FUN"), types });
		}
	}
	return out;
}

let fcuBoxCache: BoxDef[] | null = null;
function fcuBoxes(): BoxDef[] {
	fcuBoxCache ??= boxesOf(fcuLibSource(), FCU_LIB_FILE, "FCU-Bibliothek", (k) => (k === "FB" ? "Funktionsbaustein der FCU-Bibliothek" : "Funktion der FCU-Bibliothek"));
	return fcuBoxCache;
}

/** The boxes an FBD object can use beyond the standard ones: the project's other FBs and functions (from
 *  their declarations; not itself - no recursion) and the FCU library. */
export function boxesFor(p: Project, o: PlcObject): BoxDef[] {
	const own = p.objects.filter((x) => x.id !== o.id && (x.kind === "FB" || x.kind === "FUN"))
		.flatMap((x) => boxesOf(`${x.decl}\n${END[x.kind]}\n`, x.name, "Projekt", (k) => (k === "FB" ? "Funktionsbaustein des Projekts" : "Funktion des Projekts")));
	return [...own, ...fcuBoxes()];
}

/** An object as ST (FBD/SFC: lowered; the problems of the lowering are in generate()). An FBD needs its
 *  project for the project's FBs and functions as boxes. */
export function sourceOf(o: PlcObject, p?: Project): string {
	return generate(o, p).text;
}

export function generate(o: PlcObject, p?: Project): Generated {
	if (o.lang === "FBD") return fbdSource({ name: o.name, decl: o.decl, networks: o.networks ?? [], extra: p ? boxesFor(p, o) : fcuBoxes() }, END[o.kind]);
	if (o.lang === "SFC") return sfcSource({ name: o.name, decl: o.decl, chart: o.chart ?? [] }, END[o.kind]);
	return { text: hasBody(o.kind) ? `${o.decl}\n${o.body}\n${END[o.kind]}\n` : `${o.decl}\n`, problems: [] };
}

/** The project's objects as source files. */
export function objectSources(p: Project): { file: string; text: string }[] {
	return p.objects.map((o) => ({ file: o.name, text: sourceOf(o, p) }));
}

/** Sources for the FCU: the given ones, the I/O variables and the FCU library. */
export function deviceSources(file: string, text: string): { file: string; text: string }[];
export function deviceSources(sources: { file: string; text: string }[]): { file: string; text: string }[];
export function deviceSources(a: string | { file: string; text: string }[], text?: string): { file: string; text: string }[] {
	const own = typeof a === "string" ? [{ file: a, text: text! }] : a;
	return [...own, { file: IO_FILE, text: ioSource() }, { file: FCU_LIB_FILE, text: fcuLibSource() }, { file: SFC_FILE, text: SFC_STEP_SOURCE }];
}

/** The project hash an image of the project's current state has (also while it does not compile). */
export function projectHash(p: Project): number {
	return projectHashOf(deviceSources(objectSources(p)));
}

export function build(p: Project, tasks = p.tasks): Build {
	const generated = new Map<string, Generated>();
	const sources = p.objects.map((o) => {
		const g = generate(o, p);
		if (o.lang !== "ST") generated.set(o.id, g);
		return { file: o.name, text: g.text };
	});
	const result = compile(deviceSources(sources), tasks, { io: ioBindings(), ioMapHash: ioMapHash() });
	const byFile = new Map(p.objects.map((o) => [o.name, o]));
	const located = result.diagnostics.map((d): Located => {
		const s = d.span.start;
		if (s.file === IO_FILE) return { object: null, part: "io", line: s.line, col: s.col, diagnostic: d };
		const o = byFile.get(s.file);
		if (!o) return { object: null, part: "other", line: s.line, col: s.col, diagnostic: d };
		return { object: o.id, ...partLine(o, s.line), col: s.col, diagnostic: d };
	});
	const ok = result.ok && [...generated.values()].every((g) => g.problems.length === 0);
	return { result, located, generated, ok };
}
