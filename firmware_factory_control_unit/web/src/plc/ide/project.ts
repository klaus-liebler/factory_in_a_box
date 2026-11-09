// The PLC project (docs/plc-iec61131.md sections 5.8 and 11): from P4 any number of objects - programs,
// function blocks, functions, data types, global variable lists - each its own source file for the
// compiler (the file name is the object's name). Declaration and implementation are edited
// separately; for the compiler they are one text.
import { compile, projectHashOf, type CompileResult } from "../compiler/compiler.ts";
import { Diagnostics, type Diagnostic } from "../compiler/diagnostics.ts";
import { lex } from "../compiler/lexer.ts";
import { FCU_LIB_FILE, fcuLibSource } from "./fcu-lib.ts";
import { IO_FILE, ioBindings, ioMapHash, ioSource } from "./io-list.ts";

export type ObjectKind = "PRG" | "FB" | "FUN" | "DUT" | "GVL";
export const OBJECT_KINDS: readonly ObjectKind[] = ["PRG", "FB", "FUN", "DUT", "GVL"];
export const KIND_NAMES: Record<ObjectKind, string> = {
	PRG: "Programm", FB: "Funktionsbaustein", FUN: "Funktion", DUT: "Datentyp", GVL: "Globale Variablenliste",
};

export interface PlcObject {
	id: string;
	name: string;
	kind: ObjectKind;
	lang: "ST";
	/** PRG/FB/FUN: the header ("PROGRAM PLC_PRG") and the VAR sections; DUT: the whole TYPE ... END_TYPE;
	 *  GVL: the VAR_GLOBAL sections. */
	decl: string;
	/** PRG/FB/FUN: the statements (without END_...); DUT/GVL: empty. */
	body: string;
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
		],
		tasks: [{ name: "MainTask", intervalCycles: 1, programs: ["PLC_PRG"] }],
	};
}

// ---- new objects -----------------------------------------------------------------------------------------
const IDENT = /^[A-Za-z_][A-Za-z0-9_]*$/;

/** A new object's declaration and implementation. */
export function template(kind: ObjectKind, name: string, returnType = "INT"): { decl: string; body: string } {
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

export function addObject(p: Project, kind: ObjectKind, name: string, returnType?: string): { project: Project; object: PlcObject } {
	let id = name.toLowerCase();
	for (let i = 2; p.objects.some((o) => o.id === id); i++) id = `${name.toLowerCase()}_${i}`;
	const object: PlcObject = { id, name, kind, lang: "ST", ...template(kind, name, returnType) };
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
	return {
		...p,
		objects: p.objects.map((x) => ({
			...x,
			name: x.id === id ? to : x.name,
			decl: refs ? renameIn(x.decl, from, to) : x.decl,
			body: refs ? renameIn(x.body, from, to) : x.body,
		})),
		tasks: p.tasks.map((t) => ({ ...t, programs: t.programs.map((n) => (n.toUpperCase() === up ? to : n)) })),
	};
}

/** The name in an object's header (PROGRAM x / FUNCTION_BLOCK x / FUNCTION x / TYPE x); null: none. */
export function headerName(o: PlcObject): string | null {
	if (o.kind === "GVL") return null;
	const m = /^\s*(?:PROGRAM|FUNCTION_BLOCK|FUNCTION|TYPE)\s+([A-Za-z_][A-Za-z0-9_]*)/i.exec(o.decl.replace(/\(\*[\s\S]*?\*\)/g, " "));
	return m ? m[1] : null;
}

// ---- storage ---------------------------------------------------------------------------------------------
const STORAGE_KEY = "fcu.plc.project";

/** Why the parsed JSON is no project (null: it is one). */
function projectProblem(p: Project): string | null {
	if (p?.format !== 1 || !Array.isArray(p.objects) || p.objects.length === 0) return "kein FCU-SPS-Projekt (Format 1)";
	const names = new Set<string>();
	for (const o of p.objects) {
		if (!OBJECT_KINDS.includes(o.kind) || typeof o.decl !== "string" || typeof o.name !== "string" || !IDENT.test(o.name)) return `Objekt '${o.name}' ungültig`;
		if (names.has(o.name.toUpperCase())) return `Objektname '${o.name}' doppelt`;
		names.add(o.name.toUpperCase());
	}
	if (!Array.isArray(p.tasks) || p.tasks.length === 0) return "keine Task";
	return null;
}

function normalize(p: Project): Project {
	return { ...p, objects: p.objects.map((o, i) => ({ ...o, id: o.id || `obj${i}`, lang: "ST", body: o.body ?? "" })) };
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

export interface Build {
	result: CompileResult;
	located: Located[];
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

export function sourceOf(o: PlcObject): string {
	return hasBody(o.kind) ? `${o.decl}\n${o.body}\n${END[o.kind]}\n` : `${o.decl}\n`;
}

/** The project's objects as source files. */
export function objectSources(p: Project): { file: string; text: string }[] {
	return p.objects.map((o) => ({ file: o.name, text: sourceOf(o) }));
}

/** Sources for the FCU: the given ones, the I/O variables and the FCU library. */
export function deviceSources(file: string, text: string): { file: string; text: string }[];
export function deviceSources(sources: { file: string; text: string }[]): { file: string; text: string }[];
export function deviceSources(a: string | { file: string; text: string }[], text?: string): { file: string; text: string }[] {
	const own = typeof a === "string" ? [{ file: a, text: text! }] : a;
	return [...own, { file: IO_FILE, text: ioSource() }, { file: FCU_LIB_FILE, text: fcuLibSource() }];
}

/** The project hash an image of the project's current state has (also while it does not compile). */
export function projectHash(p: Project): number {
	return projectHashOf(deviceSources(objectSources(p)));
}

export function build(p: Project): Build {
	const result = compile(deviceSources(objectSources(p)), p.tasks, { io: ioBindings(), ioMapHash: ioMapHash() });
	const byFile = new Map(p.objects.map((o) => [o.name, o]));
	const located = result.diagnostics.map((d): Located => {
		const s = d.span.start;
		if (s.file === IO_FILE) return { object: null, part: "io", line: s.line, col: s.col, diagnostic: d };
		const o = byFile.get(s.file);
		if (!o) return { object: null, part: "other", line: s.line, col: s.col, diagnostic: d };
		return { object: o.id, ...partLine(o, s.line), col: s.col, diagnostic: d };
	});
	return { result, located };
}
