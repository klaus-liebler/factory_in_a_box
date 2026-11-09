// The PLC project (docs/plc-iec61131.md section 11): Stage 1 has exactly one program PLC_PRG in
// ST. Declaration and implementation are edited separately; for the compiler they are one text.
import { compile, type CompileResult } from "../compiler/compiler.ts";
import type { Diagnostic } from "../compiler/diagnostics.ts";
import { IO_FILE, ioBindings, ioMapHash, ioSource } from "./io-list.ts";

export interface PlcObject {
	id: string;
	name: string;
	kind: "PRG";
	lang: "ST";
	/** "PROGRAM PLC_PRG" and the VAR sections. */
	decl: string;
	/** The statements (without END_PROGRAM). */
	body: string;
}

export interface Project {
	format: 1;
	name: string;
	objects: PlcObject[];
	tasks: { name: string; intervalCycles: number; programs: string[] }[];
}

export const EXAMPLE_DECL = `PROGRAM PLC_PRG
VAR
    Enable : BOOL := TRUE;      (* conveyor on *)
    Pieces : CTU;               (* pieces seen at light barrier 1 *)
    AtGate : R_TRIG;            (* a piece reaches light barrier 2 *)
    Eject : TP;                 (* pulse for the ejector valve *)
    Ejected : INT;
END_VAR`;

export const EXAMPLE_BODY = `(* Sorting conveyor: stops after 10 pieces, every second piece is
   pushed off at light barrier 2 *)
Pieces(CU := Lightbarrier1, R := NOT Enable, PV := 10);
AtGate(CLK := Lightbarrier2);

IF Enable AND NOT Pieces.Q THEN
    ConveyorPwm := 600;
ELSE
    ConveyorPwm := 0;
END_IF;

Eject(IN := AtGate.Q AND (Pieces.CV MOD 2 = 0), PT := T#300ms);
Valve1 := Eject.Q;
IF AtGate.Q AND (Pieces.CV MOD 2 = 0) THEN
    Ejected := Ejected + 1;
END_IF;
`;

export function exampleProject(): Project {
	return {
		format: 1,
		name: "Sortieranlage",
		objects: [{ id: "plc_prg", name: "PLC_PRG", kind: "PRG", lang: "ST", decl: EXAMPLE_DECL, body: EXAMPLE_BODY }],
		tasks: [{ name: "MainTask", intervalCycles: 1, programs: ["PLC_PRG"] }],
	};
}

const STORAGE_KEY = "fcu.plc.project";

export function loadProject(): Project {
	try {
		const raw = localStorage.getItem(STORAGE_KEY);
		if (raw) {
			const p = JSON.parse(raw) as Project;
			if (p.format === 1 && p.objects?.length === 1) return p;
		}
	} catch {
		// private mode, broken data: start with the example
	}
	return exampleProject();
}

export function saveProject(p: Project): void {
	try {
		localStorage.setItem(STORAGE_KEY, JSON.stringify(p));
	} catch {
		// not persistent then
	}
}

/** Where a diagnostic belongs in the IDE. */
export interface Located {
	part: "decl" | "body" | "io" | "other";
	line: number; // 1-based within the part
	col: number;
	diagnostic: Diagnostic;
}

export interface Build {
	result: CompileResult;
	located: Located[];
	/** Lines of the declaration: line n of the compiled text is body line n - declLines - 1. */
	declLines: number;
}

export function sourceOf(o: PlcObject): string {
	return `${o.decl}\n${o.body}\nEND_PROGRAM\n`;
}

export function build(p: Project): Build {
	const o = p.objects[0];
	const declLines = o.decl.split("\n").length;
	const result = compile([{ file: o.name, text: sourceOf(o) }, { file: IO_FILE, text: ioSource() }], p.tasks,
		{ io: ioBindings(), ioMapHash: ioMapHash() });
	const located = result.diagnostics.map((d): Located => {
		const s = d.span.start;
		if (s.file === IO_FILE) return { part: "io", line: s.line, col: s.col, diagnostic: d };
		if (s.file !== o.name) return { part: "other", line: s.line, col: s.col, diagnostic: d };
		if (s.line <= declLines) return { part: "decl", line: s.line, col: s.col, diagnostic: d };
		return { part: "body", line: s.line - declLines, col: s.col, diagnostic: d };
	});
	return { result, located, declLines };
}
