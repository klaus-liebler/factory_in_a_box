// The declaration part as a table (CODESYS tabular declaration, OpenPLC variables table): rows are
// read from the text and written back into it, so the table and the text are the same thing. Lines
// the table does not understand stay as they are.

export const VAR_CLASSES = ["VAR", "VAR_INPUT", "VAR_OUTPUT", "VAR_IN_OUT", "VAR_TEMP"] as const;
export type VarClass = (typeof VAR_CLASSES)[number];
/** The classes of a global variable list. */
export const GVL_CLASSES = ["VAR_GLOBAL"] as const;

export interface DeclRow {
	/** Index of the line in the declaration text. */
	line: number;
	cls: string;
	qualifier: string; // CONSTANT, RETAIN or ""
	name: string;
	type: string;
	init: string;
	comment: string;
}

const SECTION = /^\s*(VAR(?:_INPUT|_OUTPUT|_IN_OUT|_TEMP|_GLOBAL|_EXTERNAL)?)\b\s*(CONSTANT|RETAIN|PERSISTENT)?\s*$/i;
const ROW = /^\s*([A-Za-z_]\w*)\s*:\s*([^:;]+?)\s*(?::=\s*(.+?))?\s*;\s*(?:\(\*\s*(.*?)\s*\*\))?\s*$/;

export function parseRows(decl: string): DeclRow[] {
	const rows: DeclRow[] = [];
	let cls = "";
	let qualifier = "";
	decl.split("\n").forEach((text, line) => {
		const s = SECTION.exec(text);
		if (s) {
			cls = s[1].toUpperCase();
			qualifier = (s[2] ?? "").toUpperCase();
			return;
		}
		if (/^\s*END_VAR\b/i.test(text)) {
			cls = "";
			return;
		}
		const r = cls ? ROW.exec(text) : null;
		if (r) rows.push({ line, cls, qualifier, name: r[1], type: r[2].trim(), init: r[3]?.trim() ?? "", comment: r[4] ?? "" });
	});
	return rows;
}

export function formatRow(r: Pick<DeclRow, "name" | "type" | "init" | "comment">): string {
	let s = `    ${r.name} : ${r.type}${r.init ? ` := ${r.init}` : ""};`;
	if (r.comment) s = `${s.padEnd(32)} (* ${r.comment.replace(/\*\)/g, "* )")} *)`;
	return s;
}

/** Replaces the row's line; a row may also move to another section (class). */
export function updateRow(decl: string, row: DeclRow, change: Partial<DeclRow>): string {
	const next = { ...row, ...change };
	if (next.cls !== row.cls || next.qualifier !== row.qualifier) {
		return addRow(deleteRow(decl, row), next);
	}
	const lines = decl.split("\n");
	lines[row.line] = formatRow(next);
	return lines.join("\n");
}

export function deleteRow(decl: string, row: DeclRow): string {
	const lines = decl.split("\n");
	lines.splice(row.line, 1);
	return lines.join("\n");
}

/** Appends the row to the last section of its class (and qualifier), or a new section. */
export function addRow(decl: string, row: Pick<DeclRow, "cls" | "qualifier" | "name" | "type" | "init" | "comment">): string {
	const lines = decl.split("\n");
	let cls = "";
	let qual = "";
	let insertAt = -1;
	lines.forEach((text, i) => {
		const s = SECTION.exec(text);
		if (s) {
			cls = s[1].toUpperCase();
			qual = (s[2] ?? "").toUpperCase();
		} else if (/^\s*END_VAR\b/i.test(text)) {
			if (cls === row.cls && qual === row.qualifier) insertAt = i;
			cls = "";
		}
	});
	if (insertAt >= 0) {
		lines.splice(insertAt, 0, formatRow(row));
	} else {
		lines.push(`${row.cls}${row.qualifier ? " " + row.qualifier : ""}`, formatRow(row), "END_VAR");
	}
	return lines.join("\n");
}

/** A free name: Var1, Var2, ... */
export function freeName(rows: DeclRow[], base = "Var"): string {
	const taken = new Set(rows.map((r) => r.name.toUpperCase()));
	for (let i = 1; ; i++) if (!taken.has(`${base}${i}`.toUpperCase())) return `${base}${i}`;
}
