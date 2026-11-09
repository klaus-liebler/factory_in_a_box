// SPDX-License-Identifier: MIT
// Source positions and diagnostics of the IEC 61131-3 compiler. DOM-free.

export interface Pos {
	/** Name of the source (a POU, a GVL, "stdlib"). */
	file: string;
	/** 1-based. */
	line: number;
	/** 1-based, in UTF-16 code units. */
	col: number;
	/** Offset into the source text. */
	offset: number;
}

export interface Span {
	start: Pos;
	end: Pos;
}

export type Severity = "error" | "warning";

export interface Diagnostic {
	severity: Severity;
	message: string;
	span: Span;
}

export class Diagnostics {
	readonly list: Diagnostic[] = [];
	error(span: Span, message: string): void {
		this.list.push({ severity: "error", message, span });
	}
	warning(span: Span, message: string): void {
		this.list.push({ severity: "warning", message, span });
	}
	get hasErrors(): boolean {
		return this.list.some((d) => d.severity === "error");
	}
}

export function formatDiagnostic(d: Diagnostic): string {
	const p = d.span.start;
	return `${p.file}:${p.line}:${p.col}: ${d.severity}: ${d.message}`;
}
