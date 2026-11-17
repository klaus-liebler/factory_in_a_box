// SPDX-License-Identifier: MIT
// IEC 61131-3 lexer (Ed. 3): keywords and identifiers are case-insensitive, comments (* *) and //,
// pragmas { } are skipped. Numeric literals keep their text; the parser interprets them. DOM-free.
import type { Diagnostics, Pos, Span } from "./diagnostics.ts";

export type TokKind =
	| "ident" | "keyword" | "int" | "real" | "typed" | "time" | "string" | "direct" | "op" | "eof";

export interface Token {
	kind: TokKind;
	/** keyword: upper case; ident: as written; op: the operator. */
	text: string;
	/** ident: upper case (for lookups). */
	key: string;
	span: Span;
}

export const KEYWORDS = new Set([
	"PROGRAM", "END_PROGRAM", "FUNCTION", "END_FUNCTION", "FUNCTION_BLOCK", "END_FUNCTION_BLOCK",
	"VAR", "VAR_INPUT", "VAR_OUTPUT", "VAR_IN_OUT", "VAR_TEMP", "VAR_GLOBAL", "VAR_EXTERNAL", "END_VAR",
	"CONSTANT", "RETAIN", "NON_RETAIN", "PERSISTENT", "AT",
	"TYPE", "END_TYPE", "STRUCT", "END_STRUCT", "ARRAY", "OF", "STRING", "WSTRING",
	"IF", "THEN", "ELSIF", "ELSE", "END_IF", "CASE", "END_CASE", "FOR", "TO", "BY", "DO", "END_FOR",
	"WHILE", "END_WHILE", "REPEAT", "UNTIL", "END_REPEAT", "EXIT", "CONTINUE", "RETURN",
	"AND", "OR", "XOR", "NOT", "MOD", "TRUE", "FALSE",
	"NAMESPACE", "END_NAMESPACE", "USING", "REF_TO", "METHOD", "INTERFACE", "EXTENDS", "IMPLEMENTS",
	"CONFIGURATION", "RESOURCE", "TASK", "WITH", "INITIAL_STEP", "STEP", "TRANSITION", "ACTION",
]);

// longest first
const OPERATORS = [":=", "=>", "<=", ">=", "<>", "**", "..", "+", "-", "*", "/", "(", ")", "[", "]", ",", ";", ":",
	".", "=", "<", ">", "&", "#", "^"];

const isLetter = (c: string) => (c >= "a" && c <= "z") || (c >= "A" && c <= "Z") || c === "_";
const isDigit = (c: string) => c >= "0" && c <= "9";

export function lex(text: string, file: string, diag: Diagnostics): Token[] {
	const tokens: Token[] = [];
	let i = 0;
	let line = 1;
	let lineStart = 0;
	const pos = (): Pos => ({ file, line, col: i - lineStart + 1, offset: i });
	const advance = (n: number) => {
		for (let k = 0; k < n; k++) {
			if (text[i] === "\n") {
				line++;
				lineStart = i + 1;
			}
			i++;
		}
	};
	const push = (kind: TokKind, start: Pos, t: string, key = t) => {
		tokens.push({ kind, text: t, key, span: { start, end: pos() } });
	};

	while (i < text.length) {
		const c = text[i];
		if (c === " " || c === "\t" || c === "\r" || c === "\n") {
			advance(1);
			continue;
		}
		const start = pos();
		if (c === "(" && text[i + 1] === "*") {
			const end = text.indexOf("*)", i + 2);
			if (end < 0) {
				diag.error({ start, end: start }, "comment not closed: missing '*)'");
				advance(text.length - i);
			} else {
				advance(end + 2 - i);
			}
			continue;
		}
		if (c === "/" && text[i + 1] === "/") {
			while (i < text.length && text[i] !== "\n") advance(1);
			continue;
		}
		if (c === "{") {
			const end = text.indexOf("}", i + 1);
			advance(end < 0 ? text.length - i : end + 1 - i);
			continue;
		}
		if (isLetter(c)) {
			let j = i;
			while (j < text.length && (isLetter(text[j]) || isDigit(text[j]))) j++;
			const word = text.slice(i, j);
			const up = word.toUpperCase();
			// typed literal: TYPE#value (INT#5, T#1s, TIME#1s, E#Value for enums)
			if (text[j] === "#") {
				let k = j + 1;
				const timeLike = up === "T" || up === "TIME" || up === "LT" || up === "LTIME";
				if (timeLike) {
					while (k < text.length && /[0-9a-zA-Z_.\-+]/.test(text[k])) k++;
					advance(k - i);
					push("time", start, text.slice(start.offset, i), up);
					continue;
				}
				if (k < text.length && (isLetter(text[k]) || isDigit(text[k]) || text[k] === "-" || text[k] === "+")) {
					if (text[k] === "-" || text[k] === "+") k++;
					while (k < text.length && /[0-9a-zA-Z_.#]/.test(text[k])) {
						// a following 'E+'/'E-' belongs to a real exponent
						if ((text[k] === "e" || text[k] === "E") && (text[k + 1] === "+" || text[k + 1] === "-")) k++;
						k++;
					}
					advance(k - i);
					push("typed", start, text.slice(start.offset, i), up);
					continue;
				}
			}
			advance(j - i);
			if (KEYWORDS.has(up)) push("keyword", start, up, up);
			else {
				if (word.includes("__") && !file.startsWith("<")) {
					diag.error({ start, end: pos() }, `'${word}': an identifier must not contain two underscores in a row`);
				}
				push("ident", start, word, up);
			}
			continue;
		}
		if (isDigit(c)) {
			let j = i;
			// based literal 2#..., 8#..., 16#...
			while (j < text.length && (isDigit(text[j]) || text[j] === "_")) j++;
			if (text[j] === "#") {
				j++;
				while (j < text.length && /[0-9a-fA-F_]/.test(text[j])) j++;
				advance(j - i);
				push("int", start, text.slice(start.offset, i));
				continue;
			}
			let kind: TokKind = "int";
			if (text[j] === "." && isDigit(text[j + 1] ?? "")) {
				kind = "real";
				j++;
				while (j < text.length && (isDigit(text[j]) || text[j] === "_")) j++;
			}
			if ((text[j] === "e" || text[j] === "E") && /[0-9+-]/.test(text[j + 1] ?? "")) {
				kind = "real";
				j++;
				if (text[j] === "+" || text[j] === "-") j++;
				while (j < text.length && isDigit(text[j])) j++;
			}
			advance(j - i);
			push(kind, start, text.slice(start.offset, i));
			continue;
		}
		if (c === "%") {
			let j = i + 1;
			while (j < text.length && /[A-Za-z0-9.*]/.test(text[j])) j++;
			advance(j - i);
			push("direct", start, text.slice(start.offset, i).toUpperCase());
			continue;
		}
		if (c === "'" || c === '"') {
			let j = i + 1;
			while (j < text.length && text[j] !== c && text[j] !== "\n") j += text[j] === "$" ? 2 : 1;
			if (text[j] !== c) diag.error({ start, end: start }, "string not closed");
			advance(Math.min(j + 1, text.length) - i);
			push("string", start, text.slice(start.offset, i));
			continue;
		}
		const op = OPERATORS.find((o) => text.startsWith(o, i));
		if (op) {
			advance(op.length);
			push("op", start, op);
			continue;
		}
		advance(1);
		diag.error({ start, end: pos() }, `unexpected character '${c}'`);
	}
	const end = pos();
	tokens.push({ kind: "eof", text: "", key: "", span: { start: end, end } });
	return tokens;
}
