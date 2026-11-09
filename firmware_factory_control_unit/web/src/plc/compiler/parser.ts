// SPDX-License-Identifier: MIT
// Recursive-descent parser for IEC 61131-3 Structured Text (Ed. 3): POUs, TYPE, VAR_GLOBAL.
// Error recovery: a broken statement or declaration is skipped up to the next ';', so one run
// reports several errors. DOM-free.
import type * as A from "./ast.ts";
import type { Diagnostics, Span } from "./diagnostics.ts";
import { lex, type Token } from "./lexer.ts";

class ParseError {}

const VAR_SECTIONS = new Set(["VAR", "VAR_INPUT", "VAR_OUTPUT", "VAR_IN_OUT", "VAR_TEMP", "VAR_GLOBAL", "VAR_EXTERNAL"]);
const STMT_END = new Set(["END_IF", "ELSIF", "ELSE", "END_CASE", "END_FOR", "END_WHILE", "UNTIL", "END_REPEAT",
	"END_PROGRAM", "END_FUNCTION", "END_FUNCTION_BLOCK"]);

/** Elementary type names a typed literal (INT#5) may start with. */
export const ELEMENTARY = new Set(["BOOL", "SINT", "INT", "DINT", "LINT", "USINT", "UINT", "UDINT", "ULINT", "BYTE",
	"WORD", "DWORD", "LWORD", "REAL", "LREAL", "TIME"]);

export function parse(text: string, file: string, diag: Diagnostics): A.Unit[] {
	return new Parser(lex(text, file, diag), file, diag).units();
}

/** TIME literal (T#1d2h3m4s5ms, T#1.5s, T#-250ms) to milliseconds; null: malformed. */
export function parseTime(body: string): number | null {
	let s = body.replace(/_/g, "").toLowerCase();
	let sign = 1;
	if (s.startsWith("-")) {
		sign = -1;
		s = s.slice(1);
	} else if (s.startsWith("+")) s = s.slice(1);
	if (s === "") return null;
	const units: Record<string, number> = { d: 86_400_000, h: 3_600_000, m: 60_000, s: 1000, ms: 1, us: 0.001, ns: 0.000001 };
	let total = 0;
	const re = /(\d+(?:\.\d+)?)(ms|us|ns|d|h|m|s)/y;
	let at = 0;
	while (at < s.length) {
		re.lastIndex = at;
		const m = re.exec(s);
		if (!m) return null;
		total += Number(m[1]) * units[m[2]];
		at = re.lastIndex;
	}
	return sign * total;
}

/** Integer literal text (with _ and 2#/8#/16#) to a bigint; null: malformed. */
export function parseIntText(t: string): bigint | null {
	let s = t.replace(/_/g, "");
	let neg = false;
	if (s.startsWith("-")) {
		neg = true;
		s = s.slice(1);
	} else if (s.startsWith("+")) s = s.slice(1);
	const hash = s.indexOf("#");
	let v: bigint;
	if (hash >= 0) {
		const base = s.slice(0, hash);
		const digits = s.slice(hash + 1);
		const prefix = base === "2" ? "0b" : base === "8" ? "0o" : base === "16" ? "0x" : null;
		if (prefix === null || digits === "") return null;
		try {
			v = BigInt(prefix + digits);
		} catch {
			return null;
		}
	} else {
		if (!/^\d+$/.test(s)) return null;
		v = BigInt(s);
	}
	return neg ? -v : v;
}

class Parser {
	private i = 0;
	constructor(private readonly t: Token[], private readonly file: string, private readonly diag: Diagnostics) {}

	// ---- token helpers ----
	private get tok(): Token {
		return this.t[this.i];
	}
	private peek(k = 1): Token {
		return this.t[Math.min(this.i + k, this.t.length - 1)];
	}
	private next(): Token {
		const t = this.t[this.i];
		if (t.kind !== "eof") this.i++;
		return t;
	}
	private isKw(k: string): boolean {
		return this.tok.kind === "keyword" && this.tok.text === k;
	}
	private isOp(o: string): boolean {
		return this.tok.kind === "op" && this.tok.text === o;
	}
	private acceptKw(k: string): boolean {
		if (!this.isKw(k)) return false;
		this.next();
		return true;
	}
	private acceptOp(o: string): boolean {
		if (!this.isOp(o)) return false;
		this.next();
		return true;
	}
	private fail(message: string, span: Span = this.tok.span): never {
		this.diag.error(span, message);
		throw new ParseError();
	}
	private describe(t: Token): string {
		return t.kind === "eof" ? "end of text" : `'${t.text}'`;
	}
	private expectKw(k: string): Token {
		if (!this.isKw(k)) this.fail(`expected ${k}, found ${this.describe(this.tok)}`);
		return this.next();
	}
	private expectOp(o: string): Token {
		if (!this.isOp(o)) this.fail(`expected '${o}', found ${this.describe(this.tok)}`);
		return this.next();
	}
	private ident(): Token {
		if (this.tok.kind !== "ident") this.fail(`expected a name, found ${this.describe(this.tok)}`);
		return this.next();
	}
	private span(from: Span): Span {
		const prev = this.t[Math.max(this.i - 1, 0)];
		return { start: from.start, end: prev.span.end };
	}
	/** Skip to after the next ';' (or up to a block end keyword). */
	private recover(): void {
		while (this.tok.kind !== "eof") {
			if (this.isOp(";")) {
				this.next();
				return;
			}
			if (this.tok.kind === "keyword" && (STMT_END.has(this.tok.text) || this.tok.text === "END_VAR" ||
				this.tok.text === "END_TYPE" || this.tok.text === "END_STRUCT")) return;
			this.next();
		}
	}

	// ---- units ----
	units(): A.Unit[] {
		const out: A.Unit[] = [];
		while (this.tok.kind !== "eof") {
			try {
				if (this.isKw("PROGRAM") || this.isKw("FUNCTION_BLOCK") || this.isKw("FUNCTION")) out.push(this.pou());
				else if (this.isKw("TYPE")) out.push(...this.types());
				else if (this.isKw("VAR_GLOBAL")) {
					const start = this.tok.span;
					out.push({ kind: "globals", vars: this.varSection(), span: this.span(start) });
				} else this.fail(`expected PROGRAM, FUNCTION_BLOCK, FUNCTION, TYPE or VAR_GLOBAL, found ${this.describe(this.tok)}`);
			} catch (e) {
				if (!(e instanceof ParseError)) throw e;
				// skip to the next unit
				const starts = ["PROGRAM", "FUNCTION_BLOCK", "FUNCTION", "TYPE", "VAR_GLOBAL"];
				for (let t = this.tok; t.kind !== "eof" && !(t.kind === "keyword" && starts.includes(t.text)); t = this.tok) this.next();
			}
		}
		return out;
	}

	private pou(): A.Pou {
		const start = this.tok.span;
		const kw = this.next().text as A.PouKind;
		const name = this.ident();
		let returnType: A.TypeRef | null = null;
		if (kw === "FUNCTION") {
			this.expectOp(":");
			returnType = this.typeRef();
		}
		const vars: A.VarDecl[] = [];
		while (this.tok.kind === "keyword" && VAR_SECTIONS.has(this.tok.text)) vars.push(...this.varSection());
		const end = `END_${kw}`;
		const body = this.statements(new Set([end]));
		if (!this.acceptKw(end)) this.diag.error(this.tok.span, `expected ${end}, found ${this.describe(this.tok)}`);
		return { kind: "pou", pouKind: kw, name: name.text, key: name.key, returnType, vars, body, span: this.span(start), file: this.file };
	}

	private varSection(): A.VarDecl[] {
		const cls = this.next().text as A.VarClass;
		let constant = false;
		let retain = false;
		for (;;) {
			if (this.acceptKw("CONSTANT")) constant = true;
			else if (this.acceptKw("RETAIN") || this.acceptKw("PERSISTENT")) retain = true;
			else if (this.acceptKw("NON_RETAIN")) retain = false;
			else break;
		}
		const out: A.VarDecl[] = [];
		while (!this.isKw("END_VAR") && this.tok.kind !== "eof") {
			try {
				out.push(...this.varDecl(cls, constant, retain));
			} catch (e) {
				if (!(e instanceof ParseError)) throw e;
				this.recover();
			}
		}
		this.expectKw("END_VAR");
		return out;
	}

	private varDecl(cls: A.VarClass, constant: boolean, retain: boolean): A.VarDecl[] {
		const names: Token[] = [this.ident()];
		while (this.acceptOp(",")) names.push(this.ident());
		let at: string | null = null;
		if (this.acceptKw("AT")) {
			if (this.tok.kind !== "direct") this.fail("expected a direct address (%IX0.0, %QW20, ...)");
			at = this.next().text;
		}
		this.expectOp(":");
		const type = this.typeRef();
		const init = this.acceptOp(":=") ? this.init() : null;
		this.expectOp(";");
		return names.map((n) => ({ name: n.text, key: n.key, cls, constant, retain, at, type, init, span: this.span(n.span) }));
	}

	typeRef(): A.TypeRef {
		const start = this.tok.span;
		if (this.acceptKw("ARRAY")) {
			this.expectOp("[");
			const dims: { lo: A.Expr; hi: A.Expr }[] = [];
			do {
				const lo = this.expr();
				this.expectOp("..");
				dims.push({ lo, hi: this.expr() });
			} while (this.acceptOp(","));
			this.expectOp("]");
			this.expectKw("OF");
			return { kind: "array", dims, of: this.typeRef(), span: this.span(start) };
		}
		if (this.acceptKw("STRING") || this.acceptKw("WSTRING")) {
			let length: A.Expr | null = null;
			if (this.acceptOp("[")) {
				length = this.expr();
				this.expectOp("]");
			} else if (this.isOp("(")) {
				this.next();
				length = this.expr();
				this.expectOp(")");
			}
			return { kind: "string", length, span: this.span(start) };
		}
		const n = this.ident();
		const named: A.TypeRef = { kind: "named", name: n.text, key: n.key, span: n.span };
		// subrange: INT (0..100)
		if (this.isOp("(") && !(this.peek().kind === "ident" && (this.peek(2).text === ":=" || this.peek(2).text === "," || this.peek(2).text === ")"))) {
			this.next();
			const lo = this.expr();
			this.expectOp("..");
			const hi = this.expr();
			this.expectOp(")");
			return { kind: "subrange", base: named, lo, hi, span: this.span(start) };
		}
		return named;
	}

	private init(): A.Init {
		const start = this.tok.span;
		if (this.acceptOp("[")) {
			const items: { repeat: A.Expr | null; value: A.Init | null; span: Span }[] = [];
			if (!this.isOp("]")) {
				do {
					const s = this.tok.span;
					// n(value) repetition
					if ((this.tok.kind === "int") && this.peek().text === "(") {
						const repeat = this.expr0Primary();
						this.expectOp("(");
						const value = this.isOp(")") ? null : this.init();
						this.expectOp(")");
						items.push({ repeat, value, span: this.span(s) });
					} else {
						items.push({ repeat: null, value: this.init(), span: this.span(s) });
					}
				} while (this.acceptOp(","));
			}
			this.expectOp("]");
			return { kind: "array", items, span: this.span(start) };
		}
		if (this.isOp("(") && this.peek().kind === "ident" && this.peek(2).text === ":=") {
			this.next();
			const fields: { name: string; key: string; value: A.Init; span: Span }[] = [];
			do {
				const n = this.ident();
				this.expectOp(":=");
				fields.push({ name: n.text, key: n.key, value: this.init(), span: this.span(n.span) });
			} while (this.acceptOp(","));
			this.expectOp(")");
			return { kind: "struct", fields, span: this.span(start) };
		}
		return { kind: "expr", expr: this.expr() };
	}

	private types(): A.TypeDecl[] {
		this.expectKw("TYPE");
		const out: A.TypeDecl[] = [];
		while (!this.isKw("END_TYPE") && this.tok.kind !== "eof") {
			try {
				out.push(this.typeDecl());
			} catch (e) {
				if (!(e instanceof ParseError)) throw e;
				this.recover();
				if (this.isKw("END_STRUCT")) {
					this.next();
					this.acceptOp(";");
				}
			}
		}
		this.expectKw("END_TYPE");
		return out;
	}

	private typeDecl(): A.TypeDecl {
		const n = this.ident();
		this.expectOp(":");
		const start = n.span;
		if (this.acceptKw("STRUCT")) {
			const fields: A.VarDecl[] = [];
			while (!this.isKw("END_STRUCT") && this.tok.kind !== "eof") fields.push(...this.varDecl("VAR", false, false));
			this.expectKw("END_STRUCT");
			this.acceptOp(";");
			return { kind: "type", name: n.text, key: n.key, def: { kind: "struct", fields }, span: this.span(start) };
		}
		// enumeration: (A, B := 2) or BASE (A := 1, ...)
		let base: A.TypeRef | null = null;
		if (this.tok.kind === "ident" && this.peek().text === "(" && this.peek(2).kind === "ident" &&
			[":=", ",", ")"].includes(this.peek(3).text)) {
			const b = this.next();
			base = { kind: "named", name: b.text, key: b.key, span: b.span };
		}
		if (this.isOp("(") && this.peek().kind === "ident") {
			this.next();
			const values: { name: string; key: string; value: A.Expr | null; span: Span }[] = [];
			do {
				const v = this.ident();
				const value = this.acceptOp(":=") ? this.expr() : null;
				values.push({ name: v.text, key: v.key, value, span: this.span(v.span) });
			} while (this.acceptOp(","));
			this.expectOp(")");
			let init: string | null = null;
			if (this.acceptOp(":=")) init = this.ident().key;
			this.expectOp(";");
			return { kind: "type", name: n.text, key: n.key, def: { kind: "enum", base, values, init }, span: this.span(start) };
		}
		const type = this.typeRef();
		const init = this.acceptOp(":=") ? this.init() : null;
		this.expectOp(";");
		return { kind: "type", name: n.text, key: n.key, def: { kind: "alias", type, init }, span: this.span(start) };
	}

	// ---- statements ----
	private statements(stop: Set<string>): A.Stmt[] {
		const out: A.Stmt[] = [];
		while (this.tok.kind !== "eof" && !(this.tok.kind === "keyword" && (stop.has(this.tok.text) || STMT_END.has(this.tok.text)))) {
			try {
				out.push(this.statement());
			} catch (e) {
				if (!(e instanceof ParseError)) throw e;
				this.recover();
			}
		}
		return out;
	}

	private statement(): A.Stmt {
		const start = this.tok.span;
		if (this.acceptOp(";")) return { kind: "empty", span: start };
		if (this.tok.kind === "keyword") {
			switch (this.tok.text) {
				case "IF": return this.ifStmt();
				case "CASE": return this.caseStmt();
				case "FOR": return this.forStmt();
				case "WHILE": {
					this.next();
					const cond = this.expr();
					this.expectKw("DO");
					const body = this.statements(new Set(["END_WHILE"]));
					this.expectKw("END_WHILE");
					this.acceptOp(";");
					return { kind: "while", cond, body, span: this.span(start) };
				}
				case "REPEAT": {
					this.next();
					const body = this.statements(new Set(["UNTIL"]));
					this.expectKw("UNTIL");
					const cond = this.expr();
					this.expectKw("END_REPEAT");
					this.acceptOp(";");
					return { kind: "repeat", body, cond, span: this.span(start) };
				}
				case "EXIT":
				case "CONTINUE":
				case "RETURN": {
					const k = this.next().text.toLowerCase() as "exit" | "continue" | "return";
					this.expectOp(";");
					return { kind: k, span: this.span(start) };
				}
			}
		}
		const target = this.postfix();
		if (this.acceptOp(":=")) {
			const value = this.expr();
			this.expectOp(";");
			return { kind: "assign", target, value, span: this.span(start) };
		}
		if (target.kind === "call") {
			this.expectOp(";");
			return { kind: "call", call: target, span: this.span(start) };
		}
		this.fail(`expected ':=' or a call, found ${this.describe(this.tok)}`);
	}

	private ifStmt(): A.Stmt {
		const start = this.tok.span;
		this.expectKw("IF");
		const branches: { cond: A.Expr; body: A.Stmt[] }[] = [];
		let cond = this.expr();
		this.expectKw("THEN");
		branches.push({ cond, body: this.statements(new Set()) });
		while (this.acceptKw("ELSIF")) {
			cond = this.expr();
			this.expectKw("THEN");
			branches.push({ cond, body: this.statements(new Set()) });
		}
		const otherwise = this.acceptKw("ELSE") ? this.statements(new Set()) : null;
		this.expectKw("END_IF");
		this.acceptOp(";");
		return { kind: "if", branches, otherwise, span: this.span(start) };
	}

	private caseStmt(): A.Stmt {
		const start = this.tok.span;
		this.expectKw("CASE");
		const selector = this.expr();
		this.expectKw("OF");
		const branches: { labels: A.CaseLabel[]; body: A.Stmt[] }[] = [];
		while (!this.isKw("ELSE") && !this.isKw("END_CASE") && this.tok.kind !== "eof") {
			const labels: A.CaseLabel[] = [];
			do {
				const lo = this.expr();
				if (this.acceptOp("..")) labels.push({ kind: "range", lo, hi: this.expr() });
				else labels.push({ kind: "value", value: lo });
			} while (this.acceptOp(","));
			this.expectOp(":");
			branches.push({ labels, body: this.caseBody() });
		}
		const otherwise = this.acceptKw("ELSE") ? this.statements(new Set(["END_CASE"])) : null;
		this.expectKw("END_CASE");
		this.acceptOp(";");
		return { kind: "case", selector, branches, otherwise, span: this.span(start) };
	}

	/** Statements of a CASE branch: up to the next label ("value :" / "a..b :" / "a, b :"), ELSE or END_CASE. */
	private caseBody(): A.Stmt[] {
		const out: A.Stmt[] = [];
		while (this.tok.kind !== "eof" && !this.isKw("ELSE") && !this.isKw("END_CASE") && !this.atCaseLabel()) {
			try {
				out.push(this.statement());
			} catch (e) {
				if (!(e instanceof ParseError)) throw e;
				this.recover();
			}
		}
		return out;
	}

	private atCaseLabel(): boolean {
		// a label is a constant (literal, enum value, negative number) followed by ':' ',' or '..'
		let k = 0;
		if (this.peek(k).text === "-" || this.peek(k).text === "+") k++;
		const t = this.peek(k);
		if (!["int", "typed", "ident"].includes(t.kind)) return false;
		const after = this.peek(k + 1).text;
		if (t.kind === "ident") return after === ":" || after === "," || after === "..";
		return after === ":" || after === "," || after === "..";
	}

	private forStmt(): A.Stmt {
		const start = this.tok.span;
		this.expectKw("FOR");
		const n = this.ident();
		const variable: A.Expr = { kind: "name", name: n.text, key: n.key, span: n.span };
		this.expectOp(":=");
		const from = this.expr();
		this.expectKw("TO");
		const to = this.expr();
		const by = this.acceptKw("BY") ? this.expr() : null;
		this.expectKw("DO");
		const body = this.statements(new Set(["END_FOR"]));
		this.expectKw("END_FOR");
		this.acceptOp(";");
		return { kind: "for", variable, from, to, by, body, span: this.span(start) };
	}

	// ---- expressions (Ed. 3 table 71: OR < XOR < AND < = <> < relational < + - < * / MOD < ** < unary) ----
	expr(): A.Expr {
		return this.binaryLevel(0);
	}

	private static LEVELS: string[][] = [["OR"], ["XOR"], ["AND", "&"], ["=", "<>"], ["<", ">", "<=", ">="], ["+", "-"], ["*", "/", "MOD"], ["**"]];

	private binaryLevel(level: number): A.Expr {
		if (level === Parser.LEVELS.length) return this.unary();
		let left = this.binaryLevel(level + 1);
		for (;;) {
			const t = this.tok;
			const sym = t.kind === "keyword" || t.kind === "op" ? t.text : "";
			if (!Parser.LEVELS[level].includes(sym)) return left;
			this.next();
			const right = this.binaryLevel(level + 1);
			const op = (sym === "&" ? "AND" : sym) as A.BinaryOp;
			left = { kind: "binary", op, left, right, span: { start: left.span.start, end: right.span.end } };
		}
	}

	private unary(): A.Expr {
		const start = this.tok.span;
		if (this.isOp("-") || this.isOp("+") || this.isKw("NOT")) {
			const op = this.next().text as "-" | "+" | "NOT";
			const operand = this.unary();
			// fold a negative literal so that -128 fits SINT and -2147483648 fits DINT
			if (op === "-" && operand.kind === "int" && operand.type === null) {
				return { ...operand, value: -operand.value, text: "-" + operand.text, span: this.span(start) };
			}
			if (op === "-" && operand.kind === "real" && operand.type === null) {
				return { ...operand, value: -operand.value, text: "-" + operand.text, span: this.span(start) };
			}
			return { kind: "unary", op, operand, span: this.span(start) };
		}
		return this.postfix();
	}

	private postfix(): A.Expr {
		let e = this.primary();
		for (;;) {
			if (this.isOp(".")) {
				this.next();
				const n = this.ident();
				e = { kind: "member", object: e, name: n.text, key: n.key, span: { start: e.span.start, end: n.span.end } };
			} else if (this.isOp("[")) {
				this.next();
				const indices = [this.expr()];
				while (this.acceptOp(",")) indices.push(this.expr());
				const end = this.expectOp("]");
				e = { kind: "index", object: e, indices, span: { start: e.span.start, end: end.span.end } };
			} else if (this.isOp("(") && (e.kind === "name" || e.kind === "member" || e.kind === "index")) {
				e = this.callArgs(e);
			} else return e;
		}
	}

	private callArgs(callee: A.Expr): A.CallExpr {
		this.expectOp("(");
		const args: A.CallArg[] = [];
		if (!this.isOp(")")) {
			do {
				const s = this.tok.span;
				let negated = false;
				if (this.isKw("NOT") && this.peek().kind === "ident" && this.peek(2).text === "=>") {
					this.next();
					negated = true;
				}
				if (this.tok.kind === "ident" && (this.peek().text === ":=" || this.peek().text === "=>")) {
					const n = this.next();
					const output = this.next().text === "=>";
					args.push({ name: n.text, key: n.key, output, negated, value: this.expr(), span: this.span(s) });
				} else {
					args.push({ name: null, key: null, output: false, negated: false, value: this.expr(), span: this.span(s) });
				}
			} while (this.acceptOp(","));
		}
		const end = this.expectOp(")");
		return { kind: "call", callee, args, span: { start: callee.span.start, end: end.span.end } };
	}

	private expr0Primary(): A.Expr {
		return this.primary();
	}

	private primary(): A.Expr {
		const t = this.tok;
		switch (t.kind) {
			case "int": {
				this.next();
				const v = parseIntText(t.text);
				if (v === null) this.fail(`malformed number '${t.text}'`, t.span);
				return { kind: "int", text: t.text, value: v, type: null, span: t.span };
			}
			case "real": {
				this.next();
				return { kind: "real", text: t.text, value: Number(t.text.replace(/_/g, "")), type: null, span: t.span };
			}
			case "time": {
				this.next();
				const body = t.text.slice(t.text.indexOf("#") + 1);
				const ms = parseTime(body);
				if (ms === null) this.fail(`malformed TIME literal '${t.text}'`, t.span);
				if (!Number.isInteger(ms)) this.fail(`'${t.text}': TIME has a resolution of 1 ms`, t.span);
				return { kind: "time", ms, span: t.span };
			}
			case "typed": {
				this.next();
				const hash = t.text.indexOf("#");
				const prefix = t.text.slice(0, hash);
				const up = prefix.toUpperCase();
				const body = t.text.slice(hash + 1);
				if (up === "BOOL") {
					if (body === "1" || body.toUpperCase() === "TRUE") return { kind: "bool", value: true, span: t.span };
					if (body === "0" || body.toUpperCase() === "FALSE") return { kind: "bool", value: false, span: t.span };
					this.fail(`malformed BOOL literal '${t.text}'`, t.span);
				}
				if (up === "REAL" || up === "LREAL") {
					const v = Number(body.replace(/_/g, ""));
					if (Number.isNaN(v)) this.fail(`malformed ${up} literal '${t.text}'`, t.span);
					return { kind: "real", text: body, value: v, type: up, span: t.span };
				}
				if (ELEMENTARY.has(up)) {
					const v = parseIntText(body);
					if (v === null) this.fail(`malformed ${up} literal '${t.text}'`, t.span);
					return { kind: "int", text: body, value: v, type: up, span: t.span };
				}
				return { kind: "enumLit", type: prefix, typeKey: up, name: body, key: body.toUpperCase(), span: t.span };
			}
			case "string": {
				this.next();
				return { kind: "string", value: t.text.slice(1, -1), span: t.span };
			}
			case "ident": {
				this.next();
				return { kind: "name", name: t.text, key: t.key, span: t.span };
			}
			case "keyword":
				if (t.text === "TRUE" || t.text === "FALSE") {
					this.next();
					return { kind: "bool", value: t.text === "TRUE", span: t.span };
				}
				break;
			case "op":
				if (t.text === "(") {
					this.next();
					const e = this.expr();
					this.expectOp(")");
					return e;
				}
				break;
		}
		this.fail(`expected an expression, found ${this.describe(t)}`);
	}
}
