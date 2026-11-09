// SPDX-License-Identifier: MIT
// The syntax tree of IEC 61131-3 Structured Text (and, from P5/P6, the target of the FBD and SFC
// lowering). Every node has a span for diagnostics and the debugger. DOM-free.
import type { Span } from "./diagnostics.ts";

// ---- types as written ---------------------------------------------------------------------------
export type TypeRef =
	| { kind: "named"; name: string; key: string; span: Span }
	| { kind: "array"; dims: { lo: Expr; hi: Expr }[]; of: TypeRef; span: Span }
	| { kind: "string"; length: Expr | null; span: Span }
	| { kind: "subrange"; base: TypeRef; lo: Expr; hi: Expr; span: Span };

// ---- initializers -------------------------------------------------------------------------------
export type Init =
	| { kind: "expr"; expr: Expr }
	| { kind: "array"; items: { repeat: Expr | null; value: Init | null; span: Span }[]; span: Span }
	| { kind: "struct"; fields: { name: string; key: string; value: Init; span: Span }[]; span: Span };

// ---- declarations -------------------------------------------------------------------------------
export type VarClass = "VAR" | "VAR_INPUT" | "VAR_OUTPUT" | "VAR_IN_OUT" | "VAR_TEMP" | "VAR_GLOBAL" | "VAR_EXTERNAL";

export interface VarDecl {
	name: string;
	key: string;
	cls: VarClass;
	constant: boolean;
	retain: boolean;
	at: string | null; // %IX0.0 ...
	type: TypeRef;
	init: Init | null;
	span: Span;
}

export interface TypeDecl {
	kind: "type";
	name: string;
	key: string;
	def:
		| { kind: "alias"; type: TypeRef; init: Init | null }
		| { kind: "struct"; fields: VarDecl[] }
		| { kind: "enum"; base: TypeRef | null; values: { name: string; key: string; value: Expr | null; span: Span }[]; init: string | null };
	span: Span;
	/** The enclosing namespace (upper case, dotted; "" or absent: global) and the namespaces made
	 *  visible by USING where the unit is (IEC 61131-3 Ed. 3, 6.9). */
	ns?: string;
	using?: string[];
}

export type PouKind = "PROGRAM" | "FUNCTION_BLOCK" | "FUNCTION";

export interface Pou {
	kind: "pou";
	pouKind: PouKind;
	name: string;
	key: string;
	returnType: TypeRef | null; // FUNCTION
	vars: VarDecl[];
	body: Stmt[];
	span: Span;
	/** The source unit (file) the POU comes from. */
	file: string;
	/** The enclosing namespace (upper case, dotted; "" or absent: global) and the namespaces made
	 *  visible by USING where the unit is (IEC 61131-3 Ed. 3, 6.9). */
	ns?: string;
	using?: string[];
}

export interface GlobalVars {
	kind: "globals";
	vars: VarDecl[];
	span: Span;
	/** The enclosing namespace (upper case, dotted; "" or absent: global) and the namespaces made
	 *  visible by USING where the unit is (IEC 61131-3 Ed. 3, 6.9). */
	ns?: string;
	using?: string[];
}

export type Unit = TypeDecl | Pou | GlobalVars;

// ---- statements ---------------------------------------------------------------------------------
export type Stmt =
	| { kind: "assign"; target: Expr; value: Expr; span: Span }
	| { kind: "call"; call: CallExpr; span: Span }
	| { kind: "if"; branches: { cond: Expr; body: Stmt[] }[]; otherwise: Stmt[] | null; span: Span }
	| { kind: "case"; selector: Expr; branches: { labels: CaseLabel[]; body: Stmt[] }[]; otherwise: Stmt[] | null; span: Span }
	| { kind: "for"; variable: Expr; from: Expr; to: Expr; by: Expr | null; body: Stmt[]; span: Span }
	| { kind: "while"; cond: Expr; body: Stmt[]; span: Span }
	| { kind: "repeat"; body: Stmt[]; cond: Expr; span: Span }
	| { kind: "exit"; span: Span }
	| { kind: "continue"; span: Span }
	| { kind: "return"; span: Span }
	| { kind: "empty"; span: Span };

export type CaseLabel = { kind: "value"; value: Expr } | { kind: "range"; lo: Expr; hi: Expr };

// ---- expressions --------------------------------------------------------------------------------
export interface CallArg {
	/** Formal: name := value or name => target; null: positional. */
	name: string | null;
	key: string | null;
	output: boolean; // =>
	negated: boolean; // NOT name => (output negation, rarely used)
	value: Expr;
	span: Span;
}

export interface CallExpr {
	kind: "call";
	callee: Expr; // a name (function, FB instance) or member access (instance in a structure)
	args: CallArg[];
	span: Span;
}

export type Expr =
	| { kind: "int"; text: string; value: bigint; type: string | null; span: Span }
	| { kind: "real"; text: string; value: number; type: string | null; span: Span }
	| { kind: "bool"; value: boolean; span: Span }
	| { kind: "time"; ms: number; span: Span }
	| { kind: "string"; value: string; span: Span }
	| { kind: "enumLit"; type: string; typeKey: string; name: string; key: string; span: Span }
	| { kind: "name"; name: string; key: string; span: Span }
	| { kind: "member"; object: Expr; name: string; key: string; span: Span }
	| { kind: "index"; object: Expr; indices: Expr[]; span: Span }
	| { kind: "unary"; op: "-" | "+" | "NOT"; operand: Expr; span: Span }
	| { kind: "binary"; op: BinaryOp; left: Expr; right: Expr; span: Span }
	| CallExpr;

export type BinaryOp = "**" | "*" | "/" | "MOD" | "+" | "-" | "<" | ">" | "<=" | ">=" | "=" | "<>" | "AND" | "XOR" | "OR";

export const spanOf = (a: { span: Span }, b: { span: Span }): Span => ({ start: a.span.start, end: b.span.end });
