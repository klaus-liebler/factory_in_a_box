// Breakpoints and single steps over the debug information of a program (docs/plc-iec61131.md 10.3),
// DOM-free: line breakpoints become the code offsets of the statements that begin on those lines,
// a step becomes a stop predicate for the VM (rtos::plc / vm.ts `stop`), a halt becomes a frame chain.
// The simulator uses it directly; the FCU gets the same offsets (P3, device side).
import type { DebugInfo, StatementInfo } from "../compiler/compiler.ts";
import type { StopPredicate } from "../vm.ts";

export type StepKind = "into" | "over" | "out";

/** A place in the source: the file is the POU's source (e.g. "PLC_PRG", "<stdlib>", "<fcu>"). */
export interface SourcePlace {
	file: string;
	line: number;
	pou: string;
}

/** One entry of the call stack, the innermost first. */
export interface Frame extends SourcePlace {
	pc: number;
}

export class Debugger {
	private readonly statements: StatementInfo[];  // by pc
	private readonly starts = new Set<number>();
	private readonly breakpoints = new Set<number>();  // pcs
	private step: { kind: StepKind; depth: number } | null = null;

	constructor(readonly debug: DebugInfo) {
		this.statements = [...debug.statements].sort((a, b) => a.pc - b.pc);
		for (const s of this.statements) this.starts.add(s.pc);
	}

	/** Lines of `file` a breakpoint can go on (a statement begins there). */
	breakableLines(file: string): Set<number> {
		return new Set(this.statements.filter((s) => s.span.start.file === file).map((s) => s.span.start.line));
	}

	/** The breakpoints of `file` (replacing its earlier ones); returns the lines that got one. */
	setBreakpoints(file: string, lines: Iterable<number>): number[] {
		const inFile = this.statements.filter((s) => s.span.start.file === file);
		for (const s of inFile) this.breakpoints.delete(s.pc);
		const wanted = new Set(lines);
		const set = new Set<number>();
		for (const s of inFile) {
			if (wanted.has(s.span.start.line)) {
				this.breakpoints.add(s.pc);
				set.add(s.span.start.line);
			}
		}
		return [...set].sort((a, b) => a - b);
	}

	/** The code offsets of every breakpoint (the device gets these). */
	breakpointOffsets(): number[] {
		return [...this.breakpoints].sort((a, b) => a - b);
	}

	/** One step from a halt at call depth `depth`: into - the next statement anywhere, over - the next one
	 *  at this depth or above (calls run through), out - the next one in a caller. */
	stepFrom(kind: StepKind, depth: number): void {
		this.step = { kind, depth };
	}

	/** Continue: only breakpoints stop. */
	resume(): void {
		this.step = null;
	}

	/** A cycle ended while a step was under way: it stops at the first statement of the next cycle
	 *  (stepping over or out of the end of the program). */
	cycleEnded(): void {
		if (this.step) this.step = { kind: "into", depth: 0 };
	}

	get stepping(): boolean {
		return this.step !== null;
	}

	/** The VM's predicate; null when nothing can stop (no breakpoint, no step): the VM runs at full speed. */
	predicate(): StopPredicate | null {
		const step = this.step;
		if (this.breakpoints.size === 0 && step === null) return null;
		const bps = this.breakpoints;
		const starts = this.starts;
		return (pc, depth) => {
			if (bps.has(pc)) return true;
			if (step === null || !starts.has(pc)) return false;
			switch (step.kind) {
				case "into": return true;
				case "over": return depth <= step.depth;
				case "out": return depth < step.depth;
			}
		};
	}

	/** The statement that contains `pc` (the last one starting at or before it). */
	placeOf(pc: number): SourcePlace | null {
		let lo = 0, hi = this.statements.length - 1, found = -1;
		while (lo <= hi) {
			const mid = (lo + hi) >> 1;
			if (this.statements[mid].pc <= pc) {
				found = mid;
				lo = mid + 1;
			} else {
				hi = mid - 1;
			}
		}
		if (found < 0) return null;
		const s = this.statements[found];
		return { file: s.span.start.file, line: s.span.start.line, pou: s.pou };
	}

	/** The call stack of a halt: the halted statement, then each caller at its call (`calls`: the VM
	 *  context's chain, the program's base entry first). */
	frames(pc: number, calls: readonly { returnPc: number }[]): Frame[] {
		const out: Frame[] = [];
		const here = this.placeOf(pc);
		if (here) out.push({ ...here, pc });
		for (let i = calls.length - 1; i >= 1; i--) {
			const at = calls[i].returnPc - 1;  // inside the CALL instruction
			const p = this.placeOf(at);
			if (p) out.push({ ...p, pc: at });
		}
		return out;
	}
}
