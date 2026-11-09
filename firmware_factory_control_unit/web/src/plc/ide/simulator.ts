// The simulated device: the program runs in the TypeScript VM in the browser, cycle by cycle on a
// timer, with the plant model for its inputs. Same states as the device (docs/plc-iec61131.md 4.2),
// so the IDE works the same against the simulator and (from P2) the FCU.
import type { CompileResult, DebugInfo } from "../compiler/compiler.ts";
import { TRAP_NAMES } from "../generated/isa.ts";
import { Vm, type RunResult } from "../vm.ts";
import { Debugger, type StepKind } from "./debugger.ts";
import { COMMAND_VARS, IO_VARS } from "./io-list.ts";
import { Plant } from "./plant.ts";
import type { HaltInfo, PlcDevice } from "./device.ts";
import { readValue, writeValue, type Value } from "./values.ts";

/** HALTED: a breakpoint or a step stopped the program inside a cycle (docs/plc-iec61131.md 4.2, 10.3). */
export type PlcState = "NO_PROGRAM" | "STOP" | "RUN" | "HALTED" | "ERROR";

export interface PlcError {
	trap: string;
	/** TRAP's operand. */
	code?: number;
	pc: number;
	/** Line in the compiled source of PLC_PRG (null: not found). */
	line: number | null;
	pou: string;
}

export interface LogLine {
	time: Date;
	level: "info" | "warn" | "error";
	text: string;
}

const DATA_CAPACITY = 32 * 1024;

export class Simulator extends EventTarget implements PlcDevice {
	readonly kind = "simulator";
	readonly label = "Simulator";
	readonly canForce = true;
	readonly canDebug = true;
	readonly canSingleCycle = true;
	halt: HaltInfo | null = null;
	private dbg: Debugger | null = null;
	/** Breakpoint lines per source file: they survive a new download (applied to its debug information). */
	private readonly breakpointLines = new Map<string, number[]>();
	readonly hasPlant = true;
	readonly hasControlSource = false;
	connected = true;
	/** The simulator has no fieldbus: the program always switches the outputs. */
	iec = true;
	autostart = false;
	storeProblem = "";
	/** One browser, no lease. */
	readonly leasedByOther = false;
	get forcedOnDevice(): number {
		return this.forces.size;
	}
	attach(): void {
		// the simulator's debug information comes with its download
	}
	activate(): void {}
	deactivate(): void {}
	setSource(): void {}
	setAutostart(): void {}
	readonly data = new Uint8Array(DATA_CAPACITY);
	private vm = new Vm(this.data);
	private image: Uint8Array | null = null;
	debug: DebugInfo | null = null;
	state: PlcState = "NO_PROGRAM";
	error: PlcError | null = null;
	cycleMs = 50;
	cycles = 0;
	lastInstructions = 0;
	maxInstructions = 0;
	plantEnabled = true;
	readonly plant = new Plant();
	readonly forces = new Map<string, Value>();
	readonly log: LogLine[] = [];
	private timer: ReturnType<typeof setInterval> | null = null;
	/** Hash of the project the image was built from ("project = device"). */
	projectHash = 0;

	private emit(): void {
		this.dispatchEvent(new Event("update"));
	}

	private note(level: LogLine["level"], text: string): void {
		this.log.push({ time: new Date(), level, text });
		if (this.log.length > 500) this.log.shift();
	}

	/** Download: STOP, load the image, initial values. */
	download(r: CompileResult): boolean {
		this.stop();
		if (!r.ok || !r.image || !r.debug) return false;
		const e = this.vm.load(r.image);
		if (e !== "none") {
			this.note("error", `download refused: ${e}`);
			this.state = "NO_PROGRAM";
			this.emit();
			return false;
		}
		this.image = r.image;
		this.debug = r.debug;
		this.dbg = new Debugger(r.debug);
		for (const [file, lines] of this.breakpointLines) this.dbg.setBreakpoints(file, lines);
		this.halt = null;
		this.projectHash = new DataView(r.image.buffer, r.image.byteOffset).getUint32(28, true);
		this.state = "STOP";
		this.error = null;
		this.cycles = 0;
		this.maxInstructions = 0;
		this.forces.clear();
		this.plant.reset();
		this.note("info", `program loaded: ${r.image.length} bytes image, ${r.debug.dataSize} bytes data`);
		this.emit();
		return true;
	}

	start(): void {
		if (this.state !== "STOP") return;
		this.state = "RUN";
		this.note("info", "RUN");
		this.timer = setInterval(() => this.cycle(), this.cycleMs);
		this.emit();
	}

	stop(): void {
		if (this.timer !== null) clearInterval(this.timer);
		this.timer = null;
		if (this.state === "RUN" || this.state === "HALTED") {
			this.vm.abandon(0);  // a halted cycle stays unfinished
			this.dbg?.resume();  // no step carries over
			this.halt = null;
			this.state = "STOP";
			this.note("info", "STOP");
		}
		this.emit();
	}

	// ---- debugging ------------------------------------------------------------------------------------
	setBreakpoints(file: string, lines: number[]): number[] {
		this.breakpointLines.set(file, [...lines]);
		const set = this.dbg ? this.dbg.setBreakpoints(file, lines) : lines;
		this.emit();
		return set;
	}

	continueRun(): void {
		if (this.state !== "HALTED" || !this.dbg) return;
		this.dbg.resume();
		this.proceed(true);
	}

	step(kind: StepKind): void {
		if (this.state !== "HALTED" || !this.dbg) return;
		this.dbg.stepFrom(kind, this.vm.contexts[0].calls.length);
		this.proceed(true);
	}

	/** Runs the cycle (or resumes the halted one) with the debugger's stop predicate. */
	private proceed(resume: boolean): void {
		const debug = this.debug!;
		this.vm.stop = this.dbg?.predicate() ?? null;
		const r: RunResult = resume ? this.vm.resume(0) : this.vm.runCycle(0, (this.cycles * this.cycleMs) >>> 0);
		this.lastInstructions = resume ? this.lastInstructions + r.instructions : r.instructions;
		if (r.state === "halted") {
			const c = this.vm.contexts[0];
			const stepping = this.dbg?.stepping ?? false;
			this.halt = { frames: this.dbg!.frames(c.pc, c.calls), reason: stepping ? "step" : "breakpoint" };
			if (this.state !== "HALTED") {
				const top = this.halt.frames[0];
				this.note("info", `HALTED (${this.halt.reason}) in ${top?.pou ?? "?"}${top ? ` line ${top.line}` : ""}`);
			}
			this.state = "HALTED";
			this.emit();
			return;
		}
		this.halt = null;
		this.applyForces();
		this.cycles++;
		this.maxInstructions = Math.max(this.maxInstructions, this.lastInstructions);
		if (r.state === "trapped") {
			const st = [...debug.statements].filter((s) => s.pc <= r.pc).sort((a, b) => b.pc - a.pc)[0];
			this.error = { trap: TRAP_NAMES[r.trap] ?? String(r.trap), pc: r.pc, line: st ? st.span.start.line : null, pou: st?.pou ?? "?" };
			this.stop();
			this.state = "ERROR";
			this.note("error", `runtime error ${this.error.trap} in ${this.error.pou}${this.error.line ? ` line ${this.error.line}` : ""} (pc ${r.pc})`);
		} else {
			this.dbg?.cycleEnded();  // a step past the end halts at the next cycle's first statement
			if (this.state === "HALTED") this.state = this.timer !== null ? "RUN" : "STOP";  // STOP: a single cycle
		}
		this.emit();
	}

	/** Warm and cold reset are the same until RETAIN exists (P7): variables to their initial values. */
	reset(kind: "warm" | "cold"): void {
		if (!this.image) return;
		const wasRunning = this.state === "RUN";
		this.stop();
		this.vm.load(this.image);
		this.state = "STOP";
		this.error = null;
		this.cycles = 0;
		this.plant.reset();
		this.note("info", `${kind} reset`);
		if (wasRunning) this.start();
		this.emit();
	}

	/** One cycle by hand (in STOP). */
	singleCycle(): void {
		if (this.state === "STOP") this.cycle();
	}

	private applyForces(): void {
		if (!this.debug) return;
		for (const [path, v] of this.forces) {
			const s = this.debug.resolve(path);
			if (s) writeValue(this.data, s, v);
		}
	}

	private cycle(): void {
		// ERROR (and NO_PROGRAM) run nothing: the first error and its location stay until a reset
		if (!this.debug || this.state === "ERROR" || this.state === "NO_PROGRAM") return;
		const debug = this.debug;
		if (this.plantEnabled) {
			this.plant.step(this.cycleMs, {
				get: (name) => {
					const s = debug.resolve(name);
					const v = s ? readValue(this.data, s) : null;
					return v === null ? undefined : typeof v === "bigint" ? Number(v) : v;
				},
				set: (name, v) => {
					if (this.forces.has(name.toUpperCase())) return;
					const s = debug.resolve(name);
					const key = name.toUpperCase();
					// the plant writes inputs and the commands' answers, never the program's outputs
					const writable = IO_VARS.some((x) => x.direction === "input" && x.name.toUpperCase() === key) ||
						COMMAND_VARS.some((c) => c.name.toUpperCase() === key);
					if (s && writable) writeValue(this.data, s, v);
				},
			});
		}
		// HALTED: the plant (time) runs on, the program waits for CONTINUE or a step - it never goes on by itself
		if (this.state === "HALTED") {
			this.emit();
			return;
		}
		this.applyForces();
		this.proceed(false);
	}

	read(path: string): { value: Value | null; symbol: ReturnType<DebugInfo["resolve"]> } {
		const s = this.debug?.resolve(path) ?? null;
		return { value: s ? readValue(this.data, s) : null, symbol: s };
	}

	write(path: string, v: Value): boolean {
		const s = this.debug?.resolve(path);
		if (!s) return false;
		writeValue(this.data, s, v);
		this.note("info", `write ${path} := ${String(v)}`);
		this.emit();
		return true;
	}

	force(path: string, v: Value | null): void {
		const key = path.toUpperCase();
		if (v === null) {
			if (this.forces.delete(key)) this.note("info", `unforce ${path}`);
		} else {
			this.forces.set(key, v);
			this.note("warn", `force ${path} := ${String(v)}`);
			this.applyForces();
		}
		this.emit();
	}

	unforceAll(): void {
		if (this.forces.size > 0) this.note("info", `all ${this.forces.size} forces released`);
		this.forces.clear();
		this.emit();
	}
}
