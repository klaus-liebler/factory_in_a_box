// The devices the IDE works with: the simulator in the browser and the FCU over the WebSocket
// (namespace "plc", schema/best_binary_buffers/plc.cs). The IDE uses only this interface, so both
// behave the same for it; what one of them cannot do yet (forcing on the FCU: P3) it says through
// its capabilities.
import type { CompileResult, DebugInfo } from "../compiler/compiler.ts";
import { TRAP_NAMES } from "../generated/isa.ts";
import { Hdr, crc32 } from "../image.ts";
import { plc } from "../../../generated/ws-protocol.ts";
import { wsRequest } from "../../ws-client.ts";
import type { LogLine, PlcError, PlcState } from "./simulator.ts";
import { sizeOf } from "../compiler/types.ts";
import { readValue, writeValue, type Value } from "./values.ts";
import { Debugger, type Frame, type StepKind } from "./debugger.ts";

export interface HaltInfo {
	frames: Frame[];
	reason: "breakpoint" | "step";
}

export interface PlcDevice extends EventTarget {
	readonly kind: "simulator" | "fcu";
	readonly label: string;
	readonly canForce: boolean;
	readonly canSingleCycle: boolean;
	readonly hasPlant: boolean;
	/** The FCU: control source and autostart are device settings. */
	readonly hasControlSource: boolean;
	state: PlcState;
	error: PlcError | null;
	/** Debug information of the program on the device (null: not known in this browser). */
	debug: DebugInfo | null;
	cycleMs: number;
	cycles: number;
	lastInstructions: number;
	maxInstructions: number;
	readonly data: Uint8Array;
	readonly forces: Map<string, Value>;
	readonly log: LogLine[];
	/** Project hash of the image on the device (0: none). */
	projectHash: number;
	connected: boolean;
	iec: boolean;
	autostart: boolean;
	/** Why downloads are refused ("": they are not). */
	storeProblem: string;
	/** Another browser holds the debug lease (it forces): this one may watch, not change. */
	leasedByOther: boolean;
	/** Forced values on the device, any browser's (the simulator: its own). */
	forcedOnDevice: number;

	download(r: CompileResult): boolean | Promise<boolean>;
	start(): void | Promise<void>;
	stop(): void | Promise<void>;
	reset(kind: "warm" | "cold"): void | Promise<void>;
	singleCycle(): void;
	read(path: string): { value: Value | null; symbol: ReturnType<DebugInfo["resolve"]> };
	write(path: string, v: Value): boolean | Promise<boolean>;
	force(path: string, v: Value | null): void;
	unforceAll(): void;
	setSource(iec: boolean): void | Promise<void>;
	setAutostart(on: boolean): void | Promise<void>;
	/** The IDE's current build: its debug information is used when the device runs that program. */
	attach(build: CompileResult | null): void;
	/** The PLC page is visible (poll) or not. */
	activate(): void;
	deactivate(): void;
	/** Breakpoints and single steps (docs/plc-iec61131.md 10.3); false: not on this device yet. */
	readonly canDebug: boolean;
	/** Where the program halted (state HALTED), the innermost frame first. */
	halt: HaltInfo | null;
	/** The breakpoints of a source file (replacing its earlier ones); returns the lines that got one. */
	setBreakpoints(file: string, lines: number[]): number[];
	/** HALTED: run on (only breakpoints stop). */
	continueRun(): void | Promise<void>;
	/** HALTED: one statement (into calls, over them, out of the current POU). */
	step(kind: StepKind): void | Promise<void>;
	/** The project source on the device (the FCU: its SD card, docs/plc-iec61131.md 4.5); absent: the
	 *  device keeps none. */
	storeProject?(json: string): Promise<boolean>;
	/** The stored project source; null: none (the log says why). */
	loadProject?(): Promise<string | null>;
}

const STATE: Record<number, PlcState> = { 0: "NO_PROGRAM", 1: "STOP", 2: "RUN", 3: "ERROR", 4: "HALTED" };
const STEP_COMMAND: Record<StepKind, plc.PlcCommand> = {
	into: plc.PlcCommand.StepInto, over: plc.PlcCommand.StepOver, out: plc.PlcCommand.StepOut,
};
const CHUNK = 1024;
const POLL_MS = 400;
/** The flash commit can take seconds (an STM32H743 sector erase). */
const COMMIT_TIMEOUT_MS = 30_000;

/** The FCU as a device: status and variables by polling, commands as requests. */
export class FcuDevice extends EventTarget implements PlcDevice {
	readonly kind = "fcu";
	readonly label = "FCU";
	readonly canForce = true;
	readonly canDebug = true;
	halt: HaltInfo | null = null;
	private dbg: Debugger | null = null;
	private dbgFor: DebugInfo | null = null;
	private readonly breakpointLines = new Map<string, number[]>();

	/** The debugger for the program on the device (its debug information is known in this browser). */
	private debugger(): Debugger | null {
		if (!this.debug) return null;
		if (this.dbgFor !== this.debug) {
			this.dbg = new Debugger(this.debug);
			for (const [file, lines] of this.breakpointLines) this.dbg.setBreakpoints(file, lines);
			this.dbgFor = this.debug;
		}
		return this.dbg;
	}

	/** Breakpoints: the lines become the statement offsets of the program on the device. */
	setBreakpoints(file: string, lines: number[]): number[] {
		this.breakpointLines.set(file, [...lines]);
		const d = this.debugger();
		if (!d) {
			if (lines.length) this.note("warn", "breakpoints: the program on the device is not known in this browser - load it from here first");
			return lines;
		}
		const set = d.setBreakpoints(file, lines);
		const offsets = d.breakpointOffsets();
		void (async () => {
			try {
				const r = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.BreakpointsRequest.encode({ requestId, offsets }), plc.BreakpointsResponse);
				if (!r.ok) this.note("error", `breakpoints: ${r.message}`);
			} catch (e) {
				this.note("error", `breakpoints: ${(e as Error).message}`);
			}
		})();
		return set;
	}

	async continueRun(): Promise<void> {
		await this.control(plc.PlcCommand.Continue, "continue");
		void this.poll();
	}

	async step(kind: StepKind): Promise<void> {
		await this.control(STEP_COMMAND[kind], `step ${kind}`);
		void this.poll();
	}

	/** HALTED: where (the program counter and the call chain) -> the call stack. */
	private async readHalt(): Promise<void> {
		const r = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.HaltRequest.encode({ requestId }), plc.HaltResponse);
		const d = this.debugger();
		if (!r.halted || !d) {
			this.halt = null;
			return;
		}
		this.halt = { frames: d.frames(r.pc, r.returnPcs.map((returnPc) => ({ returnPc }))), reason: r.reason === 2 ? "step" : "breakpoint" };
	}
	readonly canSingleCycle = false;
	readonly hasPlant = false;
	readonly hasControlSource = true;
	state: PlcState = "NO_PROGRAM";
	error: PlcError | null = null;
	debug: DebugInfo | null = null;
	cycleMs = 50;
	cycles = 0;
	lastInstructions = 0;
	maxInstructions = 0;
	data = new Uint8Array(32 * 1024);
	readonly forces = new Map<string, Value>();
	readonly log: LogLine[] = [];
	projectHash = 0;
	connected = false;
	iec = false;
	autostart = false;
	storeProblem = "";
	leasedByOther = false;
	forcedOnDevice = 0;
	private imageBytes = 0;
	private builds: { hash: number; bytes: number; debug: DebugInfo }[] = [];
	private timer: ReturnType<typeof setTimeout> | null = null;
	private busy = false;

	private emit(): void {
		this.dispatchEvent(new Event("update"));
	}

	private note(level: LogLine["level"], text: string): void {
		this.log.push({ time: new Date(), level, text });
		if (this.log.length > 500) this.log.shift();
		this.emit();
	}

	attach(build: CompileResult | null): void {
		if (build?.ok && build.image && build.debug) this.remember(build);
		this.matchDebug();
	}

	private remember(r: CompileResult): void {
		const hash = new DataView(r.image!.buffer, r.image!.byteOffset).getUint32(Hdr.projectHash, true);
		this.builds = [{ hash, bytes: r.image!.length, debug: r.debug! }, ...this.builds.filter((b) => b.hash !== hash)].slice(0, 8);
	}

	private matchDebug(): void {
		const b = this.builds.find((x) => x.hash === this.projectHash && x.bytes === this.imageBytes);
		this.debug = this.projectHash !== 0 && b ? b.debug : null;
	}

	activate(): void {
		if (this.timer === null) void this.poll();
	}

	deactivate(): void {
		if (this.timer !== null) clearTimeout(this.timer);
		this.timer = null;
	}

	private async poll(): Promise<void> {
		this.timer = setTimeout(() => void this.poll(), POLL_MS);
		if (this.busy) return;
		try {
			const s = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.StatusRequest.encode({ requestId }), plc.StatusResponse);
			this.connected = true;
			this.state = STATE[s.state] ?? "NO_PROGRAM";
			this.iec = s.source === plc.ControlSource.Iec;
			this.autostart = s.autostart;
			this.cycleMs = s.cycleMs;
			this.cycles = s.cycles;
			this.lastInstructions = s.lastInstructions;
			this.maxInstructions = s.maxInstructions;
			this.projectHash = s.projectHash;
			this.imageBytes = s.imageBytes;
			this.storeProblem = s.storeProblem;
			this.leasedByOther = s.leasedByOther;
			this.forcedOnDevice = s.forces;
			if (s.forces === 0 && this.forces.size > 0) {
				// released on the device: STOP, a switch, a download, another browser or 'plc unforce'
				this.forces.clear();
				this.note("info", "the device released every forced value");
			}
			this.matchDebug();
			this.error = this.state === "ERROR" ? this.errorAt(s.trap, s.trapCode, s.trapPc) : null;
			if (this.state === "HALTED") await this.readHalt();
			else this.halt = null;
			if (this.debug && (this.state === "RUN" || this.state === "STOP" || this.state === "ERROR" || this.state === "HALTED")) await this.readData();
		} catch {
			if (this.connected) this.note("warn", "the FCU does not answer");
			this.connected = false;
		}
		this.emit();
	}

	private errorAt(trap: number, code: number, pc: number): PlcError {
		const st = this.debug ? [...this.debug.statements].filter((s) => s.pc <= pc).sort((a, b) => b.pc - a.pc)[0] : undefined;
		return { trap: TRAP_NAMES[trap] ?? String(trap), pc, line: st ? st.span.start.line : null, pou: st?.pou ?? "?", code } as PlcError;
	}

	/** The variables (not the frame areas at the end): up to 4 KB per poll. */
	private async readData(): Promise<void> {
		const end = Math.min(this.debug!.dataSize, 4096);
		for (let at = 0; at < end; at += CHUNK) {
			const r = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.ReadRequest.encode({ requestId, address: at, length: Math.min(CHUNK, end - at) }), plc.ReadResponse);
			if (!r.ok) return;
			this.data.set(r.data, r.address);
		}
	}

	private async control(command: plc.PlcCommand, what: string): Promise<boolean> {
		try {
			const r = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.ControlRequest.encode({ requestId, command }), plc.ControlResponse);
			if (!r.ok) this.note("error", `${what}: ${r.message}`);
			else this.note("info", what);
			return r.ok;
		} catch (e) {
			this.note("error", `${what}: ${(e as Error).message}`);
			return false;
		}
	}

	async download(r: CompileResult): Promise<boolean> {
		if (!r.ok || !r.image || !r.debug) return false;
		this.busy = true;
		try {
			if (this.state === "RUN" && !(await this.control(plc.PlcCommand.Stop, "STOP"))) return false;
			const img = r.image;
			this.note("info", `download: ${img.length} bytes`);
			for (let offset = 0; offset < img.length; offset += CHUNK) {
				const data = Array.from(img.subarray(offset, offset + CHUNK));
				const u = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.UploadRequest.encode({ requestId, totalBytes: img.length, offset, data }), plc.UploadResponse);
				if (!u.ok) {
					this.note("error", `download refused: ${u.message}`);
					return false;
				}
			}
			const c = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.CommitRequest.encode({ requestId, totalBytes: img.length, crc: crc32(img) }), plc.CommitResponse, COMMIT_TIMEOUT_MS);
			if (!c.ok) {
				this.note("error", `download refused: ${c.message}`);
				return false;
			}
			this.remember(r);
			this.debug = r.debug;  // the device runs exactly this build now (breakpoints are sent right after)
			this.note("info", "program stored in the flash and loaded (STOP)");
			return true;
		} catch (e) {
			this.note("error", `download: ${(e as Error).message}`);
			return false;
		} finally {
			this.busy = false;
			void this.poll();
		}
	}

	async start(): Promise<void> {
		await this.control(plc.PlcCommand.Run, "RUN");
		void this.poll();
	}
	async stop(): Promise<void> {
		await this.control(plc.PlcCommand.Stop, "STOP");
		void this.poll();
	}
	async reset(kind: "warm" | "cold"): Promise<void> {
		await this.control(kind === "warm" ? plc.PlcCommand.ResetWarm : plc.PlcCommand.ResetCold, `${kind} reset`);
		void this.poll();
	}
	singleCycle(): void {
		// the FCU runs cycles with the I/O cycle only
	}
	async setSource(iec: boolean): Promise<void> {
		await this.control(iec ? plc.PlcCommand.SourceIec : plc.PlcCommand.SourceFieldbus, iec ? "control source IEC 61131" : "control source fieldbus");
		void this.poll();
	}
	async setAutostart(on: boolean): Promise<void> {
		await this.control(on ? plc.PlcCommand.AutostartOn : plc.PlcCommand.AutostartOff, on ? "autostart on" : "autostart off");
		void this.poll();
	}

	read(path: string): { value: Value | null; symbol: ReturnType<DebugInfo["resolve"]> } {
		const s = this.debug?.resolve(path) ?? null;
		return { value: s ? readValue(this.data, s) : null, symbol: s };
	}

	async write(path: string, v: Value): Promise<boolean> {
		const s = this.debug?.resolve(path);
		if (!s) return false;
		// the value's bytes as they are in the data segment
		const bytes = new Uint8Array(sizeOf(s.type));
		writeValue(bytes, { addr: 0, type: s.type }, v);
		try {
			const r = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.WriteRequest.encode({ requestId, address: s.addr, data: Array.from(bytes) }), plc.WriteResponse);
			this.note(r.ok ? "info" : "error", `write ${path} := ${String(v)}${r.ok ? "" : " refused"}`);
			return r.ok;
		} catch (e) {
			this.note("error", `write ${path}: ${(e as Error).message}`);
			return false;
		}
	}

	/** The project source onto the SD card, in chunks; the device replaces the stored one only when all arrived. */
	async storeProject(json: string): Promise<boolean> {
		const bytes = new TextEncoder().encode(json);
		try {
			for (let offset = 0; offset < bytes.length; offset += CHUNK) {
				const data = Array.from(bytes.subarray(offset, offset + CHUNK));
				const r = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.PutProjectRequest.encode({ requestId, totalBytes: bytes.length, offset, data }), plc.PutProjectResponse);
				if (!r.ok) {
					this.note("warn", `project source not stored on the device: ${r.message}`);
					return false;
				}
			}
			this.note("info", `project source stored on the device's SD card (${bytes.length} bytes)`);
			return true;
		} catch (e) {
			this.note("warn", `project source not stored on the device: ${(e as Error).message}`);
			return false;
		}
	}

	async loadProject(): Promise<string | null> {
		try {
			const parts: number[] = [];
			let total = 1;
			while (parts.length < total) {
				const r = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.GetProjectRequest.encode({ requestId, offset: parts.length }), plc.GetProjectResponse);
				if (!r.ok) {
					this.note("warn", `no project from the device: ${r.message}`);
					return null;
				}
				total = r.totalBytes;
				if (r.offset !== parts.length || (r.data.length === 0 && parts.length < total)) {
					this.note("error", "project from the device: chunks out of order");
					return null;
				}
				parts.push(...r.data);
			}
			this.note("info", `project source read from the device (${total} bytes)`);
			return new TextDecoder().decode(new Uint8Array(parts));
		} catch (e) {
			this.note("error", `project from the device: ${(e as Error).message}`);
			return null;
		}
	}

	/** Force (v) or release (null) a variable on the device: its bytes after the inputs and after the
	 *  program, every cycle (docs/plc-iec61131.md 10.3). Forcing takes the debug lease. */
	force(path: string, v: Value | null): void {
		void this.forceAsync(path, v);
	}

	private async forceAsync(path: string, v: Value | null): Promise<void> {
		const s = this.debug?.resolve(path);
		if (!s) {
			this.note("error", `force ${path}: not a variable of the program on the device`);
			return;
		}
		const key = path.toUpperCase();
		try {
			if (v === null) {
				const r = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.UnforceRequest.encode({ requestId, address: s.addr }), plc.UnforceResponse);
				if (!r.ok) return this.note("error", `unforce ${path}: ${r.message}`);
				this.forces.delete(key);
				this.note("info", `unforce ${path}`);
			} else {
				const bytes = new Uint8Array(sizeOf(s.type));
				writeValue(bytes, { addr: 0, type: s.type }, v);
				if (bytes.length > 8) return this.note("error", `force ${path}: only scalar variables (at most 8 bytes)`);
				const r = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.ForceRequest.encode({ requestId, address: s.addr, data: Array.from(bytes) }), plc.ForceResponse);
				if (!r.ok) return this.note("error", `force ${path}: ${r.message}`);
				this.forces.set(key, v);
				this.note("warn", `force ${path} := ${String(v)}`);
			}
		} catch (e) {
			this.note("error", `force ${path}: ${(e as Error).message}`);
		}
		void this.poll();
	}

	unforceAll(): void {
		void (async () => {
			try {
				const r = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.UnforceRequest.encode({ requestId, address: 0xffffffff }), plc.UnforceResponse);
				if (!r.ok) return this.note("error", `release the forces: ${r.message}`);
				this.forces.clear();
				this.note("info", "every forced value released");
			} catch (e) {
				this.note("error", `release the forces: ${(e as Error).message}`);
			}
			void this.poll();
		})();
	}
}
