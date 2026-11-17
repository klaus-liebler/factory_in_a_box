// The devices the IDE works with: the simulator in the browser and - registered by the app, devices.ts - the
// FCU over the WebSocket (web/src/plc-fcu/fcu-device.ts). The IDE uses only this interface, so both
// behave the same for it; what one of them cannot do yet (forcing on the FCU: P3) it says through
// its capabilities.
import type { CompileResult, DebugInfo } from "../compiler/compiler.ts";
import type { Symbol } from "../compiler/compiler.ts";
import type { LogLine, PlcError, PlcState } from "./simulator.ts";
import type { Value } from "./values.ts";
import type { Frame, StepKind } from "./debugger.ts";

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
	/** A variable's value (the FCU: its bytes are watched from now on). */
	value(s: Symbol): Value | null;
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
