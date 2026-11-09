// The I/O variables of the IEC program, derived from the register map (generated/register-map.ts):
// global variables that the image binds to registers (docs/plc-iec61131.md section 6.1). The device
// copies the input registers into them before each cycle and the outputs into the holding registers
// after it; the simulator's plant model reads and writes them directly.
//
// Command registers (handshakes the firmware answers in the same register, docs/plc-iec61131.md 6.3)
// are not I/O variables: a cyclic output would overwrite the answers. They come with the FCU library
// blocks (FCU_StepperHome, FCU_RoArmRunMission). Until the register schema carries the class
// (Kind = Output | Command | Parameter), the list below names them.
import { REGIONS, type RegisterDef } from "../../../generated/register-map.ts";
import type { IoBindingDef } from "../compiler/compiler.ts";

const COMMAND_REGISTERS = new Set(["STEPPER1_START_HOMING", "STEPPER2_START_HOMING", "RO_ARM_MISSION_CONTROL"]);

export type IoDirection = "input" | "output";

export interface IoVar {
	/** IEC name, e.g. ConveyorPwm. */
	name: string;
	type: "BOOL" | "INT" | "UINT" | "DINT" | "UDINT";
	direction: IoDirection;
	bits: 1 | 16 | 32;
	signed: boolean;
	/** %IW180, %QW40, %QD61 */
	address: string;
	register: number;
	region: string;
	description: string;
	unit?: string;
	min?: number;
	max?: number;
}

/** VALVE1 -> Valve1, CONVEYOR_PWM -> ConveyorPwm. */
export function iecName(registerName: string): string {
	return registerName.toLowerCase().split("_").map((p) => p.charAt(0).toUpperCase() + p.slice(1)).join("");
}

function ioVar(def: RegisterDef, region: string): IoVar | null {
	const count = def.combine?.count ?? 1;
	if (count > 2) return null; // chip UID and the like: not a process value
	if (COMMAND_REGISTERS.has(def.name)) return null;
	const type: IoVar["type"] = def.display === "bool" ? "BOOL" : count === 2 ? (def.signed ? "DINT" : "UDINT") : def.signed ? "INT" : "UINT";
	const direction: IoDirection = def.bank === "input" ? "input" : "output";
	const prefix = direction === "input" ? "%I" : "%Q";
	const size = type === "BOOL" ? "X" : count === 2 ? "D" : "W";
	return {
		name: iecName(def.name), type, direction, bits: type === "BOOL" ? 1 : count === 2 ? 32 : 16, signed: type === "INT" || type === "DINT",
		address: `${prefix}${size}${def.address}`, register: def.address, region,
		description: def.description ?? "", unit: def.unit, min: def.min, max: def.max,
	};
}

export const IO_VARS: readonly IoVar[] = REGIONS.flatMap((r) => r.registers.map((d) => ioVar(d, r.title)).filter((v): v is IoVar => v !== null))
	.sort((a, b) => (a.direction === b.direction ? a.register - b.register : a.direction === "input" ? -1 : 1));

export const IO_FILE = "I/O";

/** The I/O variables as a global variable list (read-only in the IDE). */
export function ioSource(): string {
	const lines = ["(* generated from the register map - read-only *)", "VAR_GLOBAL"];
	let region = "";
	for (const v of IO_VARS) {
		if (v.region !== region) {
			region = v.region;
			lines.push(`    (* ---- ${region} *)`);
		}
		const comment = [v.address, v.description, v.unit ? `[${v.unit}]` : ""].filter(Boolean).join("  ").replace(/\*\)/g, "* )");
		lines.push(`    ${v.name} : ${v.type}; (* ${comment} *)`);
	}
	lines.push("END_VAR");
	return lines.join("\n") + "\n";
}

/** The bindings for the compiler: every I/O variable to its register. */
export function ioBindings(): IoBindingDef[] {
	return IO_VARS.map((v) => ({
		name: v.name, register: v.register, area: v.direction === "input" ? "input" : "holding", kind: v.direction,
		bits: v.bits, signed: v.signed,
	}));
}

/** CRC-32 of the I/O list: the image carries it (ioMapHash), so a changed register map is noticed. */
export function ioMapHash(): number {
	const text = IO_VARS.map((v) => `${v.name}:${v.direction}:${v.register}:${v.bits}:${v.signed ? "s" : "u"}`).join(";");
	let crc = 0xffffffff;
	for (const b of new TextEncoder().encode(text)) {
		crc ^= b;
		for (let k = 0; k < 8; k++) crc = crc & 1 ? 0xedb88320 ^ (crc >>> 1) : crc >>> 1;
	}
	return ~crc >>> 0;
}
