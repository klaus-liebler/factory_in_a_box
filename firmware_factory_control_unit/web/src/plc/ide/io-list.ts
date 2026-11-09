// The I/O variables of the IEC program, derived from the register map (generated/register-map.ts):
// global variables that the image binds to registers (docs/plc-iec61131.md section 6.1). The device
// copies the input registers into them before each cycle and the outputs into the holding registers
// after it; the simulator's plant model reads and writes them directly.
//
// The register classes come from the schema (Kind, docs/plc-iec61131.md 6.3): outputs are %Q variables,
// parameters read-only %I variables; command registers (handshakes the firmware answers in the same
// register) are no I/O variables - a cyclic output would overwrite the answers. They are bound to
// reserved globals (__Stepper1StartHoming, ...) that only the FCU library blocks use (fcu-lib.ts).
import { REGIONS, REGISTER_MAP_HASH, type RegisterDef } from "../../../generated/register-map.ts";
import type { IoBindingDef } from "../compiler/compiler.ts";

export type IoDirection = "input" | "output";

export interface IoVar {
	/** IEC name, e.g. ConveyorPwm. */
	name: string;
	type: "BOOL" | "INT" | "UINT" | "DINT" | "UDINT";
	direction: IoDirection;
	/** The binding: input and parameter are read by the program, output written. */
	binding: "input" | "output" | "parameter";
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

/** A command register, bound to a reserved global of the FCU library. */
export interface CommandVar {
	/** __Stepper1StartHoming */
	name: string;
	type: IoVar["type"];
	bits: 16 | 32;
	signed: boolean;
	register: number;
}

/** VALVE1 -> Valve1, CONVEYOR_PWM -> ConveyorPwm. */
export function iecName(registerName: string): string {
	return registerName.toLowerCase().split("_").map((p) => p.charAt(0).toUpperCase() + p.slice(1)).join("");
}

function typeOf(def: RegisterDef): IoVar["type"] | null {
	const count = def.combine?.count ?? 1;
	if (count > 2) return null; // chip UID and the like: not a process value
	return def.display === "bool" ? "BOOL" : count === 2 ? (def.signed ? "DINT" : "UDINT") : def.signed ? "INT" : "UINT";
}

function ioVar(def: RegisterDef, region: string): IoVar | null {
	const type = typeOf(def);
	if (type === null || def.kind === "command") return null;
	const count = def.combine?.count ?? 1;
	const binding = def.bank === "input" ? "input" : def.kind === "parameter" ? "parameter" : "output";
	const direction: IoDirection = binding === "output" ? "output" : "input";
	const prefix = direction === "input" ? "%I" : "%Q";
	const size = type === "BOOL" ? "X" : count === 2 ? "D" : "W";
	return {
		name: iecName(def.name), type, direction, binding, bits: type === "BOOL" ? 1 : count === 2 ? 32 : 16,
		signed: type === "INT" || type === "DINT", address: `${prefix}${size}${def.address}`, register: def.address, region,
		description: def.description ?? "", unit: def.unit, min: def.min, max: def.max,
	};
}

export const IO_VARS: readonly IoVar[] = REGIONS.flatMap((r) => r.registers.map((d) => ioVar(d, r.title)).filter((v): v is IoVar => v !== null))
	.sort((a, b) => (a.direction === b.direction ? a.register - b.register : a.direction === "input" ? -1 : 1));

export const COMMAND_VARS: readonly CommandVar[] = REGIONS.flatMap((r) => r.registers)
	.filter((d) => d.kind === "command")
	.map((d): CommandVar => {
		const type = typeOf(d);
		if (type === null || type === "BOOL") throw new Error(`command register ${d.name}: needs a 16- or 32-bit integer`);
		const bits = type === "DINT" || type === "UDINT" ? 32 : 16;
		return { name: `__${iecName(d.name)}`, type, bits, signed: type === "INT" || type === "DINT", register: d.address };
	})
	.sort((a, b) => a.register - b.register);

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
		const comment = [v.address, v.binding === "parameter" ? "parameter" : "", v.description, v.unit ? `[${v.unit}]` : ""]
			.filter(Boolean).join("  ").replace(/\*\)/g, "* )");
		lines.push(`    ${v.name} : ${v.type}; (* ${comment} *)`);
	}
	lines.push("END_VAR");
	return lines.join("\n") + "\n";
}

/** The bindings for the compiler: every I/O variable and every command register to its register. */
export function ioBindings(): IoBindingDef[] {
	return [
		...IO_VARS.map((v): IoBindingDef => ({
			name: v.name, register: v.register, area: v.binding === "input" ? "input" : "holding", kind: v.binding,
			bits: v.bits, signed: v.signed,
		})),
		...COMMAND_VARS.map((c): IoBindingDef => ({
			name: c.name, register: c.register, area: "holding", kind: "command", bits: c.bits, signed: c.signed,
		})),
	];
}

/** The register map's layout hash (generated, the same as register_map::map_hash in the firmware): the
 *  image carries it, the device refuses an image built for another map. */
export function ioMapHash(): number {
	return REGISTER_MAP_HASH >>> 0;
}
