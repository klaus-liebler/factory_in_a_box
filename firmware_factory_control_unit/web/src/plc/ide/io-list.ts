// The I/O variables of the IEC program, derived from the register map (generated/register-map.ts).
// Mockup state: they are plain global variables; the binding to the registers (%I/%Q, register
// classes) comes with P2 (docs/plc-iec61131.md section 6).
import { REGIONS, type RegisterDef } from "../../../generated/register-map.ts";

export type IoDirection = "input" | "output";

export interface IoVar {
	/** IEC name, e.g. ConveyorPwm. */
	name: string;
	type: "BOOL" | "INT" | "UINT" | "DINT" | "UDINT";
	direction: IoDirection;
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
	const type: IoVar["type"] = def.display === "bool" ? "BOOL" : count === 2 ? (def.signed ? "DINT" : "UDINT") : def.signed ? "INT" : "UINT";
	const direction: IoDirection = def.bank === "input" ? "input" : "output";
	const prefix = direction === "input" ? "%I" : "%Q";
	const size = type === "BOOL" ? "X" : count === 2 ? "D" : "W";
	return {
		name: iecName(def.name), type, direction, address: `${prefix}${size}${def.address}`, register: def.address, region,
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
