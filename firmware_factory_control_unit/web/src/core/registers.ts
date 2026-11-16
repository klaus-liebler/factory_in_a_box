// The register map over the binary WebSocket (schema/best_binary_buffers/modbus.cs; get_registers() /
// write_holding() in src/web.cpp): reading all registers, writing a holding register, and access by name with
// the register map's layout (32-bit pairs high word first, signed as declared) - so no workspace keeps its own
// lookup or decoding.
import { modbus } from "../../generated/ws-protocol.ts";
import { REGIONS, type RegisterDef } from "../../generated/register-map.ts";
import { wsRequest } from "../ws-client.ts";

export interface RegisterValues {
	holding: number[];
	input: number[];
}

export async function fetchRegisters(): Promise<RegisterValues> {
	const resp = await wsRequest(modbus.NAMESPACE_ID, (requestId) => modbus.GetRegistersRequest.encode({ requestId }), modbus.RegistersMessage);
	return { holding: resp.holding, input: resp.input };
}

/** Writes one holding register by address; throws when the device refuses (e.g. IEC 61131 is the control source). */
export async function writeHolding(address: number, value: number): Promise<void> {
	const resp = await wsRequest(modbus.NAMESPACE_ID, (requestId) => modbus.WriteHoldingRequest.encode({ requestId, address, value }), modbus.WriteHoldingResponse);
	if (!resp.success) throw new Error(`Schreiben von Holding-Register ${address} fehlgeschlagen`);
}

const BY_NAME = new Map<string, RegisterDef>(REGIONS.flatMap((r) => r.registers.map((x) => [x.name, x] as const)));

/** A register of the map by name (e.g. "COMPRESSOR_PWM"); throws for an unknown name (a programming error). */
export function register(name: string): RegisterDef {
	const d = BY_NAME.get(name);
	if (!d) throw new Error(`register ${name} is not in the register map`);
	return d;
}

/** A register's value (32-bit pairs high word first, signed as declared); undefined: not in the values. */
export function regValue(values: RegisterValues | null, reg: string | RegisterDef): number | undefined {
	const d = typeof reg === "string" ? BY_NAME.get(reg) : reg;
	if (!values || !d) return undefined;
	const bank = d.bank === "holding" ? values.holding : values.input;
	const hi = bank[d.address];
	if (hi === undefined) return undefined;
	if (d.combine?.count === 2) {
		const lo = bank[d.address + 1];
		if (lo === undefined) return undefined;
		const u = ((hi << 16) | lo) >>> 0;
		return d.signed ? u | 0 : u;
	}
	return d.signed && hi > 0x7fff ? hi - 0x10000 : hi;
}

/** A BOOL-like register (0 / not 0); undefined: not in the values. */
export function regBool(values: RegisterValues | null, reg: string | RegisterDef): boolean | undefined {
	const v = regValue(values, reg);
	return v === undefined ? undefined : v !== 0;
}

/** Writes a 16-bit holding register by name. */
export function writeRegister(reg: string | RegisterDef, value: number): Promise<void> {
	const d = typeof reg === "string" ? register(reg) : reg;
	if (d.bank !== "holding") return Promise.reject(new Error(`${d.name} is not a holding register`));
	return writeHolding(d.address, value);
}

/** The bits of HEALTH_STATE (enum HealthBits in schema/register_map/Diagnostics.cs; bit 0 "ok" is set when
 *  no other is). */
export const HEALTH_BITS = ["ok", "Übertemperatur", "Versorgung", "CAN-Fehler", "Ethernet ohne Link", "I²C-Sensor", "Schrittmotor"] as const;

/** The health problems of a HEALTH_STATE value (empty: healthy). */
export function healthProblems(state: number): string[] {
	return HEALTH_BITS.flatMap((name, bit) => (bit > 0 && (state & (1 << bit)) !== 0 ? [name] : []));
}
