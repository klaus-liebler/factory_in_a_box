// The register map over the binary WebSocket (schema/best_binary_buffers/modbus.cs; get_registers() /
// write_holding() in src/web.cpp): reading all registers, writing a holding register, and access by name with
// the register map's layout (32-bit pairs high word first, signed as declared) - so no workspace keeps its own
// lookup or decoding.
import { modbus } from "../../generated/ws-protocol.ts";
import { REGIONS, type RegisterDef } from "../../generated/register-map.ts";
import { wsRequest } from "./ws-client.ts";

export interface RegisterValues {
	holding: number[];
	input: number[];
}

export async function fetchRegisters(): Promise<RegisterValues> {
	const resp = await wsRequest(modbus.NAMESPACE_ID, (correlationId) => modbus.GetRegistersRequest.encode({}, correlationId), modbus.RegistersMessage);
	return { holding: resp.holding, input: resp.input };
}

/** Writes consecutive holding registers from `address` in one write (a 32-bit value never arrives half);
 *  throws when the device refuses (e.g. IEC 61131 is the control source). */
export async function writeHolding(address: number, values: number[]): Promise<void> {
	const resp = await wsRequest(modbus.NAMESPACE_ID, (correlationId) => modbus.WriteHoldingRequest.encode({ address, values }, correlationId), modbus.WriteHoldingResponse);
	if (!resp.success) throw new Error(`Schreiben von Holding-Register ${address} fehlgeschlagen`);
}

/** The words of a register's value, high word first (as the map stores 32-bit values); signed values in
 *  two's complement. */
export function registerWords(reg: RegisterDef, value: number): number[] {
	const count = reg.combine?.count ?? 1;
	if (count === 1) return [value & 0xffff];
	const u = value >>> 0;
	return [u >>> 16, u & 0xffff];
}

/** The pixel value of an "index-rgb565" register: high word the index, low word the colour. */
export function indexRgb565(index: number, rgb565: number): number {
	return ((index & 0xffff) * 0x10000 + (rgb565 & 0xffff)) >>> 0;
}

/** "#rrggbb" -> RGB565 (each component's top bits). */
export function hexToRgb565(hex: string): number {
	const n = parseInt(hex.slice(1), 16);
	const r = (n >> 16) & 0xff, g = (n >> 8) & 0xff, b = n & 0xff;
	return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

/** RGB565 -> "#rrggbb", as the firmware expands it (src/ws2812.hpp rgb888: top bits repeated). */
export function rgb565ToHex(c: number): string {
	const r5 = (c >> 11) & 0x1f, g6 = (c >> 5) & 0x3f, b5 = c & 0x1f;
	const r = (r5 << 3) | (r5 >> 2), g = (g6 << 2) | (g6 >> 4), b = (b5 << 3) | (b5 >> 2);
	return "#" + [r, g, b].map((v) => v.toString(16).padStart(2, "0")).join("");
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

/** Writes a holding register by name, all its words in one write (16 or 32 bits). */
export function writeRegister(reg: string | RegisterDef, value: number): Promise<void> {
	const d = typeof reg === "string" ? register(reg) : reg;
	if (d.bank !== "holding") return Promise.reject(new Error(`${d.name} is not a holding register`));
	return writeHolding(d.address, registerWords(d, value));
}

/** The bits of HEALTH_STATE (enum HealthBits in schema/register_map/Diagnostics.cs; bit 0 "ok" is set when
 *  no other is). */
export const HEALTH_BITS = ["ok", "Übertemperatur", "Versorgung", "CAN-Fehler", "Ethernet ohne Link", "I²C-Sensor", "Schrittmotor"] as const;

/** The health problems of a HEALTH_STATE value (empty: healthy). */
export function healthProblems(state: number): string[] {
	return HEALTH_BITS.flatMap((name, bit) => (bit > 0 && (state & (1 << bit)) !== 0 ? [name] : []));
}
