// Phase "ReadModbusRegisterMapAndGenerateFiles" (s. docs/build-process.md Abschnitt 6) --
// Nachfolger von tools/generate-register-map.mjs, inhaltlich unveraendert, nur das Ziel hat sich
// geaendert: es wird ins Board-Archiv geschrieben (oder, ohne Board-Kontext, direkt nach
// Core/generated/web/generated, s. docs/build-process.md Abschnitt 4). Immer "Forced" --
// schema-getrieben und billig, ein Ueberspringen braechte keinen Vorteil.
//
// Erzeugt drei reine Konstanten-Fragmente (register_input.inc, register_holding.inc,
// register_maxindex.inc) und register-map.ts (TypeScript) aus der programmiersprachen-neutralen
// Quelle register-map.json. Die .inc-Fragmente enthalten bewusst NUR constexpr-Zeilen -- keine
// namespace-Deklaration, keine geschweiften Klammern. Die umschliessenden Namespaces
// (ModbusRegisters::Input/::Holding) stehen fest in Core/Src/modbus_register_model.hh (von Hand
// gepflegt, bindet die drei Fragmente per #include ein).
import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import path from "node:path";
import { boardGeneratedDir } from "../board-archive.ts";
import { resolveBoardId } from "../board-context.ts";
import { coreGeneratedDir, rootDir, webGeneratedDir } from "../paths.ts";

// register-map.json ist von Hand gepflegtes Schema, kein generierter Code -- hier bewusst locker
// (unknown/any) typisiert statt ein volles Schema-Interface nachzubilden, das bei jeder
// register-map.json-Erweiterung mitgepflegt werden muesste.
/* eslint-disable @typescript-eslint/no-explicit-any */
type Schema = any;
type RegisterEntry = any;
type Region = any;

function regCommentParts(reg: RegisterEntry): string[] {
	const parts: string[] = [];
	if (reg.description) parts.push(reg.description);
	if (reg.unit) parts.push(`[${reg.unit}]`);
	if (reg.signed) parts.push("signed");
	if (reg.gpio) parts.push(reg.gpio);
	if (reg.i2c) parts.push(`${reg.i2c.bus}${reg.i2c.irqPin ? ", IRQ " + reg.i2c.irqPin : ""}`);
	return parts;
}

// Ein Eintrag in region.registers ist entweder ein normales Register (hat "name") oder eine
// combine-Gruppe (hat "combine", darin ein "registers"-Array mit den zugrundeliegenden
// Einzelregistern -- s. register-map.json, z.B. CHIP_ID/CAN_TX_COUNT). Fuer die C++-Ausgabe
// spielt die Gruppierung keine Rolle, nur die zugrundeliegenden Einzelregister zaehlen.
function flattenRegisters(registers: RegisterEntry[]): RegisterEntry[] {
	const result: RegisterEntry[] = [];
	for (const entry of registers) {
		if (entry.combine) {
			result.push(...entry.combine.registers);
		} else {
			result.push(entry);
		}
	}
	return result;
}

function genFlatBank(bank: { regions: Region[] }): string {
	let out = "";
	for (const region of bank.regions) {
		out += `// ${region.title} (Region-Start ${region.startAddress}, Reserve bis ${region.endAddress})\n`;
		for (const reg of flattenRegisters(region.registers)) {
			const comment = regCommentParts(reg).join(" -- ");
			out += `constexpr uint16_t ${reg.name} = ${reg.address};${comment ? " // " + comment : ""}\n`;
		}
		out += `constexpr uint16_t ${region.id}_REGION_START = ${region.startAddress};\n`;
		out += `constexpr uint16_t ${region.id}_REGION_END   = ${region.endAddress};\n\n`;
	}
	return out;
}

const INC_FILE_HEADER = `// GENERIERT von builder/src/phases/read-modbus-register-map.ts aus register-map.json -- nicht
// von Hand editieren. Aenderungen an der Register-Map gehoeren in register-map.json,
// anschliessend "npm run phase:register-map" erneut ausfuehren.
//
// Nur per #include aus Core/Src/modbus_register_model.hh eingebunden (innerhalb des dort
// bereits geoeffneten namespace-Blocks), nicht eigenstaendig uebersetzbar -- enthaelt bewusst
// nur constexpr-Zeilen, keine eigene namespace-Deklaration/Klammern.
`;

function generateInputInc(schema: Schema): string {
	let healthBits = "// HealthState-Bitfeld (Input::HEALTH_STATE), s. register-map.json healthBits\n";
	for (const bit of schema.healthBits) {
		healthBits += `constexpr uint16_t HEALTH_BIT_${bit.name} = ${bit.bit};\n`;
	}

	return `${INC_FILE_HEADER}//
// Adressen sind Offsets in ModbusRegisterModel::input_registers_.
// 32-Bit-Werte belegen zwei Register (High-Word zuerst), siehe MbapHeader::serialize.
// Block-Markierungen heissen *_REGION_START/*_REGION_END statt *_BASE/*_END, da z.B.
// ETH_BASE/PWR_BASE bereits als CMSIS-Peripherie-Basisadressen vergeben sind.

${genFlatBank(schema.banks.input)}${healthBits}`;
}

function generateHoldingInc(schema: Schema): string {
	return `${INC_FILE_HEADER}//
// Adressen sind Offsets in ModbusRegisterModel::holding_registers_.
// 32-Bit-Werte belegen zwei Register (High-Word zuerst), siehe MbapHeader::serialize.

${genFlatBank(schema.banks.holding)}`;
}

function generateMaxIndexInc(schema: Schema): string {
	const lastInputRegion = schema.banks.input.regions.at(-1);
	const lastHoldingRegion = schema.banks.holding.regions.at(-1);

	return `${INC_FILE_HEADER}//
// Hoechster belegter Index je Register-Bank -- bestimmt die Groesse der
// ModbusRegisterModel-Arrays (s. modbus_register_model.hh). Bei Erweiterung der Register-Map
// (neue Region ans Ende angehaengt) hier mitziehen. Referenziert Input::/Holding::, die im
// umschliessenden modbus_register_model.hh bereits vor dieser #include definiert sind.
constexpr uint16_t INPUT_REGISTER_MAX_INDEX   = Input::${lastInputRegion.id}_REGION_END;
constexpr uint16_t HOLDING_REGISTER_MAX_INDEX = Holding::${lastHoldingRegion.id}_REGION_END;
`;
}

function tsStringLiteral(s: string): string {
	return JSON.stringify(s);
}

// entry ist entweder ein normales Register (hat "name") oder eine combine-Gruppe (hat
// "combine", darin "registers": [...] mit den zugrundeliegenden Einzelregistern -- s.
// register-map.json). Erzeugt in beiden Faellen GENAU einen RegisterDef-Eintrag; die
// combine-Gruppe nutzt ihre eigenen label/description/display-Angaben statt derer des ersten
// zugrundeliegenden Registers.
function genTsRegister(entry: RegisterEntry, bank: "input" | "holding"): string {
	if (entry.combine) {
		const group = entry.combine;
		const first = group.registers[0];
		const fields = [`name: ${tsStringLiteral(group.label)}`, `address: ${first.address}`, `bank: ${tsStringLiteral(bank)}`];
		const description = group.description ?? first.description;
		if (description) fields.push(`description: ${tsStringLiteral(description)}`);
		if (first.unit) fields.push(`unit: ${tsStringLiteral(first.unit)}`);
		if (group.signed) fields.push(`signed: true`);
		fields.push(`display: ${tsStringLiteral(group.display ?? "decimal")}`);
		fields.push(`combine: { count: ${group.registers.length} }`);
		return `{ ${fields.join(", ")} }`;
	}

	const reg = entry;
	const fields = [`name: ${tsStringLiteral(reg.name)}`, `address: ${reg.address}`, `bank: ${tsStringLiteral(bank)}`];
	if (reg.description) fields.push(`description: ${tsStringLiteral(reg.description)}`);
	if (reg.unit) fields.push(`unit: ${tsStringLiteral(reg.unit)}`);
	if (reg.gpio) fields.push(`gpio: ${tsStringLiteral(reg.gpio)}`);
	if (reg.signed) fields.push(`signed: true`);
	if (reg.i2c) {
		const i2cFields = [`bus: ${tsStringLiteral(reg.i2c.bus)}`];
		if (reg.i2c.irqPin) i2cFields.push(`irqPin: ${tsStringLiteral(reg.i2c.irqPin)}`);
		fields.push(`i2c: { ${i2cFields.join(", ")} }`);
	}
	if (reg.min !== undefined) fields.push(`min: ${reg.min}`);
	if (reg.max !== undefined) fields.push(`max: ${reg.max}`);
	fields.push(`display: ${tsStringLiteral(reg.display ?? "decimal")}`);
	return `{ ${fields.join(", ")} }`;
}

function generateTs(schema: Schema): string {
	const orderedByOriginalLayout: { title: string; bank: "input" | "holding"; registers: RegisterEntry[] }[] = [];
	for (const region of schema.banks.input.regions) {
		if (region.registers.length > 0) orderedByOriginalLayout.push({ title: region.title, bank: "input", registers: region.registers });
	}
	for (const region of schema.banks.holding.regions) {
		if (region.registers.length > 0) orderedByOriginalLayout.push({ title: region.title, bank: "holding", registers: region.registers });
	}

	const regionsSource = orderedByOriginalLayout
		.map((region) => {
			const regsSource = region.registers.map((entry: RegisterEntry) => "\t\t\t" + genTsRegister(entry, region.bank)).join(",\n");
			return `\t{\n\t\ttitle: ${tsStringLiteral(region.title)},\n\t\tregisters: [\n${regsSource}\n\t\t]\n\t}`;
		})
		.join(",\n");

	const lastInputRegion = schema.banks.input.regions.at(-1);
	const lastHoldingRegion = schema.banks.holding.regions.at(-1);

	return `// GENERIERT von builder/src/phases/read-modbus-register-map.ts aus register-map.json -- nicht
// von Hand editieren. Aenderungen an der Register-Map gehoeren in register-map.json,
// anschliessend "npm run phase:register-map" erneut ausfuehren.

export type RegisterBank = "input" | "holding";
// Eine Darstellungsform fuers Lesen UND Schreiben (s. register-panel.ts):
//   - "decimal" (Default): Zahl; bei Holding-Registern generisches Zahlenfeld + Schreiben-Button.
//   - "hex" / "binary": Zahl als 0x.../0b...-String; Schreiben wie "decimal".
//   - "bool": bei Input-Registern grau/gruen eingefaerbtes Badge (0=aus/1=an), bei
//     Holding-Registern zusaetzlich ein Toggle-Switch zum Schreiben.
//   - "bool-error": wie "bool", aber grau/rot statt grau/gruen -- fuer Register, bei denen 1
//     einen Fehler-/Alarmzustand meldet statt eines neutralen "an" (z.B. PWR_ALERT).
//   - "status": Fehlercode-Konvention (0 = ok, ungleich 0 = Fehler) -- 0 grau/neutral, jeder
//     andere Wert rot MIT der tatsaechlichen Zahl (nicht nur "1"), fuer *_STATUS-Register, die
//     kuenftig auch verschiedene Fehlercodes tragen koennten (aktuell ueberall nur 0/1).
//   - "range": Slider (braucht min/max); nur fuer Holding-Register sinnvoll.
export type RegisterDisplay = "decimal" | "hex" | "binary" | "bool" | "bool-error" | "status" | "range";

export interface RegisterDef {
	name: string;
	address: number;
	bank: RegisterBank;
	display: RegisterDisplay;
	description?: string;
	unit?: string;
	signed?: boolean;
	gpio?: string;
	i2c?: { bus: string; irqPin?: string };
	min?: number;
	max?: number;
	// Fasst "count" aufeinanderfolgende Register (ab "address", MSB zuerst) zu einem
	// gemeinsamen Anzeigewert zusammen (z.B. 6 Register -> eine 96-Bit Chip-ID als Hex-String,
	// oder 2 Register -> ein 32-Bit-Zaehlerstand). Nur lesend, nur sinnvoll bei Input-Registern.
	combine?: { count: number };
}

export interface RegisterRegion {
	title: string;
	registers: RegisterDef[];
}

export const REGIONS: RegisterRegion[] = [
${regionsSource}
];

// Muss mit ModbusRegisters::INPUT_REGISTER_MAX_INDEX / HOLDING_REGISTER_MAX_INDEX
// (generated/register_maxindex.inc) uebereinstimmen -- bestimmt, wie viele Werte
// /api/registers liefert.
export const INPUT_REGISTER_COUNT = ${lastInputRegion.endAddress + 1}; // Index 0..${lastInputRegion.endAddress}
export const HOLDING_REGISTER_COUNT = ${lastHoldingRegion.endAddress + 1}; // Index 0..${lastHoldingRegion.endAddress}
`;
}

export function readModbusRegisterMapAndGenerateFiles(explicitBoardId?: string): void {
	const schemaPath = path.join(rootDir, "register-map.json");
	const schema: Schema = JSON.parse(readFileSync(schemaPath, "utf8"));

	const inputInc = generateInputInc(schema);
	const holdingInc = generateHoldingInc(schema);
	const maxIndexInc = generateMaxIndexInc(schema);
	const ts = generateTs(schema);

	const boardId = resolveBoardId(explicitBoardId);
	const coreOut = boardId ? boardGeneratedDir(boardId) : coreGeneratedDir;
	const webOut = boardId ? boardGeneratedDir(boardId) : webGeneratedDir;

	mkdirSync(coreOut, { recursive: true });
	mkdirSync(webOut, { recursive: true });
	writeFileSync(path.join(coreOut, "register_input.inc"), inputInc);
	writeFileSync(path.join(coreOut, "register_holding.inc"), holdingInc);
	writeFileSync(path.join(coreOut, "register_maxindex.inc"), maxIndexInc);
	writeFileSync(path.join(webOut, "register-map.ts"), ts);

	if (boardId) {
		console.log(`${schemaPath} -> Board-Archiv (${boardId})`);
	} else {
		console.warn(
			"Kein Board-Kontext bekannt -- schreibe Register-Map direkt nach Core/generated bzw. web/generated (kein Archiv-Eintrag)."
		);
	}
}
