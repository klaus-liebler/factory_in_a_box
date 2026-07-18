#!/usr/bin/env node
// Generiert drei reine Konstanten-Fragmente (Core/Src/generated/register_input.inc,
// register_holding.inc, register_maxindex.inc) und web/src/register-map.ts (TypeScript) aus
// der programmiersprachen-neutralen Quelle register-map.json. Manuell aufrufen bei
// Aenderungen an register-map.json:
//   node tools/generate-register-map.mjs
//
// Die .inc-Fragmente enthalten bewusst NUR constexpr-Zeilen -- keine namespace-Deklaration,
// keine geschweiften Klammern. Die umschliessenden Namespaces (ModbusRegisters::Input/::Holding)
// stehen fest in Core/Src/modbus_register_model.hh (von Hand gepflegt, bindet die drei
// Fragmente per #include ein, gleiches Muster wie stm32_libs/common_stm32/gpio/generated/*.inc).
//
// Alle Ausgabedateien werden eingecheckt (wie stm32_libs/common_stm32/gpio/generated/*.inc
// und Core/Src/generated/modbus_ui_page.c) -- kein Build-Schritt haengt automatisch daran.
import { readFileSync, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const rootDir = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const schemaPath = path.join(rootDir, "register-map.json");
const generatedDir = path.join(rootDir, "Core", "Src", "generated");
const inputIncPath = path.join(generatedDir, "register_input.inc");
const holdingIncPath = path.join(generatedDir, "register_holding.inc");
const maxIndexIncPath = path.join(generatedDir, "register_maxindex.inc");
const tsPath = path.join(rootDir, "web", "src", "register-map.ts");

const schema = JSON.parse(readFileSync(schemaPath, "utf8"));

function regCommentParts(reg) {
	const parts = [];
	if (reg.description) parts.push(reg.description);
	if (reg.unit) parts.push(`[${reg.unit}]`);
	if (reg.signed) parts.push("signed");
	if (reg.gpio) parts.push(reg.gpio);
	if (reg.i2c) parts.push(`${reg.i2c.bus}${reg.i2c.irqPin ? ", IRQ " + reg.i2c.irqPin : ""}`);
	return parts;
}

// ---------------------------------------------------------------------------
// C++: Core/Src/generated/register_input.inc / register_holding.inc / register_maxindex.inc
// ---------------------------------------------------------------------------
// Jede Datei enthaelt NUR constexpr-Zeilen (keine namespace-Deklaration, keine geschweiften
// Klammern) -- die umschliessenden namespace Input {}/Holding {} stehen in
// Core/Src/modbus_register_model.hh.

function genFlatBank(bank) {
	let out = "";
	for (const region of bank.regions) {
		out += `// ${region.title} (Region-Start ${region.startAddress}, Reserve bis ${region.endAddress})\n`;
		for (const reg of region.registers) {
			const comment = regCommentParts(reg).join(" -- ");
			out += `constexpr uint16_t ${reg.name} = ${reg.address};${comment ? " // " + comment : ""}\n`;
		}
		out += `constexpr uint16_t ${region.id}_REGION_START = ${region.startAddress};\n`;
		out += `constexpr uint16_t ${region.id}_REGION_END   = ${region.endAddress};\n\n`;
	}
	return out;
}

const incFileHeader = `// GENERIERT von tools/generate-register-map.mjs aus register-map.json -- nicht von Hand
// editieren. Aenderungen an der Register-Map gehoeren in register-map.json, anschliessend
// "node tools/generate-register-map.mjs" erneut ausfuehren.
//
// Nur per #include aus Core/Src/modbus_register_model.hh eingebunden (innerhalb des dort
// bereits geoeffneten namespace-Blocks), nicht eigenstaendig uebersetzbar -- enthaelt bewusst
// nur constexpr-Zeilen, keine eigene namespace-Deklaration/Klammern.
`;

function generateInputInc() {
	let healthBits = "// HealthState-Bitfeld (Input::HEALTH_STATE), s. register-map.json healthBits\n";
	for (const bit of schema.healthBits) {
		healthBits += `constexpr uint16_t HEALTH_BIT_${bit.name} = ${bit.bit};\n`;
	}

	return `${incFileHeader}//
// Adressen sind Offsets in ModbusRegisterModel::input_registers_.
// 32-Bit-Werte belegen zwei Register (High-Word zuerst), siehe MbapHeader::serialize.
// Block-Markierungen heissen *_REGION_START/*_REGION_END statt *_BASE/*_END, da z.B.
// ETH_BASE/PWR_BASE bereits als CMSIS-Peripherie-Basisadressen vergeben sind.

${genFlatBank(schema.banks.input)}${healthBits}`;
}

function generateHoldingInc() {
	return `${incFileHeader}//
// Adressen sind Offsets in ModbusRegisterModel::holding_registers_.
// 32-Bit-Werte belegen zwei Register (High-Word zuerst), siehe MbapHeader::serialize.

${genFlatBank(schema.banks.holding)}`;
}

function generateMaxIndexInc() {
	const lastInputRegion = schema.banks.input.regions.at(-1);
	const lastHoldingRegion = schema.banks.holding.regions.at(-1);

	return `${incFileHeader}//
// Hoechster belegter Index je Register-Bank -- bestimmt die Groesse der
// ModbusRegisterModel-Arrays (s. modbus_register_model.hh). Bei Erweiterung der Register-Map
// (neue Region ans Ende angehaengt) hier mitziehen. Referenziert Input::/Holding::, die im
// umschliessenden modbus_register_model.hh bereits vor dieser #include definiert sind.
constexpr uint16_t INPUT_REGISTER_MAX_INDEX   = Input::${lastInputRegion.id}_REGION_END;
constexpr uint16_t HOLDING_REGISTER_MAX_INDEX = Holding::${lastHoldingRegion.id}_REGION_END;
`;
}

// ---------------------------------------------------------------------------
// TypeScript: web/src/register-map.ts
// ---------------------------------------------------------------------------

function tsStringLiteral(s) {
	return JSON.stringify(s);
}

function genTsRegister(reg, bank) {
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
	if (bank === "holding") {
		fields.push(`control: ${tsStringLiteral(reg.control ?? "number")}`);
		if (reg.min !== undefined) fields.push(`min: ${reg.min}`);
		if (reg.max !== undefined) fields.push(`max: ${reg.max}`);
	} else {
		fields.push(`control: "readonly"`);
	}
	return `{ ${fields.join(", ")} }`;
}

function generateTs() {
	// Reihenfolge in der UI: Input-Regionen (Sensoren) zuerst, dann Holding (Aktuatoren) --
	// gleiche Reihenfolge wie in der bisherigen handgeschriebenen register-map.ts.
	const orderedByOriginalLayout = [];
	for (const region of schema.banks.input.regions) {
		if (region.registers.length > 0) orderedByOriginalLayout.push({ title: region.title, bank: "input", registers: region.registers });
	}
	for (const region of schema.banks.holding.regions) {
		if (region.registers.length > 0) orderedByOriginalLayout.push({ title: region.title, bank: "holding", registers: region.registers });
	}

	const regionsSource = orderedByOriginalLayout
		.map((region) => {
			const regsSource = region.registers.map((reg) => "\t\t\t" + genTsRegister(reg, region.bank)).join(",\n");
			return `\t{\n\t\ttitle: ${tsStringLiteral(region.title)},\n\t\tregisters: [\n${regsSource}\n\t\t]\n\t}`;
		})
		.join(",\n");

	const lastInputRegion = schema.banks.input.regions.at(-1);
	const lastHoldingRegion = schema.banks.holding.regions.at(-1);

	return `// GENERIERT von tools/generate-register-map.mjs aus register-map.json -- nicht von Hand
// editieren. Aenderungen an der Register-Map gehoeren in register-map.json, anschliessend
// "node tools/generate-register-map.mjs" erneut ausfuehren.

export type RegisterBank = "input" | "holding";
export type RegisterControl = "readonly" | "toggle" | "slider" | "number";

export interface RegisterDef {
	name: string;
	address: number;
	bank: RegisterBank;
	control: RegisterControl;
	description?: string;
	unit?: string;
	signed?: boolean;
	gpio?: string;
	i2c?: { bus: string; irqPin?: string };
	min?: number;
	max?: number;
}

export interface RegisterRegion {
	title: string;
	registers: RegisterDef[];
}

export const REGIONS: RegisterRegion[] = [
${regionsSource}
];

// Muss mit ModbusRegisters::INPUT_REGISTER_MAX_INDEX / HOLDING_REGISTER_MAX_INDEX
// (generated/modbus_register_map.inc) uebereinstimmen -- bestimmt, wie viele Werte
// /api/registers liefert.
export const INPUT_REGISTER_COUNT = ${lastInputRegion.endAddress + 1}; // Index 0..${lastInputRegion.endAddress}
export const HOLDING_REGISTER_COUNT = ${lastHoldingRegion.endAddress + 1}; // Index 0..${lastHoldingRegion.endAddress}
`;
}

writeFileSync(inputIncPath, generateInputInc());
writeFileSync(holdingIncPath, generateHoldingInc());
writeFileSync(maxIndexIncPath, generateMaxIndexInc());
writeFileSync(tsPath, generateTs());
console.log(`${schemaPath} -> ${inputIncPath}`);
console.log(`${schemaPath} -> ${holdingIncPath}`);
console.log(`${schemaPath} -> ${maxIndexIncPath}`);
console.log(`${schemaPath} -> ${tsPath}`);
