#!/usr/bin/env node
// Generiert Core/Src/modbus_register_map.hpp (C++) und web/src/register-map.ts (TypeScript)
// aus der programmiersprachen-neutralen Quelle register-map.json. Manuell aufrufen bei
// Aenderungen an register-map.json:
//   node tools/generate-register-map.mjs
// Beide Ausgabedateien werden eingecheckt (wie stm32_libs/common_stm32/gpio/generated/*.inc
// und Core/Src/generated/modbus_ui_page.c) -- kein Build-Schritt haengt automatisch daran.
import { readFileSync, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const rootDir = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const schemaPath = path.join(rootDir, "register-map.json");
const hppPath = path.join(rootDir, "Core", "Src", "modbus_register_map.hpp");
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
// C++: Core/Src/modbus_register_map.hpp
// ---------------------------------------------------------------------------

function genCppBank(bank, namespaceLabel) {
	let out = `namespace ${namespaceLabel} {\n\n`;
	for (const region of bank.regions) {
		out += `    // ${region.title} (Region-Start ${region.startAddress}, Reserve bis ${region.endAddress})\n`;
		for (const reg of region.registers) {
			const comment = regCommentParts(reg).join(" -- ");
			out += `    constexpr uint16_t ${reg.name} = ${reg.address};${comment ? " // " + comment : ""}\n`;
		}
		out += `    constexpr uint16_t ${region.id}_REGION_START = ${region.startAddress};\n`;
		out += `    constexpr uint16_t ${region.id}_REGION_END   = ${region.endAddress};\n\n`;
	}
	out += `} // namespace ${namespaceLabel}\n`;
	return out;
}

function generateHpp() {
	const lastInputRegion = schema.banks.input.regions.at(-1);
	const lastHoldingRegion = schema.banks.holding.regions.at(-1);

	let healthBits = "namespace HealthBit {\n";
	for (const bit of schema.healthBits) {
		healthBits += `    constexpr uint16_t ${bit.name} = ${bit.bit};\n`;
	}
	healthBits += "}\n";

	return `#pragma once
// GENERIERT von tools/generate-register-map.mjs aus register-map.json -- nicht von Hand
// editieren. Aenderungen an der Register-Map gehoeren in register-map.json, anschliessend
// "node tools/generate-register-map.mjs" erneut ausfuehren.
//
// Adressen sind Offsets in ModbusTcpServer::m_registers.{input,holding}_registers.
// 32-Bit-Werte belegen zwei Register (High-Word zuerst), siehe MbapHeader::serialize.
// Block-Markierungen heissen *_REGION_START/*_REGION_END statt *_BASE/*_END, da z.B.
// ETH_BASE/PWR_BASE bereits als CMSIS-Peripherie-Basisadressen vergeben sind.

#include <cstdint>

namespace ModbusRegisters {

// ---------------------------------------------------------------------------
// Input-Register (FC04, read-only)
// ---------------------------------------------------------------------------

${genCppBank(schema.banks.input, "Input")}
// HealthState Bitfeld (Input::HEALTH_STATE)
${healthBits}
// ---------------------------------------------------------------------------
// Holding-Register (FC03/06/16, read/write)
// ---------------------------------------------------------------------------

${genCppBank(schema.banks.holding, "Holding")}
// Hoechster belegter Index je Register-Bank -- bestimmt die benoetigte Groesse
// der ModbusTcpServer::RegisterModel-Vektoren (siehe app.cpp). Bei Erweiterung
// der Register-Map (neue Region ans Ende angehaengt) hier mitziehen.
constexpr uint16_t INPUT_REGISTER_MAX_INDEX   = Input::${lastInputRegion.id}_REGION_END;
constexpr uint16_t HOLDING_REGISTER_MAX_INDEX = Holding::${lastHoldingRegion.id}_REGION_END;

} // namespace ModbusRegisters
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
// (modbus_register_map.hpp) uebereinstimmen -- bestimmt, wie viele Werte /api/registers
// liefert.
export const INPUT_REGISTER_COUNT = ${lastInputRegion.endAddress + 1}; // Index 0..${lastInputRegion.endAddress}
export const HOLDING_REGISTER_COUNT = ${lastHoldingRegion.endAddress + 1}; // Index 0..${lastHoldingRegion.endAddress}
`;
}

writeFileSync(hppPath, generateHpp());
writeFileSync(tsPath, generateTs());
console.log(`${schemaPath} -> ${hppPath}`);
console.log(`${schemaPath} -> ${tsPath}`);
