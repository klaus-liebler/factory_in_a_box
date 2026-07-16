// Gzip-komprimiert dist/index.html (Ergebnis von "vite build") und schreibt es als C-Array
// nach ../Core/Src/generated/modbus_ui_page.{c,h}. Wird manuell aufgerufen (npm run embed),
// nicht automatisch vom STM32-CMake-Build -- die generierten Dateien werden eingecheckt,
// analog zu stm32_libs/common_stm32/gpio/generated/*.inc.
import { gzipSync } from "node:zlib";
import { readFileSync, writeFileSync, mkdirSync } from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";

const webDir = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const distFile = path.join(webDir, "dist", "index.html");
const outDir = path.join(webDir, "..", "Core", "Src", "generated");
const outC = path.join(outDir, "modbus_ui_page.c");
const outH = path.join(outDir, "modbus_ui_page.h");

const html = readFileSync(distFile);
const gzipped = gzipSync(html, { level: 9 });

const BYTES_PER_LINE = 20;
const bodyLines = [];
for (let i = 0; i < gzipped.length; i += BYTES_PER_LINE) {
	const chunk = gzipped.subarray(i, i + BYTES_PER_LINE);
	bodyLines.push("    " + Array.from(chunk).map((b) => `0x${b.toString(16).padStart(2, "0")}`).join(", ") + ",");
}

const header = `#pragma once
// Generiert von web/scripts/embed-into-firmware.mjs aus web/dist/index.html (vite build).
// NICHT von Hand editieren -- bei UI-Aenderungen "npm run embed" im web/-Verzeichnis erneut
// ausfuehren und diese generierten Dateien miteinchecken (siehe web/scripts/embed-into-firmware.mjs).
#include <stddef.h>

extern const unsigned char MODBUS_UI_HTML_GZ[];
extern const size_t MODBUS_UI_HTML_GZ_LEN;
`;

const source = `// Generiert von web/scripts/embed-into-firmware.mjs aus web/dist/index.html (vite build).
// Quelle: ${html.length} Bytes unkomprimiert, ${gzipped.length} Bytes gzip.
// NICHT von Hand editieren.
#include "modbus_ui_page.h"

const unsigned char MODBUS_UI_HTML_GZ[] = {
${bodyLines.join("\n")}
};

const size_t MODBUS_UI_HTML_GZ_LEN = sizeof(MODBUS_UI_HTML_GZ);
`;

mkdirSync(outDir, { recursive: true });
writeFileSync(outH, header);
writeFileSync(outC, source);

console.log(`${distFile} (${html.length} B) -> gzip ${gzipped.length} B -> ${outC}`);
