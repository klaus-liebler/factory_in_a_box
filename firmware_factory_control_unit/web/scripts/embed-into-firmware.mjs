// Brotli-komprimiert dist/index.html (Ergebnis von "vite build": viteSingleFile() inlined
// JS+CSS, inlineSingleFileMinifyPlugin() entfernt zusaetzliche Leerzeichen, siehe
// vite.config.ts) und schreibt das Ergebnis als rohes Binary nach ../assets/index.html.br.
// Wird manuell aufgerufen (npm run embed), nicht automatisch vom STM32-CMake-Build.
//
// Die .br-Datei wird eingecheckt (wie assets/device_certificate.der/device_key.der) und beim
// Firmware-Build per objcopy in die .flash_assets-Linker-Section eingebunden (siehe
// CMakeLists.txt) -- kein generiertes C-Array mehr noetig.
import { brotliCompressSync, constants as zlibConstants } from "node:zlib";
import { readFileSync, writeFileSync, mkdirSync } from "node:fs";
import { fileURLToPath } from "node:url";
import path from "node:path";

const webDir = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const distFile = path.join(webDir, "dist", "index.html");
const assetsDir = path.join(webDir, "..", "assets");
const outFile = path.join(assetsDir, "index.html.br");

const html = readFileSync(distFile);
const compressed = brotliCompressSync(html, {
	params: {
		[zlibConstants.BROTLI_PARAM_QUALITY]: zlibConstants.BROTLI_MAX_QUALITY,
		[zlibConstants.BROTLI_PARAM_SIZE_HINT]: html.length
	}
});

mkdirSync(assetsDir, { recursive: true });
writeFileSync(outFile, compressed);

console.log(`${distFile} (${html.length} B) -> brotli ${compressed.length} B -> ${outFile}`);
