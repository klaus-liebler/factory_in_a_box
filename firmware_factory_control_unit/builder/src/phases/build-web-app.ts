// Phase "BuildWebApp" (s. docs/build-process.md Abschnitt 6) -- ruft Vite's Build-API direkt auf
// (statt einen "npm run build"-Kindprozess zu spawnen: unter Windows scheitert das Spawnen von
// npm.cmd via execFileSync ohne "shell: true" mit EINVAL, besonders wenn builder/ selbst schon
// aus einem laufenden "npm run ..." heraus aufgerufen wird -- die programmatische API umgeht das
// Problem komplett und ist ohnehin die direktere Variante). Das neue singleFileFirmwareAssetPlugin
// (s. builder/vite-plugin-single-file-firmware-asset.ts) erledigt Inlining, Minifizierung UND das
// Brotli-Schreiben nach assets/index.html.br in einem Rutsch -- kein separater
// "npm run embed"-Schritt mehr noetig.
import path from "node:path";
import { build } from "vite";
import { rootDir, webDir } from "../paths.ts";

export async function buildWebApp(): Promise<void> {
	await build({ root: webDir, configFile: path.join(webDir, "vite.config.ts") });
	console.log(`Web-App gebaut, ${path.join(rootDir, "assets", "index.html.br")} geschrieben.`);
}
