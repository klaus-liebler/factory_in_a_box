// Zentrale Pfad-Konstanten fuer alle builder/-Phasen. Ausgefuehrt wird ausschliesslich per
// "node builder/src/cli.ts ..." (nie ueber vite/tsc), deshalb muss jeder relative Import in
// diesem Verzeichnis die tatsaechliche Dateiendung (".ts") tragen -- Node's natives
// TypeScript-Type-Stripping loest anders als tsc/vite keine ".js"-Spezifizierer auf ".ts"-Dateien
// auf (siehe docs/build-process.md).
import path from "node:path";
import { fileURLToPath } from "node:url";

const here = path.dirname(fileURLToPath(import.meta.url));

// builder/src -> Repo-Wurzel (firmware_factory_control_unit/)
export const rootDir = path.resolve(here, "..", "..");

export const coreGeneratedDir = path.join(rootDir, "Core", "generated");
export const webDir = path.join(rootDir, "web");
export const webGeneratedDir = path.join(webDir, "generated");
export const assetsDir = path.join(rootDir, "assets");
export const buildDir = path.join(rootDir, "build");
export const boardIdCacheFile = path.join(buildDir, ".last-board-id");
