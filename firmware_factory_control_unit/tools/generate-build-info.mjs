#!/usr/bin/env node
// Erzeugt web/src/generated/build-info.ts mit Git-Informationen ueber den Stand des Web-Quellcodes
// zum Zeitpunkt des Builds -- das Node-seitige Gegenstueck zu cmake/generate_git_constants.cmake
// (das dieselben Infos fuer die Firmware selbst nach Core/Src/generated/gitconstants.hh schreibt),
// aber unabhaengig vom CMake-Build lauffaehig (auch unter "npm run dev"). Manuell aufrufen bei
// Bedarf (z.B. vor einem Release-Build):
//   node tools/generate-build-info.mjs
//
// Die Ausgabedatei wird eingecheckt (gleiches Muster wie web/src/register-map.ts) -- kein
// Build-Schritt haengt automatisch daran.
import { execSync } from "node:child_process";
import { writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const rootDir = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const outPath = path.join(rootDir, "web", "src", "generated", "build-info.ts");

function git(command, fallback = "unknown") {
	try {
		return execSync(`git ${command}`, { cwd: rootDir, encoding: "utf8" }).trim() || fallback;
	} catch {
		return fallback;
	}
}

const commitHash = git("rev-parse --short HEAD");
const branch = git("rev-parse --abbrev-ref HEAD");
const tag = git("describe --tags --always");
const commitDate = git("log -1 --format=%ai");
const isDirty = git("status --porcelain", "") !== "";
const buildTimestamp = new Date().toISOString();

const content = `// GENERIERT von tools/generate-build-info.mjs -- nicht von Hand editieren. Neu erzeugen per
// "node tools/generate-build-info.mjs" (liest den aktuellen Git-Stand dieses Arbeitsverzeichnisses).
// Build-Zeit-Konstanten fuer die System-Info-App (web/src/apps/system-info-app.ts) -- Laufzeitdaten
// (Uptime, freier Heap, IP/MAC, ...) kommen stattdessen ueber GET /api/system, s. api.ts.

export const GIT_COMMIT_HASH = ${JSON.stringify(commitHash)};
export const GIT_BRANCH = ${JSON.stringify(branch)};
export const GIT_TAG = ${JSON.stringify(tag)};
export const GIT_IS_DIRTY = ${isDirty};
export const GIT_COMMIT_DATE = ${JSON.stringify(commitDate)};
export const BUILD_TIMESTAMP = ${JSON.stringify(buildTimestamp)};
`;

writeFileSync(outPath, content);
console.log(`Wrote ${outPath}`);
console.log(`  Commit: ${commitHash} (${branch}), tag ${tag}, dirty=${isDirty}`);
