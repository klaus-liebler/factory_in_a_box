// Phase "CopyGeneratedFilesToBuildDirectory" (s. docs/build-process.md Abschnitt 6) -- reiner
// Kopiervorgang, keine eigene Generierungslogik. Einzige Phase, die Core/generated/, web/generated/
// und assets/ beschreibt (Grundprinzip 4: pro Zielordner genau ein Schreiber). Quelle ist das
// Board-Archiv des zuletzt erkannten Boards (s. board-context.ts).
import { copyFileSync, existsSync, mkdirSync } from "node:fs";
import path from "node:path";
import { boardGeneratedDir } from "../board-archive.ts";
import { requireBoardId } from "../board-context.ts";
import { assetsDir, coreGeneratedDir, webGeneratedDir } from "../paths.ts";

const CORE_FILES = ["device_ids.hh", "gitconstants.hh", "register_input.inc", "register_holding.inc", "register_maxindex.inc"];
const ASSET_FILES = ["device_certificate.der", "device_key.der"];
const WEB_FILES = ["register-map.ts", "build-info.ts"];

function copyIfExists(srcDir: string, destDir: string, filename: string): void {
	const src = path.join(srcDir, filename);
	if (!existsSync(src)) {
		console.warn(`Uebersprungen: ${filename} nicht im Board-Archiv vorhanden (${src}).`);
		return;
	}
	mkdirSync(destDir, { recursive: true });
	const dest = path.join(destDir, filename);
	copyFileSync(src, dest);
	console.log(`${src} -> ${dest}`);
}

export function copyGeneratedFilesToBuildDirectory(explicitBoardId?: string): void {
	const boardId = requireBoardId(explicitBoardId);
	const src = boardGeneratedDir(boardId);
	if (!existsSync(src)) {
		throw new Error(
			`Board-Archiv ${src} existiert nicht -- zuerst ReadHardwareIDsAndGenerateFilesAndCertsLazy/Forced, ` +
				`ReadModbusRegisterMapAndGenerateFiles und ReadGitStatusAndGenerateFiles fuer Board ${boardId} ausfuehren.`
		);
	}

	for (const file of CORE_FILES) copyIfExists(src, coreGeneratedDir, file);
	for (const file of ASSET_FILES) copyIfExists(src, assetsDir, file);
	for (const file of WEB_FILES) copyIfExists(src, webGeneratedDir, file);
}
