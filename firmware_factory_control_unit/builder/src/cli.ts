#!/usr/bin/env node
// Einstiegspunkt fuer alle Build-Phasen (s. docs/build-process.md Abschnitt 6).
//
// Aufruf:
//   node builder/src/cli.ts <Phase> [--preset Debug|Release|Debug-Nucleo] [--board <boardId>]
//
// Einzelne Phasen (Reihenfolge s. Abhaengigkeitsgraph in der Doku):
//   ReadHardwareIDsAndGenerateFilesAndCertsLazy
//   ReadHardwareIDsAndGenerateFilesAndCertsForced
//   ReadModbusRegisterMapAndGenerateFiles
//   ReadGitStatusAndGenerateFiles
//   CopyGeneratedFilesToBuildDirectory
//   BuildWebApp
//   BuildFirmware
//   FlashFirmware
//
// Zusammengesetzte Pipelines:
//   pipeline:lazy    -- Lazy  -> RegisterMap -> GitStatus -> Copy -> BuildWebApp -> BuildFirmware
//   pipeline:forced  -- Forced -> RegisterMap -> GitStatus -> Copy -> BuildWebApp -> BuildFirmware
//   pipeline:flash   -- pipeline:lazy, danach FlashFirmware
import { isValidPreset, buildFirmware, type Preset } from "./phases/build-firmware.ts";
import { buildWebApp } from "./phases/build-web-app.ts";
import { copyGeneratedFilesToBuildDirectory } from "./phases/copy-generated-to-build-directory.ts";
import { flashFirmware } from "./phases/flash-firmware.ts";
import { readGitStatusAndGenerateFiles } from "./phases/read-git-status.ts";
import { readHardwareIdsAndGenerateFilesAndCerts } from "./phases/read-hardware-ids.ts";
import { readModbusRegisterMapAndGenerateFiles } from "./phases/read-modbus-register-map.ts";

interface Args {
	phase: string;
	preset: Preset;
	board?: string;
}

function parseArgs(argv: string[]): Args {
	const [phase, ...rest] = argv;
	if (!phase) {
		throw new Error("Kein Phasenname angegeben. Beispiel: node builder/src/cli.ts ReadModbusRegisterMapAndGenerateFiles");
	}

	let preset = "Debug";
	let board: string | undefined;
	for (let i = 0; i < rest.length; i += 1) {
		if (rest[i] === "--preset") {
			preset = rest[i + 1] ?? preset;
			i += 1;
		} else if (rest[i] === "--board") {
			board = rest[i + 1];
			i += 1;
		}
	}

	if (!isValidPreset(preset)) {
		throw new Error(`Unbekanntes Preset "${preset}". Gueltig: Debug, Release, Debug-Nucleo.`);
	}

	return { phase, preset, board };
}

async function runPhase(phase: string, preset: Preset, board: string | undefined): Promise<void> {
	switch (phase) {
		case "ReadHardwareIDsAndGenerateFilesAndCertsLazy":
			readHardwareIdsAndGenerateFilesAndCerts(false);
			return;
		case "ReadHardwareIDsAndGenerateFilesAndCertsForced":
			readHardwareIdsAndGenerateFilesAndCerts(true);
			return;
		case "ReadModbusRegisterMapAndGenerateFiles":
			readModbusRegisterMapAndGenerateFiles(board);
			return;
		case "ReadGitStatusAndGenerateFiles":
			readGitStatusAndGenerateFiles(preset, board);
			return;
		case "CopyGeneratedFilesToBuildDirectory":
			copyGeneratedFilesToBuildDirectory(board);
			return;
		case "BuildWebApp":
			await buildWebApp();
			return;
		case "BuildFirmware":
			buildFirmware(preset);
			return;
		case "FlashFirmware":
			flashFirmware(preset, board);
			return;
		case "pipeline:lazy":
			readHardwareIdsAndGenerateFilesAndCerts(false);
			readModbusRegisterMapAndGenerateFiles(board);
			readGitStatusAndGenerateFiles(preset, board);
			copyGeneratedFilesToBuildDirectory(board);
			await buildWebApp();
			buildFirmware(preset);
			return;
		case "pipeline:forced":
			readHardwareIdsAndGenerateFilesAndCerts(true);
			readModbusRegisterMapAndGenerateFiles(board);
			readGitStatusAndGenerateFiles(preset, board);
			copyGeneratedFilesToBuildDirectory(board);
			await buildWebApp();
			buildFirmware(preset);
			return;
		case "pipeline:flash":
			readHardwareIdsAndGenerateFilesAndCerts(false);
			readModbusRegisterMapAndGenerateFiles(board);
			readGitStatusAndGenerateFiles(preset, board);
			copyGeneratedFilesToBuildDirectory(board);
			await buildWebApp();
			buildFirmware(preset);
			flashFirmware(preset, board);
			return;
		default:
			throw new Error(`Unbekannte Phase "${phase}".`);
	}
}

try {
	const { phase, preset, board } = parseArgs(process.argv.slice(2));
	await runPhase(phase, preset, board);
} catch (err) {
	console.error(`\nFehler: ${(err as Error).message}`);
	process.exit(1);
}
