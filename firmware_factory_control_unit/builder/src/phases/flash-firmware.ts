// Phase "FlashFirmware" (s. docs/build-process.md Abschnitt 6/12) -- ruft STM32_Programmer_CLI
// direkt auf (der bisherige Standardprozess lief nur interaktiv ueber STM32CubeIDE/VS-Code-Debug-
// Launch, s. .vscode/launch.json). "-w <elf> -v -rst": schreiben mit Verify, danach Reset+Start.
// Nach einem verifiziert erfolgreichen Flash wird ein Eintrag in builder.db geschrieben
// (Abschnitt 8) -- ein fehlgeschlagener Flash erzeugt bewusst KEINEN Eintrag.
import { execFileSync } from "node:child_process";
import { existsSync, readFileSync } from "node:fs";
import path from "node:path";
import type { GitInfo } from "../git-info.ts";
import { boardGeneratedDir } from "../board-archive.ts";
import { requireBoardId } from "../board-context.ts";
import { recordSuccessfulFlash } from "../db.ts";
import { stm32ProgrammerCli } from "../environment-config.ts";
import { rootDir } from "../paths.ts";
import type { Preset } from "./build-firmware.ts";

function detectStlinkProbeSerial(): string | undefined {
	try {
		const output = execFileSync(stm32ProgrammerCli(), ["-l"], { encoding: "utf8" });
		const match = output.match(/ST-LINK SN\s*:\s*(\S+)/);
		return match?.[1];
	} catch {
		// Bewusst nicht fatal -- die Sondenseriennummer ist nur eine Zusatzinformation im
		// DB-Eintrag, kein Grund, den Flash-Vorgang selbst abzubrechen.
		return undefined;
	}
}

function readArchivedGitInfo(boardId: string): GitInfo | undefined {
	const jsonPath = path.join(boardGeneratedDir(boardId), "gitstatus.json");
	if (!existsSync(jsonPath)) return undefined;
	return JSON.parse(readFileSync(jsonPath, "utf8")) as GitInfo;
}

// boardId hat die Form "<shortId>_<volle96-Bit-UID-klein>" (s. board-context.ts/read-hardware-ids.ts).
function parseBoardId(boardId: string): { chipUid: string; hostname: string } {
	const [shortId, fullUid] = boardId.split("_");
	return { chipUid: (fullUid ?? boardId).toUpperCase(), hostname: `factory-box-${shortId}` };
}

export function flashFirmware(preset: Preset, explicitBoardId?: string): void {
	const elfPath = path.join(rootDir, "build", preset, "firmware_factory_control_unit.elf");
	if (!existsSync(elfPath)) {
		throw new Error(`Firmware-Elf nicht gefunden: ${elfPath} -- zuerst BuildFirmware fuer Preset "${preset}" ausfuehren.`);
	}

	console.log(`Flashe ${elfPath} ueber STM32_Programmer_CLI...`);
	execFileSync(stm32ProgrammerCli(), ["-c", "port=SWD", "-w", elfPath, "-v", "-rst"], { stdio: "inherit" });
	console.log("Flash erfolgreich verifiziert.");

	const boardId = requireBoardId(explicitBoardId);
	const { chipUid, hostname } = parseBoardId(boardId);
	const gitInfo = readArchivedGitInfo(boardId);
	if (!gitInfo) {
		console.warn(
			"Kein gitstatus.json im Board-Archiv gefunden (ReadGitStatusAndGenerateFiles nicht gelaufen?) -- " +
				"kein builder.db-Eintrag."
		);
		return;
	}

	recordSuccessfulFlash({
		chipUid,
		hostname,
		stlinkProbe: detectStlinkProbeSerial(),
		cmakePreset: preset,
		gitCommitHash: gitInfo.commitHash,
		gitBranch: gitInfo.branch,
		gitIsDirty: gitInfo.isDirty,
		firmwareVersion: gitInfo.version,
	});
}
