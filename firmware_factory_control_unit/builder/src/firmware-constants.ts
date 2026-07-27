// Liest Core/Inc/constants.hh -- dort von Hand gepflegt (Versions-Bumps, Board-Name-Varianten),
// hier nur ausgelesen, damit build-info.ts (fuers Web-UI) dieselben Werte bekommt, ohne sie ein
// zweites Mal von Hand pflegen zu muessen (s. docs/build-process.md). Bewusst kein
// allgemeiner C++-Parser, sondern gezielte Patterns fuer das aktuell einzige Format in dieser
// Datei -- schlaegt absichtlich hart fehl (statt z.B. still 0/"" zu liefern), wenn sich das
// Format so aendert, dass ein Pattern nicht mehr passt.
import { readFileSync } from "node:fs";
import path from "node:path";
import { rootDir } from "./paths.ts";

export interface FirmwareConstants {
	boardName: string;
	fwVersionMajor: number;
	fwVersionMinor: number;
	fwVersionPatch: number;
}

function extractRequired(source: string, pattern: RegExp, label: string): string {
	const match = source.match(pattern);
	if (!match) {
		throw new Error(
			`Konnte ${label} nicht aus Core/Inc/constants.hh lesen (Pattern passt nicht -- Datei von Hand geaendert?): ${pattern}`
		);
	}
	return match[1]!;
}

// preset entscheidet, welche der beiden BOARD_NAME-Varianten gilt -- exakt das Kriterium, das
// auch CMakeLists.txt fuer BOARD_NUCLEO_H563ZI verwendet (Preset "Debug-Nucleo").
export function readFirmwareConstants(preset: string): FirmwareConstants {
	const constantsPath = path.join(rootDir, "Core", "Inc", "constants.hh");
	const source = readFileSync(constantsPath, "utf8");

	const nucleoBoardName = extractRequired(
		source,
		/BOARD_NUCLEO_H563ZI[\s\S]*?BOARD_NAME\s*=\s*"([^"]+)"/,
		"BOARD_NAME (Nucleo-Testrig-Variante)"
	);
	const defaultBoardName = extractRequired(
		source,
		/#else\s*\n\s*constexpr\s+std::string_view\s+BOARD_NAME\s*=\s*"([^"]+)"/,
		"BOARD_NAME (Standard-Variante)"
	);

	return {
		boardName: preset === "Debug-Nucleo" ? nucleoBoardName : defaultBoardName,
		fwVersionMajor: Number(extractRequired(source, /FW_VERSION_MAJOR\s*=\s*(\d+)/, "FW_VERSION_MAJOR")),
		fwVersionMinor: Number(extractRequired(source, /FW_VERSION_MINOR\s*=\s*(\d+)/, "FW_VERSION_MINOR")),
		fwVersionPatch: Number(extractRequired(source, /FW_VERSION_PATCH\s*=\s*(\d+)/, "FW_VERSION_PATCH")),
	};
}
