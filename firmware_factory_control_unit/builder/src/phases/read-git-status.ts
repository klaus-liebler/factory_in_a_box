// Phase "ReadGitStatusAndGenerateFiles" (s. docs/build-process.md Abschnitt 6) -- ersetzt sowohl
// cmake/generate_git_constants.cmake als auch das vormalige tools/generate-build-info.mjs durch
// eine einzige Implementierung. Erzeugt drei Ausgaben:
//   - gitconstants.hh  (C++, fuer Core/Src/app.cc) -- nur Git-Daten, die Firmware hat
//     BOARD_NAME/FW_VERSION_*/device_ids.hh bereits nativ.
//   - build-info.ts    (TypeScript, fuer web/src/apps/system-info-app.ts) -- Git-Daten PLUS alle
//     Compile-Zeit-Konstanten, die frueher redundant zur Laufzeit ueber /api/system uebertragen
//     wurden (Board-Name, Firmware-Version, Hostname, Chip-UID, MAC-Adressen): das sind fuer ein
//     gegebenes Firmware-Image/Board feste Werte, kein Grund, sie bei jedem Seitenaufruf per HTTP
//     zu holen.
//   - gitstatus.json   (maschinenlesbar, fuer den builder.db-Eintrag in phases/flash-firmware.ts)
// Immer "Forced" (s. Phasentabelle) -- Git-Status ist billig zu ermitteln, ein Ueberspringen
// braechte keinen Vorteil.
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import path from "node:path";
import { type GitInfo, readGitInfo } from "../git-info.ts";
import { type FirmwareConstants, readFirmwareConstants } from "../firmware-constants.ts";
import { boardGeneratedDir } from "../board-archive.ts";
import { resolveBoardId } from "../board-context.ts";
import { coreGeneratedDir, webGeneratedDir } from "../paths.ts";
import type { DeviceIdentity } from "./read-hardware-ids.ts";

// gitconstants.hh baut roh GIT_COMMIT_MESSAGE in ein C++-String-Literal ein -- Backslashes und
// doppelte Anfuehrungszeichen (z.B. Zitate im Commit-Titel) muessen deshalb escaped werden.
function escapeCppString(value: string): string {
	return value.replace(/\\/g, "\\\\").replace(/"/g, '\\"');
}

function renderGitConstantsHh(info: GitInfo): string {
	return `#pragma once

/**
 * @file gitconstants.hh
 * @brief Auto-generated Git information constants
 * Generated at epoch: ${info.buildTimestampEpoch}
 */

#include <cstdint>
#include <string_view>

namespace git {

/// Git commit short hash
constexpr std::string_view COMMIT_HASH = "${escapeCppString(info.commitHash)}";

/// Git branch name
constexpr std::string_view BRANCH = "${escapeCppString(info.branch)}";

/// Git tag or commit hash
constexpr std::string_view TAG = "${escapeCppString(info.tag)}";

/// Last commit date and time, Unix-Epoch-Sekunden (Formatierung/Zeitzone am Anzeigeort)
constexpr int64_t COMMIT_DATE_EPOCH = ${info.commitDateEpoch};

/// Last commit author
constexpr std::string_view COMMIT_AUTHOR = "${escapeCppString(info.commitAuthor)}";

/// Last commit message
constexpr std::string_view COMMIT_MESSAGE = "${escapeCppString(info.commitMessage)}";

/// Is the working directory dirty (has uncommitted changes)?
constexpr bool IS_DIRTY = ${info.isDirty ? "true" : "false"};

/// Build timestamp, Unix-Epoch-Sekunden (Formatierung/Zeitzone am Anzeigeort)
constexpr int64_t BUILD_TIMESTAMP_EPOCH = ${info.buildTimestampEpoch};

/// Full version string
constexpr std::string_view VERSION = "${escapeCppString(info.version)}";

} // namespace git
`;
}

// deviceIdentity ist undefined, wenn kein Board-Kontext bekannt ist (s.
// docs/build-process.md Abschnitt 4) -- dann bleiben Hostname/Chip-UID/MAC-Adressen leer statt
// die Generierung abzubrechen (Board-Name/Firmware-Version aus constants.hh sind davon
// unabhaengig und immer vorhanden).
function renderBuildInfoTs(info: GitInfo, fw: FirmwareConstants, deviceIdentity: DeviceIdentity | undefined): string {
	return `// GENERIERT von builder/src/phases/read-git-status.ts -- nicht von Hand editieren. Neu
// erzeugen per "npm run phase:git-status" (liest Git-Stand, Core/Inc/constants.hh und das
// Board-Archiv). Kompakte Sammlung aller zur Build-/Flash-Zeit bereits feststehenden Werte fuer
// die System-Info-App (web/src/apps/system-info-app.ts) -- NUR echte Laufzeitdaten (Uptime,
// freier Heap, IP/Netzmaske, Reset-Ursache, PHY-Diagnose) kommen weiterhin ueber GET /api/system,
// s. api.ts.

export const GIT_COMMIT_HASH = ${JSON.stringify(info.commitHash)};
export const GIT_BRANCH = ${JSON.stringify(info.branch)};
export const GIT_TAG = ${JSON.stringify(info.tag)};
export const GIT_IS_DIRTY = ${info.isDirty};
// Unix-Epoch-Sekunden (int64-tauglich) statt vorformatierter Strings -- im Browser als lokale
// Zeit darstellen, z.B. "new Date(GIT_COMMIT_DATE_EPOCH * 1000).toLocaleString()".
export const GIT_COMMIT_DATE_EPOCH = ${info.commitDateEpoch};
export const BUILD_TIMESTAMP_EPOCH = ${info.buildTimestampEpoch};

export const BOARD_NAME = ${JSON.stringify(fw.boardName)};
export const FW_VERSION_MAJOR = ${fw.fwVersionMajor};
export const FW_VERSION_MINOR = ${fw.fwVersionMinor};
export const FW_VERSION_PATCH = ${fw.fwVersionPatch};

// Leer, wenn beim Generieren kein Board-Kontext bekannt war (s. docs/build-process.md Abschnitt 4).
export const DEVICE_HOSTNAME = ${JSON.stringify(deviceIdentity?.hostname ?? "")};
export const DEVICE_CHIP_UID = ${JSON.stringify(deviceIdentity?.chipUid ?? "")};
export const DEVICE_ETH_MAC = ${JSON.stringify(deviceIdentity?.ethMac ?? "")};
export const DEVICE_USB_NCM_MAC = ${JSON.stringify(deviceIdentity?.usbNcmMac ?? "")};

// TLS-Zertifikat, mit dem die Firmware das HTTPS-Web-UI ausliefert (s.
// builder/src/phases/read-hardware-ids.ts readCertificateInfo()) -- Aussteller + Gueltigkeitszeitraum
// direkt aus dem Zertifikat gelesen, nicht separat gepflegt. 0, wenn kein Board-Kontext bekannt war.
export const DEVICE_CERT_ISSUER = ${JSON.stringify(deviceIdentity?.certificate.issuer ?? "")};
export const DEVICE_CERT_ISSUED_AT_EPOCH = ${deviceIdentity?.certificate.issuedAtEpoch ?? 0};
export const DEVICE_CERT_VALID_UNTIL_EPOCH = ${deviceIdentity?.certificate.validUntilEpoch ?? 0};
`;
}

function readDeviceIdentity(boardId: string): DeviceIdentity | undefined {
	const jsonPath = path.join(boardGeneratedDir(boardId), "device-ids.json");
	if (!existsSync(jsonPath)) return undefined;
	return JSON.parse(readFileSync(jsonPath, "utf8")) as DeviceIdentity;
}

export function readGitStatusAndGenerateFiles(preset: string, explicitBoardId?: string): void {
	const info = readGitInfo();
	const fw = readFirmwareConstants(preset);
	const boardId = resolveBoardId(explicitBoardId);
	const deviceIdentity = boardId ? readDeviceIdentity(boardId) : undefined;

	const hhContent = renderGitConstantsHh(info);
	const tsContent = renderBuildInfoTs(info, fw, deviceIdentity);
	const jsonContent = JSON.stringify(info, null, 2) + "\n";

	if (boardId) {
		const dir = boardGeneratedDir(boardId);
		mkdirSync(dir, { recursive: true });
		writeFileSync(path.join(dir, "gitconstants.hh"), hhContent);
		writeFileSync(path.join(dir, "build-info.ts"), tsContent);
		writeFileSync(path.join(dir, "gitstatus.json"), jsonContent);
		console.log(`Git-Status (${info.commitHash}, ${info.branch}, dirty=${info.isDirty}) -> Board-Archiv (${boardId}).`);
		if (!deviceIdentity) {
			console.warn(`Kein device-ids.json im Board-Archiv gefunden -- Hostname/Chip-UID/MAC in build-info.ts bleiben leer.`);
		}
		return;
	}

	// Kein Board-Kontext bekannt (s. docs/build-process.md Abschnitt 4) -- Degradierter Pfad:
	// direkt in die vom Build konsumierten Zielordner schreiben, kein Archiv-Eintrag.
	console.warn(
		"Kein Board-Kontext bekannt -- schreibe Git-Status direkt nach Core/generated bzw. web/generated (kein Archiv-Eintrag)."
	);
	mkdirSync(coreGeneratedDir, { recursive: true });
	mkdirSync(webGeneratedDir, { recursive: true });
	writeFileSync(path.join(coreGeneratedDir, "gitconstants.hh"), hhContent);
	writeFileSync(path.join(webGeneratedDir, "build-info.ts"), tsContent);
}
