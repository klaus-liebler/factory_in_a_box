// Phase "ReadGitStatusAndGenerateFiles" (s. docs/build-process.md Abschnitt 2) -- erzeugt vier Ausgaben:
//   - gitconstants.hh      (C++, fuer Core/Src/app.cc)
//   - firmware_constants.hh (C++, BOARD_NAME/FW_VERSION_*, s. RenderFirmwareConstantsHh unten)
//   - build-info.ts        (TypeScript, fuer web/src/apps/system-info-app.ts)
//   - gitstatus.json       (maschinenlesbar, fuer den board.json-Eintrag in Phases/FlashFirmware.cs)
// Immer "Forced" -- Git-Status ist billig zu ermitteln, ein Ueberspringen braechte keinen Vorteil.
using System.Text.Json;

namespace Builder.Phases;

public static class ReadGitStatus
{
	// gitconstants.hh baut roh GIT_COMMIT_MESSAGE in ein C++-String-Literal ein -- Backslashes und
	// doppelte Anfuehrungszeichen (z.B. Zitate im Commit-Titel) muessen deshalb escaped werden.
	private static string EscapeCppString(string value) => value.Replace("\\", "\\\\").Replace("\"", "\\\"");

	private static string RenderGitConstantsHh(GitInfo info)
	{
		return $$"""
			#pragma once

			/**
			 * @file gitconstants.hh
			 * @brief Auto-generated Git information constants
			 * Generated at epoch: {{info.BuildTimestampEpoch}}
			 */

			#include <cstdint>
			#include <string_view>

			namespace git {

			/// Git commit short hash
			constexpr std::string_view COMMIT_HASH = "{{EscapeCppString(info.CommitHash)}}";

			/// Git branch name
			constexpr std::string_view BRANCH = "{{EscapeCppString(info.Branch)}}";

			/// Git tag or commit hash
			constexpr std::string_view TAG = "{{EscapeCppString(info.Tag)}}";

			/// Last commit date and time, Unix-Epoch-Sekunden (Formatierung/Zeitzone am Anzeigeort)
			constexpr int64_t COMMIT_DATE_EPOCH = {{info.CommitDateEpoch}};

			/// Last commit author
			constexpr std::string_view COMMIT_AUTHOR = "{{EscapeCppString(info.CommitAuthor)}}";

			/// Last commit message
			constexpr std::string_view COMMIT_MESSAGE = "{{EscapeCppString(info.CommitMessage)}}";

			/// Is the working directory dirty (has uncommitted changes)?
			constexpr bool IS_DIRTY = {{(info.IsDirty ? "true" : "false")}};

			/// Build timestamp, Unix-Epoch-Sekunden (Formatierung/Zeitzone am Anzeigeort)
			constexpr int64_t BUILD_TIMESTAMP_EPOCH = {{info.BuildTimestampEpoch}};

			/// Full version string
			constexpr std::string_view VERSION = "{{EscapeCppString(info.Version)}}";

			} // namespace git

			""";
	}

	// Globaler Namespace (bewusst NICHT "namespace git {...}"): Core/Src/app.cc verwendet
	// BOARD_NAME/FW_VERSION_MAJOR unqualifiziert. Ersetzt die vormals von Hand in
	// Core/Inc/constants.hh gepflegten Konstanten (samt #ifdef BOARD_NUCLEO_H563ZI-Umschaltung fuer
	// BOARD_NAME) -- boardName kommt hier bereits fertig aufgeloest an (Board-Archiv-Nachschlag in
	// board.json bzw. Default, s. Phases/ReadHardwareIds.cs), fwVersion aus firmware-version.json
	// (s. FirmwareVersion.cs).
	private static string RenderFirmwareConstantsHh(FirmwareVersionInfo fwVersion, string boardName)
	{
		return $$"""
			#pragma once
			// GENERIERT von builder/Phases/ReadGitStatus.cs aus firmware-version.json bzw. dem
			// Board-Archiv -- nicht von Hand editieren. Fuer einen Versions-Bump firmware-version.json
			// aendern, fuer eine Board-Type-Korrektur board.json (s. docs/build-process.md Abschnitt 5),
			// anschliessend die Phase "ReadGitStatusAndGenerateFiles" erneut ausfuehren.
			#include <cstdint>
			#include <string_view>

			constexpr std::string_view BOARD_NAME = "{{EscapeCppString(boardName)}}";
			constexpr uint16_t FW_VERSION_MAJOR = {{fwVersion.Major}};
			constexpr uint16_t FW_VERSION_MINOR = {{fwVersion.Minor}};
			constexpr uint16_t FW_VERSION_PATCH = {{fwVersion.Patch}};

			""";
	}

	private static string TsStringLiteral(string s) => JsonSerializer.Serialize(s, Json.Compact);

	// deviceIdentity ist null, wenn kein Board-Kontext bekannt ist (s. docs/build-process.md
	// Abschnitt 4) -- dann bleiben Hostname/Chip-UID/MAC-Adressen leer statt die Generierung
	// abzubrechen (boardName/fwVersion sind davon unabhaengig und immer vorhanden, s. Run() unten).
	private static string RenderBuildInfoTs(GitInfo info, FirmwareVersionInfo fwVersion, string boardName, DeviceIdentity? deviceIdentity)
	{
		return $$"""
			// GENERIERT von builder/Phases/ReadGitStatus.cs -- nicht von Hand editieren. Neu erzeugen
			// per der Phase "ReadGitStatusAndGenerateFiles" (liest Git-Stand, firmware-version.json und
			// das Board-Archiv). Kompakte Sammlung aller zur Build-/Flash-Zeit bereits feststehenden
			// Werte fuer die System-Info-App (web/src/apps/system-info-app.ts) -- NUR echte
			// Laufzeitdaten (Uptime, freier Heap, IP/Netzmaske, Reset-Ursache, PHY-Diagnose) kommen
			// weiterhin ueber GET /api/system, s. api.ts.

			export const GIT_COMMIT_HASH = {{TsStringLiteral(info.CommitHash)}};
			export const GIT_BRANCH = {{TsStringLiteral(info.Branch)}};
			export const GIT_TAG = {{TsStringLiteral(info.Tag)}};
			export const GIT_IS_DIRTY = {{(info.IsDirty ? "true" : "false")}};
			// Unix-Epoch-Sekunden (int64-tauglich) statt vorformatierter Strings -- im Browser als lokale
			// Zeit darstellen, z.B. "new Date(GIT_COMMIT_DATE_EPOCH * 1000).toLocaleString()".
			export const GIT_COMMIT_DATE_EPOCH = {{info.CommitDateEpoch}};
			export const BUILD_TIMESTAMP_EPOCH = {{info.BuildTimestampEpoch}};

			export const BOARD_NAME = {{TsStringLiteral(boardName)}};
			export const FW_VERSION_MAJOR = {{fwVersion.Major}};
			export const FW_VERSION_MINOR = {{fwVersion.Minor}};
			export const FW_VERSION_PATCH = {{fwVersion.Patch}};

			// Leer, wenn beim Generieren kein Board-Kontext bekannt war (s. docs/build-process.md Abschnitt 4).
			export const DEVICE_HOSTNAME = {{TsStringLiteral(deviceIdentity?.Hostname ?? "")}};
			export const DEVICE_CHIP_UID = {{TsStringLiteral(deviceIdentity?.ChipUid ?? "")}};
			export const DEVICE_ETH_MAC = {{TsStringLiteral(deviceIdentity?.EthMac ?? "")}};
			export const DEVICE_USB_NCM_MAC = {{TsStringLiteral(deviceIdentity?.UsbNcmMac ?? "")}};

			// TLS-Zertifikat, mit dem die Firmware das HTTPS-Web-UI ausliefert (s.
			// Phases/ReadHardwareIds.cs ReadCertificateInfo()) -- Aussteller + Gueltigkeitszeitraum
			// direkt aus dem Zertifikat gelesen, nicht separat gepflegt. 0, wenn kein Board-Kontext bekannt war.
			export const DEVICE_CERT_ISSUER = {{TsStringLiteral(deviceIdentity?.Certificate.Issuer ?? "")}};
			export const DEVICE_CERT_ISSUED_AT_EPOCH = {{deviceIdentity?.Certificate.IssuedAtEpoch ?? 0}};
			export const DEVICE_CERT_VALID_UNTIL_EPOCH = {{deviceIdentity?.Certificate.ValidUntilEpoch ?? 0}};

			""";
	}

	private static DeviceIdentity? ReadDeviceIdentity(string boardId)
	{
		var jsonPath = Path.Combine(BoardArchive.BoardGeneratedDir(boardId), "device-ids.json");
		if (!File.Exists(jsonPath)) return null;
		return JsonSerializer.Deserialize<DeviceIdentity>(File.ReadAllText(jsonPath), Json.Compact);
	}

	public static void Run(string? explicitBoardId)
	{
		var info = GitInfoReader.ReadGitInfo();
		var fwVersion = FirmwareVersion.Read();
		var boardId = BoardContext.ResolveBoardId(explicitBoardId);
		var deviceIdentity = boardId is not null ? ReadDeviceIdentity(boardId) : null;
		var boardName = deviceIdentity?.BoardName ?? BuilderOptions.Current.DefaultBoardTypeName;

		var hhContent = RenderGitConstantsHh(info);
		var firmwareConstantsContent = RenderFirmwareConstantsHh(fwVersion, boardName);
		var tsContent = RenderBuildInfoTs(info, fwVersion, boardName, deviceIdentity);
		var jsonContent = JsonSerializer.Serialize(info, Json.Options) + "\n";

		if (boardId is not null)
		{
			var dir = BoardArchive.BoardGeneratedDir(boardId);
			Directory.CreateDirectory(dir);
			File.WriteAllText(Path.Combine(dir, "gitconstants.hh"), hhContent);
			File.WriteAllText(Path.Combine(dir, "firmware_constants.hh"), firmwareConstantsContent);
			File.WriteAllText(Path.Combine(dir, "build-info.ts"), tsContent);
			File.WriteAllText(Path.Combine(dir, "gitstatus.json"), jsonContent);
			Console.WriteLine($"Git-Status ({info.CommitHash}, {info.Branch}, dirty={(info.IsDirty ? "true" : "false")}) -> Board-Archiv ({boardId}).");
			if (deviceIdentity is null)
			{
				Console.WriteLine("Warnung: Kein device-ids.json im Board-Archiv gefunden -- Hostname/Chip-UID/MAC in build-info.ts bleiben leer.");
			}
			return;
		}

		// Kein Board-Kontext bekannt (s. docs/build-process.md Abschnitt 4) -- Degradierter Pfad:
		// direkt in die vom Build konsumierten Zielordner schreiben, kein Archiv-Eintrag.
		Console.WriteLine(
			"Warnung: Kein Board-Kontext bekannt -- schreibe Git-Status direkt nach Core/generated bzw. web/generated (kein Archiv-Eintrag).");
		Directory.CreateDirectory(Paths.CoreGeneratedDir);
		Directory.CreateDirectory(Paths.WebGeneratedDir);
		File.WriteAllText(Path.Combine(Paths.CoreGeneratedDir, "gitconstants.hh"), hhContent);
		File.WriteAllText(Path.Combine(Paths.CoreGeneratedDir, "firmware_constants.hh"), firmwareConstantsContent);
		File.WriteAllText(Path.Combine(Paths.WebGeneratedDir, "build-info.ts"), tsContent);
	}
}
