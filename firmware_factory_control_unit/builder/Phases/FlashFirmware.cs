// Phase "FlashFirmware" (s. docs/build-process.md Abschnitt 2) -- ruft STM32_Programmer_CLI direkt
// auf. "-w <elf> -v -rst": schreiben mit Verify, danach Reset+Start. Nach einem verifiziert
// erfolgreichen Flash wird board.json/flash_events.jsonl im Board-Archiv geschrieben (Abschnitt 5)
// -- ein fehlgeschlagener Flash erzeugt bewusst KEINEN Eintrag.
using System.Text.Json;
using System.Text.RegularExpressions;

namespace Builder.Phases;

public static class FlashFirmware
{
	private static string? DetectStlinkProbeSerial()
	{
		try
		{
			var output = ProcessRunner.Run(BuilderOptions.Current.ResolveStm32ProgrammerCli(), ["-l"]);
			var match = Regex.Match(output, @"ST-LINK SN\s*:\s*(\S+)");
			return match.Success ? match.Groups[1].Value : null;
		}
		catch
		{
			// Bewusst nicht fatal -- die Sondenseriennummer ist nur eine Zusatzinformation im
			// DB-Eintrag, kein Grund, den Flash-Vorgang selbst abzubrechen.
			return null;
		}
	}

	private static GitInfo? ReadArchivedGitInfo(string boardId)
	{
		var jsonPath = Path.Combine(BoardArchive.BoardGeneratedDir(boardId), "gitstatus.json");
		if (!File.Exists(jsonPath)) return null;
		return JsonSerializer.Deserialize<GitInfo>(File.ReadAllText(jsonPath), Json.Compact);
	}

	// BoardName kommt aus demselben device-ids.json, das ReadHardwareIds bereits per
	// Board-Archiv-Nachschlag (board.json, bzw. Default) aufgeloest hat -- FlashFirmware leitet
	// hier bewusst NICHT mehr selbst aus dem Preset ab (s. BoardStore.RecordSuccessfulFlash).
	private static DeviceIdentity? ReadArchivedDeviceIdentity(string boardId)
	{
		var jsonPath = Path.Combine(BoardArchive.BoardGeneratedDir(boardId), "device-ids.json");
		if (!File.Exists(jsonPath)) return null;
		return JsonSerializer.Deserialize<DeviceIdentity>(File.ReadAllText(jsonPath), Json.Compact);
	}

	// boardId hat die Form "<shortId>_<volle96-Bit-UID-klein>" (s. BoardContext/ReadHardwareIds).
	private static (string ChipUid, string Hostname) ParseBoardId(string boardId)
	{
		var parts = boardId.Split('_');
		var shortId = parts[0];
		var fullUid = parts.Length > 1 ? parts[1] : boardId;
		return (fullUid.ToUpperInvariant(), $"factory-box-{shortId}");
	}

	public static void Run(string preset, string? explicitBoardId)
	{
		var elfPath = Path.Combine(Paths.RootDir, "build", preset, "firmware_factory_control_unit.elf");
		if (!File.Exists(elfPath))
		{
			throw new InvalidOperationException($"Firmware-Elf nicht gefunden: {elfPath} -- zuerst BuildFirmware fuer Preset \"{preset}\" ausfuehren.");
		}

		Console.WriteLine($"Flashe {elfPath} ueber STM32_Programmer_CLI...");
		ProcessRunner.RunInherit(BuilderOptions.Current.ResolveStm32ProgrammerCli(), ["-c", "port=SWD", "-w", elfPath, "-v", "-rst"]);
		Console.WriteLine("Flash erfolgreich verifiziert.");

		var boardId = BoardContext.RequireBoardId(explicitBoardId);
		var (chipUid, hostname) = ParseBoardId(boardId);
		var gitInfo = ReadArchivedGitInfo(boardId);
		if (gitInfo is null)
		{
			Console.WriteLine(
				"Warnung: Kein gitstatus.json im Board-Archiv gefunden (ReadGitStatusAndGenerateFiles nicht gelaufen?) -- " +
				"kein board.json/flash_events.jsonl-Eintrag.");
			return;
		}
		var deviceIdentity = ReadArchivedDeviceIdentity(boardId);
		var boardName = deviceIdentity?.BoardName ?? BuilderOptions.Current.DefaultBoardTypeName;

		BoardStore.RecordSuccessfulFlash(new BoardStore.RecordFlashParams(
			BoardId: boardId,
			ChipUid: chipUid,
			Hostname: hostname,
			StlinkProbe: DetectStlinkProbeSerial(),
			BoardTypeName: boardName,
			CmakePreset: preset,
			GitCommitHash: gitInfo.CommitHash,
			GitBranch: gitInfo.Branch,
			GitIsDirty: gitInfo.IsDirty,
			FirmwareVersion: gitInfo.Version));
	}
}
