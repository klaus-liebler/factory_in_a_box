// Einstiegspunkt fuer alle Build-Phasen (s. docs/build-process.md Abschnitt 2).
//
// Aufruf:
//   dotnet run --project builder -- <Phase> [--preset Debug|Release|Debug-Nucleo] [--board <boardId>]
//     [--ws-protocol-path <Verzeichnis-oder-Datei>]...
//
// --ws-protocol-path ist wiederholbar (nur fuer ReadWebSocketProtocolAndGenerateFiles/die
// Pipelines relevant) und fasst die *.json-Namespace-Dateien aus mehreren Verzeichnissen/
// Einzeldateien zu EINEM ws_protocol.hh/ws-protocol.ts zusammen (s. docs/websocket-protocol.md,
// ReadWebSocketProtocol.Run()). Ohne Angabe: nur "ws-protocol/" im Repo-Root (bisheriges
// Alleinverhalten).
//
// Einzelne Phasen (Reihenfolge s. Abhaengigkeitsgraph in der Doku):
//   ReadHardwareIDsAndGenerateFilesAndCertsLazy
//   ReadHardwareIDsAndGenerateFilesAndCertsForced
//   ReadModbusRegisterMapAndGenerateFiles
//   ReadWebSocketProtocolAndGenerateFiles
//   ReadGitStatusAndGenerateFiles
//   CopyGeneratedFilesToBuildDirectory
//   BuildWebApp
//   BuildFirmware
//   FlashFirmware
//
// Zusammengesetzte Pipelines:
//   pipeline:lazy    -- Lazy  -> RegisterMap -> WsProtocol -> GitStatus -> Copy -> BuildWebApp -> BuildFirmware
//   pipeline:forced  -- Forced -> RegisterMap -> WsProtocol -> GitStatus -> Copy -> BuildWebApp -> BuildFirmware
//   pipeline:flash   -- pipeline:lazy, danach FlashFirmware
using Builder;
using Builder.Phases;

static Args ParseArgs(string[] argv)
{
	if (argv.Length == 0)
	{
		throw new ArgumentException("Kein Phasenname angegeben. Beispiel: dotnet run --project builder -- ReadModbusRegisterMapAndGenerateFiles");
	}
	var phase = argv[0];

	var preset = "Debug";
	string? board = null;
	var wsProtocolPaths = new List<string>();
	for (var i = 1; i < argv.Length; i += 1)
	{
		if (argv[i] == "--preset" && i + 1 < argv.Length)
		{
			preset = argv[i + 1];
			i += 1;
		}
		else if (argv[i] == "--board" && i + 1 < argv.Length)
		{
			board = argv[i + 1];
			i += 1;
		}
		else if (argv[i] == "--ws-protocol-path" && i + 1 < argv.Length)
		{
			wsProtocolPaths.Add(argv[i + 1]);
			i += 1;
		}
	}

	if (!BuildFirmware.IsValidPreset(preset))
	{
		throw new ArgumentException($"Unbekanntes Preset \"{preset}\". Gueltig: Debug, Release, Debug-Nucleo.");
	}

	return new Args(phase, preset, board, wsProtocolPaths);
}

static void RunPhase(string phase, string preset, string? board, IReadOnlyList<string> wsProtocolPaths)
{
	switch (phase)
	{
		case "ReadHardwareIDsAndGenerateFilesAndCertsLazy":
			ReadHardwareIds.Run(forced: false);
			return;
		case "ReadHardwareIDsAndGenerateFilesAndCertsForced":
			ReadHardwareIds.Run(forced: true);
			return;
		case "ReadModbusRegisterMapAndGenerateFiles":
			ReadModbusRegisterMap.Run(board);
			return;
		case "ReadWebSocketProtocolAndGenerateFiles":
			ReadWebSocketProtocol.Run(board, wsProtocolPaths);
			return;
		case "ReadGitStatusAndGenerateFiles":
			ReadGitStatus.Run(board);
			return;
		case "CopyGeneratedFilesToBuildDirectory":
			CopyGeneratedToBuildDirectory.Run(board);
			return;
		case "BuildWebApp":
			BuildWebApp.Run();
			return;
		case "BuildFirmware":
			BuildFirmware.Run(preset);
			return;
		case "FlashFirmware":
			FlashFirmware.Run(preset, board);
			return;
		case "pipeline:lazy":
			ReadHardwareIds.Run(forced: false);
			ReadModbusRegisterMap.Run(board);
			ReadWebSocketProtocol.Run(board, wsProtocolPaths);
			ReadGitStatus.Run(board);
			CopyGeneratedToBuildDirectory.Run(board);
			BuildWebApp.Run();
			BuildFirmware.Run(preset);
			return;
		case "pipeline:forced":
			ReadHardwareIds.Run(forced: true);
			ReadModbusRegisterMap.Run(board);
			ReadWebSocketProtocol.Run(board, wsProtocolPaths);
			ReadGitStatus.Run(board);
			CopyGeneratedToBuildDirectory.Run(board);
			BuildWebApp.Run();
			BuildFirmware.Run(preset);
			return;
		case "pipeline:flash":
			ReadHardwareIds.Run(forced: false);
			ReadModbusRegisterMap.Run(board);
			ReadWebSocketProtocol.Run(board, wsProtocolPaths);
			ReadGitStatus.Run(board);
			CopyGeneratedToBuildDirectory.Run(board);
			BuildWebApp.Run();
			BuildFirmware.Run(preset);
			FlashFirmware.Run(preset, board);
			return;
		default:
			throw new ArgumentException($"Unbekannte Phase \"{phase}\".");
	}
}

try
{
	var (phase, preset, board, wsProtocolPaths) = ParseArgs(args);
	RunPhase(phase, preset, board, wsProtocolPaths);
}
catch (Exception err)
{
	Console.Error.WriteLine($"\nFehler: {err.Message}");
	Environment.Exit(1);
}

sealed record Args(string Phase, string Preset, string? Board, List<string> WsProtocolPaths);
