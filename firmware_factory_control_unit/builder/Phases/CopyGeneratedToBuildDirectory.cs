// Phase "CopyGeneratedFilesToBuildDirectory" (s. docs/build-process.md Abschnitt 2) -- reiner
// Kopiervorgang, keine eigene Generierungslogik. Einzige Phase, die Core/generated/, web/generated/
// und build/assets/ beschreibt.
// Quelle ist das Board-Archiv des zuletzt erkannten Boards (s. BoardContext).
namespace Builder.Phases;

public static class CopyGeneratedToBuildDirectory
{
	private static readonly string[] CoreFiles =
		["device_ids.hh", "gitconstants.hh", "firmware_constants.hh", "register_input.inc", "register_holding.inc", "register_maxindex.inc", "ws_protocol.hh"];
	private static readonly string[] AssetFiles = ["device_certificate.der", "device_key.der"];
	private static readonly string[] WebFiles = ["register-map.ts", "build-info.ts", "ws-protocol.ts"];

	private static void CopyIfExists(string srcDir, string destDir, string filename)
	{
		var src = Path.Combine(srcDir, filename);
		if (!File.Exists(src))
		{
			Console.WriteLine($"Uebersprungen: {filename} nicht im Board-Archiv vorhanden ({src}).");
			return;
		}
		Directory.CreateDirectory(destDir);
		var dest = Path.Combine(destDir, filename);
		File.Copy(src, dest, overwrite: true);
		Console.WriteLine($"{src} -> {dest}");
	}

	public static void Run(string? explicitBoardId)
	{
		var boardId = BoardContext.RequireBoardId(explicitBoardId);
		var src = BoardArchive.BoardGeneratedDir(boardId);
		if (!Directory.Exists(src))
		{
			throw new InvalidOperationException(
				$"Board-Archiv {src} existiert nicht -- zuerst ReadHardwareIDsAndGenerateFilesAndCertsLazy/Forced, " +
				$"ReadModbusRegisterMapAndGenerateFiles und ReadGitStatusAndGenerateFiles fuer Board {boardId} ausfuehren.");
		}

		foreach (var file in CoreFiles) CopyIfExists(src, Paths.CoreGeneratedDir, file);
		foreach (var file in AssetFiles) CopyIfExists(src, Paths.AssetsDir, file);
		foreach (var file in WebFiles) CopyIfExists(src, Paths.WebGeneratedDir, file);
	}
}
