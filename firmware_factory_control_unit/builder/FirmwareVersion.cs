// Liest firmware-version.json (Repo-Root) -- Quelle der Wahrheit fuer die Produktversion, von Hand
// gepflegt (Versions-Bumps). Pendant zu register-map.json/ReadModbusRegisterMap.cs: eine reine
// JSON-Datei statt eines C++-Headers zu parsen, damit Versions-Bumps keine Regex-Anpassung in
// FirmwareConstants.cs mehr noetig machen (das gab es hier vorher, s. Git-Historie).
using System.Text.Json;

namespace Builder;

public sealed record FirmwareVersionInfo(int Major, int Minor, int Patch);

public static class FirmwareVersion
{
	public static FirmwareVersionInfo Read()
	{
		var path = Path.Combine(Paths.RootDir, "firmware-version.json");
		using var doc = JsonDocument.Parse(File.ReadAllText(path));
		var root = doc.RootElement;
		return new FirmwareVersionInfo(
			root.GetProperty("major").GetInt32(),
			root.GetProperty("minor").GetInt32(),
			root.GetProperty("patch").GetInt32());
	}
}
