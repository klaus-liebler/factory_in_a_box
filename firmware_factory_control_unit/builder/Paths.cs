// Zentrale Pfad-Konstanten fuer alle Phasen.
namespace Builder;

public static class Paths
{
	// builder/ -> Repo-Wurzel (firmware_factory_control_unit/). AppContext.BaseDirectory zeigt zur
	// Laufzeit auf den Build-Output-Ordner (bin/<Config>/<TFM>/...), dessen Tiefe je nach Aufrufart
	// variiert (dotnet run/build vs. dotnet publish, ggf. mit RID-Unterordner) -- deshalb robust
	// nach oben bis zum CMakeLists.txt der Repo-Wurzel suchen statt eine feste Anzahl
	// ".."-Sprünge anzunehmen.
	public static readonly string RootDir = FindRootDir();

	private static string FindRootDir()
	{
		var dir = new DirectoryInfo(AppContext.BaseDirectory);
		while (dir is not null)
		{
			if (File.Exists(Path.Combine(dir.FullName, "CMakeLists.txt")) && Directory.Exists(Path.Combine(dir.FullName, "builder")))
			{
				return dir.FullName;
			}
			dir = dir.Parent;
		}
		throw new InvalidOperationException(
			$"Konnte Repo-Wurzel nicht finden (gesucht ab {AppContext.BaseDirectory} nach oben, Marker: CMakeLists.txt + builder/).");
	}

	public static readonly string CoreGeneratedDir = Path.Combine(RootDir, "Core", "generated");
	public static readonly string BestBinaryBuffersSchemaDir = Path.Combine(RootDir, "best_binary_buffers_schema");
	public static readonly string RegisterMapSchemaDir = Path.Combine(RootDir, "register_map_schema");
	public static readonly string WebDir = Path.Combine(RootDir, "web");
	public static readonly string WebGeneratedDir = Path.Combine(WebDir, "generated");
	public static readonly string BuildDir = Path.Combine(RootDir, "build");
	// build/assets/ statt eines eigenen Top-Level-Ordners: vollstaendig generierter, gitignorter
	// Snapshot (Geraetezertifikat/-schluessel aus dem Board-Archiv, index.html.br aus dem
	// Web-Build), preset-unabhaengig genau wie BoardIdCacheFile -- liegt deshalb unter BuildDir,
	// nicht unter build/<preset>/ (s. CMakeLists.txt).
	public static readonly string AssetsDir = Path.Combine(BuildDir, "assets");
	public static readonly string BoardIdCacheFile = Path.Combine(BuildDir, ".last-board-id");
}
