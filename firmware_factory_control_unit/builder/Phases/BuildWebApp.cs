// Phase "BuildWebApp" (s. docs/build-process.md Abschnitt 2) -- Vite hat keine C#-API, deshalb
// ruft diese Phase stattdessen Vite's CLI-Einstiegsscript (node_modules/vite/bin/vite.js) als
// Kindprozess ueber node auf. Das bestehende
// singleFileFirmwareAssetPlugin (s. builder/vite-plugin-single-file-firmware-asset.ts, von
// web/vite.config.ts eingebunden) bleibt unveraendert und erledigt Inlining, Minifizierung UND das
// Brotli-Schreiben nach build/assets/index.html.br weiterhin in einem Rutsch -- Node/npm muessen dafuer
// weiterhin installiert sein (s. docs/build-process.md).
namespace Builder.Phases;

public static class BuildWebApp
{
	public static void Run()
	{
		var viteEntry = Path.Combine(Paths.RootDir, "node_modules", "vite", "bin", "vite.js");
		if (!File.Exists(viteEntry))
		{
			throw new InvalidOperationException(
				$"Vite nicht gefunden unter {viteEntry} -- zuerst \"npm install\" im Repo-Root ausfuehren (s. docs/build-process.md).");
		}
		var viteConfig = Path.Combine(Paths.WebDir, "vite.config.ts");

		// [root] als Vite CLI-Positionsargument statt Option, "-c"/"--config" fuer den Konfigpfad --
		// entspricht build-web-app.ts' build({ root: webDir, configFile: ... }).
		ProcessRunner.RunInherit("node", [viteEntry, "build", Paths.WebDir, "--config", viteConfig], Paths.WebDir);
		Console.WriteLine($"Web-App gebaut, {Path.Combine(Paths.AssetsDir, "index.html.br")} geschrieben.");
	}
}
