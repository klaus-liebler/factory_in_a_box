// Konfiguration ueber die .NET-Standardmechanik (Microsoft.Extensions.Configuration): appsettings.json
// gebunden an eine typsichere POCO-Klasse -- Nachfolger von EnvironmentConfig.cs (dort waren
// Pfade als freie Funktionen definiert; die zusaetzliche Flexibilitaet, die das erlaubt haette
// (beliebiger Code statt reiner Werte), wird hier nicht gebraucht). appsettings.json selbst ist
// gitignored (persoenliche Maschinenpfade), getrackt ist nur appsettings.json.template -- gleiches
// Muster wie zuvor bei EnvironmentConfig.cs/.template (s. EnsureAppSettings-Target in builder.csproj).
using Microsoft.Extensions.Configuration;

namespace Builder;

public sealed class BuilderOptions
{
	// STM32CubeProgrammer-CLI -- Rohwert aus appsettings.json; ueberschreibbar per Umgebungsvariable
	// STM32_PRG_PATH, falls STM32CubeProgrammer nicht am Standardpfad installiert ist (z.B. anderer
	// Rechner/CI). Aufloesung (Env-Var-Override + Verzeichnis-Normalisierung) s. ResolveStm32ProgrammerCli().
	public string Stm32ProgrammerCli { get; set; } = "";

	// Wiederverwendete pro-Board-Zertifikate + Board-Archiv (s. docs/build-process.md).
	public string BoardsDir { get; set; } = "";

	// Private CA (rootCA.pem.{crt,key}), die alle Board-Zertifikate signiert.
	public string CertsDir { get; set; } = "";

	public string SubjectPrefix { get; set; } = "";

	public string CertDays { get; set; } = "3000";

	// Fallback-Board-Typ-Name fuer ein Board, das noch in keinem board.json steckt (s.
	// BoardStore.TryGetBoardTypeName) -- wird gleichzeitig als Anzeigename in Firmware und Web-UI
	// verwendet (s. Phases/ReadGitStatus.cs).
	public string DefaultBoardTypeName { get; set; } = "FactoryControlUnit";

	public string CaCert => Path.Combine(CertsDir, "rootCA.pem.crt");

	public string CaKey => Path.Combine(CertsDir, "rootCA.pem.key");

	// STM32_PRG_PATH darf sowohl auf die .exe selbst als auch nur auf deren bin/-Ordner zeigen --
	// ein Ordner wird hier automatisch um den .exe-Namen ergaenzt, statt weiter unten mit einem
	// kryptischen ENOENT beim Spawnen eines Verzeichnisses zu scheitern.
	public string ResolveStm32ProgrammerCli()
	{
		var configured = Environment.GetEnvironmentVariable("STM32_PRG_PATH");
		if (string.IsNullOrEmpty(configured))
		{
			configured = Stm32ProgrammerCli;
		}
		if (Directory.Exists(configured))
		{
			return Path.Combine(configured, "STM32_Programmer_CLI.exe");
		}
		return configured;
	}

	private static readonly Lazy<BuilderOptions> LazyCurrent = new(Load);

	public static BuilderOptions Current => LazyCurrent.Value;

	private static BuilderOptions Load()
	{
		var config = new ConfigurationBuilder()
			.SetBasePath(AppContext.BaseDirectory)
			.AddJsonFile("appsettings.json", optional: false)
			.Build();
		return config.Get<BuilderOptions>()
			?? throw new InvalidOperationException("appsettings.json konnte nicht gebunden werden.");
	}
}
