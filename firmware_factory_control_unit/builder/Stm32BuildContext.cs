using FirmwareBuilder.Common;

namespace Builder;

// Implementiert IBuildContextStm32 auf Basis von AbstractBuildContext (geteilte Lib) -- ersetzt die
// vormalige Wucherung aus 8 einzelnen *Request-Records + Fabrikfunktionen in Program.cs. Board-
// Identitaets-Properties (BoardUid/ChipId/BoardArchiveDir/WebAdminPassword/BoardSettings/
// BoardTypeName/BoardTypeVersion/Hostname) lesen bei JEDEM Zugriff frisch von der Platte (kein
// gecachtes Feld), damit ein PrepareContextWithRealHardware/PrepareContextWithCommandLineArguments-
// Aufruf FRUEHER in derselben Pipeline auch fuer spaeter aufgerufene Schritte sofort sichtbar ist.
public sealed class Stm32BuildContext : AbstractBuildContext, IBuildContextStm32
{
    // --- STM32-eigene Pfade, NICHT Teil von AbstractBuildContext (andere generated/-Struktur --
    // Core/generated + web/generated statt eines gemeinsamen GeneratedRoot wie bei sensact,
    // register_map_schema, Board-Id-Cache-Datei). Diejenigen, die nur von dieser Klasse selbst
    // gebraucht werden (Backing fuer die Properties unten), sind privat; die von Program.cs direkt
    // referenzierten bleiben public static.
    private static readonly string _webGeneratedDir = Path.Combine(BuildContextPaths.WebDir(RootDirStatic), "generated");
    private static readonly string _coreGeneratedDir = Path.Combine(RootDirStatic, "Core", "generated");
    private static readonly string _buildDir = BuildContextPaths.BuildDir(RootDirStatic);

    public static readonly string RegisterMapSchemaDir = Path.Combine(RootDirStatic, "register_map_schema");

    // build/assets/ statt eines eigenen Top-Level-Ordners: vollstaendig generierter, gitignorter
    // Snapshot (Geraetezertifikat/-schluessel aus dem Board-Archiv, index.html.br aus dem
    // Web-Build), preset-unabhaengig genau wie BoardIdCacheFile -- liegt deshalb unter BuildDir,
    // nicht unter build/<preset>/ (s. CMakeLists.txt).
    public static readonly string AssetsDir = Path.Combine(_buildDir, "assets");
    public static readonly string BoardIdCacheFile = Path.Combine(_buildDir, ".last-board-id");

    public Stm32BuildContext(string[] args) : base(args)
    {
    }

    protected override IBuilderAppSettings Settings => BuilderSettings.Current;

    public override string WebGeneratedDir => _webGeneratedDir;
    public override string FirmwareGeneratedDir => _coreGeneratedDir;
    public string BuildDir => _buildDir;

    // Release ist Default, nicht Debug: SecurityPolicy#Basic256Sha256's Software-RSA-2048 (kein
    // Hardware-PKA auf diesem STM32H563) verursacht in einem unoptimierten Debug-Build 13-17s
    // ThreadX-Thread-Starvation-Stalls pro Handshake -- das Board wirkt dann fuer jeden, der es
    // testet, wie haengengeblieben. --preset Debug explizit angeben, wenn wirklich gesteppt wird.
    public string Preset
    {
        get
        {
            var preset = Cli.GetOptionalArgValue(Args, "--preset") ?? "Release";
            if (!CmakeFirmwareBuildService.IsValidPreset(preset))
            {
                throw new ArgumentException($"Unbekanntes Preset \"{preset}\". Gueltig: {string.Join(", ", CmakeFirmwareBuildService.ValidPresets)}.");
            }
            return preset;
        }
    }

    private string? ExplicitBoardId => Cli.GetOptionalArgValue(Args, "--board");

    public override string BoardUid => BoardArchiveContext.RequireBoardId(ExplicitBoardId, BoardIdCacheFile, "PrepareContextWithRealHardware");

    public override string BoardArchiveDir => BoardArchiveContext.BoardArchiveDir(BuilderSettings.Current.BoardStorage, BoardUid);

    public string ChipUid
    {
        get
        {
            var parts = BoardUid.Split('_');
            return (parts.Length > 1 ? parts[1] : BoardUid).ToUpperInvariant();
        }
    }

    // 24 Hex-Zeichen (96 Bit) -> 3x uint32, s. ChipId-Klassenkommentar (deckt sich mit
    // Stm32UidReadResult.Words, wie sie waehrend PrepareContextWithRealHardware gelesen werden).
    public override ChipId ChipId => ChipId.FromStm32Words([
        Convert.ToUInt32(ChipUid[..8], 16),
        Convert.ToUInt32(ChipUid[8..16], 16),
        Convert.ToUInt32(ChipUid[16..24], 16),
    ]);

    public IStm32ProgrammerOptions Stm32Programmer => BuilderSettings.Current.Stm32Programmer;

    public override string? WebAdminPassword => BoardStateStore.TryReadBoard(BuilderSettings.Current.BoardStorage, BoardUid)?.WebAdminPassword;

    public override IReadOnlyDictionary<string, string> BoardSettings =>
        BoardStateStore.TryReadBoard(BuilderSettings.Current.BoardStorage, BoardUid)?.BoardSettings ?? new Dictionary<string, string>();

    public override string BoardTypeName =>
        BoardStateStore.TryGetBoardTypeName(BuilderSettings.Current.BoardStorage, BoardUid)
        ?? BuilderSettings.Current.BoardDefaults.DefaultBoardTypeName;

    public override string BoardTypeVersion =>
        BoardStateStore.TryGetBoardTypeVersion(BuilderSettings.Current.BoardStorage, BoardUid)
        ?? BuilderSettings.Current.BoardDefaults.DefaultBoardTypeVersion;

    // "factory-box-{shortId}" deckt sich bewusst mit dem bereits bestehenden
    // Stm32HardwareIdentityService.BuildIdentity's Hostname-Format (dort waehrend ReadHardwareIds
    // aus den ChipUid-Woertern berechnet) -- keine zweite, abweichende Namenskonvention fuers selbe
    // Board einfuehren.
    public override string Hostname => BoardSettings.GetValueOrDefault(BoardSettingsKeys.OverrideHostname) is { Length: > 0 } name
        ? name
        : $"factory-box-{ChipUid[^6..].ToLowerInvariant()}";
}
