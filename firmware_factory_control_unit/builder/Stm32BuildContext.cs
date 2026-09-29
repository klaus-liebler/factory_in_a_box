using FirmwareBuilder.Common;

namespace Builder;

// Projekteigene Pfad-Konventionen auf Basis von Stm32BuildContextBase (geteilte Lib) -- alles
// chip-generische (BoardUid/ChipId/Hostname/BoardSettings/Preset/...) lebt dort, hier bleibt nur,
// was fuer firmware_factory_control_unit spezifisch ist: die generated/-Verzeichnisstruktur
// (Core/generated + web/generated statt eines gemeinsamen GeneratedRoot wie bei sensact) und die
// appsettings-Bindung.
public sealed class Stm32BuildContext : Stm32BuildContextBase
{
    private static readonly string _webGeneratedDir = Path.Combine(BuildContextPaths.WebDir(RootDirStatic), "generated");
    private static readonly string _coreGeneratedDir = Path.Combine(RootDirStatic, "Core", "generated");
    private static readonly string _buildDir = BuildContextPaths.BuildDir(RootDirStatic);

    public static readonly string RegisterMapSchemaDir = Path.Combine(RootDirStatic, "register_map_schema");

    // build/assets/ statt eines eigenen Top-Level-Ordners: vollstaendig generierter, gitignorter
    // Snapshot (Geraetezertifikat/-schluessel aus dem Board-Archiv, index.html.br aus dem
    // Web-Build), preset-unabhaengig genau wie BoardIdCacheFile -- liegt deshalb unter BuildDir,
    // nicht unter build/<preset>/ (s. CMakeLists.txt).
    public static readonly string AssetsDir = Path.Combine(_buildDir, "assets");
    public static readonly string BoardIdCacheFileStatic = Path.Combine(_buildDir, ".last-board-id");

    public Stm32BuildContext(string[] args) : base(args)
    {
    }

    protected override IBuilderAppSettingsStm32 Stm32Settings => BuilderSettings.Current;

    public override string WebGeneratedDir => _webGeneratedDir;
    public override string FirmwareGeneratedDir => _coreGeneratedDir;
    public override string BuildDir => _buildDir;
    public override string BoardIdCacheFile => BoardIdCacheFileStatic;
}
