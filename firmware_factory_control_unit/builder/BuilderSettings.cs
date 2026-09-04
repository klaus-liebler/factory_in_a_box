using FirmwareBuilder.Common;

namespace Builder;

// BoardStorageOptions/Stm32ProgrammerOptions/CertificateAuthorityOptions sind zentralisiert in
// FirmwareBuilder.Common (BoardsDirectoryOptions/Stm32ProgrammerOptions/CertificateAuthorityOptions) --
// nur noch BoardDefaultsOptions ist projekt-lokal (STM32-spezifische Board-Typ-Defaults).
public sealed class BoardDefaultsOptions
{
    public string DefaultBoardTypeName { get; set; } = "FactoryControlUnit";

    // Es gibt (noch) keine echte SemVer-Versionierung der Board-Hardware -- Platzhalter, bis eine
    // Hardware-Revision das tatsaechlich braucht (s. IBuildContext.BoardTypeVersion).
    public string DefaultBoardTypeVersion { get; set; } = "1.0.0";
}

public sealed class BuilderSettings : IBuilderAppSettings
{
    public BoardsDirectoryOptions BoardStorage { get; set; } = new();
    public Stm32ProgrammerOptions Stm32Programmer { get; set; } = new();
    public CertificateAuthorityOptions Certificates { get; set; } = new();
    public BoardDefaultsOptions BoardDefaults { get; set; } = new();

    IBoardsDirectoryOptions IBuilderAppSettings.BoardStorage => BoardStorage;
    ICertificateAuthorityOptions IBuilderAppSettings.Certificates => Certificates;

    private static readonly Lazy<BuilderSettings> LazyCurrent = new(Load);

    public static BuilderSettings Current => LazyCurrent.Value;

    private static BuilderSettings Load() => BuilderAppSettings.LoadFromAppBase<BuilderSettings>();
}
