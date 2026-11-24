using FirmwareBuilder.Common;

namespace Builder;

// BoardStorageOptions/Stm32ProgrammerOptions/CertificateAuthorityOptions sind zentralisiert in
// FirmwareBuilder.Common (BoardsDirectoryOptions/Stm32ProgrammerOptions/CertificateAuthorityOptions) --
// nur noch BoardDefaultsOptions ist projekt-lokal (STM32-spezifische Board-Typ-Defaults).
public sealed class BoardDefaultsOptions : IBoardDefaultsOptions
{
    public string DefaultBoardTypeName { get; set; } = "FactoryControlUnit";

    // Es gibt (noch) keine echte SemVer-Versionierung der Board-Hardware -- Platzhalter, bis eine
    // Hardware-Revision das tatsaechlich braucht (s. IBuildContext.BoardTypeVersion).
    public string DefaultBoardTypeVersion { get; set; } = "1.0.0";
}

// Web UI login (docs/security.md): the per-device one-time password lives in the board archive
// (board_info.json, webAdminPassword) and is reused on every later build; only
// ForceRecreateOneTimeWebUiPassword (or the CLI flag --force-recreate-one-time-web-ui-password)
// makes a new one. The firmware gets its PBKDF2-HMAC-SHA256 record only.
public sealed class WebAuthOptions
{
    public bool ForceRecreateOneTimeWebUiPassword { get; set; } = false;
    /// About 170 ms on the STM32H563 at 250 MHz (rtoos authbench: 7000 instructions per iteration).
    public int Pbkdf2Iterations { get; set; } = 6000;
    public int PasswordLength { get; set; } = 16;
    public string AdminUser { get; set; } = "admin";
}

// The label of each provisioned board (ZPL for a Zebra-type printer, PNG/PDF preview). Without Host
// only the files are written; with Host the ZPL goes raw to Host:Port.
public sealed class LabelPrinterOptions
{
    public string? Host { get; set; }
    /// Instead of ZPL over the network: the PNG through this Windows printer driver (printers without ZPL).
    public string? WindowsPrinter { get; set; }
    public int Port { get; set; } = 9100;
    public double WidthMm { get; set; } = 51;
    public double HeightMm { get; set; } = 25;
    public int Dpi { get; set; } = 300;
}

public sealed class BuilderSettings : IBuilderAppSettingsStm32
{
    public BoardsDirectoryOptions BoardStorage { get; set; } = new();
    public Stm32ProgrammerOptions Stm32Programmer { get; set; } = new();
    public CertificateAuthorityOptions Certificates { get; set; } = new();
    public BoardDefaultsOptions BoardDefaults { get; set; } = new();
    public WebAuthOptions WebAuth { get; set; } = new();
    public LabelPrinterOptions LabelPrinter { get; set; } = new();

    IBoardsDirectoryOptions IBuilderAppSettings.BoardStorage => BoardStorage;
    ICertificateAuthorityOptions IBuilderAppSettings.Certificates => Certificates;
    IStm32ProgrammerOptions IBuilderAppSettingsStm32.Stm32Programmer => Stm32Programmer;
    IBoardDefaultsOptions IBuilderAppSettingsStm32.BoardDefaults => BoardDefaults;

    private static readonly Lazy<BuilderSettings> LazyCurrent = new(Load);

    public static BuilderSettings Current => LazyCurrent.Value;

    private static BuilderSettings Load() => BuilderAppSettings.LoadFromAppBase<BuilderSettings>();
}
