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
    /// Login mode per network interface (docs/security.md "Login mode per interface"): "Ethernet" and
    /// "UsbNcm", each "OneTimePassword" (login needed: the label's one-time password, then the
    /// accounts; the default) or "Open" (Programmer without login; accounts and terminal still need
    /// an admin login; USB-NCM still counts as physical presence). Missing keys stay OneTimePassword.
    public Dictionary<string, string> Modes { get; set; } = new(StringComparer.OrdinalIgnoreCase);

    public static readonly string[] Interfaces = ["Ethernet", "UsbNcm"];

    /// The interface is open (validated: unknown interfaces or modes throw).
    public bool IsOpen(string iface)
    {
        foreach (var key in Modes.Keys)
        {
            if (!Interfaces.Contains(key, StringComparer.OrdinalIgnoreCase))
            {
                throw new ArgumentException($"WebAuth:Modes: unbekanntes Interface \"{key}\" (bekannt: {string.Join(", ", Interfaces)}).");
            }
        }
        var mode = Modes.FirstOrDefault(kv => string.Equals(kv.Key, iface, StringComparison.OrdinalIgnoreCase)).Value?.Trim();
        return mode switch
        {
            null or "" => false,
            _ when string.Equals(mode, "OneTimePassword", StringComparison.OrdinalIgnoreCase) => false,
            _ when string.Equals(mode, "Open", StringComparison.OrdinalIgnoreCase) => true,
            _ => throw new ArgumentException($"WebAuth:Modes:{iface}: \"{mode}\" unbekannt (OneTimePassword, Open)."),
        };
    }
}

// The label of each provisioned board (docs/labels.md). Type selects the printer:
//   Zebra (or empty): ZPL (51 x 25 mm default) raw to Host:Port, or the PNG through WindowsPrinter
//   BrotherQL:        Brother QL raster (QL-700, 62 mm endless etc., see BrotherQlOptions)
//   None:             files only
// Files go to the board archive, <BoardsDir>/<board id>/labels/ (PrintLabelSheet: <BoardsDir>/labels_A4.pdf).
// DryRun (or --dry-run) only writes the print job there and sends nothing.
public sealed class LabelPrinterOptions
{
    public string? Type { get; set; }
    public bool DryRun { get; set; }
    /// Every printed label additionally as an A4 sheet at true scale (<BoardsDir>/<board id>/labels/<hostname>.A4.pdf).
    public bool A4Sheet { get; set; }
    /// Cell of the A4 sheet (PrintLabelSheet): 62 mm wide like the QL tape.
    public double SheetCellWidthMm { get; set; } = 62;
    public double SheetCellHeightMm { get; set; } = 40;
    public BrotherQlOptions BrotherQL { get; set; } = new();
    public string? Host { get; set; }
    /// Instead of ZPL over the network: the PNG through this Windows printer driver (printers without ZPL).
    public string? WindowsPrinter { get; set; }
    public int Port { get; set; } = 9100;
    public double WidthMm { get; set; } = 51;
    public double HeightMm { get; set; } = 25;
    public int Dpi { get; set; } = 300;
}

// Brother QL label printer (LabelPrinter:Type = BrotherQL), docs/labels.md.
public sealed class BrotherQlOptions
{
    /// QL-700 (USB, no compression, no mode switch); QL-710W/720NW/800/810W/820NWB work too.
    public string Model { get; set; } = "QL-700";
    /// "62" = DK-22205 62 mm endless (auto length, cut), "29x90" = DK-11201, "17x54" = DK-11204.
    public string Label { get; set; } = "62";
    /// WindowsRaw:    the raster job through the Windows queue of the Brother driver, datatype RAW (no Zadig)
    /// WindowsDriver: the PNG through the Brother driver (System.Drawing), driver does the raster
    /// Tcp:           raster job raw to Target "host[:port]" (network QL or a print server, port 9100)
    /// BrotherQlCli:  the PNG via the python brother_ql CLI, Target = its printer identifier
    ///                (usb://0x04f9:0x2042, tcp://host:9100, file:///dev/usb/lp0)
    /// File:          only write the raster job (to Target, if set)
    public string Backend { get; set; } = "WindowsRaw";
    /// Queue name (WindowsRaw/WindowsDriver, default "Brother QL-700"), host[:port] or identifier.
    public string? Target { get; set; }
    public string BrotherQlCommand { get; set; } = "brother_ql";
    /// pyusb | network | linux_kernel (passed as -b to brother_ql; empty: guessed from Target)
    public string? BrotherQlBackend { get; set; }
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
