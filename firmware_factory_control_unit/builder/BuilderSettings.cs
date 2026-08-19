using Microsoft.Extensions.Configuration;
using FirmwareBuilder.Common;

namespace Builder;

public sealed class BoardStorageOptions : IBoardsDirectoryOptions
{
    public string BoardsDir { get; set; } = "";
}

public sealed class Stm32ProgrammerOptions : IStm32ProgrammerOptions
{
    public string Stm32ProgrammerCli { get; set; } = "";

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
}

public sealed class CertificateAuthorityOptions : ICertificateAuthorityOptions
{
    public string CertsDir { get; set; } = "";
    public string SubjectPrefix { get; set; } = "";
    public string CertDays { get; set; } = "3000";
    public string KeyAlgorithm { get; set; } = "RSA_2048";
    public bool IncludeServerAuthEku { get; set; } = true;
    public bool IncludeClientAuthEku { get; set; } = true;
    public bool IncludeSubjectKeyIdentifier { get; set; } = false;
    public bool IncludeAuthorityKeyIdentifier { get; set; } = true;
    public int NotBeforeBackdateDays { get; set; } = 1;
    public string? SanIpAddress { get; set; } = "192.168.173.1";
    public List<string> SanDnsEntries { get; set; } = ["{hostname}", "{hostname}.local", "{hostname}.fritz.box"];

    public string CaCertPath => Path.Combine(CertsDir, "rootCA.pem.crt");
    public string CaKeyPath => Path.Combine(CertsDir, "rootCA.pem.key");

    IReadOnlyList<string> ICertificateAuthorityOptions.SanDnsEntries => SanDnsEntries;
}

public sealed class BoardDefaultsOptions
{
    public string DefaultBoardTypeName { get; set; } = "FactoryControlUnit";
}

public sealed class BuilderSettings
{
    public BoardStorageOptions BoardStorage { get; set; } = new();
    public Stm32ProgrammerOptions Stm32Programmer { get; set; } = new();
    public CertificateAuthorityOptions Certificates { get; set; } = new();
    public BoardDefaultsOptions BoardDefaults { get; set; } = new();

    private static readonly Lazy<BuilderSettings> LazyCurrent = new(Load);

    public static BuilderSettings Current => LazyCurrent.Value;

    private static BuilderSettings Load()
    {
        var config = new ConfigurationBuilder()
            .SetBasePath(AppContext.BaseDirectory)
            .AddJsonFile("appsettings.json", optional: false)
            .Build();

        return config.Get<BuilderSettings>()
            ?? throw new InvalidOperationException("appsettings.json konnte nicht gebunden werden.");
    }
}
