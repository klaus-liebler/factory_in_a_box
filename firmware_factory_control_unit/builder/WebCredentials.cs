using System.Globalization;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using FirmwareBuilder.Common;

namespace Builder;

// The web UI's one-time password of a board (docs/security.md "Provisioning"):
//
//   * it lives in the board archive next to the board's other data (board_info.json,
//     "webAdminPassword", keyed by the chip UID like everything there) and is reused on every later
//     build - the label stays valid. Only WebAuth:ForceRecreateOneTimeWebUiPassword in appsettings.json
//     or the flag --force-recreate-one-time-web-ui-password makes a new one (then print a new label);
//   * the firmware gets its PBKDF2-HMAC-SHA256 record only (generated/device_credentials.hh: user,
//     iterations, salt, hash). The salt is derived from chip UID and password, so the record - and
//     with it the firmware's provisioning epoch - changes exactly when the password changes;
//   * every creation goes into the provisioning record (stm32_boards/provisioning.csv: board ID,
//     hostname, revision, date, event - never the password).
public static class WebCredentials
{
    public const string ForceFlag = "--force-recreate-one-time-web-ui-password";

    // no 0/O, 1/l/I: read off a label
    private const string Alphabet = "ABCDEFGHJKLMNPQRSTUVWXYZabcdefghjkmnpqrstuvwxyz23456789";

    public sealed record Device(string BoardId, string ChipUid, string Hostname, string EthMac, string Revision, string Password,
                                DateTimeOffset ProvisionedAt);

    public static Device Generate(IBuildContextStm32 ctx)
    {
        var settings = BuilderSettings.Current.WebAuth;
        var ids = JsonDocument.Parse(File.ReadAllText(Path.Combine(ctx.FirmwareGeneratedDir, "device-ids.json"))).RootElement;
        var chipUid = ids.GetProperty("chipUid").GetString()!;
        var hostname = ids.GetProperty("hostname").GetString()!;
        var ethMac = ids.GetProperty("ethMac").GetString()!;

        var path = BoardStateStore.BoardJsonPath(BuilderSettings.Current.BoardStorage, ctx.BoardUid);  // the archive's record
        var board = JsonSerializer.Deserialize<BoardRecord>(File.ReadAllText(path), JsonDefaults.Compact)
            ?? throw new InvalidOperationException($"{path} konnte nicht gelesen werden.");
        var force = settings.ForceRecreateOneTimeWebUiPassword || Cli.HasFlag(ctx.Args, ForceFlag);
        var password = board.WebAdminPassword;
        string? evt = null;
        if (string.IsNullOrEmpty(password) || force)
        {
            evt = string.IsNullOrEmpty(password) ? "created" : "recreated";
            password = NewPassword(settings.PasswordLength);
            var updated = board with { WebAdminPassword = password };
            File.WriteAllText(path, JsonSerializer.Serialize(updated, JsonDefaults.Pretty) + "\n");
            Console.WriteLine($"Web-UI-Einmalpasswort fuer {hostname} {(evt == "created" ? "angelegt" : "NEU erzeugt")} " +
                              $"(gespeichert in {path}) -- neues Etikett drucken.");
        }
        else
        {
            Console.WriteLine($"Web-UI-Einmalpasswort fuer {hostname} aus dem Board-Archiv uebernommen.");
        }

        var revision = $"{ctx.Preset} ({ctx.BoardTypeVersion})";
        var device = new Device(ctx.BoardUid, chipUid, hostname, ethMac, revision, password,
                                DateTimeOffset.FromUnixTimeSeconds(board.FirstConnectedAtEpoch));
        WriteHeader(ctx, settings, chipUid, password);
        if (evt is not null)
        {
            AppendProvisioningRecord(ctx, device, evt);
        }
        return device;
    }

    public static string NewPassword(int length)
    {
        var chars = RandomNumberGenerator.GetItems<char>(Alphabet, length);
        // groups of four: easier to type from the label
        var sb = new StringBuilder();
        for (var i = 0; i < chars.Length; ++i)
        {
            if (i > 0 && i % 4 == 0) sb.Append('-');
            sb.Append(chars[i]);
        }
        return sb.ToString();
    }

    public static byte[] Salt(string chipUid, string password) =>
        SHA256.HashData(Encoding.UTF8.GetBytes($"fcu-web-salt\0{chipUid}\0{password}"))[..16];

    public static byte[] Hash(string password, byte[] salt, int iterations) =>
        Rfc2898DeriveBytes.Pbkdf2(Encoding.UTF8.GetBytes(password), salt, iterations, HashAlgorithmName.SHA256, 32);

    private static void WriteHeader(IBuildContextStm32 ctx, WebAuthOptions settings, string chipUid, string password)
    {
        var salt = Salt(chipUid, password);
        var hash = Hash(password, salt, settings.Pbkdf2Iterations);
        static string Bytes(byte[] b) => string.Join(", ", b.Select(x => $"0x{x:X2}"));
        var text = $$"""
            #pragma once
            // Generiert von builder/WebCredentials.cs -- nicht von Hand editieren, nicht einchecken.
            // Das Einmalpasswort der Web-UI dieses Boards als PBKDF2-HMAC-SHA256-Datensatz (src/auth.hpp); das
            // Passwort selbst steht nur im Board-Archiv (board_info.json) und auf dem Etikett.
            #include <cstdint>
            inline constexpr char DEVICE_WEB_ADMIN_USER[] = "{{settings.AdminUser}}";
            inline constexpr std::uint32_t DEVICE_WEB_PBKDF2_ITERATIONS = {{settings.Pbkdf2Iterations.ToString(CultureInfo.InvariantCulture)}};
            inline constexpr std::uint8_t DEVICE_WEB_PASSWORD_SALT[16] = {{{Bytes(salt)}}};
            inline constexpr std::uint8_t DEVICE_WEB_PASSWORD_HASH[32] = {{{Bytes(hash)}}};

            """;
        var file = Path.Combine(ctx.FirmwareGeneratedDir, "device_credentials.hh");
        if (!File.Exists(file) || File.ReadAllText(file) != text)
        {
            File.WriteAllText(file, text);  // unchanged: keep the time stamp (no rebuild)
        }
    }

    private static void AppendProvisioningRecord(IBuildContextStm32 ctx, Device d, string evt)
    {
        var csv = Path.Combine(ctx.BoardsDir, "provisioning.csv");
        if (!File.Exists(csv))
        {
            File.WriteAllText(csv, "board_id;chip_uid;hostname;eth_mac;revision;date_utc;event\n");
        }
        File.AppendAllText(csv, string.Join(';', d.BoardId, d.ChipUid, d.Hostname, d.EthMac, d.Revision,
                                            DateTimeOffset.UtcNow.ToString("yyyy-MM-ddTHH:mm:ssZ", CultureInfo.InvariantCulture),
                                            $"web password {evt}") + "\n");
    }
}
