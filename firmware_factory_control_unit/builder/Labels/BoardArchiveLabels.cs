using System.Text.Json;

namespace Builder.Labels;

// Label data of archived boards, read only from the board archive (<BoardsDir>/<board id>/):
// board_info.json (webAdminPassword, firstConnectedAtEpoch, boardTypeName/-Version) and
// generated/hardware-identity.json (hostname, chip UID, Ethernet MAC). Never creates or changes a
// password: a board without one is skipped (GenerateWebCredentials creates it).
public static class BoardArchiveLabels
{
    public static IReadOnlyList<string> AllBoardIds(string boardsDir) =>
        Directory.Exists(boardsDir)
            ? Directory.GetDirectories(boardsDir)
                .Where(d => File.Exists(Path.Combine(d, "board_info.json")))
                .Select(Path.GetFileName)
                .OfType<string>()
                .Order(StringComparer.Ordinal)
                .ToList()
            : [];

    /// A board ID, or a unique prefix of one ("303537", or the hostname "factory-box-303537").
    public static string Resolve(string boardsDir, string idOrPrefix)
    {
        var key = idOrPrefix.StartsWith("factory-box-", StringComparison.OrdinalIgnoreCase) ? idOrPrefix[12..] : idOrPrefix;
        var all = AllBoardIds(boardsDir);
        var exact = all.FirstOrDefault(b => string.Equals(b, key, StringComparison.OrdinalIgnoreCase));
        if (exact is not null)
        {
            return exact;
        }
        var matches = all.Where(b => b.StartsWith(key, StringComparison.OrdinalIgnoreCase)).ToList();
        return matches.Count switch
        {
            1 => matches[0],
            0 => throw new ArgumentException($"Kein Board \"{idOrPrefix}\" im Board-Archiv {boardsDir}."),
            _ => throw new ArgumentException($"\"{idOrPrefix}\" ist nicht eindeutig: {string.Join(", ", matches)}."),
        };
    }

    public static WebCredentials.Device? TryLoad(string boardsDir, string boardId, string defaultBoardTypeName,
                                                 string defaultBoardTypeVersion, out string? whyNot)
    {
        var dir = Path.Combine(boardsDir, boardId);
        var infoPath = Path.Combine(dir, "board_info.json");
        if (!File.Exists(infoPath))
        {
            whyNot = "keine board_info.json";
            return null;
        }
        using var info = JsonDocument.Parse(File.ReadAllText(infoPath));
        var root = info.RootElement;
        var password = Str(root, "webAdminPassword");
        if (string.IsNullOrEmpty(password))
        {
            whyNot = "kein Web-Passwort im Board-Archiv (zuerst GenerateWebCredentials bzw. eine Pipeline fuer dieses Board)";
            return null;
        }
        string? hostname = null, chipUid = null, ethMac = null;
        var hwPath = Path.Combine(dir, "generated", "hardware-identity.json");
        if (File.Exists(hwPath))
        {
            using var hw = JsonDocument.Parse(File.ReadAllText(hwPath));
            hostname = Str(hw.RootElement, "hostname");
            chipUid = Str(hw.RootElement, "chipUid");
            if (hw.RootElement.TryGetProperty("ethMacBytes", out var mac) && mac.ValueKind == JsonValueKind.String)
            {
                ethMac = string.Join(":", mac.GetBytesFromBase64().Select(b => b.ToString("X2")));
            }
        }
        chipUid ??= Str(root, "chipUid") ?? (boardId.Contains('_') ? boardId.Split('_')[1] : boardId).ToUpperInvariant();
        hostname ??= Str(root, "hostname") ?? $"factory-box-{chipUid[^6..].ToLowerInvariant()}";

        var typeName = Str(root, "boardTypeName") ?? defaultBoardTypeName;
        var version = Str(root, "boardTypeVersion") ?? defaultBoardTypeVersion;
        // as the build names it (Stm32BuildContext.DefaultPreset): FactoryControlUnit 1.x.y -> rev1
        var revision = typeName == defaultBoardTypeName && int.TryParse(version.Split('.')[0], out var major)
            ? $"rev{major} ({version})"
            : $"{typeName} ({version})";
        var first = root.TryGetProperty("firstConnectedAtEpoch", out var fc) && fc.TryGetInt64(out var epoch) ? epoch : 0;
        whyNot = null;
        return new WebCredentials.Device(boardId, chipUid, hostname, ethMac ?? "unbekannt", revision, password,
                                         DateTimeOffset.FromUnixTimeSeconds(first));
    }

    private static string? Str(JsonElement e, string name) =>
        e.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() : null;
}
