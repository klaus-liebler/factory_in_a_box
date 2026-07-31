// Datei-basierte Board-Verwaltung (s. docs/build-process.md Abschnitt 5) -- ersetzt die vormalige
// SQLite-Datei boards.db (BuilderDb.cs). Board-Zustand + Flash-Historie leben direkt im
// Board-Archiv (stm32_boards/<boardId>/board.json bzw. flash_events.jsonl), analog zum
// @klaus-liebler/espidf-vite-Vorbild (Repo npm-packages, Commit "Generate everything in special
// 'GENERATED'-Directory", 2025-01-26): dort wurde exakt dieselbe SQLite-Interimsloesung
// (database.ts) zugunsten von genau einem Verzeichnis pro physischem Board verworfen. boardId
// (s. Phases/ReadHardwareIds.cs: "{shortId}_{volle Chip-UID}") ist bereits eine Chip-UID-Ableitung
// und zugleich der Board-Archiv-Ordnername -- kein separater Chip-UID->boardId-Index noetig.
//
// KEIN zentraler Typ-Katalog (die vormaligen mcu_types/board_types-Tabellen entfallen ersatzlos):
// mcu_types enthielt ohnehin nur je einen Eintrag (McuTypeName unten ist projektweit konstant),
// board_types.version war immer 1 und .settings immer NULL -- beides tote Spalten, nie
// geschrieben. board.json traegt mcuType/boardTypeName deshalb direkt als Klartext-Feld, ohne
// Fremdschluessel. Falls kuenftig eine vom angeschlossenen Board unabhaengige Liste ALLER
// bekannten Board-Typen gebraucht wird (z.B. fuer eine Auswahl-UI): das laesst sich jederzeit
// nachtraeglich per Verzeichnis-Scan ueber alle stm32_boards/*/board.json ermitteln (Distinct auf
// boardTypeName), ohne dass diese Klasse dafuer etwas vorhalten muss.
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Builder;

// Inhalt von board.json -- aktueller Zustand EINES physischen Boards (ersetzt die vormalige
// "boards"-Tabellenzeile). Wird bei jedem erfolgreichen Flash komplett neu geschrieben (nicht
// gepatcht) -- FirstConnectedAtEpoch bleibt dabei von einer vorherigen Version uebernommen, alle
// anderen Felder werden ersetzt (s. BoardStore.RecordSuccessfulFlash()).
public sealed record BoardRecord(
	[property: JsonPropertyName("chipUid")] string ChipUid,
	[property: JsonPropertyName("hostname")] string Hostname,
	[property: JsonPropertyName("mcuType")] string McuType,
	[property: JsonPropertyName("boardTypeName")] string BoardTypeName,
	[property: JsonPropertyName("firstConnectedAtEpoch")] long FirstConnectedAtEpoch,
	[property: JsonPropertyName("lastConnectedAtEpoch")] long LastConnectedAtEpoch,
	[property: JsonPropertyName("lastStlinkProbeSerial")] string? LastStlinkProbeSerial);

// Eine Zeile in flash_events.jsonl -- ein Eintrag pro erfolgreich verifiziertem Flash-Vorgang
// (ersetzt die vormalige "flash_events"-Tabelle). Bewusst JSON Lines statt einer JSON-Array-Datei:
// echtes Append-Only-Log (File.AppendAllText, kein Read-Modify-Write der kompletten Historie bei
// jedem Flash-Vorgang -- ein Abbruch mitten im Schreiben kann so hoechstens die neueste Zeile
// beschaedigen, nie die bisherige Historie).
public sealed record FlashEvent(
	[property: JsonPropertyName("flashedAtEpoch")] long FlashedAtEpoch,
	[property: JsonPropertyName("cmakePreset")] string CmakePreset,
	[property: JsonPropertyName("gitCommitHash")] string GitCommitHash,
	[property: JsonPropertyName("gitBranch")] string GitBranch,
	[property: JsonPropertyName("gitIsDirty")] bool GitIsDirty,
	[property: JsonPropertyName("firmwareVersion")] string FirmwareVersion);

public static class BoardStore
{
	// Es gibt in diesem Projekt genau einen MCU-Typ -- anders als bei board_types (individuelle
	// Board-Revisionen/-Rollen) rechtfertigt das keinen eigenen Katalog, s. Datei-Kommentar oben.
	public const string McuTypeName = "STM32H563ZI";

	private static string BoardJsonPath(string boardId) => Path.Combine(BoardArchive.BoardArchiveDir(boardId), "board.json");
	private static string FlashEventsPath(string boardId) => Path.Combine(BoardArchive.BoardArchiveDir(boardId), "flash_events.jsonl");

	private static BoardRecord? TryReadBoard(string boardId)
	{
		var path = BoardJsonPath(boardId);
		if (!File.Exists(path)) return null;
		return JsonSerializer.Deserialize<BoardRecord>(File.ReadAllText(path), Json.Compact);
	}

	// Board-Type-Zuordnung ueber die tatsaechliche Chip-UID (bzw. das daraus abgeleitete boardId)
	// statt (wie vor der SQLite-Einfuehrung) aus dem CMake-Preset: ein einmal erfolgreich
	// geflashtes Board behaelt seinen board_type auch dann, wenn es versehentlich mit einem
	// anderen Preset erneut geflasht wird, und eine Fehlklassifizierung laesst sich per direktem
	// Edit von board.json korrigieren, ohne Code anzufassen. Liefert null, wenn dieses Board noch
	// nie erfolgreich geflasht wurde -- der Aufrufer (s. Phases/ReadHardwareIds.cs) faellt dann auf
	// BuilderOptions.DefaultBoardTypeName zurueck.
	public static string? TryGetBoardTypeName(string boardId) => TryReadBoard(boardId)?.BoardTypeName;

	public sealed record RecordFlashParams(
		string BoardId,
		string ChipUid,
		string Hostname,
		string? StlinkProbe,
		string BoardTypeName,
		string CmakePreset,
		string GitCommitHash,
		string GitBranch,
		bool GitIsDirty,
		string FirmwareVersion);

	// Wird ausschliesslich nach einem von STM32_Programmer_CLI verifiziert erfolgreichen
	// Schreibvorgang aufgerufen (s. Phases/FlashFirmware.cs) -- ein fehlgeschlagener Flash erzeugt
	// bewusst weder ein board.json-Update noch einen flash_events.jsonl-Eintrag.
	public static void RecordSuccessfulFlash(RecordFlashParams p)
	{
		var now = DateTimeOffset.UtcNow.ToUnixTimeSeconds();
		var existing = TryReadBoard(p.BoardId);

		// LastStlinkProbeSerial wird bewusst UNBEDINGT mit p.StlinkProbe ueberschrieben (auch mit
		// null, falls DetectStlinkProbeSerial() diesmal nichts liefert) -- exakt das Verhalten der
		// vormaligen SQL-Upsert-Logik (excluded.last_stlink_probe), nicht stillschweigend geaendert.
		var board = new BoardRecord(
			ChipUid: p.ChipUid,
			Hostname: p.Hostname,
			McuType: McuTypeName,
			BoardTypeName: p.BoardTypeName,
			FirstConnectedAtEpoch: existing?.FirstConnectedAtEpoch ?? now,
			LastConnectedAtEpoch: now,
			LastStlinkProbeSerial: p.StlinkProbe);

		var boardDir = BoardArchive.BoardArchiveDir(p.BoardId);
		Directory.CreateDirectory(boardDir);
		File.WriteAllText(BoardJsonPath(p.BoardId), JsonSerializer.Serialize(board, Json.Options) + "\n");

		var flashEvent = new FlashEvent(now, p.CmakePreset, p.GitCommitHash, p.GitBranch, p.GitIsDirty, p.FirmwareVersion);
		File.AppendAllText(FlashEventsPath(p.BoardId), JsonSerializer.Serialize(flashEvent, Json.Compact) + "\n");

		Console.WriteLine($"{BoardJsonPath(p.BoardId)}: Flash-Ereignis fuer {p.ChipUid} ({p.Hostname}) protokolliert.");
	}
}
