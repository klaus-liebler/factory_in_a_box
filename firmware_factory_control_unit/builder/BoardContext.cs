// "Letztes bekanntes Board"-Caching (s. docs/build-process.md Abschnitt 4).
namespace Builder;

public static class BoardContext
{
	public static string? ReadCachedBoardId()
	{
		if (!File.Exists(Paths.BoardIdCacheFile)) return null;
		var content = File.ReadAllText(Paths.BoardIdCacheFile).Trim();
		return content.Length > 0 ? content : null;
	}

	public static void WriteCachedBoardId(string boardId)
	{
		Directory.CreateDirectory(Paths.BuildDir);
		File.WriteAllText(Paths.BoardIdCacheFile, boardId);
	}

	public static string? ResolveBoardId(string? explicitBoardId) => explicitBoardId ?? ReadCachedBoardId();

	// Fuer Phasen, die ohne Board-Kontext keinen Sinn ergeben (Copy, Flash) -- klare Fehlermeldung
	// statt eines kryptischen Fehlers beim Zugriff auf einen nicht existierenden Archiv-Ordner.
	public static string RequireBoardId(string? explicitBoardId)
	{
		var boardId = ResolveBoardId(explicitBoardId);
		if (boardId is null)
		{
			throw new InvalidOperationException(
				"Kein Board-Kontext bekannt (weder --board angegeben noch build/.last-board-id vorhanden). " +
				"Zuerst 'ReadHardwareIDsAndGenerateFilesAndCertsLazy' mit angeschlossenem Board ausfuehren.");
		}
		return boardId;
	}
}
