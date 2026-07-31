// Pfade ins Board-Archiv (s. docs/build-process.md Abschnitt 3/5).
namespace Builder;

public static class BoardArchive
{
	public static string BoardArchiveDir(string boardId) => Path.Combine(BuilderOptions.Current.BoardsDir, boardId);

	public static string BoardGeneratedDir(string boardId) => Path.Combine(BoardArchiveDir(boardId), "generated");
}
