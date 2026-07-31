// Eine einzige Implementierung fuer Git-Versionsinformationen, gemeinsam genutzt von
// gitconstants.hh/build-info.ts/gitstatus.json (s. Phases/ReadGitStatus.cs).
using System.Diagnostics;
using System.Text.Json.Serialization;

namespace Builder;

public sealed class GitInfo
{
	[JsonPropertyName("commitHash")]
	public required string CommitHash { get; init; }

	[JsonPropertyName("branch")]
	public required string Branch { get; init; }

	[JsonPropertyName("tag")]
	public required string Tag { get; init; }

	// Alle Zeitpunkte als Unix-Epoch-Sekunden statt vorformatierter Strings -- Formatierung/
	// Zeitzone passiert ausschliesslich am Anzeigeort (Browser).
	[JsonPropertyName("commitDateEpoch")]
	public required long CommitDateEpoch { get; init; }

	[JsonPropertyName("commitAuthor")]
	public required string CommitAuthor { get; init; }

	[JsonPropertyName("commitMessage")]
	public required string CommitMessage { get; init; }

	[JsonPropertyName("isDirty")]
	public required bool IsDirty { get; init; }

	[JsonPropertyName("buildTimestampEpoch")]
	public required long BuildTimestampEpoch { get; init; }

	[JsonPropertyName("version")]
	public required string Version { get; init; }
}

public static class GitInfoReader
{
	// WORKING_DIRECTORY ist die Repo-Wurzel dieses Projektunterordners -- git findet das
	// umschliessende factory_in_a_box-Repo trotzdem automatisch (laeuft die Verzeichnisse hoch).
	private static string Git(string arguments, string fallback = "unknown")
	{
		try
		{
			var psi = new ProcessStartInfo("git", arguments)
			{
				WorkingDirectory = Paths.RootDir,
				RedirectStandardOutput = true,
				RedirectStandardError = true,
				UseShellExecute = false,
			};
			using var process = Process.Start(psi)!;
			var output = process.StandardOutput.ReadToEnd();
			process.WaitForExit();
			var trimmed = output.Trim();
			return process.ExitCode == 0 && trimmed.Length > 0 ? trimmed : fallback;
		}
		catch
		{
			return fallback;
		}
	}

	public static GitInfo ReadGitInfo()
	{
		var commitHash = Git("rev-parse --short HEAD");
		var branch = Git("rev-parse --abbrev-ref HEAD");
		var tag = Git("describe --tags --always");
		// %at statt %ai: liefert direkt Unix-Epoch-Sekunden, kein Parsen eines formatierten Datums noetig.
		var commitDateEpoch = long.Parse(Git("log -1 --format=%at", "0"));
		var commitAuthor = Git("log -1 --format=%an");
		var commitMessage = Git("log -1 --format=%s");
		var isDirty = Git("status --porcelain", "") != "";
		var buildTimestampEpoch = DateTimeOffset.UtcNow.ToUnixTimeSeconds();

		return new GitInfo
		{
			CommitHash = commitHash,
			Branch = branch,
			Tag = tag,
			CommitDateEpoch = commitDateEpoch,
			CommitAuthor = commitAuthor,
			CommitMessage = commitMessage,
			IsDirty = isDirty,
			BuildTimestampEpoch = buildTimestampEpoch,
			Version = $"{tag}-{commitHash}",
		};
	}
}
