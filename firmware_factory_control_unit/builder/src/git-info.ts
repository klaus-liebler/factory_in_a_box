// Eine einzige Implementierung fuer Git-Versionsinformationen -- ersetzt sowohl
// cmake/generate_git_constants.cmake als auch das vormalige tools/generate-build-info.mjs
// (s. docs/build-process.md Abschnitt 6, Phase "ReadGitStatusAndGenerateFiles"). Wird sowohl fuer
// gitconstants.hh/build-info.ts als auch (ueber gitstatus.json) fuer den builder.db-Flash-Log-
// Eintrag verwendet.
import { execSync } from "node:child_process";
import { rootDir } from "./paths.ts";

export interface GitInfo {
	commitHash: string;
	branch: string;
	tag: string;
	// Alle Zeitpunkte als Unix-Epoch-Sekunden (int64-tauglich, s. docs/build-process.md) statt
	// vorformatierter Strings -- Formatierung/Zeitzone passiert ausschliesslich am Anzeigeort
	// (Browser: lokale Zeitzone via Date/Intl, s. web/src/apps/system-info-app.ts).
	commitDateEpoch: number;
	commitAuthor: string;
	commitMessage: string;
	isDirty: boolean;
	buildTimestampEpoch: number;
	version: string;
}

// WORKING_DIRECTORY ist die Repo-Wurzel dieses Projektunterordners -- git findet das
// umschliessende factory_in_a_box-Repo trotzdem automatisch (laeuft die Verzeichnisse hoch),
// exakt wie zuvor im CMake-Skript.
function git(command: string, fallback = "unknown"): string {
	try {
		return execSync(`git ${command}`, { cwd: rootDir, encoding: "utf8" }).trim() || fallback;
	} catch {
		return fallback;
	}
}

export function readGitInfo(): GitInfo {
	const commitHash = git("rev-parse --short HEAD");
	const branch = git("rev-parse --abbrev-ref HEAD");
	const tag = git("describe --tags --always");
	// %at statt %ai: liefert direkt Unix-Epoch-Sekunden, kein Parsen eines formatierten Datums noetig.
	const commitDateEpoch = Number(git("log -1 --format=%at", "0"));
	const commitAuthor = git("log -1 --format=%an");
	const commitMessage = git("log -1 --format=%s");
	const isDirty = git("status --porcelain", "") !== "";
	const buildTimestampEpoch = Math.floor(Date.now() / 1000);

	return {
		commitHash,
		branch,
		tag,
		commitDateEpoch,
		commitAuthor,
		commitMessage,
		isDirty,
		buildTimestampEpoch,
		version: `${tag}-${commitHash}`,
	};
}
