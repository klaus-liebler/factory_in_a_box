// Pfade ins Board-Archiv (s. docs/build-process.md Abschnitt 3/5): ein Ordner pro physischem
// Board unter boardsDir(), darin "generated/" als vollstaendiger, bei jedem Lauf ueberschriebener
// Snapshot aller fuer dieses Board zuletzt erzeugten Artefakte.
import path from "node:path";
import { boardsDir } from "./environment-config.ts";

export function boardArchiveDir(boardId: string): string {
	return path.join(boardsDir(), boardId);
}

export function boardGeneratedDir(boardId: string): string {
	return path.join(boardArchiveDir(boardId), "generated");
}
