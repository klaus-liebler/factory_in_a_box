// "Letztes bekanntes Board"-Caching (s. docs/build-process.md Abschnitt 4): Phasen, die kein
// eigenes Board auslesen (Register-Map, Git-Status, Copy, Flash), muessen trotzdem wissen, in
// welchen Board-Archiv-Ordner sie schreiben/aus welchem sie lesen sollen. Der zuletzt von
// ReadHardwareIDsAndGenerateFilesAndCerts{Lazy,Forced} erkannte boardId wird dafuer in einer
// gitignoreten Datei unter build/ gecacht.
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { boardIdCacheFile, buildDir } from "./paths.ts";

export function readCachedBoardId(): string | undefined {
	if (!existsSync(boardIdCacheFile)) return undefined;
	const content = readFileSync(boardIdCacheFile, "utf8").trim();
	return content.length > 0 ? content : undefined;
}

export function writeCachedBoardId(boardId: string): void {
	mkdirSync(buildDir, { recursive: true });
	writeFileSync(boardIdCacheFile, boardId);
}

export function resolveBoardId(explicit?: string): string | undefined {
	return explicit ?? readCachedBoardId();
}

// Fuer Phasen, die ohne Board-Kontext keinen Sinn ergeben (Copy, Flash) -- klare Fehlermeldung
// statt eines kryptischen ENOENT beim Zugriff auf einen nicht existierenden Archiv-Ordner.
export function requireBoardId(explicit?: string): string {
	const boardId = resolveBoardId(explicit);
	if (!boardId) {
		throw new Error(
			"Kein Board-Kontext bekannt (weder --board angegeben noch build/.last-board-id vorhanden). " +
				"Zuerst 'ReadHardwareIDsAndGenerateFilesAndCertsLazy' mit angeschlossenem Board ausfuehren."
		);
	}
	return boardId;
}
