// Flash-Log (s. docs/build-process.md Abschnitt 8) -- SQLite-Datei builder.db neben den
// Board-Archiven. Schema angelehnt an das bestehende esp32_boards/builder.db (mcu_types ->
// board_types -> boards), erweitert um eine echte Append-Only-Tabelle flash_events: das
// ESP32-Schema kennt nur den *letzten* Verbindungszeitpunkt (ueberschrieben bei jedem Connect),
// hier soll aber JEDER erfolgreiche Flash-Vorgang einen eigenen Datensatz bekommen.
//
// node:sqlite ist noch als "experimental" markiert (Warnung auf stderr beim ersten Zugriff),
// funktioniert aber ohne Zusatzabhaengigkeit -- passt zur Entscheidung gegen ts-node/tsx & Co.
// Sollte sich das in der Praxis als zu instabil erweisen, ist better-sqlite3 der Fallback
// (s. docs/build-process.md Abschnitt 8.3).
import type { DatabaseSync } from "node:sqlite";
import { mkdirSync } from "node:fs";
import { createRequire } from "node:module";
import { boardsDir, dbPath } from "./environment-config.ts";

const SCHEMA_SQL = `
CREATE TABLE IF NOT EXISTS mcu_types (
	id   INTEGER PRIMARY KEY AUTOINCREMENT,
	name TEXT NOT NULL UNIQUE
);

CREATE TABLE IF NOT EXISTS board_types (
	id       INTEGER PRIMARY KEY AUTOINCREMENT,
	name     TEXT NOT NULL,
	version  INTEGER NOT NULL,
	mcu_id   INTEGER NOT NULL REFERENCES mcu_types(id),
	settings TEXT,
	UNIQUE(name, version)
);

CREATE TABLE IF NOT EXISTS boards (
	chip_uid           TEXT PRIMARY KEY,
	hostname           TEXT NOT NULL,
	mcu_type_id        INTEGER REFERENCES mcu_types(id),
	board_type_id      INTEGER REFERENCES board_types(id),
	first_connected_dt INTEGER NOT NULL,
	last_connected_dt  INTEGER NOT NULL,
	last_stlink_probe  TEXT,
	cert_issued_dt     INTEGER,
	settings           TEXT
);

CREATE TABLE IF NOT EXISTS flash_events (
	id               INTEGER PRIMARY KEY AUTOINCREMENT,
	chip_uid         TEXT NOT NULL REFERENCES boards(chip_uid),
	flashed_dt       INTEGER NOT NULL,
	cmake_preset     TEXT NOT NULL,
	git_commit_hash  TEXT NOT NULL,
	git_branch       TEXT NOT NULL,
	git_is_dirty     INTEGER NOT NULL,
	firmware_version TEXT NOT NULL
);
`;

let db: DatabaseSync | undefined;

function getDb(): DatabaseSync {
	if (!db) {
		// Lazy statt statischer Top-Level-Import: node:sqlite emittiert seine
		// ExperimentalWarning beim Laden des Moduls, nicht erst bei new DatabaseSync() -- ein
		// statischer Import wuerde die Warnung damit bei JEDER CLI-Phase ausgeben (cli.ts
		// importiert alle Phasenmodule statisch), auch bei solchen ganz ohne DB-Zugriff.
		// createRequire() statt "await import(...)", damit getDb() synchron bleibt (wird von
		// recordSuccessfulFlash() synchron aus phases/flash-firmware.ts aufgerufen).
		const nodeRequire = createRequire(import.meta.url);
		const { DatabaseSync } = nodeRequire("node:sqlite") as typeof import("node:sqlite");
		mkdirSync(boardsDir(), { recursive: true });
		db = new DatabaseSync(dbPath());
		db.exec(SCHEMA_SQL);
	}
	return db;
}

export const MCU_TYPE_NAME = "STM32H563ZI";

// Board-Typ wird aus dem CMake-Preset abgeleitet (s. CMakeLists.txt BOARD_NUCLEO_H563ZI):
// "Debug-Nucleo" -> Test-Rig, alles andere -> die echte Factory-Control-Unit-Platine.
export function boardTypeNameForPreset(preset: string): { name: string; version: number } {
	if (preset === "Debug-Nucleo") {
		return { name: "Nucleo-H563ZI-TestRig", version: 1 };
	}
	return { name: "FactoryControlUnit", version: 1 };
}

function ensureMcuType(name: string): number {
	const database = getDb();
	database.prepare("INSERT OR IGNORE INTO mcu_types (name) VALUES (?)").run(name);
	const row = database.prepare("SELECT id FROM mcu_types WHERE name = ?").get(name) as { id: number };
	return row.id;
}

function ensureBoardType(name: string, version: number, mcuId: number): number {
	const database = getDb();
	database
		.prepare("INSERT OR IGNORE INTO board_types (name, version, mcu_id) VALUES (?, ?, ?)")
		.run(name, version, mcuId);
	const row = database
		.prepare("SELECT id FROM board_types WHERE name = ? AND version = ?")
		.get(name, version) as { id: number };
	return row.id;
}

export interface RecordFlashParams {
	chipUid: string;
	hostname: string;
	stlinkProbe?: string;
	cmakePreset: string;
	gitCommitHash: string;
	gitBranch: string;
	gitIsDirty: boolean;
	firmwareVersion: string;
}

// Wird ausschliesslich nach einem von STM32_Programmer_CLI verifiziert erfolgreichen
// Schreibvorgang aufgerufen (s. phases/flash-firmware.ts) -- ein fehlgeschlagener Flash erzeugt
// bewusst keinen Eintrag.
export function recordSuccessfulFlash(params: RecordFlashParams): void {
	const database = getDb();
	const { name: boardTypeName, version: boardTypeVersion } = boardTypeNameForPreset(params.cmakePreset);
	const mcuId = ensureMcuType(MCU_TYPE_NAME);
	const boardTypeId = ensureBoardType(boardTypeName, boardTypeVersion, mcuId);
	const now = Math.floor(Date.now() / 1000);

	database
		.prepare(
			`INSERT INTO boards (chip_uid, hostname, mcu_type_id, board_type_id, first_connected_dt, last_connected_dt, last_stlink_probe)
			 VALUES (?, ?, ?, ?, ?, ?, ?)
			 ON CONFLICT(chip_uid) DO UPDATE SET
				hostname = excluded.hostname,
				mcu_type_id = excluded.mcu_type_id,
				board_type_id = excluded.board_type_id,
				last_connected_dt = excluded.last_connected_dt,
				last_stlink_probe = excluded.last_stlink_probe`
		)
		.run(params.chipUid, params.hostname, mcuId, boardTypeId, now, now, params.stlinkProbe ?? null);

	database
		.prepare(
			`INSERT INTO flash_events (chip_uid, flashed_dt, cmake_preset, git_commit_hash, git_branch, git_is_dirty, firmware_version)
			 VALUES (?, ?, ?, ?, ?, ?, ?)`
		)
		.run(
			params.chipUid,
			now,
			params.cmakePreset,
			params.gitCommitHash,
			params.gitBranch,
			params.gitIsDirty ? 1 : 0,
			params.firmwareVersion
		);

	console.log(`builder.db: Flash-Ereignis fuer ${params.chipUid} (${params.hostname}) protokolliert.`);
}
