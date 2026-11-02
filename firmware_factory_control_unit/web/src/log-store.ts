// Firmware log in the browser: every system.LogMessage the firmware sends (it keeps no history
// itself) lands here for the whole lifetime of the page, so nothing is lost while another page
// is open. At most MAX_LOG_LINES are kept; the Terminal page shows them as a table.
import { system } from "../generated/ws-protocol.js";
import { registerWsProtocolListener, type WsProtocolListener } from "./ws-client.js";

export const MAX_LOG_LINES = 10000;

export interface LogEntry {
	seq: number; // the browser's own running number (stable row key)
	level: system.LogLevel;
	uptimeMs: number;
	unixMs: number; // 0: the device had no wall clock (SNTP) yet
	logger: string;
	text: string;
}

type Listener = (added: LogEntry[], removedCount: number) => void;

const entries: LogEntry[] = [];
const listeners = new Set<Listener>();
let nextSeq = 1;
let lastDeviceSequence = 0;
let pending: LogEntry[] = [];
let flushScheduled = false;

/** All entries kept now (oldest first). */
export function logEntries(): readonly LogEntry[] {
	return entries;
}

/** Called with new entries (batched per animation frame) and how many old ones were dropped. */
export function subscribeLog(listener: Listener): () => void {
	listeners.add(listener);
	return () => listeners.delete(listener);
}

export function clearLog(): void {
	const removed = entries.length;
	entries.length = 0;
	for (const l of listeners) l([], removed);
}

function push(entry: LogEntry): void {
	pending.push(entry);
	if (!flushScheduled) {
		flushScheduled = true;
		requestAnimationFrame(flush);
	}
}

function flush(): void {
	flushScheduled = false;
	const added = pending;
	pending = [];
	entries.push(...added);
	const removed = Math.max(0, entries.length - MAX_LOG_LINES);
	if (removed > 0) entries.splice(0, removed);
	for (const l of listeners) l(added, removed);
}

function note(text: string, level = system.LogLevel.LOG_WARN): void {
	push({ seq: nextSeq++, level, uptimeMs: 0, unixMs: Date.now(), logger: "browser", text });
}

class LogListener implements WsProtocolListener {
	onWsMessage(messageTypeId: number, view: DataView): void {
		if (messageTypeId !== system.LogMessage.TYPE_ID) return;
		const m = system.LogMessage.decode(view, 0);
		// gaps: records the firmware's small buffer dropped, or that the WebSocket missed
		const missed = lastDeviceSequence !== 0 && m.sequence > lastDeviceSequence + 1 ? m.sequence - lastDeviceSequence - 1 : 0;
		if (m.lost > 0 || missed > 0) {
			note(`... ${Math.max(m.lost, missed)} log line(s) lost on the way`);
		}
		lastDeviceSequence = m.sequence;
		push({ seq: nextSeq++, level: m.level, uptimeMs: m.uptimeMs, unixMs: m.unixMs, logger: m.logger, text: m.text });
	}
}

/** For the whole page lifetime (app.ts). */
export function startLogStore(): void {
	registerWsProtocolListener(system.NAMESPACE_ID, new LogListener());
}
