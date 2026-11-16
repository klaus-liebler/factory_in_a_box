// Formatting shared by the workspaces: durations, sizes, clock times (the browser's time zone, German), and
// the fixed-size NUL-terminated names of the binary protocol.

/** "3 d 4 h", "2 h 5 min", "4 min 7 s", "12 s". */
export function formatUptime(totalSeconds: number): string {
	const s = Math.max(0, Math.floor(totalSeconds));
	const d = Math.floor(s / 86400), h = Math.floor((s % 86400) / 3600), m = Math.floor((s % 3600) / 60);
	if (d) return `${d} d ${h} h`;
	if (h) return `${h} h ${m} min`;
	if (m) return `${m} min ${s % 60} s`;
	return `${s} s`;
}

/** "512 B", "3.4 KiB", "1.20 MiB". */
export function formatBytes(bytes: number): string {
	if (bytes < 1024) return `${bytes} B`;
	if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KiB`;
	return `${(bytes / 1024 / 1024).toFixed(2)} MiB`;
}

/** The time of day "14:03:27". */
export function clock(t: Date | number): string {
	return (typeof t === "number" ? new Date(t) : t).toLocaleTimeString("de-DE");
}

/** Date and time "16.11.2026, 14:03:27" of Unix seconds. */
export function dateTimeOfEpoch(epochSeconds: number): string {
	return new Date(epochSeconds * 1000).toLocaleString("de-DE");
}

/** A fixed-size byte field with a NUL-terminated ASCII name (thread and pool names of the firmware). */
export function cString(bytes: readonly number[]): string {
	const end = bytes.indexOf(0);
	return String.fromCharCode(...(end < 0 ? bytes : bytes.slice(0, end)));
}
