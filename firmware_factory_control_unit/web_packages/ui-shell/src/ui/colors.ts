// The design tokens (ui/tokens.css) for colours drawn by script (canvas, SVG): the same values, so a chart
// and the CSS around it agree. Series colours for charts with several curves.
export const COLORS = {
	text: "#1f2937",
	muted: "#6b7280",
	grid: "#e5e7eb",
	track: "#e5e7eb",
	accent: "#2563eb",
	ok: "#16a34a",
	warn: "#d97706",
	error: "#dc2626",
	info: "#0891b2",
	ledGreen: "#22c55e",
	ledAmber: "#f59e0b",
	ledRed: "#ef4444",
	ledBlue: "#3b82f6",
} as const;

/** Distinguishable series colours (charts with several curves). */
export const SERIES = ["#2563eb", "#e08a1e", "#16a34a", "#dc2626", "#7c3aed", "#0891b2", "#be185d", "#4d7c0f"] as const;

/** A value's colour on a three-level scale (ok below warn, warning, error from alarm). */
export function levelColor(v: number, warn: number, alarm: number): string {
	return v >= alarm ? COLORS.error : v >= warn ? COLORS.warn : COLORS.ok;
}
