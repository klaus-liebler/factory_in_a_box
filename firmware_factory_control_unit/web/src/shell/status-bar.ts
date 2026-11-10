// The status bar of the whole web application (app-shell.ts renders it; it stays when the working
// area is maximized). From the left: the WebSocket connection, the state or mode of the app that is
// shown (loading, compiling, a timeout ...), then whatever that app puts there. Apps do not show
// such states inside their own page any more:
//
//   setAppStatus(this, { text: "Lade Werte …", kind: "busy" });
//   setAppStatus(this, { text: "RUN", kind: "ok", extra: html`<span>Zyklus 50 ms</span>` });
//
// The shell shows the status of the visible app; a hidden app may keep calling setAppStatus.
import type { TemplateResult } from "lit";

/** ok: fine; busy: working or waiting; warning; error; idle: nothing to say. */
export type StatusKind = "ok" | "busy" | "warning" | "error" | "idle";

export interface AppStatus {
	text: string;
	kind: StatusKind;
	/** Free content after the state: chips, counters (classes sb-chip, sb-warn, sb-right in styles.css). */
	extra?: TemplateResult | null;
}

const statuses = new WeakMap<HTMLElement, AppStatus>();
const listeners = new Set<() => void>();

export function setAppStatus(app: HTMLElement, status: AppStatus): void {
	const old = statuses.get(app);
	// same text and kind and no free content: nothing to render again
	if (old && old.text === status.text && old.kind === status.kind && !old.extra && !status.extra) return;
	statuses.set(app, status);
	for (const l of listeners) l();
}

export function appStatusOf(app: HTMLElement | null): AppStatus | null {
	return (app && statuses.get(app)) ?? null;
}

/** Called after every change; returns the function that ends it. */
export function onAppStatusChange(listener: () => void): () => void {
	listeners.add(listener);
	return () => listeners.delete(listener);
}

/** The kinds of the apps' earlier status texts. */
export function kindOf(variant: "success" | "warning" | "error"): StatusKind {
	return variant === "success" ? "ok" : variant === "warning" ? "busy" : "error";
}
