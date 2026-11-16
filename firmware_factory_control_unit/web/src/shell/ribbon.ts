// The ribbon of the application (app-shell.ts renders it, like a desktop CAD/office program): every
// workspace (app) publishes its tabs with groups of commands; the shell shows the ones of the visible
// workspace and adds its own "Ansicht" tab. Apps keep the ribbon up to date like their status:
//
//   setAppRibbon(this, [{ id: "online", label: "Online", groups: [
//       { label: "Steuerung", commands: [{ id: "run", label: "Start", icon: "run", run: () => this.start(), disabled: !stopped }] },
//   ] }]);
//
// A command is a large button (icon over label), a small one (three per column), or a toggle (`active`).
// Every group's caption opens a menu with all its commands (also the ones a narrow window hides).
import type { IconName } from "./icons.ts";

export interface RibbonCommand {
	id: string;
	label: string;
	icon: IconName;
	/** The tooltip (default: the label). */
	title?: string;
	run(): void;
	disabled?: boolean;
	/** A toggle or one of a choice: shown pressed. */
	active?: boolean;
	/** "small": a row of a three-row column, icon left of the label. */
	size?: "large" | "small";
}

export interface RibbonGroup {
	label: string;
	commands: RibbonCommand[];
}

export interface RibbonTab {
	id: string;
	label: string;
	groups: RibbonGroup[];
}

const ribbons = new WeakMap<HTMLElement, RibbonTab[]>();
const listeners = new Set<() => void>();
let scheduled = false;

/** Publishes the workspace's ribbon (call it whenever what it shows may have changed, e.g. in willUpdate). */
export function setAppRibbon(app: HTMLElement, tabs: RibbonTab[]): void {
	ribbons.set(app, tabs);
	// many apps publish on every render: tell the shell once per frame
	if (scheduled) return;
	scheduled = true;
	queueMicrotask(() => {
		scheduled = false;
		for (const l of listeners) l();
	});
}

export function appRibbonOf(app: HTMLElement | null): RibbonTab[] {
	return (app && ribbons.get(app)) ?? [];
}

export function onAppRibbonChange(listener: () => void): () => void {
	listeners.add(listener);
	return () => listeners.delete(listener);
}

/** Switches the workspace (an app's path), e.g. from a dashboard tile or a ribbon command. */
export function openWorkspace(path: string): void {
	if (window.location.pathname === path) return;
	window.history.pushState(null, "", path);
	window.dispatchEvent(new PopStateEvent("popstate"));
}
