// Minimal path router for the shell (app-shell.ts) - inspired by RouterMenu from
// @klaus-liebler/web-components, but without any WebSocket coupling and without regex parameters (the
// workspaces have no dynamic URL segments, an exact path match is enough). History API (pushState/popstate)
// instead of hash routing, so that the URL in the address bar looks like a "real" sub-page (e.g. "/power").

//
// Hash mode (useHashRouting(), "#/plc"): for an app on a plain static web server, which has no fallback from
// "/plc" to index.html and may serve the app below a sub-path.

let hashMode = false;

/** Workspace paths in the fragment ("#/plc") instead of the path; call before AppShell.Start(). */
export function useHashRouting(on = true): void {
	hashMode = on;
}

/** The workspace path of the current URL. */
export function currentPath(): string {
	return (hashMode ? window.location.hash.slice(1) : window.location.pathname) || "/";
}

/** The URL (for href and pushState) of a workspace path. */
export function workspaceUrl(path: string): string {
	return hashMode ? `#${path}` : path;
}

export interface RouteDef {
	path: string;
	title: string;
	icon: string;
}

export class Router extends EventTarget {
	constructor(private readonly routes: readonly RouteDef[]) {
		super();
		window.addEventListener("popstate", () => this.dispatchChange());
	}

	get Routes(): readonly RouteDef[] {
		return this.routes;
	}

	CurrentPath(): string {
		return currentPath();
	}

	// Falls back to the first route if the current path matches no registered route (e.g. first start, or a
	// bookmark to a path removed in the meantime).
	MatchCurrent(): RouteDef {
		const path = this.CurrentPath();
		return this.routes.find((r) => r.path === path) ?? this.routes[0];
	}

	Navigate(path: string): void {
		if (this.CurrentPath() === path) return;
		window.history.pushState(null, "", workspaceUrl(path));
		this.dispatchChange();
	}

	private dispatchChange(): void {
		this.dispatchEvent(new Event("change"));
	}
}
