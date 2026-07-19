// Minimaler Pfad-Router fuer die Dashboard-Shell (app-shell.ts) -- inspiriert von RouterMenu aus
// @klaus-liebler/web-components, aber ohne jede Websocket-Kopplung und ohne Regex-Parameter
// (die drei Apps hier haben keine dynamischen URL-Segmente, ein exakter Pfadabgleich genuegt).
// History-API (pushState/popstate) statt Hash-Routing, damit die URL in der Adresszeile wie eine
// "richtige" Unterseite aussieht (z.B. "/power").

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
		return window.location.pathname || "/";
	}

	// Faellt auf die erste Route zurueck, wenn der aktuelle Pfad keiner registrierten Route
	// entspricht (z.B. Erststart, oder ein Lesezeichen auf einen inzwischen entfernten Pfad).
	MatchCurrent(): RouteDef {
		const path = this.CurrentPath();
		return this.routes.find((r) => r.path === path) ?? this.routes[0];
	}

	Navigate(path: string): void {
		if (this.CurrentPath() === path) return;
		window.history.pushState(null, "", path);
		this.dispatchChange();
	}

	private dispatchChange(): void {
		this.dispatchEvent(new Event("change"));
	}
}
