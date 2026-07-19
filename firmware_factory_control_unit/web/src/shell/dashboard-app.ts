// Optionales Lifecycle-Interface fuer die drei App-Elemente, die app-shell.ts einhaengt. Alle
// drei App-Elemente werden GENAU EINMAL erzeugt und bleiben dauerhaft im DOM (nur per "hidden"
// sichtbar/unsichtbar geschaltet) -- Zustand (z.B. der Ringpuffer der Power-Management-App)
// bleibt so beim Wechseln zwischen Apps erhalten. onShow()/onHide() ersetzen dafuer
// connectedCallback()/disconnectedCallback() als Start/Stop-Signal fuer Polling o.ae.
export interface DashboardApp {
	onShow(): void;
	onHide(): void;
}

export function hasDashboardLifecycle(el: HTMLElement): el is HTMLElement & DashboardApp {
	return typeof (el as Partial<DashboardApp>).onShow === "function";
}
