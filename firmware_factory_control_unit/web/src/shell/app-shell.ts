// Ganzseiten-Dashboard-Shell: Header + Sidebar-Navigation (auf schmalen Bildschirmen ein
// Hamburger-Drawer) + Inhaltsbereich, in dem app.ts die drei Apps einhaengt. Angelehnt an
// AppController/RouterMenu aus @klaus-liebler/web-components, aber bewusst ohne Websocket,
// Login, Chatbot oder Easter-Egg -- diese Firmware spricht nur schlankes HTTP/Fetch (s. api.ts).
//
// Alle App-Elemente werden bei Start() genau einmal erzeugt und in .app-shell-content gemountet
// (imperativ per appendChild, NICHT ueber ein lit-html-Binding) -- render() bindet auf diesen
// Container nie etwas, lit-html laesst seinen Kindknoten-Inhalt also unangetastet, egal wie oft
// die Shell wegen activePath/menuOpen neu rendert. So bleibt z.B. der Zeitreihen-Ringpuffer der
// Power-Management-App beim Wechseln zu einer anderen App erhalten (nur "hidden" wechselt).
import { LitElement, html } from "lit";
import { customElement, state, query } from "lit/decorators.js";
import { unsafeHTML } from "lit/directives/unsafe-html.js";
import { Router, type RouteDef } from "./router.js";
import { hasDashboardLifecycle } from "./dashboard-app.js";
import { appStatusOf, onAppStatusChange } from "./status-bar.js";
import { onWsStateChange, wsState } from "../ws-client.js";
import logoSvg from "../assets/factory-control-unit-logo.svg?raw";
import headerPatternSvg from "../assets/header-pattern.svg?raw";

interface AppEntry extends RouteDef {
	element: HTMLElement;
}

// Kachelbarer Header-Hintergrund (s. assets/header-pattern.svg) als Data-URL -- gleiches Muster
// wie der SVG-Wellenhintergrund in AppController.Startup() (@klaus-liebler/web-components), nur
// mit den Logo-eigenen Markenfarben statt der dortigen blauen Wellen.
const HEADER_BACKGROUND_URL = `data:image/svg+xml;base64,${btoa(headerPatternSvg)}`;

@customElement("app-shell")
export class AppShell extends LitElement {
	protected createRenderRoot() {
		return this;
	}

	@state() private activePath = "";
	@state() private menuOpen = false;
	// Maximiert: Sidebar und grosser Header weg, nur eine schmale Kopfleiste mit "Menü einblenden"
	// bleibt (immer sichtbar, nie ein versteckter Griff). Gemerkt nur fuer diesen Tab (sessionStorage):
	// eine neu geoeffnete Seite startet immer in der Normalansicht.
	@state() private maximized = AppShell.loadMaximized();

	private static readonly MAXIMIZED_KEY = "fcu.shell.maximized";

	private static loadMaximized(): boolean {
		try {
			return sessionStorage.getItem(AppShell.MAXIMIZED_KEY) === "1";
		} catch {
			return false;
		}
	}

	private setMaximized(on: boolean): void {
		this.maximized = on;
		this.menuOpen = false;
		// maximized also means full screen: the browser's tabs and address bar go too (it needs the click
		// this call comes from; Esc leaves full screen, the working area stays maximized)
		if (on && !document.fullscreenElement) void document.documentElement.requestFullscreen?.().catch(() => undefined);
		if (!on && document.fullscreenElement) void document.exitFullscreen?.().catch(() => undefined);
		try {
			sessionStorage.setItem(AppShell.MAXIMIZED_KEY, on ? "1" : "0");
		} catch {
			/* ohne Speicher eben nur fuer diese Sitzung */
		}
		// Apps, die ihre Groesse nur bei window-resize neu berechnen (Power-Management-Diagramme),
		// bekommen den geaenderten Inhaltsbereich mit.
		this.updateComplete.then(() => window.dispatchEvent(new Event("resize")));
	}

	// Dark mode: the user's choice (kept in the browser), else what the system prefers.
	@state() private dark = AppShell.loadDark();
	private static readonly THEME_KEY = "fcu.shell.theme";

	private static loadDark(): boolean {
		try {
			const saved = localStorage.getItem(AppShell.THEME_KEY);
			if (saved === "dark" || saved === "light") return saved === "dark";
		} catch {
			/* no storage: the system's preference */
		}
		return window.matchMedia?.("(prefers-color-scheme: dark)").matches ?? false;
	}

	private setDark(on: boolean): void {
		this.dark = on;
		try {
			localStorage.setItem(AppShell.THEME_KEY, on ? "dark" : "light");
		} catch {
			/* for this page only then */
		}
		// canvases are drawn by script: let the apps draw again
		this.updateComplete.then(() => window.dispatchEvent(new Event("resize")));
	}

	private renderThemeButton() {
		return html`<button class="shell-theme" type="button" title=${this.dark ? "Helle Darstellung" : "Dunkle Darstellung"}
			aria-label="Dunkle Darstellung umschalten" aria-pressed=${this.dark} @click=${() => this.setDark(!this.dark)}>${this.dark ? "☀" : "🌙"}</button>`;
	}

	private entries: AppEntry[] = [];
	private router?: Router;
	private activeElement: HTMLElement | null = null;

	@query(".app-shell-content") private contentEl!: HTMLElement;

	// Von app.ts vor Start() fuer jede App aufgerufen -- Reihenfolge der Aufrufe bestimmt die
	// Reihenfolge in der Sidebar.
	public RegisterApp(path: string, title: string, icon: string, element: HTMLElement): void {
		element.hidden = true;
		this.entries.push({ path, title, icon, element });
	}

	public Start(): void {
		// the status bar follows the connection and the visible app's status
		onWsStateChange(() => this.requestUpdate());
		onAppStatusChange(() => this.requestUpdate());
		this.router = new Router(this.entries.map(({ path, title, icon }) => ({ path, title, icon })));
		this.router.addEventListener("change", () => this.sync());
		// "entries" ist bewusst kein @state (RegisterApp() wird oft aufgerufen, ein Re-Render pro
		// Aufruf waere unnoetig) -- ein einzelner requestUpdate() hier holt die Sidebar-Navigation
		// einmalig auf den aktuellen Stand, bevor die App-Elemente gemountet werden.
		this.requestUpdate();
		this.updateComplete.then(() => {
			for (const entry of this.entries) {
				this.contentEl.appendChild(entry.element);
			}
			this.sync();
		});
	}

	private sync(): void {
		if (!this.router) return;
		const match = this.router.MatchCurrent();
		const next = this.entries.find((e) => e.path === match.path) ?? this.entries[0];
		if (!next || next.element === this.activeElement) return;

		if (this.activeElement && hasDashboardLifecycle(this.activeElement)) {
			this.activeElement.onHide();
		}
		this.activeElement = next.element;
		this.activeElement.hidden = false;
		if (hasDashboardLifecycle(this.activeElement)) {
			this.activeElement.onShow();
		}
		this.activePath = next.path;
		this.menuOpen = false;

		for (const entry of this.entries) {
			if (entry.element !== this.activeElement) {
				entry.element.hidden = true;
			}
		}
	}

	private onNavClick(path: string, ev: Event): void {
		ev.preventDefault();
		this.router?.Navigate(path);
	}

	protected updated(): void {
		this.classList.toggle("shell-maximized", this.maximized);
		document.documentElement.dataset.theme = this.dark ? "dark" : "light";
	}

	/** The status bar: the WebSocket connection, the visible app's state, then the app's own content. */
	private renderStatusBar() {
		const ws = wsState();
		const wsText = ws === "open" ? "verbunden" : ws === "connecting" ? "verbinde …" : "getrennt";
		const status = appStatusOf(this.activeElement);
		return html`
			<footer class="shell-status" role="status">
				<span class="sb-ws sb-ws-${ws}" title="WebSocket-Verbindung zur Control Unit"><span class="sb-dot"></span>${wsText}</span>
				${status && status.text ? html`<span class="sb-mode sb-mode-${status.kind}">${status.text}</span>` : ""}
				${status?.extra ?? ""}
			</footer>
		`;
	}

	protected render() {
		return html`
			<header class="shell-header" style="background-image: url('${HEADER_BACKGROUND_URL}')">
				<button
					class="shell-hamburger"
					type="button"
					aria-label="Menü umschalten"
					@click=${() => (this.menuOpen = !this.menuOpen)}
				>☰</button>
				<span class="shell-logo">${unsafeHTML(logoSvg)}</span>
				${this.maximized
					? html`${this.renderThemeButton()}<span class="shell-slim-title">Factory Control Unit · ${this.entries.find((e) => e.path === this.activePath)?.title ?? ""}</span>
						<button
							class="shell-restore"
							type="button"
							title="Header und Menü wieder einblenden (Normalansicht)"
							@click=${() => this.setMaximized(false)}
						>☰ Menü einblenden</button>`
					: html`${this.renderThemeButton()}<button
						class="shell-maximize"
						type="button"
						title="Header und Menü ausblenden (Arbeitsfläche maximieren)"
						aria-label="Arbeitsfläche maximieren"
						@click=${() => this.setMaximized(true)}
					>⤢</button>`}
			</header>
			<div class="shell-body">
				<nav class="shell-nav ${this.menuOpen ? "shell-nav-open" : ""}">
					<ul>
						${this.entries.map(
							(entry) => html`
								<li>
									<a
										href=${entry.path}
										class=${entry.path === this.activePath ? "shell-nav-active" : ""}
										@click=${(ev: Event) => this.onNavClick(entry.path, ev)}
									>
										<span class="shell-nav-icon">${entry.icon}</span>
										<span class="shell-nav-label">${entry.title}</span>
									</a>
								</li>
							`
						)}
					</ul>
				</nav>
				${this.menuOpen ? html`<div class="shell-nav-scrim" @click=${() => (this.menuOpen = false)}></div>` : ""}
				<main class="app-shell-content"></main>
			</div>
			${this.renderStatusBar()}
		`;
	}
}
