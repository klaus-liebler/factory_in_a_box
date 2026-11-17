// The application window, styled like a desktop engineering program: a title bar (quick access, the
// "document" tab with the workspace and the device, connection, notifications, theme, full screen), the
// workspace switcher (the former menu entries are workspaces, like the workspaces of a CAD program) with the
// ribbon of the active workspace (its tabs, groups and commands, published by the app through
// shell/ribbon.ts, plus the shell's "Ansicht" tab), the working area and the status bar.
//
// Nothing here knows the device: the app passes what the title and status bar show (connection, notifications,
// host, logo) as a ShellConfig to Start(). The first registered workspace is the home workspace.
//
// Every app element is created once by app.ts and mounted into .app-shell-content imperatively; render()
// never binds anything into that container, so lit leaves the apps alone however often the shell renders,
// and an app keeps its state while another workspace is shown (only "hidden" changes).
import { LitElement, html, nothing } from "lit";
import { customElement, state, query } from "lit/decorators.js";
import { unsafeHTML } from "lit/directives/unsafe-html.js";
import { Router, type RouteDef } from "./router.ts";
import { hasWorkspaceLifecycle } from "./workspace-app.ts";
import { appStatusOf, onAppStatusChange } from "./status-bar.ts";
import { appRibbonOf, onAppRibbonChange, openWorkspace, type RibbonCommand, type RibbonGroup, type RibbonTab } from "./ribbon.ts";
import { icon, type IconName } from "./icons.ts";
import "./shell.css";

/** A workspace: an app with its place in the switcher. */
export interface WorkspaceDef {
	/** The switcher's section ("Übersicht", "Automatisierung", ...). */
	group: string;
	description: string;
}

export type ConnectionState = "open" | "connecting" | "closed";

/** A connection shown in the title and status bar (the FCU app: the WebSocket to the device). */
export interface ShellConnection {
	state(): ConnectionState;
	onChange(cb: () => void): void;
	/** Tooltip, e.g. "WebSocket-Verbindung zur Control Unit". */
	title: string;
}

/** The bell in the title bar: counts events (e.g. log warnings) until its workspace is opened. */
export interface ShellNotifications {
	/** Events that are already there at the start. */
	initial(): number;
	/** cb(n): n new events. */
	subscribe(cb: (n: number) => void): void;
	/** The workspace that shows them; opening it clears the count. */
	path: string;
	/** Tooltip for n unseen events (0: none). */
	title(n: number): string;
}

/** What the application window shows about the product and the device. */
export interface ShellConfig {
	/** "Factory Control Unit": the logo's tooltip and the end of the page title. */
	productName: string;
	/** The logo as SVG markup. */
	logoSvg?: string;
	/** Shown next to the workspace in the title bar, e.g. "factory-box-303537.local". */
	hostLabel?: string;
	connection?: ShellConnection;
	notifications?: ShellNotifications;
}

interface AppEntry extends RouteDef, WorkspaceDef {
	iconName: IconName;
	element: HTMLElement;
}

const RIBBON_KEY = "fcu.shell.ribbonCollapsed";
const THEME_KEY = "fcu.shell.theme";

@customElement("app-shell")
export class AppShell extends LitElement {
	protected createRenderRoot() {
		return this;
	}

	@state() private activePath = "";
	@state() private switcherOpen = false;
	/** The open menu of a ribbon group (its caption was clicked). */
	@state() private groupMenu: { group: RibbonGroup; left: number; top: number } | null = null;
	/** The selected ribbon tab per workspace. */
	@state() private tabOf = new Map<string, string>();
	@state() private collapsed = AppShell.load(RIBBON_KEY, sessionStorage) === "1";
	@state() private dark = AppShell.loadDark();
	@state() private fullscreen = false;
	/** Notifications (the FCU: warnings and errors in the firmware log) since their workspace was last shown. */
	@state() private unseen = 0;

	private config: ShellConfig = { productName: "" };

	private entries: AppEntry[] = [];
	private router?: Router;
	private activeElement: HTMLElement | null = null;

	@query(".app-shell-content") private contentEl!: HTMLElement;

	private static load(key: string, storage: Storage): string | null {
		try {
			return storage.getItem(key);
		} catch {
			return null;
		}
	}

	private static loadDark(): boolean {
		const saved = AppShell.load(THEME_KEY, localStorage);
		if (saved === "dark" || saved === "light") return saved === "dark";
		return window.matchMedia?.("(prefers-color-scheme: dark)").matches ?? false;
	}

	private setDark(on: boolean): void {
		this.dark = on;
		try {
			localStorage.setItem(THEME_KEY, on ? "dark" : "light");
		} catch {
			/* for this page only */
		}
		// canvases are drawn by script: let the apps draw again
		this.updateComplete.then(() => window.dispatchEvent(new Event("resize")));
	}

	private setCollapsed(on: boolean): void {
		this.collapsed = on;
		try {
			sessionStorage.setItem(RIBBON_KEY, on ? "1" : "0");
		} catch {
			/* for this page only */
		}
		this.updateComplete.then(() => window.dispatchEvent(new Event("resize")));
	}

	private toggleFullscreen(): void {
		if (document.fullscreenElement) void document.exitFullscreen?.().catch(() => undefined);
		else void document.documentElement.requestFullscreen?.().catch(() => undefined);
	}

	/** Called by app.ts for every app before Start(); the order is the order in the switcher. */
	public RegisterApp(path: string, title: string, iconName: IconName, element: HTMLElement, ws: WorkspaceDef): void {
		element.hidden = true;
		this.entries.push({ path, title, icon: "", iconName, element, ...ws });
	}

	public Start(config: ShellConfig): void {
		this.config = config;
		config.connection?.onChange(() => this.requestUpdate());
		onAppStatusChange(() => this.requestUpdate());
		onAppRibbonChange(() => this.requestUpdate());
		document.addEventListener("fullscreenchange", () => (this.fullscreen = !!document.fullscreenElement));
		window.addEventListener("keydown", (e) => {
			if (e.key === "Escape" && (this.switcherOpen || this.groupMenu)) {
				this.switcherOpen = false;
				this.groupMenu = null;
			}
		});
		const notes = config.notifications;
		if (notes) {
			notes.subscribe((n) => {
				if (this.activePath !== notes.path && n) this.unseen += n;
			});
			this.unseen = notes.initial();
		}
		this.router = new Router(this.entries.map(({ path, title, icon }) => ({ path, title, icon })));
		this.router.addEventListener("change", () => this.sync());
		this.requestUpdate();
		this.updateComplete.then(() => {
			for (const entry of this.entries) this.contentEl.appendChild(entry.element);
			this.sync();
		});
	}

	private sync(): void {
		if (!this.router) return;
		const match = this.router.MatchCurrent();
		const next = this.entries.find((e) => e.path === match.path) ?? this.entries[0];
		if (!next || next.element === this.activeElement) return;
		if (this.activeElement && hasWorkspaceLifecycle(this.activeElement)) this.activeElement.onHide();
		this.activeElement = next.element;
		this.activeElement.hidden = false;
		if (hasWorkspaceLifecycle(this.activeElement)) this.activeElement.onShow();
		this.activePath = next.path;
		this.switcherOpen = false;
		this.groupMenu = null;
		if (next.path === this.config.notifications?.path) this.unseen = 0;
		for (const entry of this.entries) if (entry.element !== this.activeElement) entry.element.hidden = true;
		document.title = this.config.productName ? `${next.title} – ${this.config.productName}` : next.title;
	}

	private go(path: string): void {
		this.switcherOpen = false;
		openWorkspace(path);
	}

	protected updated(): void {
		this.classList.toggle("shell-ribbon-collapsed", this.collapsed);
		document.documentElement.dataset.theme = this.dark ? "dark" : "light";
	}

	/** The first registered workspace. */
	private get home(): AppEntry | undefined {
		return this.entries[0];
	}

	// ---- ribbon --------------------------------------------------------------------------------------------
	/** The shell's own tab, after the workspace's tabs. */
	private viewTab(): RibbonTab {
		return {
			id: "shell-view", label: "Ansicht",
			groups: [
				{ label: "Navigation", commands: [
					...(this.home ? [{ id: "home", label: this.home.title, icon: this.home.iconName, run: () => this.go(this.home!.path), disabled: this.activePath === this.home.path }] : []),
					{ id: "back", label: "Zurück", icon: "left", size: "small", run: () => history.back() },
					{ id: "forward", label: "Vor", icon: "right", size: "small", run: () => history.forward() },
				] },
				{ label: "Darstellung", commands: [
					{ id: "theme", label: this.dark ? "Hell" : "Dunkel", icon: "theme", title: "Helle/dunkle Darstellung", run: () => this.setDark(!this.dark) },
					{ id: "collapse", label: "Ribbon einklappen", icon: "collapse", active: this.collapsed, title: "Nur die Reiter zeigen (Doppelklick auf einen Reiter)", run: () => this.setCollapsed(!this.collapsed) },
				] },
				{ label: "Fenster", commands: [
					{ id: "fullscreen", label: "Vollbild", icon: "fullscreen", active: this.fullscreen, run: () => this.toggleFullscreen() },
				] },
			],
		};
	}

	private tabs(): RibbonTab[] {
		return [...appRibbonOf(this.activeElement), this.viewTab()];
	}

	private selectTab(id: string): void {
		const next = new Map(this.tabOf);
		next.set(this.activePath, id);
		this.tabOf = next;
		if (this.collapsed) this.setCollapsed(false);
	}

	private renderCommand(c: RibbonCommand, small = false) {
		return html`<button class="rb-cmd ${small ? "rb-small" : "rb-large"} ${c.active ? "rb-active" : ""}" title=${c.title ?? c.label}
			?disabled=${c.disabled} aria-pressed=${c.active === undefined ? nothing : String(c.active)}
			@click=${() => { this.groupMenu = null; c.run(); }}>
			${icon(c.icon, small ? 16 : 32)}<span class="rb-label">${c.label}</span></button>`;
	}

	/** Large commands as columns, small ones in columns of three. */
	private renderGroupBody(g: RibbonGroup) {
		const cols: unknown[] = [];
		let smalls: RibbonCommand[] = [];
		const flush = () => {
			if (smalls.length) cols.push(html`<div class="rb-smallcol">${smalls.map((c) => this.renderCommand(c, true))}</div>`);
			smalls = [];
		};
		for (const c of g.commands) {
			if (c.size === "small") {
				smalls.push(c);
				if (smalls.length === 3) flush();
			} else {
				flush();
				cols.push(this.renderCommand(c));
			}
		}
		flush();
		return cols;
	}

	private openGroupMenu(e: MouseEvent, g: RibbonGroup): void {
		const r = (e.currentTarget as HTMLElement).getBoundingClientRect();
		this.groupMenu = this.groupMenu?.group.label === g.label ? null : { group: g, left: r.left, top: r.bottom + 2 };
	}

	private renderRibbon(entry: AppEntry | undefined) {
		const tabs = this.tabs();
		const activeId = this.tabOf.get(this.activePath);
		const tab = tabs.find((t) => t.id === activeId) ?? tabs[0];
		return html`
			<div class="rb">
				<button class="rb-switcher" title="Arbeitsbereich wechseln" aria-haspopup="menu" aria-expanded=${this.switcherOpen}
					@click=${() => (this.switcherOpen = !this.switcherOpen)}>
					<span class="rb-switcher-icon">${entry ? icon(entry.iconName, 28) : nothing}</span>
					<span class="rb-switcher-name">${entry?.title ?? ""}</span>
					<span class="rb-caret">▾</span>
				</button>
				<div class="rb-main">
					<div class="rb-tabs" role="tablist">
						${tabs.map((t) => html`<button role="tab" class="rb-tab ${t === tab ? "rb-tab-active" : ""}" aria-selected=${t === tab}
							@click=${() => this.selectTab(t.id)} @dblclick=${() => this.setCollapsed(!this.collapsed)}>${t.label}</button>`)}
					</div>
					${this.collapsed ? nothing : html`
						<div class="rb-groups" role="toolbar" aria-label=${tab.label}>
							${tab.groups.map((g) => html`
								<div class="rb-group">
									<div class="rb-group-body">${this.renderGroupBody(g)}</div>
									<button class="rb-group-caption" title="Alle Befehle der Gruppe" @click=${(e: MouseEvent) => this.openGroupMenu(e, g)}>${g.label} ▾</button>
								</div>`)}
						</div>`}
				</div>
			</div>`;
	}

	private renderSwitcher() {
		const groups = [...new Set(this.entries.map((e) => e.group))];
		return html`
			<div class="ws-scrim" @click=${() => (this.switcherOpen = false)}></div>
			<div class="ws-switcher-menu" role="menu" aria-label="Arbeitsbereiche">
				${groups.map((g) => html`
					<div class="ws-menu-group">${g}</div>
					${this.entries.filter((e) => e.group === g).map((e) => html`
						<button role="menuitemradio" aria-checked=${e.path === this.activePath} class="ws-menu-item ${e.path === this.activePath ? "ws-menu-active" : ""}"
							@click=${() => this.go(e.path)}>
							${icon(e.iconName, 28)}
							<span class="ws-menu-text"><span class="ws-menu-title">${e.title}</span><span class="ws-menu-desc">${e.description}</span></span>
							${e.path === this.activePath ? html`<span class="ws-menu-check">✓</span>` : nothing}
						</button>`)}`)}
			</div>`;
	}

	private renderGroupMenu() {
		const m = this.groupMenu!;
		const left = Math.min(m.left, window.innerWidth - 240);
		return html`
			<div class="ws-scrim" @click=${() => (this.groupMenu = null)}></div>
			<div class="rb-menu" role="menu" style="left:${Math.max(4, left)}px;top:${m.top}px">
				${m.group.commands.map((c) => html`
					<button role="menuitem" class="rb-menu-item ${c.active ? "rb-active" : ""}" ?disabled=${c.disabled} title=${c.title ?? c.label}
						@click=${() => { this.groupMenu = null; c.run(); }}>${icon(c.icon, 20)}<span>${c.label}</span>${c.active ? html`<span class="ws-menu-check">✓</span>` : nothing}</button>`)}
			</div>`;
	}

	private static connText(s: ConnectionState): string {
		return s === "open" ? "verbunden" : s === "connecting" ? "verbinde …" : "getrennt";
	}

	private renderTitleBar(entry: AppEntry | undefined) {
		const { productName, logoSvg, hostLabel, connection, notifications } = this.config;
		const ws = connection?.state();
		const home = this.home;
		return html`
			<header class="tb">
				${logoSvg ? html`<span class="tb-logo" title=${productName}>${unsafeHTML(logoSvg)}</span>` : nothing}
				<div class="tb-quick">
					${home ? html`<button class="tb-btn" title=${home.title} @click=${() => this.go(home.path)}>${icon("home", 20)}</button>` : nothing}
					<button class="tb-btn" title="Zurück" @click=${() => history.back()}>${icon("left", 18)}</button>
					<button class="tb-btn" title="Vor" @click=${() => history.forward()}>${icon("right", 18)}</button>
				</div>
				<div class="tb-doc" title="${entry?.title ?? ""}${hostLabel ? ` · ${hostLabel}` : ""}">
					${entry ? icon(entry.iconName, 18) : nothing}
					<span class="tb-doc-title">${entry?.title ?? ""}</span>
					${hostLabel ? html`<span class="tb-doc-host">${hostLabel}</span>` : nothing}
				</div>
				<div class="tb-right">
					${connection && ws ? html`<span class="tb-conn tb-conn-${ws}" title=${connection.title}><span class="tb-dot"></span>${AppShell.connText(ws)}</span>` : nothing}
					${notifications ? html`<button class="tb-btn tb-bell" title=${notifications.title(this.unseen)}
						@click=${() => this.go(notifications.path)}>${icon("bell", 20)}${this.unseen ? html`<span class="tb-badge">${this.unseen > 99 ? "99+" : this.unseen}</span>` : nothing}</button>` : nothing}
					<button class="tb-btn" title=${this.dark ? "Helle Darstellung" : "Dunkle Darstellung"} @click=${() => this.setDark(!this.dark)}>${icon("theme", 20)}</button>
					<button class="tb-btn ${this.fullscreen ? "tb-on" : ""}" title="Vollbild" @click=${() => this.toggleFullscreen()}>${icon("fullscreen", 20)}</button>
				</div>
			</header>`;
	}

	private renderStatusBar() {
		const connection = this.config.connection;
		const ws = connection?.state();
		const status = appStatusOf(this.activeElement);
		return html`
			<footer class="shell-status" role="status">
				${connection && ws ? html`<span class="sb-ws sb-ws-${ws}" title=${connection.title}><span class="sb-dot"></span>${AppShell.connText(ws)}</span>` : nothing}
				${status && status.text ? html`<span class="sb-mode sb-mode-${status.kind}">${status.text}</span>` : ""}
				${status?.extra ?? ""}
			</footer>`;
	}

	protected render() {
		const entry = this.entries.find((e) => e.path === this.activePath);
		return html`
			${this.renderTitleBar(entry)}
			${this.renderRibbon(entry)}
			${this.switcherOpen ? this.renderSwitcher() : nothing}
			${this.groupMenu ? this.renderGroupMenu() : nothing}
			<main class="app-shell-content"></main>
			${this.renderStatusBar()}`;
	}
}
