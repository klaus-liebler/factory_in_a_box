// The HMI runtime ("Bedienen & Beobachten"): the project's operating pages with the device's values, full
// screen if wanted, and the management functions of VDI 3814 group 3 - the alarm list (come, go, acknowledge,
// history) and the line recorders. Device: the simulator of this browser or the FCU. Project: the one of
// this browser (the IDE's) or the one stored on the FCU's SD card - so an operator PC needs no IDE: it loads
// the project from the FCU, compiles it for the variable addresses and shows the pages.
import { LitElement, html, nothing } from "lit";
import { customElement, state } from "lit/decorators.js";
import type { DashboardApp } from "../../shell/dashboard-app.ts";
import { setAppStatus } from "../../shell/status-bar.ts";
import type { PlcDevice } from "../ide/device.ts";
import { fcuDevice, simulator } from "../ide/devices.ts";
import { build, loadProject, parseProject, projectHash, type Project } from "../ide/project.ts";
import { AlarmList, PRIO_NAMES, type HmiElement, type HmiPage } from "./hmi.ts";
import { sampleTrends } from "./hmi-trends.ts";
import "./hmi-view.ts";
import "../../styles.css";

const PREFS_KEY = "fcu.hmi.runtime";

@customElement("hmi-runtime-app")
export class HmiRuntimeApp extends LitElement implements DashboardApp {
	@state() private dev: PlcDevice = simulator;
	@state() private source: "browser" | "device" = "browser";
	@state() private project: Project | null = null;
	@state() private pageId = "";
	@state() private tick = 0;
	@state() private notice: string | null = null;
	@state() private showAlarms = true;
	@state() private loading = false;
	private readonly alarms = new AlarmList();
	private shown = false;

	constructor() {
		super();
		try {
			const p = JSON.parse(localStorage.getItem(PREFS_KEY) ?? "{}") as { device?: string; source?: string; page?: string };
			if (p.device === "fcu") this.dev = fcuDevice;
			if (p.source === "device") this.source = "device";
			if (p.page) this.pageId = p.page;
		} catch {
			// defaults
		}
		for (const d of [simulator, fcuDevice]) d.addEventListener("update", () => this.onUpdate(d));
	}

	protected createRenderRoot() {
		return this;
	}

	private savePrefs(): void {
		try {
			localStorage.setItem(PREFS_KEY, JSON.stringify({ device: this.dev.kind === "fcu" ? "fcu" : "sim", source: this.source, page: this.pageId }));
		} catch {
			// for this tab only
		}
	}

	onShow(): void {
		this.shown = true;
		this.dev.activate();
		void this.loadSource();
	}

	onHide(): void {
		this.shown = false;
		this.dev.deactivate();
	}

	private get pages(): HmiPage[] {
		return this.project?.hmi ?? [];
	}

	private onUpdate(d: PlcDevice): void {
		if (!this.shown || d !== this.dev) return;
		sampleTrends(d, this.pages);
		this.alarms.update(this.pages, (e) => this.condition(e), Date.now());
		this.tick++;
	}

	/** An alarm element's condition: its BOOL variable (null: not readable now). */
	private condition(e: HmiElement): boolean | null {
		if (!e.var.trim() || !this.dev.debug || this.dev.state === "NO_PROGRAM") return null;
		const r = this.dev.read(e.var.trim());
		if (!r.symbol || r.value === null) return null;
		return r.value === true || (typeof r.value === "number" && r.value !== 0) || (typeof r.value === "bigint" && r.value !== 0n);
	}

	// ---- project and device -----------------------------------------------------------------------------------
	/** Loads the project from its source and makes its variables known to the device. */
	private async loadSource(): Promise<void> {
		let p: Project | null = null;
		if (this.source === "browser") {
			p = loadProject();
		} else if (this.dev.loadProject) {
			this.loading = true;
			try {
				const text = await this.dev.loadProject();
				if (text) p = parseProject(text);
				else this.flash("Kein Projekt auf dem Gerät (SD-Karte) - siehe Geräteprotokoll der IDE");
			} catch (e) {
				this.flash(`Projekt vom Gerät: ${(e as Error).message}`);
			} finally {
				this.loading = false;
			}
		} else {
			this.flash("Der Simulator hat kein gespeichertes Projekt: Quelle 'dieser Browser'");
			this.source = "browser";
			p = loadProject();
		}
		this.project = p;
		if (p) {
			// the device knows the variable addresses of a program only from a build of the same project
			const b = build(p);
			this.dev.attach(b.result);
			if (!this.pages.some((x) => x.id === this.pageId)) this.pageId = this.pages[0]?.id ?? "";
		}
	}

	private selectDevice(kind: string): void {
		const next = kind === "fcu" ? fcuDevice : simulator;
		if (next === this.dev) return;
		this.dev.deactivate();
		this.dev = next;
		if (this.source === "device" && next.kind !== "fcu") this.source = "browser";
		if (this.shown) next.activate();
		this.savePrefs();
		void this.loadSource();
	}

	private selectSource(s: "browser" | "device"): void {
		this.source = s;
		this.savePrefs();
		void this.loadSource();
	}

	/** Simulator without the project's program: compile, load and start it. */
	private async startSimulator(): Promise<void> {
		if (!this.project) return;
		const b = build(this.project);
		if (!b.ok) {
			this.flash("Das Projekt übersetzt nicht - Fehler in der IDE ansehen");
			return;
		}
		if (await simulator.download(b.result)) await simulator.start();
	}

	// ---- operating ----------------------------------------------------------------------------------------------
	private flash(text: string): void {
		this.notice = text;
		setTimeout(() => {
			if (this.notice === text) this.notice = null;
		}, 3000);
	}

	/** Acknowledges (one or all) and pulses the acknowledge variables: TRUE, after 300 ms FALSE. */
	private ack(key: string | null): void {
		const vars = this.alarms.ack(key, Date.now());
		for (const v of vars) {
			void (async () => {
				const r = this.dev.read(v);
				if (!r.symbol) return this.flash(`Quittiervariable ${v} fehlt im Programm`);
				if (!(await this.dev.write(v, true))) return this.flash(`${v}: Schreiben abgelehnt`);
				setTimeout(() => void this.dev.write(v, false), 300);
			})();
		}
		this.tick++;
	}

	private ackElement(e: HmiElement): void {
		const page = this.pages.find((p) => p.elements.includes(e));
		if (page) this.ack(`${page.id}/${e.id}`);
	}

	private fullscreen(): void {
		const el = this.querySelector(".hmi-rt") as HTMLElement | null;
		if (document.fullscreenElement) void document.exitFullscreen();
		else void el?.requestFullscreen?.().catch(() => this.flash("Vollbild nicht möglich"));
	}

	// ---- rendering ----------------------------------------------------------------------------------------------
	protected willUpdate(): void {
		const d = this.dev;
		const unacked = this.alarms.unacked;
		setAppStatus(this, {
			text: d.connected ? `${d.label}: ${d.state}` : `${d.label} antwortet nicht`,
			kind: !d.connected || d.state === "ERROR" ? "error" : unacked ? "warning" : d.state === "RUN" ? "ok" : "idle",
			extra: html`${unacked ? html`<span class="sb-warn">🚨 ${unacked} unquittiert</span>` : nothing}
				<span class="sb-chip">${this.source === "device" ? "Projekt vom FCU" : "Projekt dieses Browsers"}</span>`,
		});
	}

	render() {
		void this.tick;
		const d = this.dev;
		const page = this.pages.find((p) => p.id === this.pageId) ?? this.pages[0];
		const list = this.alarms.list();
		const stale = this.project && d.projectHash !== 0 && d.projectHash !== projectHash(this.project);
		return html`
			<div class="hmi-rt">
				<div class="hmi-rt-bar">
					<div class="hmi-tabs" role="tablist">
						${this.pages.map((p) => html`<button class="hmi-tab ${p.id === page?.id ? "hmi-tab-active" : ""}" role="tab"
							@click=${() => { this.pageId = p.id; this.savePrefs(); }}>${p.name}</button>`)}
					</div>
					<span class="hmi-grow"></span>
					<span class="hmi-chip ${d.state === "RUN" ? "ok" : d.state === "ERROR" || !d.connected ? "bad" : ""}">${d.connected ? d.state : "offline"}</span>
					${stale ? html`<span class="hmi-chip bad" title="Das Programm auf dem Gerät ist nicht aus diesem Projekt: Variablen können fehlen">Programm ≠ Projekt</span>` : nothing}
					${d.state === "NO_PROGRAM" && d.kind === "simulator" && this.project
						? html`<button class="hmi-small-btn on" @click=${() => void this.startSimulator()}>▶ Projekt im Simulator starten</button>` : nothing}
					<select title="Gerät" @change=${(e: Event) => this.selectDevice((e.target as HTMLSelectElement).value)}>
						<option value="sim" ?selected=${d.kind === "simulator"}>Simulator</option>
						<option value="fcu" ?selected=${d.kind === "fcu"}>FCU</option>
					</select>
					<select title="Woher die Bedienseiten kommen" @change=${(e: Event) => this.selectSource((e.target as HTMLSelectElement).value as "browser" | "device")}>
						<option value="browser" ?selected=${this.source === "browser"}>Projekt dieses Browsers</option>
						<option value="device" ?selected=${this.source === "device"} ?disabled=${d.kind !== "fcu"}>Projekt vom FCU (SD-Karte)</option>
					</select>
					<button class="hmi-small-btn" title="Projekt neu laden" ?disabled=${this.loading} @click=${() => void this.loadSource()}>⟳</button>
					<button class="hmi-small-btn ${this.showAlarms ? "on" : ""}" @click=${() => (this.showAlarms = !this.showAlarms)}>🚨 Meldungen${list.length ? ` (${list.length})` : ""}</button>
					<button class="hmi-small-btn" title="Vollbild" @click=${() => this.fullscreen()}>⛶</button>
				</div>
				<div class="hmi-rt-main">
					<div class="hmi-rt-page">
						${this.loading ? html`<div class="hmi-empty">Projekt wird vom Gerät geladen …</div>`
							: page ? html`<hmi-view .page=${page} .device=${d} mode="run" .tick=${this.tick}
								.alarmState=${(e: HmiElement) => this.alarmOf(page, e)}
								@ack=${(e: CustomEvent<HmiElement>) => this.ackElement(e.detail)}
								@notice=${(e: CustomEvent<string>) => this.flash(e.detail)}></hmi-view>`
							: html`<div class="hmi-empty">Das Projekt hat keine Bedienseiten. In der IDE (SPS → Visualisierung) anlegen.</div>`}
					</div>
					${this.showAlarms ? this.renderAlarms(list) : nothing}
				</div>
				${this.notice ? html`<div class="hmi-toast">${this.notice}</div>` : nothing}
			</div>`;
	}

	private alarmOf(page: HmiPage, e: HmiElement): { active: boolean; acked: boolean } | null {
		const x = this.alarms.entries.get(`${page.id}/${e.id}`);
		return x ? { active: x.active, acked: x.acked } : null;
	}

	private renderAlarms(list: ReturnType<AlarmList["list"]>) {
		const t = (ms: number) => new Date(ms).toLocaleTimeString("de-DE");
		const color = (p: number) => (p <= 3 ? "#e5383b" : p === 4 ? "#f48c06" : p === 5 ? "#e9c46a" : "#4895ef");
		return html`
			<aside class="hmi-rt-alarms" aria-label="Meldungen">
				<h4>Anstehende Meldungen <button class="hmi-small-btn" ?disabled=${this.alarms.unacked === 0} @click=${() => this.ack(null)}>Alle quittieren</button></h4>
				${list.length === 0 ? html`<p class="hmi-history">Keine Meldungen.</p>` : nothing}
				${list.map((x) => html`
					<div class="hmi-alarm-row ${x.acked ? "" : "unacked"}">
						<span class="hmi-prio" style="background:${color(x.prio)}"></span>
						<div>
							<div class="hmi-alarm-title">${x.text}</div>
							<div class="hmi-sub">${PRIO_NAMES[x.prio] ?? `Klasse ${x.prio}`} · ${x.page} · kam ${t(x.came)}${x.went ? ` · ging ${t(x.went)}` : ""}</div>
						</div>
						${x.acked ? html`<span class="hmi-sub">quittiert</span>` : html`<button class="hmi-small-btn" @click=${() => this.ack(x.key)}>Quittieren</button>`}
					</div>`)}
				<h4>Verlauf</h4>
				<div class="hmi-history">
					${this.alarms.history.slice(0, 50).map((h) => html`<div><span>${t(h.time)}</span><span>${h.what}</span><span>${h.text}</span></div>`)}
				</div>
			</aside>`;
	}
}

declare global {
	interface HTMLElementTagNameMap {
		"hmi-runtime-app": HmiRuntimeApp;
	}
}
