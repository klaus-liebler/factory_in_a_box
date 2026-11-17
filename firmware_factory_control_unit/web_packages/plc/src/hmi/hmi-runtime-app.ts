// The HMI runtime ("Bedienen & Beobachten"): the project's operating pages with the device's values, full
// screen if wanted, and the management functions of VDI 3814 group 3 - the alarm list (come, go, acknowledge,
// history) and the line recorders. Device: the simulator of this browser or the FCU. Project: the one of
// this browser (the IDE's) or the one stored on the FCU's SD card - so an operator PC needs no IDE: it loads
// the project from the FCU, compiles it for the variable addresses and shows the pages.
import { LitElement, html, nothing } from "lit";
import { customElement, state } from "lit/decorators.js";
import type { WorkspaceApp } from "@fiab/ui-shell/shell/workspace-app.ts";
import { setAppStatus } from "@fiab/ui-shell/shell/status-bar.ts";
import { setAppRibbon } from "@fiab/ui-shell/shell/ribbon.ts";
import { clock } from "@fiab/ui-shell/ui/format.ts";
import type { PlcDevice } from "../ide/device.ts";
import { remoteDevice, simulator } from "../ide/devices.ts";
import { build, loadProject, parseProject, projectHash, type Project } from "../ide/project.ts";
import { AlarmList, PRIO_NAMES, prioColor, type HmiElement, type HmiPage } from "./hmi.ts";
import { sampleTrends } from "./hmi-trends.ts";
import "./hmi-view.ts";

const PREFS_KEY = "fcu.hmi.runtime";

@customElement("hmi-runtime-app")
export class HmiRuntimeApp extends LitElement implements WorkspaceApp {
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
			const remote = remoteDevice();
			if (p.device === "fcu" && remote) this.dev = remote;
			if (p.source === "device" && this.dev.kind === "fcu") this.source = "device";
			if (p.page) this.pageId = p.page;
		} catch {
			// defaults
		}
		for (const d of [simulator, remoteDevice()]) d?.addEventListener("update", () => this.onUpdate(d));
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
		const next = (kind === "fcu" && remoteDevice()) || simulator;
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
	private get stale(): boolean {
		return !!this.project && this.dev.projectHash !== 0 && this.dev.projectHash !== projectHash(this.project);
	}

	protected willUpdate(): void {
		const d = this.dev;
		const unacked = this.alarms.unacked;
		setAppStatus(this, {
			text: d.connected ? `${d.label}: ${d.state}` : `${d.label} antwortet nicht`,
			kind: !d.connected || d.state === "ERROR" ? "error" : unacked ? "warning" : d.state === "RUN" ? "ok" : "idle",
			extra: html`${unacked ? html`<span class="sb-warn">🚨 ${unacked} unquittiert</span>` : nothing}
				${this.stale ? html`<span class="sb-warn" title="Das Programm auf dem Gerät ist nicht aus diesem Projekt: Variablen können fehlen">⚠ Programm ≠ Projekt</span>` : nothing}
				<span class="sb-chip">${this.source === "device" ? "Projekt vom FCU" : "Projekt dieses Browsers"}</span>`,
		});
		const page = this.pages.find((p) => p.id === this.pageId) ?? this.pages[0];
		setAppRibbon(this, [
			{ id: "operate", label: "Bedienen", groups: [
				{ label: "Seiten", commands: this.pages.length
					? this.pages.map((p) => ({ id: `page-${p.id}`, label: p.name, icon: "page" as const, active: p === page,
						run: () => { this.pageId = p.id; this.savePrefs(); } }))
					: [{ id: "none", label: "Keine Seiten", icon: "page" as const, disabled: true, run: () => undefined }] },
				{ label: "Meldungen", commands: [
					{ id: "alarms", label: "Meldeliste", icon: "alarm", active: this.showAlarms, run: () => (this.showAlarms = !this.showAlarms) },
					{ id: "ack-all", label: "Alle quittieren", icon: "ack", disabled: unacked === 0, run: () => this.ack(null) },
				] },
				{ label: "Darstellung", commands: [
					{ id: "present", label: "Präsentation", icon: "fullscreen", title: "Nur die Bedienseite im Vollbild", run: () => this.fullscreen() },
				] },
			] },
			{ id: "connect", label: "Verbindung", groups: [
				{ label: "Gerät", commands: [
					{ id: "sim", label: "Simulator", icon: "simulator", active: d.kind === "simulator", run: () => this.selectDevice("sim") },
					...(remoteDevice() ? [{ id: "fcu", label: remoteDevice()!.label, icon: "device" as const, active: d.kind === "fcu", run: () => this.selectDevice("fcu") }] : []),
				] },
				{ label: "Projekt", commands: [
					{ id: "src-browser", label: "Dieser Browser", icon: "open", active: this.source === "browser", title: "Das Projekt der IDE in diesem Browser", run: () => this.selectSource("browser") },
					...(remoteDevice() ? [{ id: "src-device", label: "Vom FCU", icon: "from-device" as const, active: this.source === "device", disabled: d.kind !== "fcu",
						title: "Das beim letzten Laden auf der SD-Karte der FCU gespeicherte Projekt", run: () => this.selectSource("device") }] : []),
					{ id: "reload", label: "Neu laden", icon: "refresh", size: "small", disabled: this.loading, run: () => void this.loadSource() },
					{ id: "start-sim", label: "Im Simulator starten", icon: "download-run", size: "small",
						disabled: d.kind !== "simulator" || !this.project, title: "Das Projekt übersetzen, in den Simulator laden und starten", run: () => void this.startSimulator() },
				] },
			] },
		]);
	}

	render() {
		void this.tick;
		const d = this.dev;
		const page = this.pages.find((p) => p.id === this.pageId) ?? this.pages[0];
		const list = this.alarms.list();
		return html`
			<div class="hmi-rt">
				${d.state === "NO_PROGRAM" && d.kind === "simulator" && this.project ? html`
					<div class="hmi-rt-hint">Im Simulator läuft kein Programm.
						<button class="hmi-small-btn on" @click=${() => void this.startSimulator()}>▶ Projekt im Simulator starten</button></div>` : nothing}
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
		const t = (ms: number) => clock(ms);
		return html`
			<aside class="hmi-rt-alarms" aria-label="Meldungen">
				<h4>Anstehende Meldungen <button class="hmi-small-btn" ?disabled=${this.alarms.unacked === 0} @click=${() => this.ack(null)}>Alle quittieren</button></h4>
				${list.length === 0 ? html`<p class="hmi-history">Keine Meldungen.</p>` : nothing}
				${list.map((x) => html`
					<div class="hmi-alarm-row ${x.acked ? "" : "unacked"}">
						<span class="hmi-prio" style="background:${prioColor(x.prio)}"></span>
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
