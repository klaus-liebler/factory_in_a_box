// The dashboard workspace (the start page): tiles with the device's state at a glance - identity and
// uptime, temperature, CPU load, memory, supply, network, PLC, actuators and sensors, drives, buses, the
// latest log events - each with a link into its workspace. Polls only while it is shown: registers every
// second, tasks every 2 s, PLC status every 2 s, system information every 10 s.
import { LitElement, html, nothing } from "lit";
import { customElement, state } from "lit/decorators.js";
import type { DashboardApp } from "../shell/dashboard-app.ts";
import { setAppStatus } from "../shell/status-bar.ts";
import { openWorkspace, setAppRibbon } from "../shell/ribbon.ts";
import { icon, type IconName } from "../shell/icons.ts";
import { fetchRegisters, healthProblems, regValue, type RegisterValues } from "../core/registers.ts";
import { Poller, pollingGroup } from "../core/poller.ts";
import { cString, clock, formatUptime } from "../core/format.ts";
import { fetchPlcStatus, isIec, PLC_STATE_NAMES, type PlcStatus } from "../core/plc.ts";
import { fetchSystemInfo, RESET_CAUSE_NAMES, type SystemInfo } from "../core/system.ts";
import { system, tasks } from "../../generated/ws-protocol.ts";
import { registerWsProtocolListener, sendBinary } from "../ws-client.ts";
import { logEntries, subscribeLog, type LogEntry } from "../log-store.ts";
import * as buildInfo from "../../generated/build-info.ts";
import { i2cBusSections } from "../i2c-scan.ts";
import { ring, sparkline } from "../ui/charts.ts";
import { COLORS } from "../ui/colors.ts";
import "./overview.css";

const HISTORY = 120;

const tempColor = (t: number) => (t >= 80 ? COLORS.error : t >= 65 ? COLORS.warn : COLORS.info);
const loadColor = (f: number) => (f >= 0.85 ? COLORS.error : f >= 0.6 ? COLORS.warn : COLORS.accent);

@customElement("overview-app")
export class OverviewApp extends LitElement implements DashboardApp {
	@state() private regs: RegisterValues | null = null;
	@state() private info: SystemInfo | null = null;
	@state() private taskList: tasks.TaskInfo[] = [];
	@state() private pools: tasks.PoolInfo[] = [];
	@state() private plcStatus: PlcStatus | null = null;
	@state() private events: LogEntry[] = [];

	private readonly hist = { temp: [] as number[], load: [] as number[], power: [] as number[], rx: [] as number[] };
	private lastRx: { t: number; rx: number; tx: number } | null = null;
	private rates = { rx: 0, tx: 0 };
	private taskAnswers = 0;
	private unlisten: (() => void) | null = null;
	private unlog: (() => void) | null = null;

	/** The registers drive the status bar; the others follow their own pace. */
	private readonly registerPoll = new Poller(this, () => this.pollRegisters(), 1000);
	private readonly pollers = [
		this.registerPoll,
		new Poller(this, () => {
			sendBinary(tasks.TaskManagerRequest.encode({ requestId: 0 }));
			sendBinary(tasks.PoolListRequest.encode({ requestId: 0 }));
		}, 2000),
		new Poller(this, async () => void (this.plcStatus = await fetchPlcStatus().catch(() => null)), 2000),
		new Poller(this, async () => void (this.info = await fetchSystemInfo()), 10_000),
	];

	protected createRenderRoot() {
		return this;
	}

	onShow(): void {
		this.events = this.recentEvents(logEntries());
		this.unlog = subscribeLog(() => (this.events = this.recentEvents(logEntries())));
		this.unlisten = registerWsProtocolListener(tasks.NAMESPACE_ID, {
			onWsMessage: (type, view) => {
				if (type === tasks.TaskListMessage.TYPE_ID) {
					this.taskList = tasks.TaskListMessage.decode(view, 0).items;
					if (++this.taskAnswers >= 2) this.push("load", this.cpuLoad());
				} else if (type === tasks.PoolListMessage.TYPE_ID) {
					this.pools = tasks.PoolListMessage.decode(view, 0).pools;
				}
			},
		});
		for (const p of this.pollers) p.start();
	}

	onHide(): void {
		for (const p of this.pollers) p.stop();
		this.unlisten?.();
		this.unlog?.();
		this.unlisten = this.unlog = null;
	}

	private push(key: keyof OverviewApp["hist"], v: number): void {
		const a = this.hist[key];
		a.push(v);
		if (a.length > HISTORY) a.shift();
	}

	private async pollRegisters(): Promise<void> {
		const v = await fetchRegisters();
		this.regs = v;
		const t = regValue(v, "CPU_TEMPERATURE_C");
		if (t !== undefined) this.push("temp", t);
		const p = regValue(v, "PWR_POWER_MW");
		if (p !== undefined) this.push("power", p / 1000);
		const rx = regValue(v, "ETH_RX_COUNT"), tx = regValue(v, "ETH_TX_COUNT");
		if (rx !== undefined && tx !== undefined) {
			const now = Date.now();
			if (this.lastRx && now > this.lastRx.t) {
				const dt = (now - this.lastRx.t) / 1000;
				this.rates = { rx: Math.max(0, (rx - this.lastRx.rx) / dt), tx: Math.max(0, (tx - this.lastRx.tx) / dt) };
				this.push("rx", this.rates.rx + this.rates.tx);
			}
			this.lastRx = { t: now, rx, tx };
		}
	}

	private get failed(): boolean {
		return this.registerPoll.state === "error";
	}

	/** Total CPU load 0..1: everything but the idle thread. */
	private cpuLoad(): number {
		const busy = this.taskList.filter((t) => !/idle/i.test(cString(t.name))).reduce((s, t) => s + t.cpuPermille, 0);
		return Math.min(1, busy / 1000);
	}

	private recentEvents(all: readonly LogEntry[]): LogEntry[] {
		const out: LogEntry[] = [];
		for (let i = all.length - 1; i >= 0 && out.length < 6; i--) if (all[i].level >= system.LogLevel.LOG_WARN) out.push(all[i]);
		return out;
	}

	// ---- rendering ---------------------------------------------------------------------------------------------
	protected willUpdate(): void {
		setAppStatus(this, this.registerPoll.status());
		setAppRibbon(this, [
			{ id: "overview", label: "Übersicht", groups: [
				pollingGroup(this.pollers, "die Live-Daten"),
				{ label: "Arbeitsbereiche", commands: [
					{ id: "plc", label: "SPS-Entwicklung", icon: "plc", run: () => openWorkspace("/plc") },
					{ id: "hmi", label: "Bedienen & Beobachten", icon: "hmi", run: () => openWorkspace("/hmi") },
					{ id: "actuators", label: "Aktorentest", icon: "actuator", run: () => openWorkspace("/actuators") },
					{ id: "registers", label: "Register", icon: "registers", size: "small", run: () => openWorkspace("/registers") },
					{ id: "system", label: "System", icon: "chip", size: "small", run: () => openWorkspace("/system") },
					{ id: "tasks", label: "Task Manager", icon: "tasks", size: "small", run: () => openWorkspace("/tasks") },
				] },
			] },
		]);
	}

	private tile(title: string, ic: IconName, body: unknown, opts: { link?: string; wide?: boolean; tone?: string } = {}) {
		return html`
			<section class="ov-tile ${opts.wide ? "ov-wide" : ""} ${opts.tone ?? ""}">
				<header class="ov-tile-head">
					${icon(ic, 20)}<span class="ov-tile-title">${title}</span>
					${opts.link ? html`<button class="ov-link" title="Arbeitsbereich öffnen" @click=${() => openWorkspace(opts.link!)}>öffnen ›</button>` : nothing}
				</header>
				<div class="ov-tile-body">${body}</div>
			</section>`;
	}

	private kv(label: string, value: unknown) {
		return html`<div class="ov-kv"><span>${label}</span><b>${value}</b></div>`;
	}

	private led(on: boolean | undefined, label: string, color = "green") {
		return html`<span class="led-item"><span class="led ${on === undefined ? "unknown" : on ? `on ${color}` : ""}"></span>${label}</span>`;
	}

	render() {
		const r = this.regs;
		const v = (n: string) => regValue(r, n);
		return html`
			<div class="ov">
				${this.renderHero()}
				<div class="ov-grid">
					${this.renderTemperature(v("CPU_TEMPERATURE_C"))}
					${this.renderCpu()}
					${this.renderMemory()}
					${this.renderPower(v)}
					${this.renderNetwork(v)}
					${this.renderPlc()}
					${this.renderIo(v)}
					${this.renderDrives(v)}
					${this.renderBuses(v)}
					${this.renderEvents()}
				</div>
			</div>`;
	}

	private renderHero() {
		const i = this.info;
		const ver = `${buildInfo.FW_VERSION_MAJOR}.${buildInfo.FW_VERSION_MINOR}.${buildInfo.FW_VERSION_PATCH}`;
		const health = regValue(this.regs, "HEALTH_STATE");
		const quick: [string, IconName, string][] = [["/plc", "plc", "SPS"], ["/hmi", "hmi", "Bedienen"], ["/registers", "registers", "Register"], ["/terminal", "terminal", "Terminal"]];
		return html`
			<section class="ov-hero">
				<div class="ov-hero-id">
					<div class="ov-hero-name">${buildInfo.DEVICE_HOSTNAME}<span>.local</span></div>
					<div class="ov-hero-sub">${buildInfo.BOARD_NAME} · Firmware ${ver} · ${buildInfo.GIT_COMMIT_HASH.slice(0, 8)}${buildInfo.GIT_IS_DIRTY ? "+" : ""}</div>
					<div class="ov-hero-chips">
						<span class="ov-chip ${this.failed ? "bad" : this.regs ? "ok" : ""}">${this.failed ? "offline" : this.regs ? "online" : "verbinde …"}</span>
						${i ? html`<span class="ov-chip">IP ${i.ipAddress.join(".")}</span>` : nothing}
						${health !== undefined ? html`<span class="ov-chip ${healthProblems(health).length ? "warn" : "ok"}" title="Health-Bitfeld 0x${health.toString(16).padStart(4, "0")}">${healthProblems(health).length ? healthProblems(health).join(", ") : "gesund"}</span>` : nothing}
						${i ? html`<span class="ov-chip" title="Ursache des letzten Resets">Reset: ${RESET_CAUSE_NAMES[i.resetCauseCode] ?? "?"}</span>` : nothing}
					</div>
				</div>
				<div class="ov-hero-uptime">
					<span class="ov-hero-label">Betriebszeit</span>
					<span class="ov-hero-big">${i ? formatUptime(i.uptimeSeconds) : "–"}</span>
				</div>
				<div class="ov-hero-quick">
					${quick.map(([p, ic, l]) => html`<button class="ov-quick" @click=${() => openWorkspace(p)}>${icon(ic, 28)}<span>${l}</span></button>`)}
				</div>
			</section>`;
	}

	private renderTemperature(t: number | undefined) {
		const h = this.hist.temp;
		return this.tile("CPU-Temperatur", "temperature", html`
			<div class="ov-row">
				${ring(t === undefined ? 0 : (t - 20) / 80, tempColor(t ?? 0), t === undefined ? "–" : `${t} °C`, "Chip-Sensor")}
				<div class="ov-col">
					${this.kv("Minimum", h.length ? `${Math.min(...h)} °C` : "–")}
					${this.kv("Maximum", h.length ? `${Math.max(...h)} °C` : "–")}
				</div>
			</div>
			${sparkline(h, { color: tempColor(t ?? 0), capacity: HISTORY })}`, { link: "/system" });
	}

	private renderCpu() {
		const load = this.taskAnswers >= 2 ? this.cpuLoad() : null;
		const top = [...this.taskList].filter((t) => !/idle/i.test(cString(t.name))).sort((a, b) => b.cpuPermille - a.cpuPermille).slice(0, 3);
		return this.tile("CPU-Last", "chip", html`
			<div class="ov-row">
				${ring(load ?? 0, loadColor(load ?? 0), load === null ? "–" : `${Math.round(load * 100)} %`, `${this.taskList.length} Tasks`)}
				<div class="ov-col ov-bars">
					${top.map((t) => html`<div class="ov-bar-row"><span>${cString(t.name)}</span>
						<span class="meter"><span style="width:${Math.min(100, t.cpuPermille / 10)}%"></span></span><b>${(t.cpuPermille / 10).toFixed(1)} %</b></div>`)}
				</div>
			</div>
			${sparkline(this.hist.load.map((x) => x * 100), { color: COLORS.accent, min: 0, max: 100, capacity: HISTORY })}`, { link: "/tasks" });
	}

	private renderMemory() {
		const heap = this.info?.freeHeapBytes;
		const worstStack = this.taskList.reduce((m, t) => Math.max(m, t.stackSizeBytes ? t.stackUsedHighWaterBytes / t.stackSizeBytes : 0), 0);
		return this.tile("Speicher", "memory", html`
			${this.kv("Freier Heap", heap === undefined ? "–" : `${(heap / 1024).toFixed(1)} KiB`)}
			${this.kv("Höchste Stack-Auslastung", this.taskList.length ? `${Math.round(worstStack * 100)} %` : "–")}
			${this.pools.map((p) => {
				const used = p.capacity ? 1 - p.free / p.capacity : 0;
				const peak = p.capacity ? 1 - p.minFree / p.capacity : 0;
				return html`<div class="ov-pool"><span>${cString(p.name)}</span>
					<span class="meter"><span style="width:${used * 100}%"></span><i style="left:${peak * 100}%" title="Spitze"></i></span>
					<b>${Math.round(used * 100)} %</b></div>`;
			})}`, { link: "/tasks" });
	}

	private renderPower(v: (n: string) => number | undefined) {
		const u = v("PWR_BUS_VOLTAGE_MV"), i = v("PWR_CURRENT_MA"), p = v("PWR_POWER_MW");
		const pdU = v("PWR_PD_VOLTAGE_MV"), pdI = v("PWR_PD_CURRENT_MA"), pd = v("PWR_PD_STATUS");
		const pdText = pd === 0 ? `${((pdU ?? 0) / 1000).toFixed(0)} V / ${((pdI ?? 0) / 1000).toFixed(1)} A` : pd === 1 ? "verbunden, Spannung nicht einstellbar" : pd === 2 ? "kein USB-PD-Netzteil" : "–";
		return this.tile("Versorgung", "power", html`
			<div class="ov-metrics">
				<div><span>Spannung</span><b>${u === undefined ? "–" : (u / 1000).toFixed(2)}</b><small>V</small></div>
				<div><span>Strom</span><b>${i === undefined ? "–" : (i / 1000).toFixed(3)}</b><small>A</small></div>
				<div><span>Leistung</span><b>${p === undefined ? "–" : (p / 1000).toFixed(2)}</b><small>W</small></div>
			</div>
			${sparkline(this.hist.power, { color: COLORS.ledAmber, capacity: HISTORY })}
			${this.kv("USB-PD", pdText)}`, { link: "/power" });
	}

	private renderNetwork(v: (n: string) => number | undefined) {
		const up = v("ETH_LINK_STATUS"), speed = v("ETH_LINK_SPEED"), duplex = v("ETH_LINK_DUPLEX"), err = v("ETH_ERROR_COUNT");
		return this.tile("Netzwerk", "network", html`
			<div class="ov-row ov-row-tight">
				${this.led(up === undefined ? undefined : up !== 0, up ? `Link ${speed} Mbit/s ${duplex ? "voll" : "halb"}duplex` : "kein Link")}
			</div>
			${this.kv("IP-Adresse", this.info ? this.info.ipAddress.join(".") : "–")}
			${this.kv("Empfangen / Gesendet", `${this.rates.rx.toFixed(0)} / ${this.rates.tx.toFixed(0)} Pakete/s`)}
			${this.kv("Fehler", err ?? "–")}
			${sparkline(this.hist.rx, { color: COLORS.info, min: 0, capacity: HISTORY })}`, { link: "/system" });
	}

	private renderPlc() {
		const s = this.plcStatus;
		const st = s ? PLC_STATE_NAMES[s.state] ?? "?" : "–";
		const tone = !s ? "" : s.state === 2 ? "ov-ok" : s.state === 3 ? "ov-bad" : s.state === 4 ? "ov-warn" : "";
		return this.tile("SPS (IEC 61131-3)", "plc", html`
			<div class="ov-plc-state ${tone}">${st}</div>
			${this.kv("Ausgänge schaltet", s ? (isIec(s) ? "das IEC-Programm" : "der Feldbus") : "–")}
			${this.kv("Zyklus", s ? `${s.cycleMs} ms · ${s.cycles.toLocaleString("de-DE")}` : "–")}
			${this.kv("Programm", s && s.imageBytes ? `${(s.imageBytes / 1024).toFixed(1)} KiB${s.autostart ? " · Autostart" : ""}` : "keins geladen")}`, { link: "/plc" });
	}

	private renderIo(v: (n: string) => number | undefined) {
		const b = (n: string) => { const x = v(n); return x === undefined ? undefined : x !== 0; };
		const pwm = (n: string) => v(n) ?? 0;
		return this.tile("Aktoren & Sensoren", "actuator", html`
			<div class="ov-leds">
				${this.led(b("VALVE1"), "Ventil 1", "amber")}${this.led(b("VALVE2"), "Ventil 2", "amber")}${this.led(b("VALVE3"), "Ventil 3", "amber")}
				${this.led(b("VACUUM_PUMP"), "Vakuumpumpe", "amber")}${this.led(b("VACUUM_VALVE"), "Vakuumventil", "amber")}
				${this.led(b("LIGHTBARRIER1"), "Lichtschranke 1")}${this.led(b("LIGHTBARRIER2"), "Lichtschranke 2")}${this.led(b("LIGHTBARRIER3"), "Lichtschranke 3")}
			</div>
			<div class="ov-pool"><span>Förderband</span><span class="meter"><span style="width:${pwm("CONVEYOR_PWM") / 10}%"></span></span><b>${pwm("CONVEYOR_PWM") / 10} %</b></div>
			<div class="ov-pool"><span>Kompressor</span><span class="meter"><span style="width:${pwm("COMPRESSOR_PWM") / 10}%"></span></span><b>${pwm("COMPRESSOR_PWM") / 10} %</b></div>
			${this.kv("Druck (roh)", v("PRESSURE_RAW") ?? "–")}
			${this.kv("Abstand ToF 1 / 2 / 3", [1, 2, 3].map((k) => (v(`TOF${k}_STATUS`) === 0 ? `${v(`TOF${k}_DISTANCE_MM`)} mm` : "–")).join(" / "))}`, { link: "/registers", wide: false });
	}

	private renderDrives(v: (n: string) => number | undefined) {
		const axis = (k: 1 | 2) => {
			const st = v(`STEPPER${k}_STATUS`) ?? 0;
			return html`<div class="ov-axis">
				<span class="ov-axis-name">Achse ${k}</span>
				<b>${v(`STEPPER${k}_POSITION`) ?? "–"}</b><small>Schritte</small>
				${this.led((st & 1) !== 0, "frei")}${this.led((st & 2) !== 0, "fährt", "blue")}${this.led((st & 32) !== 0, "referenziert")}
			</div>`;
		};
		return this.tile("Antriebe", "stepper", html`
			${this.kv("Treiber", v("STEPPER_ENABLE") ? "freigegeben" : "gesperrt")}
			${axis(1)}${axis(2)}`, { link: "/steppers" });
	}

	private renderBuses(v: (n: string) => number | undefined) {
		const scans = this.info ? i2cBusSections(this.info.i2cScans) : [];
		return this.tile("Busse", "can", html`
			${this.kv("CAN-Zustand", v("CAN_BUS_STATE") ?? "–")}
			${this.kv("CAN Rx / Tx / Fehler", `${v("CAN_RX_COUNT") ?? "–"} / ${v("CAN_TX_COUNT") ?? "–"} / ${v("CAN_ERROR_COUNT") ?? "–"}`)}
			${scans.map((s) => this.kv(`I²C ${s.label}`, `${s.addresses.filter(Boolean).length} Geräte`))}`, { link: "/system" });
	}

	private renderEvents() {
		const lvl = (l: number) => (l >= system.LogLevel.LOG_ERROR ? "err" : "warn");
		return this.tile("Letzte Warnungen & Fehler", "bell", this.events.length === 0
			? html`<div class="ov-calm">${icon("ack", 28)}<span>Keine Warnungen seit dem Öffnen der Seite</span></div>`
			: html`<ul class="ov-events">${this.events.map((e) => html`<li class="ov-ev-${lvl(e.level)}">
				<span class="ov-ev-time">${e.unixMs ? clock(e.unixMs) : `${(e.uptimeMs / 1000).toFixed(0)} s`}</span>
				<span class="ov-ev-src">${e.logger}</span><span class="ov-ev-text">${e.text}</span></li>`)}</ul>`, { link: "/terminal", wide: true });
	}
}

declare global {
	interface HTMLElementTagNameMap {
		"overview-app": OverviewApp;
	}
}
