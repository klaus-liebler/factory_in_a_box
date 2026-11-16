// The dashboard workspace (the start page): tiles with the device's state at a glance - identity and
// uptime, temperature, CPU load, memory, supply, network, PLC, actuators and sensors, drives, buses, the
// latest log events - each with a link into its workspace. Polls only while it is shown: registers every
// second, tasks every 2 s, PLC status every 2 s, system information every 10 s.
import { LitElement, html, nothing, svg } from "lit";
import { customElement, state } from "lit/decorators.js";
import type { DashboardApp } from "../shell/dashboard-app.js";
import { setAppStatus } from "../shell/status-bar.js";
import { openWorkspace, setAppRibbon } from "../shell/ribbon.js";
import { icon, type IconName } from "../shell/icons.js";
import { REGIONS, type RegisterDef } from "../../generated/register-map.js";
import { fetchRegisters, type RegisterValues } from "../registers.js";
import { plc, system, tasks } from "../../generated/ws-protocol.js";
import { registerWsProtocolListener, sendBinary, wsRequest } from "../ws-client.js";
import { logEntries, subscribeLog, type LogEntry } from "../log-store.js";
import * as buildInfo from "../../generated/build-info.js";
import { i2cBusSections } from "../i2c-scan.js";
import "./overview.css";

const HISTORY = 120;
const PLC_STATES = ["Kein Programm", "STOP", "RUN", "Fehler", "Angehalten"];
const RESET_CAUSES: Record<number, string> = { 0: "unbekannt", 1: "Watchdog (IWDG)", 2: "Watchdog (WWDG)", 3: "Software", 4: "Low-Power", 5: "Brown-Out", 6: "Reset-Pin / Einschalten" };

const REG = new Map<string, RegisterDef>(REGIONS.flatMap((r) => r.registers.map((x) => [x.name, x] as const)));

/** A register's value by name (32-bit pairs high word first, signed as declared); undefined: not there. */
export function regValue(values: RegisterValues | null, name: string): number | undefined {
	const d = REG.get(name);
	if (!values || !d) return undefined;
	const bank = d.bank === "holding" ? values.holding : values.input;
	const hi = bank[d.address];
	if (hi === undefined) return undefined;
	if (d.combine?.count === 2) {
		const lo = bank[d.address + 1];
		if (lo === undefined) return undefined;
		const u = ((hi << 16) | lo) >>> 0;
		return d.signed ? u | 0 : u;
	}
	return d.signed && hi > 0x7fff ? hi - 0x10000 : hi;
}

const decodeName = (bytes: number[]) => {
	const end = bytes.indexOf(0);
	return String.fromCharCode(...(end < 0 ? bytes : bytes.slice(0, end)));
};

function formatUptime(s: number): string {
	const d = Math.floor(s / 86400), h = Math.floor((s % 86400) / 3600), m = Math.floor((s % 3600) / 60);
	return d ? `${d} d ${h} h` : h ? `${h} h ${m} min` : `${m} min ${s % 60} s`;
}

/** A sparkline (area + line) of the values over the tile's width. */
function sparkline(values: number[], color: string, min?: number, max?: number) {
	if (values.length < 2) return html`<div class="ov-spark ov-spark-empty"></div>`;
	const lo = min ?? Math.min(...values), hi = max ?? Math.max(...values);
	const span = hi - lo || 1;
	const pts = values.map((v, i) => `${((i / (HISTORY - 1)) * 100 + (HISTORY - values.length) * (100 / (HISTORY - 1))).toFixed(2)},${(30 - ((v - lo) / span) * 28 - 1).toFixed(2)}`);
	const id = `g${color.replace("#", "")}`;
	return html`<svg class="ov-spark" viewBox="0 0 100 30" preserveAspectRatio="none">
		<defs><linearGradient id=${id} x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color=${color} stop-opacity=".35"/><stop offset="1" stop-color=${color} stop-opacity="0"/></linearGradient></defs>
		${svg`<polygon points="${pts[0].split(",")[0]},30 ${pts.join(" ")} 100,30" fill="url(#${id})"/><polyline points=${pts.join(" ")} fill="none" stroke=${color} stroke-width="1.6" vector-effect="non-scaling-stroke"/>`}
	</svg>`;
}

/** A ring gauge 0..1 with the value in the middle. */
function ring(f: number, color: string, text: string, sub: string) {
	const r = 34, c = 2 * Math.PI * r, v = Math.max(0, Math.min(1, f));
	return html`<svg class="ov-ring" viewBox="0 0 84 84">
		${svg`<circle cx="42" cy="42" r=${r} fill="none" stroke="#e5e7eb" stroke-width="8"/>
		<circle cx="42" cy="42" r=${r} fill="none" stroke=${color} stroke-width="8" stroke-linecap="round"
			stroke-dasharray="${(c * v).toFixed(1)} ${c.toFixed(1)}" transform="rotate(-90 42 42)"/>
		<text x="42" y="44" text-anchor="middle" class="ov-ring-value">${text}</text>
		<text x="42" y="58" text-anchor="middle" class="ov-ring-sub">${sub}</text>`}
	</svg>`;
}

const tempColor = (t: number) => (t >= 80 ? "#ef4444" : t >= 65 ? "#f59e0b" : "#14b8a6");
const loadColor = (f: number) => (f >= 0.85 ? "#ef4444" : f >= 0.6 ? "#f59e0b" : "#3b82f6");

@customElement("overview-app")
export class OverviewApp extends LitElement implements DashboardApp {
	@state() private regs: RegisterValues | null = null;
	@state() private info: system.SystemInfoMessage.Payload | null = null;
	@state() private taskList: tasks.TaskInfo[] = [];
	@state() private pools: tasks.PoolInfo[] = [];
	@state() private plcStatus: plc.StatusResponse.Payload | null = null;
	@state() private events: LogEntry[] = [];
	@state() private paused = false;
	@state() private lastUpdate = 0;
	@state() private failed = false;

	private readonly hist = { temp: [] as number[], load: [] as number[], power: [] as number[], rx: [] as number[] };
	private lastRx: { t: number; rx: number; tx: number } | null = null;
	private rates = { rx: 0, tx: 0 };
	private taskAnswers = 0;
	private timers: ReturnType<typeof setInterval>[] = [];
	private unlisten: (() => void) | null = null;
	private unlog: (() => void) | null = null;

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
		this.start();
	}

	onHide(): void {
		this.stop();
		this.unlisten?.();
		this.unlog?.();
		this.unlisten = this.unlog = null;
	}

	private start(): void {
		this.stop();
		if (this.paused) return;
		void this.pollRegisters();
		void this.pollInfo();
		this.pollTasks();
		void this.pollPlc();
		this.timers = [
			setInterval(() => void this.pollRegisters(), 1000),
			setInterval(() => this.pollTasks(), 2000),
			setInterval(() => void this.pollPlc(), 2000),
			setInterval(() => void this.pollInfo(), 10_000),
		];
	}

	private stop(): void {
		for (const t of this.timers) clearInterval(t);
		this.timers = [];
	}

	private push(key: keyof OverviewApp["hist"], v: number): void {
		const a = this.hist[key];
		a.push(v);
		if (a.length > HISTORY) a.shift();
	}

	private async pollRegisters(): Promise<void> {
		try {
			const v = await fetchRegisters();
			this.regs = v;
			this.failed = false;
			this.lastUpdate = Date.now();
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
		} catch {
			this.failed = true;
		}
	}

	private async pollInfo(): Promise<void> {
		try {
			this.info = await wsRequest(system.NAMESPACE_ID, (requestId) => system.SystemInfoRequest.encode({ requestId }), system.SystemInfoMessage);
		} catch {
			// the register poll reports the connection
		}
	}

	private pollTasks(): void {
		sendBinary(tasks.TaskManagerRequest.encode({ requestId: 0 }));
		sendBinary(tasks.PoolListRequest.encode({ requestId: 0 }));
	}

	private async pollPlc(): Promise<void> {
		try {
			this.plcStatus = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.StatusRequest.encode({ requestId }), plc.StatusResponse);
		} catch {
			this.plcStatus = null;
		}
	}

	private refreshNow(): void {
		this.paused = false;
		this.start();
	}

	/** Total CPU load 0..1: everything but the idle thread. */
	private cpuLoad(): number {
		const busy = this.taskList.filter((t) => !/idle/i.test(decodeName(t.name))).reduce((s, t) => s + t.cpuPermille, 0);
		return Math.min(1, busy / 1000);
	}

	private recentEvents(all: readonly LogEntry[]): LogEntry[] {
		const out: LogEntry[] = [];
		for (let i = all.length - 1; i >= 0 && out.length < 6; i--) if (all[i].level >= system.LogLevel.LOG_WARN) out.push(all[i]);
		return out;
	}

	// ---- rendering ---------------------------------------------------------------------------------------------
	protected willUpdate(): void {
		const age = this.lastUpdate ? Math.round((Date.now() - this.lastUpdate) / 1000) : null;
		setAppStatus(this, {
			text: this.paused ? "Aktualisierung angehalten" : this.failed ? "Keine Verbindung zur Control Unit" : this.regs ? "Live" : "Lade …",
			kind: this.paused ? "idle" : this.failed ? "error" : this.regs ? "ok" : "busy",
			extra: age !== null ? html`<span class="sb-chip">Stand ${new Date(this.lastUpdate).toLocaleTimeString("de-DE")}</span>` : null,
		});
		setAppRibbon(this, [
			{ id: "overview", label: "Übersicht", groups: [
				{ label: "Daten", commands: [
					{ id: "refresh", label: "Aktualisieren", icon: "refresh", run: () => this.refreshNow() },
					{ id: "pause", label: this.paused ? "Fortsetzen" : "Anhalten", icon: this.paused ? "run" : "pause", active: this.paused,
						title: "Die Live-Abfrage anhalten oder fortsetzen", run: () => { this.paused = !this.paused; if (this.paused) this.stop(); else this.start(); } },
				] },
				{ label: "Arbeitsbereiche", commands: [
					{ id: "plc", label: "SPS-Entwicklung", icon: "plc", run: () => openWorkspace("/plc") },
					{ id: "hmi", label: "Bedienen & Beobachten", icon: "hmi", run: () => openWorkspace("/hmi") },
					{ id: "registers", label: "Register", icon: "registers", run: () => openWorkspace("/registers") },
					{ id: "system", label: "System", icon: "chip", size: "small", run: () => openWorkspace("/system") },
					{ id: "tasks", label: "Task Manager", icon: "tasks", size: "small", run: () => openWorkspace("/tasks") },
					{ id: "power", label: "Power", icon: "power", size: "small", run: () => openWorkspace("/power") },
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
		return html`<span class="ov-led-item"><span class="ov-led ov-led-${on === undefined ? "na" : on ? color : "off"}"></span>${label}</span>`;
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
						${health !== undefined ? html`<span class="ov-chip ${health === 0 ? "ok" : "warn"}" title="Health-Bitfeld">Health 0x${health.toString(16).padStart(4, "0")}</span>` : nothing}
						${i ? html`<span class="ov-chip" title="Ursache des letzten Resets">Reset: ${RESET_CAUSES[i.resetCauseCode] ?? "?"}</span>` : nothing}
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
			${sparkline(h, tempColor(t ?? 0))}`, { link: "/system" });
	}

	private renderCpu() {
		const load = this.taskAnswers >= 2 ? this.cpuLoad() : null;
		const top = [...this.taskList].filter((t) => !/idle/i.test(decodeName(t.name))).sort((a, b) => b.cpuPermille - a.cpuPermille).slice(0, 3);
		return this.tile("CPU-Last", "chip", html`
			<div class="ov-row">
				${ring(load ?? 0, loadColor(load ?? 0), load === null ? "–" : `${Math.round(load * 100)} %`, `${this.taskList.length} Tasks`)}
				<div class="ov-col ov-bars">
					${top.map((t) => html`<div class="ov-bar-row"><span>${decodeName(t.name)}</span>
						<span class="ov-bar"><span style="width:${Math.min(100, t.cpuPermille / 10)}%"></span></span><b>${(t.cpuPermille / 10).toFixed(1)} %</b></div>`)}
				</div>
			</div>
			${sparkline(this.hist.load.map((x) => x * 100), "#3b82f6", 0, 100)}`, { link: "/tasks" });
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
				return html`<div class="ov-pool"><span>${decodeName(p.name)}</span>
					<span class="ov-bar"><span style="width:${used * 100}%"></span><i style="left:${peak * 100}%" title="Spitze"></i></span>
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
			${sparkline(this.hist.power, "#f59e0b")}
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
			${sparkline(this.hist.rx, "#14b8a6", 0)}`, { link: "/system" });
	}

	private renderPlc() {
		const s = this.plcStatus;
		const st = s ? PLC_STATES[s.state] ?? "?" : "–";
		const tone = !s ? "" : s.state === 2 ? "ov-ok" : s.state === 3 ? "ov-bad" : s.state === 4 ? "ov-warn" : "";
		return this.tile("SPS (IEC 61131-3)", "plc", html`
			<div class="ov-plc-state ${tone}">${st}</div>
			${this.kv("Ausgänge schaltet", s ? (s.source === plc.ControlSource.Iec ? "das IEC-Programm" : "der Feldbus") : "–")}
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
			<div class="ov-pool"><span>Förderband</span><span class="ov-bar"><span style="width:${pwm("CONVEYOR_PWM") / 10}%"></span></span><b>${pwm("CONVEYOR_PWM") / 10} %</b></div>
			<div class="ov-pool"><span>Kompressor</span><span class="ov-bar"><span style="width:${pwm("COMPRESSOR_PWM") / 10}%"></span></span><b>${pwm("COMPRESSOR_PWM") / 10} %</b></div>
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
				<span class="ov-ev-time">${e.unixMs ? new Date(e.unixMs).toLocaleTimeString("de-DE") : `${(e.uptimeMs / 1000).toFixed(0)} s`}</span>
				<span class="ov-ev-src">${e.logger}</span><span class="ov-ev-text">${e.text}</span></li>`)}</ul>`, { link: "/terminal", wide: true });
	}
}

declare global {
	interface HTMLElementTagNameMap {
		"overview-app": OverviewApp;
	}
}
