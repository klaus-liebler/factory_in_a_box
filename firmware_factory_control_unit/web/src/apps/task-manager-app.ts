import { LitElement, html } from "lit";
import { customElement, state } from "lit/decorators.js";
import "../styles.css";
import * as WsProtocol from "../../generated/ws-protocol.js";
import { sendBinary, setTaskListListener } from "../ws-client.js";
import { drawSparkline } from "./sparkline.js";
import type { DashboardApp } from "../shell/dashboard-app.js";

const POLL_INTERVAL_MS = 1000;

// ~40 Sekunden Verlauf bei 1s-Poll-Intervall -- lang genug, um einen Lasttrend zu erkennen, ohne
// den Ringpuffer unnoetig aufzublaehen (reine Diagnose-Seite, keine Langzeit-Historie noetig).
const CPU_HISTORY_LENGTH = 40;

// cpuPermille ist 0..1000 (0.0%..100.0%, s. best_binary_buffers_schema/tasks.cs) -- fester
// Skalen-Maximalwert fuer die Sparkline, s. deren Klassenkommentar zur Begruendung.
const CPU_SPARKLINE_MAX_PERMILLE = 1000;

// Ab diesen Anteilen des Stack-Puffers (High-Water-Mark / Groesse) faerbt sich die Stack-Spalte
// ein -- macht eine drohende Stack-Ueberlauf-Situation auf einen Blick sichtbar, statt nur eine
// Zahlenliste zu zeigen.
const STACK_WARNING_RATIO = 0.8;
const STACK_ERROR_RATIO = 0.95;

// Deutsche Kurzbeschreibung je ThreadX-Zustand (s. best_binary_buffers_schema/tasks.cs,
// entspricht 1:1 TX_READY..TX_PRIORITY_CHANGE).
const STATE_LABELS: Record<number, string> = {
	[WsProtocol.tasks.ThreadState.Ready]: "Bereit / läuft",
	[WsProtocol.tasks.ThreadState.Completed]: "Beendet",
	[WsProtocol.tasks.ThreadState.Terminated]: "Terminiert",
	[WsProtocol.tasks.ThreadState.Suspended]: "Suspendiert",
	[WsProtocol.tasks.ThreadState.Sleep]: "Schläft",
	[WsProtocol.tasks.ThreadState.QueueSuspended]: "Wartet auf Queue",
	[WsProtocol.tasks.ThreadState.SemaphoreSuspended]: "Wartet auf Semaphore",
	[WsProtocol.tasks.ThreadState.EventFlag]: "Wartet auf Event-Flag",
	[WsProtocol.tasks.ThreadState.BlockMemory]: "Wartet auf Block-Pool",
	[WsProtocol.tasks.ThreadState.ByteMemory]: "Wartet auf Byte-Pool",
	[WsProtocol.tasks.ThreadState.IoDriver]: "Wartet auf I/O-Treiber",
	[WsProtocol.tasks.ThreadState.File]: "Wartet auf Dateisystem",
	[WsProtocol.tasks.ThreadState.TcpIp]: "Wartet auf TCP/IP",
	[WsProtocol.tasks.ThreadState.MutexSuspended]: "Wartet auf Mutex",
	[WsProtocol.tasks.ThreadState.PriorityChange]: "Prioritätswechsel",
	[WsProtocol.tasks.ThreadState.Unknown]: "Unbekannt",
};

// Badge-Variante je Zustand -- gruppiert die 16 ThreadX-Zustaende in vier optisch unterscheidbare
// Kategorien (laeuft/schlaeft/wartet-auf-Ressource/beendet), s. .state-badge-* in styles.css.
const STATE_BADGE_VARIANTS: Record<number, string> = {
	[WsProtocol.tasks.ThreadState.Ready]: "running",
	[WsProtocol.tasks.ThreadState.Sleep]: "idle",
	[WsProtocol.tasks.ThreadState.Suspended]: "waiting",
	[WsProtocol.tasks.ThreadState.QueueSuspended]: "waiting",
	[WsProtocol.tasks.ThreadState.SemaphoreSuspended]: "waiting",
	[WsProtocol.tasks.ThreadState.EventFlag]: "waiting",
	[WsProtocol.tasks.ThreadState.BlockMemory]: "waiting",
	[WsProtocol.tasks.ThreadState.ByteMemory]: "waiting",
	[WsProtocol.tasks.ThreadState.IoDriver]: "waiting",
	[WsProtocol.tasks.ThreadState.File]: "waiting",
	[WsProtocol.tasks.ThreadState.TcpIp]: "waiting",
	[WsProtocol.tasks.ThreadState.MutexSuspended]: "waiting",
	[WsProtocol.tasks.ThreadState.PriorityChange]: "waiting",
	[WsProtocol.tasks.ThreadState.Completed]: "stopped",
	[WsProtocol.tasks.ThreadState.Terminated]: "stopped",
	[WsProtocol.tasks.ThreadState.Unknown]: "unknown",
};

// tasks.TaskInfo.name ist ein fest 32 Byte grosses, null-gepolstertes ASCII-Feld (s.
// best_binary_buffers_schema/tasks.cs) -- hier auf den Teil vor dem ersten Null-Byte gekuerzt.
function decodeName(bytes: number[]): string {
	const end = bytes.indexOf(0);
	const slice = end === -1 ? bytes : bytes.slice(0, end);
	return String.fromCharCode(...slice);
}

function formatBytes(bytes: number): string {
	return `${(bytes / 1024).toFixed(2)} KiB`;
}

@customElement("task-manager-app")
export class TaskManagerApp extends LitElement implements DashboardApp {
	protected createRenderRoot() {
		return this;
	}

	@state() private tasks: WsProtocol.tasks.TaskInfo[] = [];
	@state() private statusMessage = "Warte auf Verbindung...";
	@state() private statusVariant: "warning" | "success" | "error" = "warning";

	// Die allererste Antwort nach dem Oeffnen der Seite hat noch keinen Vorwert fuer die
	// CPU-Last-Berechnung (s. task_monitor.cpp) -- cpuPermille ist dann fuer JEDEN Task 0, was
	// sich nicht von einem tatsaechlich untaetigen Thread unterscheiden liesse. Deshalb wird die
	// CPU-Spalte erst ab der zweiten Antwort angezeigt.
	@state() private receivedCount = 0;

	private pollTimeoutHandle?: ReturnType<typeof setTimeout>;
	private stopped = true;

	// Schluessel: Thread-Name (stabil ueber die Laufzeit) -- persistiert bewusst ueber onShow()/
	// onHide()-Zyklen hinweg (wie "samples" bei power-management-app.ts), damit ein kurzer Abstecher
	// auf eine andere Seite und zurueck nicht die ganze Verlaufskurve verwirft.
	private readonly cpuHistory = new Map<string, number[]>();

	private readonly onTaskList = (payload: WsProtocol.tasks.TaskListMessage.Payload) => {
		this.tasks = payload.items;
		this.receivedCount++;
		// Die allererste Antwort hat kein sinnvolles cpuPermille (s. Kommentar bei receivedCount) --
		// erst ab der zweiten Antwort in die Historie aufnehmen, sonst faengt jede Sparkline mit
		// einem irrefuehrenden 0%-Punkt an.
		if (this.receivedCount >= 2) {
			for (const task of payload.items) {
				const name = decodeName(task.name);
				const history = this.cpuHistory.get(name) ?? [];
				history.push(task.cpuPermille);
				while (history.length > CPU_HISTORY_LENGTH) history.shift();
				this.cpuHistory.set(name, history);
			}
		}
		this.statusMessage = `Verbunden -- zuletzt aktualisiert ${new Date().toLocaleTimeString("de-DE")}`;
		this.statusVariant = "success";
	};

	onShow(): void {
		this.stopped = false;
		setTaskListListener(this.onTaskList);
		this.scheduleNextPoll(0);
	}

	onHide(): void {
		this.stopped = true;
		setTaskListListener(null);
		if (this.pollTimeoutHandle) {
			clearTimeout(this.pollTimeoutHandle);
		}
	}

	private scheduleNextPoll(delayMs: number) {
		if (this.stopped) return;
		this.pollTimeoutHandle = setTimeout(() => {
			sendBinary(WsProtocol.tasks.TaskManagerRequest.encode({ requestId: 0 }));
			this.scheduleNextPoll(POLL_INTERVAL_MS);
		}, delayMs);
	}

	private renderStackCell(task: WsProtocol.tasks.TaskInfo) {
		const ratio = task.stackSizeBytes > 0 ? task.stackUsedHighWaterBytes / task.stackSizeBytes : 0;
		const percentText = `${(ratio * 100).toFixed(0)}%`;
		const cssClass =
			ratio >= STACK_ERROR_RATIO ? "status-text status-error" : ratio >= STACK_WARNING_RATIO ? "status-text status-warning" : "";
		return html`
			${formatBytes(task.stackUsedCurrentBytes)} aktuell / ${formatBytes(task.stackUsedHighWaterBytes)} Peak
			von ${formatBytes(task.stackSizeBytes)}
			<span class=${cssClass}>(${percentText})</span>
		`;
	}

	private renderCpuCell(task: WsProtocol.tasks.TaskInfo) {
		// Canvas IMMER rendern (auch vor der zweiten Antwort) -- haelt die Zeilenstruktur stabil,
		// damit drawSparklines() Canvas und this.tasks per Index zuordnen kann (s. updated()).
		return html`
			<span class="cpu-cell">
				<canvas class="cpu-sparkline"></canvas>
				<span>${this.receivedCount < 2 ? "–" : `${(task.cpuPermille / 10).toFixed(1)} %`}</span>
			</span>
		`;
	}

	private renderStateBadge(task: WsProtocol.tasks.TaskInfo) {
		const variant = STATE_BADGE_VARIANTS[task.state] ?? "unknown";
		const label = STATE_LABELS[task.state] ?? "Unbekannt";
		return html`<span class="state-badge state-badge-${variant}">${label}</span>`;
	}

	// Canvas-Inhalte liegen ausserhalb von Lits deklarativem Rendering -- nach jedem Update per
	// Index den passenden Verlauf aus cpuHistory in die (in derselben Reihenfolge wie this.tasks
	// gerenderte, s. renderCpuCell()) Sparkline-Canvases zeichnen. Analog zu updated()/redraw() bei
	// power-management-app.ts.
	protected updated(): void {
		const canvases = this.querySelectorAll<HTMLCanvasElement>(".cpu-sparkline");
		canvases.forEach((canvas, i) => {
			const task = this.tasks[i];
			if (!task) return;
			const history = this.cpuHistory.get(decodeName(task.name)) ?? [];
			drawSparkline(canvas, history, { max: CPU_SPARKLINE_MAX_PERMILLE });
		});
	}

	render() {
		return html`
			<div class="container">
				<section class="header-section app-panel">
					<div class="panel-label">Task Manager</div>
					<div class="status-text status-${this.statusVariant}">${this.statusMessage}</div>
				</section>

				<div class="panel-section">
					<div class="panel-label">ThreadX-Threads</div>
					${this.tasks.length === 0
						? html`<p class="panel-text">Noch keine Daten empfangen.</p>`
						: html`
								<div class="register-table-wrap">
									<table class="register-table register-table-fixed">
										<colgroup>
											<col style="width: 190px" />
											<col style="width: 170px" />
											<col style="width: 70px" />
											<col style="width: 320px" />
											<col style="width: 130px" />
											<col style="width: 110px" />
										</colgroup>
										<thead>
											<tr>
												<th>Name</th>
												<th>Zustand</th>
												<th>Priorität</th>
												<th>Stack</th>
												<th>CPU</th>
												<th>Läufe seit Boot</th>
											</tr>
										</thead>
										<tbody>
											${this.tasks.map(
												(task) => html`
													<tr>
														<td>${decodeName(task.name)}</td>
														<td>${this.renderStateBadge(task)}</td>
														<td>${task.priority}</td>
														<td>${this.renderStackCell(task)}</td>
														<td>${this.renderCpuCell(task)}</td>
														<td>${task.runCount}</td>
													</tr>
												`
											)}
										</tbody>
									</table>
								</div>
							`}
				</div>
			</div>
		`;
	}
}
