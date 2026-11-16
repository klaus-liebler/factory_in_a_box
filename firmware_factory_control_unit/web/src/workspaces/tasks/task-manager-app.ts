// Task manager: the firmware's threads (priority, stack, CPU load with a short history) and memory pools.
// The requests only send; the answers arrive as pushed messages of the "tasks" namespace.
import { LitElement, html } from "lit";
import { customElement, state } from "lit/decorators.js";
import "./task-manager.css";
import * as WsProtocol from "../../../generated/ws-protocol.ts";
import { registerWsProtocolListener, sendBinary, type WsProtocolListener } from "../../core/ws-client.ts";
import { sparkline } from "../../ui/charts.ts";
import type { WorkspaceApp } from "../../shell/workspace-app.ts";
import { setAppStatus } from "../../shell/status-bar.ts";
import { openWorkspace, setAppRibbon } from "../../shell/ribbon.ts";
import { Poller, pollingGroup } from "../../core/poller.ts";
import { cString, formatBytes } from "../../core/format.ts";

// ~40 s of history at one answer per second: enough to see a trend.
const CPU_HISTORY_LENGTH = 40;

// cpuPermille is 0..1000 (schema/best_binary_buffers/tasks.cs): a fixed scale for the sparkline.
const CPU_SPARKLINE_MAX_PERMILLE = 1000;

// From these shares of the stack (high-water mark / size) the stack column is coloured.
const STACK_WARNING_RATIO = 0.8;
const STACK_ERROR_RATIO = 0.95;

@customElement("task-manager-app")
export class TaskManagerApp extends LitElement implements WorkspaceApp, WsProtocolListener {
	protected willUpdate(): void {
		setAppStatus(this, this.poller.status("Warte auf Antwort …"));
		setAppRibbon(this, [
			{ id: "tasks", label: "Tasks", groups: [
				pollingGroup(this.poller, "Threads und Pools"),
				{ label: "Weitere", commands: [
					{ id: "dash", label: "Dashboard", icon: "dashboard", run: () => openWorkspace("/") },
					{ id: "system", label: "System", icon: "chip", run: () => openWorkspace("/system") },
				] },
			] },
		]);
	}

	protected createRenderRoot() {
		return this;
	}

	@state() private tasks: WsProtocol.tasks.TaskInfo[] = [];
	@state() private pools: WsProtocol.tasks.PoolInfo[] = [];
	@state() private freeHeapBytes = 0;

	// The first answer after opening has no previous value for the CPU load (task_monitor.cpp): every
	// cpuPermille is 0 then. The CPU column is shown from the second answer on.
	@state() private receivedCount = 0;

	private readonly poller = new Poller(this, () => {
		sendBinary(WsProtocol.tasks.TaskManagerRequest.encode({}, 0));
		sendBinary(WsProtocol.tasks.PoolListRequest.encode({}, 0));
	}, 1000);

	// Key: the thread name (stable at run time); kept across onShow()/onHide() so a short visit elsewhere
	// keeps the curves.
	private readonly cpuHistory = new Map<string, number[]>();

	private unregisterWsListener: (() => void) | null = null;

	// listener of the tasks namespace, registered between onShow() and onHide()
	onWsMessage(messageTypeId: number, view: DataView): void {
		switch (messageTypeId) {
			case WsProtocol.tasks.TaskListMessage.TYPE_ID:
				this.onTaskList(WsProtocol.tasks.TaskListMessage.decode(view, 0));
				return;
			case WsProtocol.tasks.PoolListMessage.TYPE_ID:
				this.onPoolList(WsProtocol.tasks.PoolListMessage.decode(view, 0));
				return;
		}
	}

	private readonly onTaskList = (payload: WsProtocol.tasks.TaskListMessage.Payload) => {
		this.tasks = payload.items;
		this.receivedCount++;
		// the first answer has no meaningful cpuPermille (see receivedCount): no history point from it
		if (this.receivedCount >= 2) {
			for (const task of payload.items) {
				const name = cString(task.name);
				const history = this.cpuHistory.get(name) ?? [];
				history.push(task.cpuPermille);
				while (history.length > CPU_HISTORY_LENGTH) history.shift();
				this.cpuHistory.set(name, history);
			}
		}
		this.poller.ok();
	};

	private readonly onPoolList = (payload: WsProtocol.tasks.PoolListMessage.Payload) => {
		this.pools = payload.pools;
		this.freeHeapBytes = payload.freeHeapBytes;
	};

	onShow(): void {
		this.unregisterWsListener = registerWsProtocolListener(WsProtocol.tasks.NAMESPACE_ID, this);
		this.poller.start();
	}

	onHide(): void {
		this.poller.stop();
		this.unregisterWsListener?.();
		this.unregisterWsListener = null;
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
		return html`n			<span class="cpu-cell">
				<span class="cpu-sparkline">${sparkline(this.cpuHistory.get(cString(task.name)) ?? [], { min: 0, max: CPU_SPARKLINE_MAX_PERMILLE, capacity: CPU_HISTORY_LENGTH, area: false })}</span>
				<span>${this.receivedCount < 2 ? "–" : `${(task.cpuPermille / 10).toFixed(1)} %`}</span>
			</span>
		`;
	}

	// The same warning/error thresholds as renderStackCell(): a pool near its capacity is the same kind of
	// signal as a thread near its stack limit.
	private renderPoolRow(pool: WsProtocol.tasks.PoolInfo) {
		const used = pool.capacity - pool.free;
		const ratio = pool.capacity > 0 ? used / pool.capacity : 0;
		const percentText = `${(ratio * 100).toFixed(0)}%`;
		const cssClass =
			ratio >= STACK_ERROR_RATIO ? "status-text status-error" : ratio >= STACK_WARNING_RATIO ? "status-text status-warning" : "";
		return html`
			<tr>
				<td>${cString(pool.name)}</td>
				<td>${pool.free}</td>
				<td title="der geringste freie Stand seit dem Start (beim wolfSSL-Pool: seine Spitzenlast)">${pool.minFree}</td>
				<td>${pool.capacity}</td>
				<td><span class=${cssClass}>${percentText}</span></td>
			</tr>
		`;
	}

	render() {
		return html`
			<div class="container">

				<div class="panel-section">
					<div class="panel-label">Threads</div>
					${this.tasks.length === 0
						? html`<p class="panel-text">Noch keine Daten empfangen.</p>`
						: html`
								<div class="register-table-wrap">
									<table class="register-table register-table-fixed">
										<colgroup>
											<col style="width: 190px" />
											<col style="width: 70px" />
											<col style="width: 320px" />
											<col style="width: 130px" />
											<col style="width: 110px" />
										</colgroup>
										<thead>
											<tr>
												<th>Name</th>
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
														<td>${cString(task.name)}</td>
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

				<div class="panel-section">
					<div class="panel-label">Pools &amp; Heap</div>
					${this.pools.length === 0
						? html`<p class="panel-text">Noch keine Daten empfangen.</p>`
						: html`
								<div class="register-table-wrap">
									<table class="register-table">
										<thead>
											<tr>
												<th>Pool</th>
												<th>Frei</th>
												<th>Min. frei</th>
												<th>Kapazität</th>
												<th>Auslastung</th>
											</tr>
										</thead>
										<tbody>
											${this.pools.map((pool) => this.renderPoolRow(pool))}
											<tr>
												<td>newlib-Heap (B)</td>
												<td colspan="3">${formatBytes(this.freeHeapBytes)} frei</td>
											</tr>
										</tbody>
									</table>
								</div>
							`}
				</div>
			</div>
		`;
	}
}
