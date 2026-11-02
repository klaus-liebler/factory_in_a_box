// "Terminal" page: an interactive console for the firmware's test commands on top, the live
// firmware log as a table below.
//
// Console: classic look (black, white monospace, prompt, blinking block cursor). Everything but
// running a command happens in the browser: history (arrow up/down), Tab completion of command
// names and parameter values, the parameter tooltip, help and clear. The command catalog
// (generated/terminal-commands.ts, from terminal_schema/commands.json) is compiled into the web
// app; parameter values are listed there, computed here (command names, registers) or fetched
// live from the device (terminal.LookupRequest, cached for a moment).
//
// Log: rows are appended to the DOM as they arrive (no re-render of up to 10000 rows); the
// level filter is CSS on the table (instant), auto-scroll can be switched off.
import { LitElement, html, nothing } from "lit";
import { customElement, query, state } from "lit/decorators.js";
import "../styles.css";
import "./terminal-app.css";
import { system, terminal } from "../../generated/ws-protocol.js";
import { TERMINAL_COMMANDS, TERMINAL_LOOKUPS, type TerminalCommand, type TerminalParam, type TerminalValue } from "../../generated/terminal-commands.js";
import { analyse, complete, findCommand, matches, meantCommand, paramAt, sourceValues, type Word } from "../terminal/command-line.js";
import { wsRequest } from "../ws-client.js";
import { clearLog, logEntries, MAX_LOG_LINES, subscribeLog, type LogEntry } from "../log-store.js";
import type { DashboardApp } from "../shell/dashboard-app.js";

const PROMPT = "fcu> ";
const MAX_CONSOLE_LINES = 2000;
const MAX_HISTORY = 100;
const MAX_BUBBLE_VALUES = 12;
const LOOKUP_MAX_AGE_MS = 1000; // "live": a lookup older than this is fetched again

type LineKind = "input" | "output" | "error";

// ---- log ------------------------------------------------------------------------------------------------
const LEVELS: { level: system.LogLevel; name: string }[] = [
	{ level: system.LogLevel.LOG_TRACE, name: "TRACE" },
	{ level: system.LogLevel.LOG_DEBUG, name: "DEBUG" },
	{ level: system.LogLevel.LOG_INFO, name: "INFO" },
	{ level: system.LogLevel.LOG_WARN, name: "WARN" },
	{ level: system.LogLevel.LOG_ERROR, name: "ERROR" },
	{ level: system.LogLevel.LOG_FATAL, name: "FATAL" },
];

function levelName(level: system.LogLevel): string {
	return LEVELS.find((l) => l.level === level)?.name ?? String(level);
}

function formatTime(e: LogEntry): string {
	if (e.unixMs > 0) {
		const d = new Date(e.unixMs);
		return `${d.toLocaleTimeString("de-DE")}.${String(d.getMilliseconds()).padStart(3, "0")}`;
	}
	return `+${(e.uptimeMs / 1000).toFixed(3)} s`;
}

@customElement("terminal-app")
export class TerminalApp extends LitElement implements DashboardApp {
	protected createRenderRoot() {
		return this;
	}

	// ---- console -------------------------------------------------------------------------
	@state() private lines: { text: string; kind: LineKind }[] = [
		{ text: "Factory Control Unit terminal -- 'help' lists the commands, Tab completes, arrow up repeats.", kind: "output" },
	];
	@state() private input = "";
	@state() private cursor = 0;
	@state() private busy = false;
	@state() private bubbleHidden = false; // Escape, until the line changes
	private history: string[] = [];
	private historyIndex = -1; // -1: the line being typed
	private draft = "";

	// live values from the device, per lookup
	@state() private lookups = new Map<number, { values: TerminalValue[]; time: number; error?: string }>();
	private lookupsPending = new Map<number, Promise<TerminalValue[]>>();

	@query(".term") private term!: HTMLElement;
	@query(".term-keyboard") private inputElement!: HTMLInputElement;
	@query(".term-screen") private screen!: HTMLElement;
	@query(".term-prompt") private promptLine!: HTMLElement;
	@query(".term-bubble") private bubble?: HTMLElement;

	// ---- log ---------------------------------------------------------------------------------
	@state() private hiddenLevels = new Set<system.LogLevel>([system.LogLevel.LOG_TRACE]);
	@state() private autoScroll = true;
	@state() private lineCount = 0;
	@query(".log-body") private logBody!: HTMLTableSectionElement;
	@query(".log-scroll") private logScroll!: HTMLElement;
	private unsubscribe: (() => void) | null = null;

	onShow(): void {
		this.updateComplete.then(() => {
			this.rebuildLog();
			this.unsubscribe = subscribeLog((added, removed) => this.onLog(added, removed));
			this.inputElement?.focus();
		});
	}

	onHide(): void {
		this.unsubscribe?.();
		this.unsubscribe = null;
	}

	// ---- console: output ---------------------------------------------------------------------
	private print(text: string, kind: LineKind): void {
		const added = text.split("\n").map((t) => ({ text: t, kind }));
		this.lines = [...this.lines, ...added].slice(-MAX_CONSOLE_LINES);
		this.updateComplete.then(() => {
			if (this.screen) this.screen.scrollTop = this.screen.scrollHeight;
		});
	}

	// ---- parameter values ----------------------------------------------------------------------
	private fetchLookup(id: number): Promise<TerminalValue[]> {
		const pending = this.lookupsPending.get(id);
		if (pending) return pending;
		const promise = wsRequest(terminal.NAMESPACE_ID, (requestId) => terminal.LookupRequest.encode({ requestId, lookup: id }), terminal.LookupResponse)
			.then((r) => {
				if (!r.ok) throw new Error(`the device does not know lookup '${TERMINAL_LOOKUPS[id]?.name}'`);
				const values = r.values
					.split("\n")
					.filter((l) => l.length > 0)
					.map((l) => {
						const [value, doc = ""] = l.split("\t");
						return { value, doc };
					});
				this.lookups = new Map(this.lookups).set(id, { values, time: Date.now() });
				return values;
			})
			.catch((e: Error) => {
				this.lookups = new Map(this.lookups).set(id, { values: [], time: Date.now(), error: e.message });
				return [];
			})
			.finally(() => this.lookupsPending.delete(id));
		this.lookupsPending.set(id, promise);
		return promise;
	}

	/** The values of a parameter now (a lookup: the cached ones; refreshed when stale). */
	private valuesNow(param: TerminalParam, words: Word[]): TerminalValue[] | undefined {
		if (param.values) return param.values;
		if (param.source) return sourceValues(param, words);
		if (param.lookup === undefined) return undefined;
		const cached = this.lookups.get(param.lookup);
		if (!cached || Date.now() - cached.time > LOOKUP_MAX_AGE_MS) void this.fetchLookup(param.lookup);
		return cached?.values;
	}

	private async valuesFresh(param: TerminalParam, words: Word[]): Promise<TerminalValue[]> {
		if (param.lookup === undefined) return this.valuesNow(param, words) ?? [];
		const cached = this.lookups.get(param.lookup);
		if (cached && Date.now() - cached.time <= LOOKUP_MAX_AGE_MS) return cached.values;
		return this.fetchLookup(param.lookup);
	}

	// ---- console: input --------------------------------------------------------------------
	private syncFromInput(): void {
		const value = this.inputElement.value;
		if (value !== this.input) this.bubbleHidden = false;
		this.input = value;
		this.cursor = this.inputElement.selectionStart ?? value.length;
	}

	private setInput(value: string, cursor = value.length): void {
		this.inputElement.value = value;
		this.inputElement.setSelectionRange(cursor, cursor);
		this.syncFromInput();
	}

	private async run(line: string): Promise<void> {
		this.print(PROMPT + line, "input");
		const trimmed = line.trim();
		if (trimmed.length === 0) return;
		if (this.history[this.history.length - 1] !== trimmed) {
			this.history.push(trimmed);
			if (this.history.length > MAX_HISTORY) this.history.shift();
		}
		const words = trimmed.split(/\s+/);
		const command = findCommand(words[0]);
		if (!command) {
			this.print(`unknown command '${words[0]}' - try help`, "error");
			return;
		}
		if (command.client) {
			this.runClient(command, words.slice(1));
			return;
		}
		this.busy = true;
		try {
			const r = await wsRequest(terminal.NAMESPACE_ID, (requestId) => terminal.CommandRequest.encode({ requestId, line: trimmed }), terminal.CommandResponse);
			if (r.output.length > 0) this.print(r.output, r.ok ? "output" : "error");
		} catch (e) {
			this.print(`error: ${(e as Error).message}`, "error");
		} finally {
			this.busy = false;
			this.updateComplete.then(() => this.inputElement?.focus());
		}
	}

	private runClient(command: TerminalCommand, args: string[]): void {
		switch (command.name) {
			case "clear":
				this.lines = [];
				return;
			case "help": {
				if (args.length === 0) {
					const width = Math.max(...TERMINAL_COMMANDS.map((c) => c.usage.length));
					this.print(TERMINAL_COMMANDS.map((c) => `${c.usage.padEnd(width)}  ${c.doc}`).join("\n"), "output");
					return;
				}
				const c = findCommand(args[0]);
				if (!c) {
					this.print(`unknown command '${args[0]}'`, "error");
					return;
				}
				const out = [`${c.usage}`, `  ${c.doc}`];
				for (const p of c.params) {
					out.push(`  <${p.name}>${p.optional ? " (optional)" : ""}: ${p.doc}`);
					if (p.values) for (const v of p.values) out.push(`      ${v.value}${v.doc ? `  ${v.doc}` : ""}`);
					if (p.lookup !== undefined) out.push(`      (live from the device: ${TERMINAL_LOOKUPS[p.lookup]?.doc})`);
					if (p.source === "commands") out.push("      (a command name)");
					if (p.source === "registers") out.push("      (the register map of the bank)");
				}
				this.print(out.join("\n"), "output");
				return;
			}
		}
	}

	private async complete(): Promise<void> {
		const line = this.input;
		const cursor = this.cursor;
		const { words, index } = analyse(line, cursor);
		let values: TerminalValue[];
		if (index === 0) {
			values = TERMINAL_COMMANDS.map((c) => ({ value: c.name, doc: c.doc }));
		} else {
			const param = findCommand(words[0].text)?.params[index - 1];
			if (!param) return;
			values = await this.valuesFresh(param, words);
			if (this.input !== line) return; // typed on while the device answered
		}
		const r = complete(line, cursor, values);
		if (!r) return;
		if ("list" in r) {
			this.print(PROMPT + line, "input");
			this.print(r.list.join("  "), "output");
		} else {
			this.setInput(r.line, r.cursor);
		}
	}

	private readonly onKeyDown = (e: KeyboardEvent) => {
		if (this.busy) {
			if (e.key === "Enter" || e.key === "Tab") e.preventDefault();
			return;
		}
		switch (e.key) {
			case "Enter": {
				e.preventDefault();
				const line = this.inputElement.value;
				this.historyIndex = -1;
				this.setInput("");
				void this.run(line);
				return;
			}
			case "ArrowUp":
				e.preventDefault();
				if (this.history.length === 0) return;
				if (this.historyIndex === -1) {
					this.draft = this.inputElement.value;
					this.historyIndex = this.history.length - 1;
				} else if (this.historyIndex > 0) {
					this.historyIndex--;
				}
				this.setInput(this.history[this.historyIndex]);
				return;
			case "ArrowDown":
				e.preventDefault();
				if (this.historyIndex === -1) return;
				if (this.historyIndex < this.history.length - 1) {
					this.historyIndex++;
					this.setInput(this.history[this.historyIndex]);
				} else {
					this.historyIndex = -1;
					this.setInput(this.draft);
				}
				return;
			case "Tab":
				e.preventDefault();
				void this.complete();
				return;
			case "Escape":
				this.bubbleHidden = true;
				return;
			case "l":
				if (e.ctrlKey) {
					e.preventDefault();
					this.lines = [];
				}
				return;
		}
	};

	// ---- console: the parameter tooltip ---------------------------------------------------------
	private renderBubble() {
		if (this.busy || this.bubbleHidden) return nothing;
		const { words, current, index } = analyse(this.input, this.cursor);
		const command = meantCommand(words, index);
		if (!command) return nothing;
		const { param, tooMany } = paramAt(command, index, current);
		const prefix = this.input.slice(current.start, this.cursor);
		const values = param ? this.valuesNow(param, words) : undefined;
		const shown = values ? matches(values, prefix) : [];
		const lookupState = param?.lookup !== undefined ? this.lookups.get(param.lookup) : undefined;
		return html`
			<div class="term-bubble" style="--bubble-col: ${PROMPT.length + current.start}">
				<div class="bubble-usage">
					<span class=${index === 0 ? "bubble-current" : ""}>${command.name}</span>
					${command.params.map((p, i) => {
						const text = p.optional ? `[<${p.name}>]` : `<${p.name}>`;
						return html` <span class=${i === index - 1 ? "bubble-current" : ""}>${text}</span>`;
					})}
				</div>
				<div class="bubble-doc">${command.doc}</div>
				${param
					? html`<div class="bubble-param"><b>${param.name}</b>${param.optional ? " (optional)" : ""}: ${param.doc}</div>
							${lookupState?.error ? html`<div class="bubble-error">${lookupState.error}</div>` : nothing}
							${values === undefined && param.lookup !== undefined ? html`<div class="bubble-more">asking the device ...</div>` : nothing}
							${shown.length > 0
								? html`<table class="bubble-values">
										${shown.slice(0, MAX_BUBBLE_VALUES).map((v) => html`<tr><td>${v.value}</td><td>${v.doc}</td></tr>`)}
									</table>`
								: nothing}
							${shown.length > MAX_BUBBLE_VALUES ? html`<div class="bubble-more">+${shown.length - MAX_BUBBLE_VALUES} more (type to narrow, Tab completes)</div>` : nothing}`
					: tooMany
						? html`<div class="bubble-error">too many parameters</div>`
						: nothing}
			</div>
		`;
	}

	protected updated(): void {
		// the bubble above the prompt line, or below it when there is no room above
		if (!this.bubble || !this.promptLine || !this.term) return;
		const termBox = this.term.getBoundingClientRect();
		const promptBox = this.promptLine.getBoundingClientRect();
		const top = promptBox.top - termBox.top;
		const height = this.bubble.offsetHeight;
		this.bubble.style.top = top - height - 6 >= 0 ? `${top - height - 6}px` : `${top + promptBox.height + 6}px`;
		this.bubble.classList.toggle("below", top - height - 6 < 0);
	}

	// ---- log: rows outside of Lit ----------------------------------------------------------------------
	private makeRow(e: LogEntry): HTMLTableRowElement {
		const tr = document.createElement("tr");
		tr.className = `lvl-${e.level}`;
		for (const [cls, text] of [
			["col-level", levelName(e.level)],
			["col-time", formatTime(e)],
			["col-logger", e.logger],
			["col-text", e.text],
		]) {
			const td = document.createElement("td");
			td.className = cls;
			td.textContent = text;
			tr.appendChild(td);
		}
		return tr;
	}

	private rebuildLog(): void {
		if (!this.logBody) return;
		const fragment = document.createDocumentFragment();
		for (const e of logEntries()) fragment.appendChild(this.makeRow(e));
		this.logBody.replaceChildren(fragment);
		this.lineCount = logEntries().length;
		this.scrollLogToEnd();
	}

	private onLog(added: LogEntry[], removed: number): void {
		if (!this.logBody) return;
		if (added.length === 0 && removed > 0 && logEntries().length === 0) {
			this.logBody.replaceChildren();
		} else {
			for (let i = 0; i < removed && this.logBody.firstChild; i++) this.logBody.firstChild.remove();
			const fragment = document.createDocumentFragment();
			for (const e of added) fragment.appendChild(this.makeRow(e));
			this.logBody.appendChild(fragment);
		}
		this.lineCount = logEntries().length;
		if (this.autoScroll) this.scrollLogToEnd();
	}

	private scrollLogToEnd(): void {
		if (this.logScroll) this.logScroll.scrollTop = this.logScroll.scrollHeight;
	}

	private toggleLevel(level: system.LogLevel): void {
		const next = new Set(this.hiddenLevels);
		if (next.has(level)) next.delete(level);
		else next.add(level);
		this.hiddenLevels = next;
		this.updateComplete.then(() => {
			if (this.autoScroll) this.scrollLogToEnd();
		});
	}

	private toggleAutoScroll(): void {
		this.autoScroll = !this.autoScroll;
		if (this.autoScroll) this.scrollLogToEnd();
	}

	// ---- view --------------------------------------------------------------------------------------
	render() {
		const before = this.input.slice(0, this.cursor);
		const atCursor = this.input.slice(this.cursor, this.cursor + 1) || " ";
		const after = this.input.slice(this.cursor + 1);
		const hiddenClasses = [...this.hiddenLevels].map((l) => `hide-${l}`).join(" ");
		return html`
			<div class="terminal-page">
				<section class="term" @click=${() => this.inputElement.focus()}>
					<div class="term-screen" @scroll=${() => this.requestUpdate()}>
						${this.lines.map((l) => html`<div class="term-line term-${l.kind}">${l.text}</div>`)}
						<!-- no whitespace inside: .term-line is white-space: pre-wrap (terminal-app.test.ts) -->
						<div class="term-line term-prompt">${this.busy ? html`<span class="term-busy">...</span>` : html`${PROMPT}${before}<span class="term-cursor">${atCursor}</span>${after}`}</div>
					</div>
					${this.renderBubble()}
					<input
						class="term-keyboard"
						autocomplete="off"
						autocapitalize="off"
						spellcheck="false"
						@keydown=${this.onKeyDown}
						@input=${() => this.syncFromInput()}
						@keyup=${() => this.syncFromInput()}
						@click=${() => this.syncFromInput()}
					/>
				</section>

				<section class="log">
					<div class="log-toolbar">
						<span class="log-title">Log</span>
						${LEVELS.map(
							(l) => html`<button
								class="log-level lvl-${l.level} ${this.hiddenLevels.has(l.level) ? "off" : "on"}"
								title="show/hide ${l.name}"
								@click=${() => this.toggleLevel(l.level)}
							>
								${l.name}
							</button>`,
						)}
						<label class="log-autoscroll">
							<input type="checkbox" .checked=${this.autoScroll} @change=${() => this.toggleAutoScroll()} /> auto-scroll
						</label>
						<button class="log-clear" @click=${() => clearLog()}>clear</button>
						<span class="log-count">${this.lineCount} / ${MAX_LOG_LINES} lines</span>
					</div>
					<div class="log-scroll">
						<table class="log-table ${hiddenClasses}">
							<thead>
								<tr><th class="col-level">Level</th><th class="col-time">Time</th><th class="col-logger">Logger</th><th class="col-text">Text</th></tr>
							</thead>
							<tbody class="log-body"></tbody>
						</table>
					</div>
				</section>
			</div>
		`;
	}
}
