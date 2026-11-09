// The PLC page of the web UI (docs/plc-iec61131.md section 10.2): an IEC 61131-3 programming
// environment after CODESYS and the OpenPLC Editor - project tree, editors for declaration and
// implementation, messages, watch lists, device log, status bar. Mockup state: the device is the
// simulator in the browser (TypeScript VM + plant model); the FCU as a device comes with P2.
import { LitElement, html, nothing, svg } from "lit";
import { customElement, query, state } from "lit/decorators.js";
import "../../styles.css";
import "./plc-ide.css";
import "./st-editor.ts";
import { parse } from "../compiler/parser.ts";
import { Diagnostics } from "../compiler/diagnostics.ts";
import { STDLIB_FILE, STDLIB_SOURCE } from "../compiler/stdlib.ts";
import { KEYWORDS } from "../compiler/lexer.ts";
import { elemOf, typeName, type Type } from "../compiler/types.ts";
import type { Symbol } from "../compiler/compiler.ts";
import type * as A from "../compiler/ast.ts";
import type { DashboardApp } from "../../shell/dashboard-app.ts";
import { IO_VARS, ioSource } from "./io-list.ts";
import { build, exampleProject, loadProject, saveProject, sourceOf, type Build, type Located, type Project } from "./project.ts";
import { Simulator } from "./simulator.ts";
import { formatValue, parseValue, readValue, type Value } from "./values.ts";
import { addRow, deleteRow, freeName, parseRows, updateRow, VAR_CLASSES, type DeclRow } from "./decl-table.ts";
import type { LineMarker, StEditor } from "./st-editor.ts";

type View = "prg" | "io" | "lib";
type Panel = "messages" | "watch" | "log" | "plant";

const WATCH_KEY = "fcu.plc.watch";
const P2 = "kommt mit P2 (Laufzeit auf dem FCU)";
const P3 = "Haltepunkte und Einzelschritt kommen mit P3";
const P4 = "Weitere POUs (FB, FUN, DUT, GVL) kommen mit P4";

interface StdBlock {
	name: string;
	inputs: string[];
	outputs: string[];
}

function stdBlocks(): StdBlock[] {
	const units = parse(STDLIB_SOURCE, STDLIB_FILE, new Diagnostics());
	const t = (v: A.VarDecl) => `${v.name} : ${v.type.kind === "named" ? v.type.name : "…"}`;
	return units.filter((u): u is A.Pou => u.kind === "pou").map((p) => ({
		name: p.name,
		inputs: p.vars.filter((v) => v.cls === "VAR_INPUT").map(t),
		outputs: p.vars.filter((v) => v.cls === "VAR_OUTPUT").map(t),
	}));
}

@customElement("plc-ide-app")
export class PlcIdeApp extends LitElement implements DashboardApp {
	@state() private project: Project = loadProject();
	@state() private lastBuild: Build | null = null;
	@state() private view: View = "prg";
	@state() private declMode: "table" | "text" = "table";
	@state() private panel: Panel = "messages";
	@state() private watch: string[] = [];
	@state() private prepared = new Map<string, string>();
	@state() private tick = 0;
	@state() private downloadedSource: string | null = null;
	@state() private notice: string | null = null;
	@query("#body-editor") private bodyEditor?: StEditor;
	@query("#decl-editor") private declEditor?: StEditor;

	private sim = new Simulator();
	private readonly std = stdBlocks();
	private buildTimer: ReturnType<typeof setTimeout> | null = null;

	constructor() {
		super();
		try {
			this.watch = JSON.parse(localStorage.getItem(WATCH_KEY) ?? "[]") as string[];
		} catch {
			this.watch = [];
		}
		if (this.watch.length === 0) this.watch = ["PLC_PRG.Pieces.CV", "PLC_PRG.Ejected", "ConveyorPwm", "Lightbarrier2"];
		this.sim.addEventListener("update", () => this.tick++);
		this.lastBuild = build(this.project);
	}

	protected createRenderRoot() {
		return this;
	}

	onShow(): void {
		window.addEventListener("keydown", this.onKey);
	}
	onHide(): void {
		window.removeEventListener("keydown", this.onKey);
	}

	private onKey = (e: KeyboardEvent) => {
		if (e.ctrlKey && e.shiftKey && e.key.toUpperCase() === "B") {
			e.preventDefault();
			this.doBuild(true);
		} else if (e.ctrlKey && e.key.toUpperCase() === "S") {
			e.preventDefault();
			saveProject(this.project);
			this.flash("Projekt gespeichert (im Browser)");
		}
	};

	private flash(text: string): void {
		this.notice = text;
		setTimeout(() => {
			if (this.notice === text) this.notice = null;
		}, 2500);
	}

	// ---- project ------------------------------------------------------------------------------------
	private get prg() {
		return this.project.objects[0];
	}

	private edit(change: Partial<{ decl: string; body: string }>): void {
		this.project = { ...this.project, objects: [{ ...this.prg, ...change }] };
		saveProject(this.project);
		// diagnostics while typing (debounced)
		if (this.buildTimer) clearTimeout(this.buildTimer);
		this.buildTimer = setTimeout(() => this.doBuild(false), 300);
	}

	private doBuild(explicit: boolean): Build {
		const b = build(this.project);
		this.lastBuild = b;
		if (explicit) {
			const errors = b.located.filter((l) => l.diagnostic.severity === "error").length;
			const warnings = b.located.length - errors;
			this.flash(errors ? `Übersetzen: ${errors} Fehler, ${warnings} Warnungen` : `Übersetzen erfolgreich${warnings ? `, ${warnings} Warnungen` : ""} - ${b.result.image?.length ?? 0} Byte`);
			if (errors) this.panel = "messages";
		}
		return b;
	}

	private currentSource(): string {
		return sourceOf(this.prg) + ioSource();
	}

	private get stale(): boolean {
		return this.downloadedSource !== null && this.downloadedSource !== this.currentSource();
	}

	private download(start: boolean): void {
		const b = this.doBuild(true);
		if (!b.result.ok) return;
		if (this.sim.download(b.result)) {
			this.downloadedSource = this.currentSource();
			if (start) this.sim.start();
		}
	}

	private exportProject(): void {
		const blob = new Blob([JSON.stringify(this.project, null, 2)], { type: "application/json" });
		const a = document.createElement("a");
		a.href = URL.createObjectURL(blob);
		a.download = `${this.project.name}.fcuplc.json`;
		a.click();
		URL.revokeObjectURL(a.href);
	}

	private importProject(): void {
		const input = document.createElement("input");
		input.type = "file";
		input.accept = ".json,application/json";
		input.onchange = async () => {
			const f = input.files?.[0];
			if (!f) return;
			try {
				const p = JSON.parse(await f.text()) as Project;
				if (p.format !== 1 || !Array.isArray(p.objects) || p.objects.length !== 1) throw new Error("kein FCU-SPS-Projekt (Format 1, eine POU)");
				this.project = p;
				saveProject(p);
				this.doBuild(true);
			} catch (e) {
				this.flash(`Import fehlgeschlagen: ${(e as Error).message}`);
			}
		};
		input.click();
	}

	private resetToExample(): void {
		if (!confirm("Das Beispielprojekt laden? Das aktuelle Projekt im Browser wird ersetzt.")) return;
		this.project = exampleProject();
		saveProject(this.project);
		this.doBuild(true);
	}

	// ---- online values ----------------------------------------------------------------------------
	private get online(): boolean {
		return this.sim.state !== "NO_PROGRAM" && this.sim.debug !== null;
	}

	private resolve(name: string): Symbol | null {
		const d = this.sim.debug;
		if (!d) return null;
		return d.resolve(`${this.prg.name}.${name}`) ?? d.resolve(name);
	}

	private varValue(name: string): { text: string; type: Type } | null {
		const s = this.resolve(name);
		if (!s || elemOf(s.type) === null) return null;
		return { text: formatValue(s.type, readValue(this.sim.data, s)), type: s.type };
	}

	/** Inline values: the variables of each line, as "name = value". */
	private inlineValues(text: string): Map<number, string> {
		const out = new Map<number, string>();
		if (!this.online) return out;
		let inComment = false;
		text.split("\n").forEach((line, i) => {
			let code = "";
			for (let k = 0; k < line.length; k++) {
				if (inComment) {
					if (line.startsWith("*)", k)) {
						inComment = false;
						k++;
					}
					continue;
				}
				if (line.startsWith("(*", k)) {
					inComment = true;
					k++;
					continue;
				}
				if (line.startsWith("//", k)) break;
				code += line[k];
			}
			const seen = new Set<string>();
			const parts: string[] = [];
			for (const m of code.matchAll(/[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*/g)) {
				const name = m[0];
				const prev = code[(m.index ?? 0) - 1];
				if (prev === "#" || KEYWORDS.has(name.toUpperCase()) || seen.has(name.toUpperCase())) continue;
				const v = this.varValue(name);
				if (!v) continue;
				seen.add(name.toUpperCase());
				parts.push(`${name} = ${v.text}`);
			}
			if (parts.length) out.set(i + 1, parts.slice(0, 5).join("   "));
		});
		return out;
	}

	private markers(part: "decl" | "body"): LineMarker[] {
		const ms: LineMarker[] = (this.lastBuild?.located ?? []).filter((l) => l.part === part).map((l) => ({
			line: l.line, severity: l.diagnostic.severity, message: l.diagnostic.message,
		}));
		const err = this.sim.error;
		if (err?.line && this.lastBuild) {
			const declLines = this.lastBuild.declLines;
			const inBody = err.line > declLines;
			if ((part === "body") === inBody) ms.push({ line: inBody ? err.line - declLines : err.line, severity: "runtime", message: `Laufzeitfehler ${err.trap}` });
		}
		return ms;
	}

	private reveal(l: Located): void {
		if (l.part === "decl" || l.part === "body") {
			this.view = "prg";
			if (l.part === "decl") this.declMode = "text";
			void this.updateComplete.then(() => (l.part === "decl" ? this.declEditor : this.bodyEditor)?.reveal(l.line, l.col));
		} else if (l.part === "io") this.view = "io";
	}

	// ---- watch ----------------------------------------------------------------------------------------
	private saveWatch(): void {
		try {
			localStorage.setItem(WATCH_KEY, JSON.stringify(this.watch));
		} catch {
			// ignore
		}
	}

	private watchPath(expr: string): string {
		const d = this.sim.debug;
		if (!d) return expr;
		if (d.resolve(expr)) return expr;
		return `${this.prg.name}.${expr}`;
	}

	private applyPrepared(force: boolean): void {
		let n = 0;
		for (const [expr, text] of this.prepared) {
			const path = this.watchPath(expr);
			const s = this.sim.debug?.resolve(path);
			if (!s) continue;
			const v = parseValue(s.type, text);
			if (v === null) {
				this.flash(`'${text}' ist kein Wert für ${typeName(s.type)} (${expr})`);
				return;
			}
			if (force) this.sim.force(path, v);
			else this.sim.write(path, v);
			n++;
		}
		this.prepared = new Map();
		this.flash(`${n} Wert(e) ${force ? "geforct" : "geschrieben"}`);
	}

	private knownPaths(): string[] {
		const d = this.sim.debug ?? this.lastBuild?.result.debug;
		if (!d) return [];
		const out: string[] = [];
		const walk = (prefix: string, t: Type, depth: number) => {
			if (elemOf(t) !== null) {
				out.push(prefix);
				return;
			}
			if (depth > 2) return;
			const fields = t.k === "fb" ? t.layout.fields : t.k === "struct" ? t.fields : [];
			for (const f of fields) walk(`${prefix}.${f.name}`, f.type, depth + 1);
		};
		for (const [k, s] of d.symbols) {
			if (s.type.k === "fb") walk(this.prg.name.toUpperCase() === k ? this.prg.name : k, s.type, 0);
			else out.push(IO_VARS.find((v) => v.name.toUpperCase() === k)?.name ?? k);
		}
		return out;
	}

	// ---- rendering ----------------------------------------------------------------------------------
	render() {
		void this.tick;
		return html`
			<div class="ide">
				${this.renderToolbar()}
				<div class="ide-main">
					${this.renderTree()}
					<section class="ide-center">
						${this.view === "prg" ? this.renderPrg() : this.view === "io" ? this.renderIo() : this.renderLib()}
					</section>
				</div>
				${this.renderPanel()}
				${this.renderStatus()}
				${this.notice ? html`<div class="ide-toast">${this.notice}</div>` : nothing}
			</div>`;
	}

	private renderToolbar() {
		const s = this.sim.state;
		const ok = this.lastBuild?.result.ok ?? false;
		return html`
			<div class="ide-toolbar" role="toolbar">
				<div class="ide-tb-group">
					<button title="Übersetzen (Strg+Umschalt+B)" @click=${() => this.doBuild(true)}>⚙ Übersetzen</button>
				</div>
				<div class="ide-tb-group">
					<select title="Gerät" class="ide-device">
						<option selected>Simulator (im Browser)</option>
						<option disabled>FCU - ${P2}</option>
					</select>
					<button title="Projekt in den Simulator laden (SPS geht in STOP)" ?disabled=${!ok} @click=${() => this.download(false)}>⇩ Laden</button>
					<button class="ide-primary" title="Übersetzen, laden und starten" ?disabled=${!ok} @click=${() => this.download(true)}>⇩▶ Laden &amp; Starten</button>
				</div>
				<div class="ide-tb-group">
					<button title="Start (RUN)" ?disabled=${s !== "STOP"} @click=${() => this.sim.start()}>▶ Start</button>
					<button title="Stop" ?disabled=${s !== "RUN"} @click=${() => this.sim.stop()}>■ Stop</button>
					<button title="Ein Zyklus (in STOP)" ?disabled=${s !== "STOP"} @click=${() => this.sim.singleCycle()}>⏭ Zyklus</button>
					<button title="Reset warm: Variablen auf Initialwerte" ?disabled=${s === "NO_PROGRAM"} @click=${() => this.sim.reset("warm")}>↺ Reset</button>
				</div>
				<div class="ide-tb-group" title=${P3}>
					<button disabled title="Haltepunkt (P3)">●</button>
					<button disabled title="Prozedurschritt (P3)">⤼</button>
					<button disabled title="Einzelschritt (P3)">⤵</button>
					<button disabled title="Ausführen bis Rücksprung (P3)">⤴</button>
				</div>
				<div class="ide-tb-group ide-tb-right">
					<button title="Alle Force-Werte aufheben" ?disabled=${this.sim.forces.size === 0} @click=${() => this.sim.unforceAll()}>⊘ Forces (${this.sim.forces.size})</button>
				</div>
			</div>`;
	}

	private renderTree() {
		const node = (view: View, icon: string, label: string, sub = "") => html`
			<button class="ide-node ${this.view === view ? "ide-node-active" : ""}" @click=${() => (this.view = view)}>
				<span class="ide-node-icon">${icon}</span><span>${label}</span>${sub ? html`<span class="ide-node-sub">${sub}</span>` : nothing}
			</button>`;
		const s = this.sim.state;
		return html`
			<nav class="ide-tree" aria-label="Projekt">
				<div class="ide-tree-head">
					<input class="ide-project-name" .value=${this.project.name} title="Projektname"
						@change=${(e: Event) => {
							this.project = { ...this.project, name: (e.target as HTMLInputElement).value || "Projekt" };
							saveProject(this.project);
						}} />
				</div>
				<div class="ide-tree-device"><span class="ide-dot ide-dot-${s.toLowerCase()}"></span> Simulator <span class="ide-node-sub">${s}</span></div>
				<div class="ide-tree-group">Applikation</div>
				${node("prg", "📄", this.prg.name, "PRG · ST")}
				<div class="ide-node ide-node-static" title="Ein zyklischer Task; mehrere Tasks: P7"><span class="ide-node-icon">⏱</span><span>MainTask</span><span class="ide-node-sub">${this.sim.cycleMs} ms</span></div>
				<button class="ide-node ide-add" disabled title=${P4}>＋ Objekt hinzufügen…</button>
				<div class="ide-tree-group">Gerät</div>
				${node("io", "🔌", "I/O-Abbild", `${IO_VARS.length} Variablen`)}
				<div class="ide-tree-group">Bibliotheken</div>
				${node("lib", "📚", "Standard", "IEC 61131-3")}
				<div class="ide-tree-actions">
					<button @click=${() => this.exportProject()} title="Projekt als Datei speichern">Export</button>
					<button @click=${() => this.importProject()} title="Projektdatei laden">Import</button>
					<button @click=${() => this.resetToExample()} title="Beispielprojekt laden">Beispiel</button>
				</div>
			</nav>`;
	}

	private renderPrg() {
		const o = this.prg;
		return html`
			<div class="ide-tabs"><span class="ide-tab ide-tab-active">${o.name}${this.stale ? html` <span class="ide-modified" title="geändert seit dem Laden">●</span>` : nothing}</span></div>
			<div class="ide-pou">
				<div class="ide-decl">
					<div class="ide-part-head">
						<span>Deklaration</span>
						<div class="ide-seg" role="group" aria-label="Ansicht der Deklaration">
							<button class=${this.declMode === "table" ? "on" : ""} @click=${() => (this.declMode = "table")}>Tabelle</button>
							<button class=${this.declMode === "text" ? "on" : ""} @click=${() => (this.declMode = "text")}>Text</button>
						</div>
					</div>
					${this.declMode === "table" ? this.renderDeclTable() : html`
						<plc-st-editor id="decl-editor" .value=${o.decl} .markers=${this.markers("decl")} .minLines=${4}
							@change=${(e: CustomEvent<string>) => this.edit({ decl: e.detail })}></plc-st-editor>`}
				</div>
				<div class="ide-impl">
					<div class="ide-part-head">
						<span>Implementierung (ST)</span>
						${this.online ? html`<span class="ide-online ${this.stale ? "ide-online-stale" : ""}">${this.stale ? "Werte des geladenen Programms - Projekt geändert" : "online: Werte"}</span>` : nothing}
					</div>
					<plc-st-editor id="body-editor" .value=${o.body} .markers=${this.markers("body")} .inline=${this.inlineValues(o.body)} .stale=${this.stale} .minLines=${12}
						@change=${(e: CustomEvent<string>) => this.edit({ body: e.detail })}></plc-st-editor>
				</div>
			</div>`;
	}

	private renderDeclTable() {
		const decl = this.prg.decl;
		const rows = parseRows(decl);
		const errorsByLine = new Map<number, string>();
		for (const m of this.markers("decl")) errorsByLine.set(m.line - 1, m.message);
		const set = (r: DeclRow, change: Partial<DeclRow>) => this.edit({ decl: updateRow(decl, r, change) });
		const cell = (r: DeclRow, key: "name" | "type" | "init" | "comment", wide = false) => html`
			<input class=${wide ? "wide" : ""} .value=${r[key]} spellcheck="false"
				@change=${(e: Event) => set(r, { [key]: (e.target as HTMLInputElement).value.trim() })} />`;
		return html`
			<div class="ide-decl-table">
				<table>
					<thead><tr><th>Klasse</th><th>Name</th><th>Typ</th><th>Initialwert</th><th>Kommentar</th>${this.online ? html`<th>Wert</th>` : nothing}<th></th></tr></thead>
					<tbody>
						${rows.map((r) => {
							const err = errorsByLine.get(r.line);
							const v = this.online ? this.varValue(r.name) : null;
							return html`
							<tr class=${err ? "ide-row-error" : ""} title=${err ?? nothing}>
								<td>
									<select @change=${(e: Event) => set(r, { cls: (e.target as HTMLSelectElement).value })}>
										${VAR_CLASSES.map((c) => html`<option ?selected=${c === r.cls}>${c}</option>`)}
									</select>${r.qualifier ? html`<span class="ide-qual">${r.qualifier}</span>` : nothing}
								</td>
								<td>${cell(r, "name")}</td>
								<td>${cell(r, "type")}</td>
								<td>${cell(r, "init")}</td>
								<td>${cell(r, "comment", true)}</td>
								${this.online ? html`<td class="ide-value">${v?.text ?? ""}</td>` : nothing}
								<td><button class="ide-icon-btn" title="Variable löschen" @click=${() => this.edit({ decl: deleteRow(decl, r) })}>✕</button></td>
							</tr>`;
						})}
					</tbody>
				</table>
				<button class="ide-add-row" @click=${() => this.edit({ decl: addRow(decl, { cls: "VAR", qualifier: "", name: freeName(rows), type: "BOOL", init: "", comment: "" }) })}>＋ Variable</button>
			</div>`;
	}

	private renderIo() {
		const online = this.online;
		const input = (name: string, type: Type, current: string) => html`
			<input class="ide-io-edit" .value=${current} placeholder="Wert" title="Eingang setzen (Enter); im Simulator überschreibt das Anlagenmodell ihn im nächsten Zyklus, außer er ist geforct"
				@change=${(e: Event) => {
					const v = parseValue(type, (e.target as HTMLInputElement).value);
					if (v === null) this.flash(`kein gültiger Wert für ${typeName(type)}`);
					else this.sim.write(name, v);
				}} />`;
		let region = "";
		return html`
			<div class="ide-tabs"><span class="ide-tab ide-tab-active">I/O-Abbild <span class="ide-node-sub">aus der Registerkarte erzeugt, nur lesbar</span></span></div>
			<div class="ide-io">
				<p class="ide-hint">Diese globalen Variablen sind in jedem Programm sichtbar. Im Mockup sind es einfache Variablen, die das Anlagenmodell des Simulators liest und schreibt; die Kopplung an die Register (%I/%Q) folgt mit P2.</p>
				<table>
					<thead><tr><th>Name</th><th>Adresse</th><th>Typ</th><th>Richtung</th><th>Wert</th><th>Force</th><th>Beschreibung</th></tr></thead>
					<tbody>
						${IO_VARS.map((v) => {
							const head = v.region !== region ? ((region = v.region), html`<tr class="ide-io-region"><td colspan="7">${v.region}</td></tr>`) : nothing;
							const s = online ? this.sim.debug!.resolve(v.name) : null;
							const value = s ? this.sim.read(v.name).value : null;
							const text = s ? formatValue(s.type, value) : "";
							const forced = this.sim.forces.has(v.name.toUpperCase());
							return html`${head}<tr class=${forced ? "ide-forced" : ""}>
								<td class="ide-mono">${v.name}</td>
								<td class="ide-mono">${v.address}</td>
								<td class="ide-mono">${v.type}</td>
								<td>${v.direction === "input" ? "Eingang" : "Ausgang"}</td>
								<td class="ide-value">${online && v.direction === "input" && s ? input(v.name, s.type, text) : text}</td>
								<td>${s ? html`<input type="checkbox" .checked=${forced} title="Wert festhalten (forcen)"
									@change=${(e: Event) => this.sim.force(v.name, (e.target as HTMLInputElement).checked ? (value as Value) : null)} />` : nothing}</td>
								<td class="ide-desc">${v.description}${v.unit ? html` <span class="ide-node-sub">[${v.unit}]</span>` : nothing}</td>
							</tr>`;
						})}
					</tbody>
				</table>
			</div>`;
	}

	private renderLib() {
		return html`
			<div class="ide-tabs"><span class="ide-tab ide-tab-active">Bibliothek Standard <span class="ide-node-sub">IEC 61131-3 Ed. 3</span></span></div>
			<div class="ide-lib">
				<p class="ide-hint">Funktionsbausteine werden als Instanz deklariert (<code>t : TON;</code>) und formal aufgerufen (<code>t(IN := x, PT := T#1s);</code>), die Ausgänge über <code>t.Q</code> gelesen. Eigene Bibliotheken: P7.</p>
				<div class="ide-lib-grid">
					${this.std.map((b) => html`
						<div class="ide-fb">
							<div class="ide-fb-name">${b.name}</div>
							<div class="ide-fb-body">
								<div>${b.inputs.map((i) => html`<div class="ide-fb-pin">▸ ${i}</div>`)}</div>
								<div class="ide-fb-out">${b.outputs.map((o) => html`<div class="ide-fb-pin">${o} ▸</div>`)}</div>
							</div>
						</div>`)}
				</div>
				<h4>Funktionen</h4>
				<p class="ide-mono ide-funcs">ABS SQRT LN LOG EXP EXPT SIN COS TAN ASIN ACOS ATAN · MIN MAX LIMIT SEL MUX MOVE · SHL SHR ROL ROR · ADD SUB MUL DIV MOD · GT GE EQ LE LT NE · TRUNC · <i>typ</i>_TO_<i>typ</i></p>
			</div>`;
	}

	private renderPanel() {
		const tab = (p: Panel, label: string, badge = "") => html`
			<button class="ide-ptab ${this.panel === p ? "on" : ""}" @click=${() => (this.panel = p)}>${label}${badge ? html` <span class="ide-badge">${badge}</span>` : nothing}</button>`;
		const located = this.lastBuild?.located ?? [];
		const errors = located.filter((l) => l.diagnostic.severity === "error").length + (this.sim.error ? 1 : 0);
		return html`
			<section class="ide-panel">
				<div class="ide-ptabs">
					${tab("messages", "Meldungen", errors ? String(errors) : "")}
					${tab("watch", "Überwachung")}
					${tab("plant", "Anlage (Simulation)")}
					${tab("log", "Geräte-Log")}
				</div>
				<div class="ide-pbody">
					${this.panel === "messages" ? this.renderMessages(located) : this.panel === "watch" ? this.renderWatch() : this.panel === "plant" ? this.renderPlant() : this.renderLog()}
				</div>
			</section>`;
	}

	private renderMessages(located: Located[]) {
		const where = (l: Located) => (l.part === "decl" ? "Deklaration" : l.part === "body" ? "Implementierung" : l.part === "io" ? "I/O" : l.diagnostic.span.start.file);
		const err = this.sim.error;
		if (located.length === 0 && !err) return html`<div class="ide-empty">✔ Keine Meldungen. ${this.lastBuild?.result.ok ? `Image: ${this.lastBuild.result.image!.length} Byte, Daten: ${this.lastBuild.result.debug!.dataSize} Byte.` : ""}</div>`;
		return html`<ul class="ide-messages">
			${err ? html`<li class="ide-msg ide-msg-runtime" @click=${() => {
				const declLines = this.lastBuild?.declLines ?? 0;
				if (err.line) this.reveal({ part: err.line > declLines ? "body" : "decl", line: err.line > declLines ? err.line - declLines : err.line, col: 1, diagnostic: { severity: "error", message: "", span: { start: { file: "", line: 0, col: 0, offset: 0 }, end: { file: "", line: 0, col: 0, offset: 0 } } } });
			}}><span class="ide-msg-icon">⛔</span> Laufzeitfehler <b>${err.trap}</b> in ${err.pou}${err.line ? `, Zeile ${err.line - (this.lastBuild?.declLines ?? 0)} der Implementierung` : ""} - die SPS ist in ERROR. Reset setzt die Variablen zurück.</li>` : nothing}
			${located.map((l) => html`
				<li class="ide-msg ide-msg-${l.diagnostic.severity}" @click=${() => this.reveal(l)}>
					<span class="ide-msg-icon">${l.diagnostic.severity === "error" ? "✖" : "⚠"}</span>
					<span class="ide-msg-loc">${this.prg.name} · ${where(l)} ${l.line}:${l.col}</span>
					<span>${l.diagnostic.message}</span>
				</li>`)}
		</ul>`;
	}

	private renderWatch() {
		const online = this.online;
		const add = (e: Event) => {
			const input = e.target as HTMLInputElement;
			const v = input.value.trim();
			if (v && !this.watch.includes(v)) {
				this.watch = [...this.watch, v];
				this.saveWatch();
			}
			input.value = "";
		};
		return html`
			<div class="ide-watch">
				<table>
					<thead><tr><th>Ausdruck</th><th>Typ</th><th>Wert</th><th>Vorbereiteter Wert</th><th></th></tr></thead>
					<tbody>
						${this.watch.map((expr) => {
							const path = this.watchPath(expr);
							const s = online ? this.sim.debug!.resolve(path) : null;
							const value = s ? this.sim.read(path).value : null;
							const forced = this.sim.forces.has(path.toUpperCase());
							return html`<tr class=${forced ? "ide-forced" : ""}>
								<td class="ide-mono">${expr}</td>
								<td class="ide-mono">${s ? typeName(s.type) : online ? html`<span class="ide-bad">unbekannt</span>` : ""}</td>
								<td class="ide-value">${s ? formatValue(s.type, value) : ""}${forced ? html` <span class="ide-force-tag">F</span>` : nothing}</td>
								<td><input class="ide-prep" .value=${this.prepared.get(expr) ?? ""} ?disabled=${!s} placeholder=${s ? "z.B. TRUE, 42, T#2s" : ""}
									@change=${(e: Event) => {
										const m = new Map(this.prepared);
										const t = (e.target as HTMLInputElement).value.trim();
										if (t) m.set(expr, t);
										else m.delete(expr);
										this.prepared = m;
									}} /></td>
								<td>
									${forced ? html`<button class="ide-icon-btn" title="Force aufheben" @click=${() => this.sim.force(path, null)}>⊘</button>` : nothing}
									<button class="ide-icon-btn" title="Aus der Liste entfernen" @click=${() => {
										this.watch = this.watch.filter((x) => x !== expr);
										this.saveWatch();
									}}>✕</button>
								</td>
							</tr>`;
						})}
					</tbody>
				</table>
				<div class="ide-watch-actions">
					<input list="ide-paths" placeholder="Variable hinzufügen, z.B. PLC_PRG.Eject.Q" @change=${add} />
					<datalist id="ide-paths">${this.knownPaths().map((p) => html`<option value=${p}></option>`)}</datalist>
					<button ?disabled=${!online || this.prepared.size === 0} @click=${() => this.applyPrepared(false)} title="Alle vorbereiteten Werte einmal schreiben (im selben Zyklus)">Werte schreiben</button>
					<button ?disabled=${!online || this.prepared.size === 0} @click=${() => this.applyPrepared(true)} title="Alle vorbereiteten Werte festhalten">Werte forcen</button>
				</div>
			</div>`;
	}

	private renderPlant() {
		const p = this.sim.plant;
		const st = p.state;
		const io = (n: string) => (this.online ? this.sim.read(n).value : null);
		const lb = [io("Lightbarrier1"), io("Lightbarrier2"), io("Lightbarrier3")].map(Boolean);
		const valve = Boolean(io("Valve1"));
		const pwm = Number(io("ConveyorPwm") ?? 0);
		const x = (pos: number) => 20 + (pos / 100) * 560;
		return html`
			<div class="ide-plant">
				<svg viewBox="0 0 600 120" class="ide-plant-svg" role="img" aria-label="Förderband mit Lichtschranken">
					<rect x="20" y="60" width="560" height="14" rx="7" class="belt ${pwm > 0 && this.sim.state === "RUN" ? "belt-on" : ""}" />
					${[30, 70, 96].map((pos, i) => svg`
						<line x1=${x(pos)} y1="20" x2=${x(pos)} y2="58" class="beam ${lb[i] ? "beam-on" : ""}" />
						<text x=${x(pos)} y="14" class="lbl">LS${i + 1}</text>`)}
					<rect x=${x(70) - 12} y="88" width="24" height="12" class="valve ${valve ? "valve-on" : ""}" />
					<text x=${x(70)} y="114" class="lbl">Ventil 1</text>
					${st.pieces.map((pos) => svg`<rect x=${x(pos) - 9} y="42" width="18" height="18" rx="2" class="piece" />`)}
				</svg>
				<div class="ide-plant-facts">
					<span>Band: <b>${pwm}</b> ‰</span>
					<span>Ausgeworfen: <b>${p.ejected}</b></span>
					<span>Am Bandende: <b>${p.delivered}</b></span>
					<span>Druck (roh): <b>${Math.round(st.pressure)}</b></span>
					<label><input type="checkbox" .checked=${this.sim.plantEnabled} @change=${(e: Event) => (this.sim.plantEnabled = (e.target as HTMLInputElement).checked)} /> Anlagenmodell aktiv</label>
				</div>
				<p class="ide-hint">Spielzeugmodell zum Ausprobieren ohne Hardware: Werkstücke laufen über das Band (Geschwindigkeit ∝ ConveyorPwm), Lichtschranken melden sie, Ventil 1 wirft sie an LS2 aus.</p>
			</div>`;
	}

	private renderLog() {
		const lines = this.sim.log.slice(-200).reverse();
		return html`<ul class="ide-log">
			${lines.length === 0 ? html`<li class="ide-empty">Noch keine Einträge.</li>` : nothing}
			${lines.map((l) => html`<li class="ide-log-${l.level}"><span class="ide-mono">${l.time.toLocaleTimeString()}</span> ${l.text}</li>`)}
		</ul>`;
	}

	private renderStatus() {
		const s = this.sim.state;
		return html`
			<footer class="ide-status">
				<span><span class="ide-dot ide-dot-${this.online ? "run" : "no_program"}"></span>${this.online ? "online" : "offline"} · Simulator</span>
				<span class="ide-state ide-state-${s.toLowerCase()}">${s}</span>
				<span>Quelle der Ausgänge: <b>IEC 61131</b></span>
				<span>Zyklus ${this.sim.cycleMs} ms · ${this.sim.cycles} Zyklen</span>
				<span title="ausgeführte Befehle der VM pro Zyklus (Budget 2 000 000)">Befehle ${this.sim.lastInstructions} (max ${this.sim.maxInstructions})</span>
				${this.stale ? html`<span class="ide-warn">⚠ Projekt ≠ Gerät</span>` : nothing}
				${this.sim.forces.size ? html`<span class="ide-warn">${this.sim.forces.size} geforct</span>` : nothing}
				<span class="ide-status-right">Mockup · SPS-Laufzeit auf dem FCU: P2</span>
			</footer>`;
	}
}

declare global {
	interface HTMLElementTagNameMap {
		"plc-ide-app": PlcIdeApp;
	}
}
