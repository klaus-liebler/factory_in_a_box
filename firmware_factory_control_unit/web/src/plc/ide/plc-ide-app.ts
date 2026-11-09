// The PLC page of the web UI (docs/plc-iec61131.md section 10.2): an IEC 61131-3 programming
// environment after CODESYS and the OpenPLC Editor - project tree, editors for declaration and
// implementation, messages, watch lists, device log, status bar. Mockup state: the device is the
// simulator in the browser (TypeScript VM + plant model); the FCU as a device comes with P2.
import { LitElement, html, nothing, svg } from "lit";
import { customElement, query, state } from "lit/decorators.js";
import "../../styles.css";
import "./plc-ide.css";
import "./st-editor.ts";
import "./fbd-editor.ts";
import "./sfc-editor.ts";
import { compile, type CompileResult } from "../compiler/compiler.ts";
import { parse } from "../compiler/parser.ts";
import { Diagnostics } from "../compiler/diagnostics.ts";
import { STDLIB_FILE, STDLIB_SOURCE } from "../compiler/stdlib.ts";
import { KEYWORDS } from "../compiler/lexer.ts";
import { elemOf, typeName, type Type } from "../compiler/types.ts";
import type { Symbol } from "../compiler/compiler.ts";
import type * as A from "../compiler/ast.ts";
import type { DashboardApp } from "../../shell/dashboard-app.ts";
import { IO_VARS, ioBindings, ioMapHash, ioSource } from "./io-list.ts";
import { build, deviceSources, exampleProject, loadProject, parseProject, projectHash, saveProject, type Build, type Located, type Project } from "./project.ts";
import { exampleFbd, fbdSource, type FbdPou, type FbdProblem } from "./fbd.ts";
import { exampleSfc, sfcSource, type SfcPou, type SfcProblem } from "./sfc.ts";
import { Simulator } from "./simulator.ts";
import { FcuDevice, type PlcDevice } from "./device.ts";
import { formatValue, parseValue, readValue, type Value } from "./values.ts";
import { addRow, deleteRow, freeName, parseRows, updateRow, VAR_CLASSES, type DeclRow } from "./decl-table.ts";
import type { LineMarker, StEditor } from "./st-editor.ts";

type View = "prg" | "io" | "lib" | "fbd" | "sfc";

const FBD_KEY = "fcu.plc.preview.fbd";
const SFC_KEY = "fcu.plc.preview.sfc";

function loadJson<T>(key: string, fallback: () => T): T {
	try {
		const raw = localStorage.getItem(key);
		if (raw) return JSON.parse(raw) as T;
	} catch {
		// broken or private mode: the example
	}
	return fallback();
}

function saveJson(key: string, value: unknown): void {
	try {
		localStorage.setItem(key, JSON.stringify(value));
	} catch {
		// not persistent then
	}
}

/** A graphical POU as ST, compiled with the I/O list and the libraries (cached per text). */
interface GfxBuild {
	text: string;
	result: CompileResult;
}
function buildGfx(name: string, text: string, last: GfxBuild | null): GfxBuild {
	if (last && last.text === text) return last;
	return { text, result: compile(deviceSources(name, text), [{ name: "MainTask", intervalCycles: 1, programs: [name] }], { io: ioBindings(), ioMapHash: ioMapHash() }) };
}
type Panel = "messages" | "watch" | "log" | "plant" | "stack";

const WATCH_KEY = "fcu.plc.watch";
const P2 = "kommt mit P2 (Laufzeit auf dem FCU)";
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
	/** Breakpoints: lines of the implementation (body editor), sent to the device as compiled lines. */
	@state() private breakpointLines = new Set<number>();
	@state() private view: View = "prg";
	@state() private declMode: "table" | "text" = "table";
	@state() private panel: Panel = "messages";
	@state() private watch: string[] = [];
	@state() private prepared = new Map<string, string>();
	@state() private tick = 0;
	@state() private notice: string | null = null;
	/** Previews of P5/P6: an FBD and an SFC program of their own (POUs in the project come with P4). */
	@state() private fbd: FbdPou = loadJson(FBD_KEY, exampleFbd);
	@state() private sfc: SfcPou = loadJson(SFC_KEY, exampleSfc);
	private fbdBuild: GfxBuild | null = null;
	private sfcBuild: GfxBuild | null = null;
	@query("#body-editor") private bodyEditor?: StEditor;
	@query("#decl-editor") private declEditor?: StEditor;

	private sim = new Simulator();
	private fcu = new FcuDevice();
	@state() private dev: PlcDevice = this.sim;
	private shown = false;
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
		this.fcu.addEventListener("update", () => this.tick++);
		this.lastBuild = build(this.project);
		this.fcu.attach(this.lastBuild.result);
		// a link to a view: /plc?view=fbd
		const view = new URLSearchParams(window.location.search).get("view");
		if (view === "prg" || view === "io" || view === "lib" || view === "fbd" || view === "sfc") this.view = view;
	}

	protected createRenderRoot() {
		return this;
	}

	onShow(): void {
		window.addEventListener("keydown", this.onKey);
		this.shown = true;
		this.dev.activate();
	}
	onHide(): void {
		window.removeEventListener("keydown", this.onKey);
		this.shown = false;
		this.dev.deactivate();
	}

	private selectDevice(kind: string): void {
		const next = kind === "fcu" ? this.fcu : this.sim;
		if (next === this.dev) return;
		this.dev.deactivate();
		this.dev = next;
		if (this.shown) this.dev.activate();
		this.flash(next.kind === "fcu" ? "Gerät: FCU (über die WebSocket-Verbindung)" : "Gerät: Simulator im Browser");
	}

	private onKey = (e: KeyboardEvent) => {
		if (e.ctrlKey && e.shiftKey && e.key.toUpperCase() === "B") {
			e.preventDefault();
			this.doBuild(true);
		} else if (e.ctrlKey && e.key.toUpperCase() === "S") {
			e.preventDefault();
			saveProject(this.project);
			this.flash("Projekt gespeichert (im Browser)");
		} else if (this.dev.state === "HALTED" && (e.key === "F5" || e.key === "F10" || e.key === "F11")) {
			e.preventDefault();
			if (e.key === "F5") void this.dev.continueRun();
			else if (e.key === "F10") void this.dev.step("over");
			else void this.dev.step(e.shiftKey ? "out" : "into");
		}
	};

	/** On a new halt the call stack comes to the front (once per halt, the user may switch away). */
	private lastHalt: unknown = null;
	protected willUpdate(): void {
		const h = this.dev.state === "HALTED" ? this.dev.halt : null;
		if (h && h !== this.lastHalt && this.lastHalt === null) this.panel = "stack";
		this.lastHalt = h;
	}

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
		this.fcu.attach(b.result);
		if (explicit) {
			const errors = b.located.filter((l) => l.diagnostic.severity === "error").length;
			const warnings = b.located.length - errors;
			this.flash(errors ? `Übersetzen: ${errors} Fehler, ${warnings} Warnungen` : `Übersetzen erfolgreich${warnings ? `, ${warnings} Warnungen` : ""} - ${b.result.image?.length ?? 0} Byte`);
			if (errors) this.panel = "messages";
		}
		return b;
	}

	/** The device runs another state of the project than the editor shows. */
	private get stale(): boolean {
		return this.dev.projectHash !== 0 && this.dev.projectHash !== projectHash(this.project);
	}

	private async download(start: boolean): Promise<void> {
		const b = this.doBuild(true);
		if (!b.result.ok) return;
		const dev = this.dev;
		if (dev.kind === "fcu") {
			if (!dev.connected) {
				this.flash("Das FCU antwortet nicht");
				return;
			}
			if (start && dev.iec && !confirm("Das Programm übernimmt die Ausgänge des FCU (Steuerquelle IEC 61131): Ventile, Motoren und Achsen können sich bewegen. Laden und starten?")) return;
		}
		if (await dev.download(b.result)) {
			this.sendBreakpoints();  // the new program's statement offsets
			// the source next to the program, for "from the device" in another browser (a missing card: a warning only)
			if (dev.storeProject) await dev.storeProject(JSON.stringify(this.project));
			if (start) await dev.start();
		} else {
			this.panel = "log";
		}
	}

	// ---- debugging: breakpoints and the halted line ----------------------------------------------------
	/** Body lines that can take a breakpoint (a statement begins there), from the last build. */
	private get breakableBody(): Set<number> | null {
		const b = this.lastBuild;
		if (!this.dev.canDebug || !b?.result.debug) return null;
		const file = this.project.objects[0].name;
		const out = new Set<number>();
		for (const s of b.result.debug.statements) {
			if (s.span.start.file === file && s.span.start.line > b.declLines) out.add(s.span.start.line - b.declLines);
		}
		return out;
	}

	/** The body line the program halted on (0: not halted there). */
	private get haltedBodyLine(): number {
		const top = this.dev.state === "HALTED" ? this.dev.halt?.frames[0] : undefined;
		const b = this.lastBuild;
		if (!top || !b || top.file !== this.project.objects[0].name) return 0;
		return top.line - b.declLines;
	}

	private toggleBreakpoint(line: number): void {
		const next = new Set(this.breakpointLines);
		const adding = !next.delete(line);
		if (adding && this.breakpointLines.size === 0 && this.dev.kind === "fcu" && this.dev.iec &&
			!confirm("Haltepunkt auf dem FCU mit Steuerquelle IEC 61131: hält das Programm dort an, behalten die Ausgänge ihren Wert (Ventile, Motoren!), der E/A-Zyklus läuft weiter. Setzen?")) {
			return;
		}
		if (adding) next.add(line);
		this.breakpointLines = next;
		this.sendBreakpoints();
	}

	private clearBreakpoints(): void {
		this.breakpointLines = new Set();
		this.sendBreakpoints();
	}

	private sendBreakpoints(): void {
		const b = this.lastBuild;
		if (!b || !this.dev.canDebug) return;
		const offset = b.declLines;
		const set = this.dev.setBreakpoints(this.project.objects[0].name, [...this.breakpointLines].map((l) => l + offset));
		this.breakpointLines = new Set(set.map((l) => l - offset));  // lines without a statement drop out
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
				const p = parseProject(await f.text());
				this.project = p;
				saveProject(p);
				this.doBuild(true);
			} catch (e) {
				this.flash(`Import fehlgeschlagen: ${(e as Error).message}`);
			}
		};
		input.click();
	}

	/** "Upload from device": the project source stored next to the program (SD card). */
	private async projectFromDevice(): Promise<void> {
		const dev = this.dev;
		if (!dev.loadProject || !dev.connected) {
			this.flash("Das Gerät antwortet nicht");
			return;
		}
		if (!confirm("Das Projekt vom Gerät laden? Das aktuelle Projekt im Browser wird ersetzt.")) return;
		const text = await dev.loadProject();
		if (text === null) {
			this.panel = "log";
			return;
		}
		try {
			const p = parseProject(text);
			this.project = p;
			saveProject(p);
			this.doBuild(true);
			const same = dev.projectHash !== 0 && dev.projectHash === projectHash(p);
			this.flash(same ? "Projekt vom Gerät geladen - es passt zum Programm auf dem Gerät"
				: "Projekt vom Gerät geladen - ACHTUNG: es ist nicht der Stand des Programms auf dem Gerät");
		} catch (e) {
			this.flash(`Projekt vom Gerät unbrauchbar: ${(e as Error).message}`);
		}
	}

	private resetToExample(): void {
		if (!confirm("Das Beispielprojekt laden? Das aktuelle Projekt im Browser wird ersetzt.")) return;
		this.project = exampleProject();
		saveProject(this.project);
		this.doBuild(true);
	}

	// ---- online values ----------------------------------------------------------------------------
	private get online(): boolean {
		return this.dev.state !== "NO_PROGRAM" && this.dev.debug !== null;
	}

	private resolve(name: string): Symbol | null {
		const d = this.dev.debug;
		if (!d) return null;
		return d.resolve(`${this.prg.name}.${name}`) ?? d.resolve(name);
	}

	private varValue(name: string): { text: string; type: Type } | null {
		const s = this.resolve(name);
		if (!s || elemOf(s.type) === null) return null;
		return { text: formatValue(s.type, readValue(this.dev.data, s)), type: s.type };
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
		const err = this.dev.error;
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
		const d = this.dev.debug;
		if (!d) return expr;
		if (d.resolve(expr)) return expr;
		return `${this.prg.name}.${expr}`;
	}

	private applyPrepared(force: boolean): void {
		let n = 0;
		for (const [expr, text] of this.prepared) {
			const path = this.watchPath(expr);
			const s = this.dev.debug?.resolve(path);
			if (!s) continue;
			const v = parseValue(s.type, text);
			if (v === null) {
				this.flash(`'${text}' ist kein Wert für ${typeName(s.type)} (${expr})`);
				return;
			}
			if (force) this.dev.force(path, v);
			else this.dev.write(path, v);
			n++;
		}
		this.prepared = new Map();
		this.flash(`${n} Wert(e) ${force ? "geforct" : "geschrieben"}`);
	}

	private knownPaths(): string[] {
		const d = this.dev.debug ?? this.lastBuild?.result.debug;
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
						${this.view === "prg" ? this.renderPrg() : this.view === "io" ? this.renderIo() : this.view === "fbd" ? this.renderFbd()
							: this.view === "sfc" ? this.renderSfc() : this.renderLib()}
					</section>
				</div>
				${this.renderPanel()}
				${this.renderStatus()}
				${this.notice ? html`<div class="ide-toast">${this.notice}</div>` : nothing}
			</div>`;
	}

	private renderToolbar() {
		const s = this.dev.state;
		const ok = this.lastBuild?.result.ok ?? false;
		return html`
			<div class="ide-toolbar" role="toolbar">
				<div class="ide-tb-group">
					<button title="Übersetzen (Strg+Umschalt+B)" @click=${() => this.doBuild(true)}>⚙ Übersetzen</button>
				</div>
				<div class="ide-tb-group">
					<select title="Gerät" class="ide-device" @change=${(e: Event) => this.selectDevice((e.target as HTMLSelectElement).value)}>
						<option value="sim" ?selected=${this.dev.kind === "simulator"}>Simulator (im Browser)</option>
						<option value="fcu" ?selected=${this.dev.kind === "fcu"}>FCU (dieses Gerät)</option>
					</select>
					<button title="Projekt in den Simulator laden (SPS geht in STOP)" ?disabled=${!ok} @click=${() => this.download(false)}>⇩ Laden</button>
					<button class="ide-primary" title="Übersetzen, laden und starten" ?disabled=${!ok} @click=${() => this.download(true)}>⇩▶ Laden &amp; Starten</button>
				</div>
				<div class="ide-tb-group">
					<button title="Start (RUN)" ?disabled=${s !== "STOP"} @click=${() => this.dev.start()}>▶ Start</button>
					<button title="Stop" ?disabled=${s !== "RUN" && s !== "HALTED"} @click=${() => this.dev.stop()}>■ Stop</button>
					<button title=${this.dev.canSingleCycle ? "Ein Zyklus (in STOP)" : "Einzelne Zyklen: nur im Simulator"} ?disabled=${s !== "STOP" || !this.dev.canSingleCycle} @click=${() => this.dev.singleCycle()}>⏭ Zyklus</button>
					<button title="Reset warm: Variablen auf Initialwerte" ?disabled=${s === "NO_PROGRAM"} @click=${() => this.dev.reset("warm")}>↺ Reset</button>
				</div>
				<div class="ide-tb-group" title=${this.dev.canDebug ? "Debuggen: Haltepunkt per Klick auf die Zeilennummer" : "Haltepunkte und Einzelschritt: auf dem FCU noch nicht (P3)"}>
					<button title="Weiter (F5): bis zum nächsten Haltepunkt" ?disabled=${s !== "HALTED"} @click=${() => this.dev.continueRun()}>▶ Weiter</button>
					<button title="Prozedurschritt (F10): die nächste Anweisung, Aufrufe laufen durch" ?disabled=${s !== "HALTED"} @click=${() => this.dev.step("over")}>⤼</button>
					<button title="Einzelschritt (F11): in den Aufruf hinein" ?disabled=${s !== "HALTED"} @click=${() => this.dev.step("into")}>⤵</button>
					<button title="Ausführen bis Rücksprung (Umschalt+F11)" ?disabled=${s !== "HALTED"} @click=${() => this.dev.step("out")}>⤴</button>
					<button title="Alle Haltepunkte entfernen" ?disabled=${this.breakpointLines.size === 0} @click=${() => this.clearBreakpoints()}>○ Haltepunkte (${this.breakpointLines.size})</button>
				</div>
				${this.dev.hasControlSource ? html`
				<div class="ide-tb-group" title="Wer schaltet die Ausgänge des FCU? Feldbus: Modbus, OPC UA, Web-UI. IEC 61131: das Programm (externe Schreibzugriffe werden abgewiesen).">
					<span class="ide-tb-label">Ausgänge:</span>
					<div class="ide-seg" role="group" aria-label="Steuerquelle">
						<button class=${!this.dev.iec ? "on" : ""} ?disabled=${!this.dev.connected} @click=${() => this.dev.setSource(false)}>Feldbus</button>
						<button class=${this.dev.iec ? "on" : ""} ?disabled=${!this.dev.connected} @click=${() => this.dev.setSource(true)}>IEC 61131</button>
					</div>
					<label class="ide-tb-check" title="Das Programm startet nach dem Einschalten von selbst"><input type="checkbox" .checked=${this.dev.autostart} ?disabled=${!this.dev.connected}
						@change=${(e: Event) => this.dev.setAutostart((e.target as HTMLInputElement).checked)} /> Autostart</label>
				</div>` : nothing}
				<div class="ide-tb-group ide-tb-right">
					<button title="Alle Force-Werte aufheben" ?disabled=${this.dev.forcedOnDevice === 0 || this.dev.leasedByOther} @click=${() => this.dev.unforceAll()}>⊘ Forces (${this.dev.forcedOnDevice})</button>
				</div>
			</div>`;
	}

	// ---- graphical editors (previews of P5/P6) -------------------------------------------------------
	/** Online: the graphical program runs in the simulator (its build is the simulator's program). */
	private gfxOnline(b: GfxBuild | null): boolean {
		return !!b?.result.ok && this.dev.kind === "simulator" && this.dev.debug === b.result.debug && this.dev.state !== "NO_PROGRAM";
	}

	/** The value of a variable path or expression for the diagram: program-local first, then global. */
	private gfxValue(pou: string, expr: string): string | null {
		const e = expr.trim();
		if (!/^[A-Za-z_][A-Za-z0-9_.]*$/.test(e)) return null;
		for (const path of [`${pou}.${e}`, e]) {
			const r = this.dev.read(path);
			if (r.symbol && r.value !== null) return formatValue(r.symbol.type, r.value);
		}
		return null;
	}

	/** Load the graphical program into the simulator and start it. */
	private async runGfx(b: GfxBuild): Promise<void> {
		if (!b.result.ok) return;
		this.selectDevice("sim");
		if (await this.dev.download(b.result)) await this.dev.start();
	}

	private renderGfx(kind: "fbd" | "sfc", name: string, decl: string, setDecl: (d: string) => void, problems: (FbdProblem | SfcProblem)[],
		b: GfxBuild, editor: unknown) {
		const errors = b.result.diagnostics.filter((d) => d.severity === "error");
		const online = this.gfxOnline(b);
		return html`
			<div class="gfx-view">
				<p class="gfx-intro">${kind === "fbd"
					? "Funktionsbausteinsprache (FBD): Bausteine aus der Palette, Verbindungen vom Ausgang zum Eingang ziehen, Eigenschaften rechts."
					: "Ablaufsprache (SFC): Schritte mit Aktionen, Transitionen mit Bedingungen, Alternativ- und Parallelzweige, Sprünge."}
					Vorschau (P5/P6): ein eigenes Programm, das zu ST übersetzt wird; als POU im Projekt mit P4.</p>
				<div class="ide-part">
					<div class="ide-part-head"><span>Deklaration</span></div>
					<plc-st-editor .value=${decl} .minLines=${3} @change=${(e: CustomEvent<string>) => setDecl(e.detail)}></plc-st-editor>
				</div>
				${editor}
				<div class="gfx-actions">
					<button class="ide-primary" ?disabled=${!b.result.ok} @click=${() => this.runGfx(b)}
						title="Übersetzen, in den Simulator laden und starten">⇩▶ Im Simulator ausführen</button>
					${online ? html`<button @click=${() => this.dev.stop()} ?disabled=${this.dev.state !== "RUN"}>■ Stop</button>
						<span class="gfx-status-ok">online: läuft im Simulator (${this.dev.state})</span>` : nothing}
					${problems.length || errors.length
						? html`<span class="gfx-status-bad">${problems.length + errors.length} Fehler - siehe unten und im generierten ST</span>`
						: html`<span class="gfx-status-ok">übersetzt: ${b.result.image?.length ?? 0} Byte</span>`}
				</div>
				<details class="gfx-st" ?open=${errors.length > 0}>
					<summary>Generiertes ST (${name})</summary>
					${errors.length ? html`<ul class="fbd-problems">${errors.map((d) => html`<li>Zeile ${d.span.start.line}: ${d.message}</li>`)}</ul>` : nothing}
					<plc-st-editor readonly .value=${b.text} .minLines=${6}
						.markers=${errors.filter((d) => d.span.start.file === name).map((d) => ({ line: d.span.start.line, severity: "error", message: d.message }))}></plc-st-editor>
				</details>
			</div>`;
	}

	private renderFbd() {
		const src = fbdSource(this.fbd);
		this.fbdBuild = buildGfx(this.fbd.name, src.text, this.fbdBuild);
		const b = this.fbdBuild;
		const online = this.gfxOnline(b);
		const set = (p: FbdPou) => {
			this.fbd = p;
			saveJson(FBD_KEY, p);
		};
		return this.renderGfx("fbd", this.fbd.name, this.fbd.decl, (d) => set({ ...this.fbd, decl: d }), src.problems, b, html`
			<plc-fbd-editor .pou=${this.fbd} .problems=${src.problems}
				.lookup=${online ? (e: string) => this.gfxValue(this.fbd.name, e) : null}
				@change=${(e: CustomEvent<FbdPou>) => set(e.detail)}></plc-fbd-editor>`);
	}

	private renderSfc() {
		const src = sfcSource(this.sfc);
		this.sfcBuild = buildGfx(this.sfc.name, src.text, this.sfcBuild);
		const b = this.sfcBuild;
		let active: Map<string, string> | null = null;
		if (this.gfxOnline(b)) {
			active = new Map();
			const debug = b.result.debug!;
			for (const s of debug.statements.length ? [...src.text.matchAll(/^ {4}(\w+) : SFC_STEP;$/gm)].map((m) => m[1]) : []) {
				const x = this.dev.read(`${this.sfc.name}.${s}.X`);
				if (x.value === true) {
					const t = this.dev.read(`${this.sfc.name}.${s}.T`);
					active.set(s.toUpperCase(), t.symbol && t.value !== null ? formatValue(t.symbol.type, t.value) : "");
				}
			}
		}
		const set = (p: SfcPou) => {
			this.sfc = p;
			saveJson(SFC_KEY, p);
		};
		return this.renderGfx("sfc", this.sfc.name, this.sfc.decl, (d) => set({ ...this.sfc, decl: d }), src.problems, b, html`
			<plc-sfc-editor .pou=${this.sfc} .problems=${src.problems} .active=${active}
				@change=${(e: CustomEvent<SfcPou>) => set(e.detail)}></plc-sfc-editor>`);
	}

	private renderTree() {
		const node = (view: View, icon: string, label: string, sub = "") => html`
			<button class="ide-node ${this.view === view ? "ide-node-active" : ""}" @click=${() => (this.view = view)}>
				<span class="ide-node-icon">${icon}</span><span>${label}</span>${sub ? html`<span class="ide-node-sub">${sub}</span>` : nothing}
			</button>`;
		const s = this.dev.state;
		return html`
			<nav class="ide-tree" aria-label="Projekt">
				<div class="ide-tree-head">
					<input class="ide-project-name" .value=${this.project.name} title="Projektname"
						@change=${(e: Event) => {
							this.project = { ...this.project, name: (e.target as HTMLInputElement).value || "Projekt" };
							saveProject(this.project);
						}} />
				</div>
				<div class="ide-tree-device"><span class="ide-dot ide-dot-${this.dev.connected ? s.toLowerCase() : "offline"}"></span> ${this.dev.label} <span class="ide-node-sub">${this.dev.connected ? s : "offline"}</span></div>
				<div class="ide-tree-group">Applikation</div>
				${node("prg", "📄", this.prg.name, "PRG · ST")}
				<div class="ide-node ide-node-static" title="Ein zyklischer Task; mehrere Tasks: P7"><span class="ide-node-icon">⏱</span><span>MainTask</span><span class="ide-node-sub">${this.dev.cycleMs} ms</span></div>
				<button class="ide-node ide-add" disabled title=${P4}>＋ Objekt hinzufügen…</button>
				<div class="ide-tree-group" title="Grafische Editoren als Vorschau (P5/P6): je ein eigenes Programm, als POU im Projekt mit P4">Vorschau</div>
				${node("fbd", "🔲", this.fbd.name, "PRG · FBD")}
				${node("sfc", "⛓", this.sfc.name, "PRG · SFC")}
				<div class="ide-tree-group">Gerät</div>
				${node("io", "🔌", "I/O-Abbild", `${IO_VARS.length} Variablen`)}
				<div class="ide-tree-group">Bibliotheken</div>
				${node("lib", "📚", "Standard", "IEC 61131-3")}
				<div class="ide-tree-actions">
					<button @click=${() => this.exportProject()} title="Projekt als Datei speichern">Export</button>
					<button @click=${() => this.importProject()} title="Projektdatei laden">Import</button>
					${this.dev.loadProject ? html`<button @click=${() => this.projectFromDevice()} ?disabled=${!this.dev.connected}
						title="Das beim letzten Laden auf dem Gerät (SD-Karte) gespeicherte Projekt holen">Vom Gerät</button>` : ""}
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
						.breakpoints=${this.breakpointLines} .breakable=${this.breakableBody} .current=${this.haltedBodyLine}
						@toggle-breakpoint=${(e: CustomEvent<number>) => this.toggleBreakpoint(e.detail)}
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
					else this.dev.write(name, v);
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
							const s = online ? this.dev.debug!.resolve(v.name) : null;
							const value = s ? this.dev.read(v.name).value : null;
							const text = s ? formatValue(s.type, value) : "";
							const forced = this.dev.forces.has(v.name.toUpperCase());
							return html`${head}<tr class=${forced ? "ide-forced" : ""}>
								<td class="ide-mono">${v.name}</td>
								<td class="ide-mono">${v.address}</td>
								<td class="ide-mono">${v.type}</td>
								<td>${v.direction === "input" ? "Eingang" : "Ausgang"}</td>
								<td class="ide-value">${online && v.direction === "input" && s ? input(v.name, s.type, text) : text}</td>
								<td>${s ? html`<input type="checkbox" .checked=${forced} ?disabled=${!this.dev.canForce} title=${this.dev.canForce ? "Wert festhalten (forcen)" : "Forcen auf dem FCU: P3"}
									@change=${(e: Event) => this.dev.force(v.name, (e.target as HTMLInputElement).checked ? (value as Value) : null)} />` : nothing}</td>
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

	/** The call stack of a halt: the halted statement first, then each caller at its call. */
	private renderStack() {
		const h = this.dev.state === "HALTED" ? this.dev.halt : null;
		if (!h) return html`<p class="ide-empty">Das Programm ist nicht angehalten. Haltepunkt: Klick auf eine Zeilennummer der Implementierung.</p>`;
		const user = this.project.objects[0].name;
		const decl = this.lastBuild?.declLines ?? 0;
		return html`
			<p class="ide-stack-why">Angehalten: ${h.reason === "breakpoint" ? "Haltepunkt" : "Einzelschritt"} - das Programm wartet auf „Weiter“ oder einen Schritt (es läuft nie von selbst weiter).</p>
			<table class="ide-table">
				<thead><tr><th></th><th>POU</th><th>Zeile</th><th>Quelle</th></tr></thead>
				<tbody>${h.frames.map((f, i) => html`
					<tr class=${i === 0 ? "ide-stack-top" : ""}>
						<td>${i === 0 ? "▶" : ""}</td>
						<td>${f.pou}</td>
						<td>${f.file === user ? f.line - decl : f.line}</td>
						<td>${f.file === user ? "Implementierung" : f.file === "<stdlib>" ? "Standardbibliothek" : f.file === "<fcu>" ? "FCU-Bibliothek" : f.file}</td>
					</tr>`)}
				</tbody>
			</table>`;
	}

	private renderPanel() {
		const tab = (p: Panel, label: string, badge = "") => html`
			<button class="ide-ptab ${this.panel === p ? "on" : ""}" @click=${() => (this.panel = p)}>${label}${badge ? html` <span class="ide-badge">${badge}</span>` : nothing}</button>`;
		const located = this.lastBuild?.located ?? [];
		const errors = located.filter((l) => l.diagnostic.severity === "error").length + (this.dev.error ? 1 : 0);
		return html`
			<section class="ide-panel">
				<div class="ide-ptabs">
					${tab("messages", "Meldungen", errors ? String(errors) : "")}
					${tab("watch", "Überwachung")}
					${tab("plant", "Anlage (Simulation)")}
					${tab("log", "Geräte-Log")}
					${tab("stack", "Aufrufliste", this.dev.state === "HALTED" ? "⏸" : "")}
				</div>
				<div class="ide-pbody">
					${this.panel === "messages" ? this.renderMessages(located) : this.panel === "watch" ? this.renderWatch()
						: this.panel === "plant" ? this.renderPlant() : this.panel === "stack" ? this.renderStack() : this.renderLog()}
				</div>
			</section>`;
	}

	private renderMessages(located: Located[]) {
		const where = (l: Located) => (l.part === "decl" ? "Deklaration" : l.part === "body" ? "Implementierung" : l.part === "io" ? "I/O" : l.diagnostic.span.start.file);
		const err = this.dev.error;
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
							const s = online ? this.dev.debug!.resolve(path) : null;
							const value = s ? this.dev.read(path).value : null;
							const forced = this.dev.forces.has(path.toUpperCase());
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
									${forced ? html`<button class="ide-icon-btn" title="Force aufheben" @click=${() => this.dev.force(path, null)}>⊘</button>` : nothing}
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
					<button ?disabled=${!online || this.prepared.size === 0 || !this.dev.canForce} @click=${() => this.applyPrepared(true)} title=${this.dev.canForce ? "Alle vorbereiteten Werte festhalten" : "Forcen auf dem FCU: P3"}>Werte forcen</button>
				</div>
			</div>`;
	}

	private renderPlant() {
		if (!this.dev.hasPlant) return html`<div class="ide-hint ide-pad">Das Anlagenmodell gehört zum Simulator. Auf dem FCU arbeitet das Programm mit der echten Anlage.</div>`;
		const p = this.sim.plant;
		const st = p.state;
		const io = (n: string) => (this.online ? this.dev.read(n).value : null);
		const lb = [io("Lightbarrier1"), io("Lightbarrier2"), io("Lightbarrier3")].map(Boolean);
		const valve = Boolean(io("Valve1"));
		const pwm = Number(io("ConveyorPwm") ?? 0);
		const x = (pos: number) => 20 + (pos / 100) * 560;
		return html`
			<div class="ide-plant">
				<svg viewBox="0 0 600 120" class="ide-plant-svg" role="img" aria-label="Förderband mit Lichtschranken">
					<rect x="20" y="60" width="560" height="14" rx="7" class="belt ${pwm > 0 && this.dev.state === "RUN" ? "belt-on" : ""}" />
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
		const lines = this.dev.log.slice(-200).reverse();
		return html`<ul class="ide-log">
			${lines.length === 0 ? html`<li class="ide-empty">Noch keine Einträge.</li>` : nothing}
			${lines.map((l) => html`<li class="ide-log-${l.level}"><span class="ide-mono">${l.time.toLocaleTimeString()}</span> ${l.text}</li>`)}
		</ul>`;
	}

	private renderStatus() {
		const s = this.dev.state;
		return html`
			<footer class="ide-status">
				<span><span class="ide-dot ide-dot-${this.online ? "run" : "no_program"}"></span>${this.online ? "online" : this.dev.connected ? "keine Werte" : "offline"} · ${this.dev.label}</span>
				<span class="ide-state ide-state-${s.toLowerCase()}">${s}</span>
				<span>Quelle der Ausgänge: <b>${this.dev.iec ? "IEC 61131" : "Feldbus"}</b></span>
				${this.dev.storeProblem ? html`<span class="ide-warn" title=${this.dev.storeProblem}>⚠ Laden gesperrt</span>` : nothing}
				<span>Zyklus ${this.dev.cycleMs} ms · ${this.dev.cycles} Zyklen</span>
				<span title="ausgeführte Befehle der VM pro Zyklus (Budget 2 000 000)">Befehle ${this.dev.lastInstructions} (max ${this.dev.maxInstructions})</span>
				${this.stale ? html`<span class="ide-warn">⚠ Projekt ≠ Gerät</span>` : nothing}
				${this.dev.forcedOnDevice ? html`<span class="ide-warn" title="geforcte Werte auf dem Gerät (aller Browser); STOP, Umschalten oder 'plc unforce' im Terminal heben sie auf">${this.dev.forcedOnDevice} geforct</span>` : nothing}
				${this.dev.leasedByOther ? html`<span class="ide-warn" title="Ein anderer Browser forct Werte (Debug-Lease): hier nur beobachten; Aufheben dort oder mit 'plc unforce' im Terminal">🔒 anderer Browser debuggt</span>` : nothing}
				<span class="ide-status-right">${this.dev.kind === "fcu" && this.dev.connected && this.dev.projectHash !== 0 && !this.dev.debug ? "Programm auf dem FCU stammt nicht aus diesem Browser: keine Online-Werte" : ""}</span>
			</footer>`;
	}
}

declare global {
	interface HTMLElementTagNameMap {
		"plc-ide-app": PlcIdeApp;
	}
}
