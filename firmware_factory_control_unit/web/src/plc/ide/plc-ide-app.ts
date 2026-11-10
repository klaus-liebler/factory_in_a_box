// The PLC page of the web UI (docs/plc-iec61131.md section 10.2): an IEC 61131-3 programming
// environment after CODESYS and the OpenPLC Editor - project tree, editors for declaration and
// implementation, messages, watch lists, device log. The device is the FCU or the simulator in the
// browser (TypeScript VM + plant model). Its state goes to the application's status bar.
import { LitElement, html, nothing, svg } from "lit";
import { customElement, query, state } from "lit/decorators.js";
import { repeat } from "lit/directives/repeat.js";
import { choiceDialog, confirmDialog, messageDialog, promptDialog } from "../../shell/dialogs.ts";
import { setAppStatus, type StatusKind } from "../../shell/status-bar.ts";
import "../../styles.css";
import "./plc-ide.css";
import "./st-editor.ts";
import "./fbd-editor.ts";
import "./sfc-editor.ts";
import { parse } from "../compiler/parser.ts";
import { Diagnostics } from "../compiler/diagnostics.ts";
import { STDLIB_FILE, STDLIB_SOURCE } from "../compiler/stdlib.ts";
import { KEYWORDS } from "../compiler/lexer.ts";
import { elemOf, typeName, type Type } from "../compiler/types.ts";
import type { Symbol } from "../compiler/compiler.ts";
import type * as A from "../compiler/ast.ts";
import type { DashboardApp } from "../../shell/dashboard-app.ts";
import { IO_VARS } from "./io-list.ts";
import {
	addObject, boxesFor, build, crossReferences, objectTree, type CrossRef,
	addFolder, allFolders, deleteFolder, folderNameProblem, moveFolder, moveFolderProblem, moveObject, objectsIn, renameFolder, type FolderNode, declLines, LANG_NAMES, LANGS_OF, type Generated, type Lang, exampleProject, hasBody, headerName, KIND_NAMES, loadProject, nameProblem, OBJECT_KINDS,
	parseProject, partLine, projectHash, removeObject, renameObject, saveProject, type Build, type Located, type ObjectKind, type PlcObject, type Project,
} from "./project.ts";
import type { FbdPou } from "./fbd.ts";
import { sfcWalk, type SfcPou } from "./sfc.ts";
import { Simulator } from "./simulator.ts";
import { FcuDevice, type PlcDevice } from "./device.ts";
import { formatValue, parseValue, readValue, type Value } from "./values.ts";
import { addRow, deleteRow, freeName, GVL_CLASSES, parseRows, updateRow, VAR_CLASSES, type DeclRow } from "./decl-table.ts";
import type { Completion, CompletionContext, LineMarker, StEditor } from "./st-editor.ts";
import { ELEM } from "../compiler/types.ts";
import type { DebugInfo } from "../compiler/compiler.ts";

/** The standard functions (the compiler's intrinsics) for code completion. */
const STD_FUNCTIONS = ["ABS", "SQRT", "LN", "LOG", "EXP", "EXPT", "SIN", "COS", "TAN", "ASIN", "ACOS", "ATAN", "MIN", "MAX", "LIMIT", "SEL",
	"MUX", "MOVE", "SHL", "SHR", "ROL", "ROR", "ADD", "SUB", "MUL", "DIV", "MOD", "GT", "GE", "EQ", "LE", "LT", "NE", "TRUNC"];

type View = "prg" | "io" | "lib" | "task";

type Panel = "messages" | "watch" | "log" | "plant" | "stack" | "xref";

const WATCH_KEY = "fcu.plc.watch";
const KIND_ICONS: Record<ObjectKind, string> = { PRG: "📄", FB: "🧩", FUN: "ƒ", DUT: "🧱", GVL: "🌐" };
const NAME_PREFIX: Record<ObjectKind, string> = { PRG: "PRG_", FB: "FB_", FUN: "FUN_", DUT: "ST_", GVL: "GVL_" };

/** The "add object" form. */
interface AddForm {
	kind: ObjectKind;
	name: string;
	returnType: string;
	lang: Lang;
}

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
	/** The object in the editor (its id). */
	@state() private selectedId = "";
	/** Breakpoints per object (id): lines of its implementation, sent to the device as compiled lines of its file. */
	@state() private breakpoints = new Map<string, Set<number>>();
	/** Online values of a function block: the instance shown, per FB (id). */
	@state() private instances = new Map<string, string>();
	@state() private adding: AddForm | null = null;
	/** Folders of the tree that are collapsed. */
	@state() private collapsed = new Set<string>();
	/** The folder selected in the tree (null: an object or a page is; ""; the root). New objects and
	 *  folders go there. */
	@state() private selectedFolder: string | null = null;
	/** Drag and drop in the tree: what is dragged, and the folder under the pointer. */
	private dragging: { object?: string; folder?: string } | null = null;
	@state() private dropTarget: string | null = null;
	/** The name the cross references are shown for. */
	@state() private xref = "";
	@state() private view: View = "prg";
	@state() private declMode: "table" | "text" = "table";
	@state() private panel: Panel = "messages";
	@state() private watch: string[] = [];
	@state() private prepared = new Map<string, string>();
	@state() private tick = 0;
	@state() private notice: string | null = null;
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
		this.selectedId = this.firstProgram().id;
		this.lastBuild = build(this.project);
		this.fcu.attach(this.lastBuild.result);
		// a link to a view: /plc?view=fbd
		const view = new URLSearchParams(window.location.search).get("view");
		if (view === "prg" || view === "io" || view === "lib" || view === "task") this.view = view;
		// /plc?view=fbd|sfc: the first object in that language
		const lang = view?.toUpperCase();
		const first = this.project.objects.find((o) => o.lang === lang);
		if (first) this.selectedId = first.id;
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
		if (h && h !== this.lastHalt) {
			// the halted statement's object comes into the editor
			const o = this.objectOfFile(h.frames[0]?.file);
			if (o && hasBody(o.kind)) {
				this.selectedId = o.id;
				this.view = "prg";
			}
		}
		this.lastHalt = h;
		this.publishStatus();
	}

	/** The state of the PLC and of the project in the application's status bar (shell/status-bar.ts). */
	private publishStatus(): void {
		const dev = this.dev;
		const s = dev.state;
		const kind: StatusKind = !dev.connected ? "error" : s === "RUN" ? "ok" : s === "ERROR" ? "error" : s === "HALTED" ? "warning" : "idle";
		const located = this.lastBuild?.located ?? [];
		const errors = located.filter((l) => l.diagnostic.severity === "error").length + this.problemCount(this.lastBuild);
		const foreign = dev.kind === "fcu" && dev.connected && dev.projectHash !== 0 && !dev.debug;
		setAppStatus(this, {
			text: dev.connected ? `${dev.label}: ${s}` : `${dev.label} antwortet nicht`,
			kind,
			extra: html`
				<span class="sb-chip">${this.online ? "online: Werte" : "keine Werte"}</span>
				<span class="sb-chip">Ausgänge: <b>${dev.iec ? "IEC 61131" : "Feldbus"}</b></span>
				<span class="sb-chip">Zyklus ${dev.cycleMs} ms · ${dev.cycles}</span>
				<span class="sb-chip" title="ausgeführte Befehle der VM pro Zyklus (Budget 2 000 000)">Befehle ${dev.lastInstructions} (max ${dev.maxInstructions})</span>
				${errors ? html`<span class="sb-warn" title="siehe Meldungen">✖ ${errors} Fehler im Projekt</span>` : html`<span class="sb-chip">Projekt übersetzt</span>`}
				${this.stale ? html`<span class="sb-warn">⚠ Projekt ≠ Gerät</span>` : nothing}
				${dev.forcedOnDevice ? html`<span class="sb-warn" title="geforcte Werte auf dem Gerät (aller Browser); STOP, Umschalten oder 'plc unforce' im Terminal heben sie auf">${dev.forcedOnDevice} geforct</span>` : nothing}
				${dev.leasedByOther ? html`<span class="sb-warn" title="Ein anderer Browser forct Werte (Debug-Lease): hier nur beobachten; Aufheben dort oder mit 'plc unforce' im Terminal">🔒 anderer Browser debuggt</span>` : nothing}
				${foreign ? html`<span class="sb-right">Programm auf dem FCU stammt nicht aus diesem Browser: keine Online-Werte</span>` : nothing}`,
		});
	}

	private flash(text: string): void {
		this.notice = text;
		setTimeout(() => {
			if (this.notice === text) this.notice = null;
		}, 2500);
	}

	// ---- project ------------------------------------------------------------------------------------
	private firstProgram(): PlcObject {
		return this.project.objects.find((o) => o.kind === "PRG") ?? this.project.objects[0];
	}

	/** The object in the editor. */
	private get obj(): PlcObject {
		return this.project.objects.find((o) => o.id === this.selectedId) ?? this.firstProgram();
	}

	private objectOfFile(file: string | null | undefined): PlcObject | undefined {
		return file ? this.project.objects.find((o) => o.name === file) : undefined;
	}

	private setProject(p: Project): void {
		this.project = p;
		saveProject(p);
		if (!p.objects.some((o) => o.id === this.selectedId)) this.selectedId = this.firstProgram().id;
	}

	private edit(change: Partial<{ decl: string; body: string }>): void {
		const o = this.obj;
		let next: PlcObject = { ...o, ...change };
		let tasks = this.project.tasks;
		// a new name in the header renames the object (references stay: "Umbenennen" changes them too)
		const h = change.decl !== undefined ? headerName(next) : null;
		if (h && h !== o.name && nameProblem(this.project, h, o.id) === null) {
			const up = o.name.toUpperCase();
			tasks = tasks.map((t) => ({ ...t, programs: t.programs.map((n) => (n.toUpperCase() === up ? h : n)) }));
			this.dev.setBreakpoints(o.name, []);
			next = { ...next, name: h };
		}
		this.setProject({ ...this.project, objects: this.project.objects.map((x) => (x.id === o.id ? next : x)), tasks });
		// diagnostics while typing (debounced)
		if (this.buildTimer) clearTimeout(this.buildTimer);
		this.buildTimer = setTimeout(() => this.doBuild(false), 300);
	}

	/** The debug information of the last build that compiled (code completion of members while typing). */
	private lastGoodDebug: DebugInfo | null = null;

	private doBuild(explicit: boolean): Build {
		const b = build(this.project);
		this.lastBuild = b;
		if (b.result.debug) this.lastGoodDebug = b.result.debug;
		this.fcu.attach(b.result);
		if (explicit) {
			const errors = b.located.filter((l) => l.diagnostic.severity === "error").length + this.problemCount(b);
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

	// ---- objects: add, rename, remove -------------------------------------------------------------------
	private openAdd(): void {
		const kind: ObjectKind = "FB";
		this.adding = { kind, name: this.freeObjectName(kind), returnType: "INT", lang: "ST" };
	}

	private freeObjectName(kind: ObjectKind): string {
		for (let i = 1; ; i++) {
			const n = `${NAME_PREFIX[kind]}${i}`;
			if (nameProblem(this.project, n) === null) return n;
		}
	}

	private confirmAdd(): void {
		const a = this.adding;
		if (!a) return;
		const problem = nameProblem(this.project, a.name.trim());
		if (problem) {
			this.flash(`'${a.name}': ${problem}`);
			return;
		}
		const { project, object } = addObject(this.project, a.kind, a.name.trim(), a.returnType.trim() || "INT", a.lang, this.targetFolder());
		this.setProject(project);
		this.selectedId = object.id;
		this.view = "prg";
		this.adding = null;
		this.doBuild(false);
	}

	private async renameSelected(): Promise<void> {
		const o = this.obj;
		const to = await promptDialog({
			title: `${KIND_NAMES[o.kind]} umbenennen`, message: "Alle Verweise im Projekt werden angepasst.", label: "Neuer Name", value: o.name,
			confirm: "Umbenennen", validate: (v) => (v === o.name ? null : nameProblem(this.project, v, o.id)),
		});
		if (!to || to === o.name) return;
		this.dev.setBreakpoints(o.name, []);
		this.setProject(renameObject(this.project, o.id, to));
		this.doBuild(false);
		this.sendBreakpoints();
		this.flash(`${o.name} heißt jetzt ${to}`);
	}

	private async removeSelected(): Promise<void> {
		const o = this.obj;
		if (this.project.objects.length === 1) {
			await messageDialog({ title: "Löschen nicht möglich", message: "Das letzte Objekt des Projekts bleibt." });
			return;
		}
		if (!(await confirmDialog({ title: `${KIND_NAMES[o.kind]} löschen`, message: `${o.name} aus dem Projekt löschen?`, confirm: "Löschen", danger: true }))) return;
		this.dev.setBreakpoints(o.name, []);
		const bps = new Map(this.breakpoints);
		bps.delete(o.id);
		this.breakpoints = bps;
		this.setProject(removeObject(this.project, o.id));
		this.doBuild(false);
	}

	private async download(start: boolean): Promise<void> {
		const b = this.doBuild(true);
		if (!b.ok) return;
		const dev = this.dev;
		if (dev.kind === "fcu") {
			if (!dev.connected) {
				this.flash("Das FCU antwortet nicht");
				return;
			}
			if (start && dev.iec && !(await confirmDialog({
				title: "Laden und starten",
				message: ["Das Programm übernimmt die Ausgänge des FCU (Steuerquelle IEC 61131).", "Ventile, Motoren und Achsen können sich bewegen."],
				confirm: "Laden und starten", danger: true,
			}))) return;
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
		const o = this.obj;
		if (!this.dev.canDebug || !b?.result.debug || !hasBody(o.kind) || o.lang !== "ST") return null;
		const d = declLines(o);
		const out = new Set<number>();
		for (const s of b.result.debug.statements) {
			if (s.span.start.file === o.name && s.span.start.line > d) out.add(s.span.start.line - d);
		}
		return out;
	}

	/** The body line the program halted on (0: not halted there). */
	private get haltedBodyLine(): number {
		const top = this.dev.state === "HALTED" ? this.dev.halt?.frames[0] : undefined;
		const o = this.obj;
		if (!top || top.file !== o.name) return 0;
		return top.line - declLines(o);
	}

	/** The breakpoints of the object in the editor. */
	private get breakpointLines(): Set<number> {
		return this.breakpoints.get(this.obj.id) ?? new Set();
	}

	private get breakpointCount(): number {
		let n = 0;
		for (const s of this.breakpoints.values()) n += s.size;
		return n;
	}

	private async toggleBreakpoint(line: number): Promise<void> {
		const next = new Set(this.breakpointLines);
		const adding = !next.delete(line);
		if (adding && this.breakpointCount === 0 && this.dev.kind === "fcu" && this.dev.iec && !(await confirmDialog({
			title: "Haltepunkt auf dem FCU",
			message: ["Die Steuerquelle ist IEC 61131: hält das Programm am Haltepunkt an, behalten die Ausgänge ihren Wert (Ventile, Motoren!).",
				"Der E/A-Zyklus läuft weiter."],
			confirm: "Haltepunkt setzen",
		}))) return;
		if (adding) next.add(line);
		this.breakpoints = new Map(this.breakpoints).set(this.obj.id, next);
		this.sendBreakpoints();
	}

	private clearBreakpoints(): void {
		this.breakpoints = new Map();
		this.sendBreakpoints();
	}

	private sendBreakpoints(): void {
		if (!this.lastBuild || !this.dev.canDebug) return;
		const next = new Map<string, Set<number>>();
		for (const o of this.project.objects) {
			if (!hasBody(o.kind) || o.lang !== "ST") continue;
			const offset = declLines(o);
			const set = this.dev.setBreakpoints(o.name, [...(this.breakpoints.get(o.id) ?? [])].map((l) => l + offset));
			if (set.length) next.set(o.id, new Set(set.map((l) => l - offset)));  // lines without a statement drop out
		}
		this.breakpoints = next;
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
				this.setProject(p);
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
		if (!(await confirmDialog({ title: "Projekt vom Gerät laden", message: "Das aktuelle Projekt im Browser wird ersetzt.", confirm: "Laden" }))) return;
		const text = await dev.loadProject();
		if (text === null) {
			this.panel = "log";
			return;
		}
		try {
			const p = parseProject(text);
			this.setProject(p);
			this.doBuild(true);
			const same = dev.projectHash !== 0 && dev.projectHash === projectHash(p);
			this.flash(same ? "Projekt vom Gerät geladen - es passt zum Programm auf dem Gerät"
				: "Projekt vom Gerät geladen - ACHTUNG: es ist nicht der Stand des Programms auf dem Gerät");
		} catch (e) {
			this.flash(`Projekt vom Gerät unbrauchbar: ${(e as Error).message}`);
		}
	}

	private async resetToExample(): Promise<void> {
		if (!(await confirmDialog({ title: "Beispielprojekt laden", message: "Das aktuelle Projekt im Browser wird ersetzt.", confirm: "Beispiel laden", danger: true }))) return;
		this.setProject(exampleProject());
		this.doBuild(true);
	}

	// ---- online values ----------------------------------------------------------------------------
	private get online(): boolean {
		return this.dev.state !== "NO_PROGRAM" && this.dev.debug !== null;
	}

	/** Where the variables of the object in the editor live: a program its instance, a function block the
	 *  chosen instance; null: no instance (functions, types, global lists: globals only). */
	private get scope(): string | null {
		const o = this.obj;
		if (o.kind === "PRG") return o.name;
		if (o.kind !== "FB") return null;
		const all = this.instancesOf(o);
		const chosen = this.instances.get(o.id);
		return chosen && all.includes(chosen) ? chosen : (all[0] ?? null);
	}

	/** The instance paths of a function block in the program on the device (programs, their FBs, nested). */
	private instancesOf(fb: PlcObject, d = this.dev.debug): string[] {
		if (!d) return [];
		const key = fb.name.toUpperCase();
		const out: string[] = [];
		const walk = (prefix: string, t: Type, depth: number) => {
			if (t.k !== "fb" || depth > 4) return;
			for (const f of t.layout.fields) {
				if (f.type.k !== "fb") continue;
				const path = `${prefix}.${f.name}`;
				if (f.type.layout.key === key) out.push(path);
				walk(path, f.type, depth + 1);
			}
		};
		for (const [k, s] of d.symbols) {
			if (s.type.k !== "fb") continue;
			const name = this.project.objects.find((o) => o.name.toUpperCase() === k)?.name ?? k;
			if (s.type.layout.key === key) out.push(name);
			walk(name, s.type, 0);
		}
		return out;
	}

	private resolve(name: string): Symbol | null {
		const d = this.dev.debug;
		if (!d) return null;
		const scope = this.scope;
		return (scope ? d.resolve(`${scope}.${name}`) : null) ?? d.resolve(name);
	}

	private varValue(name: string): { text: string; type: Type } | null {
		const s = this.resolve(name);
		if (!s || elemOf(s.type) === null) return null;
		return { text: formatValue(s.type, this.dev.value(s)), type: s.type };
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
		const o = this.obj;
		const ms: LineMarker[] = (this.lastBuild?.located ?? []).filter((l) => l.object === o.id && l.part === part).map((l) => ({
			line: l.line, severity: l.diagnostic.severity, message: l.diagnostic.message,
		}));
		const err = this.dev.error;
		if (err?.line && err.file === o.name) {
			const at = partLine(o, err.line);
			if (at.part === part) ms.push({ line: at.line, severity: "runtime", message: `Laufzeitfehler ${err.trap}` });
		}
		return ms;
	}

	private reveal(l: Pick<Located, "object" | "part" | "line" | "col">): void {
		if (l.part === "decl" || l.part === "body") {
			if (l.object) this.selectedId = l.object;
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
		return `${this.scope ?? this.firstProgram().name}.${expr}`;
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
			if (s.type.k === "fb") walk(this.project.objects.find((o) => o.name.toUpperCase() === k)?.name ?? k, s.type, 0);
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
						${this.view === "prg" ? this.renderPrg() : this.view === "io" ? this.renderIo() : this.view === "task" ? this.renderTask() : this.renderLib()}
					</section>
				</div>
				${this.renderPanel()}
				${this.notice ? html`<div class="ide-toast">${this.notice}</div>` : nothing}
			</div>`;
	}

	private renderToolbar() {
		const s = this.dev.state;
		const ok = this.lastBuild?.ok ?? false;
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
				<div class="ide-tb-group" title="Debuggen: Haltepunkt per Klick auf die Zeilennummer">
					<button title="Weiter (F5): bis zum nächsten Haltepunkt" ?disabled=${s !== "HALTED"} @click=${() => this.dev.continueRun()}>▶ Weiter</button>
					<button title="Prozedurschritt (F10): die nächste Anweisung, Aufrufe laufen durch" ?disabled=${s !== "HALTED"} @click=${() => this.dev.step("over")}>⤼</button>
					<button title="Einzelschritt (F11): in den Aufruf hinein" ?disabled=${s !== "HALTED"} @click=${() => this.dev.step("into")}>⤵</button>
					<button title="Ausführen bis Rücksprung (Umschalt+F11)" ?disabled=${s !== "HALTED"} @click=${() => this.dev.step("out")}>⤴</button>
					<button title="Alle Haltepunkte entfernen" ?disabled=${this.breakpointCount === 0} @click=${() => this.clearBreakpoints()}>○ Haltepunkte (${this.breakpointCount})</button>
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

	// ---- graphical implementations (FBD, SFC) ---------------------------------------------------------
	private problemCount(b: Build | null): number {
		let n = 0;
		for (const g of b?.generated.values() ?? []) n += g.problems.length;
		return n;
	}

	/** The value of a variable path or expression for the diagram: in the object's scope first, then global. */
	private gfxValue(expr: string): string | null {
		const e = expr.trim();
		if (!/^[A-Za-z_][A-Za-z0-9_.]*$/.test(e)) return null;
		const scope = this.scope;
		for (const path of scope ? [`${scope}.${e}`, e] : [e]) {
			const r = this.dev.read(path);
			if (r.symbol && r.value !== null) return formatValue(r.symbol.type, r.value);
		}
		return null;
	}

	/** The elementary type of a variable of the object in the editor (pin colours): from the last build's debug
	 *  information (the program's instance, a function block's first instance), else from the I/O list. */
	private gfxVarType(name: string): string | null {
		const n = name.trim();
		if (!/^[A-Za-z_][A-Za-z0-9_.]*$/.test(n)) return null;
		const o = this.obj;
		// a simple name: its declaration says the type - in the object, then in the global lists. This also
		// works while the project does not compile (right after inserting a node it never does: an open
		// pin is an error), so pins keep their colours.
		if (!n.includes(".")) {
			const up = n.toUpperCase();
			const rows = [...parseRows(o.decl), ...this.project.objects.filter((x) => x.kind === "GVL").flatMap((x) => parseRows(x.decl))];
			const type = rows.find((r) => r.name.toUpperCase() === up)?.type.trim().toUpperCase();
			if (type && type in ELEM) return type;
		}
		// members (Timer.Q) and user types: from the last build that compiled
		const debug = this.lastBuild?.result.debug ?? this.lastGoodDebug;
		if (debug) {
			const scope = o.kind === "PRG" ? o.name : o.kind === "FB" ? this.instancesOf(o, debug)[0] : undefined;
			for (const path of scope ? [`${scope}.${n}`, n] : [n]) {
				const s = debug.resolve(path);
				if (s) return elemOf(s.type);
			}
		}
		return IO_VARS.find((v) => v.name.toUpperCase() === n.toUpperCase())?.type ?? null;
	}

	/** A program alone in the simulator (it need not be in the task): compile with a task of its own, load, start. */
	private async runAlone(o: PlcObject): Promise<void> {
		const b = build(this.project, [{ name: "MainTask", intervalCycles: 1, programs: [o.name] }]);
		if (!b.ok) {
			this.flash(`${o.name} übersetzt nicht`);
			this.panel = "messages";
			return;
		}
		this.selectDevice("sim");
		if (await this.dev.download(b.result)) await this.dev.start();
		this.flash(`${o.name} läuft allein im Simulator`);
	}

	private editGraph(change: { networks: FbdPou["networks"] } | { chart: SfcPou["chart"] }): void {
		const o = this.obj;
		this.setProject({ ...this.project, objects: this.project.objects.map((x) => (x.id === o.id ? { ...x, ...change } : x)) });
		if (this.buildTimer) clearTimeout(this.buildTimer);
		this.buildTimer = setTimeout(() => this.doBuild(false), 300);
	}

	/** The SFC's active steps with their time (online). */
	private activeSteps(o: PlcObject): Map<string, string> | null {
		const scope = this.scope;
		if (!this.online || !scope || !o.chart) return null;
		const active = new Map<string, string>();
		for (const { item } of sfcWalk(o.chart)) {
			if (item.kind !== "step") continue;
			if (this.dev.read(`${scope}.${item.name}.X`).value === true) {
				const t = this.dev.read(`${scope}.${item.name}.T`);
				active.set(item.name.toUpperCase(), t.symbol && t.value !== null ? formatValue(t.symbol.type, t.value) : "");
			}
		}
		return active;
	}

	private renderGraphical(o: PlcObject, g: Generated | undefined) {
		const online = this.online && this.scope !== null;
		const decl = declLines(o);
		const errors = (this.lastBuild?.located ?? []).filter((l) => l.object === o.id && l.part === "body");
		const inTask = this.project.tasks.some((t) => t.programs.some((n) => n.toUpperCase() === o.name.toUpperCase()));
		const top = this.dev.state === "HALTED" ? this.dev.halt?.frames[0] : undefined;
		const pou = { name: o.name, decl: o.decl };
		return html`
			${o.lang === "FBD" ? html`
				<plc-fbd-editor .pou=${{ ...pou, networks: o.networks ?? [], extra: boxesFor(this.project, o) }} .problems=${g?.problems ?? []}
					.lookup=${online ? (e: string) => this.gfxValue(e) : null}
					.varType=${(name: string) => this.gfxVarType(name)}
					@change=${(e: CustomEvent<FbdPou>) => this.editGraph({ networks: e.detail.networks })}></plc-fbd-editor>` : html`
				<plc-sfc-editor .pou=${{ ...pou, chart: o.chart ?? [] }} .problems=${g?.problems ?? []} .active=${this.activeSteps(o)}
					@change=${(e: CustomEvent<SfcPou>) => this.editGraph({ chart: e.detail.chart })}></plc-sfc-editor>`}
			<div class="gfx-actions">
				${o.kind === "PRG" && !inTask ? html`<button @click=${() => this.runAlone(o)}
					title="Nur dieses Programm übersetzen, in den Simulator laden und starten (es steht nicht im MainTask)">⇩▶ Allein im Simulator ausführen</button>` : nothing}
				${(g?.problems.length ?? 0) + errors.length
					? html`<span class="gfx-status-bad">${(g?.problems.length ?? 0) + errors.length} Fehler - siehe Meldungen und das generierte ST</span>`
					: html`<span class="gfx-status-ok">übersetzt zu ST</span>`}
			</div>
			<details class="gfx-st" ?open=${errors.length > 0 || top?.file === o.name}>
				<summary>Generiertes ST (${o.name}) - nur lesbar, entsteht beim Übersetzen</summary>
				<plc-st-editor readonly .value=${g?.text ?? ""} .minLines=${6}
					.current=${top?.file === o.name ? top.line : 0}
					.markers=${errors.map((l) => ({ line: l.line + decl, severity: l.diagnostic.severity, message: l.diagnostic.message }))}></plc-st-editor>
			</details>`;
	}

	// ---- task configuration ----------------------------------------------------------------------------
	private setTaskPrograms(programs: string[]): void {
		this.setProject({ ...this.project, tasks: this.project.tasks.map((t, i) => (i === 0 ? { ...t, programs } : t)) });
		this.doBuild(false);
	}

	private renderTask() {
		const task = this.project.tasks[0];
		const called = task.programs;
		const prgs = this.project.objects.filter((o) => o.kind === "PRG");
		const isIn = (o: PlcObject) => called.some((n) => n.toUpperCase() === o.name.toUpperCase());
		const move = (i: number, d: number) => {
			const next = [...called];
			[next[i], next[i + d]] = [next[i + d], next[i]];
			this.setTaskPrograms(next);
		};
		return html`
			<div class="ide-tabs"><span class="ide-tab ide-tab-active">⏱ ${task.name} <span class="ide-node-sub">Task-Konfiguration</span></span></div>
			<div class="ide-task">
				<p class="ide-hint">Ein zyklischer Task, Zykluszeit ${this.dev.cycleMs} ms (mehrere Tasks mit eigener Zykluszeit: P7). Er ruft die angehakten Programme in dieser Reihenfolge auf; ein Programm, das er nicht ruft, wird übersetzt und geprüft, läuft aber nicht.</p>
				<table class="ide-table">
					<thead><tr><th>Aufruf</th><th>Programm</th><th>Sprache</th><th>Reihenfolge</th></tr></thead>
					<tbody>
						${repeat(called, (n) => `in:${n}`, (n, i) => {
							const o = prgs.find((x) => x.name.toUpperCase() === n.toUpperCase());
							return html`<tr>
								<td><input type="checkbox" .checked=${true} @change=${() => this.setTaskPrograms(called.filter((x) => x !== n))} /></td>
								<td class="ide-mono">${i + 1}. ${n}</td>
								<td>${o ? LANG_NAMES[o.lang] : html`<span class="ide-bad">fehlt im Projekt</span>`}</td>
								<td>
									<button class="ide-icon-btn" title="früher aufrufen" ?disabled=${i === 0} @click=${() => move(i, -1)}>▲</button>
									<button class="ide-icon-btn" title="später aufrufen" ?disabled=${i === called.length - 1} @click=${() => move(i, 1)}>▼</button>
								</td>
							</tr>`;
						})}
						${repeat(prgs.filter((o) => !isIn(o)), (o) => `out:${o.name}`, (o) => html`<tr class="ide-task-off">
							<td><input type="checkbox" .checked=${false} @change=${() => this.setTaskPrograms([...called, o.name])} /></td>
							<td class="ide-mono">${o.name}</td>
							<td>${LANG_NAMES[o.lang]}</td>
							<td class="ide-node-sub">wird nicht aufgerufen</td>
						</tr>`)}
					</tbody>
				</table>
			</div>`;
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
				<div class="ide-tree-group ide-tree-root ${this.selectedFolder === "" ? "ide-node-active" : ""} ${this.dropTarget === "" ? "ide-drop" : ""}"
					title="Wurzel des Projekts: hierher ziehen, um etwas aus einem Ordner zu holen"
					@click=${() => (this.selectedFolder = "")}
					@dragover=${(e: DragEvent) => this.dragOver(e, "")} @dragleave=${() => (this.dropTarget = null)} @drop=${(e: DragEvent) => this.drop(e, "")}>
					<span>Applikation</span>
					<span class="ide-tree-tools">
						<button class="ide-icon-btn" title="Neues Objekt (im ausgewählten Ordner)" @click=${(e: Event) => { e.stopPropagation(); this.openAdd(); }}>＋📄</button>
						<button class="ide-icon-btn" title="Neuer Ordner (im ausgewählten Ordner)" @click=${(e: Event) => { e.stopPropagation(); void this.newFolder(); }}>＋📁</button>
					</span>
				</div>
					${this.renderFolder(objectTree(this.project), 0)}
				${node("task", "⏱", this.project.tasks[0]?.name ?? "MainTask", `${this.dev.cycleMs} ms · ${this.project.tasks[0]?.programs.length ?? 0} PRG`)}
					${this.adding ? this.renderAdd(this.adding) : html`<button class="ide-node ide-add" @click=${() => this.openAdd()}>＋ Objekt hinzufügen…</button>`}
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

	private renderAdd(a: AddForm) {
		const problem = nameProblem(this.project, a.name.trim());
		return html`
			<form class="ide-add-form" @submit=${(e: Event) => {
				e.preventDefault();
				this.confirmAdd();
			}}>
				<label>Art
					<select @change=${(e: Event) => {
						const kind = (e.target as HTMLSelectElement).value as ObjectKind;
						const auto = a.name === this.freeObjectName(a.kind);
						this.adding = { ...a, kind, name: auto ? this.freeObjectName(kind) : a.name, lang: LANGS_OF[kind].includes(a.lang) ? a.lang : "ST" };
					}}>
						${OBJECT_KINDS.map((k) => html`<option value=${k} ?selected=${k === a.kind}>${KIND_NAMES[k]} (${k})</option>`)}
					</select>
				</label>
				<label>Name
					<input .value=${a.name} spellcheck="false" class=${problem ? "ide-bad-input" : ""} title=${problem ?? ""}
						@input=${(e: Event) => (this.adding = { ...a, name: (e.target as HTMLInputElement).value })} />
				</label>
				${LANGS_OF[a.kind].length > 1 ? html`<label>Sprache
					<select @change=${(e: Event) => (this.adding = { ...a, lang: (e.target as HTMLSelectElement).value as Lang })}>
						${LANGS_OF[a.kind].map((l) => html`<option value=${l} ?selected=${l === a.lang}>${LANG_NAMES[l]} (${l})</option>`)}
					</select>
				</label>` : nothing}
				${a.kind === "FUN" ? html`<label>Rückgabetyp
					<input .value=${a.returnType} spellcheck="false" @input=${(e: Event) => (this.adding = { ...a, returnType: (e.target as HTMLInputElement).value })} />
				</label>` : nothing}
				${problem ? html`<span class="ide-bad">${problem}</span>` : nothing}
				<div class="ide-add-actions">
					<button type="submit" class="ide-primary" ?disabled=${problem !== null}>Hinzufügen</button>
					<button type="button" @click=${() => (this.adding = null)}>Abbrechen</button>
				</div>
			</form>`;
	}

	private renderPrg() {
		const o = this.obj;
		const body = hasBody(o.kind);
		const instances = o.kind === "FB" && this.online ? this.instancesOf(o) : [];
		return html`
			<div class="ide-tabs">
				<span class="ide-tab ide-tab-active">${KIND_ICONS[o.kind]} ${o.name} <span class="ide-node-sub">${KIND_NAMES[o.kind]}</span>${this.stale ? html` <span class="ide-modified" title="geändert seit dem Laden">●</span>` : nothing}</span>
				<span class="ide-tab-actions">
					<button class="ide-icon-btn" title="Umbenennen (alle Verweise im Projekt werden angepasst)" @click=${() => this.renameSelected()}>✎ Umbenennen</button>
					<button class="ide-icon-btn" title="Wo wird dieses Objekt benutzt?" @click=${() => this.showXref(o.name)}>🔍 Querverweise</button>
					<button class="ide-icon-btn" title="In einen anderen Ordner verschieben (oder im Baum dorthin ziehen)" @click=${() => this.moveSelected()}>📁 Verschieben</button>
					<button class="ide-icon-btn" title="Aus dem Projekt löschen" @click=${() => this.removeSelected()}>🗑 Löschen</button>
				</span>
			</div>
			${body ? nothing : html`
			<div class="ide-pou ide-pou-decl-only">
				<div class="ide-decl">
					<div class="ide-part-head">
						<span>${o.kind === "DUT" ? "Datentyp (TYPE … END_TYPE)" : "Globale Variablen (VAR_GLOBAL … END_VAR)"}</span>
						${o.kind === "GVL" ? html`<div class="ide-seg" role="group" aria-label="Ansicht der Deklaration">
							<button class=${this.declMode === "table" ? "on" : ""} @click=${() => (this.declMode = "table")}>Tabelle</button>
							<button class=${this.declMode === "text" ? "on" : ""} @click=${() => (this.declMode = "text")}>Text</button>
						</div>` : nothing}
					</div>
					${o.kind === "GVL" && this.declMode === "table" ? this.renderDeclTable() : html`
					<plc-st-editor id="decl-editor" .value=${o.decl} .completions=${this.completionsFor} .markers=${this.markers("decl")} .minLines=${12}
						@change=${(e: CustomEvent<string>) => this.edit({ decl: e.detail })}></plc-st-editor>`}
				</div>
			</div>`}
			${!body ? nothing : html`
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
						<plc-st-editor id="decl-editor" .value=${o.decl} .completions=${this.completionsFor} .markers=${this.markers("decl")} .minLines=${4}
							@change=${(e: CustomEvent<string>) => this.edit({ decl: e.detail })}></plc-st-editor>`}
				</div>
				<div class="ide-impl">
					<div class="ide-part-head">
						<span>Implementierung (${o.lang})</span>
						${instances.length ? html`<label class="ide-instance" title="Die Werte dieser Instanz anzeigen">Instanz
							<select @change=${(e: Event) => (this.instances = new Map(this.instances).set(o.id, (e.target as HTMLSelectElement).value))}>
								${instances.map((i) => html`<option ?selected=${i === this.scope}>${i}</option>`)}
							</select></label>` : nothing}
						${o.kind === "FB" && this.online && !instances.length ? html`<span class="ide-node-sub">keine Instanz im geladenen Programm</span>` : nothing}
						${o.kind === "FUN" && this.online ? html`<span class="ide-node-sub">Funktionen: keine Online-Werte (lokale Variablen nur während des Aufrufs)</span>` : nothing}
						${this.online && this.scope ? html`<span class="ide-online ${this.stale ? "ide-online-stale" : ""}">${this.stale ? "Werte des geladenen Programms - Projekt geändert" : "online: Werte"}</span>` : nothing}
					</div>
					${o.lang !== "ST" ? this.renderGraphical(o, this.lastBuild?.generated.get(o.id)) : html`
					<plc-st-editor id="body-editor" .value=${o.body} .completions=${this.completionsFor} .markers=${this.markers("body")} .inline=${this.inlineValues(o.body)} .stale=${this.stale} .minLines=${12}
						.breakpoints=${this.breakpointLines} .breakable=${this.breakableBody} .current=${this.haltedBodyLine}
						@toggle-breakpoint=${(e: CustomEvent<number>) => this.toggleBreakpoint(e.detail)}
						@change=${(e: CustomEvent<string>) => this.edit({ body: e.detail })}></plc-st-editor>`}
				</div>
			</div>`}`;
	}

	private renderDeclTable() {
		const decl = this.obj.decl;
		// a GVL has global variables only; the POUs the classes of their declaration part
		const classes: readonly string[] = this.obj.kind === "GVL" ? GVL_CLASSES : VAR_CLASSES;
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
							const v = this.online && (this.scope || this.obj.kind === "GVL") ? this.varValue(r.name) : null;
							return html`
							<tr class=${err ? "ide-row-error" : ""} title=${err ?? nothing}>
								<td>
									<select @change=${(e: Event) => set(r, { cls: (e.target as HTMLSelectElement).value })}>
										${classes.map((c) => html`<option ?selected=${c === r.cls}>${c}</option>`)}
									</select>${r.qualifier ? html`<span class="ide-qual">${r.qualifier}</span>` : nothing}
								</td>
								<td>${cell(r, "name")}</td>
								<td>${cell(r, "type")}</td>
								<td>${cell(r, "init")}</td>
								<td>${cell(r, "comment", true)}</td>
								${this.online ? html`<td class="ide-value">${v?.text ?? ""}</td>` : nothing}
								<td class="ide-row-actions"><button class="ide-icon-btn" title="Querverweise: wo wird ${r.name} benutzt?" @click=${() => this.showXref(r.name)}>🔍</button><button class="ide-icon-btn" title="Variable löschen" @click=${() => this.edit({ decl: deleteRow(decl, r) })}>✕</button></td>
							</tr>`;
						})}
					</tbody>
				</table>
				<button class="ide-add-row" @click=${() => this.edit({ decl: addRow(decl, { cls: classes[0], qualifier: "", name: freeName(rows), type: "BOOL", init: "", comment: "" }) })}>＋ Variable</button>
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
				<p class="ide-hint">Diese globalen Variablen sind in jedem Programm sichtbar. Jede ist an ein Register des FCU gebunden: Eingänge (%I) liest das FCU vor jedem Zyklus aus den Registern, Ausgänge (%Q) schreibt es danach hinein. Im Simulator liest und schreibt sie das Anlagenmodell.</p>
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
								<td>${s ? html`<input type="checkbox" .checked=${forced} ?disabled=${!this.dev.canForce} title="Wert festhalten (forcen)"
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
		return html`
			<p class="ide-stack-why">Angehalten: ${h.reason === "breakpoint" ? "Haltepunkt" : "Einzelschritt"} - das Programm wartet auf „Weiter“ oder einen Schritt (es läuft nie von selbst weiter).</p>
			<table class="ide-table">
				<thead><tr><th></th><th>POU</th><th>Zeile</th><th>Quelle</th></tr></thead>
				<tbody>${h.frames.map((f, i) => {
					const o = this.objectOfFile(f.file);
					const at = o ? partLine(o, f.line) : null;
					return html`
					<tr class="${i === 0 ? "ide-stack-top" : ""} ${o ? "ide-stack-link" : ""}" title=${o ? "Im Editor zeigen" : ""}
						@click=${() => o && at && this.reveal({ object: o.id, part: at.part, line: at.line, col: 1 })}>
						<td>${i === 0 ? "▶" : ""}</td>
						<td>${f.pou}</td>
						<td>${at ? at.line : f.line}</td>
						<td>${o ? `${o.name} · ${at!.part === "body" ? "Implementierung" : "Deklaration"}` : f.file === "<stdlib>" ? "Standardbibliothek" : f.file === "<fcu>" ? "FCU-Bibliothek" : f.file}</td>
					</tr>`;
				})}
				</tbody>
			</table>`;
	}

	private renderPanel() {
		const tab = (p: Panel, label: string, badge = "") => html`
			<button class="ide-ptab ${this.panel === p ? "on" : ""}" @click=${() => (this.panel = p)}>${label}${badge ? html` <span class="ide-badge">${badge}</span>` : nothing}</button>`;
		const located = this.lastBuild?.located ?? [];
		const errors = located.filter((l) => l.diagnostic.severity === "error").length + (this.dev.error ? 1 : 0) + this.problemCount(this.lastBuild);
		return html`
			<section class="ide-panel">
				<div class="ide-ptabs">
					${tab("messages", "Meldungen", errors ? String(errors) : "")}
					${tab("watch", "Überwachung")}
					${tab("plant", "Anlage (Simulation)")}
					${tab("log", "Geräte-Log")}
					${tab("stack", "Aufrufliste", this.dev.state === "HALTED" ? "⏸" : "")}
					${tab("xref", "Querverweise")}
				</div>
				<div class="ide-pbody">
					${this.panel === "messages" ? this.renderMessages(located) : this.panel === "watch" ? this.renderWatch()
						: this.panel === "plant" ? this.renderPlant() : this.panel === "stack" ? this.renderStack() : this.panel === "xref" ? this.renderXref() : this.renderLog()}
				</div>
			</section>`;
	}

	// ---- code completion ------------------------------------------------------------------------------------
	/** The candidates at the caret of an ST editor of the object in the editor: after "x." the fields of x
	 *  (its type from the last build that compiled; a standard FB also from its declaration), else every
	 *  name: keywords, types, blocks, functions, the project's objects, the object's variables, the globals
	 *  and the I/O variables. The editor filters by the prefix. */
	private completionsFor = (ctx: CompletionContext): Completion[] => {
		const o = this.obj;
		if (ctx.object !== null) return this.memberCompletions(o, ctx.object);
		const out: Completion[] = [];
		for (const k of KEYWORDS) if (!(k in ELEM)) out.push({ label: k, detail: "Schlüsselwort", kind: "keyword" });
		for (const t of Object.keys(ELEM)) out.push({ label: t, detail: "Datentyp", kind: "type" });
		for (const b of this.std) out.push({ label: b.name, detail: "Standard-FB", kind: "fb" });
		for (const f of STD_FUNCTIONS) out.push({ label: f, detail: "Standardfunktion", kind: "function" });
		for (const b of boxesFor(this.project, o)) out.push({ label: b.type, detail: b.title, kind: b.kind === "fb" ? "fb" : "function" });
		for (const x of this.project.objects) {
			if (x.kind === "PRG" || x.kind === "DUT") out.push({ label: x.name, detail: KIND_NAMES[x.kind], kind: x.kind === "DUT" ? "type" : "pou" });
			if (x.kind === "GVL") for (const r of parseRows(x.decl)) out.push({ label: r.name, detail: `global (${x.name}) : ${r.type}`, kind: "variable" });
		}
		if (o.kind !== "GVL") for (const r of parseRows(o.decl)) out.push({ label: r.name, detail: `${r.cls} : ${r.type}`, kind: "variable" });
		for (const v of IO_VARS) out.push({ label: v.name, detail: `${v.type}${v.unit ? ` [${v.unit}]` : ""} - ${v.description}`, kind: "io" });
		const seen = new Set<string>();
		return out.filter((c) => !seen.has(c.label.toUpperCase()) && seen.add(c.label.toUpperCase()));
	};

	private memberCompletions(o: PlcObject, object: string): Completion[] {
		const d = this.lastGoodDebug;
		const scope = o.kind === "PRG" ? o.name : o.kind === "FB" && d ? this.instancesOf(o, d)[0] : undefined;
		const s = d ? (scope ? d.resolve(`${scope}.${object}`) : null) ?? d.resolve(object) : null;
		const fields = s?.type.k === "fb" ? s.type.layout.fields : s?.type.k === "struct" ? s.type.fields : null;
		if (fields) return fields.map((f) => ({ label: f.name, detail: typeName(f.type), kind: "field" as const }));
		// not compiled yet: a standard FB instance of the declaration
		const row = parseRows(o.decl).find((r) => r.name.toUpperCase() === object.toUpperCase());
		const std = row && this.std.find((b) => b.name === row.type.toUpperCase());
		if (!std) return [];
		return [...std.inputs, ...std.outputs].map((pin) => {
			const [name, type] = pin.split(" : ");
			return { label: name, detail: type ?? "", kind: "field" as const };
		});
	}

	// ---- folders and cross references --------------------------------------------------------------------
	// ---- the explorer tree: folders -----------------------------------------------------------------------
	/** Where a new object or folder goes: the selected folder, else the folder of the object in the editor. */
	private targetFolder(): string {
		return this.selectedFolder ?? (this.view === "prg" ? this.obj.folder ?? "" : "");
	}

	private toggleFolder(path: string): void {
		const c = new Set(this.collapsed);
		if (!c.delete(path)) c.add(path);
		this.collapsed = c;
	}

	private expand(path: string): void {
		const c = new Set(this.collapsed);
		for (let f = path; f; f = f.includes("/") ? f.slice(0, f.lastIndexOf("/")) : "") c.delete(f);
		this.collapsed = c;
	}

	private async newFolder(): Promise<void> {
		const parent = this.targetFolder();
		const name = await promptDialog({
			title: "Neuer Ordner", message: parent ? `im Ordner „${parent}“` : "in der Wurzel des Projekts", label: "Name", confirm: "Anlegen",
			validate: (v) => folderNameProblem(this.project, parent, v),
		});
		if (!name) return;
		this.setProject(addFolder(this.project, parent, name));
		this.expand(parent);
		this.selectedFolder = parent ? `${parent}/${name}` : name;
	}

	private async renameFolderUi(path: string): Promise<void> {
		const parent = path.includes("/") ? path.slice(0, path.lastIndexOf("/")) : "";
		const old = path.slice(path.lastIndexOf("/") + 1);
		const name = await promptDialog({
			title: "Ordner umbenennen", label: "Neuer Name", value: old, confirm: "Umbenennen",
			validate: (v) => (v === old ? null : folderNameProblem(this.project, parent, v, path)),
		});
		if (!name || name === old) return;
		this.setProject(renameFolder(this.project, path, name));
		this.selectedFolder = parent ? `${parent}/${name}` : name;
	}

	private async deleteFolderUi(path: string): Promise<void> {
		const inside = objectsIn(this.project, path);
		let contents: "delete" | "toRoot" = "toRoot";
		if (inside.length > 0) {
			// not empty: what happens to the objects?
			const answer = await choiceDialog({
				title: "Ordner löschen",
				message: [`Der Ordner „${path}“ enthält ${inside.length} Objekt(e): ${inside.map((o) => o.name).join(", ")}.`, "Was soll mit ihnen geschehen?"],
				choices: [{ id: "toRoot", label: "In die Wurzel verschieben", primary: true }, { id: "delete", label: "Mit löschen", danger: true }],
			});
			if (answer === null) return;
			contents = answer as "delete" | "toRoot";
			if (contents === "delete" && inside.length === this.project.objects.length) {
				await messageDialog({ title: "Löschen nicht möglich", message: "Der Ordner enthält alle Objekte des Projekts; das letzte Objekt bleibt." });
				return;
			}
		}
		if (contents === "delete") {
			const bps = new Map(this.breakpoints);
			for (const o of inside) {
				this.dev.setBreakpoints(o.name, []);
				bps.delete(o.id);
			}
			this.breakpoints = bps;
		}
		this.setProject(deleteFolder(this.project, path, contents));
		this.selectedFolder = null;
		this.doBuild(false);
	}

	/** The object in the editor into another folder, chosen from a list (the tree: drag it there). */
	private async moveSelected(): Promise<void> {
		const o = this.obj;
		const here = o.folder ?? "";
		const folders = ["", ...allFolders(this.project)].filter((f) => f !== here);
		if (folders.length === 0) {
			await messageDialog({ title: "Verschieben", message: "Es gibt noch keinen Ordner. Ordner anlegen: ＋📁 über dem Projektbaum." });
			return;
		}
		const to = await choiceDialog({
			title: `${o.name} verschieben`, message: `Jetzt in: ${here || "Wurzel des Projekts"}`,
			choices: folders.map((f) => ({ id: `f:${f}`, label: f || "Wurzel des Projekts" })),
		});
		if (to === null) return;
		this.setProject(moveObject(this.project, o.id, to.slice(2)));
		this.expand(to.slice(2));
	}

	private dragStart(e: DragEvent, what: { object?: string; folder?: string }): void {
		this.dragging = what;
		e.dataTransfer?.setData("text/plain", what.object ?? what.folder ?? "");
		if (e.dataTransfer) e.dataTransfer.effectAllowed = "move";
		e.stopPropagation();
	}

	/** Why the dragged thing cannot be dropped into `folder` (null: it can). */
	private dropProblem(folder: string): string | null {
		const d = this.dragging;
		if (!d) return "nichts gezogen";
		if (d.object !== undefined) {
			const o = this.project.objects.find((x) => x.id === d.object);
			return o && (o.folder ?? "") !== folder ? null : "liegt schon dort";
		}
		return moveFolderProblem(this.project, d.folder!, folder);
	}

	private dragOver(e: DragEvent, folder: string): void {
		if (this.dropProblem(folder) !== null) return;  // not a target: the browser shows "no drop"
		e.preventDefault();
		e.stopPropagation();
		if (this.dropTarget !== folder) this.dropTarget = folder;
	}

	private drop(e: DragEvent, folder: string): void {
		e.preventDefault();
		e.stopPropagation();
		const d = this.dragging;
		const problem = this.dropProblem(folder);
		this.dragging = null;
		this.dropTarget = null;
		if (!d || problem !== null) return;
		if (d.object !== undefined) this.setProject(moveObject(this.project, d.object, folder));
		else {
			const name = d.folder!.slice(d.folder!.lastIndexOf("/") + 1);
			this.setProject(moveFolder(this.project, d.folder!, folder));
			this.selectedFolder = folder ? `${folder}/${name}` : name;
		}
		this.expand(folder);
	}

	/** One level of the explorer tree: the folders (each with its content below it), then the objects. */
	private renderFolder(node: FolderNode, depth: number): unknown {
		const indent = (extra = 0) => `padding-left:${6 + depth * 14 + extra}px`;
		return html`
			${node.folders.map((f) => {
				const open = !this.collapsed.has(f.path);
				return html`
					<div class="ide-node ide-folder ${this.selectedFolder === f.path ? "ide-node-active" : ""} ${this.dropTarget === f.path ? "ide-drop" : ""}"
						style=${indent()} role="treeitem" aria-expanded=${open} draggable="true" title=${f.path}
						@click=${() => (this.selectedFolder = f.path)} @dblclick=${() => this.toggleFolder(f.path)}
						@dragstart=${(e: DragEvent) => this.dragStart(e, { folder: f.path })} @dragend=${() => { this.dragging = null; this.dropTarget = null; }}
						@dragover=${(e: DragEvent) => this.dragOver(e, f.path)} @dragleave=${() => (this.dropTarget = null)} @drop=${(e: DragEvent) => this.drop(e, f.path)}>
						<button class="ide-chevron" title=${open ? "zuklappen" : "aufklappen"} @click=${(e: Event) => { e.stopPropagation(); this.toggleFolder(f.path); }}>${open ? "▾" : "▸"}</button>
						<span class="ide-node-icon">${open ? "📂" : "📁"}</span><span class="ide-node-name">${f.name}</span>
						<span class="ide-node-tools">
							<button class="ide-icon-btn" title="Ordner umbenennen" @click=${(e: Event) => { e.stopPropagation(); void this.renameFolderUi(f.path); }}>✎</button>
							<button class="ide-icon-btn" title="Ordner löschen" @click=${(e: Event) => { e.stopPropagation(); void this.deleteFolderUi(f.path); }}>🗑</button>
						</span>
					</div>
					${open ? this.renderFolder(f, depth + 1) : nothing}`;
			})}
			${node.objects.map((o) => html`
				<button class="ide-node ${this.selectedFolder === null && this.view === "prg" && o.id === this.obj.id ? "ide-node-active" : ""}"
					style=${indent(18)} role="treeitem" draggable="true" title="${KIND_NAMES[o.kind]} - ziehen, um ihn in einen Ordner zu legen"
					@dragstart=${(e: DragEvent) => this.dragStart(e, { object: o.id })} @dragend=${() => { this.dragging = null; this.dropTarget = null; }}
					@click=${() => {
						this.selectedId = o.id;
						this.selectedFolder = null;
						this.view = "prg";
					}}>
					<span class="ide-node-icon">${KIND_ICONS[o.kind]}</span><span class="ide-node-name">${o.name}</span><span class="ide-node-sub">${hasBody(o.kind) ? `${o.kind} · ${o.lang}` : o.kind}</span>
				</button>`)}`;
	}

	private showXref(name: string): void {
		this.xref = name;
		this.panel = "xref";
	}

	/** Names worth looking up: the objects, the declared variables of every object, the I/O variables. */
	private xrefNames(): string[] {
		const names = new Set<string>();
		for (const o of this.project.objects) {
			names.add(o.name);
			for (const r of parseRows(o.decl)) names.add(r.name);
		}
		for (const v of IO_VARS) names.add(v.name);
		return [...names];
	}

	private openXref(r: CrossRef): void {
		if (r.where === "decl" || r.where === "body") {
			this.reveal({ object: r.object, part: r.where, line: r.line, col: r.col });
			return;
		}
		this.selectedId = r.object;
		this.view = "prg";
	}

	private renderXref() {
		const refs = this.xref ? crossReferences(this.project, this.xref) : [];
		const where = (r: CrossRef) => (r.where === "decl" ? "Deklaration" : r.where === "body" ? "Implementierung" : r.where === "fbd" ? `FBD Netzwerk ${r.line}` : "SFC");
		return html`
			<div class="ide-xref">
				<div class="ide-watch-actions">
					<input list="ide-xref-names" placeholder="Name (Variable, POU, Typ, Instanz, Schritt)" .value=${this.xref}
						@change=${(e: Event) => (this.xref = (e.target as HTMLInputElement).value.trim())} />
					<datalist id="ide-xref-names">${this.xrefNames().map((n) => html`<option value=${n}></option>`)}</datalist>
					${this.xref ? html`<span class="ide-node-sub">${refs.length} Stelle(n) - nach Namen, nicht nach Gültigkeitsbereich: gleichnamige lokale Variablen anderer POUs erscheinen mit ihrem Objekt</span>` : nothing}
				</div>
				${this.xref && refs.length === 0 ? html`<p class="ide-empty">„${this.xref}“ kommt im Projekt nicht vor${IO_VARS.some((v) => v.name.toUpperCase() === this.xref.toUpperCase()) ? " (eine I/O-Variable, die kein Objekt benutzt)" : ""}.</p>` : nothing}
				${refs.length ? html`
				<table class="ide-table">
					<thead><tr><th>Objekt</th><th>Ort</th><th>Zeile</th><th></th><th>Text</th></tr></thead>
					<tbody>${refs.map((r) => {
						const o = this.project.objects.find((x) => x.id === r.object);
						return html`<tr class="ide-stack-link" title="Im Editor zeigen" @click=${() => this.openXref(r)}>
							<td>${o ? `${KIND_ICONS[o.kind]} ${o.name}` : ""}</td>
							<td>${where(r)}</td>
							<td>${r.where === "decl" || r.where === "body" ? `${r.line}:${r.col}` : ""}</td>
							<td>${r.declares ? html`<span class="ide-force-tag" title="hier deklariert">D</span>` : nothing}</td>
							<td class="ide-mono">${r.text}</td>
						</tr>`;
					})}</tbody>
				</table>` : nothing}
			</div>`;
	}

	private renderMessages(located: Located[]) {
		const where = (l: Located) => {
			const o = this.project.objects.find((x) => x.id === l.object);
			if (l.part === "body" && o && o.lang !== "ST") return `${o.name} · generiertes ST ${l.line + declLines(o)}:${l.col}`;
			return `${l.part === "decl" ? `${o?.name} · Deklaration` : l.part === "body" ? `${o?.name} · Implementierung` : l.part === "io" ? "I/O" : l.diagnostic.span.start.file} ${l.line}:${l.col}`;
		};
		const problems = [...(this.lastBuild?.generated ?? [])].flatMap(([id, g]) => g.problems.map((pr) => ({ o: this.project.objects.find((x) => x.id === id)!, pr })));
		const err = this.dev.error;
		const errObj = this.objectOfFile(err?.file);
		const errAt = errObj && err?.line ? partLine(errObj, err.line) : null;
		if (located.length === 0 && !err && problems.length === 0) return html`<div class="ide-empty">✔ Keine Meldungen. ${this.lastBuild?.result.ok ? `Image: ${this.lastBuild.result.image!.length} Byte, Daten: ${this.lastBuild.result.debug!.dataSize} Byte.` : ""}</div>`;
		return html`<ul class="ide-messages">
			${err ? html`<li class="ide-msg ide-msg-runtime" @click=${() => {
				if (errObj && errAt) this.reveal({ object: errObj.id, part: errAt.part, line: errAt.line, col: 1 });
			}}><span class="ide-msg-icon">⛔</span> Laufzeitfehler <b>${err.trap}</b> in ${err.pou}${errAt ? `, Zeile ${errAt.line} der ${errAt.part === "body" ? "Implementierung" : "Deklaration"}` : ""} - die SPS ist in ERROR. Reset setzt die Variablen zurück.</li>` : nothing}
			${located.map((l) => html`
				<li class="ide-msg ide-msg-${l.diagnostic.severity}" @click=${() => this.reveal(l)}>
					<span class="ide-msg-icon">${l.diagnostic.severity === "error" ? "✖" : "⚠"}</span>
					<span class="ide-msg-loc">${where(l)}</span>
					<span>${l.diagnostic.message}</span>
				</li>`)}
			${problems.map(({ o, pr }) => html`
				<li class="ide-msg ide-msg-error" @click=${() => {
					this.selectedId = o.id;
					this.view = "prg";
				}}>
					<span class="ide-msg-icon">✖</span>
					<span class="ide-msg-loc">${o.name} · ${o.lang}</span>
					<span>${pr.message}</span>
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
					<button ?disabled=${!online || this.prepared.size === 0 || !this.dev.canForce} @click=${() => this.applyPrepared(true)} title="Alle vorbereiteten Werte festhalten">Werte forcen</button>
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

}

declare global {
	interface HTMLElementTagNameMap {
		"plc-ide-app": PlcIdeApp;
	}
}
