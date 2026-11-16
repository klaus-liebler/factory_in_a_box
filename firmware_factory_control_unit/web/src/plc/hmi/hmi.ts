// The operating and monitoring interface (HMI, "Bedienen und Beobachten"; VDI 3814 group 3: operating,
// display and management functions): pages of elements bound to PLC variables. Part of the PLC project
// (`Project.hmi`), so it is stored with the project (browser, file, the FCU's SD card) and its variables are
// resolved with the program's debug information. The editor (hmi-editor.ts) designs pages, the runtime
// (hmi-runtime-app.ts and the editor's live view) shows them with the device's values. DOM-free.
import type { DebugInfo } from "../compiler/compiler.ts";
import { ELEM, elemOf, type Type } from "../compiler/types.ts";
import type { Value } from "../ide/values.ts";
import { COLORS } from "../../ui/colors.ts";

export type HmiKind = "button" | "switch" | "knob" | "slider" | "digital" | "gauge" | "bar" | "trend" | "led" | "label" | "alarm";

export const LED_COLORS = ["red", "green", "yellow", "blue", "white", "orange"] as const;
export type LedColor = (typeof LED_COLORS)[number];
export const LED_COLOR_NAMES: Record<LedColor, string> = { red: "Rot", green: "Grün", yellow: "Gelb", blue: "Blau", white: "Weiß", orange: "Orange" };

/** A trend pen (the line recorder's curves). */
export interface HmiPen {
	var: string;
	color: string;
	label?: string;
}

export interface HmiElement {
	id: string;
	kind: HmiKind;
	x: number;
	y: number;
	w: number;
	h: number;
	/** The caption shown with the element. */
	label: string;
	/** The bound variable ("PLC_PRG.Speed", a global, an I/O variable); empty: none. */
	var: string;
	/** knob, slider, gauge, bar, trend: the range; digital: unused. */
	min?: number;
	max?: number;
	/** knob, slider: the step of a change (0: continuous). */
	step?: number;
	unit?: string;
	decimals?: number;
	/** led, button, switch, digital, label: the colour. */
	color?: string;
	/** gauge, bar: limits shown as coloured zones (warning, alarm; above them). */
	warn?: number;
	alarm?: number;
	/** led: blinks while on; alarm: blinks while unacknowledged (always). */
	blink?: boolean;
	/** led, switch, button: the variable's FALSE is "on". */
	invert?: boolean;
	/** button, switch, label, alarm: the text. */
	text?: string;
	/** label: font size [px]. */
	fontSize?: number;
	/** trend: the pens and the time span [s]. */
	pens?: HmiPen[];
	span?: number;
	/** alarm: the message class (1 danger to life ... 8 other, as VDI3814.NC) and the acknowledge variable
	 *  (BOOL; the HMI writes TRUE, then FALSE - e.g. an NC's Ack). */
	prio?: number;
	ackVar?: string;
	/** knob, slider, switch, button: only shown, no operation. */
	readonly?: boolean;
}

export interface HmiPage {
	id: string;
	name: string;
	width: number;
	height: number;
	elements: HmiElement[];
}

/** A property of an element kind for the editor's property panel. */
export interface HmiProp {
	key: keyof HmiElement;
	label: string;
	type: "text" | "number" | "var" | "color" | "led" | "bool" | "prio";
	hint?: string;
}

export interface HmiKindInfo {
	kind: HmiKind;
	name: string;
	icon: string;
	group: "Bedienen" | "Anzeigen" | "Gestaltung";
	/** What the bound variable should be. */
	varHint: string;
	defaults: Omit<HmiElement, "id" | "kind" | "x" | "y">;
	props: HmiProp[];
}

const P = {
	label: { key: "label", label: "Beschriftung", type: "text" },
	var: { key: "var", label: "Variable", type: "var" },
	min: { key: "min", label: "Minimum", type: "number" },
	max: { key: "max", label: "Maximum", type: "number" },
	step: { key: "step", label: "Schrittweite", type: "number", hint: "0: stufenlos" },
	unit: { key: "unit", label: "Einheit", type: "text" },
	decimals: { key: "decimals", label: "Nachkommastellen", type: "number" },
	color: { key: "color", label: "Farbe", type: "color" },
	text: { key: "text", label: "Text", type: "text" },
	readonly: { key: "readonly", label: "nur anzeigen", type: "bool" },
	invert: { key: "invert", label: "invertiert (FALSE = an)", type: "bool" },
	warn: { key: "warn", label: "Warnung ab", type: "number", hint: "leer: keine" },
	alarm: { key: "alarm", label: "Alarm ab", type: "number", hint: "leer: keiner" },
} satisfies Record<string, HmiProp>;

export const HMI_KINDS: readonly HmiKindInfo[] = [
	{
		kind: "button", name: "Taster", icon: "🔘", group: "Bedienen", varHint: "BOOL: TRUE solange gedrückt",
		defaults: { w: 120, h: 56, label: "", var: "", text: "Taster", color: "#2f6fde" },
		props: [P.text, P.label, P.var, P.color, P.invert, P.readonly],
	},
	{
		kind: "switch", name: "Schalter", icon: "🎚", group: "Bedienen", varHint: "BOOL: jeder Klick schaltet um",
		defaults: { w: 130, h: 56, label: "Schalter", var: "", text: "", color: "#1f9d55" },
		props: [P.label, P.var, P.color, P.invert, P.readonly],
	},
	{
		kind: "knob", name: "Drehpotentiometer", icon: "🎛", group: "Bedienen", varHint: "Zahl (INT, REAL ...): Ziehen, Mausrad, Pfeiltasten",
		defaults: { w: 120, h: 140, label: "Sollwert", var: "", min: 0, max: 100, step: 1, unit: "", decimals: 0, color: "#2f6fde" },
		props: [P.label, P.var, P.min, P.max, P.step, P.unit, P.decimals, P.color, P.readonly],
	},
	{
		kind: "slider", name: "Schiebepotentiometer", icon: "🎚", group: "Bedienen", varHint: "Zahl: waagerecht oder senkrecht (nach der Form)",
		defaults: { w: 240, h: 70, label: "Stellwert", var: "", min: 0, max: 100, step: 1, unit: "%", decimals: 0, color: "#2f6fde" },
		props: [P.label, P.var, P.min, P.max, P.step, P.unit, P.decimals, P.color, P.readonly],
	},
	{
		kind: "digital", name: "Digitalanzeige", icon: "🔢", group: "Anzeigen", varHint: "jede Variable (Zahl, BOOL, TIME)",
		defaults: { w: 170, h: 80, label: "Istwert", var: "", unit: "", decimals: 1, color: "#39ff7a" },
		props: [P.label, P.var, P.unit, P.decimals, P.color],
	},
	{
		kind: "gauge", name: "Analoganzeige", icon: "⏲", group: "Anzeigen", varHint: "Zahl: Zeigerinstrument",
		defaults: { w: 180, h: 170, label: "Druck", var: "", min: 0, max: 100, unit: "", decimals: 1 },
		props: [P.label, P.var, P.min, P.max, P.unit, P.decimals, P.warn, P.alarm],
	},
	{
		kind: "bar", name: "Balkenanzeige", icon: "📶", group: "Anzeigen", varHint: "Zahl: Füllstand/Balken",
		defaults: { w: 70, h: 200, label: "Füllstand", var: "", min: 0, max: 100, unit: "%", decimals: 0, color: "#2f9bde" },
		props: [P.label, P.var, P.min, P.max, P.unit, P.decimals, P.color, P.warn, P.alarm],
	},
	{
		kind: "trend", name: "Linienschreiber", icon: "📈", group: "Anzeigen", varHint: "bis zu 4 Zahlen/BOOL als Kurven",
		defaults: { w: 420, h: 220, label: "Verlauf", var: "", min: 0, max: 100, span: 60, decimals: 1, unit: "", pens: [] },
		props: [P.label, P.min, P.max, { key: "span", label: "Zeitbereich [s]", type: "number" }, P.unit, P.decimals],
	},
	{
		kind: "led", name: "LED", icon: "💡", group: "Anzeigen", varHint: "BOOL: an/aus",
		defaults: { w: 60, h: 80, label: "", var: "", color: "red", blink: false },
		props: [P.label, P.var, { key: "color", label: "Farbe", type: "led" }, { key: "blink", label: "blinkt wenn an", type: "bool" }, P.invert],
	},
	{
		kind: "alarm", name: "Meldung", icon: "🚨", group: "Anzeigen", varHint: "BOOL: Meldung steht an (z. B. VDI3814.NC.Active)",
		defaults: { w: 260, h: 44, label: "", var: "", text: "Störung", prio: 4, ackVar: "" },
		props: [P.text, P.var, { key: "prio", label: "Meldeklasse", type: "prio" },
			{ key: "ackVar", label: "Quittiervariable", type: "var", hint: "BOOL, bekommt beim Quittieren einen Impuls" }],
	},
	{
		kind: "label", name: "Text", icon: "🔤", group: "Gestaltung", varHint: "",
		defaults: { w: 200, h: 40, label: "", var: "", text: "Überschrift", fontSize: 22, color: "" },
		props: [P.text, { key: "fontSize", label: "Schriftgröße", type: "number" }, P.color],
	},
];

export const KIND_INFO: Record<HmiKind, HmiKindInfo> = Object.fromEntries(HMI_KINDS.map((k) => [k.kind, k])) as Record<HmiKind, HmiKindInfo>;

/** The message classes (VDI 3814 Blatt 4.3 example list, as VDI3814.NC). */
export const PRIO_NAMES: Record<number, string> = {
	1: "Gefahr für Leben", 2: "Sicherheit", 3: "Alarm", 4: "Störung", 5: "Wartung", 6: "System", 7: "Handeingriff", 8: "Sonstiges",
};

export const PEN_COLORS = ["#e4572e", "#2f6fde", "#1f9d55", "#b07d00"];

/** The colour of a message class: 1-3 alarm (red), 4 fault (orange), 5 maintenance (yellow), 6-8 info (blue). */
export function prioColor(p: number): string {
	return p <= 3 ? COLORS.error : p === 4 ? "#f48c06" : p === 5 ? "#e9c46a" : COLORS.ledBlue;
}
export const GRID = 10;
export const MAX_PENS = 4;

// ---- pages and elements ------------------------------------------------------------------------------------
let counter = 0;
const newId = (prefix: string) => `${prefix}${Date.now().toString(36)}${(counter++).toString(36)}`;

export function newPage(name: string): HmiPage {
	return { id: newId("p"), name, width: 1280, height: 800, elements: [] };
}

export const snap = (v: number) => Math.round(v / GRID) * GRID;

/** A new element of `kind` with its defaults at (x, y) (snapped, inside the page). */
export function newElement(kind: HmiKind, x: number, y: number, page?: Pick<HmiPage, "width" | "height">): HmiElement {
	const d = KIND_INFO[kind].defaults;
	const e: HmiElement = { id: newId("e"), kind, x: 0, y: 0, ...structuredClone(d) };
	return place(e, x, y, page);
}

/** The element at (x, y), snapped to the grid and kept inside the page. */
export function place(e: HmiElement, x: number, y: number, page?: Pick<HmiPage, "width" | "height">): HmiElement {
	const W = page?.width ?? Infinity, H = page?.height ?? Infinity;
	return { ...e, x: Math.max(0, Math.min(snap(x), W - e.w)), y: Math.max(0, Math.min(snap(y), H - e.h)) };
}

export const MIN_SIZE = 20;

/** The element with a new size (snapped, at least MIN_SIZE, inside the page). */
export function resize(e: HmiElement, w: number, h: number, page?: Pick<HmiPage, "width" | "height">): HmiElement {
	const W = page?.width ?? Infinity, H = page?.height ?? Infinity;
	return { ...e, w: Math.max(MIN_SIZE, Math.min(snap(w), W - e.x)), h: Math.max(MIN_SIZE, Math.min(snap(h), H - e.y)) };
}

export function updateElement(p: HmiPage, id: string, change: Partial<HmiElement>): HmiPage {
	return { ...p, elements: p.elements.map((e) => (e.id === id ? { ...e, ...change, id: e.id, kind: e.kind } : e)) };
}

export function removeElements(p: HmiPage, ids: ReadonlySet<string>): HmiPage {
	return { ...p, elements: p.elements.filter((e) => !ids.has(e.id)) };
}

/** Copies of the elements, moved by one grid step down and right (new ids). */
export function duplicate(p: HmiPage, ids: ReadonlySet<string>): { page: HmiPage; ids: Set<string> } {
	const copies = p.elements.filter((e) => ids.has(e.id)).map((e) => place({ ...structuredClone(e), id: newId("e") }, e.x + 2 * GRID, e.y + 2 * GRID, p));
	return { page: { ...p, elements: [...p.elements, ...copies] }, ids: new Set(copies.map((c) => c.id)) };
}

/** Moves the elements to the front (last drawn) or back. */
export function reorder(p: HmiPage, ids: ReadonlySet<string>, front: boolean): HmiPage {
	const moving = p.elements.filter((e) => ids.has(e.id));
	const rest = p.elements.filter((e) => !ids.has(e.id));
	return { ...p, elements: front ? [...rest, ...moving] : [...moving, ...rest] };
}

/** Every variable path an element uses (bindings, pens, acknowledgement). */
export function elementVars(e: HmiElement): string[] {
	return [e.var, e.ackVar ?? "", ...(e.pens ?? []).map((p) => p.var)].map((v) => v.trim()).filter(Boolean);
}

/** The example page of the example project: operates and watches the sorting conveyor. */
export function examplePages(): HmiPage[] {
	const p: HmiPage = { id: "hmi_main", name: "Übersicht", width: 1280, height: 800, elements: [] };
	const add = (kind: HmiKind, x: number, y: number, over: Partial<HmiElement>) => {
		p.elements.push({ ...newElement(kind, x, y, p), ...over, id: `ex${p.elements.length + 1}` });
	};
	add("label", 40, 30, { text: "Sortieranlage", fontSize: 30, w: 400, h: 50 });
	add("switch", 40, 110, { label: "Anlage", var: "Enable", w: 150 });
	add("knob", 230, 100, { label: "Stückzahl", var: "PiecesMax", min: 0, max: 30, step: 1 });
	add("digital", 390, 110, { label: "Teile", var: "PLC_PRG.Pieces.CV", decimals: 0 });
	add("digital", 590, 110, { label: "Ausgeschleust", var: "PLC_PRG.Ejected", decimals: 0 });
	add("led", 800, 110, { label: "Lichtschranke 1", var: "Lightbarrier1", color: "green", w: 110 });
	add("led", 920, 110, { label: "Lichtschranke 2", var: "Lightbarrier2", color: "green", w: 110 });
	add("led", 1040, 110, { label: "Ventil 1", var: "Valve1", color: "yellow", w: 90 });
	add("gauge", 40, 290, { label: "Förderband", var: "ConveyorPwm", min: 0, max: 1000, unit: "‰", decimals: 0, warn: 800, alarm: 950 });
	add("bar", 250, 280, { label: "Teile", var: "PLC_PRG.Pieces.CV", min: 0, max: 30, unit: "", h: 190 });
	add("trend", 360, 280, { label: "Verlauf", min: 0, max: 1000, span: 30, w: 520, h: 240, pens: [
		{ var: "ConveyorPwm", color: PEN_COLORS[0], label: "Band ‰" },
		{ var: "Lightbarrier2", color: PEN_COLORS[1], label: "LS 2" },
	] });
	add("alarm", 40, 560, { text: "Stückzahl erreicht - Anlage steht", var: "PLC_PRG.Pieces.Q", prio: 5, w: 420 });
	return [p];
}

// ---- checking a stored project's pages -----------------------------------------------------------------------
const KINDS = new Set<string>(HMI_KINDS.map((k) => k.kind));
const num = (v: unknown) => typeof v === "number" && Number.isFinite(v);

/** Why the stored pages are no HMI (null: they are). */
export function hmiProblem(pages: unknown): string | null {
	if (!Array.isArray(pages)) return "Bedienseiten: keine Liste";
	const ids = new Set<string>();
	for (const p of pages as HmiPage[]) {
		if (typeof p?.id !== "string" || typeof p.name !== "string" || !num(p.width) || !num(p.height) || !Array.isArray(p.elements)) return "Bedienseite ungültig";
		if (ids.has(p.id)) return `Bedienseite '${p.name}' doppelt`;
		ids.add(p.id);
		for (const e of p.elements) {
			if (!KINDS.has(e?.kind) || typeof e.id !== "string" || ![e.x, e.y, e.w, e.h].every(num) || typeof e.var !== "string" || typeof e.label !== "string") {
				return `Bedienseite '${p.name}': Element ungültig`;
			}
			if (e.pens !== undefined && !(Array.isArray(e.pens) && e.pens.every((x) => typeof x?.var === "string" && typeof x.color === "string"))) {
				return `Bedienseite '${p.name}': Linienschreiber ungültig`;
			}
		}
	}
	return null;
}

// ---- variables ---------------------------------------------------------------------------------------------
export interface VarChoice {
	path: string;
	type: string;
}

/** The scalar variables of a program as paths for binding: globals and I/O, program instances with their
 *  variables and function block members (up to `depth` levels), array elements of small arrays. Library
 *  internals ("__") are left out. Sorted: globals first, then by path. */
export function scalarPaths(debug: DebugInfo, depth = 3, names?: (key: string) => string): VarChoice[] {
	const out: VarChoice[] = [];
	const typeText = (t: Type) => elemOf(t) ?? (t.k === "enum" ? t.name : "?");
	const walk = (prefix: string, t: Type, d: number) => {
		if (elemOf(t) !== null) {
			out.push({ path: prefix, type: t.k === "enum" ? t.name : typeText(t) });
			return;
		}
		if (d >= depth) return;
		if (t.k === "fb" || t.k === "struct") {
			const fields = t.k === "fb" ? t.layout.fields : t.fields;
			for (const f of fields) if (!f.name.startsWith("__")) walk(`${prefix}.${f.name}`, f.type, d + 1);
		} else if (t.k === "array" && t.dims.length === 1 && t.dims[0].hi - t.dims[0].lo < 16) {
			for (let i = t.dims[0].lo; i <= t.dims[0].hi; i++) walk(`${prefix}[${i}]`, t.elem, d + 1);
		}
	};
	for (const [key, s] of debug.symbols) {
		if (key.startsWith("__")) continue;
		walk(names?.(key) ?? key, s.type, s.type.k === "fb" ? 0 : 1);
	}
	return out.sort((a, b) => Number(a.path.includes(".")) - Number(b.path.includes(".")) || a.path.localeCompare(b.path));
}

/** The value to write for a number the operator chose: BOOL from <> 0, integers rounded and clamped to the
 *  type's range, TIME in ms, REAL as is. null: not a scalar. */
export function toValue(t: Type, x: number | boolean): Value | null {
	const e = elemOf(t);
	if (e === null) return null;
	if (e === "BOOL") return typeof x === "boolean" ? x : x !== 0;
	const n = typeof x === "boolean" ? (x ? 1 : 0) : x;
	if (e === "REAL" || e === "LREAL") return n;
	const info = ELEM[e];
	if (info.size === 8) {
		const v = BigInt(Math.round(n));
		return v < info.min ? info.min : v > info.max ? info.max : v;
	}
	return Math.min(Number(info.max), Math.max(Number(info.min), Math.round(n)));
}

/** A value as a number for displays and trends (BOOL 0/1). */
export function toNumber(v: Value | null): number | null {
	if (v === null) return null;
	return typeof v === "boolean" ? (v ? 1 : 0) : Number(v);
}

/** A knob/slider position: the value within [min, max] on the step grid (step 0: continuous). */
export function quantize(x: number, min: number, max: number, step: number): number {
	const lo = Math.min(min, max), hi = Math.max(min, max);
	const v = Math.min(hi, Math.max(lo, x));
	if (!(step > 0)) return v;
	const q = lo + Math.round((v - lo) / step) * step;
	return Number(Math.min(hi, q).toFixed(10));
}

/** The fraction 0..1 of v in [min, max] (clamped; min = max: 0). */
export function fraction(v: number, min: number, max: number): number {
	if (max === min || !Number.isFinite(v)) return 0;
	return Math.min(1, Math.max(0, (v - min) / (max - min)));
}

// ---- alarms (management) ------------------------------------------------------------------------------------
export interface AlarmEntry {
	/** page id + element id */
	key: string;
	page: string;
	text: string;
	prio: number;
	active: boolean;
	acked: boolean;
	/** When it came (ms since the epoch); gone: when it went (else null). */
	came: number;
	went: number | null;
	ackVar: string;
}

export interface AlarmEvent {
	time: number;
	text: string;
	prio: number;
	what: "kommt" | "geht" | "quittiert";
}

/** The alarm list of the runtime: the alarm elements of every page, their state per evaluation (come, go,
 *  acknowledged), and a history of events. An entry stays in the list while it is active or not acknowledged
 *  (VDI3814.NC's states 1-3). */
export class AlarmList {
	readonly entries = new Map<string, AlarmEntry>();
	readonly history: AlarmEvent[] = [];
	static readonly HISTORY = 200;

	/** Takes the current conditions (key -> active) of the alarm elements; `now` in ms. */
	update(pages: readonly HmiPage[], active: (e: HmiElement) => boolean | null, now: number): void {
		const seen = new Set<string>();
		for (const p of pages) {
			for (const e of p.elements) {
				if (e.kind !== "alarm") continue;
				const key = `${p.id}/${e.id}`;
				seen.add(key);
				const on = active(e);
				if (on === null) continue;  // no value (yet): no change
				const text = e.text || e.var;
				const prio = e.prio ?? 4;
				const x = this.entries.get(key);
				if (on && (!x || !x.active)) {
					this.entries.set(key, { key, page: p.name, text, prio, active: true, acked: false, came: now, went: null, ackVar: e.ackVar ?? "" });
					this.log(now, text, prio, "kommt");
				} else if (!on && x?.active) {
					x.active = false;
					x.went = now;
					this.log(now, text, prio, "geht");
					if (x.acked) this.entries.delete(key);
				} else if (x) {
					x.text = text;
					x.prio = prio;
				}
			}
		}
		for (const k of [...this.entries.keys()]) if (!seen.has(k)) this.entries.delete(k);  // element removed
	}

	/** Acknowledges one entry (or all); returns the acknowledge variables to pulse. */
	ack(key: string | null, now: number): string[] {
		const vars: string[] = [];
		for (const x of [...this.entries.values()]) {
			if ((key !== null && x.key !== key) || x.acked) continue;
			x.acked = true;
			this.log(now, x.text, x.prio, "quittiert");
			if (x.ackVar) vars.push(x.ackVar);
			if (!x.active) this.entries.delete(x.key);
		}
		return [...new Set(vars)];
	}

	/** The open entries: unacknowledged first, then by class (1 first), then newest first. */
	list(): AlarmEntry[] {
		return [...this.entries.values()].sort((a, b) => Number(a.acked) - Number(b.acked) || a.prio - b.prio || b.came - a.came);
	}

	get unacked(): number {
		return [...this.entries.values()].filter((x) => !x.acked).length;
	}

	private log(time: number, text: string, prio: number, what: AlarmEvent["what"]): void {
		this.history.unshift({ time, text, prio, what });
		if (this.history.length > AlarmList.HISTORY) this.history.pop();
	}
}

// ---- trends -------------------------------------------------------------------------------------------------
/** The samples of a line recorder: per pen (time [ms], value) pairs, at most `max`, older than the span dropped. */
export class TrendBuffer {
	readonly t: number[] = [];
	readonly v: (number | null)[][] = [];

	constructor(readonly pens: number, readonly max = 2000) {
		for (let i = 0; i < pens; i++) this.v.push([]);
	}

	push(time: number, values: (number | null)[], spanMs: number): void {
		const last = this.t[this.t.length - 1];
		if (last !== undefined && time <= last) return;
		this.t.push(time);
		for (let i = 0; i < this.pens; i++) this.v[i].push(values[i] ?? null);
		let drop = 0;
		while (drop < this.t.length - 1 && (this.t[drop] < time - spanMs * 1.05 || this.t.length - drop > this.max)) drop++;
		if (drop) {
			this.t.splice(0, drop);
			for (const s of this.v) s.splice(0, drop);
		}
	}
}
