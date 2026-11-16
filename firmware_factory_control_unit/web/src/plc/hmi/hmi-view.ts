// One HMI page as SVG (scaled to its container): the elements with the device's values. Mode "run": the
// operating elements write to the PLC (buttons while pressed, switches on a click, knobs and sliders by
// dragging, the wheel or the arrow keys). Mode "edit": select (click, Shift+click, a rubber band), move,
// resize (the handle), drop new elements from the palette - the page changes are events for the editor.
import { LitElement, html, nothing, svg, type TemplateResult } from "lit";
import { customElement, property, query, state } from "lit/decorators.js";
import { elemOf } from "../compiler/types.ts";
import type { PlcDevice } from "../ide/device.ts";
import { formatTime, formatValue, type Value } from "../ide/values.ts";
import {
	fraction, KIND_INFO, LED_COLORS, place, PRIO_NAMES, prioColor, quantize, resize, toNumber, toValue, TrendBuffer, type HmiElement, type HmiKind, type HmiPage,
} from "./hmi.ts";
import { trendOf } from "./hmi-trends.ts";
import { COLORS } from "../../ui/colors.ts";
import "./hmi.css";

/** The drag type of a new element from the palette. */
export const HMI_KIND_MIME = "application/x-fcu-hmi-kind";

const LED_RGB: Record<string, [string, string]> = {
	red: ["#ff3b30", "#4a1512"], green: ["#30d158", "#123a1d"], yellow: ["#ffd60a", "#453b0b"],
	blue: ["#0a84ff", "#0d2a4a"], white: ["#f5f5f7", "#3a3a3c"], orange: ["#ff9f0a", "#4a2e08"],
};

interface Read {
	value: Value | null;
	/** The variable is bound and the program on the device has it. */
	ok: boolean;
	bool: boolean;
	text: string;
	num: number | null;
}

type Drag =
	| { kind: "move"; ids: Set<string>; x0: number; y0: number; start: Map<string, { x: number; y: number }> }
	| { kind: "resize"; id: string; x0: number; y0: number; w: number; h: number }
	| { kind: "band"; x0: number; y0: number; x1: number; y1: number; additive: boolean }
	| { kind: "value"; id: string; y0: number; x0: number; v0: number; last: number };

@customElement("hmi-view")
export class HmiView extends LitElement {
	@property({ attribute: false }) page: HmiPage | null = null;
	@property({ attribute: false }) device: PlcDevice | null = null;
	@property() mode: "run" | "edit" = "run";
	@property({ attribute: false }) selected: ReadonlySet<string> = new Set();
	/** Changes with every device update (the parent counts them). */
	@property({ type: Number }) tick = 0;
	/** Alarm state of an alarm element (from the runtime's alarm list): blinking while unacknowledged. */
	@property({ attribute: false }) alarmState: ((e: HmiElement) => { active: boolean; acked: boolean } | null) | null = null;

	/** While dragging in edit mode: the page as it will be. */
	@state() private draft: HmiPage | null = null;
	@state() private drag: Drag | null = null;
	/** A knob/slider being operated: its value shown until the device has it. */
	@state() private operating: { id: string; value: number; until: number } | null = null;
	private pressed = new Set<string>();
	private lastWrite = 0;
	@query("svg") private svgEl?: SVGSVGElement;

	protected createRenderRoot() {
		return this;
	}

	// ---- values ------------------------------------------------------------------------------------------
	private read(path: string, e?: HmiElement): Read {
		const p = path.trim();
		if (!p || !this.device) return { value: null, ok: false, bool: false, text: "", num: null };
		const r = this.device.read(p);
		if (!r.symbol) return { value: null, ok: false, bool: false, text: "", num: null };
		const v = r.value;
		const el = elemOf(r.symbol.type);
		let text = "";
		if (v !== null) {
			if (el === "BOOL") text = v ? "EIN" : "AUS";
			else if (el === "TIME") text = formatTime(Number(v)).replace(/^T#/, "");
			else if (el === "REAL" || el === "LREAL") text = Number(v).toFixed(Math.max(0, Math.min(6, e?.decimals ?? 1)));
			else if (r.symbol.type.k === "enum") text = formatValue(r.symbol.type, v);
			else text = e?.decimals ? Number(v).toFixed(Math.min(6, e.decimals)) : String(v);
		}
		return { value: v, ok: true, bool: v === true || (typeof v === "number" && v !== 0) || (typeof v === "bigint" && v !== 0n), text, num: toNumber(v) };
	}

	private async write(path: string, x: number | boolean): Promise<void> {
		const dev = this.device;
		const p = path.trim();
		if (!dev || !p) return;
		const r = dev.read(p);
		if (!r.symbol) {
			this.notice(`${p}: keine Variable des Programms auf dem Gerät`);
			return;
		}
		const v = toValue(r.symbol.type, x);
		if (v === null) {
			this.notice(`${p}: keine einfache Variable`);
			return;
		}
		if (dev.state === "NO_PROGRAM") {
			this.notice("kein Programm geladen");
			return;
		}
		const ok = await dev.write(p, v);
		if (!ok) this.notice(dev.leasedByOther ? "ein anderer Browser debuggt: hier nur beobachten" : `${p}: Schreiben abgelehnt`);
	}

	private notice(text: string): void {
		this.dispatchEvent(new CustomEvent("notice", { detail: text, bubbles: true, composed: true }));
	}

	// ---- coordinates -------------------------------------------------------------------------------------
	private toPage(ev: { clientX: number; clientY: number }): { x: number; y: number } {
		const s = this.svgEl;
		const m = s?.getScreenCTM();
		if (!s || !m) return { x: 0, y: 0 };
		const pt = new DOMPoint(ev.clientX, ev.clientY).matrixTransform(m.inverse());
		return { x: pt.x, y: pt.y };
	}

	private get shown(): HmiPage | null {
		return this.draft ?? this.page;
	}

	// ---- edit mode -----------------------------------------------------------------------------------------
	private select(ids: Set<string>): void {
		this.dispatchEvent(new CustomEvent("select", { detail: ids }));
	}

	private onElementDown(ev: PointerEvent, e: HmiElement): void {
		if (this.mode !== "edit" || ev.button !== 0) return;
		ev.stopPropagation();
		let ids = new Set(this.selected);
		if (ev.shiftKey || ev.ctrlKey) {
			if (ids.has(e.id)) ids.delete(e.id);
			else ids.add(e.id);
			this.select(ids);
			return;
		}
		if (!ids.has(e.id)) ids = new Set([e.id]);
		this.select(ids);
		const p = this.toPage(ev);
		const page = this.page!;
		this.drag = { kind: "move", ids, x0: p.x, y0: p.y, start: new Map(page.elements.filter((x) => ids.has(x.id)).map((x) => [x.id, { x: x.x, y: x.y }])) };
		(ev.currentTarget as Element).setPointerCapture?.(ev.pointerId);
	}

	private onHandleDown(ev: PointerEvent, e: HmiElement): void {
		ev.stopPropagation();
		const p = this.toPage(ev);
		this.drag = { kind: "resize", id: e.id, x0: p.x, y0: p.y, w: e.w, h: e.h };
		(ev.currentTarget as Element).setPointerCapture?.(ev.pointerId);
	}

	private onBackgroundDown(ev: PointerEvent): void {
		if (this.mode !== "edit" || ev.button !== 0) return;
		const p = this.toPage(ev);
		const additive = ev.shiftKey || ev.ctrlKey;
		if (!additive) this.select(new Set());
		this.drag = { kind: "band", x0: p.x, y0: p.y, x1: p.x, y1: p.y, additive };
		(ev.currentTarget as Element).setPointerCapture?.(ev.pointerId);
	}

	private onMove(ev: PointerEvent): void {
		const d = this.drag;
		if (!d) return;
		const p = this.toPage(ev);
		const page = this.page!;
		if (d.kind === "move") {
			const dx = p.x - d.x0, dy = p.y - d.y0;
			this.draft = { ...page, elements: page.elements.map((e) => {
				const s = d.start.get(e.id);
				return s ? place(e, s.x + dx, s.y + dy, page) : e;
			}) };
		} else if (d.kind === "resize") {
			this.draft = { ...page, elements: page.elements.map((e) => (e.id === d.id ? resize(e, d.w + p.x - d.x0, d.h + p.y - d.y0, page) : e)) };
		} else if (d.kind === "band") {
			this.drag = { ...d, x1: p.x, y1: p.y };
		} else if (d.kind === "value") {
			this.operateDrag(d, p, ev);
		}
	}

	private onUp(): void {
		const d = this.drag;
		this.drag = null;
		if (!d) return;
		if (d.kind === "band") {
			const [x0, x1] = [Math.min(d.x0, d.x1), Math.max(d.x0, d.x1)];
			const [y0, y1] = [Math.min(d.y0, d.y1), Math.max(d.y0, d.y1)];
			if (x1 - x0 < 3 && y1 - y0 < 3) return;
			const hit = this.page!.elements.filter((e) => e.x < x1 && e.x + e.w > x0 && e.y < y1 && e.y + e.h > y0).map((e) => e.id);
			this.select(new Set([...(d.additive ? this.selected : []), ...hit]));
			return;
		}
		if (d.kind === "value") {
			const e = this.page?.elements.find((x) => x.id === d.id);
			if (e) void this.write(e.var, d.last);
			this.operating = { id: d.id, value: d.last, until: Date.now() + 1500 };
			return;
		}
		const draft = this.draft;
		this.draft = null;
		if (draft) this.dispatchEvent(new CustomEvent("pagechange", { detail: draft }));
	}

	private onDragOver(ev: DragEvent): void {
		if (this.mode === "edit" && ev.dataTransfer?.types.includes(HMI_KIND_MIME)) {
			ev.preventDefault();
			ev.dataTransfer.dropEffect = "copy";
		}
	}

	private onDrop(ev: DragEvent): void {
		const kind = ev.dataTransfer?.getData(HMI_KIND_MIME) as HmiKind | undefined;
		if (this.mode !== "edit" || !kind || !KIND_INFO[kind]) return;
		ev.preventDefault();
		const p = this.toPage(ev);
		const d = KIND_INFO[kind].defaults;
		this.dispatchEvent(new CustomEvent("addelement", { detail: { kind, x: p.x - d.w / 2, y: p.y - d.h / 2 } }));
	}

	// ---- run mode: operating ----------------------------------------------------------------------------------
	private canOperate(e: HmiElement): boolean {
		return this.mode === "run" && !e.readonly && !!e.var.trim();
	}

	private press(e: HmiElement, down: boolean, ev?: PointerEvent): void {
		if (!this.canOperate(e)) return;
		if (down) {
			this.pressed.add(e.id);
			(ev?.currentTarget as Element | undefined)?.setPointerCapture?.(ev!.pointerId);
		} else if (!this.pressed.delete(e.id)) return;
		this.requestUpdate();
		void this.write(e.var, down !== !!e.invert);
	}

	private toggle(e: HmiElement): void {
		if (!this.canOperate(e)) return;
		void this.write(e.var, !this.read(e.var).bool);
	}

	private range(e: HmiElement): { min: number; max: number; step: number } {
		return { min: e.min ?? 0, max: e.max ?? 100, step: e.step ?? 0 };
	}

	private valueDown(ev: PointerEvent, e: HmiElement): void {
		if (!this.canOperate(e) || ev.button !== 0) return;
		ev.preventDefault();
		(ev.currentTarget as Element).setPointerCapture?.(ev.pointerId);
		const p = this.toPage(ev);
		const v0 = this.read(e.var).num ?? this.range(e).min;
		this.drag = { kind: "value", id: e.id, x0: p.x, y0: p.y, v0, last: v0 };
		if (e.kind === "slider") this.operateDrag(this.drag, p, ev);
	}

	private operateDrag(d: Extract<Drag, { kind: "value" }>, p: { x: number; y: number }, ev: PointerEvent): void {
		const e = this.page?.elements.find((x) => x.id === d.id);
		if (!e) return;
		const { min, max, step } = this.range(e);
		let v: number;
		if (e.kind === "slider") {
			const g = this.sliderGeom(e);
			const f = g.horizontal ? (p.x - e.x - g.x0) / (g.x1 - g.x0) : (e.y + g.y0 - p.y) / (g.y0 - g.y1);
			v = min + Math.min(1, Math.max(0, f)) * (max - min);
		} else {
			// knob: 200 px of vertical travel (or horizontal) over the whole range; Shift: fine
			const travel = (d.y0 - p.y + (p.x - d.x0)) / (ev.shiftKey ? 1000 : 200);
			v = d.v0 + travel * (max - min);
		}
		const q = quantize(v, min, max, step);
		this.drag = { ...d, last: q };
		this.operating = { id: e.id, value: q, until: Date.now() + 1500 };
		if (Date.now() - this.lastWrite > 100) {
			this.lastWrite = Date.now();
			void this.write(e.var, q);
		}
	}

	private nudge(e: HmiElement, dir: number, big = false): void {
		if (!this.canOperate(e)) return;
		const { min, max, step } = this.range(e);
		const s = step > 0 ? step : (max - min) / 100;
		const cur = this.operating?.id === e.id ? this.operating.value : this.read(e.var).num ?? min;
		const q = quantize(cur + dir * s * (big ? 10 : 1), min, max, step);
		this.operating = { id: e.id, value: q, until: Date.now() + 1500 };
		void this.write(e.var, q);
	}

	private valueKey(ev: KeyboardEvent, e: HmiElement): void {
		const dir = ev.key === "ArrowUp" || ev.key === "ArrowRight" ? 1 : ev.key === "ArrowDown" || ev.key === "ArrowLeft" ? -1 : 0;
		if (dir === 0 && ev.key !== "PageUp" && ev.key !== "PageDown") return;
		ev.preventDefault();
		this.nudge(e, dir || (ev.key === "PageUp" ? 1 : -1), dir === 0);
	}

	/** The value a knob/slider shows: the operator's while operating (and shortly after), else the device's. */
	private shownValue(e: HmiElement): number | null {
		const o = this.operating;
		if (o && o.id === e.id && (this.drag?.kind === "value" || Date.now() < o.until)) return o.value;
		return this.read(e.var).num;
	}

	// ---- rendering --------------------------------------------------------------------------------------------
	render() {
		void this.tick;
		const page = this.shown;
		if (!page) return html`<div class="hmi-empty">Keine Seite</div>`;
		const edit = this.mode === "edit";
		const band = this.drag?.kind === "band" ? this.drag : null;
		return html`
			<svg class="hmi-svg ${edit ? "hmi-edit" : "hmi-run"}" viewBox="0 0 ${page.width} ${page.height}" preserveAspectRatio="xMidYMid meet"
				@pointermove=${(e: PointerEvent) => this.onMove(e)} @pointerup=${() => this.onUp()} @pointercancel=${() => this.onUp()}
				@dragover=${(e: DragEvent) => this.onDragOver(e)} @drop=${(e: DragEvent) => this.onDrop(e)}>
				<defs>
					<pattern id="hmi-grid" width="10" height="10" patternUnits="userSpaceOnUse"><path d="M10 0H0V10" class="hmi-grid-line" /></pattern>
					<radialGradient id="hmi-led-shine" cx="35%" cy="30%" r="60%"><stop offset="0" stop-color="#fff" stop-opacity="0.85" /><stop offset="1" stop-color="#fff" stop-opacity="0" /></radialGradient>
				</defs>
				<rect class="hmi-page" x="0" y="0" width=${page.width} height=${page.height} @pointerdown=${(e: PointerEvent) => this.onBackgroundDown(e)} />
				${edit ? svg`<rect x="0" y="0" width=${page.width} height=${page.height} fill="url(#hmi-grid)" pointer-events="none" />` : nothing}
				${page.elements.map((e) => this.renderElement(e, edit))}
				${band ? svg`<rect class="hmi-band" x=${Math.min(band.x0, band.x1)} y=${Math.min(band.y0, band.y1)} width=${Math.abs(band.x1 - band.x0)} height=${Math.abs(band.y1 - band.y0)} />` : nothing}
			</svg>`;
	}

	private renderElement(e: HmiElement, edit: boolean): TemplateResult {
		const sel = edit && this.selected.has(e.id);
		const vars = e.kind === "trend" ? (e.pens ?? []).map((p) => p.var) : e.kind === "label" ? [] : [e.var];
		const missing = !edit && this.device?.debug && vars.some((v) => v.trim() && !this.device!.read(v.trim()).symbol);
		const unbound = e.kind !== "label" && vars.every((v) => !v.trim());
		return svg`
			<g class="hmi-el hmi-${e.kind} ${sel ? "hmi-selected" : ""}" transform="translate(${e.x} ${e.y})"
				@pointerdown=${(ev: PointerEvent) => this.onElementDown(ev, e)}>
				${this.body(e)}
				${missing ? svg`<rect class="hmi-missing" x="1" y="1" width=${e.w - 2} height=${e.h - 2}><title>Variable fehlt im Programm auf dem Gerät: ${vars.join(", ")}</title></rect>` : nothing}
				${edit ? svg`<rect class="hmi-hit" x="0" y="0" width=${e.w} height=${e.h}><title>${KIND_INFO[e.kind].name}${vars[0] ? `: ${vars.join(", ")}` : ""}</title></rect>` : nothing}
				${edit && unbound ? svg`<text class="hmi-unbound" x=${e.w - 4} y="12" text-anchor="end">⚠ ohne Variable</text>` : nothing}
				${sel ? svg`<rect class="hmi-sel" x="-2" y="-2" width=${e.w + 4} height=${e.h + 4} />
					${this.selected.size === 1 ? svg`<rect class="hmi-handle" x=${e.w - 6} y=${e.h - 6} width="12" height="12"
						@pointerdown=${(ev: PointerEvent) => this.onHandleDown(ev, e)} />` : nothing}` : nothing}
			</g>`;
	}

	/** The caption row (when the element has a label): its height. */
	private caption(e: HmiElement, at: "top" | "bottom" = "top"): { h: number; t: TemplateResult | typeof nothing } {
		if (!e.label) return { h: 0, t: nothing };
		const y = at === "top" ? 14 : e.h - 4;
		return { h: 20, t: svg`<text class="hmi-caption" x=${e.w / 2} y=${y} text-anchor="middle">${e.label}</text>` };
	}

	private body(e: HmiElement): TemplateResult {
		switch (e.kind) {
			case "button": return this.button(e);
			case "switch": return this.switch(e);
			case "knob": return this.knob(e);
			case "slider": return this.slider(e);
			case "digital": return this.digital(e);
			case "gauge": return this.gauge(e);
			case "bar": return this.bar(e);
			case "trend": return this.trend(e);
			case "led": return this.led(e);
			case "alarm": return this.alarm(e);
			case "label": return svg`<text class="hmi-label" x="4" y=${e.h / 2} dominant-baseline="central"
				style="font-size:${e.fontSize ?? 20}px;${e.color ? `fill:${e.color}` : ""}">${e.text ?? ""}</text>`;
		}
	}

	private button(e: HmiElement): TemplateResult {
		const c = this.caption(e);
		const r = this.read(e.var);
		const on = this.pressed.has(e.id) || (r.ok && r.bool !== !!e.invert);
		const h = e.h - c.h;
		const col = e.color || "#2f6fde";
		return svg`${c.t}
			<g class="hmi-press ${this.canOperate(e) ? "hmi-active" : ""}" transform="translate(0 ${c.h})"
				@pointerdown=${(ev: PointerEvent) => this.press(e, true, ev)} @pointerup=${() => this.press(e, false)}
				@pointercancel=${() => this.press(e, false)} @lostpointercapture=${() => this.press(e, false)}>
				<rect class="hmi-btn-shadow" x="2" y="4" width=${e.w - 4} height=${h - 4} rx="10" />
				<rect x="2" y=${on ? 4 : 1} width=${e.w - 4} height=${h - 5} rx="10" fill=${on ? col : "var(--ui-btn-neutral-inactive)"} stroke=${col} stroke-width="2" />
				<text class="hmi-btn-text" x=${e.w / 2} y=${(on ? 4 : 1) + (h - 5) / 2} text-anchor="middle" dominant-baseline="central"
					style=${on ? "fill:#fff" : `fill:${col}`}>${e.text ?? ""}</text>
			</g>`;
	}

	private switch(e: HmiElement): TemplateResult {
		const c = this.caption(e);
		const r = this.read(e.var);
		const on = r.ok && r.bool !== !!e.invert;
		const ch = e.h - c.h;
		const th = Math.min(ch - 8, 34), tw = Math.min(e.w * 0.55, th * 2);
		const y = c.h + (ch - th) / 2;
		const col = e.color || "#1f9d55";
		return svg`${c.t}
			<g class=${this.canOperate(e) ? "hmi-active" : ""} @click=${() => this.toggle(e)}>
				<rect x="0" y=${c.h} width=${e.w} height=${ch} fill="transparent" />
				<rect x="4" y=${y} width=${tw} height=${th} rx=${th / 2} fill=${on ? col : "var(--ui-color-border-input)"} />
				<circle cx=${on ? 4 + tw - th / 2 : 4 + th / 2} cy=${y + th / 2} r=${th / 2 - 3} class="hmi-thumb" />
				<text class="hmi-state" x=${tw + 12} y=${y + th / 2} dominant-baseline="central">${!r.ok ? "–" : on ? "EIN" : "AUS"}</text>
			</g>`;
	}

	private knob(e: HmiElement): TemplateResult {
		const c = this.caption(e);
		const { min, max } = this.range(e);
		const v = this.shownValue(e);
		const size = Math.max(20, Math.min(e.w, e.h - c.h - 22));
		const cx = e.w / 2, cy = c.h + size / 2, rad = size / 2 - 6;
		const f = v === null ? 0 : fraction(v, min, max);
		const a0 = -225, a = a0 + 270 * f;
		const col = e.color || "#2f6fde";
		const pt = (deg: number, r: number) => [cx + r * Math.cos((deg * Math.PI) / 180), cy + r * Math.sin((deg * Math.PI) / 180)];
		const arc = (from: number, to: number, r: number) => {
			const [x0, y0] = pt(from, r), [x1, y1] = pt(to, r);
			return `M${x0} ${y0} A${r} ${r} 0 ${to - from > 180 ? 1 : 0} 1 ${x1} ${y1}`;
		};
		const [px, py] = pt(a, rad - 12);
		const text = v === null ? "–" : `${v.toFixed(Math.max(0, Math.min(6, e.decimals ?? 0)))}${e.unit ? ` ${e.unit}` : ""}`;
		return svg`${c.t}
			<g class=${this.canOperate(e) ? "hmi-active hmi-focusable" : ""} tabindex=${this.canOperate(e) ? 0 : -1}
				@pointerdown=${(ev: PointerEvent) => this.valueDown(ev, e)} @keydown=${(ev: KeyboardEvent) => this.valueKey(ev, e)}
				@wheel=${(ev: WheelEvent) => { if (this.canOperate(e)) { ev.preventDefault(); this.nudge(e, ev.deltaY < 0 ? 1 : -1, ev.shiftKey); } }}>
				<circle cx=${cx} cy=${cy} r=${rad + 4} fill="transparent" />
				<path d=${arc(a0, a0 + 270, rad)} class="hmi-track-arc" />
				${f > 0.001 ? svg`<path d=${arc(a0, a, rad)} stroke=${col} class="hmi-value-arc" />` : nothing}
				<circle cx=${cx} cy=${cy} r=${rad - 8} class="hmi-knob-cap" />
				<line x1=${cx} y1=${cy} x2=${px} y2=${py} stroke=${col} class="hmi-knob-ptr" />
			</g>
			<text class="hmi-value" x=${cx} y=${c.h + size + 16} text-anchor="middle">${text}</text>`;
	}

	private sliderGeom(e: HmiElement) {
		const c = e.label ? 20 : 0;
		const horizontal = e.w >= e.h - c;
		return horizontal
			? { horizontal, x0: 12, x1: e.w - 12, y0: c + (e.h - c) / 2 - 6, y1: c + (e.h - c) / 2 - 6 }
			: { horizontal, x0: e.w / 2, x1: e.w / 2, y0: e.h - 26, y1: c + 10 };
	}

	private slider(e: HmiElement): TemplateResult {
		const c = this.caption(e);
		const { min, max } = this.range(e);
		const g = this.sliderGeom(e);
		const v = this.shownValue(e);
		const f = v === null ? 0 : fraction(v, min, max);
		const tx = g.x0 + (g.x1 - g.x0) * f, ty = g.y0 + (g.y1 - g.y0) * f;
		const col = e.color || "#2f6fde";
		const text = v === null ? "–" : `${v.toFixed(Math.max(0, Math.min(6, e.decimals ?? 0)))}${e.unit ? ` ${e.unit}` : ""}`;
		return svg`${c.t}
			<g class=${this.canOperate(e) ? "hmi-active hmi-focusable" : ""} tabindex=${this.canOperate(e) ? 0 : -1}
				@pointerdown=${(ev: PointerEvent) => this.valueDown(ev, e)} @keydown=${(ev: KeyboardEvent) => this.valueKey(ev, e)}
				@wheel=${(ev: WheelEvent) => { if (this.canOperate(e)) { ev.preventDefault(); this.nudge(e, ev.deltaY < 0 ? 1 : -1, ev.shiftKey); } }}>
				<rect x="0" y=${c.h} width=${e.w} height=${e.h - c.h} fill="transparent" />
				<line x1=${g.x0} y1=${g.y0} x2=${g.x1} y2=${g.y1} class="hmi-slider-track" />
				<line x1=${g.x0} y1=${g.y0} x2=${tx} y2=${ty} stroke=${col} class="hmi-slider-fill" />
				<circle cx=${tx} cy=${ty} r="10" class="hmi-thumb" stroke=${col} />
			</g>
			<text class="hmi-value" x=${g.horizontal ? e.w / 2 : e.w / 2} y=${g.horizontal ? g.y0 + 28 : e.h - 4} text-anchor="middle">${text}</text>`;
	}

	private digital(e: HmiElement): TemplateResult {
		const c = this.caption(e);
		const r = this.read(e.var, e);
		const h = e.h - c.h;
		const fs = Math.max(10, Math.min(h * 0.6, e.w / Math.max(4, r.text.length + (e.unit ? e.unit.length * 0.6 : 0)) * 1.5));
		return svg`${c.t}
			<rect x="0" y=${c.h} width=${e.w} height=${h} rx="6" class="hmi-lcd" />
			<text class="hmi-digits" x=${e.w - 8 - (e.unit ? fs * 0.45 * e.unit.length + 6 : 0)} y=${c.h + h / 2} text-anchor="end" dominant-baseline="central"
				style="font-size:${fs}px;fill:${e.color || "#39ff7a"}">${r.ok ? r.text || "–" : "----"}</text>
			${e.unit ? svg`<text class="hmi-digits-unit" x=${e.w - 8} y=${c.h + h / 2 + fs * 0.25} text-anchor="end" style="fill:${e.color || "#39ff7a"};font-size:${Math.max(9, fs * 0.45)}px">${e.unit}</text>` : nothing}`;
	}

	private gauge(e: HmiElement): TemplateResult {
		const c = this.caption(e);
		const { min, max } = this.range(e);
		const r = this.read(e.var, e);
		const cx = e.w / 2;
		const rad = Math.max(10, Math.min(e.w / 2 - 8, (e.h - c.h - 12) / 1.5));
		const cy = c.h + rad + 6;
		const a0 = 150, sweep = 240;
		const ang = (v: number) => a0 + sweep * fraction(v, min, max);
		const pt = (deg: number, rr: number) => [cx + rr * Math.cos((deg * Math.PI) / 180), cy + rr * Math.sin((deg * Math.PI) / 180)];
		const arc = (from: number, to: number, rr: number) => {
			const [x0, y0] = pt(from, rr), [x1, y1] = pt(to, rr);
			return `M${x0} ${y0} A${rr} ${rr} 0 ${to - from > 180 ? 1 : 0} 1 ${x1} ${y1}`;
		};
		const zones: TemplateResult[] = [];
		const hasWarn = e.warn !== undefined && e.warn !== null && Number.isFinite(e.warn);
		const hasAlarm = e.alarm !== undefined && e.alarm !== null && Number.isFinite(e.alarm);
		if (hasWarn) zones.push(svg`<path d=${arc(ang(e.warn!), hasAlarm ? ang(e.alarm!) : a0 + sweep, rad - 4)} class="hmi-zone-warn" />`);
		if (hasAlarm) zones.push(svg`<path d=${arc(ang(e.alarm!), a0 + sweep, rad - 4)} class="hmi-zone-alarm" />`);
		const ticks: TemplateResult[] = [];
		for (let i = 0; i <= 10; i++) {
			const d = a0 + (sweep * i) / 10;
			const [x0, y0] = pt(d, rad - (i % 5 === 0 ? 14 : 9)), [x1, y1] = pt(d, rad);
			ticks.push(svg`<line x1=${x0} y1=${y0} x2=${x1} y2=${y1} class="hmi-tick" />`);
			if (i % 5 === 0 && rad > 40) {
				const [lx, ly] = pt(d, rad - 26);
				const val = min + ((max - min) * i) / 10;
				ticks.push(svg`<text class="hmi-tick-label" x=${lx} y=${ly} text-anchor="middle" dominant-baseline="central">${Number(val.toFixed(2))}</text>`);
			}
		}
		const [nx, ny] = pt(r.num === null ? a0 : ang(r.num), rad - 10);
		const level = r.num === null ? "" : hasAlarm && r.num >= e.alarm! ? "hmi-alarm-text" : hasWarn && r.num >= e.warn! ? "hmi-warn-text" : "";
		return svg`${c.t}
			<path d=${arc(a0, a0 + sweep, rad)} class="hmi-gauge-face" />
			${zones}${ticks}
			${r.ok ? svg`<line x1=${cx} y1=${cy} x2=${nx} y2=${ny} class="hmi-needle" />` : nothing}
			<circle cx=${cx} cy=${cy} r="6" class="hmi-needle-hub" />
			<text class="hmi-value ${level}" x=${cx} y=${cy + rad * 0.55} text-anchor="middle">${r.ok ? `${r.text}${e.unit ? ` ${e.unit}` : ""}` : "–"}</text>`;
	}

	private bar(e: HmiElement): TemplateResult {
		const c = this.caption(e);
		const { min, max } = this.range(e);
		const r = this.read(e.var, e);
		const h = e.h - c.h - 20;
		const vertical = h >= e.w;
		const f = r.num === null ? 0 : fraction(r.num, min, max);
		const col = r.num !== null && e.alarm !== undefined && e.alarm !== null && r.num >= e.alarm ? COLORS.error
			: r.num !== null && e.warn !== undefined && e.warn !== null && r.num >= e.warn ? COLORS.warn : e.color || COLORS.info;
		const fill = vertical
			? svg`<rect x="4" y=${c.h + 2 + (h - 4) * (1 - f)} width=${e.w - 8} height=${(h - 4) * f} fill=${col} rx="2" />`
			: svg`<rect x="2" y=${c.h + 4} width=${(e.w - 4) * f} height=${h - 8} fill=${col} rx="2" />`;
		return svg`${c.t}
			<rect x="0" y=${c.h} width=${e.w} height=${h} rx="4" class="hmi-bar-frame" />
			${fill}
			<text class="hmi-value" x=${e.w / 2} y=${e.h - 4} text-anchor="middle">${r.ok ? `${r.text}${e.unit ? ` ${e.unit}` : ""}` : "–"}</text>`;
	}

	private trend(e: HmiElement): TemplateResult {
		const c = this.caption(e);
		const pens = e.pens ?? [];
		const buf: TrendBuffer | null = this.device ? trendOf(this.device, this.page?.id ?? "", e) : null;
		const { min, max } = this.range(e);
		const span = Math.max(1, e.span ?? 60) * 1000;
		const left = 46, right = 8, top = c.h + 22, bottom = 18;
		const pw = e.w - left - right, ph = e.h - top - bottom;
		const now = Date.now();
		const isBool = pens.map((p) => (this.device?.read(p.var.trim()).symbol ? elemOf(this.device.read(p.var.trim()).symbol!.type) === "BOOL" : false));
		const yOf = (v: number, b: boolean) => top + ph * (1 - (b ? 0.05 + 0.2 * v : fraction(v, min, max)));
		const lines = pens.map((p, i) => {
			if (!buf) return nothing;
			const pts: string[] = [];
			let seg = "";
			for (let k = 0; k < buf.t.length; k++) {
				const v = buf.v[i]?.[k];
				const x = left + pw * (1 - (now - buf.t[k]) / span);
				if (v === null || v === undefined || x < left - 1) {
					if (seg) pts.push(seg);
					seg = "";
					continue;
				}
				if (isBool[i] && seg) {
					// a binary trace is drawn as steps
					const prev = buf.v[i][k - 1];
					if (prev !== null && prev !== undefined && prev !== v) seg += ` ${x.toFixed(1)},${yOf(prev, true).toFixed(1)}`;
				}
				seg += ` ${x.toFixed(1)},${yOf(v, isBool[i]).toFixed(1)}`;
			}
			if (seg) pts.push(seg);
			return pts.map((s) => svg`<polyline points=${s} fill="none" stroke=${p.color} stroke-width="2" />`);
		});
		const grid: TemplateResult[] = [];
		for (let i = 0; i <= 4; i++) {
			const y = top + (ph * i) / 4;
			grid.push(svg`<line x1=${left} y1=${y} x2=${left + pw} y2=${y} class="hmi-grid-major" />`);
			grid.push(svg`<text class="hmi-axis" x=${left - 4} y=${y} text-anchor="end" dominant-baseline="central">${Number((max - ((max - min) * i) / 4).toFixed(2))}</text>`);
		}
		for (let i = 0; i <= 4; i++) {
			const x = left + (pw * i) / 4;
			grid.push(svg`<line x1=${x} y1=${top} x2=${x} y2=${top + ph} class="hmi-grid-major" />`);
			grid.push(svg`<text class="hmi-axis" x=${x} y=${top + ph + 12} text-anchor="middle">${i === 4 ? "jetzt" : `−${Math.round(((4 - i) * span) / 4000)} s`}</text>`);
		}
		let lx = left;
		const legend = pens.map((p) => {
			const r = this.read(p.var, e);
			const t = `${p.label || p.var}: ${r.ok ? r.text : "?"}`;
			const g = svg`<rect x=${lx} y=${c.h + 6} width="10" height="10" fill=${p.color} /><text class="hmi-legend" x=${lx + 14} y=${c.h + 15}>${t}</text>`;
			lx += 22 + t.length * 6.2;
			return g;
		});
		return svg`${c.t}
			<rect x="0" y=${c.h} width=${e.w} height=${e.h - c.h} rx="4" class="hmi-trend-frame" />
			${grid}
			<svg x=${left} y=${top} width=${pw} height=${ph} viewBox="${left} ${top} ${pw} ${ph}" overflow="hidden">${lines}</svg>
			${legend}
			${pens.length === 0 ? svg`<text class="hmi-axis" x=${left + pw / 2} y=${top + ph / 2} text-anchor="middle">keine Kurven (Eigenschaften → Kurven)</text>` : nothing}`;
	}

	private led(e: HmiElement): TemplateResult {
		const c = this.caption(e, "bottom");
		const r = this.read(e.var);
		const blinkOff = e.blink && Math.floor(Date.now() / 500) % 2 === 1;
		const on = r.ok && r.bool !== !!e.invert && !blinkOff;
		const col = LED_RGB[(LED_COLORS as readonly string[]).includes(e.color ?? "") ? e.color! : "red"];
		const size = Math.max(8, Math.min(e.w, e.h - c.h) - 8);
		const cx = e.w / 2, cy = (e.h - c.h) / 2, rr = size / 2;
		return svg`
			${on ? svg`<circle cx=${cx} cy=${cy} r=${rr + 6} fill=${col[0]} opacity="0.35" class="hmi-glow" />` : nothing}
			<circle cx=${cx} cy=${cy} r=${rr} fill=${on ? col[0] : col[1]} class="hmi-led-body" />
			<circle cx=${cx} cy=${cy} r=${rr} fill="url(#hmi-led-shine)" opacity=${on ? 0.6 : 0.25} />
			${c.t}`;
	}

	private alarm(e: HmiElement): TemplateResult {
		const r = this.read(e.var);
		const st = this.alarmState?.(e) ?? null;
		const active = r.ok && r.bool;
		const unacked = st ? !st.acked : false;
		const blinkOff = unacked && Math.floor(Date.now() / 500) % 2 === 1;
		const prio = e.prio ?? 4;
		const col = prioColor(prio);
		const lit = (active || unacked) && !blinkOff;
		return svg`
			<g class=${this.mode === "run" && unacked ? "hmi-active" : ""} @click=${() => {
				if (this.mode === "run" && unacked) this.dispatchEvent(new CustomEvent("ack", { detail: e, bubbles: true, composed: true }));
			}}>
				<rect x="0" y="0" width=${e.w} height=${e.h} rx="6" class="hmi-alarm-box" style=${lit ? `fill:${col};stroke:${col}` : `stroke:${active || unacked ? col : "var(--ui-color-border)"}`} />
				<text class="hmi-alarm-icon" x="10" y=${e.h / 2} dominant-baseline="central" style=${lit ? "fill:#fff" : ""}>${active ? "▲" : unacked ? "△" : "●"}</text>
				<text class="hmi-alarm-text" x="32" y=${e.h / 2} dominant-baseline="central" style=${lit ? "fill:#fff" : ""}>${e.text ?? ""}</text>
				<title>${PRIO_NAMES[prio] ?? ""}${unacked ? " - klicken zum Quittieren" : ""}</title>
			</g>`;
	}
}

declare global {
	interface HTMLElementTagNameMap {
		"hmi-view": HmiView;
	}
}
