// The HMI editor (a view of the PLC IDE): pages as tabs, the element palette (drag onto the page or click),
// the page with the device's live values (edit: select, move, resize; "Bedienen": operate it like the
// runtime), the properties of the selection with the variables of the last build to choose from. Every change
// is a "change" event with all pages; undo/redo keep the last 50 states.
import { LitElement, html, nothing } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { confirmDialog } from "../../shell/dialogs.ts";
import type { DebugInfo } from "../compiler/compiler.ts";
import type { PlcDevice } from "../ide/device.ts";
import {
	duplicate, GRID, HMI_KINDS, KIND_INFO, LED_COLOR_NAMES, LED_COLORS, MAX_PENS, newElement, newPage, PEN_COLORS, place, PRIO_NAMES, removeElements, reorder,
	scalarPaths, updateElement, type HmiElement, type HmiKind, type HmiPage, type HmiProp,
} from "./hmi.ts";
import { HMI_KIND_MIME } from "./hmi-view.ts";
import "./hmi-view.ts";

const UNDO_MAX = 50;

@customElement("hmi-editor")
export class HmiEditor extends LitElement {
	@property({ attribute: false }) pages: HmiPage[] = [];
	@property({ attribute: false }) device: PlcDevice | null = null;
	/** The debug information of the last build (variables to choose from, binding check). */
	@property({ attribute: false }) debug: DebugInfo | null = null;
	/** Upper-case symbol key -> the name as written (globals, programs). */
	@property({ attribute: false }) names: ((key: string) => string) | null = null;
	@property({ type: Number }) tick = 0;
	@property() pageId = "";

	@state() private selected = new Set<string>();
	@state() private live = false;
	private undo: HmiPage[][] = [];
	private redo: HmiPage[][] = [];
	private choicesFor: DebugInfo | null = null;
	private choices: { path: string; type: string }[] = [];

	protected createRenderRoot() {
		return this;
	}

	private get page(): HmiPage | undefined {
		return this.pages.find((p) => p.id === this.pageId) ?? this.pages[0];
	}

	// ---- changes ---------------------------------------------------------------------------------------------
	private emit(pages: HmiPage[], record = true): void {
		if (record) {
			this.undo.push(this.pages);
			if (this.undo.length > UNDO_MAX) this.undo.shift();
			this.redo = [];
		}
		this.dispatchEvent(new CustomEvent("change", { detail: pages }));
	}

	private setPage(p: HmiPage): void {
		this.emit(this.pages.map((x) => (x.id === p.id ? p : x)));
	}

	private undoStep(back: boolean): void {
		const from = back ? this.undo : this.redo;
		const to = back ? this.redo : this.undo;
		const s = from.pop();
		if (!s) return;
		to.push(this.pages);
		this.selected = new Set();
		this.emit(s, false);
	}

	private selectPage(id: string): void {
		this.selected = new Set();
		this.dispatchEvent(new CustomEvent("page", { detail: id }));
	}

	private addPage(): void {
		const names = new Set(this.pages.map((p) => p.name));
		let n = this.pages.length + 1;
		while (names.has(`Seite ${n}`)) n++;
		const p = newPage(`Seite ${n}`);
		this.emit([...this.pages, p]);
		this.selectPage(p.id);
	}

	private async removePage(): Promise<void> {
		const p = this.page;
		if (!p || !(await confirmDialog({ title: "Seite löschen", message: `"${p.name}" mit ${p.elements.length} Elementen löschen?`, confirm: "Löschen", danger: true }))) return;
		const rest = this.pages.filter((x) => x.id !== p.id);
		this.emit(rest);
		this.selectPage(rest[0]?.id ?? "");
	}

	private movePage(d: number): void {
		const i = this.pages.findIndex((p) => p.id === this.page?.id);
		const j = i + d;
		if (i < 0 || j < 0 || j >= this.pages.length) return;
		const next = [...this.pages];
		[next[i], next[j]] = [next[j], next[i]];
		this.emit(next);
	}

	private addElement(kind: HmiKind, x?: number, y?: number): void {
		const p = this.page;
		if (!p) return;
		const d = KIND_INFO[kind].defaults;
		// a click on the palette: the next free spot from the top left
		let px = x ?? 40, py = y ?? 40;
		if (x === undefined) while (p.elements.some((e) => e.x === px && e.y === py) && py < p.height - d.h) py += 2 * GRID, px += 2 * GRID;
		const e = newElement(kind, px, py, p);
		this.setPage({ ...p, elements: [...p.elements, e] });
		this.selected = new Set([e.id]);
	}

	private change(e: HmiElement, change: Partial<HmiElement>): void {
		const p = this.page;
		if (p) this.setPage(updateElement(p, e.id, change));
	}

	private removeSelected(): void {
		const p = this.page;
		if (!p || this.selected.size === 0) return;
		this.setPage(removeElements(p, this.selected));
		this.selected = new Set();
	}

	private duplicateSelected(): void {
		const p = this.page;
		if (!p || this.selected.size === 0) return;
		const r = duplicate(p, this.selected);
		this.setPage(r.page);
		this.selected = r.ids;
	}

	private nudge(dx: number, dy: number): void {
		const p = this.page;
		if (!p || this.selected.size === 0) return;
		this.setPage({ ...p, elements: p.elements.map((e) => (this.selected.has(e.id) ? place(e, e.x + dx, e.y + dy, p) : e)) });
	}

	private onKey = (ev: KeyboardEvent) => {
		// not while operating, typing in a field, or while the page (the IDE app) is hidden by the shell
		const t = ev.target;
		if (this.live || (t instanceof Element && t.closest("input, select, textarea")) || !this.isConnected || this.closest("[hidden]")) return;
		const k = ev.key;
		if ((k === "Delete" || k === "Backspace") && this.selected.size) {
			ev.preventDefault();
			this.removeSelected();
		} else if (ev.ctrlKey && k.toLowerCase() === "d") {
			ev.preventDefault();
			this.duplicateSelected();
		} else if (ev.ctrlKey && k.toLowerCase() === "z") {
			ev.preventDefault();
			this.undoStep(!ev.shiftKey);
		} else if (ev.ctrlKey && k.toLowerCase() === "y") {
			ev.preventDefault();
			this.undoStep(false);
		} else if (ev.ctrlKey && k.toLowerCase() === "a") {
			ev.preventDefault();
			this.selected = new Set(this.page?.elements.map((e) => e.id));
		} else if (k.startsWith("Arrow") && this.selected.size) {
			ev.preventDefault();
			const s = GRID * (ev.shiftKey ? 5 : 1);
			this.nudge(k === "ArrowLeft" ? -s : k === "ArrowRight" ? s : 0, k === "ArrowUp" ? -s : k === "ArrowDown" ? s : 0);
		} else if (k === "Escape") {
			this.selected = new Set();
		}
	};

	connectedCallback(): void {
		super.connectedCallback();
		window.addEventListener("keydown", this.onKey);
	}

	disconnectedCallback(): void {
		window.removeEventListener("keydown", this.onKey);
		super.disconnectedCallback();
	}

	// ---- variables -------------------------------------------------------------------------------------------
	private varChoices(): { path: string; type: string }[] {
		if (this.debug !== this.choicesFor) {
			this.choicesFor = this.debug;
			this.choices = this.debug ? scalarPaths(this.debug, 3, this.names ?? undefined) : [];
		}
		return this.choices;
	}

	/** Why a variable path cannot be bound (null: it can, or there is no build to check against). */
	private varProblem(path: string): string | null {
		const p = path.trim();
		if (!p || !this.debug) return null;
		const s = this.debug.resolve(p);
		if (!s) return "nicht im Programm (übersetzt?)";
		return s.type.k === "elem" || s.type.k === "enum" || s.type.k === "subrange" ? null : "keine einfache Variable";
	}

	// ---- rendering ---------------------------------------------------------------------------------------------
	render() {
		const p = this.page;
		return html`
			<div class="hmi-editor">
				${this.renderBar(p)}
				${this.renderPalette()}
				<div class="hmi-canvas" style=${p ? `--hmi-aspect:${p.width} / ${p.height}` : ""}>
					${p ? html`<hmi-view .page=${p} .device=${this.device} .mode=${this.live ? "run" : "edit"} .selected=${this.selected} .tick=${this.tick}
						@select=${(e: CustomEvent<Set<string>>) => (this.selected = e.detail)}
						@pagechange=${(e: CustomEvent<HmiPage>) => this.setPage(e.detail)}
						@addelement=${(e: CustomEvent<{ kind: HmiKind; x: number; y: number }>) => this.addElement(e.detail.kind, e.detail.x, e.detail.y)}></hmi-view>`
						: html`<div class="hmi-empty">Noch keine Bedienseite. <button class="hmi-small-btn" @click=${() => this.addPage()}>＋ Seite anlegen</button></div>`}
				</div>
				${this.renderProps(p)}
				<datalist id="hmi-vars">${this.varChoices().map((c) => html`<option value=${c.path}>${c.type}</option>`)}</datalist>
			</div>`;
	}

	private renderBar(p: HmiPage | undefined) {
		const one = this.selected.size > 0;
		return html`
			<div class="hmi-ed-bar">
				<div class="hmi-tabs" role="tablist">
					${this.pages.map((x) => html`<button class="hmi-tab ${x.id === p?.id ? "hmi-tab-active" : ""}" role="tab" @click=${() => this.selectPage(x.id)}>${x.name}</button>`)}
					<button class="hmi-small-btn" title="Neue Seite" @click=${() => this.addPage()}>＋</button>
				</div>
				<span class="hmi-sep"></span>
				<button class="hmi-small-btn" title="Rückgängig (Strg+Z)" ?disabled=${this.undo.length === 0} @click=${() => this.undoStep(true)}>↶</button>
				<button class="hmi-small-btn" title="Wiederholen (Strg+Y)" ?disabled=${this.redo.length === 0} @click=${() => this.undoStep(false)}>↷</button>
				<span class="hmi-sep"></span>
				<button class="hmi-small-btn" title="Duplizieren (Strg+D)" ?disabled=${!one || this.live} @click=${() => this.duplicateSelected()}>⧉</button>
				<button class="hmi-small-btn" title="Nach vorn" ?disabled=${!one || this.live} @click=${() => p && this.setPage(reorder(p, this.selected, true))}>⬆</button>
				<button class="hmi-small-btn" title="Nach hinten" ?disabled=${!one || this.live} @click=${() => p && this.setPage(reorder(p, this.selected, false))}>⬇</button>
				<button class="hmi-small-btn" title="Löschen (Entf)" ?disabled=${!one || this.live} @click=${() => this.removeSelected()}>🗑</button>
				<span class="hmi-sep"></span>
				<button class="hmi-small-btn ${this.live ? "on" : ""}" title="Die Seite bedienen wie zur Laufzeit (mit dem Gerät der IDE)"
					@click=${() => { this.live = !this.live; this.selected = new Set(); }}>${this.live ? "✎ Bearbeiten" : "▶ Bedienen"}</button>
				<a class="hmi-small-btn" href="/hmi" title="Die Laufzeit-Oberfläche (eigene Seite, auch auf einem Bedien-PC)"
					@click=${(e: Event) => { e.preventDefault(); history.pushState(null, "", "/hmi"); dispatchEvent(new PopStateEvent("popstate")); }}>Laufzeit öffnen ↗</a>
			</div>`;
	}

	private renderPalette() {
		const groups = ["Bedienen", "Anzeigen", "Gestaltung"] as const;
		return html`
			<div class="hmi-palette" aria-label="Elemente">
				${groups.map((g) => html`<h5>${g}</h5>
					${HMI_KINDS.filter((k) => k.group === g).map((k) => html`
						<button class="hmi-pal-item" draggable="true" title="${k.name}: ${k.varHint || "ohne Variable"} - auf die Seite ziehen oder klicken"
							?disabled=${this.live || !this.page}
							@dragstart=${(e: DragEvent) => { e.dataTransfer?.setData(HMI_KIND_MIME, k.kind); if (e.dataTransfer) e.dataTransfer.effectAllowed = "copy"; }}
							@click=${() => this.addElement(k.kind)}>
							<span>${k.icon}</span><span>${k.name}</span>
						</button>`)}`)}
			</div>`;
	}

	private renderProps(p: HmiPage | undefined) {
		if (!p) return html`<div class="hmi-props"></div>`;
		const sel = p.elements.filter((e) => this.selected.has(e.id));
		if (sel.length === 0) return this.renderPageProps(p);
		if (sel.length > 1) {
			return html`<div class="hmi-props"><h4>${sel.length} Elemente</h4>
				<p class="hmi-hint">Verschieben mit der Maus oder den Pfeiltasten (Umschalt: 50 px), Entf löscht, Strg+D dupliziert.</p></div>`;
		}
		const e = sel[0];
		const info = KIND_INFO[e.kind];
		return html`
			<div class="hmi-props">
				<h4>${info.icon} ${info.name}</h4>
				${info.varHint ? html`<p class="hmi-hint">${info.varHint}</p>` : nothing}
				${info.props.map((prop) => this.renderProp(e, prop))}
				${e.kind === "trend" ? this.renderPens(e) : nothing}
				<label>Position und Größe [px]</label>
				<div style="display:grid;grid-template-columns:repeat(4,1fr);gap:4px">
					${(["x", "y", "w", "h"] as const).map((k) => html`<input type="number" title=${k} .value=${String(e[k])} step=${GRID}
						@change=${(ev: Event) => {
							const v = Number((ev.target as HTMLInputElement).value);
							if (Number.isFinite(v)) this.change(e, { [k]: Math.max(k === "w" || k === "h" ? 20 : 0, Math.round(v)) });
						}} />`)}
				</div>
			</div>`;
	}

	private renderProp(e: HmiElement, prop: HmiProp) {
		const v = e[prop.key];
		const set = (value: unknown) => this.change(e, { [prop.key]: value } as Partial<HmiElement>);
		const label = html`<label>${prop.label}</label>`;
		const hint = prop.hint ? html`<p class="hmi-hint">${prop.hint}</p>` : nothing;
		switch (prop.type) {
			case "bool":
				return html`<label class="hmi-check"><input type="checkbox" .checked=${!!v} @change=${(ev: Event) => set((ev.target as HTMLInputElement).checked)} /> ${prop.label}</label>`;
			case "number":
				return html`${label}<input type="number" step="any" .value=${v === undefined || v === null ? "" : String(v)}
					@change=${(ev: Event) => {
						const t = (ev.target as HTMLInputElement).value.trim();
						set(t === "" ? undefined : Number(t));
					}} />${hint}`;
			case "var": {
				const problem = this.varProblem(String(v ?? ""));
				return html`${label}<input type="text" list="hmi-vars" spellcheck="false" placeholder="z. B. PLC_PRG.Sollwert" .value=${String(v ?? "")}
					@change=${(ev: Event) => set((ev.target as HTMLInputElement).value.trim())} />
					${problem ? html`<div class="hmi-bad">⚠ ${problem}</div>` : nothing}${hint}`;
			}
			case "color":
				return html`${label}<input type="color" .value=${String(v || "#2f6fde")} @input=${(ev: Event) => set((ev.target as HTMLInputElement).value)} />`;
			case "led":
				return html`${label}<select @change=${(ev: Event) => set((ev.target as HTMLSelectElement).value)}>
					${LED_COLORS.map((c) => html`<option value=${c} ?selected=${c === v}>${LED_COLOR_NAMES[c]}</option>`)}</select>`;
			case "prio":
				return html`${label}<select @change=${(ev: Event) => set(Number((ev.target as HTMLSelectElement).value))}>
					${Object.entries(PRIO_NAMES).map(([n, name]) => html`<option value=${n} ?selected=${Number(n) === v}>${n} ${name}</option>`)}</select>`;
			default:
				return html`${label}<input type="text" .value=${String(v ?? "")} @change=${(ev: Event) => set((ev.target as HTMLInputElement).value)} />${hint}`;
		}
	}

	private renderPens(e: HmiElement) {
		const pens = e.pens ?? [];
		const setPens = (next: typeof pens) => this.change(e, { pens: next });
		return html`
			<label>Kurven (bis ${MAX_PENS})</label>
			${pens.map((pen, i) => html`
				<div class="hmi-pen">
					<input type="color" .value=${pen.color} @input=${(ev: Event) => setPens(pens.map((x, j) => (j === i ? { ...x, color: (ev.target as HTMLInputElement).value } : x)))} />
					<input type="text" list="hmi-vars" spellcheck="false" .value=${pen.var} placeholder="Variable"
						@change=${(ev: Event) => setPens(pens.map((x, j) => (j === i ? { ...x, var: (ev.target as HTMLInputElement).value.trim() } : x)))} />
					<button class="hmi-small-btn" title="Kurve entfernen" @click=${() => setPens(pens.filter((_, j) => j !== i))}>✕</button>
				</div>
				<div class="hmi-pen"><span></span><input type="text" .value=${pen.label ?? ""} placeholder="Legende (sonst die Variable)"
					@change=${(ev: Event) => setPens(pens.map((x, j) => (j === i ? { ...x, label: (ev.target as HTMLInputElement).value } : x)))} /><span></span></div>
				${this.varProblem(pen.var) ? html`<div class="hmi-bad">⚠ ${pen.var}: ${this.varProblem(pen.var)}</div>` : nothing}`)}
			<button class="hmi-small-btn" ?disabled=${pens.length >= MAX_PENS}
				@click=${() => setPens([...pens, { var: "", color: PEN_COLORS[pens.length % PEN_COLORS.length] }])}>＋ Kurve</button>
			<p class="hmi-hint">BOOL-Kurven werden als Stufen im unteren Fünftel gezeichnet.</p>`;
	}

	private renderPageProps(p: HmiPage) {
		const unbound = p.elements.filter((e) => e.kind !== "label" && !e.var.trim() && !(e.pens ?? []).some((x) => x.var.trim())).length;
		const broken = p.elements.filter((e) => [e.var, e.ackVar ?? "", ...(e.pens ?? []).map((x) => x.var)].some((v) => this.varProblem(v))).length;
		return html`
			<div class="hmi-props">
				<h4>Seite</h4>
				<label>Name</label>
				<input type="text" .value=${p.name} @change=${(ev: Event) => this.setPage({ ...p, name: (ev.target as HTMLInputElement).value.trim() || p.name })} />
				<label>Größe [px]</label>
				<div style="display:grid;grid-template-columns:1fr 1fr;gap:4px">
					<input type="number" min="320" step="10" .value=${String(p.width)} @change=${(ev: Event) => this.setPage({ ...p, width: Math.max(320, Number((ev.target as HTMLInputElement).value) || p.width) })} />
					<input type="number" min="200" step="10" .value=${String(p.height)} @change=${(ev: Event) => this.setPage({ ...p, height: Math.max(200, Number((ev.target as HTMLInputElement).value) || p.height) })} />
				</div>
				<p class="hmi-hint">Die Seite wird zur Laufzeit auf das Fenster skaliert (Seitenverhältnis bleibt).</p>
				<div style="display:flex;gap:4px;margin-top:8px">
					<button class="hmi-small-btn" title="Seite nach links" @click=${() => this.movePage(-1)}>◀</button>
					<button class="hmi-small-btn" title="Seite nach rechts" @click=${() => this.movePage(1)}>▶</button>
					<button class="hmi-small-btn" @click=${() => void this.removePage()}>🗑 Seite löschen</button>
				</div>
				<label>Prüfung</label>
				<p class="hmi-hint">${p.elements.length} Elemente${unbound ? `, ${unbound} ohne Variable` : ""}${broken ? html`, <span class="hmi-bad">${broken} mit unbekannter Variable</span>` : ""}${this.debug ? "" : " (keine Übersetzung zum Prüfen)"}</p>
				<label>Bedienung</label>
				<p class="hmi-hint">Elemente aus der Palette ziehen oder anklicken. Auswählen per Klick, Umschalt+Klick oder Rahmen; Größe am Griff unten rechts.
				Entf löscht, Strg+D dupliziert, Strg+Z/Y rückgängig/wiederholen, Pfeiltasten verschieben.</p>
			</div>`;
	}
}

declare global {
	interface HTMLElementTagNameMap {
		"hmi-editor": HmiEditor;
	}
}
