// The SFC editor (docs/plc-iec61131.md 5.4, 10.2): the chart laid out by HTML/CSS from its tree - a
// sequence is a column, every row a symmetric grid around the column's centre line (the step or the
// transition in the middle, the action block or the condition on the right), a branch a row of
// columns between two bars (alternative: single, parallel: double) drawn by CSS. No coordinates: the
// structure is the layout. Names, conditions and actions are edited in place; the toolbar inserts
// and removes structure at the selected element. Online: active steps highlighted with their time.
import { LitElement, html, nothing, type TemplateResult } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { sfcId, sfcWalk, type Qualifier, type SfcItem, type SfcPou, type SfcProblem } from "./sfc.ts";

const QUALIFIERS: { q: Qualifier; title: string }[] = [
	{ q: "N", title: "N: solange der Schritt aktiv ist" },
	{ q: "S", title: "S: setzen (bleibt gesetzt)" },
	{ q: "R", title: "R: rücksetzen" },
	{ q: "P", title: "P: einmal beim Aktivieren" },
	{ q: "D", title: "D: zeitverzögert - nach der Zeit, solange der Schritt aktiv ist" },
];

@customElement("plc-sfc-editor")
export class SfcEditor extends LitElement {
	@property({ attribute: false }) pou!: SfcPou;
	@property({ attribute: false }) problems: SfcProblem[] = [];
	/** Online: the active steps and their times ("1.2 s"). */
	@property({ attribute: false }) active: Map<string, string> | null = null;
	@state() private sel: string | null = null;

	protected createRenderRoot() {
		return this;
	}

	private changed(): void {
		this.pou = { ...this.pou, chart: [...this.pou.chart] };
		this.dispatchEvent(new CustomEvent("change", { detail: this.pou, bubbles: true }));
	}

	/** The selected item, its sequence and index there. */
	private where(): { item: SfcItem; seq: SfcItem[]; index: number } | null {
		if (!this.sel) return null;
		for (const w of sfcWalk(this.pou.chart)) if (w.item.id === this.sel) return w;
		return null;
	}

	private stepNames(): string[] {
		return [...sfcWalk(this.pou.chart)].map((w) => w.item).filter((i) => i.kind === "step").map((s) => (s as { name: string }).name);
	}

	private freshName(base: string): string {
		const used = new Set(this.stepNames().map((n) => n.toUpperCase()));
		let i = 1;
		while (used.has(`${base}${i}`.toUpperCase())) i++;
		return `${base}${i}`;
	}

	private step = (name?: string): SfcItem => ({ kind: "step", id: sfcId(), name: name ?? this.freshName("Schritt"), actions: [] });
	private trans = (condition = "TRUE"): SfcItem => ({ kind: "trans", id: sfcId(), condition });

	// ---- structure ------------------------------------------------------------------------------------
	/** After a step: a transition and a step; after a transition: a step and a transition. */
	private insertAfter(): void {
		const w = this.where();
		if (!w) return;
		const add = w.item.kind === "step" || w.item.kind === "branch" && w.item.type === "par" ? [this.trans(), this.step()] : [this.step(), this.trans()];
		if (w.item.kind === "jump") return;
		w.seq.splice(w.index + 1, 0, ...add);
		this.sel = add[0].id;
		this.changed();
	}

	/** An alternative branch after a step (two paths, each transition - step - transition), then a step. */
	private insertAlt(): void {
		const w = this.where();
		if (w?.item.kind !== "step") return;
		const branch: SfcItem = {
			kind: "branch", id: sfcId(), type: "alt",
			branches: [[this.trans(), this.step(), this.trans()], [this.trans("FALSE"), this.step(this.freshName("Alt")), this.trans()]],
		};
		// unique names for the second path's step: freshName sees the first one only after it is in the chart
		w.seq.splice(w.index + 1, 0, branch, this.step(this.freshName("Weiter")));
		this.fixNames(branch);
		this.sel = branch.id;
		this.changed();
	}

	/** A parallel branch after a transition (two paths of one step each), then the joining transition. */
	private insertPar(): void {
		const w = this.where();
		if (w?.item.kind !== "trans") return;
		const branch: SfcItem = { kind: "branch", id: sfcId(), type: "par", branches: [[this.step()], [this.step()]] };
		w.seq.splice(w.index + 1, 0, branch, this.trans());
		this.fixNames(branch);
		this.sel = branch.id;
		this.changed();
	}

	/** Steps created in one go got the same fresh name: number them apart. */
	private fixNames(branch: SfcItem): void {
		const used = new Set<string>();
		for (const w of sfcWalk(this.pou.chart)) if (w.item.kind === "step" && !branchContains(branch, w.item.id)) used.add(w.item.name.toUpperCase());
		for (const w of sfcWalk(branch.kind === "branch" ? branch.branches.flat() : [])) {
			if (w.item.kind !== "step") continue;
			let name = w.item.name;
			for (let i = 2; used.has(name.toUpperCase()); i++) name = `${w.item.name.replace(/\d+$/, "")}${i}`;
			w.item.name = name;
			used.add(name.toUpperCase());
		}
	}

	private addPath(): void {
		const w = this.where();
		if (w?.item.kind !== "branch") return;
		w.item.branches.push(w.item.type === "alt" ? [this.trans("FALSE"), this.step(), this.trans()] : [this.step()]);
		this.changed();
	}

	private insertJump(): void {
		const w = this.where();
		if (w?.item.kind !== "trans" || w.index !== w.seq.length - 1) return;
		w.seq.push({ kind: "jump", id: sfcId(), target: this.stepNames()[0] ?? "" });
		this.changed();
	}

	private removeSelected(): void {
		const w = this.where();
		if (!w || (w.seq === this.pou.chart && w.index === 0)) return;  // the initial step stays
		// a step goes with the transition after it, a transition with the step after it (the chain stays alternating)
		const next = w.seq[w.index + 1];
		const pair = (w.item.kind === "step" && next?.kind === "trans") || (w.item.kind === "trans" && next?.kind === "step");
		w.seq.splice(w.index, pair ? 2 : 1);
		// a branch path that became empty goes too
		for (const b of sfcWalk(this.pou.chart)) {
			if (b.item.kind === "branch") b.item.branches = b.item.branches.filter((p) => p.length > 0);
		}
		this.sel = null;
		this.changed();
	}

	// ---- rendering ----------------------------------------------------------------------------------
	private cls(it: SfcItem): string {
		const bad = this.problems.some((p) => p.item === it.id);
		return `${this.sel === it.id ? "sfc-sel" : ""} ${bad ? "sfc-bad" : ""}`;
	}

	private select(e: Event, id: string): void {
		e.stopPropagation();
		this.sel = id;
	}

	private renderStep(it: SfcItem & { kind: "step" }, initial: boolean): TemplateResult {
		const time = this.active?.get(it.name.toUpperCase());
		return html`
			<div class="sfc-row" @click=${(e: Event) => this.select(e, it.id)}>
				<div></div>
				<div class="sfc-step ${initial ? "sfc-initial" : ""} ${time !== undefined ? "sfc-active" : ""} ${this.cls(it)}">
					<input class="sfc-name" .value=${it.name} size=${Math.max(4, it.name.length)} title="Schrittname"
						@change=${(e: Event) => { it.name = (e.target as HTMLInputElement).value.trim(); this.changed(); }} />
					${time !== undefined ? html`<span class="sfc-time">${time}</span>` : nothing}
				</div>
				<div class="sfc-actions-cell">
					${it.actions.length || this.sel === it.id ? html`
						<table class="sfc-actions">
							${it.actions.map((a, i) => html`<tr>
								<td><select title=${QUALIFIERS.find((q) => q.q === a.qualifier)?.title ?? ""}
									@change=${(e: Event) => {
										a.qualifier = (e.target as HTMLSelectElement).value as Qualifier;
										if (a.qualifier === "D" && !a.time) a.time = "T#1s";
										this.changed();
									}}>
									${QUALIFIERS.map((q) => html`<option ?selected=${q.q === a.qualifier} title=${q.title}>${q.q}</option>`)}</select>${a.qualifier === "D" ? html`<input class="sfc-time-input" .value=${a.time ?? ""} size=${Math.max(5, (a.time ?? "").length)} title="Verzögerung nach dem Aktivieren des Schritts (TIME, z.B. T#2s)" placeholder="T#1s"
									@change=${(e: Event) => { a.time = (e.target as HTMLInputElement).value.trim(); this.changed(); }} />` : nothing}</td>
								<td><input .value=${a.body} placeholder="Variable oder ST" size=${Math.max(10, a.body.length)}
									@change=${(e: Event) => { a.body = (e.target as HTMLInputElement).value; this.changed(); }} /></td>
								<td><button class="ide-icon-btn" title="Aktion löschen" @click=${() => { it.actions.splice(i, 1); this.changed(); }}>×</button></td>
							</tr>`)}
							${this.sel === it.id ? html`<tr><td colspan="3"><button class="sfc-add-action"
								@click=${() => { it.actions.push({ qualifier: "N", body: "" }); this.changed(); }}>＋ Aktion</button></td></tr>` : nothing}
						</table>` : nothing}
				</div>
			</div>`;
	}

	private renderTrans(it: SfcItem & { kind: "trans" }): TemplateResult {
		return html`
			<div class="sfc-row" @click=${(e: Event) => this.select(e, it.id)}>
				<div></div>
				<div class="sfc-trans ${this.cls(it)}"><span class="sfc-tick"></span></div>
				<div class="sfc-cond-cell">
					<input class="sfc-cond" .value=${it.condition} size=${Math.max(6, it.condition.length)} title="Transitionsbedingung (ST-Ausdruck)"
						@change=${(e: Event) => { it.condition = (e.target as HTMLInputElement).value; this.changed(); }} />
				</div>
			</div>`;
	}

	private renderJump(it: SfcItem & { kind: "jump" }): TemplateResult {
		return html`
			<div class="sfc-row" @click=${(e: Event) => this.select(e, it.id)}>
				<div></div>
				<div class="sfc-jump ${this.cls(it)}" title="Sprung">⬇</div>
				<div class="sfc-cond-cell">
					<select class="sfc-jump-target" @change=${(e: Event) => { it.target = (e.target as HTMLSelectElement).value; this.changed(); }}>
						${this.stepNames().map((n) => html`<option ?selected=${n === it.target}>${n}</option>`)}
					</select>
				</div>
			</div>`;
	}

	private renderBranch(it: SfcItem & { kind: "branch" }): TemplateResult {
		return html`
			<div class="sfc-branch sfc-${it.type} ${this.cls(it)}" @click=${(e: Event) => this.select(e, it.id)}
				title=${it.type === "alt" ? "Alternativ-Verzweigung: der linkeste Pfad mit wahrer Transition" : "Parallel-Verzweigung: alle Pfade, die Transition danach wartet auf alle"}>
				<div class="sfc-cols">
					${it.branches.map((b) => html`<div class="sfc-col">${this.renderSeq(b, false)}<div class="sfc-fill"></div></div>`)}
				</div>
			</div>`;
	}

	private renderSeq(seq: SfcItem[], main: boolean): TemplateResult {
		return html`<div class="sfc-seq">${seq.map((it, i) => {
			switch (it.kind) {
				case "step": return this.renderStep(it, main && i === 0);
				case "trans": return this.renderTrans(it);
				case "branch": return this.renderBranch(it);
				case "jump": return this.renderJump(it);
			}
		})}</div>`;
	}

	render() {
		const w = this.where();
		const k = w?.item.kind;
		const lastTrans = k === "trans" && w!.index === w!.seq.length - 1;
		return html`
			<div class="sfc">
				<div class="sfc-toolbar">
					<button ?disabled=${!w || k === "jump"} @click=${() => this.insertAfter()}
						title="Nach dem ausgewählten Element: Schritt und Transition einfügen">＋ Schritt/Transition</button>
					<button ?disabled=${k !== "step"} @click=${() => this.insertAlt()} title="Nach dem ausgewählten Schritt">＋ Alternativ-Zweig</button>
					<button ?disabled=${k !== "trans"} @click=${() => this.insertPar()} title="Nach der ausgewählten Transition">＋ Parallel-Zweig</button>
					<button ?disabled=${k !== "branch"} @click=${() => this.addPath()} title="Der ausgewählten Verzweigung einen Pfad hinzufügen">＋ Pfad</button>
					<button ?disabled=${!lastTrans} @click=${() => this.insertJump()} title="Nach der letzten Transition eines Pfads">＋ Sprung</button>
					<button ?disabled=${!w || (w.seq === this.pou.chart && w.index === 0)} @click=${() => this.removeSelected()} title="Das ausgewählte Element löschen">🗑 Löschen</button>
				</div>
				<div class="sfc-canvas" @click=${() => (this.sel = null)}>${this.renderSeq(this.pou.chart, true)}</div>
				${this.problems.length ? html`<ul class="fbd-problems">${this.problems.map((p) => html`<li>${p.message}</li>`)}</ul>` : nothing}
			</div>`;
	}
}

function branchContains(branch: SfcItem, id: string): boolean {
	if (branch.kind !== "branch") return false;
	for (const w of sfcWalk(branch.branches.flat())) if (w.item.id === id) return true;
	return false;
}
