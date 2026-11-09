// The FBD editor (docs/plc-iec61131.md 5.3, 10.2): one SVG per network. Boxes, input and output
// variables are moved by dragging; a wire is drawn from an output pin (right) to an input pin (left)
// and replaces the wire that pin had; a click selects a node or a wire, Delete removes it. The
// palette adds boxes; the side panel edits the selected node (instance name, expression, target).
// Online: the values of the variables and instance outputs at the pins, a TRUE wire in green.
import { LitElement, html, nothing, svg } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import { BOXES, catalog, networkTypes, newId, pinKey, pinsOf, type FbdNetwork, type FbdNode, type BoxCatalog, type FbdPou, type FbdProblem, type NetworkTypes, type PinRef, type TypeClass } from "./fbd.ts";

const BOX_W = 112;
const HEAD = 22;
const PIN_DY = 20;
const VAR_H = 24;
const SNAP = 10;

type Selection = { net: string; node: string } | { net: string; wire: number } | null;
interface NearPin {
	net: string;
	node: string;
	pin: string;
	side: "in" | "out";
}
/** How close the pointer must come to a pin to catch it (px). */
const PIN_REACH = 14;

const TYPE_NAMES: Record<TypeClass, string> = {
	bool: "BOOL", int: "Ganzzahl", real: "Gleitkomma", time: "Zeit/Datum", string: "Zeichenkette", any: "noch unbestimmt",
};

function varWidth(text: string): number {
	return Math.max(60, text.length * 7.4 + 18);
}

function size(n: FbdNode, cat: BoxCatalog): { w: number; h: number } {
	if (n.kind === "input") return { w: varWidth(n.expr || "?"), h: VAR_H };
	if (n.kind === "output") return { w: varWidth(n.target || "?"), h: VAR_H };
	const p = pinsOf(n, cat);
	return { w: BOX_W, h: HEAD + Math.max(p.inputs.length, p.outputs.length, 1) * PIN_DY + 6 };
}

/** Where a pin is: inputs on the left edge, outputs on the right. */
function pinAt(n: FbdNode, pin: string, side: "in" | "out", cat: BoxCatalog): { x: number; y: number } {
	const { w } = size(n, cat);
	if (n.kind === "input") return { x: n.x + w, y: n.y + VAR_H / 2 };
	if (n.kind === "output") return { x: n.x, y: n.y + VAR_H / 2 };
	const p = pinsOf(n, cat);
	const i = Math.max(0, (side === "in" ? p.inputs : p.outputs).indexOf(pin));
	return { x: side === "in" ? n.x : n.x + w, y: n.y + HEAD + 12 + i * PIN_DY };
}

/** Orthogonal route: out horizontally, down/up shortly before the target, in horizontally. `lane`
 *  (the target pin's index) gives every input of a box its own vertical, so wires into one box do
 *  not run on top of each other. */
function route(a: { x: number; y: number }, b: { x: number; y: number }, lane = 0): string {
	if (b.x - a.x >= 24) {
		let mid = b.x - 14 - lane * 9;
		if (mid < a.x + 10) mid = Math.round((a.x + b.x) / 2);
		return `M ${a.x} ${a.y} H ${mid} V ${b.y} H ${b.x}`;
	}
	// backwards (feedback): around below
	const below = Math.max(a.y, b.y) + 30;
	return `M ${a.x} ${a.y} H ${a.x + 12} V ${below} H ${b.x - 12} V ${b.y} H ${b.x}`;
}

@customElement("plc-fbd-editor")
export class FbdEditor extends LitElement {
	@property({ attribute: false }) pou!: FbdPou;
	@property({ attribute: false }) problems: FbdProblem[] = [];
	/** Online values: a variable path or expression -> its formatted value (null: not a readable variable). */
	@property({ attribute: false }) lookup: ((expr: string) => string | null) | null = null;
	/** The elementary type of a variable (for the pin colours); null: not known. */
	@property({ attribute: false }) varType: ((name: string) => string | null) | null = null;
	@state() private sel: Selection = null;
	@state() private drag: { net: string; node: string; dx: number; dy: number; moved: boolean } | null = null;
	/** A wire being drawn: from an output pin (to an input) or from an input pin (to an output). */
	@state() private wiring: { net: string; from: NearPin; x: number; y: number } | null = null;
	/** The pin within reach of the pointer: the cursor changes, the pin shows a ring, a wire snaps to it. */
	@state() private near: NearPin | null = null;

	protected createRenderRoot() {
		return this;
	}

	private changed(): void {
		this.pou = { ...this.pou, networks: this.pou.networks.map((n) => ({ ...n })) };
		this.dispatchEvent(new CustomEvent("change", { detail: this.pou, bubbles: true }));
	}

	private net(id: string): FbdNetwork {
		return this.pou.networks.find((n) => n.id === id)!;
	}

	private svgPoint(e: PointerEvent): { x: number; y: number } {
		const svgEl = (e.currentTarget as Element).closest("svg")!;
		const r = svgEl.getBoundingClientRect();
		return { x: e.clientX - r.left, y: e.clientY - r.top };
	}

	/** The boxes this POU can use: the standard ones and its extra ones (project, FCU library). */
	private get cat(): BoxCatalog {
		return catalog(this.pou.extra);
	}

	/** A new instance's name: TON -> Ton1, FB_Ejector -> Ejector1, FCU_StepperHome -> FCU_StepperHome1. */
	private instanceName(type: string, n: number): string {
		const base = /^FB_/i.test(type) ? type.slice(3) : type === type.toUpperCase() ? `${type.charAt(0)}${type.slice(1).toLowerCase()}` : type;
		return `${base}${n}`;
	}

	// ---- pointer: drag nodes, draw wires, snap to pins -------------------------------------------------
	/** The pin nearest to `p` within PIN_REACH (of one side, not on `exclude`). */
	private nearestPin(netId: string, p: { x: number; y: number }, side?: "in" | "out", exclude?: string): NearPin | null {
		let best: NearPin | null = null;
		let bestD = PIN_REACH * PIN_REACH;
		for (const n of this.net(netId).nodes) {
			if (n.id === exclude) continue;
			const pins = pinsOf(n, this.cat);
			for (const s of ["in", "out"] as const) {
				if (side && side !== s) continue;
				for (const pin of s === "in" ? pins.inputs : pins.outputs) {
					const at = pinAt(n, pin, s, this.cat);
					const d = (at.x - p.x) ** 2 + (at.y - p.y) ** 2;
					if (d < bestD) {
						bestD = d;
						best = { net: netId, node: n.id, pin, side: s };
					}
				}
			}
		}
		return best;
	}

	private pinPos(w: NearPin): { x: number; y: number } {
		const n = this.net(w.net).nodes.find((x) => x.id === w.node)!;
		return pinAt(n, w.pin, w.side, this.cat);
	}

	private startWire(e: PointerEvent, from: NearPin): void {
		const p = this.svgPoint(e);
		this.wiring = { net: from.net, from, x: p.x, y: p.y };
		this.near = null;
		(e.currentTarget as Element).closest("svg")!.setPointerCapture(e.pointerId);
	}

	/** On a node: a pin within reach starts a wire, elsewhere the node is selected and dragged. */
	private down(e: PointerEvent, net: string, node: FbdNode): void {
		if (e.button !== 0) return;
		e.stopPropagation();
		const p = this.svgPoint(e);
		const pin = this.nearestPin(net, p);
		if (pin) return this.startWire(e, pin);
		this.sel = { net, node: node.id };
		this.drag = { net, node: node.id, dx: p.x - node.x, dy: p.y - node.y, moved: false };
		(e.currentTarget as Element).closest("svg")!.setPointerCapture(e.pointerId);
	}

	/** On the background: a pin within reach (its circle may stick out of the node) or deselect. */
	private backgroundDown(e: PointerEvent, net: string): void {
		if (e.button !== 0) return;
		const pin = this.nearestPin(net, this.svgPoint(e));
		if (pin) this.startWire(e, pin);
		else this.sel = null;
	}

	private move(e: PointerEvent, net: string): void {
		const p = this.svgPoint(e);
		if (this.drag) {
			const n = this.net(this.drag.net).nodes.find((x) => x.id === this.drag!.node);
			if (!n) return;
			n.x = Math.max(0, Math.round((p.x - this.drag.dx) / SNAP) * SNAP);
			n.y = Math.max(0, Math.round((p.y - this.drag.dy) / SNAP) * SNAP);
			this.drag.moved = true;
			this.requestUpdate();
		} else if (this.wiring) {
			// the end snaps to the nearest pin of the other side
			const w = this.wiring;
			const target = this.nearestPin(w.net, p, w.from.side === "out" ? "in" : "out", w.from.node);
			this.near = target;
			const at = target ? this.pinPos(target) : p;
			this.wiring = { ...w, x: at.x, y: at.y };
		} else {
			const pin = this.nearestPin(net, p);
			if (pin?.node !== this.near?.node || pin?.pin !== this.near?.pin || pin?.side !== this.near?.side || pin?.net !== this.near?.net) this.near = pin;
		}
	}

	private up(): void {
		if (this.drag) {
			if (this.drag.moved) this.changed();
			this.drag = null;
		} else if (this.wiring) {
			const w = this.wiring;
			const target = this.near;
			this.wiring = null;
			this.near = null;
			if (!target || target.net !== w.net || target.side === w.from.side || target.node === w.from.node) return;
			const out = w.from.side === "out" ? w.from : target;
			const inp = w.from.side === "out" ? target : w.from;
			const net = this.net(w.net);
			net.wires = net.wires.filter((x) => !(x.to.node === inp.node && x.to.pin === inp.pin));  // one source per input
			net.wires.push({ from: { node: out.node, pin: out.pin }, to: { node: inp.node, pin: inp.pin } });
			this.changed();
		}
	}

	private key(e: KeyboardEvent): void {
		if ((e.key !== "Delete" && e.key !== "Backspace") || !this.sel || (e.target as HTMLElement).tagName === "INPUT") return;
		e.preventDefault();
		this.deleteSelected();
	}

	private deleteSelected(): void {
		const s = this.sel;
		if (!s) return;
		const net = this.net(s.net);
		if ("node" in s) {
			net.nodes = net.nodes.filter((n) => n.id !== s.node);
			net.wires = net.wires.filter((w) => w.from.node !== s.node && w.to.node !== s.node);
		} else {
			net.wires = net.wires.filter((_, i) => i !== s.wire);
		}
		this.sel = null;
		this.changed();
	}

	// ---- palette and networks -----------------------------------------------------------------------
	private targetNet(): FbdNetwork {
		return (this.sel && this.net(this.sel.net)) || this.pou.networks[this.pou.networks.length - 1];
	}

	private add(kind: string): void {
		if (!kind) return;
		if (!this.pou.networks.length) this.addNetwork();
		const net = this.targetNet();
		const bottom = Math.max(10, ...net.nodes.map((n) => n.y + size(n, this.cat).h + 20));
		const id = newId(net);
		let node: FbdNode;
		if (kind === "@input") node = { kind: "input", id, x: 20, y: bottom, expr: "Variable" };
		else if (kind === "@output") node = { kind: "output", id, x: 520, y: bottom, target: "Variable" };
		else {
			const d = this.cat(kind)!;
			const n = net.nodes.filter((x) => x.kind === "box" && x.type === kind).length + 1;
			node = { kind: "box", id, x: 200, y: bottom, type: d.type, ...(d.kind === "fb" ? { instance: this.instanceName(d.type, n) } : {}) };
		}
		net.nodes.push(node);
		this.sel = { net: net.id, node: id };
		this.changed();
	}

	private addNetwork(): void {
		const id = `net${Date.now().toString(36)}`;
		this.pou.networks.push({ id, title: `Netzwerk ${this.pou.networks.length + 1}`, nodes: [], wires: [] });
		this.sel = null;
		this.changed();
	}

	private removeNetwork(id: string): void {
		if (!confirm("Das Netzwerk löschen?")) return;
		this.pou.networks = this.pou.networks.filter((n) => n.id !== id);
		this.sel = null;
		this.changed();
	}

	// ---- rendering ----------------------------------------------------------------------------------
	private renderNode(net: FbdNetwork, n: FbdNode, types: NetworkTypes) {
		const { w, h } = size(n, this.cat);
		const selected = this.sel && "node" in this.sel && this.sel.net === net.id && this.sel.node === n.id;
		const bad = this.problems.some((p) => p.network === net.id && p.node === n.id);
		const cls = `fbd-node fbd-${n.kind} ${selected ? "fbd-sel" : ""} ${bad ? "fbd-bad" : ""}`;
		const pin = (name: string, side: "in" | "out") => {
			const at = pinAt(n, name, side, this.cat);
			const t = types.pins.get(pinKey(n.id, side, name)) ?? "any";
			return svg`<circle class="fbd-pin fbd-pin-${side} fbd-t-${t}" cx=${at.x} cy=${at.y} r="5">
				<title>${name}: ${TYPE_NAMES[t]} - ziehen, um zu verbinden</title></circle>`;
		};
		const value = (expr: string, x: number, y: number, anchor: "start" | "end") => {
			const v = this.lookup?.(expr);
			return v === null || v === undefined ? nothing : svg`<text class="fbd-value" x=${x} y=${y} text-anchor=${anchor}>${v}</text>`;
		};
		if (n.kind === "input" || n.kind === "output") {
			const text = n.kind === "input" ? n.expr : n.target;
			return svg`<g class=${cls} @pointerdown=${(e: PointerEvent) => this.down(e, net.id, n)}>
				<rect x=${n.x} y=${n.y} width=${w} height=${h} rx="3" />
				<text class="fbd-var-text" x=${n.x + w / 2} y=${n.y + 16} text-anchor="middle">${text || "?"}</text>
				${n.kind === "input" ? value(n.expr, n.x + w / 2, n.y - 4, "start") : value(n.target, n.x + w / 2, n.y - 4, "start")}
				${n.kind === "input" ? pin("OUT", "out") : pin("IN", "in")}
			</g>`;
		}
		const d = this.cat(n.type);
		const p = pinsOf(n, this.cat);
		return svg`<g class=${cls} @pointerdown=${(e: PointerEvent) => this.down(e, net.id, n)}>
			${d?.kind === "fb" ? svg`<text class="fbd-instance" x=${n.x + w / 2} y=${n.y - 6} text-anchor="middle">${n.instance || "?"}</text>` : nothing}
			<rect x=${n.x} y=${n.y} width=${w} height=${h} rx="2" />
			<line class="fbd-head" x1=${n.x} y1=${n.y + HEAD} x2=${n.x + w} y2=${n.y + HEAD} />
			<text class="fbd-type" x=${n.x + w / 2} y=${n.y + 15} text-anchor="middle">${n.type}</text>
			${p.inputs.map((name, i) => svg`<text class="fbd-pin-name" x=${n.x + 7} y=${n.y + HEAD + 16 + i * PIN_DY}>${name}</text>`)}
			${p.outputs.map((name, i) => svg`<text class="fbd-pin-name" x=${n.x + w - 7} y=${n.y + HEAD + 16 + i * PIN_DY} text-anchor="end">${name}</text>`)}
			${p.inputs.map((name) => pin(name, "in"))}
			${p.outputs.map((name) => pin(name, "out"))}
			${d?.kind === "fb" ? p.outputs.map((name, i) => value(`${n.instance}.${name}`, n.x + w + 8, n.y + HEAD + 8 + i * PIN_DY, "start")) : nothing}
		</g>`;
	}

	/** The variable a source pin carries (for its online value): an input variable's expression, an instance output. */
	private sourceExpr(net: FbdNetwork, from: PinRef): string | null {
		const n = net.nodes.find((x) => x.id === from.node);
		if (!n) return null;
		if (n.kind === "input") return n.expr;
		if (n.kind === "box" && this.cat(n.type)?.kind === "fb") return `${n.instance}.${from.pin}`;
		return null;
	}

	private renderNetwork(net: FbdNetwork, index: number) {
		const byId = new Map(net.nodes.map((n) => [n.id, n]));
		const types = networkTypes(net, this.varType ?? (() => null), this.cat);
		const near = this.near && this.near.net === net.id ? this.near : null;
		const svgCls = this.drag?.net === net.id ? "fbd-grabbing" : near || this.wiring?.net === net.id ? "fbd-near" : "";
		const width = Math.max(760, ...net.nodes.map((n) => n.x + size(n, this.cat).w + 120));
		const height = Math.max(120, ...net.nodes.map((n) => n.y + size(n, this.cat).h + 50));
		const problems = this.problems.filter((p) => p.network === net.id);
		return html`
			<section class="fbd-net">
				<header class="fbd-net-head">
					<span class="fbd-net-no">${index + 1}</span>
					<input class="fbd-net-title" .value=${net.title} placeholder="Netzwerk-Kommentar"
						@change=${(e: Event) => { net.title = (e.target as HTMLInputElement).value; this.changed(); }} />
					<button class="ide-icon-btn" title="Netzwerk löschen" @click=${() => this.removeNetwork(net.id)}>🗑</button>
				</header>
				<svg class="fbd-svg ${svgCls}" width=${width} height=${height}
					@pointermove=${(e: PointerEvent) => this.move(e, net.id)} @pointerup=${() => this.up()}
					@pointerleave=${() => { if (!this.wiring && !this.drag) this.near = null; }}
					@pointerdown=${(e: PointerEvent) => this.backgroundDown(e, net.id)}>
					${net.wires.map((w, i) => {
						const a = byId.get(w.from.node), b = byId.get(w.to.node);
						if (!a || !b) return nothing;
						const src = this.sourceExpr(net, w.from);
						const v = src ? this.lookup?.(src) : null;
						const on = v === "TRUE";
						const selected = this.sel && "wire" in this.sel && this.sel.net === net.id && this.sel.wire === i;
						const t = types.pins.get(pinKey(w.from.node, "out", w.from.pin)) ?? "any";
						const mismatch = types.mismatches.has(i);
						const ins = pinsOf(b, this.cat).inputs;
						const d = route(pinAt(a, w.from.pin, "out", this.cat), pinAt(b, w.to.pin, "in", this.cat), Math.max(0, ins.length - 1 - ins.indexOf(w.to.pin)));
						return svg`<g class="fbd-wire fbd-t-${t} ${on ? "fbd-on" : ""} ${mismatch ? "fbd-mismatch" : ""} ${selected ? "fbd-sel" : ""}"
							@pointerdown=${(e: PointerEvent) => {
								e.stopPropagation();
								const pin = this.nearestPin(net.id, this.svgPoint(e));
								if (pin) this.startWire(e, pin);  // a pin under the wire's end wins
								else this.sel = { net: net.id, wire: i };
							}}>
							<title>${mismatch ? "Typen passen nicht zusammen" : TYPE_NAMES[t]}</title>
							<path class="fbd-wire-hit" d=${d} /><path class="fbd-wire-line" d=${d} /></g>`;
					})}
					${net.nodes.map((n) => this.renderNode(net, n, types))}
					${this.wiring && this.wiring.net === net.id ? (() => {
						const w = this.wiring;
						const start = this.pinPos(w.from);
						const end = { x: w.x, y: w.y };
						const d = w.from.side === "out" ? route(start, end) : route(end, start);
						return svg`<path class="fbd-rubber" d=${d} />`;
					})() : nothing}
					${near ? (() => {
						const at = this.pinPos(near);
						return svg`<circle class="fbd-pin-near" cx=${at.x} cy=${at.y} r="9" />`;
					})() : nothing}
				</svg>
				${problems.length ? html`<ul class="fbd-problems">${problems.map((p) => html`<li>${p.message}</li>`)}</ul>` : nothing}
			</section>`;
	}

	private renderProps() {
		const s = this.sel;
		if (!s) return html`<p class="fbd-hint">Auswählen: Klick. Verbinden: in der Nähe eines Anschlusses (Fadenkreuz) drücken und zum Gegenstück ziehen - die Linie rastet am nächsten passenden Anschluss ein. Löschen: Entf.</p>`;
		const net = this.net(s.net);
		if ("wire" in s) {
			const w = net.wires[s.wire];
			return html`<p>Verbindung ${w?.from.pin} → ${w?.to.pin}</p><button @click=${() => this.deleteSelected()}>Verbindung löschen</button>`;
		}
		const n = net.nodes.find((x) => x.id === s.node);
		if (!n) return nothing;
		const field = (label: string, value: string, set: (v: string) => void, hint = "") => html`
			<label class="fbd-field">${label}<input .value=${value} title=${hint}
				@change=${(e: Event) => { set((e.target as HTMLInputElement).value.trim()); this.changed(); }} /></label>`;
		if (n.kind === "input") return html`<h4>Eingangsvariable</h4>${field("Ausdruck", n.expr, (v) => (n.expr = v), "eine Variable, eine Konstante (10, T#300ms) oder ein ST-Ausdruck")}
			<button @click=${() => this.deleteSelected()}>Löschen</button>`;
		if (n.kind === "output") return html`<h4>Ausgangsvariable</h4>${field("Variable", n.target, (v) => (n.target = v))}
			<button @click=${() => this.deleteSelected()}>Löschen</button>`;
		const d = this.cat(n.type);
		return html`<h4>${n.type}</h4><p class="fbd-hint">${d?.title ?? ""}</p>
			${d?.kind === "fb" ? field("Instanz", n.instance ?? "", (v) => (n.instance = v)) : nothing}
			<button @click=${() => this.deleteSelected()}>Löschen</button>`;
	}

	render() {
		const all = [...BOXES, ...(this.pou.extra ?? [])];
		const groups = [...new Set(all.map((b) => b.group))];
		return html`
			<div class="fbd" tabindex="0" @keydown=${(e: KeyboardEvent) => this.key(e)}>
				<div class="fbd-toolbar">
					<select title="Baustein oder Variable einfügen (in das ausgewählte Netzwerk)"
						@change=${(e: Event) => { const el = e.target as HTMLSelectElement; this.add(el.value); el.value = ""; }}>
						<option value="">＋ Einfügen…</option>
						<optgroup label="Variablen"><option value="@input">Eingangsvariable</option><option value="@output">Ausgangsvariable</option></optgroup>
						${groups.map((g) => html`<optgroup label=${g}>${all.filter((b) => b.group === g).map((b) => html`<option value=${b.type}>${b.type} - ${b.title}</option>`)}</optgroup>`)}
					</select>
					<button @click=${() => this.addNetwork()}>＋ Netzwerk</button>
					<span class="fbd-legend" title="Datentypen an den Anschlüssen und Verbindungen (IEC 61131-3 schreibt keine Farben vor)">
						${(["bool", "int", "real", "time", "any"] as const).map((t) => html`<span class="fbd-chip fbd-chip-${t}">${TYPE_NAMES[t]}</span>`)}
					</span>
				</div>
				<div class="fbd-body">
					<div class="fbd-nets">${this.pou.networks.map((n, i) => this.renderNetwork(n, i))}</div>
					<aside class="fbd-props">${this.renderProps()}</aside>
				</div>
			</div>`;
	}
}
