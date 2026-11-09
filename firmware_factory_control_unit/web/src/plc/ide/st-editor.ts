// A lightweight ST editor for the mockup: a transparent textarea over a highlighted copy of the text,
// both in one scroll container (they cannot drift apart). The highlighted layer also shows error
// lines and, online, the values of the variables at the end of each line (CODESYS inline
// monitoring). No dependency; whether CodeMirror 6 replaces it is decided after the mockup review.
import { LitElement, html, nothing, type PropertyValues } from "lit";
import { customElement, property, query, state } from "lit/decorators.js";
import { unsafeHTML } from "lit/directives/unsafe-html.js";
import { KEYWORDS } from "../compiler/lexer.ts";
import { ELEM } from "../compiler/types.ts";

export interface LineMarker {
	line: number; // 1-based
	severity: "error" | "warning" | "runtime";
	message: string;
}

const TYPES = new Set([...Object.keys(ELEM), "STRING", "WSTRING", "ARRAY", "STRUCT"]);
const STD_FBS = new Set(["TON", "TOF", "TP", "R_TRIG", "F_TRIG", "SR", "RS", "CTU", "CTD", "CTUD"]);
const LINE_HEIGHT = 20;

const esc = (s: string) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");

/** HTML per line; (* *) comments may span lines. */
export function highlight(text: string): string[] {
	const out: string[] = [];
	let inComment = false;
	for (const line of text.split("\n")) {
		let h = "";
		let i = 0;
		while (i < line.length) {
			if (inComment) {
				const end = line.indexOf("*)", i);
				const stop = end < 0 ? line.length : end + 2;
				h += `<span class="t-comment">${esc(line.slice(i, stop))}</span>`;
				i = stop;
				if (end >= 0) inComment = false;
				continue;
			}
			const rest = line.slice(i);
			let m: RegExpExecArray | null;
			if (rest.startsWith("(*")) {
				inComment = true;
				continue;
			}
			if (rest.startsWith("//")) {
				h += `<span class="t-comment">${esc(rest)}</span>`;
				break;
			}
			if ((m = /^'[^']*'?/.exec(rest))) {
				h += `<span class="t-string">${esc(m[0])}</span>`;
				i += m[0].length;
			} else if ((m = /^(?:T|TIME|LT|LTIME)#[-+]?[0-9a-zA-Z_.]+/i.exec(rest)) || (m = /^[A-Za-z_]\w*#[-+]?[0-9A-Za-z_.#]+/.exec(rest))) {
				h += `<span class="t-number">${esc(m[0])}</span>`;
				i += m[0].length;
			} else if ((m = /^\d[\d_]*(?:#[0-9A-Fa-f_]+|(?:\.\d[\d_]*)?(?:[eE][-+]?\d+)?)/.exec(rest))) {
				h += `<span class="t-number">${esc(m[0])}</span>`;
				i += m[0].length;
			} else if ((m = /^[A-Za-z_]\w*/.exec(rest))) {
				const up = m[0].toUpperCase();
				const cls = TYPES.has(up) ? "t-type" : KEYWORDS.has(up) ? "t-keyword" : STD_FBS.has(up) ? "t-fb" : "t-ident";
				h += `<span class="${cls}">${esc(m[0])}</span>`;
				i += m[0].length;
			} else if ((m = /^(?::=|=>|<=|>=|<>|\*\*|[-+*/=<>&])/.exec(rest))) {
				h += `<span class="t-op">${esc(m[0])}</span>`;
				i += m[0].length;
			} else {
				h += esc(rest[0]);
				i++;
			}
		}
		out.push(h);
	}
	return out;
}

/** A completion: what is inserted, and what it is (shown greyed). */
export interface Completion {
	label: string;
	detail: string;
	kind: "keyword" | "type" | "variable" | "io" | "fb" | "function" | "pou" | "field";
}

/** What the caret is after: an identifier begun (prefix), maybe after "a.b." (object "a.b"). */
export interface CompletionContext {
	prefix: string;
	object: string | null;
}

/** The completion context at `pos` in `text`; null: not in an identifier (a comment, a string, a number). */
export function completionContext(text: string, pos: number): CompletionContext | null {
	const before = text.slice(0, pos);
	const line = before.slice(before.lastIndexOf("\n") + 1);
	// not inside a comment or a string of this line (multi-line comments: the lexer's job - good enough)
	if (/\/\/|\(\*(?!.*\*\))/.test(line.replace(/'[^']*'/g, "''")) || (line.split("'").length - 1) % 2 === 1) return null;
	const m = /((?:[A-Za-z_][A-Za-z0-9_]*\.)*)([A-Za-z_][A-Za-z0-9_]*)?$/.exec(line);
	if (!m || (!m[1] && !m[2])) return null;
	const start = line.length - m[0].length;
	if (start > 0 && /[0-9#]/.test(line[start - 1])) return null;  // 16#FF, T#1s
	return { prefix: m[2] ?? "", object: m[1] ? m[1].slice(0, -1) : null };
}

const KIND_ICON: Record<Completion["kind"], string> = { keyword: "K", type: "T", variable: "V", io: "IO", fb: "FB", function: "ƒ", pou: "P", field: "." };

@customElement("plc-st-editor")
export class StEditor extends LitElement {
	/** Code completion: the candidates for a context (null: none). */
	@property({ attribute: false }) completions: ((ctx: CompletionContext) => Completion[]) | null = null;
	/** The open completion list; x/y: where it is in the window (fixed: the editor's scroll area would clip it). */
	@state() private popup: { items: Completion[]; index: number; x: number; y: number; above: boolean; prefix: string } | null = null;
	@property({ type: String }) value = "";
	@property({ type: Boolean }) readonly = false;
	@property({ attribute: false }) markers: LineMarker[] = [];
	/** line -> text shown at its end (online values). */
	@property({ attribute: false }) inline = new Map<number, string>();
	@property({ type: Boolean }) stale = false;
	@property({ type: Number }) minLines = 4;
	/** Debugging: lines with a breakpoint, lines that can take one (null: no breakpoints here), the
	 *  line the program halted on (0: none). A click on a line number toggles ("toggle-breakpoint"). */
	@property({ attribute: false }) breakpoints = new Set<number>();
	@property({ attribute: false }) breakable: Set<number> | null = null;
	@property({ type: Number }) current = 0;
	@query("textarea") private input!: HTMLTextAreaElement;

	protected createRenderRoot() {
		return this;
	}

	/** Puts the caret on a line (jump from the messages). */
	reveal(line: number, col = 1): void {
		const lines = this.value.split("\n");
		let offset = 0;
		for (let i = 0; i < Math.min(line - 1, lines.length); i++) offset += lines[i].length + 1;
		offset += Math.max(0, col - 1);
		this.input.focus();
		this.input.setSelectionRange(offset, offset);
		const scroller = this.querySelector(".ste-scroll") as HTMLElement;
		scroller.scrollTop = Math.max(0, (line - 3) * LINE_HEIGHT);
	}

	private changed(text: string): void {
		this.value = text;
		this.dispatchEvent(new CustomEvent("change", { detail: text, bubbles: true }));
	}

	/** Opens (or updates) the completion list for the caret; `explicit`: Ctrl+Space (also an empty prefix). */
	private complete(explicit: boolean): void {
		const ta = this.input;
		if (!this.completions || this.readonly || !ta || ta.selectionStart !== ta.selectionEnd) {
			this.popup = null;
			return;
		}
		const pos = ta.selectionStart;
		const ctx = completionContext(ta.value, pos);
		if (!ctx || (!explicit && ctx.prefix === "" && ctx.object === null)) {
			this.popup = null;
			return;
		}
		const up = ctx.prefix.toUpperCase();
		const items = this.completions(ctx).filter((c) => c.label.toUpperCase().startsWith(up) && c.label.toUpperCase() !== up)
			.sort((a, b) => a.label.localeCompare(b.label)).slice(0, 60);
		if (!items.length) {
			this.popup = null;
			return;
		}
		const before = ta.value.slice(0, pos);
		const line = before.split("\n").length - 1;
		const col = pos - (before.lastIndexOf("\n") + 1) - ctx.prefix.length;
		// the caret's place in the window: the textarea's box, its padding, the monospace character width
		const r = ta.getBoundingClientRect();
		const cs = getComputedStyle(ta);
		const measure = document.createElement("canvas").getContext("2d")!;
		measure.font = `${cs.fontSize} ${cs.fontFamily}`;
		const x = r.left + parseFloat(cs.paddingLeft) + col * measure.measureText("0").width;
		const top = r.top + parseFloat(cs.paddingTop) + line * LINE_HEIGHT;
		const above = top + LINE_HEIGHT + 230 > window.innerHeight;  // near the bottom: open upwards
		this.popup = { items, index: 0, x, y: above ? top - 2 : top + LINE_HEIGHT + 2, above, prefix: ctx.prefix };
	}

	private accept(c: Completion): void {
		const ta = this.input;
		const pos = ta.selectionStart;
		ta.setRangeText(c.label, pos - (this.popup?.prefix.length ?? 0), pos, "end");
		this.popup = null;
		this.changed(ta.value);
		ta.focus();
	}

	private onKey(e: KeyboardEvent): void {
		const ta = e.target as HTMLTextAreaElement;
		if (this.readonly) return;
		const p = this.popup;
		if (p) {
			if (e.key === "ArrowDown" || e.key === "ArrowUp") {
				e.preventDefault();
				const n = p.items.length;
				this.popup = { ...p, index: (p.index + (e.key === "ArrowDown" ? 1 : n - 1)) % n };
				return;
			}
			if (e.key === "Enter" || e.key === "Tab") {
				e.preventDefault();
				this.accept(p.items[p.index]);
				return;
			}
			if (e.key === "Escape") {
				e.preventDefault();
				this.popup = null;
				return;
			}
		}
		if (e.key === " " && e.ctrlKey) {
			e.preventDefault();
			this.complete(true);
			return;
		}
		const { selectionStart: s, selectionEnd: t, value } = ta;
		if (e.key === "Tab") {
			e.preventDefault();
			ta.setRangeText("    ", s, t, "end");
			this.changed(ta.value);
		} else if (e.key === "Enter") {
			// keep the indentation of the line
			e.preventDefault();
			const lineStart = value.lastIndexOf("\n", s - 1) + 1;
			const indent = /^[ \t]*/.exec(value.slice(lineStart))![0];
			ta.setRangeText("\n" + indent, s, t, "end");
			this.changed(ta.value);
		}
	}

	protected updated(changed: PropertyValues): void {
		if (changed.has("value") && this.input && this.input.value !== this.value) this.input.value = this.value;
	}

	render() {
		const lines = highlight(this.value);
		const count = Math.max(lines.length, this.minLines);
		const byLine = new Map<number, LineMarker>();
		for (const m of this.markers) if (!byLine.has(m.line) || m.severity !== "warning") byLine.set(m.line, m);
		const longest = Math.max(...this.value.split("\n").map((l) => l.length), 20);
		const height = count * LINE_HEIGHT + 16;
		return html`
			<div class="ste ${this.readonly ? "ste-readonly" : ""}">
				<div class="ste-scroll">
					<div class="ste-gutter" style="height:${height}px">
						${Array.from({ length: count }, (_, i) => {
							const n = i + 1;
							const m = byLine.get(n);
							const can = this.breakable?.has(n) ?? false;
							const bp = this.breakpoints.has(n);
							const title = m?.message ?? (can ? (bp ? "Haltepunkt entfernen" : "Haltepunkt setzen") : nothing);
							return html`<div class="ste-ln ${m ? `ste-ln-${m.severity}` : ""} ${can ? "ste-ln-can" : ""} ${bp ? "ste-ln-bp" : ""} ${this.current === n ? "ste-ln-current" : ""}"
								title=${title}
								@click=${can ? () => this.dispatchEvent(new CustomEvent("toggle-breakpoint", { detail: n, bubbles: true })) : nothing}>${n}</div>`;
						})}
					</div>
					<div class="ste-area" style="height:${height}px; min-width: calc(${longest + 40}ch)">
						<pre class="ste-hl" aria-hidden="true">${lines.map((l, i) => {
							const m = byLine.get(i + 1);
							const v = this.inline.get(i + 1);
							return html`<div class="ste-row ${m ? `ste-row-${m.severity}` : ""} ${this.current === i + 1 ? "ste-row-current" : ""}">${unsafeHTML(l || " ")}${v ? html`<span class="ste-val ${this.stale ? "ste-val-stale" : ""}">${v}</span>` : nothing}</div>`;
						})}</pre>
						<textarea class="ste-input" spellcheck="false" autocapitalize="off" autocomplete="off" wrap="off"
							?readonly=${this.readonly}
							@input=${(e: Event) => {
								this.changed((e.target as HTMLTextAreaElement).value);
								this.complete(false);
							}}
							@keydown=${(e: KeyboardEvent) => this.onKey(e)}
							@blur=${() => setTimeout(() => (this.popup = null), 150)}
							@click=${() => (this.popup = null)}></textarea>
						${this.popup ? html`<ul class="ste-complete" role="listbox"
							style="left:${this.popup.x}px; ${this.popup.above ? `bottom:${window.innerHeight - this.popup.y}px` : `top:${this.popup.y}px`}">
							${this.popup.items.map((c, i) => html`<li role="option" class=${i === this.popup!.index ? "on" : ""} aria-selected=${i === this.popup!.index}
								@mousedown=${(e: Event) => {
									e.preventDefault();
									this.accept(c);
								}}><span class="ste-ck ste-ck-${c.kind}">${KIND_ICON[c.kind]}</span><span class="ste-cl">${c.label}</span><span class="ste-cd">${c.detail}</span></li>`)}
						</ul>` : nothing}
					</div>
				</div>
			</div>`;
	}
}
