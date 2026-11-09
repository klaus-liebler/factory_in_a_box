// A lightweight ST editor for the mockup: a transparent textarea over a highlighted copy of the text,
// both in one scroll container (they cannot drift apart). The highlighted layer also shows error
// lines and, online, the values of the variables at the end of each line (CODESYS inline
// monitoring). No dependency; whether CodeMirror 6 replaces it is decided after the mockup review.
import { LitElement, html, nothing, type PropertyValues } from "lit";
import { customElement, property, query } from "lit/decorators.js";
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

@customElement("plc-st-editor")
export class StEditor extends LitElement {
	@property({ type: String }) value = "";
	@property({ type: Boolean }) readonly = false;
	@property({ attribute: false }) markers: LineMarker[] = [];
	/** line -> text shown at its end (online values). */
	@property({ attribute: false }) inline = new Map<number, string>();
	@property({ type: Boolean }) stale = false;
	@property({ type: Number }) minLines = 4;
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

	private onKey(e: KeyboardEvent): void {
		const ta = e.target as HTMLTextAreaElement;
		if (this.readonly) return;
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
							const m = byLine.get(i + 1);
							return html`<div class="ste-ln ${m ? `ste-ln-${m.severity}` : ""}" title=${m?.message ?? nothing}>${i + 1}</div>`;
						})}
					</div>
					<div class="ste-area" style="height:${height}px; min-width: calc(${longest + 40}ch)">
						<pre class="ste-hl" aria-hidden="true">${lines.map((l, i) => {
							const m = byLine.get(i + 1);
							const v = this.inline.get(i + 1);
							return html`<div class="ste-row ${m ? `ste-row-${m.severity}` : ""}">${unsafeHTML(l || " ")}${v ? html`<span class="ste-val ${this.stale ? "ste-val-stale" : ""}">${v}</span>` : nothing}</div>`;
						})}</pre>
						<textarea class="ste-input" spellcheck="false" autocapitalize="off" autocomplete="off" wrap="off"
							?readonly=${this.readonly}
							@input=${(e: Event) => this.changed((e.target as HTMLTextAreaElement).value)}
							@keydown=${(e: KeyboardEvent) => this.onKey(e)}></textarea>
					</div>
				</div>
			</div>`;
	}
}
