import { LitElement, html } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import type { RegisterRegion, RegisterDef } from "../../../generated/register-map.ts";
import { hexToRgb565, indexRgb565, registerWords, rgb565ToHex } from "../../core/registers.ts";
import "./register-panel.css";

// Shows a register region (see register-map.ts) as a table. reg.display is the ONE representation for reading
// AND writing:
// - "decimal" (default): number; holding registers get a generic number field + write button.
// - "hex" / "binary": number as a 0x.../0b... string (written like "decimal").
// - "bool": value as a grey (0) / green (1) badge - holding registers additionally get a toggle switch for
// writing.
// - "range": slider with min/max - only sensible for holding registers.
// - "index-rgb565": 32 bits, high word an index (e.g. an LED), low word an RGB565 colour - shown as the index
// and a colour swatch ("bereit" for 0xFFFFFFFF, a command's answer), written with an index field and a colour
// picker.
// reg.combine joins several consecutive registers (from reg.address, MSB first) into one display value (e.g.
// 6 registers -> one chip ID as a hex string). The app does the firmware access (this panel only fires a
// "write-register" event upwards, with all words of the register: a 32-bit value is written in one go).
@customElement("register-panel")
export class RegisterPanel extends LitElement {
	protected createRenderRoot() {
		return this;
	}

	@property({ attribute: false }) region!: RegisterRegion;
	@property({ attribute: false }) holdingValues: number[] = [];
	@property({ attribute: false }) inputValues: number[] = [];
	/** The IEC 61131 program is the control source: the device refuses writes to every holding
	 *  register but the parameters (src/plc.cpp, permit()) - so they get no controls. */
	@property({ attribute: false }) iecControlled = false;

	@state() private pendingEdits: Record<number, string> = {};
	@state() private sliderPreview: Record<number, number> = {};
	@state() private pixelEdits: Record<number, { index: number; colour: string }> = {};

	// Reads "count" (default: 1, or reg.combine.count) consecutive raw values from reg.address of the matching
	// bank - the result can be shorter than expected while the first poll has not loaded the values (completely)
	// yet.
	private rawValuesFor(reg: RegisterDef): number[] {
		const source = reg.bank === "holding" ? this.holdingValues : this.inputValues;
		const count = reg.combine?.count ?? 1;
		const values: number[] = [];
		for (let i = 0; i < count; i++) {
			const v = source[reg.address + i];
			if (v === undefined) break;
			values.push(v);
		}
		return values;
	}

	// Combined MSB first into a decimal number - exact in a JS number for up to 3 registers (48 bits)
	// (Number.MAX_SAFE_INTEGER = 2^53-1); the 6-register chip ID uses "hex" instead (see formatValue), which
	// combines per register into hex nibbles instead of arithmetically, without a precision limit. signed
	// interprets the result as two's complement (only relevant for the 32-bit/2-register case, e.g.
	// SCALE_A_WEIGHT) - with more than 2 registers the sign bit would be ambiguous, so it is ignored there.
	private combineDecimal(raws: number[], signed: boolean | undefined): number {
		const unsigned = raws.reduce((acc, v) => acc * 65536 + v, 0);
		if (signed && raws.length === 2 && unsigned > 0x7fffffff) {
			return unsigned - 0x100000000;
		}
		return unsigned;
	}

	private formatValue(reg: RegisterDef, raws: number[]): string {
		if (raws.length === 0) return "–";
		switch (reg.display) {
			case "hex":
				return "0x" + raws.map((v) => v.toString(16).toUpperCase().padStart(4, "0")).join("");
			case "binary":
				return "0b" + raws.map((v) => v.toString(2).padStart(16, "0")).join("");
			default:
				return String(this.combineDecimal(raws, reg.signed));
		}
	}

	private writeRegister(reg: RegisterDef, value: number) {
		this.dispatchEvent(
			new CustomEvent("write-register", { detail: { address: reg.address, values: registerWords(reg, value) }, bubbles: true, composed: true })
		);
	}

	/** The value range a register can be written with: min/max of the schema, else its word count's. */
	private writeRange(reg: RegisterDef): [number, number] {
		const wide = (reg.combine?.count ?? 1) === 2;
		const min = reg.min ?? (wide && reg.signed ? -0x80000000 : reg.signed ? -0x8000 : 0);
		const max = reg.max ?? (wide ? (reg.signed ? 0x7fffffff : 0xffffffff) : reg.signed ? 0x7fff : 0xffff);
		return [min, max];
	}

	private handleToggle(reg: RegisterDef, ev: Event) {
		this.writeRegister(reg, (ev.target as HTMLInputElement).checked ? 1 : 0);
	}

	private handleSliderInput(address: number, ev: Event) {
		// While dragging, only update the live display - the write happens on "change" (release), so that the small
		// server packet pool is not flooded with one request per pixel of mouse movement.
		this.sliderPreview = { ...this.sliderPreview, [address]: Number((ev.target as HTMLInputElement).value) };
	}

	private handleSliderChange(reg: RegisterDef, ev: Event) {
		this.writeRegister(reg, Number((ev.target as HTMLInputElement).value));
	}

	private handleInput(address: number, ev: Event) {
		const input = ev.target as HTMLInputElement;
		this.pendingEdits = { ...this.pendingEdits, [address]: input.value };
	}

	private handleWrite(address: number, reg: RegisterDef) {
		const raw = this.pendingEdits[address];
		if (raw === undefined || raw === "") {
			return;
		}
		const value = Number(raw);
		const [min, max] = this.writeRange(reg);
		if (!Number.isInteger(value) || value < min || value > max) {
			return;
		}
		this.writeRegister(reg, value);
	}

	private pixelEdit(reg: RegisterDef): { index: number; colour: string } {
		// a command register: it reads "ready" (0xFFFFFFFF) once done - the edit starts at LED 0, red
		return this.pixelEdits[reg.address] ?? { index: 0, colour: "#ff0000" };
	}

	private editPixel(reg: RegisterDef, change: Partial<{ index: number; colour: string }>) {
		this.pixelEdits = { ...this.pixelEdits, [reg.address]: { ...this.pixelEdit(reg), ...change } };
	}

	private writePixel(reg: RegisterDef) {
		const { index, colour } = this.pixelEdit(reg);
		if (!Number.isInteger(index) || index < 0 || index > 0xffff) return;
		this.writeRegister(reg, indexRgb565(index, hexToRgb565(colour)));
	}

	private renderValueCell(reg: RegisterDef, raws: number[]) {
		if (reg.display === "bool" || reg.display === "bool-error") {
			const on = (raws[0] ?? 0) !== 0;
			const onClass = reg.display === "bool-error" ? "bool-badge-error" : "bool-badge-on";
			return html`<span class="bool-badge ${on ? onClass : "bool-badge-off"}">${raws.length ? (on ? 1 : 0) : "–"}</span>`;
		}
		if (reg.display === "status") {
			// Error code convention: 0 = ok (grey), non-zero = error (red) - shows the actual value, not just a 0/1
			// badge, in case several different error codes appear in the future.
			const value = raws[0] ?? 0;
			const ok = value === 0;
			return html`<span class="bool-badge ${ok ? "bool-badge-off" : "bool-badge-error"}">${raws.length ? value : "–"}</span>`;
		}
		if (reg.display === "index-rgb565") {
			if (raws.length < 2) return "–";
			if (raws[0] === 0xffff && raws[1] === 0xffff) return html`<span class="panel-text" title="0xFFFFFFFF: der letzte Schreibvorgang ist ausgeführt">bereit</span>`;
			const colour = rgb565ToHex(raws[1]!);
			return html`<span class="register-pixel">#${raws[0]} <span class="register-swatch" style="background:${colour}" title="${colour}"></span></span>`;
		}
		return this.formatValue(reg, raws);
	}

	private renderControl(reg: RegisterDef, raws: number[]) {
		const value = raws[0];
		if (reg.bank !== "holding") {
			// Input registers only show hardware state, never writable.
			return null;
		}
		if (this.iecControlled && reg.kind !== "parameter") {
			return html`<span class="register-locked" title="Steuerquelle IEC 61131: das SPS-Programm schreibt dieses Register. Umschalten auf Feldbus in der SPS-Seite.">🔒 SPS</span>`;
		}
		switch (reg.display) {
			case "bool":
			case "bool-error":
				return html`
					<label class="toggle-switch">
						<input
							type="checkbox"
							.checked=${(value ?? 0) !== 0}
							@change=${(ev: Event) => this.handleToggle(reg, ev)}
						/>
						<span class="toggle-slider-track"></span>
					</label>
				`;
			case "range": {
				const min = reg.min ?? 0;
				const max = reg.max ?? 1000;
				const preview = this.sliderPreview[reg.address] ?? value ?? min;
				return html`
					<div class="register-slider-controls">
						<input
							class="panel-slider"
							type="range"
							min=${min}
							max=${max}
							.value=${String(preview)}
							@input=${(ev: Event) => this.handleSliderInput(reg.address, ev)}
							@change=${(ev: Event) => this.handleSliderChange(reg, ev)}
						/>
						<span class="register-slider-value">${preview}</span>
					</div>
				`;
			}
			case "index-rgb565": {
				const edit = this.pixelEdit(reg);
				return html`
					<div class="register-write-controls">
						<input
							class="panel-input register-pixel-index"
							type="number"
							min="0"
							max="65535"
							title="Index"
							.value=${String(edit.index)}
							@input=${(ev: Event) => this.editPixel(reg, { index: Number((ev.target as HTMLInputElement).value) })}
						/>
						<input
							class="register-pixel-colour"
							type="color"
							title="Farbe (wird auf RGB565 gerundet)"
							.value=${edit.colour}
							@input=${(ev: Event) => this.editPixel(reg, { colour: (ev.target as HTMLInputElement).value })}
						/>
						<button @click=${() => this.writePixel(reg)}>Schreiben</button>
					</div>
				`;
			}
			default: {
				const [min, max] = this.writeRange(reg);
				return html`
					<div class="register-write-controls">
						<input
							class="panel-input register-write-input"
							type="number"
							min="${min}"
							max="${max}"
							placeholder="${String(value ?? 0)}"
							@input=${(ev: Event) => this.handleInput(reg.address, ev)}
						/>
						<button @click=${() => this.handleWrite(reg.address, reg)}>Schreiben</button>
					</div>
				`;
			}
		}
	}

	render() {
		return html`
			<div class="panel-section">
				<div class="panel-label">${this.region.title}</div>
				<div class="register-table-wrap">
					<table class="register-table">
						<thead>
							<tr>
								<th>Name</th>
								<th>Adresse</th>
								<th>Bank</th>
								<th>Wert</th>
								<th></th>
							</tr>
						</thead>
						<tbody>
							${this.region.registers.map((reg) => {
								const raws = this.rawValuesFor(reg);
								const note = [reg.description, reg.unit].filter(Boolean).join(", ");
								return html`
									<tr>
										<td>${reg.name}${note ? html`<span class="panel-text register-comment"> (${note})</span>` : null}</td>
										<td>${reg.address}</td>
										<td><span class="register-bank-tag register-bank-${reg.bank}">${reg.bank}</span></td>
										<td class="register-value">${this.renderValueCell(reg, raws)}</td>
										<td>${this.renderControl(reg, raws)}</td>
									</tr>
								`;
							})}
						</tbody>
					</table>
				</div>
			</div>
		`;
	}
}
