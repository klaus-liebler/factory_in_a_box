import { LitElement, html } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import type { RegisterRegion, RegisterDef } from "./register-map.js";

// Zeigt eine Register-Region (siehe register-map.ts) als Tabelle. reg.display ist die EINE
// Darstellungsform fuers Lesen UND Schreiben:
//   - "decimal" (Default): Zahl; Holding-Register bekommen ein generisches Zahlenfeld +
//     Schreiben-Button.
//   - "hex" / "binary": Zahl als 0x.../0b...-String (Lesen wie "decimal" geschrieben).
//   - "bool": Wert als grau (0) / gruen (1) eingefaerbtes Badge -- bei Holding-Registern
//     zusaetzlich ein Toggle-Switch zum Schreiben (Ersatz fuer den frueheren "toggle"-Control).
//   - "range": Slider mit min/max (Ersatz fuer den frueheren "slider"-Control) -- nur bei
//     Holding-Registern sinnvoll.
// reg.combine fasst mehrere aufeinanderfolgende Register (ab reg.address, MSB zuerst) zu
// einem gemeinsamen Anzeigewert zusammen (z.B. 6 Register -> eine Chip-ID als Hex-String).
// Firmware-Zugriff macht app.ts (dieses Panel feuert nur ein "write-register"-Event nach oben).
@customElement("register-panel")
export class RegisterPanel extends LitElement {
	protected createRenderRoot() {
		return this;
	}

	@property({ attribute: false }) region!: RegisterRegion;
	@property({ attribute: false }) holdingValues: number[] = [];
	@property({ attribute: false }) inputValues: number[] = [];

	@state() private pendingEdits: Record<number, string> = {};
	@state() private sliderPreview: Record<number, number> = {};

	// Liest "count" (Standard: 1, oder reg.combine.count) aufeinanderfolgende Rohwerte ab
	// reg.address aus der passenden Bank -- Rueckgabe kann kuerzer als erwartet sein, solange
	// die Werte noch nicht (vollstaendig) vom ersten Poll geladen wurden.
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

	// MSB-zuerst zu einer Dezimalzahl kombiniert -- fuer bis zu 3 Register (48 Bit) noch exakt
	// in einer JS-Number darstellbar (Number.MAX_SAFE_INTEGER = 2^53-1); fuer die 6-Register-
	// Chip-ID wird stattdessen "hex" verwendet (s. formatValue), das kombiniert stattdessen
	// pro Register einzeln zu Hex-Nibbles statt rechnerisch, keine Praezisionsgrenze.
	// signed interpretiert das Ergebnis als Zweierkomplement (nur fuer den 32-Bit/2-Register-
	// Fall relevant, z.B. SCALE_A_WEIGHT) -- bei mehr als 2 Registern bliebe das Vorzeichenbit
	// mehrdeutig, daher dort ignoriert.
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

	private writeRegister(address: number, value: number) {
		this.dispatchEvent(
			new CustomEvent("write-register", { detail: { address, value }, bubbles: true, composed: true })
		);
	}

	private handleToggle(address: number, ev: Event) {
		this.writeRegister(address, (ev.target as HTMLInputElement).checked ? 1 : 0);
	}

	private handleSliderInput(address: number, ev: Event) {
		// Nur die Live-Anzeige aktualisieren, waehrend gezogen wird -- geschrieben wird erst
		// bei "change" (Loslassen), um den kleinen Server-Paket-Pool nicht mit einem Request
		// pro Pixel Mausbewegung zu fluten.
		this.sliderPreview = { ...this.sliderPreview, [address]: Number((ev.target as HTMLInputElement).value) };
	}

	private handleSliderChange(address: number, ev: Event) {
		this.writeRegister(address, Number((ev.target as HTMLInputElement).value));
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
		const min = reg.min ?? 0;
		const max = reg.max ?? 0xffff;
		if (!Number.isFinite(value) || value < min || value > max) {
			return;
		}
		this.writeRegister(address, value);
	}

	private renderValueCell(reg: RegisterDef, raws: number[]) {
		if (reg.display === "bool") {
			const on = (raws[0] ?? 0) !== 0;
			return html`<span class="bool-badge ${on ? "bool-badge-on" : "bool-badge-off"}">${raws.length ? (on ? 1 : 0) : "–"}</span>`;
		}
		return this.formatValue(reg, raws);
	}

	private renderControl(reg: RegisterDef, value: number | undefined) {
		if (reg.bank !== "holding") {
			// Input-Register sind reine Hardware-Anzeige, nie schreibbar.
			return null;
		}
		switch (reg.display) {
			case "bool":
				return html`
					<label class="toggle-switch">
						<input
							type="checkbox"
							.checked=${(value ?? 0) !== 0}
							@change=${(ev: Event) => this.handleToggle(reg.address, ev)}
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
							@change=${(ev: Event) => this.handleSliderChange(reg.address, ev)}
						/>
						<span class="register-slider-value">${preview}</span>
					</div>
				`;
			}
			default:
				return html`
					<div class="register-write-controls">
						<input
							class="panel-input register-write-input"
							type="number"
							min="${reg.min ?? 0}"
							max="${reg.max ?? 0xffff}"
							placeholder="${String(value ?? 0)}"
							@input=${(ev: Event) => this.handleInput(reg.address, ev)}
						/>
						<button @click=${() => this.handleWrite(reg.address, reg)}>Schreiben</button>
					</div>
				`;
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
								const value = raws[0];
								const note = [reg.description, reg.unit].filter(Boolean).join(", ");
								return html`
									<tr>
										<td>${reg.name}${note ? html`<span class="panel-text register-comment"> (${note})</span>` : null}</td>
										<td>${reg.address}</td>
										<td><span class="register-bank-tag register-bank-${reg.bank}">${reg.bank}</span></td>
										<td class="register-value">${this.renderValueCell(reg, raws)}</td>
										<td>${this.renderControl(reg, value)}</td>
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
