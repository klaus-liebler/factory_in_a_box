import { LitElement, html } from "lit";
import { customElement, property, state } from "lit/decorators.js";
import type { RegisterRegion, RegisterDef } from "./register-map.js";

// Zeigt eine Register-Region (siehe register-map.ts) als Tabelle. Input-Register sind reine
// Anzeige, Holding-Register bekommen je nach reg.control ein passendes Eingabe-Widget:
// Toggle-Switch fuer 0/1-Aktuatoren (z.B. Ventile), Slider fuer 0..1000-Werte (z.B. PWM-Duty),
// generisches Zahlenfeld + Schreiben-Button fuer alles andere. Firmware-Zugriff macht app.ts
// (dieses Panel feuert nur ein "write-register"-Event nach oben).
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

	private valueFor(address: number, bank: "input" | "holding"): number | undefined {
		return bank === "holding" ? this.holdingValues[address] : this.inputValues[address];
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

	private handleWrite(address: number) {
		const raw = this.pendingEdits[address];
		if (raw === undefined || raw === "") {
			return;
		}
		const value = Number(raw);
		if (!Number.isFinite(value) || value < 0 || value > 0xffff) {
			return;
		}
		this.writeRegister(address, value);
	}

	private renderControl(reg: RegisterDef, value: number | undefined) {
		switch (reg.control) {
			case "toggle":
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
			case "slider": {
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
			case "readonly":
				return null;
			default:
				return html`
					<div class="register-write-controls">
						<input
							class="panel-input register-write-input"
							type="number"
							min="0"
							max="65535"
							placeholder="${String(value ?? 0)}"
							@input=${(ev: Event) => this.handleInput(reg.address, ev)}
						/>
						<button @click=${() => this.handleWrite(reg.address)}>Schreiben</button>
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
								const value = this.valueFor(reg.address, reg.bank);
								const note = [reg.description, reg.unit].filter(Boolean).join(", ");
								return html`
									<tr>
										<td>${reg.name}${note ? html`<span class="panel-text register-comment"> (${note})</span>` : null}</td>
										<td>${reg.address}</td>
										<td><span class="register-bank-tag register-bank-${reg.bank}">${reg.bank}</span></td>
										<td class="register-value">${value ?? "–"}</td>
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
