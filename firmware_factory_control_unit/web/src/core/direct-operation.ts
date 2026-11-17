// Direct operation: workspaces that operate actuators by hand (Aktorentest, Druckregelstrecke) write the
// output registers themselves. That only works while the fieldbus is the control source - in IEC 61131 mode
// the PLC program owns the outputs and the device refuses the writes (docs/plc-iec61131.md 6.2). This
// controller knows the control source (polled with the workspace), takes the outputs over (switches to the
// fieldbus after asking: the program stops), writes registers with a readable error, and renders the bar that
// says who switches the outputs.
//
// The steppers have their own per-axis manual mode in the firmware (stepper-lab-app.ts) and do not need it.
import { html, nothing, type ReactiveController, type ReactiveControllerHost, type TemplateResult } from "lit";
import { confirmDialog } from "@fiab/ui-shell/shell/dialogs.ts";
import { fetchPlcStatus, isIec, PLC_STATE_NAMES, setControlSource, type PlcStatus } from "./plc.ts";
import { writeRegister } from "./registers.ts";
import type { RegisterDef } from "../../generated/register-map.ts";
import "./direct-operation.css";

export class DirectOperation implements ReactiveController {
	plc: PlcStatus | null = null;
	/** The last refused write or failed take-over (cleared by the next success). */
	error: string | null = null;
	private busy = false;

	constructor(private readonly host: ReactiveControllerHost, private readonly afterWrite: () => void = () => undefined) {
		host.addController(this);
	}

	hostConnected(): void {}

	/** The IEC 61131 program switches the outputs: direct operation is refused. */
	get locked(): boolean {
		return isIec(this.plc);
	}

	/** Call from the workspace's poll. */
	async refresh(): Promise<void> {
		this.plc = await fetchPlcStatus();
		this.host.requestUpdate();
	}

	/** Writes a holding register; a refusal becomes `error` (and the status is read again). `reread`: the
	 *  workspace reads its values again afterwards (not for a controller writing every cycle). */
	async write(reg: string | RegisterDef, value: number, reread = true): Promise<boolean> {
		try {
			await writeRegister(reg, value);
			this.error = null;
			if (reread) this.afterWrite();
			return true;
		} catch {
			await this.refresh().catch(() => undefined);
			const name = typeof reg === "string" ? reg : reg.name;
			this.error = this.locked ? `${name} nicht geschrieben: das SPS-Programm schaltet die Ausgänge` : `${name} nicht geschrieben`;
			this.host.requestUpdate();
			return false;
		}
	}

	/** Switches the control source to the fieldbus after asking (the PLC program stops). */
	async takeOver(): Promise<void> {
		if (this.busy) return;
		const run = this.plc?.state === 2;
		const ok = await confirmDialog({
			title: "Ausgänge übernehmen",
			message: [
				"Die Steuerquelle wechselt von IEC 61131 auf Feldbus: alle Ausgänge gehen zuerst in den sicheren Zustand, dann gehören sie der Direktbedienung (und Modbus/OPC UA).",
				run ? "Das laufende SPS-Programm wird dabei gestoppt." : "",
			].filter(Boolean),
			confirm: "Übernehmen", danger: run,
		});
		if (!ok) return;
		this.busy = true;
		try {
			await setControlSource(false);
			this.error = null;
		} catch (e) {
			this.error = `Übernehmen abgelehnt: ${(e as Error).message}`;
		} finally {
			this.busy = false;
			await this.refresh().catch(() => undefined);
		}
	}

	/** The bar above the controls: who switches the outputs, and the take-over. */
	bar(): TemplateResult {
		const p = this.plc;
		if (!p) return html`<div class="direct-bar direct-unknown">Steuerquelle unbekannt (keine Verbindung)</div>`;
		if (this.locked) {
			return html`<div class="direct-bar direct-locked" role="status">
				<span><b>Gesperrt:</b> Das SPS-Programm (${PLC_STATE_NAMES[p.state] ?? "?"}) schaltet die Ausgänge - Direktbedienung wird vom Gerät abgewiesen.</span>
				<button class="direct-take" ?disabled=${this.busy} @click=${() => void this.takeOver()}>Ausgänge übernehmen (Feldbus)</button>
			</div>`;
		}
		return html`<div class="direct-bar direct-free" role="status">
			<span><b>Direktbedienung:</b> Die Ausgänge schaltet der Feldbus - diese Seite schreibt sie unmittelbar.</span>
			${this.error ? html`<span class="direct-error">${this.error}</span>` : nothing}
		</div>`;
	}
}
