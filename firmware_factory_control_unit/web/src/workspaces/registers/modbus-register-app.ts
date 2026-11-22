// The register console: every region of the register map with its live values; holding registers can be
// written (not the outputs and commands while the IEC 61131 program is the control source).
import { LitElement, html } from "lit";
import { customElement, state } from "lit/decorators.js";
import "./register-panel.ts";
import { REGIONS } from "../../../generated/register-map.ts";
import { fetchRegisters, writeHolding, type RegisterValues } from "../../core/registers.ts";
import { Poller, pollingGroup } from "../../core/poller.ts";
import { fetchPlcStatus, isIec } from "../../core/plc.ts";
import type { WorkspaceApp } from "@klaus-liebler/ui-shell/shell/workspace-app.ts";
import { setAppStatus } from "@klaus-liebler/ui-shell/shell/status-bar.ts";
import { openWorkspace, setAppRibbon } from "@klaus-liebler/ui-shell/shell/ribbon.ts";

@customElement("modbus-register-app")
export class ModbusRegisterApp extends LitElement implements WorkspaceApp {
	@state() private values: RegisterValues = { holding: [], input: [] };
	/** The IEC 61131 program is the control source: only parameter registers are writable. */
	@state() private iec = false;
	/** The last refused write (shown in the status bar until the next successful one). */
	@state() private writeError: string | null = null;
	private readonly poller = new Poller(this, () => this.poll(), 1000);

	protected createRenderRoot() {
		return this;
	}

	onShow(): void {
		this.poller.start();
	}

	onHide(): void {
		this.poller.stop();
	}

	private async poll(): Promise<void> {
		this.values = await fetchRegisters();
		this.iec = isIec(await fetchPlcStatus());
	}

	private async handleWriteRegister(ev: Event): Promise<void> {
		const { address, values } = (ev as CustomEvent<{ address: number; values: number[] }>).detail;
		try {
			await writeHolding(address, values);
			this.writeError = null;
			this.poller.refresh();
		} catch {
			// refused although offered: the control source may have changed to IEC since the last poll
			const iec = this.iec || isIec(await fetchPlcStatus().catch(() => null));
			this.iec = iec;
			this.writeError = iec ? `Register ${address} nicht geschrieben: das SPS-Programm ist Steuerquelle (nur Parameter schreibbar)` : `Register ${address} nicht geschrieben`;
		}
	}

	protected willUpdate(): void {
		const status = this.writeError ? { text: this.writeError, kind: "error" as const } : this.poller.status("Lade Register …");
		setAppStatus(this, {
			...status,
			extra: this.iec ? html`<span title="Das SPS-Programm schreibt die Ausgänge; schreibbar sind nur Parameter. Umschalten auf Feldbus in der SPS-Entwicklung oder im Aktorentest.">Steuerquelle IEC 61131: Ausgänge gesperrt</span>` : null,
		});
		setAppRibbon(this, [
			{ id: "registers", label: "Register", groups: [
				pollingGroup(this.poller, "die Register"),
				{ label: "Bereiche", commands: REGIONS.map((r, i) => ({
					id: `region-${i}`, label: r.title, icon: "registers" as const, size: "small" as const,
					run: () => this.querySelectorAll("register-panel")[i]?.scrollIntoView({ behavior: "smooth", block: "start" }),
				})) },
				{ label: "Steuerquelle", commands: [
					{ id: "to-plc", label: this.iec ? "IEC 61131 (SPS)" : "Feldbus", icon: this.iec ? "iec" : "fieldbus",
						title: "Wer die Ausgänge schaltet - umschalten in der SPS-Entwicklung", run: () => openWorkspace("/plc") },
				] },
			] },
		]);
	}

	render() {
		return html`
			<div class="container">
				<div class="region-grid" @write-register=${this.handleWriteRegister}>
					${REGIONS.map((region) => html`
						<register-panel .region=${region} .holdingValues=${this.values.holding} .inputValues=${this.values.input} .iecControlled=${this.iec}></register-panel>`)}
				</div>
			</div>`;
	}
}
