// The PLC's state as the other workspaces need it (the IDE has its own device model, plc/ide/device.ts): who
// switches the outputs (control source), and switching it. In IEC mode the PLC program owns every output and
// command register; the device refuses external writes to them (docs/plc-iec61131.md 6.2).
import { plc } from "../../generated/ws-protocol.ts";
import { wsRequest } from "./ws-client.ts";

export type PlcStatus = plc.StatusResponse.Payload;

export const PLC_STATE_NAMES = ["Kein Programm", "STOP", "RUN", "Fehler", "Angehalten"] as const;

export function fetchPlcStatus(): Promise<PlcStatus> {
	return wsRequest(plc.NAMESPACE_ID, (requestId) => plc.StatusRequest.encode({ requestId }), plc.StatusResponse);
}

export function isIec(s: PlcStatus | null): boolean {
	return s?.source === plc.ControlSource.Iec;
}

/** Switches the control source (outputs go to the safe state first; switching to the fieldbus stops the
 *  program). Throws with the device's reason when it refuses (e.g. another browser is debugging). */
export async function setControlSource(iec: boolean): Promise<void> {
	const command = iec ? plc.PlcCommand.SourceIec : plc.PlcCommand.SourceFieldbus;
	const r = await wsRequest(plc.NAMESPACE_ID, (requestId) => plc.ControlRequest.encode({ requestId, command }), plc.ControlResponse);
	if (!r.ok) throw new Error(r.message);
}
