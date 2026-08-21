// Registerzugriff ueber den binaeren WebSocket-Kanal (s. ws-client.ts) -- Nachfolger der
// frueheren GET /api/registers|/api/write-holding REST-Endpunkte (s. best_binary_buffers_schema/
// modbus.cs, Core/Src/webserver.cpp HandleGetRegisters()/HandleWriteHolding()). Gemeinsam
// genutzt von modbus-register-app.ts und power-management-app.ts.
import { modbus } from "../generated/ws-protocol.js";
import { wsRequest } from "./ws-client.js";

export interface RegisterValues {
	holding: number[];
	input: number[];
}

export async function fetchRegisters(): Promise<RegisterValues> {
	const resp = await wsRequest(
		(requestId) => modbus.GetRegistersRequest.encode({ requestId }),
		(view) => modbus.RegistersMessage.decode(view, 0),
	);
	return { holding: resp.holding, input: resp.input };
}

export async function writeHolding(address: number, value: number): Promise<void> {
	const resp = await wsRequest(
		(requestId) => modbus.WriteHoldingRequest.encode({ requestId, address, value }),
		(view) => modbus.WriteHoldingResponse.decode(view, 0),
	);
	if (!resp.success) {
		throw new Error(`Schreiben von Holding-Register ${address} fehlgeschlagen`);
	}
}
