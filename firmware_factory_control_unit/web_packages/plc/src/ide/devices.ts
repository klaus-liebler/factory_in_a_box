// The PLC devices of this browser tab, shared by the IDE and the HMI runtime: one simulator (its program
// keeps running when the page changes) and the device the app registers (the FCU app: the FCU over the
// WebSocket; none in the simulation-only app).
import type { PlcDevice } from "./device.ts";
import { Simulator } from "./simulator.ts";

export const simulator = new Simulator();

let remote: PlcDevice | null = null;

/** Called by the app once at the start, before the IDE and the HMI runtime are created. */
export function registerRemoteDevice(d: PlcDevice): void {
	remote = d;
}

/** The registered device; null: only the simulator. */
export function remoteDevice(): PlcDevice | null {
	return remote;
}
