// The device's system information (uptime, network, PHY registers, I2C scans, reset cause).
import { system } from "../../generated/ws-protocol.ts";
import { wsRequest } from "../ws-client.ts";

export type SystemInfo = system.SystemInfoMessage.Payload;

export function fetchSystemInfo(): Promise<SystemInfo> {
	return wsRequest(system.NAMESPACE_ID, (requestId) => system.SystemInfoRequest.encode({ requestId }), system.SystemInfoMessage);
}

/** The reset causes (enum ResetCause in src/boot.hpp). */
export const RESET_CAUSE_NAMES: Record<number, string> = {
	0: "Unbekannt",
	1: "Independent Watchdog",
	2: "Window Watchdog",
	3: "Software",
	4: "Low-Power",
	5: "Brown-Out",
	6: "Reset-Pin / Power-On",
};
