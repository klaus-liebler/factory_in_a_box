// WsProtocolListener fuer den Namespace "system": spiegelt die Firmware-Logzeilen
// (system.LogMessage, s. Core/Src/ws_log_bridge.cpp) in die Browser-Konsole. Die einzige weitere
// system-Nachricht, SystemInfoMessage, ist eine Response und wird von ws-client.ts direkt dem
// wartenden wsRequest() (system-info-app.ts) zugeordnet -- sie kommt hier nie an.
import { system } from "../generated/ws-protocol.js";
import { registerWsProtocolListener, type WsProtocolListener } from "./ws-client.js";

// 1:1 auf die console-Funktion abgebildet, die dem Original-Log-Level entspricht -- WICHTIG:
// console.debug() zaehlt in Chrome DevTools als "Verbose" und ist per Default AUSGEBLENDET, bis
// man den Verbose-Filter aktiviert. Vorher landete INFO faelschlich ebenfalls auf console.debug,
// wodurch praktisch jede Log-Zeile (die meisten sind INFO) ohne Verbose-Filter unsichtbar war.
// console.info() zaehlt dagegen als "Info" und ist per Default sichtbar -- TRACE/DEBUG bleiben
// bewusst auf console.debug (nur bei Bedarf/Verbose sichtbar), das entspricht ihrer Rolle im
// Original (log_set_level() blendet sie im UART-Log ohnehin meist ganz aus).
function consoleFnForLevel(level: system.LogLevel): (...args: unknown[]) => void {
	switch (level) {
		case system.LogLevel.LOG_WARN:
			return console.warn;
		case system.LogLevel.LOG_ERROR:
		case system.LogLevel.LOG_FATAL:
			return console.error;
		case system.LogLevel.LOG_INFO:
			return console.info;
		default:
			return console.debug;
	}
}

class SystemLogListener implements WsProtocolListener {
	onWsMessage(messageTypeId: number, view: DataView): void {
		switch (messageTypeId) {
			case system.LogMessage.TYPE_ID: {
				const msg = system.LogMessage.decode(view, 0);
				// Nachrichtentext bewusst als ERSTES Argument (nicht der Zeitstempel-Praefix davor) --
				// eine lange Millisekundenzahl vor dem Text liess die ersten Zeichen der eigentlichen
				// Meldung in der Konsole abgeschnitten wirken, s. Feedback.
				consoleFnForLevel(msg.level)(msg.text, `(t=${msg.timestampMs}ms)`);
				return;
			}
			default:
				console.debug(`WebSocket: unbekannte system-Nachricht typeId=${messageTypeId}`);
		}
	}
}

/** Fuer die gesamte Seitenlebensdauer (s. app.ts). */
export function startSystemLogListener(): void {
	registerWsProtocolListener(system.NAMESPACE_ID, new SystemLogListener());
}
