// Bidirektionale binaere WebSocket-Verbindung zum Server (s. docs/websocket-protocol.md,
// Core/Src/http_websocket_server.hpp) -- mittelfristig der einzige Kanal fuer laufenden
// Datenaustausch zwischen Browser und Firmware; HTTP GET dient nur noch dem einmaligen Laden
// dieser Seite. Jede Binaerframe traegt genau eine Nachricht, deren 4-Byte-Kopf
// (namespaceId/messageTypeId, s. ws-protocol/*.json) hier ausgewertet und an den passenden
// Decoder aus dem generierten web/generated/ws-protocol.ts weitergereicht wird (decode() erwartet
// den KOMPLETTEN Frame inkl. Kopf plus offset=0, s. docs/websocket-protocol.md).
//
// Aktuell nur EIN Nachrichtentyp verdrahtet (system.LogMessage, s. unten) -- weitere
// Namespaces/Nachrichten kommen hinzu, sobald die Firmware sie tatsaechlich sendet/erwartet.
import * as WsProtocol from "../generated/ws-protocol.js";

// Reconnect-Backoff bewusst simpel/fest (kein exponentielles Backoff): das Board ist im
// bestimmungsgemaessen Betrieb dauerhaft im selben Netz erreichbar, ein kurzer fester Abstand
// haelt die Verbindung nach einem Boot/Reset/Netzwechsel zuegig wieder her, ohne bei einem
// tatsaechlich dauerhaft nicht erreichbaren Board die Konsole mit Reconnect-Versuchen zu fluten.
const RECONNECT_DELAY_MS = 2000;

function wsUrl(): string {
	return `wss://${location.host}/ws`;
}

// 1:1 auf die console-Funktion abgebildet, die dem Original-Log-Level entspricht -- WICHTIG:
// console.debug() zaehlt in Chrome DevTools als "Verbose" und ist per Default AUSGEBLENDET, bis
// man den Verbose-Filter aktiviert. Vorher landete INFO faelschlich ebenfalls auf console.debug,
// wodurch praktisch jede Log-Zeile (die meisten sind INFO) ohne Verbose-Filter unsichtbar war.
// console.info() zaehlt dagegen als "Info" und ist per Default sichtbar -- TRACE/DEBUG bleiben
// bewusst auf console.debug (nur bei Bedarf/Verbose sichtbar), das entspricht ihrer Rolle im
// Original (log_set_level() blendet sie im UART-Log ohnehin meist ganz aus).
function consoleFnForLevel(level: WsProtocol.system.LogMessage.LogLevel): (...args: unknown[]) => void {
	switch (level) {
		case WsProtocol.system.LogMessage.LogLevel.LOG_WARN:
			return console.warn;
		case WsProtocol.system.LogMessage.LogLevel.LOG_ERROR:
		case WsProtocol.system.LogMessage.LogLevel.LOG_FATAL:
			return console.error;
		case WsProtocol.system.LogMessage.LogLevel.LOG_INFO:
			return console.info;
		default:
			return console.debug;
	}
}

// TEMPORARY diagnostics (2026-08-19) -- pinning down "LogMessage: frame too short" decode
// errors. Dumps the exact raw bytes of whatever frame failed to decode, so a truncated/corrupt
// frame can be told apart from a genuinely wrong offset/length calculation.
function hexDump(data: ArrayBuffer): string {
	return Array.from(new Uint8Array(data), (b) => b.toString(16).padStart(2, "0")).join(" ");
}

function handleMessage(data: ArrayBuffer): void {
	if (data.byteLength < 4) {
		console.warn(`[diag] WS frame too short for even the 4-byte header: ${data.byteLength} bytes, hex=${hexDump(data)}`);
		return;
	}
	const view = new DataView(data);
	const namespaceId = view.getUint16(0, true);
	const messageTypeId = view.getUint16(2, true);

	if (namespaceId === WsProtocol.system.NAMESPACE_ID && messageTypeId === WsProtocol.system.LogMessage.TYPE_ID) {
		try {
			const msg = WsProtocol.system.LogMessage.decode(view, 0);
			console.debug(`[diag] WS LogMessage ok: ${data.byteLength} bytes`);
			// Platzhalter fuer den Anfang: Firmware-Logs landen vorerst nur in der Devtools-Konsole.
			// Nachrichtentext bewusst als ERSTES Argument (nicht der Zeitstempel-Praefix davor) --
			// eine lange Millisekundenzahl vor dem Text liess die ersten Zeichen der eigentlichen
			// Meldung in der Konsole abgeschnitten wirken, s. Feedback.
			consoleFnForLevel(msg.level)(msg.text, `(t=${msg.timestampMs}ms)`);
		} catch (error) {
			console.warn(
				`[diag] WS LogMessage decode FAILED: ${(error as Error).message}`,
				`frame length=${data.byteLength} bytes`,
				`\nhex=${hexDump(data)}`,
			);
		}
		return;
	}

	// Unbekannte (namespaceId, messageTypeId)-Kombination -- z.B. eine neuere Firmware-Version
	// mit einer Nachricht, die dieser Web-UI-Build noch nicht kennt. Bewusst nur geloggt statt
	// geworfen, damit ein einzelner unbekannter Nachrichtentyp nicht die ganze Verbindung stoert.
	console.debug(`WebSocket: unbekannte Nachricht namespaceId=${namespaceId} messageTypeId=${messageTypeId}`);
}

function connect(): void {
	const ws = new WebSocket(wsUrl());
	ws.binaryType = "arraybuffer";

	ws.addEventListener("message", (event) => {
		if (event.data instanceof ArrayBuffer) {
			handleMessage(event.data);
		}
	});

	ws.addEventListener("close", () => {
		setTimeout(connect, RECONNECT_DELAY_MS);
	});
	ws.addEventListener("error", () => {
		ws.close();
	});
}

export function startWebSocketClient(): void {
	connect();
}
