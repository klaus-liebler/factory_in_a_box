// Bidirektionale binaere WebSocket-Verbindung zum Server (s. docs/websocket-protocol.md,
// Core/Src/http_websocket_server.hpp) -- mittelfristig der einzige Kanal fuer laufenden
// Datenaustausch zwischen Browser und Firmware; HTTP GET dient nur noch dem einmaligen Laden
// dieser Seite. Jede Binaerframe traegt genau eine Nachricht, deren 4-Byte-Kopf
// (namespaceId/messageTypeId, s. ws-protocol/*.json) hier ausgewertet und an den passenden
// Decoder aus dem generierten web/generated/ws-protocol.ts weitergereicht wird (decode() erwartet
// den KOMPLETTEN Frame inkl. Kopf plus offset=0, s. docs/websocket-protocol.md).
//
// Mehrere Nachrichten-"Formen" werden unterschieden:
// - system.LogMessage: reiner Server->Client-Log-Spiegel (kein Senden noetig).
// - tasks.TaskListMessage (Antwort auf tasks.TaskManagerRequest, s. sendBinary()/
//   setTaskListListener() unten, genutzt von web/src/apps/task-manager-app.ts).
// - roarm.* Request/Response (StartTeachMode, Mission-CRUD, ...): das generierte Payload traegt
//   als ERSTES Feld immer requestId (uint16, direkt hinter dem 4-Byte-Kopf) -- roarmRequest()
//   nutzt das aus, um eine Antwort anhand ihrer requestId dem wartenden Promise zuzuordnen,
//   unabhaengig vom konkreten Nachrichtentyp.
// - roarm.* Events (JointJogTarget/CartesianJogTarget als Client->Firmware, PoseFeedback als
//   Firmware->Client): kein Umlauf/keine requestId -- sendRoArmEvent() fuers Senden,
//   subscribeRoArmEvent() fuers laufende Empfangen (z.B. PoseFeedback waehrend Teach-Modus).
import * as WsProtocol from "../generated/ws-protocol.js";

// Reconnect-Backoff bewusst simpel/fest (kein exponentielles Backoff): das Board ist im
// bestimmungsgemaessen Betrieb dauerhaft im selben Netz erreichbar, ein kurzer fester Abstand
// haelt die Verbindung nach einem Boot/Reset/Netzwechsel zuegig wieder her, ohne bei einem
// tatsaechlich dauerhaft nicht erreichbaren Board die Konsole mit Reconnect-Versuchen zu fluten.
const RECONNECT_DELAY_MS = 2000;
const REQUEST_TIMEOUT_MS = 3000;

function wsUrl(): string {
	return `wss://${location.host}/ws`;
}

// Nur waehrend eine Verbindung offen ist gesetzt (s. connect() unten) -- sendBinary() ist damit
// aus jedem Aufrufer gefahrlos jederzeit aufrufbar, auch waehrend eines Reconnects.
let activeSocket: WebSocket | null = null;

// Kein Queueing bei fehlender Verbindung (bewusst, s. RECONNECT_DELAY_MS-Kommentar unten): der
// naechste Aufruf (z.B. der naechste Poll-Tick von task-manager-app.ts) greift ohnehin von
// selbst wieder, ein einzelner verlorener Request ist fuer eine Diagnose-Seite unerheblich.
export function sendBinary(data: Uint8Array): void {
	if (activeSocket && activeSocket.readyState === WebSocket.OPEN) {
		activeSocket.send(data);
	}
}

type TaskListListener = (payload: WsProtocol.tasks.TaskListMessage.Payload) => void;
let taskListListener: TaskListListener | null = null;

// Registriert/deregistriert (via null) den einzigen Abnehmer fuer tasks.TaskListMessage --
// bewusst kein genereller Event-Bus fuer (aktuell) genau einen Nachrichtentyp, s.
// task-manager-app.ts onShow()/onHide().
export function setTaskListListener(listener: TaskListListener | null): void {
	taskListListener = listener;
}

// 1:1 auf die console-Funktion abgebildet, die dem Original-Log-Level entspricht -- WICHTIG:
// console.debug() zaehlt in Chrome DevTools als "Verbose" und ist per Default AUSGEBLENDET, bis
// man den Verbose-Filter aktiviert. Vorher landete INFO faelschlich ebenfalls auf console.debug,
// wodurch praktisch jede Log-Zeile (die meisten sind INFO) ohne Verbose-Filter unsichtbar war.
// console.info() zaehlt dagegen als "Info" und ist per Default sichtbar -- TRACE/DEBUG bleiben
// bewusst auf console.debug (nur bei Bedarf/Verbose sichtbar), das entspricht ihrer Rolle im
// Original (log_set_level() blendet sie im UART-Log ohnehin meist ganz aus).
function consoleFnForLevel(level: WsProtocol.system.LogLevel): (...args: unknown[]) => void {
	switch (level) {
		case WsProtocol.system.LogLevel.LOG_WARN:
			return console.warn;
		case WsProtocol.system.LogLevel.LOG_ERROR:
		case WsProtocol.system.LogLevel.LOG_FATAL:
			return console.error;
		case WsProtocol.system.LogLevel.LOG_INFO:
			return console.info;
		default:
			return console.debug;
	}
}

function hexDump(data: ArrayBuffer): string {
	return Array.from(new Uint8Array(data), (b) => b.toString(16).padStart(2, "0")).join(" ");
}

let socket: WebSocket | null = null;
let nextRequestId = 1;
const pendingRequests = new Map<number, { resolve: (view: DataView) => void; reject: (err: Error) => void; timer: number }>();
const eventSubscribers = new Map<number, Set<(view: DataView) => void>>(); // key = typeId innerhalb roarm.NAMESPACE_ID

function handleRoarmMessage(view: DataView, typeId: number): void {
	const responseTypeIds: ReadonlySet<number> = new Set([
		WsProtocol.roarm.StartTeachModeResponse.TYPE_ID,
		WsProtocol.roarm.StopTeachModeResponse.TYPE_ID,
		WsProtocol.roarm.ListMissionsResponse.TYPE_ID,
		WsProtocol.roarm.GetMissionResponse.TYPE_ID,
		WsProtocol.roarm.SaveMissionResponse.TYPE_ID,
		WsProtocol.roarm.DeleteMissionResponse.TYPE_ID,
		WsProtocol.roarm.GetMissionGpioListResponse.TYPE_ID,
	]);

	if (responseTypeIds.has(typeId)) {
		// requestId liegt bei JEDER Response-Nachricht an derselben Stelle (erstes Feld nach dem
		// 4-Byte-Kopf, s. Dateikommentar) -- generische Zuordnung ohne nachrichtentypspezifischen Code.
		const requestId = view.getUint16(4, true);
		const pending = pendingRequests.get(requestId);
		if (pending) {
			pendingRequests.delete(requestId);
			clearTimeout(pending.timer);
			pending.resolve(view);
		}
		return;
	}

	const subscribers = eventSubscribers.get(typeId);
	if (subscribers) {
		for (const cb of subscribers) cb(view);
	}
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
			// Nachrichtentext bewusst als ERSTES Argument (nicht der Zeitstempel-Praefix davor) --
			// eine lange Millisekundenzahl vor dem Text liess die ersten Zeichen der eigentlichen
			// Meldung in der Konsole abgeschnitten wirken, s. Feedback.
			consoleFnForLevel(msg.level)(msg.text, `(t=${msg.timestampMs}ms)`);
		} catch (error) {
			console.warn(`[diag] WS LogMessage decode FAILED: ${(error as Error).message}`, `frame length=${data.byteLength} bytes`, `\nhex=${hexDump(data)}`);
		}
		return;
	}

	if (namespaceId === WsProtocol.roarm.NAMESPACE_ID) {
		handleRoarmMessage(view, messageTypeId);
		return;
	}

	if (namespaceId === WsProtocol.tasks.NAMESPACE_ID && messageTypeId === WsProtocol.tasks.TaskListMessage.TYPE_ID) {
		if (taskListListener) {
			try {
				taskListListener(WsProtocol.tasks.TaskListMessage.decode(view, 0));
			} catch (error) {
				console.warn(`WS TaskListMessage decode fehlgeschlagen: ${(error as Error).message}`);
			}
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
	socket = ws;

	ws.addEventListener("open", () => {
		activeSocket = ws;
	});

	ws.addEventListener("message", (event) => {
		if (event.data instanceof ArrayBuffer) {
			handleMessage(event.data);
		}
	});

	ws.addEventListener("close", () => {
		if (socket === ws) socket = null;
		if (activeSocket === ws) activeSocket = null;
		setTimeout(connect, RECONNECT_DELAY_MS);
	});
	ws.addEventListener("error", () => {
		ws.close();
	});
}

export function startWebSocketClient(): void {
	connect();
}

/** Feuert-und-vergisst (Event-Nachrichten wie JointJogTarget/CartesianJogTarget) -- kein Fehler,
 * falls die Verbindung gerade nicht steht (z.B. waehrend eines Reconnects); die naechste
 * Jog-Nachricht kommt ohnehin in Kuerze wieder, ein einzelner verlorener Zwischenschritt ist
 * fuer ein Live-Jogging unkritisch. */
export function sendRoArmEvent(bytes: Uint8Array): void {
	socket?.send(bytes);
}

/** Request/Response mit generischer requestId-Zuordnung (s. Dateikommentar). 'encode' bekommt die
 * vom Aufrufer zu verwendende requestId (in das Payload-Objekt einzusetzen, bevor encode()
 * aufgerufen wird) und muss die fertigen Frame-Bytes zurueckgeben. */
export function roarmRequest<TPayload>(requestId_encode: (requestId: number) => Uint8Array, decode: (view: DataView) => TPayload): Promise<TPayload> {
	return new Promise((resolve, reject) => {
		if (!socket || socket.readyState !== WebSocket.OPEN) {
			reject(new Error("WebSocket nicht verbunden"));
			return;
		}
		const requestId = nextRequestId++ & 0xffff;
		const bytes = requestId_encode(requestId);
		const timer = window.setTimeout(() => {
			pendingRequests.delete(requestId);
			reject(new Error("Zeitüberschreitung bei WS-Anfrage"));
		}, REQUEST_TIMEOUT_MS);
		pendingRequests.set(requestId, { resolve: (view) => resolve(decode(view)), reject, timer });
		socket.send(bytes);
	});
}

/** Laufende Push-Nachrichten (aktuell nur roarm.PoseFeedback) -- Rueckgabewert ist eine
 * Unsubscribe-Funktion. */
export function subscribeRoArmEvent(typeId: number, cb: (view: DataView) => void): () => void {
	let set = eventSubscribers.get(typeId);
	if (!set) {
		set = new Set();
		eventSubscribers.set(typeId, set);
	}
	set.add(cb);
	return () => set!.delete(cb);
}
