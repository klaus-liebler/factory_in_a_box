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
// - Request/Response (roarm.* wie StartTeachMode/Mission-CRUD, modbus.* wie GetRegisters/
//   WriteHolding, system.SystemInfoRequest/-Message): das generierte Payload traegt als ERSTES
//   Feld immer requestId (uint16, direkt hinter dem 4-Byte-Kopf) -- wsRequest() nutzt das aus, um
//   eine Antwort anhand ihrer requestId dem wartenden Promise zuzuordnen, unabhaengig vom
//   konkreten Namespace/Nachrichtentyp (der gemeinsame requestId-Zaehler/pendingRequests-Map ist
//   bewusst NICHT nach Namespace aufgeteilt).
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

type PoolListListener = (payload: WsProtocol.tasks.PoolListMessage.Payload) => void;
let poolListListener: PoolListListener | null = null;

// Gleiches Muster wie setTaskListListener() oben -- task-manager-app.ts sendet
// tasks.PoolListRequest im selben Poll-Tick wie tasks.TaskManagerRequest.
export function setPoolListListener(listener: PoolListListener | null): void {
	poolListListener = listener;
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

// requestId liegt bei JEDER Response-Nachricht an derselben Stelle (erstes Feld nach dem 4-Byte-
// Kopf, s. Dateikommentar) -- generische Zuordnung ohne nachrichtentyp-/namespacespezifischen Code.
function resolvePendingRequest(view: DataView): void {
	const requestId = view.getUint16(4, true);
	const pending = pendingRequests.get(requestId);
	if (pending) {
		pendingRequests.delete(requestId);
		clearTimeout(pending.timer);
		pending.resolve(view);
	}
}

// Je Namespace EIN Handler, intern per switch/case auf messageTypeId verzweigend, mit
// einheitlichem "unbekannter Typ"-Logging im default-Zweig -- s. handleMessage() fuer die
// namespaceId-Ebene, die nach demselben Muster aufgebaut ist.

function handleSystemMessage(view: DataView, typeId: number, data: ArrayBuffer): void {
	switch (typeId) {
		case WsProtocol.system.LogMessage.TYPE_ID:
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
		case WsProtocol.system.SystemInfoMessage.TYPE_ID:
			resolvePendingRequest(view);
			return;
		default:
			console.debug(`WebSocket: unbekannte system-Nachricht typeId=${typeId}`);
	}
}

function handleRoarmMessage(view: DataView, typeId: number): void {
	switch (typeId) {
		case WsProtocol.roarm.StartTeachModeResponse.TYPE_ID:
		case WsProtocol.roarm.StopTeachModeResponse.TYPE_ID:
		case WsProtocol.roarm.ListMissionsResponse.TYPE_ID:
		case WsProtocol.roarm.GetMissionResponse.TYPE_ID:
		case WsProtocol.roarm.SaveMissionResponse.TYPE_ID:
		case WsProtocol.roarm.DeleteMissionResponse.TYPE_ID:
		case WsProtocol.roarm.GetMissionGpioListResponse.TYPE_ID:
			resolvePendingRequest(view);
			return;
		default: {
			// Kein Request/Response, sondern ein Event (JointJogTarget/CartesianJogTarget als
			// Client->Firmware werden hier nie ankommen; PoseFeedback als Firmware->Client schon) --
			// s. subscribeRoArmEvent() unten.
			const subscribers = eventSubscribers.get(typeId);
			if (subscribers) {
				for (const cb of subscribers) cb(view);
			} else {
				console.debug(`WebSocket: unbekannte roarm-Nachricht typeId=${typeId}`);
			}
		}
	}
}

function handleModbusMessage(view: DataView, typeId: number): void {
	switch (typeId) {
		case WsProtocol.modbus.RegistersMessage.TYPE_ID:
		case WsProtocol.modbus.WriteHoldingResponse.TYPE_ID:
			resolvePendingRequest(view);
			return;
		default:
			console.debug(`WebSocket: unbekannte modbus-Nachricht typeId=${typeId}`);
	}
}

function handleTasksMessage(view: DataView, typeId: number): void {
	switch (typeId) {
		case WsProtocol.tasks.TaskListMessage.TYPE_ID:
			if (taskListListener) {
				try {
					taskListListener(WsProtocol.tasks.TaskListMessage.decode(view, 0));
				} catch (error) {
					console.warn(`WS TaskListMessage decode fehlgeschlagen: ${(error as Error).message}`);
				}
			}
			return;
		case WsProtocol.tasks.PoolListMessage.TYPE_ID:
			if (poolListListener) {
				try {
					poolListListener(WsProtocol.tasks.PoolListMessage.decode(view, 0));
				} catch (error) {
					console.warn(`WS PoolListMessage decode fehlgeschlagen: ${(error as Error).message}`);
				}
			}
			return;
		default:
			console.debug(`WebSocket: unbekannte tasks-Nachricht typeId=${typeId}`);
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

	switch (namespaceId) {
		case WsProtocol.system.NAMESPACE_ID:
			return handleSystemMessage(view, messageTypeId, data);
		case WsProtocol.roarm.NAMESPACE_ID:
			return handleRoarmMessage(view, messageTypeId);
		case WsProtocol.modbus.NAMESPACE_ID:
			return handleModbusMessage(view, messageTypeId);
		case WsProtocol.tasks.NAMESPACE_ID:
			return handleTasksMessage(view, messageTypeId);
		default:
			// Unbekannte namespaceId -- z.B. eine neuere Firmware-Version mit einer Nachricht, die
			// dieser Web-UI-Build noch nicht kennt. Bewusst nur geloggt statt geworfen, damit ein
			// einzelner unbekannter Nachrichtentyp nicht die ganze Verbindung stoert.
			console.debug(`WebSocket: unbekannte Nachricht namespaceId=${namespaceId} messageTypeId=${messageTypeId}`);
	}
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

/** Request/Response mit generischer requestId-Zuordnung (s. Dateikommentar), namespace-
 * unabhaengig (roarm, modbus und system.SystemInfoRequest nutzen dieselbe Funktion). 'encode'
 * bekommt die vom Aufrufer zu verwendende requestId (in das Payload-Objekt einzusetzen, bevor
 * encode() aufgerufen wird) und muss die fertigen Frame-Bytes zurueckgeben. */
export function wsRequest<TPayload>(requestId_encode: (requestId: number) => Uint8Array, decode: (view: DataView) => TPayload): Promise<TPayload> {
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
