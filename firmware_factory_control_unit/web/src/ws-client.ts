// Bidirektionale binaere WebSocket-Verbindung zum Server (s. docs/websocket-protocol.md,
// Core/Src/http_websocket_server.hpp) -- mittelfristig der einzige Kanal fuer laufenden
// Datenaustausch zwischen Browser und Firmware; HTTP GET dient nur noch dem einmaligen Laden
// dieser Seite. Jede Binaerframe traegt genau eine Nachricht mit 4-Byte-Kopf
// (namespaceId/messageTypeId, s. ws-protocol/*.json); decode() der generierten Typen aus
// web/generated/ws-protocol.ts erwartet den KOMPLETTEN Frame inkl. Kopf plus offset=0.
//
// Dieses Modul ist reiner, App-neutraler Transport -- es kennt keinen einzigen konkreten
// Nachrichtentyp:
// - Eingehende Nachrichten werden ausschliesslich anhand ihrer namespaceId an den EINEN dafuer
//   registrierten WsProtocolListener weitergereicht (registerWsProtocolListener()). Decodieren,
//   Verteilen innerhalb der App und Behandeln unbekannter messageTypeIds ist Sache der App, die
//   den Namespace besitzt (z.B. roarm -> WsRoArmBackend, tasks -> task-manager-app.ts).
// - Request/Response (wsRequest()): das generierte Payload jeder Request- und Response-Nachricht
//   traegt als ERSTES Feld requestId (uint16, direkt hinter dem 4-Byte-Kopf). Die Zuordnung einer
//   Antwort zum wartenden Promise braucht daher nur requestId plus die vom Aufrufer genannte
//   erwartete (namespaceId, TYPE_ID) -- Antworten auf laufende wsRequest()-Aufrufe erreichen den
//   Namespace-Listener nicht.
// - Fire-and-forget (Events wie roarm.JointJogTarget, Polls wie tasks.TaskManagerRequest):
//   sendBinary().

// Reconnect-Backoff bewusst simpel/fest (kein exponentielles Backoff): das Board ist im
// bestimmungsgemaessen Betrieb dauerhaft im selben Netz erreichbar, ein kurzer fester Abstand
// haelt die Verbindung nach einem Boot/Reset/Netzwechsel zuegig wieder her, ohne bei einem
// tatsaechlich dauerhaft nicht erreichbaren Board die Konsole mit Reconnect-Versuchen zu fluten.
const RECONNECT_DELAY_MS = 2000;
const REQUEST_TIMEOUT_MS = 3000;
// Wie lange wsRequest() auf eine (noch) nicht offene Verbindung wartet, bevor es aufgibt. Der
// WebSocket-Aufbau kostet einen kompletten TLS-Handshake (RSA-2048-Signatur in Software, keine
// PKA auf dem H563) -- direkt nach dem Seitenladen fragen Screens (z.B. system-info-app.ts in
// onShow()) schon an, bevor der Socket offen ist; ein sofortiges "nicht verbunden" liess diese
// erste Anfrage ohne jede Wiederholung scheitern.
const CONNECT_WAIT_TIMEOUT_MS = 10000;

/** Von einer App fuer IHREN Namespace implementiert (s. registerWsProtocolListener()). */
export interface WsProtocolListener {
	/** Jede eingehende Nachricht des registrierten Namespaces, ausser Antworten auf laufende
	 * wsRequest()-Aufrufe. 'view' umfasst den kompletten Frame inkl. 4-Byte-Kopf (direkt an das
	 * generierte decode(view, 0) weiterreichbar). */
	onWsMessage(messageTypeId: number, view: DataView): void;
}

/** Generierte Response-Nachricht (z.B. system.SystemInfoMessage) -- nur die fuer wsRequest()
 * noetigen Teile. */
export interface WsResponseType<TPayload> {
	readonly TYPE_ID: number;
	decode(view: DataView, offset: number): TPayload;
}

function wsUrl(): string {
	return `wss://${location.host}/ws`;
}

let socket: WebSocket | null = null;
let nextRequestId = 1;
// Auf das naechste "open" wartende wsRequest()-Aufrufe (s. waitForOpen()).
let openWaiters: Array<() => void> = [];
const listeners = new Map<number, WsProtocolListener>(); // key = namespaceId
const pendingRequests = new Map<
	number, // key = requestId
	{ namespaceId: number; typeId: number; resolve: (view: DataView) => void; timer: number }
>();

function isOpen(): boolean {
	return socket !== null && socket.readyState === WebSocket.OPEN;
}

/** Registriert den einzigen Abnehmer fuer alle Nachrichten eines Namespaces. Rueckgabewert meldet
 * ihn wieder ab. Eine zweite Registrierung fuer denselben Namespace ist ein Programmierfehler
 * (wirft), damit nicht stillschweigend eine App einer anderen die Nachrichten wegnimmt. */
export function registerWsProtocolListener(namespaceId: number, listener: WsProtocolListener): () => void {
	if (listeners.has(namespaceId)) {
		throw new Error(`WebSocket: fuer namespaceId=${namespaceId} ist bereits ein Listener registriert`);
	}
	listeners.set(namespaceId, listener);
	return () => {
		if (listeners.get(namespaceId) === listener) listeners.delete(namespaceId);
	};
}

/** Feuert-und-vergisst -- kein Queueing bei fehlender Verbindung (bewusst): Polls und Jog-Events
 * kommen ohnehin in Kuerze erneut, ein einzelner verlorener Frame ist unkritisch. Liefert, ob
 * gesendet wurde. */
export function sendBinary(bytes: Uint8Array): boolean {
	if (!isOpen()) return false;
	socket!.send(bytes);
	return true;
}

function hexDump(data: ArrayBuffer): string {
	return Array.from(new Uint8Array(data), (b) => b.toString(16).padStart(2, "0")).join(" ");
}

// requestId liegt bei JEDER Response an derselben Stelle (erstes Feld nach dem 4-Byte-Kopf, s.
// Dateikommentar). true = Nachricht war die erwartete Antwort und ist damit verbraucht.
function tryResolvePendingRequest(view: DataView, namespaceId: number, messageTypeId: number): boolean {
	if (view.byteLength < 6) return false;
	const requestId = view.getUint16(4, true);
	const pending = pendingRequests.get(requestId);
	if (!pending || pending.namespaceId !== namespaceId || pending.typeId !== messageTypeId) return false;
	pendingRequests.delete(requestId);
	clearTimeout(pending.timer);
	pending.resolve(view);
	return true;
}

function handleMessage(data: ArrayBuffer): void {
	if (data.byteLength < 4) {
		console.warn(`[diag] WS frame too short for even the 4-byte header: ${data.byteLength} bytes, hex=${hexDump(data)}`);
		return;
	}
	const view = new DataView(data);
	const namespaceId = view.getUint16(0, true);
	const messageTypeId = view.getUint16(2, true);

	if (tryResolvePendingRequest(view, namespaceId, messageTypeId)) return;

	const listener = listeners.get(namespaceId);
	if (!listener) {
		// Kein Abnehmer -- z.B. ein Namespace, dessen App gerade nicht angezeigt wird, oder eine
		// neuere Firmware mit einem dieser Web-UI unbekannten Namespace. Nur geloggt, damit ein
		// einzelner unerwarteter Frame nicht die ganze Verbindung stoert.
		console.debug(`WebSocket: kein Listener fuer namespaceId=${namespaceId} (messageTypeId=${messageTypeId})`);
		return;
	}
	try {
		listener.onWsMessage(messageTypeId, view);
	} catch (error) {
		console.warn(`WebSocket: Listener fuer namespaceId=${namespaceId} warf bei messageTypeId=${messageTypeId}: ${(error as Error).message}`, `\nhex=${hexDump(data)}`);
	}
}

function connect(): void {
	const ws = new WebSocket(wsUrl());
	ws.binaryType = "arraybuffer";
	socket = ws;

	ws.addEventListener("open", () => {
		const waiters = openWaiters;
		openWaiters = [];
		for (const w of waiters) w();
	});

	ws.addEventListener("message", (event) => {
		if (event.data instanceof ArrayBuffer) {
			handleMessage(event.data);
		}
	});

	ws.addEventListener("close", () => {
		if (socket === ws) socket = null;
		setTimeout(connect, RECONNECT_DELAY_MS);
	});
	ws.addEventListener("error", () => {
		ws.close();
	});
}

export function startWebSocketClient(): void {
	connect();
}

function waitForOpen(timeoutMs: number): Promise<void> {
	if (isOpen()) return Promise.resolve();
	return new Promise((resolve, reject) => {
		const waiter = () => {
			window.clearTimeout(timer);
			resolve();
		};
		const timer = window.setTimeout(() => {
			openWaiters = openWaiters.filter((w) => w !== waiter);
			reject(new Error("WebSocket nicht verbunden"));
		}, timeoutMs);
		openWaiters.push(waiter);
	});
}

/** Request/Response mit generischer requestId-Zuordnung (s. Dateikommentar). 'encode' bekommt die
 * zu verwendende requestId (in das Payload-Objekt einzusetzen) und muss die fertigen Frame-Bytes
 * zurueckgeben; 'response' ist der generierte Antworttyp im Namespace 'namespaceId'. Wartet bei
 * noch nicht offener Verbindung bis zu CONNECT_WAIT_TIMEOUT_MS auf das Oeffnen. */
export async function wsRequest<TPayload>(
	namespaceId: number,
	encode: (requestId: number) => Uint8Array,
	response: WsResponseType<TPayload>,
): Promise<TPayload> {
	await waitForOpen(CONNECT_WAIT_TIMEOUT_MS);
	return new Promise((resolve, reject) => {
		if (!isOpen()) {
			reject(new Error("WebSocket nicht verbunden"));
			return;
		}
		const requestId = nextRequestId++ & 0xffff;
		const bytes = encode(requestId);
		const timer = window.setTimeout(() => {
			pendingRequests.delete(requestId);
			reject(new Error("Zeitüberschreitung bei WS-Anfrage"));
		}, REQUEST_TIMEOUT_MS);
		pendingRequests.set(requestId, {
			namespaceId,
			typeId: response.TYPE_ID,
			resolve: (view) => {
				try {
					resolve(response.decode(view, 0));
				} catch (error) {
					reject(error as Error);
				}
			},
			timer,
		});
		socket!.send(bytes);
	});
}
