// Bidirectional binary WebSocket connection to the server (see docs/websocket-protocol.md, rtoos
// rtos/net/https_server.hpp) - the only channel for ongoing data exchange between browser and firmware; HTTP
// GET only loads this page once. Every binary frame carries exactly one message with an 8-byte header
// (namespaceId, messageTypeId, correlationId, flags; see schema/best_binary_buffers/*.cs); decode() of the
// generated types in web/generated/ws-protocol.ts expects the COMPLETE frame including the header plus offset=0.
//
// This module is pure, app-neutral transport - it knows no concrete message type:
// - Incoming messages are passed on by their namespaceId only, to the ONE WsProtocolListener registered for
// it (registerWsProtocolListener()). Decoding, dispatching within the app and handling unknown messageTypeIds
// is the job of the app that owns the namespace (e.g. roarm -> WsRoArmBackend, tasks -> task-manager-app.ts).
// - Request/response (wsRequest()): the request carries a correlationId (never 0) in its header, the device
// copies it into the response. Matching a response to the waiting promise needs nothing but that - responses
// to pending wsRequest() calls do not reach the namespace listener. A request the device cannot serve gets an
// error reply (flag FLAG_ERROR: code + message) with the same correlationId: the promise rejects at once with
// WsErrorReply instead of running into the timeout.
// - Fire-and-forget (events such as roarm.JointJogTarget, polls such as tasks.TaskManagerRequest):
// sendBinary().

import { decodeError, FLAG_ERROR, type FrameHeader, HEADER_SIZE, readHeader } from "../../generated/ws-protocol.ts";

// Reconnect backoff deliberately simple and fixed (no exponential backoff): in normal operation the board is
// permanently reachable in the same network; a short fixed interval restores the connection quickly after a
// boot/reset/network change without flooding the console with reconnect attempts when the board really is
// unreachable for good.
const RECONNECT_DELAY_MS = 2000;
const REQUEST_TIMEOUT_MS = 3000;
// How long wsRequest() waits for a connection that is not open (yet) before giving up. Opening the WebSocket
// costs a complete TLS handshake (RSA-2048 signature in software, no PKA on the H563) - right after the page
// loads, screens (e.g. the system workspace in onShow()) already ask before the socket is open; failing
// immediately with "not connected" made that first request fail without any retry.
const CONNECT_WAIT_TIMEOUT_MS = 10000;

/** Implemented by an app for ITS namespace (see registerWsProtocolListener()). */
export interface WsProtocolListener {
	/* Every incoming message of the registered namespace, except responses to pending wsRequest() calls. 'view'
	   covers the complete frame including the header (can be passed directly to the generated decode(view, 0)). */
	onWsMessage(messageTypeId: number, view: DataView): void;
}

/* Generated response message (e.g. system.SystemInfoMessage) - only the parts wsRequest() needs. */
export interface WsResponseType<TPayload> {
	readonly TYPE_ID: number;
	decode(view: DataView, offset: number): TPayload;
}

/** The device's error reply to a request (see ErrorCode in the generated protocol). */
export class WsErrorReply extends Error {
	constructor(
		readonly code: number,
		readonly deviceMessage: string,
	) {
		super(`Gerät meldet Fehler ${code}: ${deviceMessage}`);
	}
}

function wsUrl(): string {
	return `wss://${location.host}/ws`;
}

let socket: WebSocket | null = null;
/** Messages received from the device so far: a request that times out with
 *  nothing at all received since it was sent means a dead connection - a device that hangs or lost
 *  its link sends no TCP close, the socket would stay "open" for minutes. */
let messagesReceived = 0;
let lastCorrelationId = 0;

/** The connection for the status bar: connecting (also between two attempts), open, or closed
 *  (it was open and was lost; the next attempt follows after RECONNECT_DELAY_MS). */
export type WsState = "connecting" | "open" | "closed";
let state: WsState = "connecting";
const stateListeners = new Set<() => void>();
function setState(s: WsState): void {
	if (s === state) return;
	state = s;
	for (const l of stateListeners) l();
}
export function wsState(): WsState {
	return state;
}
/** Called on every change of wsState(); returns the function that ends it. */
export function onWsStateChange(listener: () => void): () => void {
	stateListeners.add(listener);
	return () => stateListeners.delete(listener);
}
// wsRequest() calls waiting for the next "open" (see waitForOpen()).
let openWaiters: Array<() => void> = [];
const listeners = new Map<number, WsProtocolListener>(); // key = namespaceId
const pendingRequests = new Map<
	number, // key = correlationId
	{ namespaceId: number; typeId: number; resolve: (view: DataView) => void; reject: (error: Error) => void; timer: number }
>();

/** 1..65535 - 0 means "no correlation" (events, pushes). */
function nextCorrelationId(): number {
	lastCorrelationId = (lastCorrelationId % 0xffff) + 1;
	return lastCorrelationId;
}

function isOpen(): boolean {
	return socket !== null && socket.readyState === WebSocket.OPEN;
}

/* Registers the only consumer of all messages of a namespace. The return value unregisters it. A second
   registration for the same namespace is a programming error (throws), so that one app cannot silently take
   the messages away from another. */
export function registerWsProtocolListener(namespaceId: number, listener: WsProtocolListener): () => void {
	if (listeners.has(namespaceId)) {
		throw new Error(`WebSocket: fuer namespaceId=${namespaceId} ist bereits ein Listener registriert`);
	}
	listeners.set(namespaceId, listener);
	return () => {
		if (listeners.get(namespaceId) === listener) listeners.delete(namespaceId);
	};
}

/* Fire-and-forget - deliberately no queueing while disconnected: polls and jog events come again shortly
   anyway, a single lost frame does no harm. Returns whether the frame was sent. */
export function sendBinary(bytes: Uint8Array): boolean {
	if (!isOpen()) return false;
	socket!.send(bytes);
	return true;
}

function hexDump(data: ArrayBuffer): string {
	return Array.from(new Uint8Array(data), (b) => b.toString(16).padStart(2, "0")).join(" ");
}

// true = the message answers a pending request (response or error reply) and is consumed.
function tryResolvePendingRequest(view: DataView, header: FrameHeader): boolean {
	if (header.correlationId === 0) return false;
	const pending = pendingRequests.get(header.correlationId);
	if (!pending) return false;
	pendingRequests.delete(header.correlationId);
	clearTimeout(pending.timer);
	if (header.flags & FLAG_ERROR) {
		const error = decodeError(view);
		pending.reject(new WsErrorReply(error.code, error.message));
	} else if (header.namespaceId !== pending.namespaceId || header.messageTypeId !== pending.typeId) {
		pending.reject(new Error(`WS-Antwort vom falschen Typ: ${header.namespaceId}/${header.messageTypeId}, erwartet ${pending.namespaceId}/${pending.typeId}`));
	} else {
		pending.resolve(view);
	}
	return true;
}

function handleMessage(data: ArrayBuffer): void {
	if (data.byteLength < HEADER_SIZE) {
		console.warn(`[diag] WS frame too short for even the header: ${data.byteLength} bytes, hex=${hexDump(data)}`);
		return;
	}
	const view = new DataView(data);
	const header = readHeader(view);
	const { namespaceId, messageTypeId } = header;

	if (tryResolvePendingRequest(view, header)) return;

	const listener = listeners.get(namespaceId);
	if (!listener) {
		// No consumer - e.g. a namespace whose app is not shown at the moment, or a newer firmware with a namespace
		// this web UI does not know. Only logged, so that a single unexpected frame does not disturb the whole
		// connection.
		console.debug(`WebSocket: kein Listener fuer namespaceId=${namespaceId} (messageTypeId=${messageTypeId})`);
		return;
	}
	try {
		listener.onWsMessage(messageTypeId, view);
	} catch (error) {
		console.warn(`WebSocket: Listener fuer namespaceId=${namespaceId} warf bei messageTypeId=${messageTypeId}: ${(error as Error).message}`, `\nhex=${hexDump(data)}`);
	}
}

/** Sockets given up as dead: their late close event (the browser waits for the closing handshake
 *  the dead peer never sends) must not start a second reconnect. */
const abandoned = new WeakSet<WebSocket>();

/** A dead connection: closed now, not when the browser's closing handshake times out. */
function abandon(ws: WebSocket): void {
	if (socket !== ws || abandoned.has(ws)) return;
	abandoned.add(ws);
	socket = null;
	ws.close();
	setState("closed");
	setTimeout(connect, RECONNECT_DELAY_MS);
}

function connect(): void {
	const ws = new WebSocket(wsUrl());
	ws.binaryType = "arraybuffer";
	socket = ws;

	ws.addEventListener("open", () => {
		setState("open");
		const waiters = openWaiters;
		openWaiters = [];
		for (const w of waiters) w();
	});

	ws.addEventListener("message", (event) => {
		messagesReceived++;
		if (event.data instanceof ArrayBuffer) {
			handleMessage(event.data);
		}
	});

	ws.addEventListener("close", () => {
		if (abandoned.has(ws)) return;  // already given up and replaced (abandon())
		if (socket === ws) socket = null;
		setState("closed");
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

/* Request/response matched by the correlationId (see the file comment). 'encode' gets the correlationId to use
   (the second argument of the generated encode()) and must return the finished frame bytes; 'response' is the
   generated response type in namespace 'namespaceId'. Rejects with WsErrorReply if the device answers with an
   error. If the connection is not open yet, waits up to CONNECT_WAIT_TIMEOUT_MS for it to open. */
export async function wsRequest<TPayload>(
	namespaceId: number,
	encode: (correlationId: number) => Uint8Array,
	response: WsResponseType<TPayload>,
	timeoutMs = REQUEST_TIMEOUT_MS,
): Promise<TPayload> {
	await waitForOpen(CONNECT_WAIT_TIMEOUT_MS);
	return new Promise((resolve, reject) => {
		if (!isOpen()) {
			reject(new Error("WebSocket nicht verbunden"));
			return;
		}
		const correlationId = nextCorrelationId();
		const bytes = encode(correlationId);
		const receivedBefore = messagesReceived;
		const sentOn = socket;
		const timer = window.setTimeout(() => {
			pendingRequests.delete(correlationId);
			// nothing at all from the device since the request: the connection is dead - close it, so
			// the status bar says so and the reconnect starts (a device that only skips this answer
			// keeps its connection)
			if (messagesReceived === receivedBefore && sentOn) abandon(sentOn);
			reject(new Error("Zeitüberschreitung bei WS-Anfrage"));
		}, timeoutMs);
		pendingRequests.set(correlationId, {
			namespaceId,
			typeId: response.TYPE_ID,
			resolve: (view) => {
				try {
					resolve(response.decode(view, 0));
				} catch (error) {
					reject(error as Error);
				}
			},
			reject,
			timer,
		});
		socket!.send(bytes);
	});
}
