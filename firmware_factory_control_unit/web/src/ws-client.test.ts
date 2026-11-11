// The WebSocket client against a fake socket: a dead connection (the device hangs or lost its link and
// sends no TCP close, so the socket stays "open") is given up after one request timeout - the status
// bar says "closed" and the reconnect starts -, while a device that only skips one answer keeps its
// connection.
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

class FakeSocket {
	static readonly OPEN = 1;
	static readonly CLOSED = 3;
	static all: FakeSocket[] = [];
	readyState = 0;
	binaryType = "";
	sent: Uint8Array[] = [];
	closeCalls = 0;
	private handlers = new Map<string, Array<(e: unknown) => void>>();

	constructor(readonly url: string) {
		FakeSocket.all.push(this);
	}
	addEventListener(type: string, h: (e: unknown) => void): void {
		this.handlers.set(type, [...(this.handlers.get(type) ?? []), h]);
	}
	send(bytes: Uint8Array): void {
		this.sent.push(bytes);
	}
	/** As a browser with a dead peer: no close event until the closing handshake gives up. */
	close(): void {
		this.closeCalls++;
	}
	emit(type: string, e: unknown = {}): void {
		if (type === "open") this.readyState = FakeSocket.OPEN;
		if (type === "close") this.readyState = FakeSocket.CLOSED;
		for (const h of this.handlers.get(type) ?? []) h(e);
	}
	/** A message of the device: namespace 99 (nobody listens), type 1. */
	message(): void {
		this.emit("message", { data: new Uint8Array([99, 0, 1, 0]).buffer });
	}
}

type Client = typeof import("./ws-client.ts");

async function startClient(): Promise<Client> {
	vi.resetModules();
	const client = await import("./ws-client.ts");
	client.startWebSocketClient();
	FakeSocket.all[0].emit("open");
	return client;
}

/** A request whose answer never comes. */
function request(client: Client): Promise<unknown> {
	return client.wsRequest(4, (id) => new Uint8Array([4, 0, 1, 0, id & 0xff, id >> 8]), { TYPE_ID: 2, decode: () => ({}) })
		.catch((e: Error) => e);
}

beforeEach(() => {
	FakeSocket.all = [];
	vi.useFakeTimers({ toFake: ["setTimeout", "clearTimeout", "performance"] });
	vi.stubGlobal("WebSocket", FakeSocket);
	vi.stubGlobal("window", globalThis);
	vi.stubGlobal("location", { host: "fcu.local" });
});

afterEach(() => {
	vi.useRealTimers();
	vi.unstubAllGlobals();
});

describe("ws-client: dead connections", () => {
	it("a request timeout with nothing received since: closed at once and reconnected", async () => {
		const client = await startClient();
		expect(client.wsState()).toBe("open");
		const answer = request(client);
		await vi.advanceTimersByTimeAsync(3000);
		expect(((await answer) as Error).message).toBe("Zeitüberschreitung bei WS-Anfrage");
		expect(FakeSocket.all[0].closeCalls).toBe(1);
		expect(client.wsState()).toBe("closed");  // without waiting for the browser's close event
		await vi.advanceTimersByTimeAsync(2000);
		expect(FakeSocket.all).toHaveLength(2);    // the reconnect

		// the late close event of the dead socket starts no second connection
		FakeSocket.all[0].emit("close");
		await vi.advanceTimersByTimeAsync(5000);
		expect(FakeSocket.all).toHaveLength(2);
	});

	it("a device that skips one answer but still sends keeps its connection", async () => {
		const client = await startClient();
		const answer = request(client);
		await vi.advanceTimersByTimeAsync(1000);
		FakeSocket.all[0].message();  // alive: e.g. a log record or another answer
		await vi.advanceTimersByTimeAsync(2000);
		expect(((await answer) as Error).message).toBe("Zeitüberschreitung bei WS-Anfrage");
		expect(FakeSocket.all[0].closeCalls).toBe(0);
		expect(client.wsState()).toBe("open");
		expect(FakeSocket.all).toHaveLength(1);
	});
});
