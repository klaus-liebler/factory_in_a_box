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
		this.receive(frame(99, 1, 0, 0));
	}
	receive(bytes: Uint8Array): void {
		this.emit("message", { data: bytes.slice().buffer });
	}
}

/** A frame: header (namespace, type, correlation, flags) + body. */
function frame(ns: number, type: number, correlation: number, flags: number, ...body: number[]): Uint8Array {
	const b = new Uint8Array(8 + body.length);
	const v = new DataView(b.buffer);
	[ns, type, correlation, flags].forEach((x, i) => v.setUint16(i * 2, x, true));
	b.set(body, 8);
	return b;
}

/** The correlation of a sent frame. */
function correlationOf(bytes: Uint8Array): number {
	return new DataView(bytes.buffer, bytes.byteOffset).getUint16(4, true);
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
	return client.wsRequest(4, (id) => frame(4, 1, id, 0), { TYPE_ID: 2, decode: () => ({}) })
		.catch((e: Error) => e);
}

/** A request of namespace 4 / type 1, answered by type 2; decode() returns the answer's first body byte. */
function askByte(client: Client): Promise<number> {
	return client.wsRequest(4, (id) => frame(4, 1, id, 0), { TYPE_ID: 2, decode: (v: DataView) => v.getUint8(8) });
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

describe("ws-client: correlation in the frame header", () => {
	it("answers are matched by their correlation, in any order, and never reach the namespace listener", async () => {
		const client = await startClient();
		const pushed: number[] = [];
		client.registerWsProtocolListener(4, { onWsMessage: (_type, v) => pushed.push(v.getUint8(8)) });
		const first = askByte(client);
		const second = askByte(client);
		await vi.advanceTimersByTimeAsync(0);  // wsRequest() sends after an await
		const [c1, c2] = FakeSocket.all[0].sent.map(correlationOf);
		expect(c1).not.toBe(0);  // 0 means "no correlation"
		expect(c2).not.toBe(c1);
		FakeSocket.all[0].receive(frame(4, 2, c2, 0, 22));  // the second answer first
		FakeSocket.all[0].receive(frame(4, 2, 0, 0, 77));   // a push: correlation 0
		FakeSocket.all[0].receive(frame(4, 2, c1, 0, 11));
		expect(await first).toBe(11);
		expect(await second).toBe(22);
		expect(pushed).toEqual([77]);
	});

	it("an error reply rejects at once with the device's code and message, no timeout", async () => {
		const client = await startClient();
		const answer = askByte(client).catch((e: unknown) => e);
		await vi.advanceTimersByTimeAsync(0);
		const c = correlationOf(FakeSocket.all[0].sent[0]);
		const message = [...new TextEncoder().encode("busy")];
		FakeSocket.all[0].receive(frame(4, 1, c, 1, 3, 0, ...message, 0));  // FLAG_ERROR, code 3 (Busy)
		const error = await answer;
		expect(error).toBeInstanceOf(client.WsErrorReply);
		expect((error as InstanceType<Client["WsErrorReply"]>).code).toBe(3);
		expect((error as Error).message).toContain("busy");
		await vi.advanceTimersByTimeAsync(5000);
		expect(FakeSocket.all[0].closeCalls).toBe(0);  // answered: the connection is alive
	});

	it("an answer of another type than expected rejects instead of being decoded wrongly", async () => {
		const client = await startClient();
		const answer = askByte(client).catch((e: unknown) => e);
		await vi.advanceTimersByTimeAsync(0);
		FakeSocket.all[0].receive(frame(4, 9, correlationOf(FakeSocket.all[0].sent[0]), 0, 1));
		expect(((await answer) as Error).message).toContain("falschen Typ");
	});
});
