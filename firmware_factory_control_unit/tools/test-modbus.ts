#!/usr/bin/env node
// Testwerkzeug fuer den Modbus-TCP-Server der Control-Unit -- Node/TypeScript-Pendant zu
// test_modbus.py (das weiterhin fuer pymodbus-Nutzer bestehen bleibt). Bewusst ohne externe
// Abhaengigkeiten (keine node_modules noetig): Node fuehrt .ts-Dateien per eingebautem
// Type-Stripping direkt aus (ab Node 22.6 mit Flag, ab Node 23.6 ungeflaggt) -- die
// Modbus-TCP-PDU-Kodierung (MBAP-Header + FC03/04/06) ist hier von Hand implementiert, es
// gibt kein pymodbus-Aequivalent in Nodes Standardbibliothek. Nur erasable TS-Syntax
// verwendet (keine Parameter-Properties/Enums/Namespaces), damit das Type-Stripping ohne
// Transform auskommt.
//
// Beispiele:
//   node tools/test-modbus.ts --host 192.168.1.50
//   node tools/test-modbus.ts --host 192.168.1.50 --set-valve 1 on
//   node tools/test-modbus.ts --host 192.168.1.50 --set-valve 1 off
//   node tools/test-modbus.ts --host 192.168.1.50 --pwm-test

import { Socket } from "node:net";

const MODBUS_PORT = 502;

// Register-Adressen, siehe Core/Src/generated/register_input.inc / register_holding.inc
const INPUT_HEALTH_STATE = 0;
const INPUT_CHIP_ID_W0_HI = 1;
const INPUT_FW_VERSION_MAJOR = 7;
const INPUT_TIMER_TICK = 10;
const INPUT_ETH_LINK_STATUS = 80;
const INPUT_LIGHTBARRIER1 = 180;
const INPUT_PRESSURE_RAW = 200;

const HOLDING_VALVE1 = 20;
const HOLDING_CONVEYOR_PWM = 40;
const HOLDING_COMPRESSOR_PWM = 41;

// Rampen-Sequenz fuer --pwm-test, Werte in Promille (0..1000), im Halbsekunden-Takt gesendet.
const PWM_TEST_COMPRESSOR_SEQUENCE = [0, 250, 500, 750, 999, 0];
const PWM_TEST_CONVEYOR_SEQUENCE = [999, 750, 500, 250, 0, 0];
const PWM_TEST_INTERVAL_MS = 500;

const HEALTH_BITS: Record<number, string> = {
	0: "OK",
	1: "OVERTEMPERATURE",
	2: "POWER_FAULT",
	3: "CAN_ERROR",
	4: "ETH_LINK_DOWN",
	5: "I2C_SENSOR_FAULT",
	6: "STEPPER_FAULT"
};

const ETH_SPEEDS: Record<number, string> = { 0: "10M", 1: "100M", 2: "1000M" };

// ---------------------------------------------------------------------------
// Minimaler Modbus-TCP-Client (MBAP-Header + FC03/04/06). Ein Request ist immer
// abgeschlossen, bevor der naechste gesendet wird -- kein Pipelining noetig fuer dieses Tool.
// ---------------------------------------------------------------------------

class ModbusError extends Error {}

class ModbusTcpClient {
	private host: string;
	private port: number;
	private socket: Socket | null = null;
	private transactionId = 0;
	private rxBuffer: Buffer = Buffer.alloc(0);
	private pendingResolve: ((frame: Buffer) => void) | null = null;
	private pendingReject: ((err: Error) => void) | null = null;

	constructor(host: string, port: number) {
		this.host = host;
		this.port = port;
	}

	connect(): Promise<void> {
		return new Promise((resolve, reject) => {
			const socket = new Socket();
			this.socket = socket;
			socket.once("error", reject);
			socket.once("connect", () => {
				socket.off("error", reject);
				socket.on("data", (chunk: Buffer) => this.onData(chunk));
				socket.on("error", (err: Error) => this.failPending(err));
				socket.on("close", () => this.failPending(new ModbusError("Verbindung geschlossen")));
				resolve();
			});
			socket.connect(this.port, this.host);
		});
	}

	close(): void {
		this.socket?.destroy();
	}

	private onData(chunk: Buffer): void {
		this.rxBuffer = Buffer.concat([this.rxBuffer, chunk]);
		if (this.rxBuffer.length < 7) return;
		const length = this.rxBuffer.readUInt16BE(4);
		const frameLength = 6 + length;
		if (this.rxBuffer.length < frameLength) return;

		const frame = this.rxBuffer.subarray(0, frameLength);
		this.rxBuffer = this.rxBuffer.subarray(frameLength);
		const resolve = this.pendingResolve;
		this.pendingResolve = null;
		this.pendingReject = null;
		resolve?.(frame);
	}

	private failPending(err: Error): void {
		const reject = this.pendingReject;
		this.pendingResolve = null;
		this.pendingReject = null;
		reject?.(err);
	}

	private sendRequest(unitId: number, pdu: Buffer): Promise<Buffer> {
		if (!this.socket) throw new ModbusError("Nicht verbunden");
		this.transactionId = (this.transactionId + 1) & 0xffff;

		const mbap = Buffer.alloc(7);
		mbap.writeUInt16BE(this.transactionId, 0);
		mbap.writeUInt16BE(0, 2); // Protocol ID
		mbap.writeUInt16BE(pdu.length + 1, 4); // Length = PDU + Unit ID
		mbap.writeUInt8(unitId, 6);

		const socket = this.socket;
		return new Promise((resolve, reject) => {
			this.pendingResolve = resolve;
			this.pendingReject = reject;
			socket.write(Buffer.concat([mbap, pdu]));
		});
	}

	async readHoldingRegisters(address: number, quantity: number): Promise<number[]> {
		return this.readRegisters(0x03, address, quantity);
	}

	async readInputRegisters(address: number, quantity: number): Promise<number[]> {
		return this.readRegisters(0x04, address, quantity);
	}

	private async readRegisters(functionCode: number, address: number, quantity: number): Promise<number[]> {
		const pdu = Buffer.alloc(5);
		pdu.writeUInt8(functionCode, 0);
		pdu.writeUInt16BE(address, 1);
		pdu.writeUInt16BE(quantity, 3);

		const response = await this.sendRequest(1, pdu);
		const responsePdu = response.subarray(7);
		checkException(responsePdu, functionCode, `Read (FC${functionCode}) @${address}`);

		const byteCount = responsePdu.readUInt8(1);
		const registers: number[] = [];
		for (let i = 0; i < byteCount / 2; i++) {
			registers.push(responsePdu.readUInt16BE(2 + i * 2));
		}
		return registers;
	}

	async writeRegister(address: number, value: number): Promise<void> {
		const pdu = Buffer.alloc(5);
		pdu.writeUInt8(0x06, 0);
		pdu.writeUInt16BE(address, 1);
		pdu.writeUInt16BE(value, 3);

		const response = await this.sendRequest(1, pdu);
		checkException(response.subarray(7), 0x06, `Write Register @${address}`);
	}
}

function checkException(responsePdu: Buffer, expectedFunctionCode: number, what: string): void {
	const functionCode = responsePdu.readUInt8(0);
	if (functionCode === (expectedFunctionCode | 0x80)) {
		const exceptionCode = responsePdu.readUInt8(1);
		throw new ModbusError(`FEHLER bei ${what}: Modbus-Exception 0x${exceptionCode.toString(16)}`);
	}
	if (functionCode !== expectedFunctionCode) {
		throw new ModbusError(`FEHLER bei ${what}: unerwarteter Function Code 0x${functionCode.toString(16)}`);
	}
}

// ---------------------------------------------------------------------------
// Kommandos
// ---------------------------------------------------------------------------

async function dumpStatus(client: ModbusTcpClient): Promise<void> {
	const health = (await client.readInputRegisters(INPUT_HEALTH_STATE, 1))[0]!;
	const flags = Object.entries(HEALTH_BITS)
		.filter(([bit]) => health & (1 << Number(bit)))
		.map(([, name]) => name);
	console.log(`HEALTH_STATE   : 0x${health.toString(16).padStart(4, "0")}  [${flags.length ? flags.join(", ") : "(keine Flags)"}]`);

	const chipIdWords = await client.readInputRegisters(INPUT_CHIP_ID_W0_HI, 6);
	const uidHex = chipIdWords.map((w) => w.toString(16).padStart(4, "0")).join("");
	console.log(`CHIP_ID        : ${uidHex}`);

	const fw = await client.readInputRegisters(INPUT_FW_VERSION_MAJOR, 3);
	console.log(`FW_VERSION     : ${fw[0]}.${fw[1]}.${fw[2]}`);

	const tick0 = (await client.readInputRegisters(INPUT_TIMER_TICK, 1))[0]!;
	console.log(`TIMER_TICK     : ${tick0}`);

	const eth = await client.readInputRegisters(INPUT_ETH_LINK_STATUS, 3);
	const linkUp = eth[0] ? "up" : "down";
	const speed = ETH_SPEEDS[eth[1]!] ?? `? (${eth[1]})`;
	const duplex = eth[2] ? "full" : "half";
	console.log(`ETH_LINK       : ${linkUp}, ${speed}, ${duplex}`);

	const ls = await client.readInputRegisters(INPUT_LIGHTBARRIER1, 2);
	console.log(`LIGHTBARRIER1/2: ${ls[0]} / ${ls[1]}`);

	const pressure = (await client.readInputRegisters(INPUT_PRESSURE_RAW, 1))[0]!;
	console.log(`PRESSURE_RAW   : ${pressure}`);

	const valves = await client.readHoldingRegisters(HOLDING_VALVE1, 3);
	console.log(`VALVE1/2/3     : ${valves[0]} / ${valves[1]} / ${valves[2]}`);
}

async function setValve(client: ModbusTcpClient, valveNumber: number, state: boolean): Promise<void> {
	const address = HOLDING_VALVE1 + (valveNumber - 1);
	await client.writeRegister(address, state ? 1 : 0);
	const readback = (await client.readHoldingRegisters(address, 1))[0]!;
	console.log(`VALVE${valveNumber} gesetzt auf ${state ? "AN" : "AUS"} (Register liest jetzt: ${readback})`);
}

function sleep(ms: number): Promise<void> {
	return new Promise((resolve) => setTimeout(resolve, ms));
}

async function pwmRampTest(client: ModbusTcpClient): Promise<void> {
	console.log("PWM-Rampentest laeuft (Strg+C zum Beenden) ...");
	let stopped = false;
	const onSigint = () => {
		stopped = true;
	};
	process.on("SIGINT", onSigint);
	try {
		while (!stopped) {
			for (let i = 0; i < PWM_TEST_COMPRESSOR_SEQUENCE.length && !stopped; i++) {
				const compressor = PWM_TEST_COMPRESSOR_SEQUENCE[i]!;
				const conveyor = PWM_TEST_CONVEYOR_SEQUENCE[i]!;
				await client.writeRegister(HOLDING_COMPRESSOR_PWM, compressor);
				await client.writeRegister(HOLDING_CONVEYOR_PWM, conveyor);
				console.log(`COMPRESSOR_PWM=${String(compressor).padStart(4)}  CONVEYOR_PWM=${String(conveyor).padStart(4)}`);
				await sleep(PWM_TEST_INTERVAL_MS);
			}
		}
	} finally {
		process.off("SIGINT", onSigint);
		console.log("\nPWM-Rampentest beendet, setze beide Register auf 0.");
		await client.writeRegister(HOLDING_COMPRESSOR_PWM, 0);
		await client.writeRegister(HOLDING_CONVEYOR_PWM, 0);
	}
}

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

interface ParsedArgs {
	host: string;
	port: number;
	setValve: { valveNumber: number; state: boolean } | null;
	pwmTest: boolean;
}

function printUsageAndExit(message?: string): never {
	if (message) console.error(message + "\n");
	console.log(
		"Testwerkzeug fuer den Modbus-TCP-Server der Control-Unit.\n\n" +
			"Verwendung:\n" +
			"  node tools/test-modbus.ts --host <ip> [--port <port>] [--set-valve N on|off] [--pwm-test]\n\n" +
			"Beispiele:\n" +
			"  node tools/test-modbus.ts --host 192.168.1.50\n" +
			"  node tools/test-modbus.ts --host 192.168.1.50 --set-valve 1 on\n" +
			"  node tools/test-modbus.ts --host 192.168.1.50 --set-valve 1 off\n" +
			"  node tools/test-modbus.ts --host 192.168.1.50 --pwm-test"
	);
	process.exit(message ? 1 : 0);
}

function parseArgs(argv: string[]): ParsedArgs {
	let host: string | null = null;
	let port = MODBUS_PORT;
	let setValve: ParsedArgs["setValve"] = null;
	let pwmTest = false;

	for (let i = 0; i < argv.length; i++) {
		const arg = argv[i];
		switch (arg) {
			case "--host":
				host = argv[++i] ?? null;
				break;
			case "--port":
				port = Number(argv[++i]);
				break;
			case "--set-valve": {
				const valveNumber = Number(argv[++i]);
				const stateStr = (argv[++i] ?? "").toLowerCase();
				if (![1, 2, 3].includes(valveNumber) || !["on", "off"].includes(stateStr)) {
					printUsageAndExit("--set-valve erwartet: N in {1,2,3} und on/off");
				}
				setValve = { valveNumber, state: stateStr === "on" };
				break;
			}
			case "--pwm-test":
				pwmTest = true;
				break;
			case "--help":
			case "-h":
				printUsageAndExit();
				break;
			default:
				printUsageAndExit(`Unbekanntes Argument: ${arg}`);
		}
	}

	if (!host) printUsageAndExit("--host ist erforderlich");
	return { host, port, setValve, pwmTest };
}

async function main(): Promise<void> {
	const args = parseArgs(process.argv.slice(2));

	const client = new ModbusTcpClient(args.host, args.port);
	try {
		await client.connect();
	} catch (err) {
		console.error(`Verbindung zu ${args.host}:${args.port} fehlgeschlagen: ${(err as Error).message}`);
		process.exit(1);
	}

	try {
		if (args.setValve) {
			await setValve(client, args.setValve.valveNumber, args.setValve.state);
		} else if (args.pwmTest) {
			await pwmRampTest(client);
		} else {
			await dumpStatus(client);
		}
	} catch (err) {
		console.error((err as Error).message);
		process.exitCode = 1;
	} finally {
		client.close();
	}
}

main();
