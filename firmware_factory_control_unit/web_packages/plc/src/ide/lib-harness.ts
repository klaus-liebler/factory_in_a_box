// Test helper for the ST libraries (VDI3814, SIM): compiles a PLC_PRG with the device's sources and runs it
// cycle by cycle in the TypeScript VM with a given cycle time.
import { compile, type DebugInfo } from "../compiler/compiler.ts";
import { formatDiagnostic } from "../compiler/diagnostics.ts";
import { Vm } from "../vm.ts";
import { ioBindings, ioMapHash } from "./io-list.ts";
import { deviceSources } from "./project.ts";
import { readValue, writeValue, type Value } from "./values.ts";

export class LibRun {
	private readonly data = new Uint8Array(32 * 1024);
	private readonly vm = new Vm(this.data);
	private readonly debug: DebugInfo;
	/** PLC time [ms]. */
	now = 0;

	/** `vars`: the VAR section's lines, `body`: the statements of PLC_PRG. */
	constructor(vars: string, body: string, readonly dtMs = 50) {
		const text = `PROGRAM PLC_PRG\nVAR\n${vars}\nEND_VAR\n${body}\nEND_PROGRAM\n`;
		const r = compile(deviceSources("PLC_PRG", text), undefined, { io: ioBindings(), ioMapHash: ioMapHash() });
		if (!r.ok) throw new Error(r.diagnostics.map(formatDiagnostic).join("\n"));
		const e = this.vm.load(r.image!);
		if (e !== "none") throw new Error(`load: ${e}`);
		this.debug = r.debug!;
	}

	private sym(path: string) {
		const s = this.debug.resolve(`PLC_PRG.${path}`) ?? this.debug.resolve(path);
		if (!s) throw new Error(`no variable ${path}`);
		return s;
	}

	get(path: string): Value {
		return readValue(this.data, this.sym(path))!;
	}
	num(path: string): number {
		return Number(this.get(path));
	}
	set(path: string, v: Value): this {
		writeValue(this.data, this.sym(path), v);
		return this;
	}

	/** n cycles (default 1); throws on a runtime error. */
	cycle(n = 1): this {
		for (let i = 0; i < n; i++) {
			const r = this.vm.runCycle(0, this.now >>> 0);
			if (r.state !== "done") throw new Error(`cycle at ${this.now} ms: ${r.state} trap ${r.trap} pc ${r.pc}`);
			this.now += this.dtMs;
		}
		return this;
	}

	/** Runs for `ms` of PLC time. */
	run(ms: number): this {
		return this.cycle(Math.round(ms / this.dtMs));
	}
}
