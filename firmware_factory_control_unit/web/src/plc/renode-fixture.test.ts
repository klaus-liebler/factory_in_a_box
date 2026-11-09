// The PLC image for the firmware's end-to-end test in Renode (tools/test-plc-renode.py): compiled
// with the I/O list of the register map, written to tools/plc-renode/ with the addresses of its
// variables. This test fails when the files are not what the current compiler produces;
// `npm run plc:corpus` (PLC_CORPUS_UPDATE=1) rewrites them.
import fs from "node:fs";
import path from "node:path";
import { describe, expect, it } from "vitest";
import { compile } from "./compiler/compiler.ts";
import { formatDiagnostic } from "./compiler/diagnostics.ts";
import { crc32 } from "./image.ts";
import { ioBindings, ioMapHash } from "./ide/io-list.ts";
import { deviceSources } from "./ide/project.ts";

const OUT_DIR = path.resolve(import.meta.dirname, "../../../tools/plc-renode");
const UPDATE = process.env.PLC_CORPUS_UPDATE === "1";

export const RENODE_PROGRAM = `PROGRAM PLC_PRG
VAR
    n : DINT;              (* cycles of the program *)
    tick : UINT;           (* a copy of the input TimerTick *)
    blink : TON;
    lamp : BOOL;
    home : FCU_StepperHome;  (* a command register: the start reaches the firmware, its answer the block *)
END_VAR
n := n + 1;
tick := TimerTick;
ConveyorPwm := 123;
Valve2 := TRUE;
Stepper1Target := 70000;   (* 32 bits: high word 1, low word 4464 *)
blink(IN := NOT blink.Q, PT := T#200ms);
IF blink.Q THEN lamp := NOT lamp; END_IF;
Valve3 := lamp;
home(Axis := 1, Execute := TRUE);
END_PROGRAM
`;

describe("PLC Renode fixture", () => {
	it("tools/plc-renode/test.img is up to date", () => {
		const r = compile(deviceSources("PLC_PRG", RENODE_PROGRAM), undefined,
			{ io: ioBindings(), ioMapHash: ioMapHash() });
		if (!r.ok) throw new Error(r.diagnostics.map(formatDiagnostic).join("\n"));
		const symbols = Object.fromEntries(["PLC_PRG.n", "PLC_PRG.tick", "PLC_PRG.lamp", "PLC_PRG.home.Busy", "PLC_PRG.home.Done", "PLC_PRG.home.Error",
			"PLC_PRG.home.ErrorID"].map((p) => [p, r.debug!.resolve(p)!.addr]));
		const meta = JSON.stringify({ bytes: r.image!.length, crc: crc32(r.image!), symbols }, null, 2) + "\n";
		const files: [string, Uint8Array][] = [["test.img", r.image!], ["test.json", new TextEncoder().encode(meta)]];
		for (const [f, bytes] of files) {
			const p = path.join(OUT_DIR, f);
			if (UPDATE) {
				fs.mkdirSync(OUT_DIR, { recursive: true });
				fs.writeFileSync(p, bytes);
			} else {
				expect(fs.existsSync(p) && Buffer.compare(fs.readFileSync(p), Buffer.from(bytes)) === 0, `${p} is out of date: npm run plc:corpus`).toBe(true);
			}
		}
	});
});
