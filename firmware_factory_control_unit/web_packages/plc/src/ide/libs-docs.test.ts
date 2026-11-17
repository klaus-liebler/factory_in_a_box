// The library documentation (docs/vdi3814-library.md, docs/sim-library.md) describes every block with every
// input and output, and its example compiles - so the documentation cannot silently drift from the code.
import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { compile } from "../compiler/compiler.ts";
import { Diagnostics, formatDiagnostic } from "../compiler/diagnostics.ts";
import { parse } from "../compiler/parser.ts";
import { ioBindings, ioMapHash } from "./io-list.ts";
import { deviceSources } from "./project.ts";
import { simLibSource } from "./sim-lib.ts";
import { vdi3814LibSource } from "./vdi3814-lib.ts";

const doc = (name: string) => readFileSync(new URL(`../../../../docs/${name}`, import.meta.url), "utf8");

/** The documentation section of `### NS.NAME` up to the next heading of level <= 3. */
function section(text: string, title: string): string | null {
	const at = text.search(new RegExp(`^### ${title.replace(".", "\\.")}\\b`, "m"));
	if (at < 0) return null;
	const rest = text.slice(at + 4);
	const end = rest.search(/^#{1,3} /m);
	return end < 0 ? rest : rest.slice(0, end);
}

/** `Q2` is documented by a range of its numbered siblings: `Q1`..`Q4`, `En1`/`V1` … `En8`/`V8`. */
function inRange(s: string, name: string): boolean {
	const m = /^(.*?)(\d+)$/.exec(name);
	if (!m || !/\.\.|…/.test(s)) return false;
	const nums = [...s.matchAll(new RegExp(`\`${m[1]}(\\d+)\``, "g"))].map((x) => Number(x[1]));
	return nums.length >= 2 && Math.min(...nums) <= Number(m[2]) && Number(m[2]) <= Math.max(...nums);
}

for (const [ns, source, file] of [["VDI3814", vdi3814LibSource(), "vdi3814-library.md"], ["SIM", simLibSource(), "sim-library.md"]] as const) {
	describe(`${file} documents the ${ns} library`, () => {
		const text = doc(file);
		const pous = parse(source, `<${ns}>`, new Diagnostics()).filter((u) => u.kind === "pou" && !u.name.startsWith("__"));
		for (const p of pous) {
			if (p.kind !== "pou") continue;
			it(`${ns}.${p.name}`, () => {
				const s = section(text, `${ns}.${p.name}`);
				expect(s, `no section "### ${ns}.${p.name}"`).not.toBeNull();
				if (p.pouKind === "FUNCTION") return;  // a function's signature is in its title line
				for (const v of p.vars.filter((x) => x.cls === "VAR_INPUT" || x.cls === "VAR_OUTPUT")) {
					expect(s!.includes(`\`${v.name}\``) || new RegExp(`\`[^\`]*\\b${v.name}\\b[^\`]*\``).test(s!) || inRange(s!, v.name),
						`${ns}.${p.name}.${v.name} not documented`).toBe(true);
				}
			});
		}
	});
}

it("the example of sim-library.md compiles", () => {
	const m = /```iecst\r?\n([\s\S]*?)```/.exec(doc("sim-library.md"));
	expect(m).not.toBeNull();
	const r = compile(deviceSources("PLC_PRG", m![1]), undefined, { io: ioBindings(), ioMapHash: ioMapHash() });
	expect(r.ok, r.diagnostics.map(formatDiagnostic).join("\n")).toBe(true);
});
