// Rewrites rtoos/tests/plc_corpus (images, run plans, traces of the differential tests) from the
// current compiler and TypeScript VM: src/plc/corpus.test.ts with PLC_CORPUS_UPDATE=1.
import { spawnSync } from "node:child_process";

const r = spawnSync("npx", ["vitest", "run", "src/plc/corpus.test.ts"], {
	stdio: "inherit",
	shell: true,
	env: { ...process.env, PLC_CORPUS_UPDATE: "1" },
});
process.exit(r.status ?? 1);
