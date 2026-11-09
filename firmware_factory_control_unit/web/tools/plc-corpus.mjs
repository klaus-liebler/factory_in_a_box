// Rewrites rtoos/tests/plc_corpus (images, run plans, traces of the differential tests) and the image
// of the firmware's Renode test (tools/plc-renode) from the
// current compiler and TypeScript VM: src/plc/corpus.test.ts with PLC_CORPUS_UPDATE=1.
import { spawnSync } from "node:child_process";

const r = spawnSync("npx", ["vitest", "run", "src/plc/corpus.test.ts", "src/plc/renode-fixture.test.ts"], {
	stdio: "inherit",
	shell: true,
	env: { ...process.env, PLC_CORPUS_UPDATE: "1" },
});
process.exit(r.status ?? 1);
