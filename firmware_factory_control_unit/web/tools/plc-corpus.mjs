// Rewrites rtoos/tests/plc_corpus (images, run plans, traces of the differential tests) and the image
// of the firmware's Renode test (tools/plc-renode) from the current compiler and TypeScript VM:
// web_packages/plc/src/corpus.test.ts and web/src/plc-fcu/renode-fixture.test.ts with PLC_CORPUS_UPDATE=1.
import { spawnSync } from "node:child_process";
import path from "node:path";

const env = { ...process.env, PLC_CORPUS_UPDATE: "1" };
const web = path.resolve(import.meta.dirname, "..");
const plc = path.resolve(web, "..", "web_packages", "plc");
for (const [cwd, test] of [[plc, "src/corpus.test.ts"], [web, "src/plc-fcu/renode-fixture.test.ts"]]) {
	const r = spawnSync("npx", ["vitest", "run", test], { cwd, stdio: "inherit", shell: true, env });
	if (r.status !== 0) process.exit(r.status ?? 1);
}
