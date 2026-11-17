// The simulation-only web app as a plain static site (dist/): relative asset paths (base "./") and hash
// routing (src/main.ts), so it runs from any directory of any web server without a rewrite rule.
import path from "node:path";
import { defineConfig, searchForWorkspaceRoot } from "vite";

export default defineConfig({
	base: "./",
	// @klaus-liebler/ui-shell kommt per file:-Link aus npm-packages: lit nur einmal buendeln und dem
	// Dev-Server die Dateien ausserhalb des Workspaces erlauben
	resolve: { dedupe: ["lit", "lit-html", "@lit/reactive-element"] },
	build: {
		target: "esnext",
	},
	server: {
		host: "localhost",
		port: 5174,
		fs: { allow: [searchForWorkspaceRoot(process.cwd()), path.join(import.meta.dirname, "..", "..", "..", "npm-packages")] },
	},
});
