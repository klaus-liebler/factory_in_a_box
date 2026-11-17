// The simulation-only web app as a plain static site (dist/): relative asset paths (base "./") and hash
// routing (src/main.ts), so it runs from any directory of any web server without a rewrite rule.
import { defineConfig } from "vite";

export default defineConfig({
	base: "./",
	build: {
		target: "esnext",
	},
	server: {
		host: "localhost",
		port: 5174,
	},
});
