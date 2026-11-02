// Unit tests of the web app (npm test). Own config: vite.config.ts builds the firmware asset.
import { defineConfig } from "vitest/config";

export default defineConfig({
	test: {
		include: ["src/**/*.test.ts"],
	},
});
