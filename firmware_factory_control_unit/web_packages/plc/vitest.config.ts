import { defineConfig } from "vitest/config";

export default defineConfig({
	test: {
		include: ["src/**/*.test.ts"],
		setupFiles: ["test/register-map.setup.ts"],
	},
});
