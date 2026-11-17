// Unit tests of the web app (npm test). Own config: vite.config.ts builds the firmware asset.
import { defineConfig } from "vitest/config";

export default defineConfig({
	test: {
		include: ["src/**/*.test.ts"],
		// the PLC (@fiab/plc) works with the FCU register map
		setupFiles: ["src/plc-fcu/register-map.ts"],
	},
});
