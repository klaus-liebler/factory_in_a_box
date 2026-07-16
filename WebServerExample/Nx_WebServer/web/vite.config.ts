import { defineConfig } from "vite";
import { viteSingleFile } from "vite-plugin-singlefile";

// Alles (JS+CSS) wird in eine einzige dist/index.html inlined, per Vite/esbuild minifiziert.
// Diese Datei wird anschliessend per scripts/embed-into-firmware.mjs gzip-komprimiert und als
// C-Array ins Firmware-Flash einkompiliert (siehe Core/Src/generated/modbus_ui_page.c).
export default defineConfig({
	plugins: [viteSingleFile()],
	build: {
		target: "esnext"
	}
});
