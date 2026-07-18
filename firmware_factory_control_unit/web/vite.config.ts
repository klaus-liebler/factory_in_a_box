import { defineConfig } from "vite";
import { viteSingleFile } from "vite-plugin-singlefile";
import { inlineSingleFileMinifyPlugin } from "./scripts/postbuild-singlefile-minify";

// Alles (JS+CSS) wird in eine einzige dist/index.html inlined (viteSingleFile), anschliessend
// entfernt inlineSingleFileMinifyPlugin() zusaetzliche Leerzeichen aus dem HTML/CSS/JS (enforce:
// "post", laeuft also nach dem Inlining auf dem fertigen Single-File-Ergebnis). Diese Datei wird
// danach per scripts/embed-into-firmware.mjs Brotli-komprimiert und als rohes Binary nach
// ../assets/index.html.br geschrieben (per objcopy/Linker-Section ins Firmware-Flash
// einkompiliert, siehe CMakeLists.txt).
export default defineConfig({
	plugins: [viteSingleFile(), inlineSingleFileMinifyPlugin()],
	build: {
		target: "esnext"
	}
});
