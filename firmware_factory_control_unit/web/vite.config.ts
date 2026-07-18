import { defineConfig } from "vite";
import { viteSingleFile } from "vite-plugin-singlefile";
import { inlineSingleFileMinifyPlugin } from "./scripts/postbuild-singlefile-minify";

// Alles (JS+CSS) wird in eine einzige dist/index.html inlined (viteSingleFile), anschliessend
// entfernt inlineSingleFileMinifyPlugin() zusaetzliche Leerzeichen aus dem HTML/CSS/JS (enforce:
// "post", laeuft also nach dem Inlining auf dem fertigen Single-File-Ergebnis). Diese Datei wird
// danach per scripts/embed-into-firmware.mjs Brotli-komprimiert und als rohes Binary nach
// ../assets/index.html.br geschrieben (per objcopy/Linker-Section ins Firmware-Flash
// einkompiliert, siehe CMakeLists.txt).
//
// Im Dev-Server (npm run dev) werden /api/*-Anfragen an die echte Control-Unit weitergeleitet,
// statt sie ebenfalls von Vite bedienen zu lassen (Vite hat keine Modbus-Bruecke) -- so laesst
// sich die UI per Hot-Reload gegen echte Hardware entwickeln, ohne bei jeder Aenderung neu zu
// flashen. Hostname ueberschreibbar per Umgebungsvariable CONTROL_UNIT_HOST (z.B. anderes
// Board oder IP-Adresse statt mDNS-Name), Default ist das aktuell bekannte Testboard.
// secure:false noetig, da die Firmware ein von der privaten CA signiertes Zertifikat nutzt
// (s. tools/provision-certificate.mjs), das Node nicht automatisch vertraut.
const CONTROL_UNIT_HOST = process.env.CONTROL_UNIT_HOST ?? "factory-box-303537.local";

export default defineConfig({
	plugins: [viteSingleFile(), inlineSingleFileMinifyPlugin()],
	build: {
		target: "esnext"
	},
	server: {
		proxy: {
			"/api": {
				target: `https://${CONTROL_UNIT_HOST}`,
				changeOrigin: true,
				secure: false
			}
		}
	}
});
