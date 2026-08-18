import { defineConfig } from "vite";
import { singleFileFirmwareAssetPlugin } from "./build-tools/vite-plugin-single-file-firmware-asset.ts";

// singleFileFirmwareAssetPlugin() (s. dort) inlined JS+CSS in eine einzige dist/index.html,
// entfernt zusaetzliche Leerzeichen und schreibt das Ergebnis direkt Brotli-komprimiert nach
// ../build/assets/index.html.br (per objcopy/Linker-Section ins Firmware-Flash einkompiliert,
// siehe CMakeLists.txt) -- ein einziger "vite build"-Aufruf genuegt, kein separater Embed-Schritt
// mehr (s. docs/build-process.md Abschnitt 9).
//
// Im Dev-Server (npm run dev) werden /api/*-Anfragen an die echte Control-Unit weitergeleitet,
// statt sie ebenfalls von Vite bedienen zu lassen (Vite hat keine Modbus-Bruecke) -- so laesst
// sich die UI per Hot-Reload gegen echte Hardware entwickeln, ohne bei jeder Aenderung neu zu
// flashen. Hostname ueberschreibbar per Umgebungsvariable CONTROL_UNIT_HOST (z.B. anderes
// Board oder IP-Adresse statt mDNS-Name), Default ist das aktuell bekannte Testboard.
// secure:false noetig, da die Firmware ein von der privaten CA signiertes Zertifikat nutzt
// (s. tools/provision_board_individual_data_and_files.mjs), das Node nicht automatisch vertraut.
const CONTROL_UNIT_HOST = process.env.CONTROL_UNIT_HOST ?? "factory-box-303537.local";

export default defineConfig({
	plugins: [singleFileFirmwareAssetPlugin()],
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
