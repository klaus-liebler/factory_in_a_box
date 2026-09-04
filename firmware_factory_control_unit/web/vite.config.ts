import { defineConfig } from "vite";
import fs from "node:fs";
import path from "node:path";
import { visualizer } from "rollup-plugin-visualizer";
import { singleFileFirmwareAssetPlugin } from "@klaus-liebler/vite-firmware-single-file";

// singleFileFirmwareAssetPlugin() (geteiltes Plugin, s. npm-packages/@klaus-liebler/vite-firmware-single-file
// -- frueher als build-tools/vite-plugin-single-file-firmware-asset.ts dupliziert, jetzt EIN
// gemeinsamer Code fuer factory_in_a_box/sensact/labathome) inlined JS+CSS in eine einzige
// dist/index.html, entfernt zusaetzliche Leerzeichen und schreibt das Ergebnis direkt
// Brotli-komprimiert nach ../build/assets/index.html.br (per objcopy/Linker-Section ins
// Firmware-Flash einkompiliert, siehe CMakeLists.txt) -- ein einziger "vite build"-Aufruf genuegt,
// kein separater Embed-Schritt mehr (s. docs/build-process.md Abschnitt 9). Expliziter zweiter
// Parameter (statt options.dir/Vites eigenem build.outDir): build/assets/ ist ein
// board-uebergreifend GETEILTER Ordner (enthaelt auch von einer vorherigen Build-Phase dort
// abgelegte device_certificate.der/device_key.der/root_ca.der), der NICHT von Vite geleert werden
// darf -- s. Kommentar im geteilten Plugin.
const ASSETS_DIR = path.join(import.meta.dirname, "..", "build", "assets");
//
// Im Dev-Server (npm run dev) werden /api/*-Anfragen an die echte Control-Unit weitergeleitet,
// statt sie ebenfalls von Vite bedienen zu lassen (Vite hat keine Modbus-Bruecke) -- so laesst
// sich die UI per Hot-Reload gegen echte Hardware entwickeln, ohne bei jeder Aenderung neu zu
// flashen. Hostname ueberschreibbar per Umgebungsvariable CONTROL_UNIT_HOST (z.B. anderes
// Board oder IP-Adresse statt mDNS-Name), Default ist das aktuell bekannte Testboard.
// secure:false noetig, da die Firmware ein von der privaten CA signiertes Zertifikat nutzt
// (s. tools/provision_board_individual_data_and_files.mjs), das Node nicht automatisch vertraut.
const CONTROL_UNIT_HOST = process.env.CONTROL_UNIT_HOST ?? "factory-box-303537.local";
const DEV_HTTPS_CERT_DIR = process.env.DEV_HTTPS_CERT_DIR ?? path.join(process.env.USERPROFILE ?? "", "OneDrive - HSOS", "certificates");
const DEV_HTTPS_CERT_KEY = process.env.DEV_HTTPS_CERT_KEY ?? path.join(DEV_HTTPS_CERT_DIR, "testserver.pem.key");
const DEV_HTTPS_CERT_CRT = process.env.DEV_HTTPS_CERT_CRT ?? path.join(DEV_HTTPS_CERT_DIR, "testserver.pem.crt");

function createHttpsConfig() {
	if (!fs.existsSync(DEV_HTTPS_CERT_KEY) || !fs.existsSync(DEV_HTTPS_CERT_CRT)) {
		return undefined;
	}

	return {
		key: fs.readFileSync(DEV_HTTPS_CERT_KEY),
		cert: fs.readFileSync(DEV_HTTPS_CERT_CRT)
	};
}

export default defineConfig(({ mode }) => {
	const isAnalyze = mode === "analyze";
	const httpsConfig = createHttpsConfig();

	return {
		plugins: [
			isAnalyze && visualizer({
				filename: "./dist/stats.html",
				template: "treemap",
				gzipSize: true,
				brotliSize: true,
				open: true
			}),
			!isAnalyze && singleFileFirmwareAssetPlugin("index.html.br", ASSETS_DIR),
		].filter(Boolean),
		build: {
			target: "esnext"
		},
		server: {
			host: "localhost",
			port: 5173,
			https: httpsConfig,
			proxy: {
				"/api": {
					target: `https://${CONTROL_UNIT_HOST}`,
					changeOrigin: true,
					secure: false
				}
			}
		}
	};
});
