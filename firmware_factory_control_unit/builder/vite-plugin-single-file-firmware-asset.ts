// Ersetzt die bisherige Kombination aus vite-plugin-singlefile + inlineSingleFileMinifyPlugin
// (web/scripts/postbuild-singlefile-minify.ts) + manuellem "npm run embed"
// (web/scripts/embed-into-firmware.mjs) durch EIN Plugin (s. docs/build-process.md Abschnitt 9):
//
//   1) Inlined JS+CSS in die index.html -- der Kern davon ist an
//      https://github.com/richardtallent/vite-plugin-singlefile (MIT-lizenziert, Autor Richard
//      Tallent) angelehnt/nachgebaut: replaceScript()/replaceCss() unten entsprechen dessen
//      gleichnamigen Funktionen (Stand main-Branch, s. src/index.ts dort), auf das hier tatsaechlich
//      benoetigte Minimum (ein JS- und ein CSS-Bundle, kein inlinePattern/removeViteModuleLoader)
//      zugeschnitten, um die Abhaengigkeit "vite-plugin-singlefile" nicht laenger einbinden zu
//      muessen.
//   2) minifyHtmlDocument() (builder/singlefile-minify.ts, unveraendert aus dem vormaligen
//      inlineSingleFileMinifyPlugin uebernommen) entfernt zusaetzliche Leerzeichen aus dem
//      inlinierten HTML/CSS/JS.
//   3) Brotli-komprimiert das Ergebnis und schreibt es direkt nach ../build/assets/index.html.br (per
//      objcopy/Linker-Section ins Firmware-Flash einkompiliert, siehe CMakeLists.txt) -- kein
//      separater "npm run embed"-Aufruf mehr noetig, BuildWebApp (s. docs/build-process.md) besteht
//      damit aus einem einzigen "vite build".
import { brotliCompressSync, constants as zlibConstants } from "node:zlib";
import { mkdirSync, writeFileSync } from "node:fs";
import path from "node:path";
import type { Plugin, UserConfig } from "vite";
import type { OutputAsset, OutputChunk, OutputBundle } from "rollup";
import { minifyHtmlDocument } from "./singlefile-minify.ts";

const isJsFile = /\.[mc]?js$/;
const isCssFile = /\.css$/;
const isHtmlFile = /\.html?$/;

// Angelehnt an vite-plugin-singlefile's replaceScript() (s. Datei-Kommentar oben).
function replaceScript(html: string, scriptFilename: string, scriptCode: string): string {
	const escapedFilename = scriptFilename.replaceAll(".", "\\.");
	const scriptTagPattern = new RegExp(`<script([^>]*?) src="(?:[^"]*?/)?${escapedFilename}"([^>]*)></script>`);
	const preloadMarker = /"?__VITE_PRELOAD__"?/g;
	const newCode = scriptCode.replace(preloadMarker, "void 0").replace(/<(\/script>|!--)/g, "\\x3C$1");
	return html.replace(scriptTagPattern, (_match, beforeSrc, afterSrc) => `<script${beforeSrc}${afterSrc}>${newCode.trim()}</script>`);
}

// Angelehnt an vite-plugin-singlefile's replaceCss() (s. Datei-Kommentar oben).
function replaceCss(html: string, styleFilename: string, cssCode: string): string {
	const escapedFilename = styleFilename.replaceAll(".", "\\.");
	const linkTagPattern = new RegExp(`<link([^>]*?) href="(?:[^"]*?/)?${escapedFilename}"([^>]*)>`);
	const newCode = cssCode.replace(`@charset "UTF-8";`, "");
	return html.replace(linkTagPattern, (_match, beforeHref, afterHref) => `<style${beforeHref}${afterHref}>${newCode.trim()}</style>`);
}

function inlineBundleIntoHtml(bundle: OutputBundle): OutputAsset | undefined {
	let htmlAsset: OutputAsset | undefined;
	const jsChunks: OutputChunk[] = [];
	const cssAssets: OutputAsset[] = [];
	const toDelete: string[] = [];

	for (const [fileName, item] of Object.entries(bundle)) {
		if (isHtmlFile.test(fileName)) {
			htmlAsset = item as OutputAsset;
		} else if (isJsFile.test(fileName) && item.type === "chunk") {
			jsChunks.push(item);
		} else if (isCssFile.test(fileName) && item.type === "asset") {
			cssAssets.push(item as OutputAsset);
		}
	}

	if (!htmlAsset) return undefined;

	let html = htmlAsset.source as string;
	for (const chunk of jsChunks) {
		html = replaceScript(html, chunk.fileName, chunk.code);
		toDelete.push(chunk.fileName);
	}
	for (const asset of cssAssets) {
		html = replaceCss(html, asset.fileName, asset.source as string);
		toDelete.push(asset.fileName);
	}

	htmlAsset.source = html;
	for (const fileName of toDelete) {
		delete bundle[fileName];
	}
	return htmlAsset;
}

export function singleFileFirmwareAssetPlugin(): Plugin {
	return {
		name: "single-file-firmware-asset",
		enforce: "post",

		// Entspricht vite-plugin-singlefile's "useRecommendedBuildConfig": alle Assets/Chunks
		// muessen in EINEM JS- und EINEM CSS-Bundle landen, damit generateBundle unten ueberhaupt
		// etwas zum Inlinen hat.
		config(): UserConfig {
			return {
				build: {
					assetsInlineLimit: () => true,
					cssCodeSplit: false,
					assetsDir: "",
					rollupOptions: {
						output: {
							inlineDynamicImports: true,
						},
					},
				},
				base: "./",
			};
		},

		async generateBundle(_options, bundle) {
			const htmlAsset = inlineBundleIntoHtml(bundle);
			if (!htmlAsset) return;

			const minified = await minifyHtmlDocument(htmlAsset.source as string);
			htmlAsset.source = minified;

			const htmlBuffer = Buffer.from(minified, "utf8");
			const compressed = brotliCompressSync(htmlBuffer, {
				params: {
					[zlibConstants.BROTLI_PARAM_QUALITY]: zlibConstants.BROTLI_MAX_QUALITY,
					[zlibConstants.BROTLI_PARAM_SIZE_HINT]: htmlBuffer.length,
				},
			});

			// build/assets/ statt eines eigenen Top-Level-Ordners (s. Paths.cs AssetsDir) -- hier
			// bewusst eigenstaendig relativ zu dieser Datei berechnet statt aus dem C#-Projekt
			// importiert (diese Datei wird direkt von web/vite.config.ts per Vite/Node geladen, nicht
			// vom C#-Compiler).
			const assetsDir = path.join(import.meta.dirname, "..", "build", "assets");
			mkdirSync(assetsDir, { recursive: true });
			const outFile = path.join(assetsDir, "index.html.br");
			writeFileSync(outFile, compressed);
			console.log(`[single-file-firmware-asset] ${htmlBuffer.length} B -> brotli ${compressed.length} B -> ${outFile}`);
		},
	};
}
