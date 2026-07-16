#!/usr/bin/env node
// Provisioniert ein Geraete-Zertifikat fuer das aktuell am ST-Link angeschlossene Board:
//
//   1) Liest die 96-Bit STM32-Unique-ID per STM32CubeProgrammer aus.
//   2) Prueft C:\Users\mail\OneDrive - HSOS\stm32_boards\<id>\ auf ein bereits vorhandenes
//      Zertifikat fuer dieses Board -- wenn vorhanden, wird es wiederverwendet.
//   3) Sonst: erzeugt per openssl ein neues, von der privaten CA
//      (C:\Users\mail\OneDrive - HSOS\certificates\rootCA.pem.{crt,key}) signiertes
//      Zertifikat mit CN=Hostname (factory-box-<letzte 6 Hex-Ziffern der Chip-ID>) und
//      speichert es dauerhaft in stm32_boards/<id>/ (Wiederverwendung bei kuenftigen
//      Neu-Provisionierungen desselben Boards).
//   4) Konvertiert Zertifikat+Key nach DER und schreibt
//      Core/Src/generated/device_certificate.c/.h (wie modbus_ui_page.c: generiert,
//      aber eingecheckt -- kein Build-Schritt haengt automatisch daran).
//
// Aufruf manuell (einmal pro Board, vor dem ersten Firmware-Build fuer dieses Board):
//   node tools/provision-certificate.mjs
import { execFileSync } from "node:child_process";
import { existsSync, mkdirSync, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import os from "node:os";

const rootDir = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const outDir = path.join(rootDir, "Core", "Src", "generated");

const STM32_PROGRAMMER_CLI =
	"C:\\Program Files\\STMicroelectronics\\STM32Cube\\STM32CubeProgrammer\\bin\\STM32_Programmer_CLI.exe";
const UID_ADDRESS = "0x08FFF800"; // UID_BASE, siehe stm32h573xx.h
const BOARDS_DIR = "C:\\Users\\mail\\OneDrive - HSOS\\stm32_boards";
const CERTS_DIR = "C:\\Users\\mail\\OneDrive - HSOS\\certificates";
const CA_CERT = path.join(CERTS_DIR, "rootCA.pem.crt");
const CA_KEY = path.join(CERTS_DIR, "rootCA.pem.key");
const SUBJECT_PREFIX = "/C=DE/ST=NRW/L=Greven/O=Klaus Lieber personal";
const CERT_DAYS = "3000";

function readUniqueId() {
	const output = execFileSync(STM32_PROGRAMMER_CLI, ["-c", "port=SWD", "-r32", UID_ADDRESS, "12"], {
		encoding: "utf8"
	});
	const match = output.match(new RegExp(`${UID_ADDRESS}\\s*:\\s*([0-9A-Fa-f]{8})\\s+([0-9A-Fa-f]{8})\\s+([0-9A-Fa-f]{8})`));
	if (!match) {
		throw new Error(`Konnte Unique-ID nicht aus STM32_Programmer_CLI-Ausgabe lesen:\n${output}`);
	}
	return (match[1] + match[2] + match[3]).toUpperCase();
}

function opensslCreateCertificate(hostname, boardDir) {
	const keyPath = path.join(boardDir, `${hostname}.pem.key`);
	const csrPath = path.join(boardDir, `${hostname}.csr`);
	const crtPath = path.join(boardDir, `${hostname}.pem.crt`);
	const extPath = path.join(os.tmpdir(), `${hostname}.ext.cnf`);

	// ECDSA (P-256/prime256v1) statt RSA-2048: deutlich billigere Signaturberechnung
	// (Stack + CPU-Zeit) beim TLS-Handshake auf dem STM32H573 -- RSA-2048-Modpow beim
	// ServerKeyExchange-Signing brauchte 16 KB Thread-Stack, siehe SERVER_STACK in app.cpp.
	execFileSync("openssl", ["ecparam", "-name", "prime256v1", "-genkey", "-noout", "-out", keyPath]);
	execFileSync("openssl", [
		"req", "-new", "-key", keyPath, "-out", csrPath,
		"-subj", `${SUBJECT_PREFIX}/CN=${hostname}`
	]);

	// Eigene, pro-Board generierte ext-Datei statt der geteilten certificates/openssl.cnf zu
	// mutieren -- die traegt das subjectAltName sonst fest fuer nur EIN Board.
	// Zwei SAN-Eintraege: der Browser prueft den Hostnamen so, wie er eingegeben wurde --
	// "<hostname>.local" (mDNS-Aufloesung ueber unseren nx_mdns-Responder) ist ein anderer
	// String als "<hostname>" und muss deshalb separat als DNS-SAN gelistet sein, sonst
	// schlaegt die Zertifikatspruefung fuer die .local-Variante fehl.
	writeFileSync(
		extPath,
		"authorityKeyIdentifier=keyid,issuer\n" +
			"basicConstraints=CA:FALSE\n" +
			"keyUsage = digitalSignature, nonRepudiation, keyEncipherment, dataEncipherment\n" +
			"extendedKeyUsage = serverAuth\n" +
			"subjectAltName = @alt_names\n\n" +
			"[alt_names]\n" +
			`DNS.1 = ${hostname}\n` +
			`DNS.2 = ${hostname}.local\n`
	);

	execFileSync("openssl", [
		"x509", "-req", "-in", csrPath,
		"-CA", CA_CERT, "-CAkey", CA_KEY, "-CAcreateserial",
		"-out", crtPath, "-days", CERT_DAYS, "-sha256", "-extfile", extPath
	]);

	return { keyPath, crtPath };
}

function pemToDer(pemPath, kind) {
	// "openssl ec -outform DER" schreibt das flache SEC1/RFC-5915-ECPrivateKey-SEQUENCE
	// (version, privateKey OCTET STRING, [0] namedCurve, [1] publicKey) -- exakt das Format,
	// das nx_secure_x509_certificate_initialize() mit NX_SECURE_X509_KEY_TYPE_EC_DER erwartet
	// (siehe nx_secure_x509_ec_private_key_parse.c). Anders als beim frueheren RSA-Key
	// (openssl rsa vs. pkey) gibt es hier keine PKCS#8-Falle: "openssl ec" liefert immer SEC1.
	const args =
		kind === "cert"
			? ["x509", "-in", pemPath, "-outform", "DER"]
			: ["ec", "-in", pemPath, "-outform", "DER"];
	return execFileSync("openssl", args, { maxBuffer: 1024 * 1024 });
}

function writeCArraySource(hostname, certDer, keyDer) {
	const toCBytes = (buf) => {
		const lines = [];
		for (let i = 0; i < buf.length; i += 20) {
			const chunk = buf.subarray(i, i + 20);
			lines.push("    " + Array.from(chunk).map((b) => `0x${b.toString(16).padStart(2, "0")}`).join(", ") + ",");
		}
		return lines.join("\n");
	};

	const header = `#pragma once
// Generiert von tools/provision-certificate.mjs -- nicht von Hand editieren. Neu erzeugen
// per "node tools/provision-certificate.mjs" (liest/erzeugt das Zertifikat fuer das aktuell
// am ST-Link angeschlossene Board).
#include <stddef.h>

extern const unsigned char DEVICE_CERT_DER[];
extern const size_t DEVICE_CERT_DER_LEN;
extern const unsigned char DEVICE_KEY_DER[];
extern const size_t DEVICE_KEY_DER_LEN;
extern const char DEVICE_HOSTNAME[];
`;

	const source = `// Generiert von tools/provision-certificate.mjs -- nicht von Hand editieren.
#include "device_certificate.h"

const unsigned char DEVICE_CERT_DER[] = {
${toCBytes(certDer)}
};
const size_t DEVICE_CERT_DER_LEN = sizeof(DEVICE_CERT_DER);

const unsigned char DEVICE_KEY_DER[] = {
${toCBytes(keyDer)}
};
const size_t DEVICE_KEY_DER_LEN = sizeof(DEVICE_KEY_DER);

const char DEVICE_HOSTNAME[] = "${hostname}";
`;

	mkdirSync(outDir, { recursive: true });
	writeFileSync(path.join(outDir, "device_certificate.h"), header);
	writeFileSync(path.join(outDir, "device_certificate.c"), source);
}

function main() {
	console.log("Lese Chip-ID vom angeschlossenen Board...");
	const uid = readUniqueId();
	const shortId = uid.slice(-6).toLowerCase();
	const hostname = `factory-box-${shortId}`;
	console.log(`Chip-ID: ${uid} -> Hostname: ${hostname}`);

	const boardDir = path.join(BOARDS_DIR, `${shortId}_${uid.toLowerCase()}`);
	const existingKey = path.join(boardDir, `${hostname}.pem.key`);
	const existingCrt = path.join(boardDir, `${hostname}.pem.crt`);

	let keyPath, crtPath;
	if (existsSync(existingKey) && existsSync(existingCrt)) {
		console.log(`Vorhandenes Zertifikat gefunden: ${boardDir}`);
		keyPath = existingKey;
		crtPath = existingCrt;
	} else {
		console.log(`Kein Zertifikat fuer dieses Board gefunden, erzeuge neues in: ${boardDir}`);
		mkdirSync(boardDir, { recursive: true });
		({ keyPath, crtPath } = opensslCreateCertificate(hostname, boardDir));
	}

	const certDer = pemToDer(crtPath, "cert");
	const keyDer = pemToDer(keyPath, "key");
	writeCArraySource(hostname, certDer, keyDer);

	console.log(`Geschrieben: ${path.join(outDir, "device_certificate.c")} (Cert ${certDer.length} B, Key ${keyDer.length} B)`);
}

main();
