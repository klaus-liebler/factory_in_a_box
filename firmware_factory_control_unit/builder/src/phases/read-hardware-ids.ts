// Phasen "ReadHardwareIDsAndGenerateFilesAndCertsLazy"/"...Forced" (s. docs/build-process.md
// Abschnitt 6) -- Nachfolger von tools/provision_board_individual_data_and_files.mjs. Die
// PKI-/UID-Logik ist inhaltlich unveraendert, geaendert hat sich nur das Ziel: es wird
// ausschliesslich ins Board-Archiv geschrieben (stm32_boards/<boardId>/generated/), nicht mehr
// direkt ins Repo -- das uebernimmt die separate Phase CopyGeneratedFilesToBuildDirectory.
//
//   1) Liest die 96-Bit STM32-Unique-ID per STM32CubeProgrammer aus.
//   2) Lazy: prueft stm32_boards/<boardId>/ auf ein bereits vorhandenes Zertifikat -- wenn
//      vorhanden, wird es wiederverwendet. Forced: erzeugt immer neu, auch wenn schon eines da ist.
//   3) device_ids.hh-Inhalt und die DER-Konvertierung sind deterministisch/billig und werden IMMER
//      neu geschrieben, unabhaengig von Lazy/Forced.
//   4) Cacht den erkannten boardId (s. board-context.ts) fuer nachfolgende Phasen.
import { execFileSync } from "node:child_process";
import { existsSync, mkdirSync, writeFileSync } from "node:fs";
import path from "node:path";
import os from "node:os";
import { boardArchiveDir, boardGeneratedDir } from "../board-archive.ts";
import { writeCachedBoardId } from "../board-context.ts";
import { caCert, caKey, certDays, stm32ProgrammerCli, subjectPrefix, uidAddress } from "../environment-config.ts";

interface UidResult {
	uid: string;
	words: [number, number, number];
}

function readUniqueId(): UidResult {
	const cliPath = stm32ProgrammerCli();
	let output: string;
	try {
		output = execFileSync(cliPath, ["-c", "port=SWD", "-r32", uidAddress(), "12"], {
			encoding: "utf8",
		});
	} catch (err: any) {
		// execFileSync wirft bei ENOENT/nicht-0-Exit ein Node-internes Error-Objekt mit
		// zirkulaerer Referenz (result.error) -- console.error/uncaught darauf dumpt eine
		// fuerchterliche, fuer Menschen kaum lesbare Stack-/Objekt-Ausgabe. Hier stattdessen
		// gezielt die zwei haeufigsten Ursachen erkennen und eine kurze, umsetzbare Meldung
		// ausgeben.
		if (err.code === "ENOENT") {
			throw new Error(
				`STM32_Programmer_CLI.exe nicht gefunden unter:\n  ${cliPath}\n` +
					`Pruefe die STM32CubeProgrammer-Installation bzw. die Umgebungsvariable STM32_PRG_PATH ` +
					`(darf auf die .exe selbst oder deren bin/-Ordner zeigen).`
			);
		}
		const details = (err.stderr || err.stdout || err.message || "").toString().trim();
		throw new Error(
			`STM32_Programmer_CLI konnte das Board nicht auslesen (Exit-Code ${err.status ?? "?"}).\n` +
				`Moegliche Ursachen: ST-LINK nicht angeschlossen/nicht mit Strom versorgt, oder bereits von ` +
				`einer anderen Anwendung belegt (z.B. eine laufende Debug-Sitzung in STM32CubeIDE -- die ` +
				`beenden und erneut versuchen).\n` +
				(details ? `Ausgabe des Tools:\n${details}` : "")
		);
	}

	const match = output.match(new RegExp(`${uidAddress()}\\s*:\\s*([0-9A-Fa-f]{8})\\s+([0-9A-Fa-f]{8})\\s+([0-9A-Fa-f]{8})`));
	if (!match) {
		throw new Error(`Konnte Unique-ID nicht aus STM32_Programmer_CLI-Ausgabe lesen:\n${output}`);
	}
	// words[0..2] entsprechen exakt HAL_GetUIDw0()/_w1()/_w2() (beide lesen denselben UID_BASE
	// aufsteigend) -- wichtig fuer computeUsbNcmMac()/computeEthMac() unten, deren Bit-Layout
	// exakt zu App::SetupBeforeThreadX()' Chip-UID-Mismatch-Check (app.cc) passen muss.
	const words = [match[1], match[2], match[3]].map((w) => parseInt(w, 16)) as [number, number, number];
	return { uid: (match[1] + match[2] + match[3]).toUpperCase(), words };
}

// Locally-administered MAC (Byte 0 = 0x02, s. IEEE 802-2014 Tabelle 8-1) fuer die virtuelle
// USB-CDC-NCM-NIC.
function computeUsbNcmMac(words: [number, number, number]): number[] {
	const [w0, w1, w2] = words;
	return [0x02, (w0 >>> 24) & 0xff, w0 & 0xff, (w1 >>> 24) & 0xff, w1 & 0xff, w2 & 0xff];
}

// Bitgenauer Nachbau des USER-CODE-Blocks "MACADDRESS" in main.c's MX_ETH_Init() -- MUSS mit
// dieser Stelle synchron bleiben, falls sich dort je etwas aendert. Andere Adresse als
// computeUsbNcmMac() oben (XOR-Faltung aller drei UID-Woerter statt nur Hi/Lo-Bytes von w0/w1/w2),
// damit die physische Ethernet-NIC (LAN8742) und die virtuelle USB-CDC-NCM-NIC nicht dieselbe
// MAC tragen.
function computeEthMac(words: [number, number, number]): number[] {
	const [w0, w1, w2] = words;
	const folded = (w0 ^ w1 ^ w2) >>> 0;
	return [0x02, (folded >>> 24) & 0xff, (folded >>> 16) & 0xff, (folded >>> 8) & 0xff, folded & 0xff, ((w0 >>> 8) ^ (w1 >>> 16) ^ (w2 >>> 24)) & 0xff];
}

function toHex2(byte: number): string {
	return byte.toString(16).toUpperCase().padStart(2, "0");
}

function opensslCreateCertificate(hostname: string, boardDir: string): { keyPath: string; crtPath: string } {
	const keyPath = path.join(boardDir, `${hostname}.pem.key`);
	const csrPath = path.join(boardDir, `${hostname}.csr`);
	const crtPath = path.join(boardDir, `${hostname}.pem.crt`);
	const extPath = path.join(os.tmpdir(), `${hostname}.ext.cnf`);

	// ECDSA (P-256/prime256v1) statt RSA-2048: deutlich billigere Signaturberechnung
	// (Stack + CPU-Zeit) beim TLS-Handshake auf dem STM32H573.
	execFileSync("openssl", ["ecparam", "-name", "prime256v1", "-genkey", "-noout", "-out", keyPath]);
	execFileSync("openssl", ["req", "-new", "-key", keyPath, "-out", csrPath, "-subj", `${subjectPrefix()}/CN=${hostname}`]);

	// Eigene, pro-Board generierte ext-Datei statt der geteilten certificates/openssl.cnf zu
	// mutieren -- die traegt das subjectAltName sonst fest fuer nur EIN Board. Zwei
	// DNS-SAN-Eintraege ("<hostname>" und "<hostname>.local") + ein IP-SAN fuer die
	// USB-CDC-NCM-NIC (192.168.173.1, keine eigene mDNS-Instanz, s. net_setup.cpp).
	writeFileSync(
		extPath,
		"authorityKeyIdentifier=keyid,issuer\n" +
			"basicConstraints=CA:FALSE\n" +
			"keyUsage = digitalSignature, nonRepudiation, keyEncipherment, dataEncipherment\n" +
			"extendedKeyUsage = serverAuth\n" +
			"subjectAltName = @alt_names\n\n" +
			"[alt_names]\n" +
			`DNS.1 = ${hostname}\n` +
			`DNS.2 = ${hostname}.local\n` +
			`IP.1 = 192.168.173.1\n`
	);

	execFileSync("openssl", [
		"x509",
		"-req",
		"-in",
		csrPath,
		"-CA",
		caCert(),
		"-CAkey",
		caKey(),
		"-CAcreateserial",
		"-out",
		crtPath,
		"-days",
		certDays(),
		"-sha256",
		"-extfile",
		extPath,
	]);

	return { keyPath, crtPath };
}

function pemToDer(pemPath: string, kind: "cert" | "key"): Buffer {
	// "openssl ec -outform DER" schreibt das flache SEC1/RFC-5915-ECPrivateKey-SEQUENCE -- exakt
	// das Format, das nx_secure_x509_certificate_initialize() mit NX_SECURE_X509_KEY_TYPE_EC_DER
	// erwartet.
	const args = kind === "cert" ? ["x509", "-in", pemPath, "-outform", "DER"] : ["ec", "-in", pemPath, "-outform", "DER"];
	return execFileSync("openssl", args, { maxBuffer: 1024 * 1024 });
}

export interface CertificateInfo {
	issuer: string;
	// Unix-Epoch-Sekunden statt vorformatierter Strings -- Formatierung/Zeitzone passiert
	// ausschliesslich im Browser (s. web/src/apps/system-info-app.ts).
	issuedAtEpoch: number;
	validUntilEpoch: number;
}

function toEpochSeconds(opensslDate: string): number {
	// Node parst OpenSSL's Default-Datumsformat ("Jul 24 08:44:47 2026 GMT") direkt.
	return Math.floor(new Date(opensslDate).getTime() / 1000);
}

// Liest Aussteller + Gueltigkeitszeitraum direkt aus dem (wiederverwendeten oder frisch
// erzeugten) Zertifikat -- keine separate Buchfuehrung noetig, das Zertifikat selbst ist die
// Quelle der Wahrheit.
function readCertificateInfo(crtPath: string): CertificateInfo {
	const output = execFileSync("openssl", ["x509", "-in", crtPath, "-noout", "-issuer", "-startdate", "-enddate"], {
		encoding: "utf8",
	});
	const issuerMatch = output.match(/^issuer=(.+)$/m);
	const startMatch = output.match(/^notBefore=(.+)$/m);
	const endMatch = output.match(/^notAfter=(.+)$/m);
	if (!issuerMatch || !startMatch || !endMatch) {
		throw new Error(`Konnte Zertifikatsinfo nicht aus openssl-Ausgabe lesen:\n${output}`);
	}
	return {
		issuer: issuerMatch[1]!,
		issuedAtEpoch: toEpochSeconds(startMatch[1]!),
		validUntilEpoch: toEpochSeconds(endMatch[1]!),
	};
}

function renderDeviceIdsHh(hostname: string, words: [number, number, number], netMac: number[], ethMac: number[]): string {
	const [w0, w1, w2] = words;
	const serialString = [w0, w1, w2].map((w) => w.toString(16).toUpperCase().padStart(8, "0")).join("");
	const netMacString = netMac.map(toHex2).join("");
	const netMacInit = netMac.map((b) => `0x${toHex2(b)}`).join(", ");
	const ethMacInit = ethMac.map((b) => `0x${toHex2(b)}`).join(", ");

	return `#pragma once
// Generiert von builder/src/phases/read-hardware-ids.ts -- nicht von Hand editieren. Neu erzeugen
// per "npm run phase:hardware-ids:lazy" bzw. "...:forced" (liest/erzeugt Zertifikat +
// Board-Identitaet fuer das aktuell am ST-Link angeschlossene Board). Alles hier ist fix pro
// physischem Board (aus dessen 96-Bit Chip-UID abgeleitet) -- deshalb als Konstanten
// eincompiliert statt bei jedem Boot per snprintf()/Bit-Fummelei neu berechnet.
// App::SetupBeforeThreadX() (app.cc) prueft beim Start, ob DEVICE_CHIP_UID_W0/1/2 noch zur
// tatsaechlichen Chip-UID des angeschlossenen Boards passen, und haelt sonst an
// (Error_Handler()) -- verhindert, dass fuer Board A gebaute Firmware unbemerkt auf Board B
// mit dessen falscher MAC-Adresse/Seriennummer laeuft.
//
// Sowohl von C++-Uebersetzungseinheiten (app.cc/net_setup.cpp/webserver.cpp) ALS AUCH von
// main.c (reines C, CubeMX-generiert, s. dortiger MX_ETH_Init()/USER-CODE-Block MACADDRESS)
// eingebunden -- DEVICE_IDS_CONST expandiert deshalb je nach Sprache zu constexpr (C++) oder
// static const (C, kein constexpr vor C23), exakt einmal definiert statt die Werte doppelt
// pflegen zu muessen.
#ifdef __cplusplus
#include <cstdint>
#define DEVICE_IDS_CONST constexpr
#else
#include <stdint.h>
#define DEVICE_IDS_CONST static const
#endif
DEVICE_IDS_CONST char DEVICE_HOSTNAME[] = "${hostname}";
DEVICE_IDS_CONST uint32_t DEVICE_CHIP_UID_W0 = 0x${w0.toString(16).toUpperCase().padStart(8, "0")};
DEVICE_IDS_CONST uint32_t DEVICE_CHIP_UID_W1 = 0x${w1.toString(16).toUpperCase().padStart(8, "0")};
DEVICE_IDS_CONST uint32_t DEVICE_CHIP_UID_W2 = 0x${w2.toString(16).toUpperCase().padStart(8, "0")};
// 24 Hex-Ziffern (3x uint32_t Chip-UID) -- USB-Seriennummer-Stringdeskriptor.
DEVICE_IDS_CONST char DEVICE_USB_SERIAL_STRING[] = "${serialString}";
// Locally-administered MAC (Byte 0 = 0x02, s. IEEE 802-2014 Tabelle 8-1) fuer die virtuelle
// USB-CDC-NCM-NIC.
DEVICE_IDS_CONST uint8_t DEVICE_USB_NCM_MAC[6] = {${netMacInit}};
// Dieselben 6 Byte als 12 Grossbuchstaben-Hexziffern ohne Trennzeichen -- CDC1.2 Ethernet
// Networking Functional Descriptor's iMACAddress-String-Format (s. usbd_device.c).
DEVICE_IDS_CONST char DEVICE_USB_NCM_MAC_STRING[] = "${netMacString}";
// Locally-administered MAC fuer die physische Ethernet-NIC (LAN8742, s. main.c MX_ETH_Init()) --
// ANDERE Adresse als DEVICE_USB_NCM_MAC oben.
DEVICE_IDS_CONST uint8_t DEVICE_ETH_MAC[6] = {${ethMacInit}};
#undef DEVICE_IDS_CONST
`;
}

export interface HardwareIdsResult {
	boardId: string;
	hostname: string;
	chipUid: string;
}

// Inhalt von device-ids.json (s. u.) -- wird von ReadGitStatusAndGenerateFiles fuer build-info.ts
// wiederverwendet.
export interface DeviceIdentity {
	hostname: string;
	chipUid: string;
	usbNcmMac: string;
	ethMac: string;
	certificate: CertificateInfo;
}

export function readHardwareIdsAndGenerateFilesAndCerts(forced: boolean): HardwareIdsResult {
	console.log("Lese Chip-ID vom angeschlossenen Board...");
	const { uid, words } = readUniqueId();
	const shortId = uid.slice(-6).toLowerCase();
	const hostname = `factory-box-${shortId}`;
	const netMac = computeUsbNcmMac(words);
	const ethMac = computeEthMac(words);
	const boardId = `${shortId}_${uid.toLowerCase()}`;
	console.log(
		`Chip-ID: ${uid} -> Hostname: ${hostname}, USB-NCM-MAC: ${netMac.map(toHex2).join(":")}, ` +
			`Ethernet-MAC: ${ethMac.map(toHex2).join(":")}`
	);

	const boardDir = boardArchiveDir(boardId);
	const generatedDir = boardGeneratedDir(boardId);
	mkdirSync(boardDir, { recursive: true });
	mkdirSync(generatedDir, { recursive: true });

	const existingKey = path.join(boardDir, `${hostname}.pem.key`);
	const existingCrt = path.join(boardDir, `${hostname}.pem.crt`);

	let keyPath: string;
	let crtPath: string;
	if (!forced && existsSync(existingKey) && existsSync(existingCrt)) {
		console.log(`Vorhandenes Zertifikat gefunden (Lazy): ${boardDir}`);
		keyPath = existingKey;
		crtPath = existingCrt;
	} else {
		console.log(`${forced ? "Forced" : "Kein Zertifikat fuer dieses Board gefunden"} -- erzeuge neues Zertifikat in: ${boardDir}`);
		({ keyPath, crtPath } = opensslCreateCertificate(hostname, boardDir));
	}

	const certDer = pemToDer(crtPath, "cert");
	const keyDer = pemToDer(keyPath, "key");
	const certificate = readCertificateInfo(crtPath);
	writeFileSync(path.join(generatedDir, "device_certificate.der"), certDer);
	writeFileSync(path.join(generatedDir, "device_key.der"), keyDer);
	writeFileSync(path.join(generatedDir, "device_ids.hh"), renderDeviceIdsHh(hostname, words, netMac, ethMac));

	// Maschinenlesbares Gegenstueck zu device_ids.hh (analog zu gitstatus.json fuer
	// gitconstants.hh) -- damit ReadGitStatusAndGenerateFiles Hostname/Chip-UID/MAC-Adressen/
	// Zertifikatsdaten fuer build-info.ts nicht per Regex aus dem C++-Header/DER-Binary
	// zurueckparsen muss.
	const deviceIds: DeviceIdentity = {
		hostname,
		chipUid: uid,
		usbNcmMac: netMac.map(toHex2).join(":"),
		ethMac: ethMac.map(toHex2).join(":"),
		certificate,
	};
	writeFileSync(path.join(generatedDir, "device-ids.json"), JSON.stringify(deviceIds, null, 2) + "\n");

	writeCachedBoardId(boardId);

	console.log(`Geschrieben: ${generatedDir} (device_ids.hh, device-ids.json, device_certificate.der [${certDer.length} B], device_key.der [${keyDer.length} B])`);
	return { boardId, hostname, chipUid: uid };
}
