// Maschinen-/personenspezifische Einstellungen fuer tools/provision-certificate.mjs (Pfade zu
// STM32CubeProgrammer und zum privaten CA-Material auf Klaus' Rechner). Bewusst als Funktionen
// statt Konstanten: jede kann so z.B. eine Umgebungsvariable auswerten oder einen Fallback zur
// Aufrufzeit berechnen, statt den Wert schon beim Modul-Import fest einzufrieren.
import path from "node:path";

// STM32CubeProgrammer-CLI -- ueberschreibbar per Umgebungsvariable STM32_PRG_PATH, falls
// STM32CubeProgrammer nicht am Standardpfad installiert ist (z.B. anderer Rechner/CI).
export function stm32ProgrammerCli() {
	return (
		process.env.STM32_PRG_PATH ??
		"C:\\Program Files\\STMicroelectronics\\STM32Cube\\STM32CubeProgrammer\\bin\\STM32_Programmer_CLI.exe"
	);
}

// UID_BASE, siehe stm32h573xx.h -- chip-fest, keine Umgebungsvariable noetig.
export function uidAddress() {
	return "0x08FFF800";
}

// Wiederverwendete pro-Board-Zertifikate (s. provision-certificate.mjs).
export function boardsDir() {
	return "C:\\Users\\mail\\OneDrive - HSOS\\stm32_boards";
}

// Private CA (rootCA.pem.{crt,key}), die alle Board-Zertifikate signiert.
export function certsDir() {
	return "C:\\Users\\mail\\OneDrive - HSOS\\certificates";
}

export function caCert() {
	return path.join(certsDir(), "rootCA.pem.crt");
}

export function caKey() {
	return path.join(certsDir(), "rootCA.pem.key");
}

export function subjectPrefix() {
	return "/C=DE/ST=NRW/L=Greven/O=Klaus Lieber personal";
}

export function certDays() {
	return "3000";
}
