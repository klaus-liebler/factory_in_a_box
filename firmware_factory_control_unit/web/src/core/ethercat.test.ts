// The interpretation of the EtherCAT slave's raw ESC registers (ethercat.ts).
import { describe, expect, it } from "vitest";
import { alCodeText, alStateText, dlStatus, eepromText, fmmuKind, sdoAbortText, syncManagerKind, systemTimeText } from "./ethercat.ts";

describe("EtherCAT status", () => {
	it("AL status: the state, an indicated error, unknown states", () => {
		expect(alStateText(0x08)).toBe("OP");
		expect(alStateText(0x14)).toBe("SAFE-OP + Fehler");
		expect(alStateText(0x05)).toBe("? (0x5)");
		expect(alCodeText(0x001d)).toBe("0x001d ungültige Ausgangskonfiguration");
		expect(alCodeText(0x9999)).toBe("0x9999 unbekannt");
		expect(sdoAbortText(0x08000021)).toBe("0x08000021 lokale Steuerung (SPS ist Steuerquelle)");
	});

	it("DL status: PDI, link, loop and communication per port", () => {
		const d = dlStatus(0x0215);  // PDI ok, link port 0, communication port 0 (bit 2: reserved here)
		expect(d.pdiOperational).toBe(true);
		expect(d.ports[0]).toEqual({ link: true, loopClosed: false, communication: true });
		expect(d.ports[1]).toEqual({ link: false, loopClosed: false, communication: false });
		expect(dlStatus(0x0900).ports[1]).toEqual({ link: false, loopClosed: false, communication: true });
	});

	it("SyncManager roles from the control byte", () => {
		expect(syncManagerKind(0x26, 128, true)).toBe("Mailbox, Master → Slave");
		expect(syncManagerKind(0x22, 128, true)).toBe("Mailbox, Slave → Master");
		expect(syncManagerKind(0x64, 8, true)).toBe("3-Puffer, Master → Slave");
		expect(syncManagerKind(0x20, 8, false)).toBe("inaktiv");
		expect(syncManagerKind(0x20, 0, true)).toBe("inaktiv");
	});

	it("EEPROM, FMMU and the DC system time", () => {
		expect(eepromText(0x0020)).toBe("emuliert, geladen");
		expect(eepromText(0x1800)).toBe("I²C-EEPROM, nicht geladen, Prüfsummenfehler");
		expect(fmmuKind(2)).toBe("schreiben (Ausgänge)");
		expect(systemTimeText(0)).toBe("steht (keine Distributed Clocks)");
		expect(systemTimeText(86_400 * 1e9)).toBe("2000-01-02 00:00:00 UTC");
	});
});
