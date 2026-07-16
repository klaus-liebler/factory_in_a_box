// Schlanker Zugriff auf die Firmware-Endpunkte -- bewusst kein JSON: /api/registers liefert
// zwei kommagetrennte Listen fester Breite (Holding zuerst, dann Input, getrennt durch "|"),
// /api/write-holding ist ein simples GET mit Query-Parametern statt POST+JSON-Body. Das
// erspart der Firmware jeglichen JSON-Parser (siehe Core/Src/app.cpp), passend zum
// "Prinzip"-Charakter dieser Demo-UI.

export interface RegisterValues {
	holding: number[];
	input: number[];
}

// Der Server-Paket-Pool der Firmware ist bewusst winzig (siehe Core/Src/app.cpp), eine
// Mehr-Paket-Antwort kann dadurch spuerbar dauern. Ein harter Timeout stellt sicher, dass ein
// haengender Request den Poll-Zyklus nicht fuer immer blockiert (app.ts plant den naechsten
// Poll ohnehin erst NACH Abschluss dieses Aufrufs, siehe RegisterUi.scheduleNextPoll).
const FETCH_TIMEOUT_MS = 5000;

async function fetchWithTimeout(url: string): Promise<Response> {
	const controller = new AbortController();
	const timeoutHandle = setTimeout(() => controller.abort(), FETCH_TIMEOUT_MS);
	try {
		return await fetch(url, { cache: "no-store", signal: controller.signal });
	} finally {
		clearTimeout(timeoutHandle);
	}
}

export async function fetchRegisters(): Promise<RegisterValues> {
	const response = await fetchWithTimeout("/api/registers");
	if (!response.ok) {
		throw new Error(`GET /api/registers fehlgeschlagen: HTTP ${response.status}`);
	}
	const text = await response.text();
	const [holdingPart, inputPart] = text.trim().split("|");
	const parseList = (part: string | undefined): number[] =>
		part ? part.split(",").filter((s) => s.length > 0).map((s) => Number(s)) : [];

	return { holding: parseList(holdingPart), input: parseList(inputPart) };
}

export async function writeHolding(address: number, value: number): Promise<void> {
	const response = await fetchWithTimeout(`/api/write-holding?address=${address}&value=${value}`);
	if (!response.ok) {
		throw new Error(`Schreiben fehlgeschlagen: HTTP ${response.status}`);
	}
}
