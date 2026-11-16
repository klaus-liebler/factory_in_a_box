// Clientseitiger P/PI/PID-Regler fuer den Reglerbetrieb-Modus der Druckregelstrecke (s.
// docs/druckregelstrecke-modes.md) -- laeuft bewusst im Browser statt in der Firmware (die bleibt
// ein reines Modbus-Register-Interface), getaktet mit fester Zykluszeit (500ms, s.
// druckregelstrecke-app.ts). Reine Regel-Arithmetik ohne jegliche DOM-/Register-/WS-Kenntnis, um
// sie unabhaengig von der App testen/nachvollziehen zu koennen.
//
// Reglergleichung (ideale/nicht-interagierende Form, s. Kahlert/Bate "PID-Einstellregeln" S.4):
//   u(t) = Arbeitspunkt + KP * (e(t) + 1/TN * Integral(e dt) + TV * de/dt)
// Diskretisiert (Rechteck-Integration, Differenzenquotient fuer den D-Anteil) mit fester
// Zykluszeit dt.
//
// Anti-Windup: "Begrenzung des Integrators auf 100%" (einzige vorerst unterstuetzte Variante,
// s. docs/druckregelstrecke-modes.md) -- der I-Anteil wird als eigener Zustand GENAU in
// Stellgroessen-Einheiten (Promille) gefuehrt und nach jedem Aufaddieren sofort auf den vollen
// Stellgroessenbereich [0, 1000] geklemmt. Das ist die einfachste Form von Integrator-Clamping:
// solange der Regler saettigt, waechst der I-Anteil nicht unbegrenzt weiter (klassisches
// Wind-up), er bleibt aber sofort wieder nutzbar, sobald die Regelabweichung das Vorzeichen
// wechselt -- kein Rueckrechnen/"Back-Calculation" noetig.
export type ControllerType = "P" | "PI" | "PID";

export interface ControllerSettings {
	type: ControllerType;
	/** Grundleistung, die zum Reglerausgang addiert wird (0..1000 Promille). */
	arbeitspunktPermille: number;
	kp: number;
	tnSec: number;
	tvSec: number;
}

export const COMPRESSOR_PWM_MIN = 0;
export const COMPRESSOR_PWM_MAX = 1000;

export class PidController {
	private integralTermPermille = 0;
	private previousError: number | null = null;

	constructor(private settings: ControllerSettings) {}

	updateSettings(settings: ControllerSettings): void {
		this.settings = settings;
	}

	/** Setzt den internen Regler-Zustand zurueck (I-Anteil, D-Vorwissen) -- beim Einschalten des
	 * Reglers aufzurufen, damit ein waehrend "Regler aus" aufgelaufener alter Zustand nicht
	 * sofort einen Sprung im Stellsignal verursacht. */
	reset(): void {
		this.integralTermPermille = 0;
		this.previousError = null;
	}

	/** Ein Regelzyklus. setpointRaw/processValueRaw in denselben Einheiten (PRESSURE_RAW-
	 * Rohwerten), dtSec = tatsaechlich vergangene Zeit seit dem letzten Aufruf. Rueckgabe: neue
	 * Kompressorleistung in Promille (0..1000, bereits geklemmt). */
	step(setpointRaw: number, processValueRaw: number, dtSec: number): number {
		const { type, arbeitspunktPermille, kp, tnSec, tvSec } = this.settings;
		const error = setpointRaw - processValueRaw;

		let output = arbeitspunktPermille + kp * error;

		if (type === "PI" || type === "PID") {
			if (tnSec > 0) {
				this.integralTermPermille += (kp / tnSec) * error * dtSec;
				this.integralTermPermille = clamp(this.integralTermPermille, COMPRESSOR_PWM_MIN, COMPRESSOR_PWM_MAX);
			}
			output += this.integralTermPermille;
		}

		if (type === "PID" && this.previousError !== null && dtSec > 0) {
			output += kp * tvSec * ((error - this.previousError) / dtSec);
		}
		this.previousError = error;

		return clamp(Math.round(output), COMPRESSOR_PWM_MIN, COMPRESSOR_PWM_MAX);
	}
}

function clamp(value: number, min: number, max: number): number {
	return Math.min(max, Math.max(min, value));
}

// --- Regler-Entwurfs-Assistent (Sprungantwort-Modus) --------------------------------------

export interface StepResponseParameters {
	/** Prozessverstaerkung Ks in Rohwert-Counts pro Promille Kompressorleistung. */
	ks: number;
	/** Totzeit Tu in Sekunden. */
	tuSec: number;
	/** Zeitkonstante T (63%-Zeit NACH Ablauf der Totzeit) in Sekunden -- s. Kommentar in
	 * druckregelstrecke-app.ts zur Messkonvention. Fuer die Chien-Hrones-Reswick-Formeln (die
	 * eigentlich die per Wendetangente ermittelte Ausgleichszeit Tg erwarten) wird hier bewusst
	 * T als Naeherung fuer Tg verwendet -- fuer ein reines PT1+Totzeit-Streckenmodell (angenommen
	 * fuer diesen Versuchsaufbau) sind beide Groessen identisch, s. Herleitung im Uebernehmen-
	 * Handler von druckregelstrecke-app.ts. */
	tSec: number;
}

export type DesignMethod = "t-summe" | "chr";
export type TSummeVariant = "normal" | "schnell";
export type ChrOvershoot = "0" | "20";
export type ChrBehavior = "fuehrung" | "stoerung";

export interface DesignMethodChoice {
	method: DesignMethod;
	controllerType: ControllerType;
	tSummeVariant: TSummeVariant;
	chrOvershoot: ChrOvershoot;
	chrBehavior: ChrBehavior;
}

export interface DesignResult {
	kp: number;
	tnSec: number;
	tvSec: number;
}

// T-Summen-Regel nach Kuhn (s. control-technology.de/ct/tsumreg.html) -- TSumme = Tu + T fuer ein
// PT1+Totzeit-Modell (Summe aller Verzoegerungen inkl. Totzeit, s. Kommentar oben).
function designTSumme(params: StepResponseParameters, type: ControllerType, variant: TSummeVariant): DesignResult | null {
	const tSumme = params.tuSec + params.tSec;
	const ks = params.ks;
	if (type === "P") {
		return { kp: 1 / ks, tnSec: 0, tvSec: 0 };
	}
	if (type === "PI") {
		return variant === "normal" ? { kp: 0.5 / ks, tnSec: 0.5 * tSumme, tvSec: 0 } : { kp: 1 / ks, tnSec: 0.7 * tSumme, tvSec: 0 };
	}
	// PID
	return variant === "normal"
		? { kp: 1 / ks, tnSec: 0.66 * tSumme, tvSec: 0.167 * tSumme }
		: { kp: 2 / ks, tnSec: 0.8 * tSumme, tvSec: 0.194 * tSumme };
}

// Chien/Hrones/Reswick (s. Kahlert/Bate "PID-Einstellregeln", S.9) -- erwartet eigentlich Tu/Tg
// aus dem Wendetangentenverfahren; Tg wird hier durch die gemessene Zeitkonstante T angenaehert
// (s. StepResponseParameters-Kommentar).
function designChr(params: StepResponseParameters, type: ControllerType, overshoot: ChrOvershoot, behavior: ChrBehavior): DesignResult {
	const { ks, tuSec: tu, tSec: tg } = params;
	const ratio = tg / tu;
	if (type === "P") {
		const kp = (overshoot === "0" ? 0.3 : 0.7) / ks * ratio;
		return { kp, tnSec: 0, tvSec: 0 };
	}
	if (type === "PI") {
		if (overshoot === "0") {
			return behavior === "stoerung" ? { kp: (0.6 / ks) * ratio, tnSec: 4 * tu, tvSec: 0 } : { kp: (0.35 / ks) * ratio, tnSec: 1.2 * tu, tvSec: 0 };
		}
		return behavior === "stoerung" ? { kp: (0.7 / ks) * ratio, tnSec: 2.3 * tu, tvSec: 0 } : { kp: (0.6 / ks) * ratio, tnSec: tg, tvSec: 0 };
	}
	// PID
	if (overshoot === "0") {
		return behavior === "stoerung"
			? { kp: (0.95 / ks) * ratio, tnSec: 2.4 * tu, tvSec: 0.42 * tu }
			: { kp: (0.6 / ks) * ratio, tnSec: tg, tvSec: 0.5 * tu };
	}
	return behavior === "stoerung"
		? { kp: (1.2 / ks) * ratio, tnSec: 2 * tu, tvSec: 0.42 * tu }
		: { kp: (0.95 / ks) * ratio, tnSec: 1.35 * tg, tvSec: 0.47 * tu };
}

export function designController(params: StepResponseParameters, choice: DesignMethodChoice): DesignResult | null {
	if (!(params.ks !== 0) || params.tuSec <= 0 || params.tSec <= 0) return null;
	return choice.method === "t-summe"
		? designTSumme(params, choice.controllerType, choice.tSummeVariant)
		: designChr(params, choice.controllerType, choice.chrOvershoot, choice.chrBehavior);
}
