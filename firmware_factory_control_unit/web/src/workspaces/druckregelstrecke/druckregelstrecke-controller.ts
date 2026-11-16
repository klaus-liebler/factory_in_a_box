// Client-side P/PI/PID controller for the Reglerbetrieb mode of the Druckregelstrecke (see
// docs/druckregelstrecke-modes.md) - deliberately runs in the browser instead of the firmware (which stays a
// pure Modbus register interface), clocked with a fixed cycle time (500ms, see druckregelstrecke-app.ts).
// Pure control arithmetic without any DOM/register/WS knowledge, so it can be tested and understood
// independently of the app.
//
// Controller equation (ideal/non-interacting form, see Kahlert/Bate "PID-Einstellregeln" p.4):
// u(t) = operating point + KP * (e(t) + 1/TN * integral(e dt) + TV * de/dt)
// Discretized (rectangle integration, difference quotient for the D part) with the fixed cycle time dt.
//
// Anti-windup: "limit the integrator to 100%" (the only variant supported for now, see
// docs/druckregelstrecke-modes.md) - the I part is kept as a state of its own EXACTLY in manipulated-variable
// units (per mille) and clamped to the full output range [0, 1000] right after every addition. This is the
// simplest form of integrator clamping: while the controller saturates, the I part does not keep growing
// without bound (classic wind-up), but it is usable again immediately as soon as the control error changes
// sign - no back-calculation needed.
export type ControllerType = "P" | "PI" | "PID";

export interface ControllerSettings {
	type: ControllerType;
	/* Base power added to the controller output (0..1000 per mille). */
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

	/* Resets the internal controller state (I part, D history) - to be called when the controller is switched
	   on, so that old state accumulated while the controller was off does not immediately cause a jump in the
	   output. */
	reset(): void {
		this.integralTermPermille = 0;
		this.previousError = null;
	}

	/* One control cycle. setpointRaw/processValueRaw in the same units (PRESSURE_RAW raw values), dtSec = time
	   actually elapsed since the last call. Returns the new compressor power in per mille (0..1000, already
	   clamped). */
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
	/* Time constant T (63% time AFTER the dead time) in seconds - see the comment in druckregelstrecke-app.ts on
	   the measuring convention. For the Chien-Hrones-Reswick formulas (which actually expect the balance time Tg
	   found with the inflection tangent) T is deliberately used as an approximation of Tg - for a pure PT1 +
	   dead time plant model (assumed for this setup) both are identical, see the derivation in the apply handler
	   of druckregelstrecke-app.ts. */
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

// T-sum rule after Kuhn (see control-technology.de/ct/tsumreg.html) - TSum = Tu + T for a PT1 + dead time
// model (sum of all delays including dead time, see the comment above).
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

// Chien/Hrones/Reswick (see Kahlert/Bate "PID-Einstellregeln", p.9) - actually expects Tu/Tg from the
// inflection tangent method; Tg is approximated here by the measured time constant T (see the
// StepResponseParameters comment).
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
