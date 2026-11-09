// The stepper test bench's logic without the DOM (stepper-lab-app.ts): what the TMC2209 status
// registers say, and the speed profile of a move as the firmware runs it (rtoos ProfileStepper:
// the same formulas). TMC2209 datasheet rev. 1.09.

export const TMC_CLOCK_HZ = 12_000_000;  // internal oscillator
export const TSTEP_STANDSTILL = 0xfffff;

/** DRV_STATUS (5.5.3). */
export interface DrvStatus {
	csActual: number;  // 0..31: the current the driver uses now, (cs+1)/32 of the full scale
	standstill: boolean;  // stst
	stealth: boolean;  // StealthChop is working
	overtemp: boolean;  // ot: shut down
	overtempWarning: boolean;  // otpw
	shortToGround: boolean;  // s2ga | s2gb
	shortLowSide: boolean;  // s2vsa | s2vsb
	openLoad: boolean;  // ola | olb (also shown in fast motion and at standstill: a hint only)
	temperature: string;  // the highest reached threshold: "<120 °C", ">120 °C" ...
}

export function decodeDrvStatus(v: number): DrvStatus {
	const bit = (n: number) => ((v >>> n) & 1) === 1;
	const t = bit(11) ? "> 157 °C" : bit(10) ? "> 150 °C" : bit(9) ? "> 143 °C" : bit(8) ? "> 120 °C" : "< 120 °C";
	return {
		csActual: (v >>> 16) & 0x1f,
		standstill: bit(31),
		stealth: bit(30),
		overtemp: bit(1),
		overtempWarning: bit(0),
		shortToGround: bit(2) || bit(3),
		shortLowSide: bit(4) || bit(5),
		openLoad: bit(6) || bit(7),
		temperature: t,
	};
}

/** PWM_SCALE (5.6): the StealthChop amplitude (pwm_scale_sum 0..255) and its automatic part (signed 9 bit). */
export function decodePwmScale(v: number): { sum: number; auto: number } {
	const a = (v >>> 16) & 0x1ff;
	return { sum: v & 0xff, auto: a >= 0x100 ? a - 0x200 : a };
}

/** Full steps per second from TSTEP (the time per 1/256 micro step in clocks); 0 at standstill. */
export function fullStepsPerSecond(tstep: number): number {
	if (tstep === 0 || tstep >= TSTEP_STANDSTILL) return 0;
	return TMC_CLOCK_HZ / (tstep * 256);
}

/** TSTEP of a velocity in full steps/s (for TPWMTHRS, TCOOLTHRS: thresholds are TSTEP values). */
export function tstepOf(fullStepsPerSecond: number): number {
	return fullStepsPerSecond <= 0 ? 0 : Math.min(TSTEP_STANDSTILL, Math.round(TMC_CLOCK_HZ / (fullStepsPerSecond * 256)));
}

/** IRUN/IHOLD/CS_ACTUAL as a share of the full scale current. */
export const currentShare = (cs: number) => (cs + 1) / 32;
/** TPOWERDOWN and IHOLDDELAY count 2^18 clocks: 21.8 ms each. */
export const delayUnitMs = (2 ** 18 / TMC_CLOCK_HZ) * 1000;

export type Shape = "linear" | "sigmoid";

export interface MoveProfile {
	startHz: number;
	cruiseHz: number;
	accel: number;  // steps/s^2 of the linear ramp; the sigmoid covers the same distance
	shape: Shape;
	steepness?: number;
}

/** Steps of a full ramp from the start to the cruise rate (ProfileStepper::ramp_steps). */
export function rampSteps(p: MoveProfile): number {
	return Math.ceil((p.cruiseHz ** 2 - p.startHz ** 2) / (2 * p.accel));
}

/** The step rate at ramp step s (ProfileStepper::rate). */
export function rampRate(p: MoveProfile, s: number): number {
	const S = rampSteps(p);
	if (s >= S) return p.cruiseHz;
	if (p.shape === "linear") return Math.sqrt(p.startHz ** 2 + 2 * p.accel * s);
	const k = p.steepness ?? 10;
	const sig = (x: number) => 1 / (1 + Math.exp(-x));
	const lo = sig(-k / 2), span = sig(k / 2) - lo;
	return p.startHz + (p.cruiseHz - p.startHz) * ((sig((k * s) / S - k / 2) - lo) / span);
}

/** The rate of every step of a move of n steps: up the ramp, cruise, down the same ramp (a
 *  triangle if n is short). */
export function moveRate(p: MoveProfile, n: number, k: number): number {
	const ramp = Math.min(rampSteps(p), Math.floor(n / 2));
	if (k < ramp) return rampRate(p, k);
	if (k >= n - ramp) return rampRate(p, n - 1 - k);
	return ramp < rampSteps(p) ? rampRate(p, ramp) : p.cruiseHz;
}

/** The speed over time of a move of n steps: about `points` samples (t in s, v in steps/s), and
 *  its duration. */
export function moveCurve(p: MoveProfile, n: number, points = 300): { t: number; v: number }[] {
	const out: { t: number; v: number }[] = [];
	const every = Math.max(1, Math.floor(n / points));
	let t = 0;
	for (let k = 0; k < n; k++) {
		const v = moveRate(p, n, k);
		if (k % every === 0 || k === n - 1) out.push({ t, v });
		t += 1 / v;
	}
	out.push({ t, v: 0 });
	return out;
}

/** What the firmware refuses (rtoos ProfileStepper::configure with a 1 MHz step timer): null if fine. */
export function profileProblem(p: MoveProfile, tickHz = 1_000_000, maxTicks = 0xffff): string | null {
	if (!(p.startHz > 0) || p.cruiseHz < p.startHz) return "Startfrequenz > 0 und ≤ Zielfrequenz";
	if (!(p.accel > 0)) return "Beschleunigung > 0";
	if (tickHz / p.cruiseHz < 10) return "Zielfrequenz zu hoch für den Schritt-Timer";
	if (Math.round(tickHz / p.startHz) > maxTicks) return `Startfrequenz zu niedrig (mindestens ${Math.ceil(tickHz / maxTicks)} Mikroschritte/s)`;
	return null;
}
