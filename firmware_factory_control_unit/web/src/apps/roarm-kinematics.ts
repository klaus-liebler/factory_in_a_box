// Geschlossene IK/FK fuer den RoArm-M3 -- 1:1 Portierung der Trigonometrie aus Waveshares
// Referenzfirmware (Core/RoArm-M3_example/RoArm-M3_config.h + RoArm-M3_module.h,
// RoArmM3_computePosbyJointRad()/RoArmM3_baseCoordinateCtrl()/simpleLinkageIkRad()), NICHT neu
// hergeleitet. Muss inhaltlich synchron bleiben mit der spaeteren C++-Portierung
// (Core/Src/setup_and_loops/roarm_kinematics.hh, s. Plan Abschnitt 2) -- beide Seiten kennen
// bislang nur die Linklaengen aus der Referenz, nicht gegen die reale Hardware verifiziert.
//
// Joint-Reihenfolge (6 DOF, wie im Wire-Format jointAnglesCentiDeg[6]):
//   0 Base (Yaw) | 1 Shoulder (Pitch) | 2 Elbow (Pitch) | 3 Wrist (Pitch) | 4 Roll | 5 Gripper

export const JOINT_COUNT = 6;
export const enum JointIndex {
	Base = 0,
	Shoulder = 1,
	Elbow = 2,
	Wrist = 3,
	Roll = 4,
	Gripper = 5,
}

export const JOINT_NAMES = ["Base", "Shoulder", "Elbow", "Wrist", "Roll", "Gripper"] as const;

// --- Linklaengen (mm) -- ARM_L1_LENGTH_MM.. aus RoArm-M3_config.h ---
const L1 = 126.06;
const L2A = 236.82;
const L2B = 30.0;
const L2 = Math.hypot(L2A, L2B);
const T2RAD = Math.atan2(L2B, L2A);
const L3A = 144.49;
const L3B = 0;
const L3 = Math.hypot(L3A, L3B);
const T3RAD = Math.atan2(L3B, L3A);
const L4A = 171.67;
const L4B = 13.69;
const LEA = L4A; // EoAT_A (Werkzeug-Offset) ist in der Referenz 0
const LEB = L4B;
const LE = Math.hypot(LEA, LEB);
const TERAD = Math.atan2(LEB, LEA);

/// Konservative Platzhalter-Gelenkgrenzen aus der Referenz (ARM_*_LIMIT_MIN/MAX_RAD) --
/// ausdruecklich NICHT gegen die reale Hardware verifiziert (siehe Plan-Kontext). Wrist/Roll
/// hatten in der Referenz keine eigenen Grenzwerte -- hier mit demselben Platzhalter (+-90deg)
/// uebernommen, bis reale Werte feststehen.
export const JOINT_LIMITS_RAD: ReadonlyArray<readonly [number, number]> = [
	[-Math.PI / 2, Math.PI / 2], // Base
	[-Math.PI / 2, Math.PI / 2], // Shoulder
	[-Math.PI / 2, Math.PI / 2], // Elbow
	[-Math.PI / 2, Math.PI / 2], // Wrist
	[-Math.PI / 2, Math.PI / 2], // Roll
	[-Math.PI / 2, Math.PI / 2], // Gripper
];

export function clampJointRad(jointIndex: number, angleRad: number): number {
	const [min, max] = JOINT_LIMITS_RAD[jointIndex];
	return Math.min(max, Math.max(min, angleRad));
}

export interface CartesianPose {
	xMm: number;
	yMm: number;
	zMm: number;
	pitchRad: number;
	rollRad: number;
	gripperRad: number;
}

function polarToCartesian(r: number, theta: number): [number, number] {
	return [r * Math.cos(theta), r * Math.sin(theta)];
}

function cartesianToPolar(x: number, y: number): [number, number] {
	return [Math.hypot(x, y), Math.atan2(y, x)];
}

/// Port von RoArmM3_computePosbyJointRad(): Gelenkwinkel (rad) -> kartesische Pose (mm/rad).
export function forwardKinematics(jointsRad: readonly number[]): CartesianPose {
	const [base, shoulder, elbow, wrist, roll, gripper] = jointsRad;

	const [aOut, bOut] = polarToCartesian(L2, Math.PI / 2 - (shoulder + T2RAD));
	const [cOut, dOut] = polarToCartesian(L3, Math.PI / 2 - (elbow + shoulder + T3RAD));
	const [eOut, fOut] = polarToCartesian(LE, Math.PI / 2 - (elbow + shoulder + wrist + TERAD));

	const rEe = aOut + cOut + eOut;
	const zEe = bOut + dOut + fOut;
	const [gOut, hOut] = polarToCartesian(rEe, base);

	return {
		xMm: gOut,
		yMm: hOut,
		zMm: zEe,
		pitchRad: elbow + shoulder + wrist - Math.PI / 2,
		rollRad: roll,
		gripperRad: gripper,
	};
}

/// Port von rotatePoint()/movePoint() (RoArm-M3_module.h) -- Hilfsfunktionen fuer die IK.
function rotatePoint(theta: number): [number, number] {
	const alpha = TERAD + theta;
	return [-LE * Math.cos(alpha), -LE * Math.sin(alpha)];
}

function movePoint(xA: number, yA: number, s: number): [number, number] {
	const distance = Math.hypot(xA, yA);
	if (distance - s <= 1e-6) return [0, 0];
	const ratio = (distance - s) / distance;
	return [xA * ratio, yA * ratio];
}

/// Port von simpleLinkageIkRad(): 2-Link-Ebenen-IK fuer Schulter/Ellbogen. Gibt NaN zurueck
/// (isReachable=false), wenn die Zielposition ausserhalb der Armreichweite liegt (acos-Argument
/// ausserhalb [-1,1]) -- entspricht der Referenz' "nanIK"-Flag.
function simpleLinkageIkRad(aIn: number, bIn: number): { shoulderRad: number; elbowRad: number; gripperOffsetRad: number } {
	let psi: number;
	let alpha: number;
	let omega: number;
	let beta: number;

	if (Math.abs(bIn) < 1e-6) {
		psi = Math.acos((L2 * L2 + aIn * aIn - L3 * L3) / (2 * L2 * aIn)) + T2RAD;
		alpha = Math.PI / 2 - psi;
		omega = Math.acos((aIn * aIn + L3 * L3 - L2 * L2) / (2 * aIn * L3));
		beta = psi + omega - T3RAD;
	} else {
		const l2c = aIn * aIn + bIn * bIn;
		const lc = Math.sqrt(l2c);
		const lambda = Math.atan2(bIn, aIn);
		psi = Math.acos((L2 * L2 + l2c - L3 * L3) / (2 * L2 * lc)) + T2RAD;
		alpha = Math.PI / 2 - lambda - psi;
		omega = Math.acos((L3 * L3 + l2c - L2 * L2) / (2 * lc * L3));
		beta = psi + omega - T3RAD;
	}

	const delta = Math.PI / 2 - alpha - beta;
	return { shoulderRad: alpha, elbowRad: beta, gripperOffsetRad: delta };
}

/// Port von RoArmM3_baseCoordinateCtrl(): kartesische Pose (mm/rad) -> Gelenkwinkel (rad).
/// isReachable=false, falls die Pose ausserhalb der Armreichweite liegt (entspricht "nanIK").
export function inverseKinematics(pose: CartesianPose): { jointsRad: number[]; isReachable: boolean } {
	const [deltaX, deltaY] = rotatePoint(pose.pitchRad - Math.PI);
	const [betaX, betaY] = movePoint(pose.xMm, pose.yMm, deltaX);
	const [baseR, baseRad] = cartesianToPolar(betaX, betaY);
	const { shoulderRad, elbowRad, gripperOffsetRad } = simpleLinkageIkRad(baseR, pose.zMm + deltaY);
	const wristRad = gripperOffsetRad + pose.pitchRad;

	const jointsRad = [baseRad, shoulderRad, elbowRad, wristRad, pose.rollRad, pose.gripperRad];
	const isReachable = jointsRad.every((v) => Number.isFinite(v));
	return { jointsRad, isReachable };
}

export function radToCentiDeg(rad: number): number {
	return Math.round(((rad * 180) / Math.PI) * 100);
}

export function centiDegToRad(centiDeg: number): number {
	return ((centiDeg / 100) * Math.PI) / 180;
}
