// Closed-form IK/FK for the RoArm-M3 - a 1:1 port of the trigonometry of Waveshare's reference firmware
// (RoArm-M3_example/RoArm-M3_config.h + RoArm-M3_module.h,
// RoArmM3_computePosbyJointRad()/RoArmM3_baseCoordinateCtrl()/simpleLinkageIkRad()), NOT derived anew. Must
// stay in sync in content with the C++ port (src/roarm/kinematics.hpp) - both sides so far only know the link
// lengths of the reference, not verified against the real hardware.
//
// Joint order (6 DOF, as in the wire format jointAnglesCentiDeg[6]):
// 0 Base (yaw) | 1 Shoulder (pitch) | 2 Elbow (pitch) | 3 Wrist (pitch) | 4 Roll | 5 Gripper

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
const LEA = L4A; // EoAT_A (tool offset) is 0 in the reference
const LEB = L4B;
const LE = Math.hypot(LEA, LEB);
const TERAD = Math.atan2(LEB, LEA);

// Joint limits the firmware clamps every jog target to - a 1:1 mirror of kin::limits in
// src/roarm/kinematics.hpp (the reasons for the values are there), MUST match it. The gizmo IK
// (roarm3d/ik.ts) intersects its URDF limits with these, so that it never offers a pose the firmware clamps
// away afterwards.
export const JOINT_LIMITS_RAD: ReadonlyArray<readonly [number, number]> = [
	[-Math.PI, Math.PI], // Base
	[-Math.PI / 2, Math.PI / 2], // Shoulder
	[0, 2.95], // Elbow
	[-Math.PI / 2, Math.PI / 2], // Wrist
	[-Math.PI, Math.PI], // Roll
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

// Port of rotatePoint()/movePoint() (RoArm-M3_module.h) - helper functions for the IK.
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

// Port of simpleLinkageIkRad(): 2-link planar IK for shoulder/elbow. Returns NaN (isReachable=false) if the
// target position is outside the arm's reach (acos argument outside [-1,1]) - corresponds to the "nanIK" flag
// of the reference.
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

// Port of RoArmM3_baseCoordinateCtrl(): cartesian pose (mm/rad) -> joint angles (rad). isReachable=false if
// the pose is outside the arm's reach (corresponds to "nanIK").
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
