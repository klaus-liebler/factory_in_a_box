#pragma once
// Geschlossene IK/FK fuer den RoArm-M3 -- 1:1 Portierung der Trigonometrie aus Waveshares
// Referenzfirmware (Core/RoArm-M3_example/RoArm-M3_config.h + RoArm-M3_module.h,
// RoArmM3_computePosbyJointRad()/RoArmM3_baseCoordinateCtrl()/simpleLinkageIkRad()), NICHT neu
// hergeleitet -- Linklaengen/Geometrie nicht gegen die reale Hardware verifiziert (s.
// Plan-Kontext). Muss inhaltlich synchron bleiben mit der TypeScript-Portierung
// web/src/apps/roarm-kinematics.ts (dort zusaetzlich per Playwright empirisch gegen die
// 3D-Ansicht verifiziert -- Achsen-/Vorzeichenkonvention hier identisch uebernommen).
//
// Joint-Reihenfolge (6 DOF, wie im Wire-Format jointAnglesCentiDeg[6] aus
// best_binary_buffers_schema/roarm.cs):
//   0 Base (Yaw) | 1 Shoulder (Pitch) | 2 Elbow (Pitch) | 3 Wrist (Pitch) | 4 Roll | 5 Gripper
#include <array>
#include <cmath>

namespace RoArmKinematics {

constexpr int kJointCount = 6;
enum JointIndex : int { Base = 0, Shoulder = 1, Elbow = 2, Wrist = 3, Roll = 4, Gripper = 5 };

// --- Linklaengen (mm) -- ARM_L1_LENGTH_MM.. aus RoArm-M3_config.h ---
inline constexpr double kL1 = 126.06;
inline constexpr double kL2A = 236.82;
inline constexpr double kL2B = 30.0;
inline const double kL2 = std::hypot(kL2A, kL2B);
inline const double kT2Rad = std::atan2(kL2B, kL2A);
inline constexpr double kL3A = 144.49;
inline constexpr double kL3B = 0.0;
inline const double kL3 = std::hypot(kL3A, kL3B);
inline const double kT3Rad = std::atan2(kL3B, kL3A);
inline constexpr double kL4A = 171.67;
inline constexpr double kL4B = 13.69;
inline constexpr double kLEA = kL4A; // EoAT_A (Werkzeug-Offset) ist in der Referenz 0
inline constexpr double kLEB = kL4B;
inline const double kLE = std::hypot(kLEA, kLEB);
inline const double kTERad = std::atan2(kLEB, kLEA);

// Konservative Platzhalter-Gelenkgrenzen aus der Referenz (ARM_*_LIMIT_MIN/MAX_RAD) --
// ausdruecklich NICHT gegen die reale Hardware verifiziert. Wrist/Roll hatten in der Referenz
// keine eigenen Grenzwerte -- hier mit demselben Platzhalter (+-90deg) uebernommen, bis reale
// Werte feststehen.
inline constexpr std::array<std::array<double, 2>, kJointCount> kJointLimitsRad = {{
    {-M_PI / 2, M_PI / 2}, // Base
    {-M_PI / 2, M_PI / 2}, // Shoulder
    {-M_PI / 2, M_PI / 2}, // Elbow
    {-M_PI / 2, M_PI / 2}, // Wrist
    {-M_PI / 2, M_PI / 2}, // Roll
    {-M_PI / 2, M_PI / 2}, // Gripper
}};

inline double ClampJointRad(int jointIndex, double angleRad) {
    const auto& lim = kJointLimitsRad[jointIndex];
    return std::min(lim[1], std::max(lim[0], angleRad));
}

struct CartesianPose {
    double xMm = 0;
    double yMm = 0;
    double zMm = 0;
    double pitchRad = 0;
    double rollRad = 0;
    double gripperRad = 0;
};

inline void PolarToCartesian(double r, double theta, double& x, double& y) {
    x = r * std::cos(theta);
    y = r * std::sin(theta);
}

inline void CartesianToPolar(double x, double y, double& r, double& theta) {
    r = std::hypot(x, y);
    theta = std::atan2(y, x);
}

// Port von RoArmM3_computePosbyJointRad(): Gelenkwinkel (rad) -> kartesische Pose (mm/rad).
inline CartesianPose ForwardKinematics(const std::array<double, kJointCount>& jointsRad) {
    const double base = jointsRad[Base];
    const double shoulder = jointsRad[Shoulder];
    const double elbow = jointsRad[Elbow];
    const double wrist = jointsRad[Wrist];
    const double roll = jointsRad[Roll];
    const double gripper = jointsRad[Gripper];

    double aOut, bOut, cOut, dOut, eOut, fOut, gOut, hOut;
    PolarToCartesian(kL2, M_PI / 2 - (shoulder + kT2Rad), aOut, bOut);
    PolarToCartesian(kL3, M_PI / 2 - (elbow + shoulder + kT3Rad), cOut, dOut);
    PolarToCartesian(kLE, M_PI / 2 - (elbow + shoulder + wrist + kTERad), eOut, fOut);

    const double rEe = aOut + cOut + eOut;
    const double zEe = bOut + dOut + fOut;
    PolarToCartesian(rEe, base, gOut, hOut);

    CartesianPose pose;
    pose.xMm = gOut;
    pose.yMm = hOut;
    pose.zMm = zEe;
    pose.pitchRad = elbow + shoulder + wrist - M_PI / 2;
    pose.rollRad = roll;
    pose.gripperRad = gripper;
    return pose;
}

// Port von rotatePoint()/movePoint() (RoArm-M3_module.h) -- Hilfsfunktionen fuer die IK.
inline void RotatePoint(double theta, double& x, double& y) {
    const double alpha = kTERad + theta;
    x = -kLE * std::cos(alpha);
    y = -kLE * std::sin(alpha);
}

inline void MovePoint(double xA, double yA, double s, double& xB, double& yB) {
    const double distance = std::hypot(xA, yA);
    if (distance - s <= 1e-6) {
        xB = 0;
        yB = 0;
        return;
    }
    const double ratio = (distance - s) / distance;
    xB = xA * ratio;
    yB = yA * ratio;
}

struct PlanarIkResult {
    double shoulderRad;
    double elbowRad;
    double gripperOffsetRad;
};

// Port von simpleLinkageIkRad(): 2-Link-Ebenen-IK fuer Schulter/Ellbogen. Liefert NaN-Werte
// (isReachable=false in InverseKinematics), wenn die Zielposition ausserhalb der Armreichweite
// liegt (acos-Argument ausserhalb [-1,1]) -- entspricht der Referenz' "nanIK"-Flag.
inline PlanarIkResult SimpleLinkageIkRad(double aIn, double bIn) {
    double psi, alpha, omega, beta;
    if (std::fabs(bIn) < 1e-6) {
        psi = std::acos((kL2 * kL2 + aIn * aIn - kL3 * kL3) / (2 * kL2 * aIn)) + kT2Rad;
        alpha = M_PI / 2 - psi;
        omega = std::acos((aIn * aIn + kL3 * kL3 - kL2 * kL2) / (2 * aIn * kL3));
        beta = psi + omega - kT3Rad;
    } else {
        const double l2c = aIn * aIn + bIn * bIn;
        const double lc = std::sqrt(l2c);
        const double lambda = std::atan2(bIn, aIn);
        psi = std::acos((kL2 * kL2 + l2c - kL3 * kL3) / (2 * kL2 * lc)) + kT2Rad;
        alpha = M_PI / 2 - lambda - psi;
        omega = std::acos((kL3 * kL3 + l2c - kL2 * kL2) / (2 * lc * kL3));
        beta = psi + omega - kT3Rad;
    }
    const double delta = M_PI / 2 - alpha - beta;
    return {alpha, beta, delta};
}

struct InverseKinematicsResult {
    std::array<double, kJointCount> jointsRad{};
    bool isReachable = false;
};

// Port von RoArmM3_baseCoordinateCtrl(): kartesische Pose (mm/rad) -> Gelenkwinkel (rad).
inline InverseKinematicsResult InverseKinematics(const CartesianPose& pose) {
    double deltaX, deltaY;
    RotatePoint(pose.pitchRad - M_PI, deltaX, deltaY);
    double betaX, betaY;
    MovePoint(pose.xMm, pose.yMm, deltaX, betaX, betaY);
    double baseR, baseRad;
    CartesianToPolar(betaX, betaY, baseR, baseRad);
    const PlanarIkResult planar = SimpleLinkageIkRad(baseR, pose.zMm + deltaY);
    const double wristRad = planar.gripperOffsetRad + pose.pitchRad;

    InverseKinematicsResult result;
    result.jointsRad = {baseRad, planar.shoulderRad, planar.elbowRad, wristRad, pose.rollRad, pose.gripperRad};
    result.isReachable = true;
    for (double v : result.jointsRad) {
        if (!std::isfinite(v)) {
            result.isReachable = false;
            break;
        }
    }
    return result;
}

inline int16_t RadToCentiDeg(double rad) { return static_cast<int16_t>(std::lround((rad * 180.0 / M_PI) * 100.0)); }
inline double CentiDegToRad(int16_t centiDeg) { return (static_cast<double>(centiDeg) / 100.0) * M_PI / 180.0; }

} // namespace RoArmKinematics
