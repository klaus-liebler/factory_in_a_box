// Kinematics of the Waveshare RoArm-M3 (5 axes + gripper): forward and inverse kinematics of
// its planar linkage, after the reference firmware (Core/RoArm-M3_example, RoArm-M3_module.h).
// Lengths in mm, angles in rad.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

namespace fcu::roarm {

inline constexpr int joint_count = 6;
enum Joint : int { Base = 0, Shoulder = 1, Elbow = 2, Wrist = 3, Roll = 4, Gripper = 5 };
using Joints = std::array<double, joint_count>;

namespace kin {
using std::numbers::pi;
inline constexpr double l2a = 236.82, l2b = 30.0, l3a = 144.49, l3b = 0.0, lea = 171.67, leb = 13.69;
inline const double l2 = std::hypot(l2a, l2b), t2 = std::atan2(l2b, l2a);
inline const double l3 = std::hypot(l3a, l3b), t3 = std::atan2(l3b, l3a);
inline const double le = std::hypot(lea, leb), te = std::atan2(leb, lea);

/// Joint limits: servo ranges (Feetech counts) within the URDF limits.
inline constexpr std::array<std::array<double, 2>, joint_count> limits{{
    {-pi, pi},          // base
    {-pi / 2, pi / 2},  // shoulder
    {0.0, 2.95},        // elbow (counts 1024..3071 = 0..pi; URDF -1..2.95)
    {-pi / 2, pi / 2},  // wrist
    {-pi, pi},          // roll
    {-pi / 2, pi / 2},  // gripper
}};
}  // namespace kin

inline double clamp_joint(int joint, double rad) { return std::clamp(rad, kin::limits[joint][0], kin::limits[joint][1]); }

inline std::int16_t rad_to_centideg(double rad) {
    return static_cast<std::int16_t>(std::lround(rad * 180.0 / std::numbers::pi * 100.0));
}
inline double centideg_to_rad(std::int16_t centideg) { return centideg / 100.0 * std::numbers::pi / 180.0; }

struct Pose {
    double x_mm = 0, y_mm = 0, z_mm = 0;
    double pitch = 0, roll = 0, gripper = 0;
};

inline Pose forward(const Joints& j) {
    using namespace kin;
    const double r = l2 * std::cos(pi / 2 - (j[Shoulder] + t2)) + l3 * std::cos(pi / 2 - (j[Elbow] + j[Shoulder] + t3)) +
                     le * std::cos(pi / 2 - (j[Elbow] + j[Shoulder] + j[Wrist] + te));
    const double z = l2 * std::sin(pi / 2 - (j[Shoulder] + t2)) + l3 * std::sin(pi / 2 - (j[Elbow] + j[Shoulder] + t3)) +
                     le * std::sin(pi / 2 - (j[Elbow] + j[Shoulder] + j[Wrist] + te));
    return {r * std::cos(j[Base]), r * std::sin(j[Base]), z, j[Elbow] + j[Shoulder] + j[Wrist] - pi / 2, j[Roll], j[Gripper]};
}

/// Joint angles for `p`; nullopt-like: `reachable` false when the linkage cannot get there.
struct InverseResult {
    Joints joints{};
    bool reachable = false;
};

inline InverseResult inverse(const Pose& p) {
    using namespace kin;
    // the wrist point: the end effector moved back along the pitch
    const double alpha_e = te + p.pitch - pi;
    const double dx = -le * std::cos(alpha_e), dy = -le * std::sin(alpha_e);
    const double distance = std::hypot(p.x_mm, p.y_mm);
    const double ratio = distance - dx <= 1e-6 ? 0.0 : (distance - dx) / distance;
    const double bx = p.x_mm * ratio, by = p.y_mm * ratio;
    const double base_r = std::hypot(bx, by), base = std::atan2(by, bx);
    const double a = base_r, b = p.z_mm + dy;
    double psi = 0, alpha = 0, omega = 0, beta = 0;
    if (std::fabs(b) < 1e-6) {
        psi = std::acos((l2 * l2 + a * a - l3 * l3) / (2 * l2 * a)) + t2;
        alpha = pi / 2 - psi;
        omega = std::acos((a * a + l3 * l3 - l2 * l2) / (2 * a * l3));
        beta = psi + omega - t3;
    } else {
        const double c2 = a * a + b * b, c = std::sqrt(c2), lambda = std::atan2(b, a);
        psi = std::acos((l2 * l2 + c2 - l3 * l3) / (2 * l2 * c)) + t2;
        alpha = pi / 2 - lambda - psi;
        omega = std::acos((l3 * l3 + c2 - l2 * l2) / (2 * c * l3));
        beta = psi + omega - t3;
    }
    const double wrist = pi / 2 - alpha - beta + p.pitch;
    InverseResult r{{base, alpha, beta, wrist, p.roll, p.gripper}, true};
    r.reachable = std::ranges::all_of(r.joints, [](double v) { return std::isfinite(v); });
    return r;
}

}  // namespace fcu::roarm
