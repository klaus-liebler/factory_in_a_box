// Trapezoidal velocity tracking per joint: each joint runs towards its target with a velocity
// and acceleration limit and brakes in time (the former RoArmMotion).
#pragma once

#include <algorithm>
#include <cmath>

#include "roarm/kinematics.hpp"

namespace fcu::roarm {

class JointTracker {
public:
    JointTracker() = default;
    JointTracker(double initial, double max_velocity, double max_acceleration)
        : current_(initial), target_(initial), max_velocity_(max_velocity), max_acceleration_(max_acceleration) {}

    void set_target(double rad) { target_ = rad; }
    [[nodiscard]] double current() const { return current_; }
    [[nodiscard]] bool moving() const { return std::fabs(target_ - current_) > 1e-4 || std::fabs(velocity_) > 1e-4; }

    double tick(double dt) {
        if (dt <= 0) {
            return current_;
        }
        const double distance = target_ - current_;
        const double direction = distance >= 0 ? 1.0 : -1.0;
        if (std::fabs(distance) <= 1e-4 && std::fabs(velocity_) < max_acceleration_ * dt) {
            current_ = target_;
            velocity_ = 0;
            return current_;
        }
        const double braking = velocity_ * velocity_ / (2 * max_acceleration_);
        const double desired = std::fabs(distance) <= braking ? 0.0 : direction * max_velocity_;
        const double max_dv = max_acceleration_ * dt;
        velocity_ += std::clamp(desired - velocity_, -max_dv, max_dv);
        double next = current_ + velocity_ * dt;
        if ((direction >= 0 && next > target_) || (direction < 0 && next < target_)) {
            next = target_;
            velocity_ = 0;
        }
        current_ = next;
        return current_;
    }

private:
    double current_ = 0, velocity_ = 0, target_ = 0;
    double max_velocity_ = 1.0, max_acceleration_ = 2.0;
};

class ArmTracker {
public:
    ArmTracker(const Joints& initial, double max_velocity, double max_acceleration) {
        for (int i = 0; i < joint_count; ++i) {
            joints_[i] = JointTracker(initial[i], max_velocity, max_acceleration);
        }
    }
    void set_targets(const Joints& t) {
        for (int i = 0; i < joint_count; ++i) joints_[i].set_target(t[i]);
    }
    [[nodiscard]] bool moving() const { return std::ranges::any_of(joints_, [](const JointTracker& j) { return j.moving(); }); }
    void tick(double dt) {
        for (auto& j : joints_) j.tick(dt);
    }
    [[nodiscard]] Joints current() const {
        Joints out{};
        for (int i = 0; i < joint_count; ++i) out[i] = joints_[i].current();
        return out;
    }

private:
    std::array<JointTracker, joint_count> joints_;
};

}  // namespace fcu::roarm
