#pragma once
// Beschleunigungsbegrenztes (trapezfoermiges) Geschwindigkeitsprofil pro Gelenk -- entscheidet bei
// jedem Tick aus der verbleibenden Distanz neu, ob beschleunigt/konstant gefahren/gebremst wird
// (kein vorab fixes Zeitprofil), reagiert daher glatt auf ein sich waehrend der Fahrt aenderndes
// Ziel (Live-Jogging) genauso wie auf ein Mission-Wegpunktziel. Konzeptionell dasselbe Prinzip wie
// SigmoidStepper (stm32_libs/sigmoid_stepper) fuer die Schrittmotoren, nur fuer Servo-Winkel statt
// Step/Dir-Pulse -- und bereits im Browser vorab erprobt (web/src/apps/roarm-motion-tracker.ts,
// identischer Algorithmus).
#include <algorithm>
#include <array>
#include <cmath>
#include "roarm_kinematics.hh"

namespace RoArmMotion {

class JointTracker {
public:
    JointTracker() = default;
    JointTracker(double initialRad, double maxVelocityRadPerSec, double maxAccelerationRadPerSec2)
        : current_(initialRad), target_(initialRad), maxVelocity_(maxVelocityRadPerSec), maxAcceleration_(maxAccelerationRadPerSec2) {}

    void SetTarget(double targetRad) { target_ = targetRad; }
    double GetCurrent() const { return current_; }
    double GetTarget() const { return target_; }

    bool IsMoving() const { return std::fabs(target_ - current_) > 1e-4 || std::fabs(velocity_) > 1e-4; }

    // dtSeconds: seit dem letzten Tick vergangene Zeit. Bremsweg wird aus der aktuellen
    // Geschwindigkeit hergeleitet (v^2 / 2a), damit rechtzeitig vor dem Ziel abgebremst wird,
    // statt es zu ueberschwingen.
    double Tick(double dtSeconds) {
        if (dtSeconds <= 0) return current_;

        const double distance = target_ - current_;
        const double direction = (distance >= 0) ? 1.0 : -1.0;
        const double absDistance = std::fabs(distance);
        const double brakingDistance = (velocity_ * velocity_) / (2 * maxAcceleration_);

        double desiredVelocity;
        if (absDistance <= 1e-4 && std::fabs(velocity_) < maxAcceleration_ * dtSeconds) {
            current_ = target_;
            velocity_ = 0;
            return current_;
        } else if (absDistance <= brakingDistance) {
            desiredVelocity = 0;
        } else {
            desiredVelocity = direction * maxVelocity_;
        }

        const double maxDeltaV = maxAcceleration_ * dtSeconds;
        const double deltaV = std::clamp(desiredVelocity - velocity_, -maxDeltaV, maxDeltaV);
        velocity_ += deltaV;

        double next = current_ + velocity_ * dtSeconds;
        if ((direction >= 0 && next > target_) || (direction < 0 && next < target_)) {
            next = target_;
            velocity_ = 0;
        }
        current_ = next;
        return current_;
    }

private:
    double current_ = 0;
    double velocity_ = 0;
    double target_ = 0;
    double maxVelocity_ = 1.0;
    double maxAcceleration_ = 2.0;
};

class MultiJointTracker {
public:
    MultiJointTracker(const std::array<double, RoArmKinematics::kJointCount>& initialRad, double maxVelocityRadPerSec,
                       double maxAccelerationRadPerSec2) {
        for (int i = 0; i < RoArmKinematics::kJointCount; i++) {
            trackers_[i] = JointTracker(initialRad[i], maxVelocityRadPerSec, maxAccelerationRadPerSec2);
        }
    }

    void SetTargets(const std::array<double, RoArmKinematics::kJointCount>& targetsRad) {
        for (int i = 0; i < RoArmKinematics::kJointCount; i++) trackers_[i].SetTarget(targetsRad[i]);
    }
    void SetTarget(int jointIndex, double targetRad) { trackers_[jointIndex].SetTarget(targetRad); }

    bool IsMoving() const {
        for (const auto& t : trackers_) {
            if (t.IsMoving()) return true;
        }
        return false;
    }

    std::array<double, RoArmKinematics::kJointCount> Tick(double dtSeconds) {
        std::array<double, RoArmKinematics::kJointCount> out{};
        for (int i = 0; i < RoArmKinematics::kJointCount; i++) out[i] = trackers_[i].Tick(dtSeconds);
        return out;
    }

    std::array<double, RoArmKinematics::kJointCount> GetCurrentRad() const {
        std::array<double, RoArmKinematics::kJointCount> out{};
        for (int i = 0; i < RoArmKinematics::kJointCount; i++) out[i] = trackers_[i].GetCurrent();
        return out;
    }

private:
    std::array<JointTracker, RoArmKinematics::kJointCount> trackers_;
};

} // namespace RoArmMotion
