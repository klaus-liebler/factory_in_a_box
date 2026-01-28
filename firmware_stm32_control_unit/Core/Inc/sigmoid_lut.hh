#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

// Timer parameters (TIM17 on G4 at 170 MHz -> 1 MHz tick with prescaler 169)

enum class MotionPhase { Idle, Accel, Cruise, Decel };

template <int32_t StartSpeedHz, int32_t EndSpeedHz, size_t AccelerationSegments,
          uint32_t StepsPerAccelerationSegment_>
class SigmoidTimingProvider {

  static constexpr uint16_t kPrescaler = 169;
  static constexpr uint32_t kTimClockHz = 170'000'000U;
  static constexpr uint32_t kTimerTickHz =
      kTimClockHz / (static_cast<uint32_t>(kPrescaler) + 1U);

  static constexpr std::size_t kLutSize = 16;

  struct LutEntry {
    int32_t slope_q16; // ARR delta per step in Q16
  };

  static constexpr double sigmoid(double x) { return 1 / (1 + std::exp(-x)); }

  static constexpr double sigmoid_derivative(double x) {
    double sig = sigmoid(x);
    return sig * (1 - sig);
  }

  static constexpr int32_t speed_to_arr16(int32_t speed_hz) {
    return static_cast<int32_t>(
        ((kTimerTickHz + speed_hz - 1) / speed_hz) -
        1); // ceiling division -->Frequenz wird immer abgerundet
  }
  static constexpr int32_t speed_to_arr32(int32_t speed_hz) {
    return speed_to_arr16(speed_hz) << 16;
  }

private:
  uint32_t remaining_steps_{0};
  uint32_t lut_segment_index_{0};
  uint32_t lut_steps_done_{0};
  int32_t motion_velocity_arr32{2000};
  MotionPhase motion_phase_{MotionPhase::Idle};

  static_assert(StartSpeedHz < EndSpeedHz,
                "Start speed must be slower than end speed");
  static constexpr int32_t StartArr16 = speed_to_arr16(StartSpeedHz);
  static constexpr int32_t EndArr16 = speed_to_arr16(EndSpeedHz);
  static constexpr uint32_t TotalAccelSteps =
      StepsPerAccelerationSegment_ * AccelerationSegments;
  static constexpr int32_t StepsPerAccelerationSegment =
      StepsPerAccelerationSegment_;
  static constexpr double x_step = 0.8; // good fit for 16 segments

  template <bool Debug = false>
  static constexpr std::array<LutEntry, kLutSize>
  make_table_impl(int32_t start_hz, int32_t end_hz) {
    if constexpr (Debug) {
      std::printf("Generating Sigmoid LUT: StartSpeed=%d Hz, EndSpeed=%d Hz, "
                  "steps_per_segment=%u\n",
                  start_hz, end_hz, StepsPerAccelerationSegment);
    }
    std::array<LutEntry, kLutSize> table{};
    double current_speed_hz = start_hz;
    int32_t current_arr32 =
        speed_to_arr32(static_cast<int32_t>(current_speed_hz));
    const double x_0 = -((kLutSize - 1) / 2.0) * x_step;

    for (std::size_t i = 0; i < kLutSize; ++i) {
      const double x = x_0 + x_step * static_cast<double>(i);
      const double target_speed = start_hz + (end_hz - start_hz) * sigmoid(x);
      const int32_t target_arr32 =
          speed_to_arr32(static_cast<int32_t>(target_speed));
      const int32_t slope32 = (target_arr32 - current_arr32) /
                              static_cast<int32_t>(StepsPerAccelerationSegment);

      current_arr32 +=
          slope32 * static_cast<int32_t>(StepsPerAccelerationSegment);
      table[i] = LutEntry{slope32};

      if constexpr (Debug) {
        // Debug-only: numeric trace for each LUT entry
        std::printf("i=%zu target_speed=%.3f target_arr32=%ld slope32=%ld "
                    "resulting_arr=%ld\n",
                    i, target_speed, static_cast<long>(target_arr32),
                    static_cast<long>(slope32),
                    static_cast<long>(current_arr32));
      }
    }
    return table;
  }

  static constexpr std::array<LutEntry, kLutSize> make_table() {
    return make_table_impl<false>(StartSpeedHz, EndSpeedHz);
  }

  static constexpr std::array<LutEntry, kLutSize> table = make_table();

  // Runtime helper for testing with optional logging
  static std::array<LutEntry, kLutSize> make_table_runtime_debug() {
    return make_table_impl<true>(StartSpeedHz, EndSpeedHz);
  }

public:
  virtual ~SigmoidTimingProvider() = default;

  void SetupNewTiming(int32_t steps_to_go) {
    remaining_steps_ = steps_to_go;
    lut_segment_index_ = 0;
    lut_steps_done_ = 0;
  }

  bool getNextArr16(uint16_t &out_arr16) {
    if (lut_segment_index_ >= table.size()) {
      out_arr16=static_cast<uint16_t>(StartArr16); //safe fallback , lowest speed
      return false;
    }
    const std::size_t idx = (motion_phase_ == MotionPhase::Decel)? (table.size() - 1U - lut_segment_index_) : lut_segment_index_;
    const auto &seg = table[idx];
    switch (motion_phase_) {
  case MotionPhase::Accel: {
    motion_velocity_arr32 += seg.slope_q16;
   

    ++lut_steps_done_;
    if (lut_steps_done_ >= StepsPerAccelerationSegment) {
      lut_steps_done_ = 0;
      ++lut_segment_index_;
    }

    if (lut_segment_index_ >= table.size()) {
      lut_segment_index_ = 0;
      motion_phase_ = MotionPhase::Cruise;
    }
    return motion_velocity_arr32 >> 16;
  }
  case MotionPhase::Cruise: {
    motion_velocity_arr32 = EndArr16<<16;
    

    if (remaining_steps_ <=
        static_cast<int32_t>(TotalAccelSteps)) {
      motion_phase_ = MotionPhase::Decel;
      lut_segment_index_ = 0;
      lut_steps_done_ = 0;
    }
    return motion_velocity_arr32 >> 16;
  }
  case MotionPhase::Decel: {
    motion_velocity_arr32 -= seg.slope_q16; // reverse slope for decel
    //printf(" Set value to %u\n", static_cast<uint16_t>(motion_velocity_arr32 >> 16) );

    ++lut_steps_done_;
    if (lut_steps_done_ >= StepsPerAccelerationSegment) {
      lut_steps_done_ = 0;
      ++lut_segment_index_;
      // When all decel segments are consumed, hold the slowest ARR and stop
      // ramping
      if (lut_segment_index_ >= table.size()) {
        motion_velocity_arr32 = StartArr16<<16;
        motion_phase_ = MotionPhase::Idle;
      }
    }
    
    return (motion_velocity_arr32 >> 16);
  }
  default:
    break;
  }
  }
};