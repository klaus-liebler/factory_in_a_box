#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>

#include "../Core/Inc/sigmoid_lut.hh"

enum class MotionPhase { Idle, Accel, Cruise, Decel };

using DefaultSigmoidProfile = SigmoidProfile<100, 1000, 16, 3>;

// Current motion state
int32_t motion_velocity_arr32{2000};
MotionPhase motion_phase_{MotionPhase::Idle};
bool motion_direction{true};

// Sigmoid LUT state
uint32_t lut_segment_index_{0};
uint32_t lut_steps_done_{0};

bool use_sigmoid_profile_{false};

// Position tracking
int32_t current_position_{0};
int32_t target_position_{0};

void reset_lut_state_() {

}

void apply_sigmoid_step() {
  const auto &table = DefaultSigmoidProfile::table;
  if (lut_segment_index_ >= table.size()) {
    return;
  }

  const std::size_t idx = (motion_phase_ == MotionPhase::Decel)
                              ? (table.size() - 1U - lut_segment_index_)
                              : lut_segment_index_;
  const auto &seg = table[idx];

  const int32_t steps_to_go = std::abs(target_position_ - current_position_);

  switch (motion_phase_) {
  case MotionPhase::Accel: {
    motion_velocity_arr32 += seg.slope_q16;
    printf(" Set value to %u\n", static_cast<uint16_t>(motion_velocity_arr32 >> 16) );

    ++lut_steps_done_;
    if (lut_steps_done_ >= DefaultSigmoidProfile::StepsPerAccelerationSegment) {
      lut_steps_done_ = 0;
      ++lut_segment_index_;
    }

    if (lut_segment_index_ >= table.size()) {
      lut_segment_index_ = 0;
      motion_phase_ = MotionPhase::Cruise;
    }
    break;
  }
  case MotionPhase::Cruise: {
    motion_velocity_arr32 = DefaultSigmoidProfile::EndArr16<<16;
    printf(" Set value to %u\n", static_cast<uint16_t>(motion_velocity_arr32 >> 16) );

    if (steps_to_go <=
        static_cast<int32_t>(DefaultSigmoidProfile::TotalAccelSteps)) {
      motion_phase_ = MotionPhase::Decel;
      lut_segment_index_ = 0;
      lut_steps_done_ = 0;
    }
    break;
  }
  case MotionPhase::Decel: {
    motion_velocity_arr32 -= seg.slope_q16; // reverse slope for decel
    printf(" Set value to %u\n", static_cast<uint16_t>(motion_velocity_arr32 >> 16) );

    ++lut_steps_done_;
    if (lut_steps_done_ >= DefaultSigmoidProfile::StepsPerAccelerationSegment) {
      lut_steps_done_ = 0;
      ++lut_segment_index_;
      // When all decel segments are consumed, hold the slowest ARR and stop
      // ramping
      if (lut_segment_index_ >= table.size()) {
        motion_velocity_arr32 = DefaultSigmoidProfile::StartArr16<<16;
        printf(" Set value to %u\n", static_cast<uint16_t>(motion_velocity_arr32 >> 16) );
        motion_phase_ = MotionPhase::Idle;
      }
    }
    break;
  }
  default:
    break;
  }
}

int main() {
  DefaultSigmoidProfile::make_table_runtime_debug();

  printf("StartArr16: %d\n", DefaultSigmoidProfile::StartArr16);
  printf("EndArr16:   %d\n", DefaultSigmoidProfile::EndArr16);
  printf("TotalAccelSteps: %u\n", DefaultSigmoidProfile::TotalAccelSteps);
  for(size_t i=0; i<DefaultSigmoidProfile::TotalAccelSteps+30; i++) {
      apply_sigmoid_step();
  }
  return 0;
}
