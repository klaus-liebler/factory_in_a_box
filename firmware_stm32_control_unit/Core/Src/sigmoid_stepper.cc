#include "sigmoid_stepper.hh"
#include <algorithm>
#include <cstdint>
#include "gpio.hh"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"



// ============================================================================
// RampStepper Implementation
// ============================================================================

SigmoidStepper::SigmoidStepper(gpio::Pin step_pin, gpio::Pin dir_pin)
        : step_pin_(step_pin),
            dir_pin_(dir_pin),
            motion_velocity_arr32(DefaultSigmoidProfile::StartArr32),
            motion_phase_(MotionPhase::Idle),
            motion_direction(true) {}

void SigmoidStepper::init() {
    gpio::Gpio::ConfigureGPIOOutput(step_pin_, false); // Initialize step pin low
    gpio::Gpio::ConfigureGPIOOutput(dir_pin_, false);  // Initialize dir pin low
    __HAL_RCC_TIM17_CLK_ENABLE();
    TIM17->PSC = kPrescaler;
    TIM17->ARR = motion_velocity_arr32>>16;
    TIM17->CCR1 = motion_velocity_arr32>>17; // 50% duty cycle
    TIM17->DIER |= TIM_DIER_UIE;    
    NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 1);
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
}

void SigmoidStepper::gotoPosition(int targetPosition) {
    const int32_t delta = static_cast<int32_t>(targetPosition) - current_position_;
    if (delta == 0) {
        return;
    }

    target_position_ = targetPosition;
    motion_direction = delta > 0;

    // Set direction pin accordingly
    gpio::Gpio::Set(dir_pin_, motion_direction);

    const uint32_t steps = static_cast<uint32_t>(std::abs(delta));
    
    use_sigmoid_profile_ = steps >= (DefaultSigmoidProfile::TotalAccelSteps * 2U);

    if (use_sigmoid_profile_) {
        // Force ARR range to the LUT design parameters
        motion_velocity_arr32 = DefaultSigmoidProfile::StartArr32;
    }
    motion_phase_ = MotionPhase::Accel;
    start_motion();
}

int32_t SigmoidStepper::getCurrentPosition() const {
    return current_position_;
}

bool SigmoidStepper::isMotionActive() const {
    return TIM_CR1_CEN & TIM17->CR1;
}




bool SigmoidStepper::getLastDirection() const {
    return motion_direction;
}

int32_t SigmoidStepper::getTargetPosition() const {
    return target_position_;
}

void SigmoidStepper::start_motion() {
    reset_lut_state_();
    TIM17->PSC = kPrescaler;
    TIM17->ARR = motion_velocity_arr32>>16;
    TIM17->CNT = 0;
    TIM17->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);
    TIM17->CR1 |= TIM_CR1_CEN;
}

void SigmoidStepper::handle_update_interrupt_() {
    // Toggle step pin
    gpio::Gpio::Toggle(step_pin_);

    // Count every toggle as one pulse
    current_position_ += motion_direction ? 1 : -1;
    if (use_sigmoid_profile_) {
        apply_sigmoid_step();
    }
    // Stop when target reached
    if (current_position_ == target_position_) {
        TIM17->CR1 &= ~TIM_CR1_CEN;
        motion_phase_ = MotionPhase::Idle;
    }
}

void SigmoidStepper::reset_lut_state_() {
    lut_segment_index_ = 0;
    lut_steps_done_ = 0;
}

void SigmoidStepper::apply_sigmoid_step() {
    const auto& table = DefaultSigmoidProfile::table;
    if (lut_segment_index_ >= table.size()) {
        return;
    }

    const std::size_t idx = (motion_phase_ == MotionPhase::Decel)
                                ? (table.size() - 1U - lut_segment_index_)
                                : lut_segment_index_;
    const auto& seg = table[idx];

    const int32_t steps_to_go = std::abs(target_position_ - current_position_);

    switch (motion_phase_) {
    case MotionPhase::Accel: {
        motion_velocity_arr32 += seg.slope_q16;
        TIM17->ARR = static_cast<uint16_t>(motion_velocity_arr32 >> 16);

        ++lut_steps_done_;
        if (lut_steps_done_ >= seg.steps) {
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
        motion_velocity_arr32 = DefaultSigmoidProfile::EndArr32;
        TIM17->ARR = static_cast<uint16_t>(motion_velocity_arr32 >> 16);

        if (steps_to_go <= static_cast<int32_t>(DefaultSigmoidProfile::TotalAccelSteps)) {
            motion_phase_ = MotionPhase::Decel;
            lut_segment_index_ = 0;
            lut_steps_done_ = 0;
        }
        break;
    }
    case MotionPhase::Decel: {
        motion_velocity_arr32 -= seg.slope_q16; // reverse slope for decel
        TIM17->ARR = static_cast<uint16_t>(motion_velocity_arr32 >> 16);

        ++lut_steps_done_;
        if (lut_steps_done_ >= seg.steps) {
            lut_steps_done_ = 0;
            ++lut_segment_index_;
            // When all decel segments are consumed, hold the slowest ARR and stop ramping
            if (lut_segment_index_ >= table.size()) {
                motion_velocity_arr32 = DefaultSigmoidProfile::StartArr32;
                TIM17->ARR = static_cast<uint16_t>(motion_velocity_arr32 >> 16);
                motion_phase_ = MotionPhase::Idle;
            }
        }
        break;
    }
    default:
        break;
    }
}

