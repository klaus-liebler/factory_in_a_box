#pragma once

#include "gpio.hh"
#include "stm32g431xx.h"
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "sigmoid_lut.hh"

/**
 * RampStepper: Generates trapezoidal motion profiles with automatic
 * acceleration/deceleration. Manages TIM17 for step pulse generation and
 * position tracking.
 */
class SigmoidStepper {
public:
  /**
   * Constructor: initializes with step and direction pins.
   * @param step_pin GPIO pin for step signal
   * @param dir_pin GPIO pin for direction signal
   */
  SigmoidStepper(gpio::Pin step_pin, gpio::Pin dir_pin);

  /**
   * Initialize the RampStepper subsystem.
   * Must be called once during setup.
   */
  void init();

  /**
   * Plan motion to an absolute position.
   * @param targetPosition target position in steps (can be positive or
   * negative)
   */
  void gotoPosition(int targetPosition);

  /**
   * Get the current position.
   * @return current position in steps
   */
  int32_t getCurrentPosition() const;

  /**
   * Check if motion is active.
   * @return true if motor is currently moving
   */
  bool isMotionActive() const;


  /**
   * Get the direction of the last motion command.
   * @return true if moving forward, false if backward
   */
  bool getLastDirection() const;

  /**
   * Get target position from last gotoPosition call.
   * @return target position in steps
   */
  int32_t getTargetPosition() const;

  /**
   * Handle timer update interrupt (called by IRQ handler).
   * Public for extern C accessibility.
   */
  void handle_update_interrupt_();

private:
  // Pin configuration
  gpio::Pin step_pin_;
  gpio::Pin dir_pin_;


  using DefaultSigmoidProfile = SigmoidProfile<500, 2000, 16, 3>;



  // Current motion state

  bool motion_direction{true};



  bool use_sigmoid_profile_{false};

  // Position tracking
  int32_t current_position_{0};
  int32_t target_position_{0};


  // Helper to start motion
  void start_motion();

  void reset_lut_state_();
  void apply_sigmoid_step();

};
