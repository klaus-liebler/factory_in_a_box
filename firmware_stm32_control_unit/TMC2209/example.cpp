// ----------------------------------------------------------------------------
// TMC2209.cpp
//
// Authors:
// Peter Polidoro peter@polidoro.io
// ----------------------------------------------------------------------------
#include "TMC2209.h"


TMC2209::TMC2209()
{
  hardware_serial_ptr_ = nullptr;
#if SOFTWARE_SERIAL_INCLUDED
  software_serial_ptr_ = nullptr;
#endif
  serial_baud_rate_ = 115200;
  serial_address_ = SERIAL_ADDRESS_0;
  hardware_enable_pin_ = -1;
  cool_step_enabled_ = false;
}

#if !defined(ARDUINO_ARCH_RENESAS)
void TMC2209::setup(HardwareSerial & serial,
  long serial_baud_rate,
  SerialAddress serial_address)
{
  hardware_serial_ptr_ = &serial;
  hardware_serial_ptr_->end();
  hardware_serial_ptr_->begin(serial_baud_rate);

  initialize(serial_baud_rate, serial_address);
}
#endif
#if defined(ESP32)
void TMC2209::setup(HardwareSerial & serial,
  long serial_baud_rate,
  SerialAddress serial_address,
  int16_t alternate_rx_pin,
  int16_t alternate_tx_pin)
{
  hardware_serial_ptr_ = &serial;
  if ((alternate_rx_pin < 0) || (alternate_tx_pin < 0))
  {
    // hardware_serial_ptr_->end();  // Causes issues with some versions of ESP32
    hardware_serial_ptr_->begin(serial_baud_rate);
  }
  else
  {
    // hardware_serial_ptr_->end();  // Causes issues with some versions of ESP32
    hardware_serial_ptr_->begin(serial_baud_rate, SERIAL_8N1, alternate_rx_pin, alternate_tx_pin);
  }

  initialize(serial_baud_rate, serial_address);
}
#elif defined(ARDUINO_ARCH_RP2040)
void TMC2209::setup(SerialUART & serial,
  long serial_baud_rate,
  SerialAddress serial_address,
  int16_t alternate_rx_pin,
  int16_t alternate_tx_pin)
{
  hardware_serial_ptr_ = &serial;
  if ((alternate_rx_pin < 0) || (alternate_tx_pin < 0))
  {
    serial.end();
    serial.begin(serial_baud_rate);
  }
  else
  {
    hardware_serial_ptr_->end();
    serial.setRX(alternate_rx_pin);
    serial.setTX(alternate_tx_pin);
    serial.begin(serial_baud_rate);
  }

  initialize(serial_baud_rate, serial_address);
}
#elif defined(ARDUINO_ARCH_RENESAS)
void TMC2209::setup(UART & serial,
  long serial_baud_rate,
  SerialAddress serial_address)
{
  hardware_serial_ptr_ = &serial;
  serial.end();
  serial.begin(serial_baud_rate);

  initialize(serial_baud_rate, serial_address);
}
#endif

#if SOFTWARE_SERIAL_INCLUDED
void TMC2209::setup(SoftwareSerial & serial,
  long serial_baud_rate,
  SerialAddress serial_address)
{
  software_serial_ptr_ = &serial;
  // software_serial_ptr_->end(); // Does not exist in some implementations
  software_serial_ptr_->begin(serial_baud_rate);

  initialize(serial_baud_rate, serial_address);
}
#endif

// unidirectional methods

void TMC2209::setHardwareEnablePin(uint8_t hardware_enable_pin)
{
  hardware_enable_pin_ = hardware_enable_pin;
  pinMode(hardware_enable_pin_, OUTPUT);
  digitalWrite(hardware_enable_pin_, HIGH);
}

void TMC2209::enable()
{
  if (hardware_enable_pin_ >= 0)
  {
    digitalWrite(hardware_enable_pin_, LOW);
  }
  chopper_config_.toff = toff_;
  writeStoredChopperConfig();
}

void TMC2209::disable()
{
  if (hardware_enable_pin_ >= 0)
  {
    digitalWrite(hardware_enable_pin_, HIGH);
  }
  chopper_config_.toff = TOFF_DISABLE;
  writeStoredChopperConfig();
}

void TMC2209::setMicrostepsPerStep(uint16_t microsteps_per_step)
{
  uint16_t microsteps_per_step_shifted = constrain_(microsteps_per_step,
    MICROSTEPS_PER_STEP_MIN,
    MICROSTEPS_PER_STEP_MAX);
  microsteps_per_step_shifted = microsteps_per_step >> 1;
  uint16_t exponent = 0;
  while (microsteps_per_step_shifted > 0)
  {
    microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
    ++exponent;
  }
  setMicrostepsPerStepPowerOfTwo(exponent);
}

void TMC2209::setMicrostepsPerStepPowerOfTwo(uint8_t exponent)
{
  switch (exponent)
  {
    case 0:
    {
      chopper_config_.mres = MRES_001;
      break;
    }
    case 1:
    {
      chopper_config_.mres = MRES_002;
      break;
    }
    case 2:
    {
      chopper_config_.mres = MRES_004;
      break;
    }
    case 3:
    {
      chopper_config_.mres = MRES_008;
      break;
    }
    case 4:
    {
      chopper_config_.mres = MRES_016;
      break;
    }
    case 5:
    {
      chopper_config_.mres = MRES_032;
      break;
    }
    case 6:
    {
      chopper_config_.mres = MRES_064;
      break;
    }
    case 7:
    {
      chopper_config_.mres = MRES_128;
      break;
    }
    case 8:
    default:
    {
      chopper_config_.mres = MRES_256;
      break;
    }
  }
  writeStoredChopperConfig();
}

void TMC2209::setRunCurrent(uint8_t percent)
{
  uint8_t run_current = percentToCurrentSetting(percent);
  driver_current_.irun = run_current;
  writeStoredDriverCurrent();
}

void TMC2209::setHoldCurrent(uint8_t percent)
{
  uint8_t hold_current = percentToCurrentSetting(percent);

  driver_current_.ihold = hold_current;
  writeStoredDriverCurrent();
}

void TMC2209::setHoldDelay(uint8_t percent)
{
  uint8_t hold_delay = percentToHoldDelaySetting(percent);

  driver_current_.iholddelay = hold_delay;
  writeStoredDriverCurrent();
}

void TMC2209::setAllCurrentValues(uint8_t run_current_percent,
  uint8_t hold_current_percent,
  uint8_t hold_delay_percent)
{
  uint8_t run_current = percentToCurrentSetting(run_current_percent);
  uint8_t hold_current = percentToCurrentSetting(hold_current_percent);
  uint8_t hold_delay = percentToHoldDelaySetting(hold_delay_percent);

  driver_current_.irun = run_current;
  driver_current_.ihold = hold_current;
  driver_current_.iholddelay = hold_delay;
  writeStoredDriverCurrent();
}

void TMC2209::setRMSCurrent(uint16_t mA,
  float rSense,
  float holdMultiplier)
{
  // Taken from https://github.com/teemuatlut/TMCStepper/blob/74e8e6881adc9241c2e626071e7328d7652f361a/src/source/TMCStepper.cpp#L41.

  uint8_t CS = 32.0*1.41421*mA/1000.0*(rSense+0.02)/0.325 - 1;
  // If Current Scale is too low, turn on high sensitivity R_sense and calculate again
  if (CS < 16) {
    enableVSense();
    CS = 32.0*1.41421*mA/1000.0*(rSense+0.02)/0.180 - 1;
  } else { // If CS >= 16, turn off high_sense_r
    disableVSense();
  }

  if (CS > 31) {
    CS = 31;
  }

  driver_current_.irun = CS;
  driver_current_.ihold = CS*holdMultiplier;
  writeStoredDriverCurrent();
}

void TMC2209::enableDoubleEdge()
{
  chopper_config_.double_edge = DOUBLE_EDGE_ENABLE;
  writeStoredChopperConfig();
}

void TMC2209::disableDoubleEdge()
{
  chopper_config_.double_edge = DOUBLE_EDGE_DISABLE;
  writeStoredChopperConfig();
}

void TMC2209::enableVSense()
{
  chopper_config_.vsense = VSENSE_ENABLE;
  writeStoredChopperConfig();
}

void TMC2209::disableVSense()
{
  chopper_config_.vsense = VSENSE_DISABLE;
  writeStoredChopperConfig();
}

void TMC2209::enableInverseMotorDirection()
{
  global_config_.shaft = 1;
  writeStoredGlobalConfig();
}

void TMC2209::disableInverseMotorDirection()
{
  global_config_.shaft = 0;
  writeStoredGlobalConfig();
}

void TMC2209::setStandstillMode(TMC2209::StandstillMode mode)
{
  pwm_config_.freewheel = mode;
  writeStoredPwmConfig();
}

void TMC2209::enableAutomaticCurrentScaling()
{
  pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_ON;
  writeStoredPwmConfig();
}

void TMC2209::disableAutomaticCurrentScaling()
{
  pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
  writeStoredPwmConfig();
}

void TMC2209::enableAutomaticGradientAdaptation()
{
  pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_ON;
  writeStoredPwmConfig();
}

void TMC2209::disableAutomaticGradientAdaptation()
{
  pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
  writeStoredPwmConfig();
}

void TMC2209::setPwmOffset(uint8_t pwm_amplitude)
{
  pwm_config_.pwm_offset = pwm_amplitude;
  writeStoredPwmConfig();
}

void TMC2209::setPwmGradient(uint8_t pwm_amplitude)
{
  pwm_config_.pwm_grad = pwm_amplitude;
  writeStoredPwmConfig();
}

void TMC2209::setPowerDownDelay(uint8_t power_down_delay)
{
  write(ADDRESS_TPOWERDOWN, power_down_delay);
}

void TMC2209::setReplyDelay(uint8_t reply_delay)
{
  if (reply_delay > REPLY_DELAY_MAX)
  {
    reply_delay = REPLY_DELAY_MAX;
  }
  ReplyDelay reply_delay_data;
  reply_delay_data.bytes = 0;
  reply_delay_data.replydelay = reply_delay;
  write(ADDRESS_REPLYDELAY, reply_delay_data.bytes);
}

void TMC2209::moveAtVelocity(int32_t microsteps_per_period)
{
  write(ADDRESS_VACTUAL, microsteps_per_period);
}

void TMC2209::moveUsingStepDirInterface()
{
  write(ADDRESS_VACTUAL, VACTUAL_STEP_DIR_INTERFACE);
}

void TMC2209::enableStealthChop()
{
  global_config_.enable_spread_cycle = 0;
  writeStoredGlobalConfig();
}

void TMC2209::disableStealthChop()
{
  global_config_.enable_spread_cycle = 1;
  writeStoredGlobalConfig();
}

void TMC2209::setCoolStepDurationThreshold(uint32_t duration_threshold)
{
  write(ADDRESS_TCOOLTHRS, duration_threshold);
}

void TMC2209::setStealthChopDurationThreshold(uint32_t duration_threshold)
{
  write(ADDRESS_TPWMTHRS, duration_threshold);
}

void TMC2209::setStallGuardThreshold(uint8_t stall_guard_threshold)
{
  write(ADDRESS_SGTHRS, stall_guard_threshold);
}

void TMC2209::enableCoolStep(uint8_t lower_threshold,
    uint8_t upper_threshold)
{
  lower_threshold = constrain_(lower_threshold, SEMIN_MIN, SEMIN_MAX);
  cool_config_.semin = lower_threshold;
  upper_threshold = constrain_(upper_threshold, SEMAX_MIN, SEMAX_MAX);
  cool_config_.semax = upper_threshold;
  write(ADDRESS_COOLCONF, cool_config_.bytes);
  cool_step_enabled_ = true;
}

void TMC2209::disableCoolStep()
{
  cool_config_.semin = SEMIN_OFF;
  write(ADDRESS_COOLCONF, cool_config_.bytes);
  cool_step_enabled_ = false;
}

void TMC2209::setCoolStepCurrentIncrement(CurrentIncrement current_increment)
{
  cool_config_.seup = current_increment;
  write(ADDRESS_COOLCONF, cool_config_.bytes);
}

void TMC2209::setCoolStepMeasurementCount(MeasurementCount measurement_count)
{
  cool_config_.sedn = measurement_count;
  write(ADDRESS_COOLCONF, cool_config_.bytes);
}







TMC2209::Status TMC2209::getStatus()
{
  DriveStatus drive_status;
  drive_status.bytes = 0;
  drive_status.bytes = read(ADDRESS_DRV_STATUS);
  return drive_status.status;
}

TMC2209::GlobalStatus TMC2209::getGlobalStatus()
{
  GlobalStatusUnion global_status_union;
  global_status_union.bytes = 0;
  global_status_union.bytes = read(ADDRESS_GSTAT);
  return global_status_union.global_status;
}

void TMC2209::clearReset()
{
  GlobalStatusUnion global_status_union;
  global_status_union.bytes = 0;
  global_status_union.global_status.reset = 1;
  write(ADDRESS_GSTAT, global_status_union.bytes);
}

void TMC2209::clearDriveError()
{
  GlobalStatusUnion global_status_union;
  global_status_union.bytes = 0;
  global_status_union.global_status.drv_err = 1;
  write(ADDRESS_GSTAT, global_status_union.bytes);
}




// private
void TMC2209::initialize(long serial_baud_rate,
  SerialAddress serial_address)
{
  serial_baud_rate_ = serial_baud_rate;

  setOperationModeToSerial(serial_address);
  setRegistersToDefaults();
  clearDriveError();

  minimizeMotorCurrent();
  disable();
  disableAutomaticCurrentScaling();
  disableAutomaticGradientAdaptation();
}






void TMC2209::setOperationModeToSerial(SerialAddress serial_address)
{
  serial_address_ = serial_address;

  global_config_.bytes = 0;
  global_config_.i_scale_analog = 0;
  global_config_.pdn_disable = 1;
  global_config_.mstep_reg_select = 1;
  global_config_.multistep_filt = 1;

  writeStoredGlobalConfig();
}



void TMC2209::minimizeMotorCurrent()
{
  driver_current_.irun = CURRENT_SETTING_MIN;
  driver_current_.ihold = CURRENT_SETTING_MIN;
  writeStoredDriverCurrent();
}
