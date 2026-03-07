#pragma once

#include "gpio.hh"
#include "log.h"

#include <climits>
#include <cstdint>
#include <utility>
#include <vector>

#define interrupts() __enable_irq()
#define noInterrupts() __disable_irq()

enum class Channel_And_Gain {
  CH_A_GAIN_128 = 0x40,
  CH_B_GAIN_32 = 0x50,
  CH_A_GAIN_64 = 0x54,
};

class HX711 {
  struct CalibrationPoint {
    int32_t reading;
    double weight;
  };

  enum class ReadoutState {
    WAIT_FOR_DATA_READY,
    ERROR,
  };

private:
  gpio::Pin clk_pin;
  gpio::Pin data_pin;
  std::vector<Channel_And_Gain> readout_order;
  int32_t current_readout_index{-1};
  int32_t channelA_last_reading{INT32_MIN};
  int32_t channelB_last_reading{INT32_MIN};
  CalibrationPoint calibration_points_chA[2] = {
    {0, 0.0},
    {1, 1.0},
  };
  CalibrationPoint calibration_points_chB[2] = {
    {0, 0.0},
    {1, 1.0},
  };
  ReadoutState readout_state{ReadoutState::ERROR};

  static uint8_t modeToPulseCount(Channel_And_Gain mode) {
    switch (mode) {
      case Channel_And_Gain::CH_A_GAIN_128:
        return 1;
      case Channel_And_Gain::CH_B_GAIN_32:
        return 2;
      case Channel_And_Gain::CH_A_GAIN_64:
        return 3;
    }
    return 1;
  }

  uint8_t shiftInMsbFirst() {
    uint8_t value = 0;
    for (uint8_t i = 0; i < 8; ++i) {
      gpio::Gpio::Set(clk_pin, true);
      value |= (gpio::Gpio::Get(data_pin) ? 1U : 0U) << (7U - i);
      gpio::Gpio::Set(clk_pin, false);
    }
    return value;
  }

  void updateLastReading(Channel_And_Gain mode_for_result, int32_t reading) {
    switch (mode_for_result) {
      case Channel_And_Gain::CH_A_GAIN_128:
      case Channel_And_Gain::CH_A_GAIN_64:
        channelA_last_reading = reading;
        break;
      case Channel_And_Gain::CH_B_GAIN_32:
        channelB_last_reading = reading;
        break;
    }
  }

  int32_t readOneCycleBitBanged(Channel_And_Gain mode_to_request_next) {
    uint8_t data[3] = {0};

    noInterrupts();
    data[2] = shiftInMsbFirst();
    data[1] = shiftInMsbFirst();
    data[0] = shiftInMsbFirst();

    const uint8_t pulses = modeToPulseCount(mode_to_request_next);
    log_debug("HX711(GPIO): start bit-banged read, request next mode=0x%02X, pulses=%u",
              static_cast<unsigned>(mode_to_request_next),
              static_cast<unsigned>(pulses));
    for (uint8_t i = 0; i < pulses; ++i) {
      gpio::Gpio::Set(clk_pin, true);
      gpio::Gpio::Set(clk_pin, false);
    }
    interrupts();

    const uint8_t filler = (data[2] & 0x80U) ? 0xFFU : 0x00U;
    const uint32_t value =
      ((uint32_t)filler << 24)
      | ((uint32_t)data[2] << 16)
      | ((uint32_t)data[1] << 8)
      | (uint32_t)data[0];

    const int32_t result = (int32_t)value;
    log_debug("HX711(GPIO): raw read complete, value=%ld", (long)result);
    return result;
  }

public:
  HX711(gpio::Pin clk_pin,
        gpio::Pin data_pin,
        std::vector<Channel_And_Gain> query_order = {
          Channel_And_Gain::CH_A_GAIN_128,
          Channel_And_Gain::CH_B_GAIN_32,
        })
    : clk_pin(clk_pin),
      data_pin(data_pin),
      readout_order(std::move(query_order)) {
    gpio::Gpio::ConfigureGPIOOutput(this->clk_pin, false);
    gpio::Gpio::ConfigureGPIOInput(this->data_pin);
  }

  void setup(void) {
    log_debug("HX711(GPIO): setup start");
    if (readout_order.empty()) {
      log_error("HX711(GPIO): setup failed, readout_order is empty");
      readout_state = ReadoutState::ERROR;
      return;
    }
    gpio::Gpio::Set(clk_pin, false);
    current_readout_index = -1;
    channelA_last_reading = INT32_MIN;
    channelB_last_reading = INT32_MIN;
    readout_state = ReadoutState::WAIT_FOR_DATA_READY;
    log_debug("HX711(GPIO): setup done, order size=%u", static_cast<unsigned>(readout_order.size()));
  }

  bool getLastRawValueA(int32_t *out_value) {
    if (channelA_last_reading == INT32_MIN) {
      return false;
    }
    *out_value = channelA_last_reading;
    return true;
  }

  bool getLastRawValueB(int32_t *out_value) {
    if (channelB_last_reading == INT32_MIN) {
      return false;
    }
    *out_value = channelB_last_reading;
    return true;
  }

  bool getLastCalibratedWeightA(double *out_value) {
    if (channelA_last_reading == INT32_MIN) {
      return false;
    }
    int32_t reading_1 = calibration_points_chA[0].reading;
    double weight_1 = calibration_points_chA[0].weight;
    int32_t reading_2 = calibration_points_chA[1].reading;
    double weight_2 = calibration_points_chA[1].weight;

    if (reading_1 == reading_2) {
      return false;
    }

    *out_value = weight_1
      + (static_cast<double>(channelA_last_reading) - reading_1)
          * (weight_2 - weight_1)
          / (reading_2 - reading_1);
    return true;
  }

  bool getLastCalibratedWeightB(double *out_value) {
    if (channelB_last_reading == INT32_MIN) {
      return false;
    }
    int32_t reading_1 = calibration_points_chB[0].reading;
    double weight_1 = calibration_points_chB[0].weight;
    int32_t reading_2 = calibration_points_chB[1].reading;
    double weight_2 = calibration_points_chB[1].weight;

    if (reading_1 == reading_2) {
      return false;
    }

    *out_value = weight_1
      + (static_cast<double>(channelB_last_reading) - reading_1)
          * (weight_2 - weight_1)
          / (reading_2 - reading_1);
    return true;
  }

  void loop(void) {
    if (readout_state == ReadoutState::ERROR) {
      log_error("HX711(GPIO): loop in ERROR state");
      return;
    }

    if (gpio::Gpio::Get(data_pin)) {
      log_debug("HX711(GPIO): WAIT_FOR_DATA_READY, DOUT is HIGH");
      return;
    }

    const auto next_mode = readout_order[(current_readout_index + 1) % readout_order.size()];
    log_debug("HX711(GPIO): DOUT ready, reading cycle; next_mode=0x%02X, current_index=%ld",
              static_cast<unsigned>(next_mode),
              (long)current_readout_index);
    const int32_t reading = readOneCycleBitBanged(next_mode);

    if (current_readout_index >= 0) {
      const auto mode_for_result = readout_order[current_readout_index];
      updateLastReading(mode_for_result, reading);
      log_debug("HX711(GPIO): stored reading=%ld for mode=0x%02X at index=%ld",
                (long)reading,
                static_cast<unsigned>(mode_for_result),
                (long)current_readout_index);
    } else {
      log_debug("HX711(GPIO): priming cycle complete, first sample intentionally not assigned");
    }

    current_readout_index = (current_readout_index + 1) % readout_order.size();
    log_debug("HX711(GPIO): next current_readout_index=%ld", (long)current_readout_index);
  }
};