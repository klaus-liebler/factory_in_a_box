#pragma once

#include "log.h"
#include <cstdint>

namespace gpio {
enum class Pin : uint8_t {
  PA00,
  PA01,
  PA02,
  PA03,
  PA04,
  PA05,
  PA06,
  PA07,
  PA08,
  PA09,
  PA10,
  PA11,
  PA12,
  PA13,
  PA14,
  PA15,
  NO_PIN = UINT8_MAX
};

class Gpio {
public:
  static void ConfigureGPIOOutput(Pin, bool) {}
  static void Set(Pin, bool) {}
  static void Toggle(Pin p) {
    //log_debug("Toggle pin %d", static_cast<int>(p));
  }
};
} // namespace gpio
