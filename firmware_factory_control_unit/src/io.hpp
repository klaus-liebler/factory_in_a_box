// The I/O thread's cycle (every 50 ms): inputs into the register map, the register map into
// the outputs, the sensors, steppers, LEDs, CAN and the RoArm - the former Io class with its
// SetupAndLoop members, in one thread as before.
#pragma once

#include <array>
#include <cstdint>
#include <span>

namespace fcu::io {

inline constexpr std::uint32_t cycle_ms = 50;

/// Power consumers (after the supply gate): pins, buses, sensors, drivers. Then the bus scans.
void setup() noexcept;

/// One cycle; `now` in ms.
void cycle(std::uint32_t now) noexcept;

/// The boot scan of I2C bus 1, 2 or 4: 128 bits, bit n = a device answered at address n.
[[nodiscard]] std::span<const std::uint8_t, 16> i2c_scan(unsigned bus) noexcept;

}  // namespace fcu::io
