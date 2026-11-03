// The I/O thread's cycle (every 50 ms): inputs into the register map, the register map into
// the outputs, the sensors, steppers, LEDs, CAN and the RoArm - the former Io class with its
// SetupAndLoop members, in one thread as before.
#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "system.hpp"

namespace fcu::io {

inline constexpr std::uint32_t cycle_ms = 50;

/// Power consumers (after the supply gate): pins, buses, sensors, drivers. Then the bus scans.
void setup() noexcept;

/// One cycle; `now` in ms.
void cycle(std::uint32_t now) noexcept;

/// Events from interrupts for the I/O thread, between the cycles: notification slot and bits.
inline constexpr std::size_t event_slot = 0;
inline constexpr std::uint32_t event_supply = 1u << 0;  ///< INA226: a measurement is ready (ALERT)
/// The thread that waits for the events (the I/O thread; before setup()).
void set_event_thread(Kernel::Thread& thread) noexcept;
/// Handle the event bits that woke the thread.
void events(std::uint32_t bits) noexcept;

/// The boot scan of I2C bus 1, 2 or 4: 128 bits, bit n = a device answered at address n.
[[nodiscard]] std::span<const std::uint8_t, 16> i2c_scan(unsigned bus) noexcept;

}  // namespace fcu::io
