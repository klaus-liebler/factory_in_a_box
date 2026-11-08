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
/// `supply_up`: the motor supply is there; only then do the steppers home (sensorless, by
/// StallGuard), each one whose TMC2209 answers.
void setup(bool supply_up) noexcept;

/// One cycle; `now` in ms.
void cycle(std::uint32_t now) noexcept;

/// Sensorless homing between the cycles: StallGuard over UART (the TMC2209's DIAG outputs are not
/// wired). SG_RESULT changes about every full step (5 ms at the homing speed of 200 full steps/s);
/// at the 50 ms of the cycle the motor would push about 10 full steps against the end stop.
inline constexpr std::uint32_t homing_poll_period_ms = 5;
/// The period to call homing_poll() at, 0: no homing running.
[[nodiscard]] std::uint32_t homing_poll_ms() noexcept;
void homing_poll(std::uint32_t now) noexcept;

/// Events from interrupts for the I/O thread, between the cycles: notification slot and bits.
inline constexpr std::size_t event_slot = 0;
inline constexpr std::uint32_t event_supply = 1u << 0;  ///< INA226: a measurement is ready (ALERT)
inline constexpr std::uint32_t event_stall = 1u << 1;   ///< TMC2209 DIAG: a stall during the homing
/// The thread that waits for the events (the I/O thread; before setup()).
void set_event_thread(Kernel::Thread& thread) noexcept;
/// Handle the event bits that woke the thread.
void events(std::uint32_t bits) noexcept;

/// The boot scan of an I2C bus: 128 bits, bit n = a device answered at address n.
/// The I2C buses by role (their numbers differ per revision: hw::i2c_tof2_bus, hw::i2c_supply_bus).
enum class I2cBus : std::uint8_t { Color = 0, Tof2 = 1, Supply = 2 };
inline constexpr std::size_t i2c_bus_count = 3;
[[nodiscard]] std::span<const std::uint8_t, 16> i2c_scan(I2cBus bus) noexcept;
/// The bus's number (I2Cn) on this hardware revision.
[[nodiscard]] unsigned i2c_bus_number(I2cBus bus) noexcept;

}  // namespace fcu::io
