// The I/O thread's cycle (every 50 ms): inputs into the register map, the register map into the
// outputs (valves, PWM, LEDs), the sensors, CAN, health; the function modules of src/features
// (steppers, RoArm, Malibotti) run in it too. The EXTI interrupts of every module end here.
#pragma once

#include <array>
#include <cstdint>
#include <span>

#include "network.hpp"
#include "system.hpp"
#include "ws_protocol.hh"

namespace fcu::io {

inline constexpr std::uint32_t cycle_ms = 50;

/// Power consumers (after the supply gate): pins, buses and their scans, sensors, drivers, the
/// function modules. `supply_up`: the motor supply is there (the steppers home only then).
void setup(bool supply_up) noexcept;

/// One cycle; `now` in ms.
void cycle(std::uint32_t now) noexcept;

/// Events from interrupts for the I/O thread, between the cycles: notification slot and bits.
inline constexpr std::size_t event_slot = 0;
inline constexpr std::uint32_t event_supply = 1u << 0;  ///< INA226: a measurement is ready (ALERT)
inline constexpr std::uint32_t event_stall = 1u << 1;   ///< TMC2209 DIAG: a stall during the homing
/// The thread that waits for the events (the I/O thread; before setup()).
void set_event_thread(Kernel::Thread& thread) noexcept;
/// Handle the event bits that woke the thread.
void events(std::uint32_t bits) noexcept;

/// The register bank's write hook (rtos::modbus::WriteHook; main.cpp): holding registers start ..
/// start+count-1 were written - by a protocol (Modbus, OPC UA, web, EtherCAT) in its thread, or by
/// the PLC, which calls it after its own writes. Commands that must not be missed between two cycles
/// (the WS2812 pixel registers, ws2812.hpp) are applied here.
void holding_written(void* context, std::uint16_t start, std::uint16_t count) noexcept;

/// The boot scan of an I2C bus: 128 bits, bit n = a device answered at address n.
/// The I2C buses by role (their numbers differ per revision: hw::i2c_tof2_bus, hw::i2c_supply_bus).
enum class I2cBus : std::uint8_t { Color = 0, Tof2 = 1, Supply = 2 };
inline constexpr std::size_t i2c_bus_count = 3;
[[nodiscard]] std::span<const std::uint8_t, 16> i2c_scan(I2cBus bus) noexcept;
/// The bus's number (I2Cn) on this hardware revision.
[[nodiscard]] unsigned i2c_bus_number(I2cBus bus) noexcept;

}  // namespace fcu::io
