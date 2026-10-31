// Start-up of the FCU: identity, reset cause, banner, and the supply gate.
//
// The power consumers (Ethernet PHY, steppers, servos, valves) start only when the supply
// carries at least 18 V. If it does not, the board asks a USB-C supply for 20 V (USB Power
// Delivery sink on UCPD1) and waits - the former App::WaitForSupplyVoltage().
#pragma once

#include <cstdint>

#include "rtos/usb_pd/sink.hpp"

namespace fcu::boot {

/// Values of the system page's reset cause (the former App::ResetCauseCode()).
enum class ResetCause : std::uint8_t {
    Unknown = 0,
    IndependentWatchdog = 1,
    WindowWatchdog = 2,
    Software = 3,
    LowPower = 4,
    BrownOut = 5,
    PinOrPowerOn = 6,
};

/// Reads and clears the reset flags (once, first thing).
void read_reset_cause() noexcept;
[[nodiscard]] ResetCause reset_cause() noexcept;

/// The firmware embeds the identity of one board (the builder's device_ids.hh): false if this
/// chip is another one.
[[nodiscard]] bool identity_matches() noexcept;

/// Boot banner and the identity registers (chip ID, version).
void announce() noexcept;

/// Blocks until the supply is up (see above). False if the supply monitor does not answer.
bool wait_for_supply() noexcept;

/// USB-PD state for the registers and the log (I/O thread).
void update_pd_registers() noexcept;
void poll_pd_events() noexcept;

}  // namespace fcu::boot
