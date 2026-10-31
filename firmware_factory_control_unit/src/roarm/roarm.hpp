// Waveshare RoArm-M3 on the Feetech servo bus (UART7): joint tracking, teach mode with joint
// and Cartesian jogging, missions (joint moves, mission GPIOs, delays) from the flash, started
// through the holding register RoArmMissionControl (index > 0; the register shows the running
// mission, 0 when done, -1 not found, -2 teach mode active, -3 broken).
//
// Everything runs in the I/O thread, which owns the servo bus, the tracker and the mission file
// system. WebSocket messages (namespace roarm) are queued by the network thread and answered
// from the I/O thread through net::outbox.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "network.hpp"

namespace fcu::roarm {

void setup() noexcept;
void cycle(std::uint32_t now) noexcept;

/// Network thread: queue a roarm message of session `id` (the whole message, header included).
void post(net::Server::SessionId id, std::span<const std::uint8_t> message) noexcept;

}  // namespace fcu::roarm
