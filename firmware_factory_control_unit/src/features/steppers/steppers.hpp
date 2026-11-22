// The two steppers (stepper 1: sorter, stepper 2: elevator): TMC2209 drivers on one UART (settings,
// StallGuard), steps by timer + DMA (rtos TimerStepper: no interrupt per step), sensorless homing,
// the register targets, and the stepper test bench of the web UI (page "Schrittmotoren", WebSocket
// namespace ws::stepper). Runs in the I/O thread.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "network.hpp"
#include "ws_protocol.hh"

namespace fcu::steppers {

/// After the supply gate: the TMC2209s (settings), the step timers, DIAG. `supply_up`: the motor
/// supply is there; only then do the axes home (sensorless, by StallGuard), each one whose TMC2209
/// answers.
void setup(bool supply_up) noexcept;

/// One I/O cycle: the test bench's requests, homing, the register targets, position and status.
void cycle(std::uint32_t now) noexcept;

/// Sensorless homing between the cycles: StallGuard over UART, on every revision; revision 1 has
/// no DIAG wires, so this is its only way (revisions 2/3: DIAG stops it at once, the poll stays as
/// the fallback). SG_RESULT changes about every full step (5 ms at the homing speed of 200 full
/// steps/s); at the 50 ms of the cycle the motor would push about 10 full steps against the end stop.
inline constexpr std::uint32_t homing_poll_period_ms = 5;
/// The period to call homing_poll() at, 0: no homing running.
[[nodiscard]] std::uint32_t homing_poll_ms() noexcept;
/// Also right after a stall (stall_from_isr): the stop at once, not at the next poll.
void homing_poll(std::uint32_t now) noexcept;

/// DIAG of axis `axis` (0, 1) rose - a stall during the homing (EXTI, revisions 2/3): latched for
/// homing_poll(), which the caller then wakes the I/O thread for.
void stall_from_isr(std::size_t axis) noexcept;

/// A message of the test bench (ws::stepper, network thread): handled in the next cycle, answered
/// through the outbox; `reply` only for refusing it here (too large, queue full).
void post(net::Server::SessionId session, WsProtocol::Responder& reply, std::span<const std::uint8_t> message) noexcept;

}  // namespace fcu::steppers
