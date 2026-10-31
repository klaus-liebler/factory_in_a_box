// Task manager of the web UI (WebSocket namespace tasks): the threads with state, priority,
// stack use (painted stacks) and CPU share since the last request (kernel run statistics),
// and the memory pools (packet buffers, wolfSSL pools).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "system.hpp"

namespace fcu::task_monitor {

/// A thread to show; its stack must have been painted with `paint` before it started.
void add(Kernel::Thread& thread, std::span<std::byte> stack) noexcept;
inline constexpr std::byte paint{0xA5};

/// Answer a tasks message (header included) into `reply`; returns the reply size (0: none).
std::size_t handle_message(std::uint16_t type, std::span<const std::uint8_t> message, std::span<std::uint8_t> reply) noexcept;

/// Bytes of `stack` ever used (from the deepest touched byte).
[[nodiscard]] std::size_t stack_used(std::span<const std::byte> stack) noexcept;

}  // namespace fcu::task_monitor
