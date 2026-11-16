// Task manager of the web UI (WebSocket namespace tasks): the threads with state, priority,
// stack use (painted stacks) and CPU share since the last request (kernel run statistics),
// and the memory pools (packet buffers, wolfSSL pools).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "system.hpp"
#include "ws_protocol.hh"

namespace fcu::task_monitor {

/// A thread to show; its stack must have been painted with `paint` before it started.
void add(Kernel::Thread& thread, std::span<std::byte> stack) noexcept;
inline constexpr std::byte paint{0xA5};

/// Network thread: answer a tasks message (`frame`: header included).
void handle_message(WsProtocol::Responder& reply, std::span<const std::uint8_t> frame) noexcept;

/// Bytes of `stack` ever used (from the deepest touched byte).
[[nodiscard]] std::size_t stack_used(std::span<const std::byte> stack) noexcept;

}  // namespace fcu::task_monitor
