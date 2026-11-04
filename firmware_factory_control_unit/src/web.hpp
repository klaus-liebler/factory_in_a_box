// WebSocket application protocol (wss://<hostname>.local/ws, ws_protocol.hh from
// schema/best_binary_buffers/*.cs): every message starts with namespace id and type id (u16 LE).
//
//   modbus  GetRegisters / WriteHolding: the register map for the web UI's register panel
//   system  SystemInfo request; LogMessage events: every log record to every client at once
//           (the device keeps no history; the browser's Terminal page does)
//   terminal command list and commands of the browser terminal (terminal.hpp)
//   roarm   teach mode, jogging, missions (roarm.hpp)
//   tasks   thread and pool list (task_monitor.hpp)
//
// Handlers run in the network thread. Events from the I/O thread (pose feedback, pneumatics)
// go through net::outbox.
#pragma once

#include <cstddef>
#include <span>

#include "network.hpp"
#include "rtos/net/service.hpp"

namespace fcu::web {

void on_opened(void*, net::Server::SessionId id) noexcept;
void on_message(void*, net::Server::SessionId id, std::span<const std::byte> data, bool binary) noexcept;
void on_closed(void*, net::Server::SessionId id) noexcept;

/// Network-thread service: sends new log records to every WebSocket client.
struct LogMirror {
    void service() noexcept;
    [[nodiscard]] std::uint32_t next_deadline() const noexcept;
};
LogMirror& log_mirror() noexcept;

/// Send an encoded message (from the I/O thread: through the outbox).
void broadcast_from_io(std::span<const std::uint8_t> message) noexcept;

}  // namespace fcu::web
