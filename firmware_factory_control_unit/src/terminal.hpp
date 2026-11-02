// Browser terminal (web UI "Terminal", WebSocket namespace terminal): commands for testing the
// inputs and outputs close to the hardware. The network thread queues a command line; the I/O
// thread, which owns the pins and buses, runs it in its next cycle and answers through the
// WebSocket outbox.
//
// The commands, their parameters and the value lookups are described in
// terminal_schema/commands.json; the builder generates the browser's catalog (completion, help,
// parameter tooltip) and Core/generated/terminal_commands.hh from it. Adding a command: an entry
// in commands.json, a function that writes its output with Out::line() and an entry in `handlers`
// (terminal.cpp; checked at compile time). A new lookup: a case in lookup().
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "network.hpp"

namespace fcu::terminal {

/// Network thread: queue a command line of session `id` with the client's request id.
void post(net::Server::SessionId id, std::uint16_t request_id, std::span<const char> line) noexcept;

/// Network thread: queue lookup number `lookup` (terminal_catalog::Lookup); false: no such lookup.
bool post_lookup(net::Server::SessionId id, std::uint16_t request_id, std::uint8_t lookup) noexcept;

/// I/O thread, every cycle: run the queued commands.
void cycle() noexcept;

}  // namespace fcu::terminal
