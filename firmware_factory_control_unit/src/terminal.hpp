// Browser terminal (web UI "Terminal", WebSocket namespace terminal): commands for testing the
// inputs and outputs close to the hardware. The network thread queues a command line; the I/O
// thread, which owns the pins and buses, runs it in its next cycle and answers through the
// WebSocket outbox.
//
// The commands, their parameters and the value lookups are described in
// schema/terminal/commands.cs (C# classes); the builder generates the browser's catalog (completion,
// help, parameter tooltip) and generated/terminal_commands.hh from it. Adding a command: a class
// with [TerminalCommand] in commands.cs, a function that writes its output with Out::line() and an
// entry in `handlers` (terminal.cpp; checked at compile time). A new lookup: a member of the enum
// Lookup in commands.cs and a case in lookup().
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "network.hpp"

namespace fcu::terminal {

enum class Posted : std::uint8_t {
    queued,   ///< the I/O thread answers
    full,     ///< the queue is full: nothing queued, the caller answers
    invalid,  ///< no such lookup: nothing queued, the caller answers
};

/// Network thread: queue a command line of session `id`; `correlation` from the request's frame header.
Posted post(net::Server::SessionId id, std::uint16_t correlation, std::span<const char> line) noexcept;

/// Network thread: queue lookup number `lookup` (terminal_catalog::Lookup).
Posted post_lookup(net::Server::SessionId id, std::uint16_t correlation, std::uint8_t lookup) noexcept;

/// I/O thread, every cycle: run the queued commands.
void cycle() noexcept;

}  // namespace fcu::terminal
