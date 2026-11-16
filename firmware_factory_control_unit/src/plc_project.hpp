// The PLC project's source on the SD card (docs/plc-iec61131.md 4.5): /PLC/project.fcuplc.json, the
// IDE's project JSON, for "Upload from device". The program itself lives in the flash slots (plc.hpp)
// and runs without a card; the source is only for the IDE. PutProject writes it in chunks as
// /PLC/project.tmp and renames it once complete (a power loss keeps the old project or the new one),
// GetProject reads it in chunks. The network thread queues the requests, the I/O thread - which owns
// the card - answers them in its cycle (through the outbox).
#pragma once

#include <cstddef>
#include <cstdint>

#include <span>

#include "network.hpp"
#include "ws_protocol.hh"

namespace fcu::plc::project {

/// The largest project source the device keeps.
inline constexpr std::uint32_t max_bytes = 512 * 1024;

/// Network thread: queue a PutProjectRequest / GetProjectRequest (`frame`: the whole message); the I/O
/// thread answers. false: not one of them.
bool post(net::Server::SessionId id, WsProtocol::Responder& reply, std::span<const std::uint8_t> frame) noexcept;

/// I/O thread, every cycle: answer the queued requests.
void cycle() noexcept;

}  // namespace fcu::plc::project
