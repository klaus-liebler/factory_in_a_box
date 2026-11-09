// The IEC 61131-3 PLC runtime of the FCU (docs/plc-iec61131.md section 4): the bytecode VM of rtoos
// (rtos/plc/vm.hpp) in its own thread, coupled to the I/O cycle, the program and its configuration in
// the A/B flash slots (rtos/plc/slot_store.hpp), the switch of the control source, and the WebSocket
// namespace "plc" (schema/best_binary_buffers/plc.cs).
//
//   io thread (every 50 ms): ... cycle ... -> plc::io_cycle_done()
//   plc thread: input registers -> program variables, one cycle of the program,
//               output variables -> holding registers (only while the control source is IEC)
//
// Control source IEC: every holding register except the command registers belongs to the program;
// Modbus, OPC UA and the web UI cannot write them (the register bank's write filter answers
// Modbus exception 0x02 / OPC UA Bad_NotWritable). Reading stays allowed everywhere.
#pragma once

#include <cstddef>
#include <cstdint>

#include "network.hpp"

namespace fcu::plc {

/// Creates the PLC thread (from main(), before the kernel starts).
void create_thread() noexcept;

/// From the I/O thread after each cycle: the inputs are fresh, the program may run.
void io_cycle_done() noexcept;

/// Is the IEC program the control source right now?
[[nodiscard]] bool iec_controls() noexcept;

/// May somebody other than the program (terminal, web UI, RoArm teach) write holding register `reg`?
[[nodiscard]] bool external_write_allowed(std::uint16_t reg) noexcept;

/// WebSocket messages of the namespace "plc" (network thread).
void on_message(net::Server::SessionId id, std::uint16_t type, const std::uint8_t* data, std::size_t len) noexcept;

}  // namespace fcu::plc
