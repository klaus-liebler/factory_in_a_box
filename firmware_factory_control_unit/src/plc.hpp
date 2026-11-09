// The IEC 61131-3 PLC runtime of the FCU (docs/plc-iec61131.md section 4): the bytecode VM of rtoos
// (rtos/plc/vm.hpp) in its own thread, coupled to the I/O cycle, the program and its configuration in
// the A/B flash slots (rtos/plc/slot_store.hpp), the switch of the control source, and the WebSocket
// namespace "plc" (schema/best_binary_buffers/plc.cs).
//
//   io thread (every 50 ms): ... cycle ... -> plc::io_cycle_done()
//   plc thread: input registers -> program variables, one cycle of the program,
//               output variables -> holding registers (only while the control source is IEC)
//
// Control source IEC: every holding register except the parameters belongs to the program (outputs
// every cycle, commands through the FCU library blocks); Modbus, OPC UA and the web UI cannot write them (the register bank's write filter answers
// Modbus exception 0x02 / OPC UA Bad_NotWritable). Reading stays allowed everywhere.
#pragma once

#include <cstddef>
#include <cstdint>

#include "network.hpp"
#include "ws_protocol.hh"

namespace fcu::plc {

/// Creates the PLC thread (from main(), before the kernel starts).
void create_thread() noexcept;

/// From the I/O thread after each cycle: the inputs are fresh, the program may run.
void io_cycle_done() noexcept;

/// Is the IEC program the control source right now?
[[nodiscard]] bool iec_controls() noexcept;

/// May somebody other than the program (terminal, web UI, RoArm teach) write holding register `reg`?
[[nodiscard]] bool external_write_allowed(std::uint16_t reg) noexcept;

/// The PLC's state for the terminal (`plc`), read from any thread.
struct Status {
    const char* state;  ///< NO_PROGRAM, STOP, RUN, ERROR
    bool iec;           ///< control source IEC 61131 (else fieldbus)
    bool autostart;
    std::uint32_t image_bytes, max_image_bytes, cycles, last_instructions, max_instructions;
    std::uint8_t trap;  ///< 0: none
    std::uint16_t trap_code;
    std::uint32_t trap_pc;
    const char* problem;  ///< why downloads and switching are refused ("": they are not)
    std::uint8_t forces;  ///< forced values (any browser's)
};
[[nodiscard]] Status status() noexcept;

/// Terminal (I/O thread): queue a command for the PLC thread - it may write the flash, the I/O thread
/// does not wait for that. The PLC thread logs the result. false: the queue is full. The debug lease
/// of a browser does not hold the terminal back.
bool command(WsProtocol::plc::PlcCommand c) noexcept;

/// Network thread: a WebSocket closed - its debug lease ends, its forced values stay.
void session_closed(net::Server::SessionId id) noexcept;

/// WebSocket messages of the namespace "plc" (network thread).
void on_message(net::Server::SessionId id, std::uint16_t type, const std::uint8_t* data, std::size_t len) noexcept;

}  // namespace fcu::plc
