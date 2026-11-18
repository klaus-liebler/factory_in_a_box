// Register access over EtherCAT for the LAN9253 bring-up test (ethercat_test.cpp): the master reads
// and writes the same register map as Modbus (rtos::modbus::RegisterMap: holding registers through
// RegisterMap::write, so the PLC's write filter applies as for Modbus TCP/RTU).
//
// No mailbox, no CoE: two areas in the ESC's process RAM, which the master reaches with FPRD/FPWR
// in any AL state (no SyncManager covers them). The firmware polls the request every I/O cycle.
//
//   request  at 0x1000, response at 0x1100, same layout, little endian:
//     +0 u16 sequence   request: written LAST by the master (the trigger), never 0;
//                       response: written last by the firmware - the request's sequence when done
//     +2 u8  function   0x03 read holding, 0x04 read input, 0x10 write holding (Modbus codes)
//     +3 u8  exception  response: 0 or the Modbus exception code (1 function, 2 address, 3 value)
//     +4 u16 start      first register address (0-based, as the Modbus PDU address)
//     +6 u16 count      1..max_count registers
//     +8 u16 values[count]  request: the values to write; response: the values read
//
// Pure (no ESC access): tests/host/ethercat_registers_test.cpp.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/modbus/modbus.hpp"

namespace fcu::ethercat::register_access {

inline constexpr std::uint16_t request_address = 0x1000, response_address = 0x1100;
inline constexpr std::size_t header_bytes = 8, max_count = 64, area_bytes = header_bytes + 2 * max_count;

using Area = std::array<std::uint8_t, area_bytes>;

constexpr std::uint16_t get16(std::span<const std::uint8_t> b, std::size_t at) noexcept {
    return static_cast<std::uint16_t>(b[at] | (b[at + 1] << 8));
}
constexpr void put16(std::span<std::uint8_t> b, std::size_t at, std::uint16_t v) noexcept {
    b[at] = static_cast<std::uint8_t>(v);
    b[at + 1] = static_cast<std::uint8_t>(v >> 8);
}

/// Bytes of the request that follow the header (the values of a write).
constexpr std::size_t request_value_bytes(std::span<const std::uint8_t> header) noexcept {
    const std::uint16_t count = get16(header, 6);
    return header[2] == rtos::modbus::function::write_multiple && count <= max_count ? 2u * count : 0u;
}

/// Bytes of the response to write back (header and the values read).
constexpr std::size_t response_bytes(std::span<const std::uint8_t> response) noexcept {
    const bool read = response[2] == rtos::modbus::function::read_holding || response[2] == rtos::modbus::function::read_input;
    return header_bytes + (read && response[3] == 0 ? 2u * get16(response, 6) : 0u);
}

/// Executes the request (header and values as in ESC RAM) on `map` and fills `response`: header with
/// the request's sequence, function, start and count, the exception, and the values read.
inline void execute(std::span<const std::uint8_t> request, rtos::modbus::RegisterMap& map, Area& response) noexcept {
    using rtos::modbus::Exception;
    using rtos::modbus::Table;
    namespace fn = rtos::modbus::function;
    response.fill(0);
    const std::uint8_t function = request[2];
    const std::uint16_t start = get16(request, 4), count = get16(request, 6);
    put16(response, 0, get16(request, 0));
    response[2] = function;
    put16(response, 4, start);
    put16(response, 6, count);
    Exception e = Exception::None;
    if (function != fn::read_holding && function != fn::read_input && function != fn::write_multiple) {
        e = Exception::IllegalFunction;
    } else if (count == 0 || count > max_count) {
        e = Exception::IllegalDataValue;
    } else {
        const Table table = function == fn::read_input ? Table::Input : Table::Holding;
        if (std::uint32_t{start} + count > map.size(table)) {
            e = Exception::IllegalDataAddress;
        } else if (function == fn::write_multiple) {
            std::array<std::uint16_t, max_count> values{};
            for (std::size_t i = 0; i < count; ++i) values[i] = get16(request, header_bytes + 2 * i);
            e = map.write(start, std::span<const std::uint16_t>(values.data(), count));
        } else {
            std::array<std::uint16_t, max_count> values{};
            e = map.read(table, start, std::span<std::uint16_t>(values.data(), count));
            if (e == Exception::None) {
                for (std::size_t i = 0; i < count; ++i) put16(response, header_bytes + 2 * i, values[i]);
            }
        }
    }
    response[3] = static_cast<std::uint8_t>(e);
}

}  // namespace fcu::ethercat::register_access
