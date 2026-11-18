// Register access over EtherCAT (src/ethercat_registers.hpp): requests as the master writes them into
// the ESC's process RAM, executed on a RegisterBank like the one Modbus uses.
#include <gtest/gtest.h>

#include <cstdint>
#include <initializer_list>

#include "ethercat_registers.hpp"
#include "rtos/modbus/register_bank.hpp"

namespace ra = fcu::ethercat::register_access;

namespace {

using Bank = rtos::modbus::RegisterBank<120, 10>;

ra::Area request(std::uint16_t sequence, std::uint8_t function, std::uint16_t start, std::uint16_t count,
                 std::initializer_list<std::uint16_t> values = {}) {
    ra::Area r{};
    ra::put16(r, 0, sequence);
    r[2] = function;
    ra::put16(r, 4, start);
    ra::put16(r, 6, count);
    std::size_t at = ra::header_bytes;
    for (const auto v : values) {
        ra::put16(r, at, v);
        at += 2;
    }
    return r;
}

}  // namespace

TEST(EthercatRegisters, WriteHoldingThenReadBack) {
    Bank bank;
    ra::Area response{};
    const auto write = request(7, 0x10, 100, 2, {1, 4});  // the WS2812 patterns of both channels
    EXPECT_EQ(ra::request_value_bytes(write), 4u);
    ra::execute(write, bank, response);
    EXPECT_EQ(ra::get16(response, 0), 7);
    EXPECT_EQ(response[3], 0);
    EXPECT_EQ(ra::response_bytes(response), ra::header_bytes);
    EXPECT_EQ(bank.holding(100), 1);
    EXPECT_EQ(bank.holding(101), 4);

    ra::execute(request(8, 0x03, 100, 2), bank, response);
    EXPECT_EQ(response[3], 0);
    EXPECT_EQ(ra::response_bytes(response), ra::header_bytes + 4);
    EXPECT_EQ(ra::get16(response, ra::header_bytes), 1);
    EXPECT_EQ(ra::get16(response, ra::header_bytes + 2), 4);
}

TEST(EthercatRegisters, ReadInput) {
    Bank bank;
    bank.set_input(3, 0xBEEF);
    ra::Area response{};
    ra::execute(request(1, 0x04, 3, 1), bank, response);
    EXPECT_EQ(response[3], 0);
    EXPECT_EQ(ra::get16(response, ra::header_bytes), 0xBEEF);
}

TEST(EthercatRegisters, ExceptionsAsModbus) {
    Bank bank;
    ra::Area response{};
    ra::execute(request(1, 0x05, 0, 1), bank, response);
    EXPECT_EQ(response[3], 1);  // illegal function
    ra::execute(request(2, 0x03, 119, 2), bank, response);
    EXPECT_EQ(response[3], 2);  // past the end
    ra::execute(request(3, 0x03, 0, 0), bank, response);
    EXPECT_EQ(response[3], 3);  // count 0
    ra::execute(request(4, 0x03, 0, ra::max_count + 1), bank, response);
    EXPECT_EQ(response[3], 3);
    EXPECT_EQ(ra::response_bytes(response), ra::header_bytes);  // no values with an exception
}

TEST(EthercatRegisters, WriteFilterAppliesAsForModbus) {
    Bank bank;
    bank.set_write_filter({nullptr, [](void*, std::uint16_t start, std::uint16_t) noexcept { return start != 100; }});
    ra::Area response{};
    ra::execute(request(1, 0x10, 100, 1, {3}), bank, response);
    EXPECT_EQ(response[3], 2);  // refused like Modbus: IllegalDataAddress, nothing written
    EXPECT_EQ(bank.holding(100), 0);
}
