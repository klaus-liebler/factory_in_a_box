// The FCU's EtherCAT configuration (src/ethercat_config.hpp): SII and object dictionary.
//
// Regression: the configuration area took the PDI type from register 0x0140, read before READY - it
// reads 0x00 then. The reload loaded "no PDI", the configuration failed (DL status: PDI not
// operational, HW_CFG never READY) and both PHYs stayed without link. Now the PDI type is a constant.
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "ethercat_config.hpp"
#include "register_map.hpp"
#include "rtos/ethercat/coe.hpp"
#include "rtos/modbus/register_bank.hpp"

namespace {

namespace ecat = fcu::ethercat;
namespace ec = rtos::ethercat;
namespace rm = register_map;
constexpr auto image = ecat::sii_image(0xCAFE'F00D);
constexpr std::string_view software = "test";
using Od = ecat::Dictionary<3, 0xCAFE'F00D, &software>;

std::uint8_t byte_of(std::size_t i) { return static_cast<std::uint8_t>(image[i / 2] >> (8 * (i % 2))); }

TEST(EthercatConfig, ConfigurationAreaKeepsTheStrappedPdiType) {
    // DS00003421 13.4: the reload must not change the PDI type the EE_EMUL straps selected
    EXPECT_EQ(image[0] & 0xFF, 0x82) << "SPI EtherCAT Direct Mapped (EE_EMUL 100)";
    EXPECT_EQ(image[0] >> 8, 0) << "ESC configuration: the PDI acknowledges the AL states";
    std::uint8_t crc = 0xFF;  // CRC-8 x^8+x^2+x+1 over words 0..6 and the CRC byte: remainder 0
    for (std::size_t i = 0; i < 15; ++i) {
        crc ^= byte_of(i);
        for (int b = 0; b < 8; ++b) crc = static_cast<std::uint8_t>((crc & 0x80) != 0 ? (crc << 1) ^ 0x07 : crc << 1);
    }
    EXPECT_EQ(crc, 0);
}

TEST(EthercatConfig, IdentityMailboxAndName) {
    EXPECT_EQ(image[0x0A] | (image[0x0B] << 16), rm::ethercat::product_code);
    EXPECT_EQ(image[0x0C], rm::ethercat::revision);
    EXPECT_EQ(image[0x0E] | (static_cast<std::uint32_t>(image[0x0F]) << 16), 0xCAFE'F00Du);
    EXPECT_EQ(image[0x18], ecat::layout.mailbox_out_start);
    EXPECT_EQ(image[0x19], ecat::layout.mailbox_out_size);
    EXPECT_EQ(image[0x1C], ec::sii::mailbox_coe);
    ASSERT_EQ(image[0x40], 10);  // STRINGS
    const std::size_t len = byte_of(2 * 0x42 + 1);
    std::string name;
    for (std::size_t i = 0; i < len; ++i) name += static_cast<char>(byte_of(2 * 0x42 + 2 + i));
    EXPECT_EQ(name, "Factory Control Unit");
}

TEST(EthercatConfig, ProcessDataBehindTheMailboxAndInTheRam) {
    const auto& l = ecat::layout;
    EXPECT_GE(l.outputs_start, l.mailbox_out_start + l.mailbox_out_size);
    EXPECT_GE(l.outputs_start, l.mailbox_in_start + l.mailbox_in_size);
    EXPECT_GE(l.inputs_start, l.outputs_start + 3 * rm::ethercat::default_outputs_bytes);  // three buffers
    EXPECT_LE(l.inputs_start + 3 * rm::ethercat::default_inputs_bytes, 0x3000);
}

// The dictionary over the real register bank: CoE reads and writes the same registers as Modbus.
TEST(EthercatConfig, DictionaryOverTheRegisterMap) {
    rtos::modbus::RegisterBank<rm::holding_count, rm::input_count> bank;
    ec::CoeServer<> coe(Od::objects, bank);
    EXPECT_EQ(coe.process_data_bytes(false), rm::ethercat::default_outputs_bytes);
    EXPECT_EQ(coe.process_data_bytes(true), rm::ethercat::default_inputs_bytes);
    std::vector<std::byte> out(122);
    const auto call = [&](std::vector<std::byte> in) {
        const std::size_t n = coe.request(in, out, 2);
        return std::vector<std::byte>(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(n));
    };
    const auto upload = [](std::uint16_t index, std::uint8_t sub) {
        return std::vector<std::byte>{std::byte{0x00}, std::byte{0x20}, std::byte{0x40}, std::byte(index & 0xFF), std::byte(index >> 8),
                                      std::byte{sub}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{0}};
    };
    // Ethernet inputs (6000h + the region's first address 80): link status
    rm::set_eth_link_status(bank, 1);
    auto r = call(upload(0x6000 + rm::input::eth_link_status, 1));
    ASSERT_EQ(r.size(), 10u);
    EXPECT_EQ(std::to_integer<int>(r[6]), 1);
    // the hardware revision string (1009h)
    r = call(upload(0x1009, 0));
    EXPECT_EQ(std::to_integer<char>(r[6]), '3');
    // a download into a WS2812 pattern (7000h + 100): through the bank
    const auto pattern = static_cast<std::uint16_t>(0x7000 + rm::holding::ws2812_ch1_pattern);
    r = call({std::byte{0x00}, std::byte{0x20}, std::byte{0x2B}, std::byte(pattern & 0xFF), std::byte(pattern >> 8), std::byte{1},
              std::byte{3}, std::byte{0}, std::byte{0}, std::byte{0}});
    EXPECT_EQ(std::to_integer<int>(r[2]), 0x60);
    EXPECT_EQ(rm::ws2812_ch1_pattern(bank), 3);
}

}  // namespace
