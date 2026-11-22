// The FCU's EtherCAT identity and SII image (src/ethercat_config.hpp).
//
// Regression: the configuration area took the PDI type from register 0x0140, read before READY - it
// reads 0x00 then. The reload loaded "no PDI", the configuration failed (DL status: PDI not
// operational, HW_CFG never READY) and both PHYs stayed without link. Now the PDI type is a constant.
#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "ethercat_config.hpp"

namespace {

namespace ecat = fcu::ethercat;
constexpr auto image = ecat::sii_image(3, 0xCAFE'F00D);

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

TEST(EthercatConfig, IdentityAndName) {
    EXPECT_EQ(image[0x0A] | (image[0x0B] << 16), ecat::product_code);
    EXPECT_EQ(image[0x0C], 3);  // the hardware revision
    EXPECT_EQ(image[0x0E] | (static_cast<std::uint32_t>(image[0x0F]) << 16), 0xCAFE'F00Du);
    ASSERT_EQ(image[0x40], 10);  // STRINGS
    const std::size_t len = byte_of(2 * 0x42 + 1);
    std::string name;
    for (std::size_t i = 0; i < len; ++i) name += static_cast<char>(byte_of(2 * 0x42 + 2 + i));
    EXPECT_EQ(name, "Factory Control Unit");
}

TEST(EthercatConfig, ProcessDataBehindTheMailbox) {
    const auto& l = ecat::layout;
    EXPECT_GE(l.outputs_start, l.mailbox_out_start + l.mailbox_out_size);
    EXPECT_GE(l.outputs_start, l.mailbox_in_start + l.mailbox_in_size);
    EXPECT_GE(l.inputs_start, l.outputs_start);
    EXPECT_LT(l.inputs_start, 0x3000);
}

}  // namespace
