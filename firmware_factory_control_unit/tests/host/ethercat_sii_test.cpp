// The emulated EEPROM of the LAN9253 bring-up test (src/ethercat_sii.hpp).
//
// Regression: the configuration area took the PDI type from register 0x0140, read before READY - it
// reads 0x00 then. The reload loaded "no PDI", the configuration failed (DL status: PDI not
// operational, HW_CFG never READY) and both PHYs stayed without link.
#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "ethercat_sii.hpp"

namespace sii = fcu::ethercat::sii;

namespace {

/// CRC-8, x^8+x^2+x+1, initial FFh, not reflected: over data and its CRC the remainder is 0.
std::uint8_t crc8(const std::uint8_t* data, int n) {
    std::uint8_t crc = 0xFF;
    for (int i = 0; i < n; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) crc = static_cast<std::uint8_t>((crc & 0x80) != 0 ? (crc << 1) ^ 0x07 : crc << 1);
    }
    return crc;
}

std::uint8_t byte(const sii::Image& image, int i) { return static_cast<std::uint8_t>(image[static_cast<std::size_t>(i / 2)] >> (8 * (i % 2))); }

}  // namespace

TEST(EthercatSii, ConfigurationAreaKeepsTheStrappedPdiType) {
    // DS00003421 13.4: the reload must not change the PDI type the EE_EMUL straps selected
    EXPECT_EQ(sii::make(true)[0] & 0xFF, 0x82) << "SPI EtherCAT Direct Mapped (EE_EMUL 100)";
    EXPECT_EQ(sii::make(false)[0] & 0xFF, 0x80) << "SPI, LAN9252 compatible (EE_EMUL 000)";
    EXPECT_EQ(sii::make(true)[0] >> 8, 0) << "ESC configuration: the PDI acknowledges the AL states";
}

TEST(EthercatSii, ConfigurationAreaChecksumIsValid) {
    for (const bool direct_mapped : {true, false}) {
        const auto image = sii::make(direct_mapped);
        std::uint8_t area[15];
        for (int i = 0; i < 15; ++i) area[i] = byte(image, i);  // words 0..6 and the CRC (low byte of word 7)
        EXPECT_EQ(crc8(area, 15), 0) << "direct mapped " << direct_mapped;
        EXPECT_EQ(image[7] >> 8, 0);
    }
}

TEST(EthercatSii, IdentityNameAndEnd) {
    const auto image = sii::make(true);
    EXPECT_EQ(image[0x0A], 0x9253);  // product code
    EXPECT_EQ(image[0x3E], 1);       // 2 Kbit
    ASSERT_EQ(image[0x40], 10);      // STRINGS
    const int len = byte(image, 2 * 0x42 + 1);
    std::string name;
    for (int i = 0; i < len; ++i) name += static_cast<char>(byte(image, 2 * 0x42 + 2 + i));
    EXPECT_EQ(name, sii::device_name);
    const std::size_t general = 0x42 + image[0x41];
    EXPECT_EQ(image[general], 30);  // GENERAL
    EXPECT_EQ(image[general + 2 + image[general + 1]], 0xFFFF);  // end
}
