// The emulated EEPROM content of the LAN9253 bring-up test (ethercat_test.cpp): the slave information
// interface (SII, ETG.2010) as 128 words. Pure and constexpr, so the host tests check it
// (tests/host/ethercat_sii_test.cpp).
//
// The ESC configuration area (words 0..7, LAN9253 DS00003421 table 11-4) is loaded into the ESC by
// the reload after reset; it must carry the PDI type the EE_EMUL straps already selected (13.4: the
// PDI type must not change). It is therefore derived from the register map found on the SPI, never
// read back from PDI control 0x0140: before the configuration is loaded (READY clear) that register
// reads 0x00 - "no PDI" - and loading it switches the PDI off, the load fails and the PHYs stay dark.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace fcu::ethercat::sii {

using Image = std::array<std::uint16_t, 128>;  ///< 2 Kbit

/// PDI types of SPI (DS00003421 table 13-3): EE_EMUL 000 and 100
inline constexpr std::uint8_t pdi_spi = 0x80, pdi_spi_direct_mapped = 0x82;

// Test identity: no ETG vendor ID of our own yet - vendor 0 marks it as a lab device.
inline constexpr std::uint32_t vendor_id = 0x0000'0000, product_code = 0x0000'9253, revision_no = 1;
inline constexpr char device_name[] = "FCU EtherCAT test";

/// CRC-8 of the ESC configuration area (words 0..6) for word 7: polynomial x^8+x^2+x+1, initial FFh.
constexpr std::uint8_t config_crc(const Image& w) noexcept {
    std::uint8_t crc = 0xFF;
    for (int i = 0; i < 14; ++i) {
        crc ^= static_cast<std::uint8_t>(w[static_cast<std::size_t>(i / 2)] >> (8 * (i % 2)));
        for (int b = 0; b < 8; ++b) crc = static_cast<std::uint8_t>((crc & 0x80) != 0 ? (crc << 1) ^ 0x07 : crc << 1);
    }
    return crc;
}

/// The image: configuration area (PDI type from the straps, everything else at the defaults of
/// table 13-4, ESC configuration 0: the PDI acknowledges AL states), identity, a name and a
/// general category.
constexpr Image make(bool direct_mapped) noexcept {
    Image sii{};
    sii[0] = direct_mapped ? pdi_spi_direct_mapped : pdi_spi;  // PDI control 0x0140; ESC configuration 0x0141: 0
    // words 1..6: PDI/SYNC configuration, SYNC pulse length, extended PDI configuration, station
    // alias, ASIC configuration, reserved - all 0 (the SPI defaults of table 13-4)
    sii[7] = config_crc(sii);
    sii[0x08] = static_cast<std::uint16_t>(vendor_id), sii[0x09] = static_cast<std::uint16_t>(vendor_id >> 16);
    sii[0x0A] = static_cast<std::uint16_t>(product_code), sii[0x0B] = static_cast<std::uint16_t>(product_code >> 16);
    sii[0x0C] = static_cast<std::uint16_t>(revision_no), sii[0x0D] = static_cast<std::uint16_t>(revision_no >> 16);
    sii[0x3E] = 1;  // size: (2 Kbit / 1 Kbit) - 1
    sii[0x3F] = 1;  // SII version
    // category STRINGS (10): one string
    std::size_t w = 0x40;
    constexpr std::size_t name_len = sizeof device_name - 1;
    constexpr std::size_t string_words = (2 + name_len + 1) / 2;  // count, length, characters
    sii[w++] = 10;
    sii[w++] = static_cast<std::uint16_t>(string_words);
    std::array<std::uint8_t, 2 * string_words> bytes{};
    bytes[0] = 1;
    bytes[1] = static_cast<std::uint8_t>(name_len);
    for (std::size_t i = 0; i < name_len; ++i) bytes[2 + i] = static_cast<std::uint8_t>(device_name[i]);
    for (std::size_t i = 0; i < string_words; ++i) sii[w++] = static_cast<std::uint16_t>(bytes[2 * i] | (bytes[2 * i + 1] << 8));
    // category GENERAL (30): group, image, order and name: string 1
    sii[w++] = 30;
    sii[w++] = 16;
    sii[w] = 0x0001;      // group index 1, image index 0
    sii[w + 1] = 0x0101;  // order index 1, name index 1
    w += 16;
    sii[w] = 0xFFFF;  // end
    return sii;
}

}  // namespace fcu::ethercat::sii
