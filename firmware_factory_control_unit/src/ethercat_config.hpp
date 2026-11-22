// The FCU's EtherCAT configuration (src/ethercat.cpp): the SII image the slave answers the master's
// EEPROM reads with, and the CoE object dictionary - the rtoos standard objects (identity, PDO
// assignment, ...) joined with the register map's objects (generated register_map_ethercat.hpp,
// which also holds the identity and the process RAM layout - the ESI is generated from the same
// options, so SII, dictionary and ESI agree). Pure and constexpr: tests/host/ethercat_config_test.cpp.
//
// The configuration area carries the PDI type the EE_EMUL straps select - SPI EtherCAT Direct
// Mapped (0x82), the only mode the driver runs (rtos::devices::Lan9253). It is a constant on
// purpose: the ESC's PDI control register 0x0140 reads 0x00 ("no PDI") before the configuration is
// loaded, and a reload of 0x00 switched the PDI off - the load failed and the PHYs stayed dark
// (LAN9253 DS00003421 13.4: the PDI type must not change).
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "register_map_ethercat.hpp"
#include "rtos/ethercat/object_dictionary.hpp"
#include "rtos/ethercat/sii.hpp"
#include "rtos/ethercat/slave.hpp"

namespace fcu::ethercat {

namespace ec = rtos::ethercat;
namespace rme = register_map::ethercat;

inline constexpr std::uint8_t pdi_spi_direct_mapped = 0x82;
inline constexpr ec::SlaveLayout layout = rme::layout;
inline constexpr std::size_t sii_words = 256;  ///< 4 Kbit

[[nodiscard]] constexpr ec::SiiConfig sii_config(std::uint32_t serial) noexcept {
    namespace sii = ec::sii;
    return {.pdi_control = pdi_spi_direct_mapped,
            .esc_config = 0,  // the PDI acknowledges the AL states
            .vendor_id = rme::vendor_id,
            .product_code = rme::product_code,
            .revision = rme::revision,
            .serial = serial,
            .mailbox_out_start = layout.mailbox_out_start,
            .mailbox_out_size = layout.mailbox_out_size,
            .mailbox_in_start = layout.mailbox_in_start,
            .mailbox_in_size = layout.mailbox_in_size,
            .mailbox_protocols = sii::mailbox_coe,
            .coe_details = sii::coe_sdo | sii::coe_sdo_info | sii::coe_pdo_assign,
            .name = rme::device_name,
            .group = rme::group_name,
            .fmmus = {sii::fmmu_outputs, sii::fmmu_inputs, sii::fmmu_sm_status},
            .sync_managers = {{
                {.start = layout.mailbox_out_start, .length = layout.mailbox_out_size, .control = 0x26, .enable = true, .type = sii::sm_mailbox_out},
                {.start = layout.mailbox_in_start, .length = layout.mailbox_in_size, .control = 0x22, .enable = true, .type = sii::sm_mailbox_in},
                {.start = layout.outputs_start, .length = rme::default_outputs_bytes, .control = 0x64, .enable = true, .type = sii::sm_outputs},
                {.start = layout.inputs_start, .length = rme::default_inputs_bytes, .control = 0x20, .enable = true, .type = sii::sm_inputs},
            }}};
}

[[nodiscard]] constexpr auto sii_image(std::uint32_t serial) noexcept { return ec::make_sii<sii_words>(sii_config(serial)); }

/// The object dictionary of a device: hardware revision (1009h), firmware version (100Ah: *Software),
/// serial number (1018h:04).
template <unsigned HardwareRevision, std::uint32_t Serial, const std::string_view* Software>
struct Dictionary {
    static constexpr std::array<char, 1> hardware{static_cast<char>('0' + HardwareRevision)};
    static constexpr ec::StandardEntries standard = ec::standard_entries({.device_type = 0,
                                                                         .name = rme::device_name,
                                                                         .hardware_version = {hardware.data(), hardware.size()},
                                                                         .software_version = *Software,
                                                                         .vendor_id = rme::vendor_id,
                                                                         .product_code = rme::product_code,
                                                                         .revision = rme::revision,
                                                                         .serial = Serial});
    static constexpr auto objects = ec::join_objects(ec::standard_objects(standard), rme::objects);
    static_assert(ec::valid_dictionary(objects), "the object dictionary: indexes unique, entries sorted, counts right");
};

}  // namespace fcu::ethercat
