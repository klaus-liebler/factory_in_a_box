// The FCU's EtherCAT identity and memory layout (src/ethercat.cpp): the SII image the slave answers
// the master's EEPROM reads with, and where the SyncManagers sit in the LAN9253's process RAM.
// Pure and constexpr: tests/host/ethercat_config_test.cpp.
//
// The configuration area carries the PDI type the EE_EMUL straps select - SPI EtherCAT Direct
// Mapped (0x82), the only mode the driver runs (rtos::devices::Lan9253). It is a constant on
// purpose: the ESC's PDI control register 0x0140 reads 0x00 ("no PDI") before the configuration is
// loaded, and a reload of 0x00 switched the PDI off - the load failed and the PHYs stayed dark
// (LAN9253 DS00003421 13.4: the PDI type must not change).
#pragma once

#include <cstdint>

#include "rtos/ethercat/sii.hpp"
#include "rtos/ethercat/slave.hpp"

namespace fcu::ethercat {

namespace ec = rtos::ethercat;

inline constexpr std::uint8_t pdi_spi_direct_mapped = 0x82;

/// No ETG vendor ID of our own yet: vendor 0 marks a lab device. The product code says "FCU", the
/// revision the hardware revision.
inline constexpr std::uint32_t vendor_id = 0x0000'0000, product_code = 0x0046'4355;  // "FCU"

/// Process RAM (1000h-2FFFh): mailbox out/in, then the process data.
inline constexpr ec::SlaveLayout layout{.outputs_start = 0x1100, .inputs_start = 0x1800};

inline constexpr std::size_t sii_words = 256;  ///< 4 Kbit

[[nodiscard]] constexpr ec::SiiConfig sii_config(unsigned hardware_revision, std::uint32_t serial) noexcept {
    return {.pdi_control = pdi_spi_direct_mapped,
            .esc_config = 0,  // the PDI acknowledges the AL states
            .vendor_id = vendor_id,
            .product_code = product_code,
            .revision = hardware_revision,
            .serial = serial,
            .mailbox_out_start = layout.mailbox_out_start,
            .mailbox_out_size = layout.mailbox_out_size,
            .mailbox_in_start = layout.mailbox_in_start,
            .mailbox_in_size = layout.mailbox_in_size,
            .name = "Factory Control Unit",
            .group = "FactoryInABox",
            .fmmus = {ec::sii::fmmu_outputs, ec::sii::fmmu_inputs, ec::sii::fmmu_unused},
            .sync_managers = {}};
}

[[nodiscard]] constexpr auto sii_image(unsigned hardware_revision, std::uint32_t serial) noexcept {
    return ec::make_sii<sii_words>(sii_config(hardware_revision, serial));
}

}  // namespace fcu::ethercat
