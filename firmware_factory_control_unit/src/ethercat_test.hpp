// Rudimentary bring-up code for the LAN9253 EtherCAT slave controller (ESC) - a hardware test, not
// an EtherCAT stack: a master (TwinCAT, SOEM, ...) shall find the device, read its identity and
// take it through INIT -> PRE-OP -> SAFE-OP -> OP. No mailbox (no CoE), no process data.
//
// - SPI by GPIO on the QUADSPI/OCTOSPI lines (READ 03h, WRITE 02h; LAN9253 DS00003421 9.2),
//   EtherCAT core registers through ECAT_CSR_DATA/ECAT_CSR_CMD (11.13.1)
// - at the start: reset released, BYTE_TEST/ID_REV/HW_CFG and the ESC's type and settings logged
// - every I/O cycle: the EEPROM commands of the ESC answered from a small SII image in RAM (the
//   board has no EEPROM: EEPROM emulation, 12.2), the AL state requests of the master acknowledged,
//   link changes per port logged
// Everything goes to the log (terminal page, serial console). Revision 1 and 3; nothing on 2.
#pragma once

#include <cstdint>

namespace fcu::ethercat {

/// Releases the ESC from reset and probes it (I/O thread, after the pins are configured).
void setup() noexcept;
/// One I/O cycle: EEPROM emulation, AL state machine, link status.
void cycle(std::uint32_t now) noexcept;

}  // namespace fcu::ethercat
