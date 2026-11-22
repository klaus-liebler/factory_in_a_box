// The EtherCAT slave (LAN9253, revisions 1 and 3): its own thread drives rtos::ethercat::Slave over
// rtos::devices::Lan9253 - EEPROM emulation (SII from ethercat_config.hpp), the AL state machine with
// its checks, the mailbox with CoE (SDO and SDO information over the register map's object dictionary:
// the same registers as Modbus and OPC UA), and once a second the ESC's diagnostics for the web UI and
// the register map.
//
// SPI by GPIO on the QUADSPI/OCTOSPI lines (rtoos has no driver for either peripheral yet; the same
// pins take it later). The thread wakes on the ESC's IRQ where the line is wired (revision 3: EXTI),
// else it polls every few milliseconds - every event is in the AL event request register.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "rtos/devices/lan9253.hpp"
#include "rtos/ethercat/slave.hpp"

namespace fcu::ethercat {

/// Starts the EtherCAT thread (after io::setup: the pins are configured, the ESC is held in reset).
/// Nothing on a revision without the ESC.
void start() noexcept;

/// The ESC's IRQ line went low (EXTI, interrupt context): wakes the thread.
void irq_from_isr() noexcept;

/// The last snapshot (taken once a second by the EtherCAT thread).
struct Status {
    rtos::devices::Lan9253Mode mode = rtos::devices::Lan9253Mode::none;
    bool ready = false, irq = false;
    std::uint32_t id_rev = 0, hw_cfg = 0;
    std::uint32_t taken_ms = 0;  ///< kernel time of the snapshot
    rtos::ethercat::Diagnostics diagnostics{};
    rtos::ethercat::SlaveCounters counters{};
    rtos::ethercat::CoeCounters coe{};
    std::uint16_t outputs_bytes = 0, inputs_bytes = 0;  ///< the assigned PDOs
    std::array<rtos::devices::Lan9253Phy, 2> phys{};
    std::size_t phy_count = 0;
};
[[nodiscard]] Status status() noexcept;

}  // namespace fcu::ethercat
