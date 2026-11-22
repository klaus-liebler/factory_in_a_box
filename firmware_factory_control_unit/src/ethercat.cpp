// The EtherCAT slave (see ethercat.hpp).
#include "ethercat.hpp"

#include <algorithm>
#include <array>
#include <span>

#include "device_ids.hh"
#include "ethercat_config.hpp"
#include "ethercat_registers.hpp"
#include "hardware.hpp"
#include "pins.hpp"
#include "system.hpp"
#include "task_monitor.hpp"

namespace fcu::ethercat {
namespace {

namespace ec = rtos::ethercat;
namespace rm = register_map;
using pins::gpio::read;
using pins::gpio::write;
using rtos::devices::Lan9253Mode;

// ---- SPI by GPIO, mode 0 (the ESC samples SI on the rising edge, drives SO on the falling) ----------
// About 5 MHz: far below the 30 MHz of READ.
struct GpioSpi {
    static void pause() noexcept {
        for (int i = 0; i < 6; ++i) __asm volatile("nop");
    }
    static void select(bool on) noexcept {
        write(pins::ethercat->cs, !on);
        pause();
    }
    static std::uint8_t transfer(std::uint8_t out) noexcept {
        const auto& p = *pins::ethercat;
        std::uint8_t in = 0;
        for (int bit = 7; bit >= 0; --bit) {
            write(p.sio0, ((out >> bit) & 1u) != 0);
            pause();
            write(p.sck, true);
            in = static_cast<std::uint8_t>((in << 1) | (read(p.sio1) ? 1u : 0u));
            pause();
            write(p.sck, false);
        }
        return in;
    }
    /// At least `ns` nanoseconds (DWT cycle counter).
    static void wait_ns(std::uint32_t ns) noexcept {
        const auto cycles = static_cast<std::uint32_t>(std::uint64_t{ns} * board::sysclk_hz / 1'000'000'000u) + 1;
        const std::uint32_t start = board::CycleCounter::now();
        while (board::CycleCounter::now() - start < cycles) {
        }
    }
};

using Esc = rtos::devices::Lan9253<GpioSpi>;
Esc esc;

/// RUN LED off by override (0138h): with the RUNLED/EE_EMUL0 strap at 0 the pin is an active-high
/// open-source output; on revision 1 it is wired straight to GND for the strap, and "off" keeps it
/// from driving high into that wire. Every AL status write ends the override: set again after each.
constexpr std::uint8_t run_led_off = 0x10;

// ---- the application side of the slave ----------------------------------------------------------
struct App {
    std::uint32_t eeprom_logged = 0;

    void al_changed(std::uint8_t from, std::uint8_t to) noexcept {
        ec::write8(esc, ec::reg::run_led_override, run_led_off);
        RTOS_LOG_INFO("ethercat: AL %.*s -> %.*s", static_cast<int>(ec::state_name(from).size()), ec::state_name(from).data(),
                      static_cast<int>(ec::state_name(to).size()), ec::state_name(to).data());
    }
    void al_refused(std::uint8_t from, std::uint8_t to, std::uint16_t code) noexcept {
        ec::write8(esc, ec::reg::run_led_override, run_led_off);
        const auto text = ec::al_code::text(code);
        RTOS_LOG_WARN("ethercat: AL %.*s -> %.*s refused: 0x%04x %.*s", static_cast<int>(ec::state_name(from).size()),
                      ec::state_name(from).data(), static_cast<int>(ec::state_name(to).size()), ec::state_name(to).data(), code,
                      static_cast<int>(text.size()), text.data());
    }
    void eeprom_command(unsigned cmd, std::uint32_t word) noexcept {
        if (cmd == ec::eeprom::cmd_read && eeprom_logged >= 8) return;  // a master reads the SII in bursts
        ++eeprom_logged;
        RTOS_LOG_INFO("ethercat: EEPROM %s word 0x%04lx",
                      cmd == ec::eeprom::cmd_read ? "read" : cmd == ec::eeprom::cmd_write ? "write" : "reload",
                      static_cast<unsigned long>(word));
    }
    static std::uint16_t outputs_size() noexcept { return 0; }
    static std::uint16_t inputs_size() noexcept { return 0; }
};
App app;

constexpr std::uint32_t serial = DEVICE_CHIP_UID_W0 ^ DEVICE_CHIP_UID_W1 ^ DEVICE_CHIP_UID_W2;
constinit const auto sii = sii_image(board::revision, serial);
ec::Slave<Esc, App, sii_words> slave{esc, sii, layout, app};

// ---- register access over EtherCAT (ethercat_registers.hpp, until CoE replaces it) ---------------------
namespace ra = register_access;
std::uint16_t last_sequence = 0;
std::uint32_t register_requests = 0;

/// One request: a new sequence number at 0x1000 triggers it; the response is written with its
/// sequence number last.
void serve_registers() noexcept {
    const std::uint16_t sequence = ec::read16(esc, ra::request_address);
    if (sequence == 0 || sequence == last_sequence) return;
    ra::Area request{};
    esc.read(ra::request_address, std::as_writable_bytes(std::span(request).first(ra::header_bytes)));
    const std::size_t values = ra::request_value_bytes(request);
    esc.read(ra::request_address + ra::header_bytes, std::as_writable_bytes(std::span(request).subspan(ra::header_bytes, values)));
    ra::Area response{};
    ra::execute(request, ::registers, response);
    esc.write(ra::response_address + 2, std::as_bytes(std::span<const std::uint8_t>(response).subspan(2, ra::response_bytes(response) - 2)));
    ec::write16(esc, ra::response_address, sequence);
    last_sequence = sequence;
    if (register_requests++ == 0) {
        RTOS_LOG_INFO("ethercat: first register request - function 0x%02x, start %u, count %u, exception %u", response[2],
                      ra::get16(response, 4), ra::get16(response, 6), response[3]);
    }
}

// ---- status -------------------------------------------------------------------------------------------
Status snapshot{};
std::uint16_t logged_dl = 0xFFFF;
std::array<bool, 2> logged_link{};

std::uint16_t saturate(unsigned v) noexcept { return static_cast<std::uint16_t>(std::min(v, 0xFFFFu)); }

/// The ESC's registers, the PHYs and the counters into the snapshot and the register map; link
/// changes into the log.
void take_status(bool ready) noexcept {
    Status s{};
    s.mode = esc.mode(), s.ready = ready, s.irq = hw::has_ethercat_irq;
    s.id_rev = esc.csr(rtos::devices::lan9253::id_rev), s.hw_cfg = esc.csr(rtos::devices::lan9253::hw_cfg);
    s.taken_ms = kernel.now();
    if (ready) {
        s.diagnostics = slave.read_diagnostics();
        s.phy_count = esc.phy_states(s.phys);
    }
    s.counters = slave.counters();
    {
        [[maybe_unused]] const Kernel::Lock lock;
        snapshot = s;
    }
    const auto& d = s.diagnostics;
    rm::set_ecat_al_status(registers, d.al_status);
    rm::set_ecat_al_status_code(registers, d.al_status_code);
    rm::set_ecat_dl_status(registers, d.dl_status);
    rm::set_ecat_station_address(registers, d.station_address);
    unsigned errors = 0, lost = 0;
    for (const auto& p : d.ports) errors += p.invalid_frames + p.rx_errors + p.forwarded_errors, lost += p.lost_links;
    rm::set_ecat_rx_errors(registers, saturate(errors));
    rm::set_ecat_lost_links(registers, saturate(lost));
    if (d.dl_status != logged_dl) {
        logged_dl = d.dl_status;
        RTOS_LOG_INFO("ethercat: DL status 0x%04x - PDI %s, port 0 link %s%s, port 1 link %s%s", d.dl_status,
                      (d.dl_status & ec::dl_status::pdi_operational) != 0 ? "ok" : "not operational",
                      ec::dl_status::link(d.dl_status, 0) ? "up" : "down", ec::dl_status::communication(d.dl_status, 0) ? " (communication)" : "",
                      ec::dl_status::link(d.dl_status, 1) ? "up" : "down", ec::dl_status::communication(d.dl_status, 1) ? " (communication)" : "");
    }
    for (std::size_t i = 0; i < s.phy_count && i < logged_link.size(); ++i) {
        if (s.phys[i].link != logged_link[i]) {
            logged_link[i] = s.phys[i].link;
            RTOS_LOG_INFO("ethercat: PHY %u link %s%s", s.phys[i].address, s.phys[i].link ? "up" : "down",
                          s.phys[i].energy ? "" : " (no energy on the cable)");
        }
    }
}

/// Once READY (the configuration area is loaded): the ESC's identity into the log, the PHYs set up,
/// the IRQ on. Reads of these registers before READY are forbidden (DS00003421 HW_CFG), writes lost.
void on_ready() noexcept {
    const std::size_t phys = esc.configure_phys();
    const ec::Diagnostics d = slave.read_diagnostics();
    RTOS_LOG_INFO("ethercat: ready - ESC type 0x%02x rev %u build 0x%04x, %u FMMUs, %u SyncManagers, %u KB RAM, "
                  "PDI 0x%02x, %u PHYs, %s",
                  d.esc_type, d.esc_revision, d.esc_build, d.fmmus, d.sync_managers, d.ram_kb, d.pdi_control,
                  static_cast<unsigned>(phys), hw::has_ethercat_irq ? "IRQ" : "polled");
    ec::write8(esc, ec::reg::run_led_override, run_led_off);
    if constexpr (hw::has_ethercat_irq) {
        esc.enable_irq(ec::al_event::al_control | ec::al_event::eeprom | ec::al_event::watchdog | ec::al_event::sm(0) |
                       ec::al_event::sm(1) | ec::al_event::sm(2) | ec::al_event::sm(3));
    }
    last_sequence = ec::read16(esc, register_access::request_address);  // no stale request
}

// ---- the thread ------------------------------------------------------------------------------------------
constexpr std::size_t notify_slot = 0;
Kernel::Thread ecat_tcb;
alignas(8) std::array<std::byte, 4 * 1024> ecat_stack;

/// Polling period without the IRQ line, and the fallback with it (a level the edge did not show).
constexpr rtos::Tick poll_ms = 2, irq_fallback_ms = 20, not_ready_ms = 1;

void ecat_main(void*) {
    write(pins::ethercat_reset, false);
    kernel.sleep(2);
    write(pins::ethercat_reset, true);  // out of reset
    // the SPI answers once the device is up: the first reads may not
    for (int i = 0; i < 100 && esc.probe() == Lan9253Mode::none; ++i) kernel.sleep(2);
    if (esc.mode() != Lan9253Mode::direct_mapped) {
        RTOS_LOG_ERROR("ethercat: %s", esc.mode() == Lan9253Mode::compatible
                                           ? "LAN9253 strapped LAN9252-compatible (PDI 0x80) - only EtherCAT Direct Mapped (0x82, EE_EMUL 100) is supported"
                                           : "no answer on SPI (BYTE_TEST) - straps, wiring, or the LAN9253 has no clock/supply");
        for (;;) {
            take_status(false);
            kernel.sleep(1000);
        }
    }
    RTOS_LOG_INFO("ethercat: LAN9253 on SPI, Direct Mapped - ID_REV 0x%08lx, HW_CFG 0x%08lx",
                  static_cast<unsigned long>(esc.csr(rtos::devices::lan9253::id_rev)),
                  static_cast<unsigned long>(esc.csr(rtos::devices::lan9253::hw_cfg)));
    bool ready = false, late_logged = false;
    const rtos::Tick since = kernel.now();
    rtos::Tick next_status = since;
    for (;;) {
        if (!ready && esc.ready()) {
            ready = true;
            GpioSpi::wait_ns(1500);  // 13.4: the new configuration takes effect 1.5 us after the last emulation write
            on_ready();
        }
        slave.poll(ready);
        if (ready) serve_registers();
        const rtos::Tick now = kernel.now();
        if (!ready && !late_logged && now - since > 2000) {
            late_logged = true;
            RTOS_LOG_WARN("ethercat: still not ready after 2 s - HW_CFG 0x%08lx, %lu EEPROM reloads served",
                          static_cast<unsigned long>(esc.csr(rtos::devices::lan9253::hw_cfg)),
                          static_cast<unsigned long>(slave.counters().eeprom_reloads));
        }
        if (static_cast<std::int32_t>(now - next_status) >= 0) {
            next_status = now + 1000;
            take_status(ready);
        }
        (void)kernel.notify_wait(notify_slot, !ready ? not_ready_ms : hw::has_ethercat_irq ? irq_fallback_ms : poll_ms);
    }
}

}  // namespace

void start() noexcept {
    if constexpr (!pins::ethercat.has_value()) {
        return;
    } else {
        std::ranges::fill(ecat_stack, task_monitor::paint);
        kernel.create_thread(ecat_tcb, {.name = "ecat", .entry = &ecat_main, .stack = ecat_stack, .priority = ethercat_priority});
        task_monitor::add(ecat_tcb, ecat_stack);
        if constexpr (hw::has_ethercat_irq) {
            hw::EscIrq::init();
            hw::EscIrq::enable();
            enable_irq(hw::EscIrq::irq, irq_priority_net);  // kernel-aware: wakes the thread
        }
    }
}

void irq_from_isr() noexcept { kernel.notify_from_isr(ecat_tcb, notify_slot, 1, rtos::NotifyAction::SetBits); }

Status status() noexcept {
    [[maybe_unused]] const Kernel::Lock lock;
    return snapshot;
}

}  // namespace fcu::ethercat
