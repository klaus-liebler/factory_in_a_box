// LAN9253 bring-up test (see ethercat_test.hpp). Register addresses and bits: LAN9253 data sheet
// DS00003421B (system CSRs: table 5-1; EtherCAT core CSRs: 11.16; EEPROM emulation: 12.2, 13.4;
// SPI EtherCAT Direct Mapped mode: 5.0, 9.3) and the EtherCAT slave information (ETG.2010).
//
// Two register maps, chosen by the EE_EMUL[2:0] straps (EESCL, EESDA, RUNLED; table 13-3):
//   000 -> PDI 0x80, LAN9252 compatible: system CSRs at 0x050.., EtherCAT core registers through
//          ECAT_CSR_CMD/ECAT_CSR_DATA
//   100 -> PDI 0x82, EtherCAT Direct Mapped (revision 3, and revision 1 with EESDA and RUNLED
//          wired to GND): the core registers at their own addresses 0x0000.., the system CSRs at
//          0x3000 + offset. Without WAIT_ACK the host keeps the worst-case timing (9.3.2): 370 ns
//          from the address to each data byte of a READ, 250 ns + 80 ns per byte after a core
//          write before the next command.
// setup() finds the map by BYTE_TEST (87654321h at 0x064 or at 0x3064).
#include "ethercat_test.hpp"

#include <algorithm>
#include <array>
#include <span>

#include "ethercat_registers.hpp"
#include "ethercat_sii.hpp"
#include "hardware.hpp"
#include "pins.hpp"
#include "system.hpp"

namespace fcu::ethercat {
namespace {

using pins::gpio::read;
using pins::gpio::write;

// ---- SPI by GPIO, mode 0 (the ESC samples SI on the rising edge, drives SO on the falling) ------
// About 5 MHz: far below the 30 MHz of READ, no timing questions for a first test.
inline void pause() noexcept {
    for (int i = 0; i < 6; ++i) __asm volatile("nop");
}

std::uint8_t transfer(std::uint8_t out) noexcept {
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
void wait_ns(std::uint32_t ns) noexcept {
    const auto cycles = static_cast<std::uint32_t>(std::uint64_t{ns} * board::sysclk_hz / 1'000'000'000u) + 1;
    const std::uint32_t start = board::CycleCounter::now();
    while (board::CycleCounter::now() - start < cycles) {
    }
}

void select(bool on) noexcept {
    write(pins::ethercat->cs, !on);
    pause();
}

// System CSRs (byte address, 14 bits; bits 15:14 = 00: the bytes of the DWORD in order); in
// Direct Mapped mode at 0x3000 + these offsets
constexpr std::uint16_t id_rev = 0x050, byte_test = 0x064, hw_cfg = 0x074, ecat_csr_data = 0x300, ecat_csr_cmd = 0x304;
constexpr std::uint16_t direct_system_base = 0x3000;
constexpr std::uint32_t byte_test_value = 0x8765'4321, hw_cfg_ready = 1u << 27;
constexpr std::uint16_t increment = 0x4000;  ///< address bits 15:14 = 01: auto-increment (9.2.5, 9.3.2)

bool direct_mapped = false;  ///< PDI 0x82: see the top of the file
std::uint16_t system_base = 0;

std::uint32_t read32(std::uint16_t offset) noexcept {
    const auto address = static_cast<std::uint16_t>(system_base + offset);
    select(true);
    transfer(0x03);  // READ
    transfer(static_cast<std::uint8_t>(address >> 8));
    transfer(static_cast<std::uint8_t>(address));
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
        // SI low for all but the last byte, high (FFh) on the last: no prefetch past it (9.2.5.1)
        v |= std::uint32_t{transfer(i == 3 ? 0xFF : 0x00)} << (8 * i);  // LSB first
    }
    select(false);
    return v;
}

void write32(std::uint16_t offset, std::uint32_t value) noexcept {
    const auto address = static_cast<std::uint16_t>(system_base + offset);
    select(true);
    transfer(0x02);  // WRITE
    transfer(static_cast<std::uint8_t>(address >> 8));
    transfer(static_cast<std::uint8_t>(address));
    for (int i = 0; i < 4; ++i) {
        transfer(static_cast<std::uint8_t>(value >> (8 * i)));
    }
    select(false);
}

// ---- EtherCAT core CSRs through ECAT_CSR_CMD/ECAT_CSR_DATA (11.13.1) ------------------------------
constexpr std::uint32_t csr_busy = 1u << 31, csr_read = 1u << 30;
bool csr_ok = true;  ///< false: a command did not finish (logged once)

bool csr_wait() noexcept {
    for (int i = 0; i < 1000; ++i) {
        if ((read32(ecat_csr_cmd) & csr_busy) == 0) return true;
    }
    if (csr_ok) RTOS_LOG_ERROR("ethercat: ECAT_CSR_CMD stays busy");
    csr_ok = false;
    return false;
}

// ---- EtherCAT core CSRs, Direct Mapped mode: READ/WRITE at their own address (9.3.2) ------------
std::uint32_t direct_read(std::uint16_t address, unsigned size) noexcept {
    select(true);
    transfer(0x03);  // READ, byte by byte, incrementing
    transfer(static_cast<std::uint8_t>((address | increment) >> 8));
    transfer(static_cast<std::uint8_t>(address));
    std::uint32_t v = 0;
    for (unsigned i = 0; i < size; ++i) {
        wait_ns(400);  // each byte: 370 ns until the core has delivered it
        v |= std::uint32_t{transfer(i + 1 == size ? 0xFF : 0x00)} << (8 * i);  // SI high on the last: no prefetch
    }
    select(false);
    return v;
}

void direct_write(std::uint16_t address, unsigned size, std::uint32_t value) noexcept {
    select(true);
    transfer(0x02);  // WRITE, incrementing
    transfer(static_cast<std::uint8_t>((address | increment) >> 8));
    transfer(static_cast<std::uint8_t>(address));
    for (unsigned i = 0; i < size; ++i) {
        transfer(static_cast<std::uint8_t>(value >> (8 * i)));
        wait_ns(250);  // the core takes 250 ns per byte
    }
    select(false);
    wait_ns(250 + 80 * size);  // no command while the core write is pending
}

std::uint32_t esc_read(std::uint16_t address, unsigned size) noexcept {
    if (direct_mapped) return direct_read(address, size);
    write32(ecat_csr_cmd, csr_busy | csr_read | (std::uint32_t{size} << 16) | address);
    if (!csr_wait()) return 0;
    const std::uint32_t v = read32(ecat_csr_data);
    return size == 4 ? v : v & ((1u << (8 * size)) - 1u);
}

void esc_write(std::uint16_t address, unsigned size, std::uint32_t value) noexcept {
    if (direct_mapped) {
        direct_write(address, size, value);
        return;
    }
    write32(ecat_csr_data, value);
    write32(ecat_csr_cmd, csr_busy | (std::uint32_t{size} << 16) | address);
    (void)csr_wait();
}

// EtherCAT core registers (11.16)
constexpr std::uint16_t esc_type = 0x0000, esc_revision = 0x0001, esc_build = 0x0002, esc_fmmus = 0x0004,
                        esc_syncmanagers = 0x0005, esc_ram_kb = 0x0006, esc_ports = 0x0007, esc_features = 0x0008,
                        dl_status = 0x0110, al_control = 0x0120, al_status = 0x0130, al_status_code = 0x0134,
                        run_led_override = 0x0138, pdi_control = 0x0140, esc_config = 0x0141, pdi_config = 0x0150, sync_latch_config = 0x0151,
                        ext_pdi_config = 0x0152, eeprom_status = 0x0502, eeprom_address = 0x0504, eeprom_data = 0x0508;
constexpr std::uint16_t eeprom_busy = 1u << 15, eeprom_emulation = 1u << 5, eeprom_8_bytes = 1u << 6;
constexpr unsigned cmd_read = 1, cmd_write = 2, cmd_reload = 4;

// ---- PHY registers through the ESC's MII management (11.16.53-58), a link diagnosis --------------
// PHY A at address 0 or 2 (device mode), PHY B at 1 (10.1.1): addresses 0..3 are scanned.
constexpr std::uint16_t mii_control = 0x0510, mii_phy_address = 0x0512, mii_register = 0x0513, mii_data = 0x0514,
                        mii_pdi_access = 0x0517;
constexpr std::uint16_t mii_busy = 1u << 15, mii_command_error = 1u << 14, mii_read_error = 1u << 13, mii_cmd_read = 0x0100,
                        mii_cmd_write = 0x0200;
// PHY registers (10.2.18)
constexpr std::uint8_t phy_basic_control = 0, phy_basic_status = 1, phy_id_msb = 2, phy_id_lsb = 3, phy_mode_control_status = 17,
                       phy_an_advertisement = 4, phy_special_modes = 18, phy_special_control_status_ind = 27,
                       phy_special_control_status = 31;
constexpr std::uint16_t bmsr_link = 1u << 2, bmsr_an_complete = 1u << 5, energy_on = 1u << 1;

bool mii_idle() noexcept {
    for (int i = 0; i < 1000; ++i) {
        if ((esc_read(mii_control, 2) & mii_busy) == 0) return true;
    }
    return false;
}

/// One PHY register; -1: no answer (busy, invalid command, read error - no PHY at that address).
std::int32_t mii_read(std::uint8_t phy, std::uint8_t reg) noexcept {
    if (!mii_idle()) return -1;
    esc_write(mii_phy_address, 1, phy);
    esc_write(mii_register, 1, reg);
    esc_write(mii_control, 2, mii_cmd_read);
    if (!mii_idle()) return -1;
    if ((esc_read(mii_control, 2) & (mii_command_error | mii_read_error)) != 0) {
        esc_write(mii_control, 2, 0);  // command 00: clears the error bits
        return -1;
    }
    return static_cast<std::int32_t>(esc_read(mii_data, 2));
}

bool mii_write(std::uint8_t phy, std::uint8_t reg, std::uint16_t value) noexcept {
    if (!mii_idle()) return false;
    esc_write(mii_phy_address, 1, phy);
    esc_write(mii_register, 1, reg);
    esc_write(mii_data, 2, value);
    esc_write(mii_control, 2, mii_cmd_write);
    if (!mii_idle()) return false;
    if ((esc_read(mii_control, 2) & mii_command_error) != 0) {
        esc_write(mii_control, 2, 0);
        return false;
    }
    return true;
}

/// The PDI takes the MII management (it stays with the PDI: no master needs it for this test).
bool mii_claim() noexcept {
    esc_write(mii_pdi_access, 1, 1);
    return (esc_read(mii_pdi_access, 1) & 1) != 0;
}

struct Phy {
    std::uint8_t address;
    std::uint16_t status;  ///< link, autonegotiation complete, ENERGYON - the last logged
};
std::array<Phy, 4> phys{};
std::size_t phy_count = 0;
std::uint32_t last_phy_poll = 0;

/// Basic status twice: the link bit latches low until read (802.3 22.2.4.2.13).
std::uint16_t phy_state(std::uint8_t address) noexcept {
    (void)mii_read(address, phy_basic_status);
    const std::int32_t bmsr = mii_read(address, phy_basic_status);
    const std::int32_t mode = mii_read(address, phy_mode_control_status);
    if (bmsr < 0 || mode < 0) return 0xFFFF;
    return static_cast<std::uint16_t>((bmsr & (bmsr_link | bmsr_an_complete)) | ((mode & energy_on) != 0 ? 0x8000 : 0));
}

void log_phy_state(std::uint8_t address, std::uint16_t s) noexcept {
    RTOS_LOG_INFO("ethercat: PHY %u - link %s, autonegotiation %s, %s", address, (s & bmsr_link) != 0 ? "up" : "down",
                  (s & bmsr_an_complete) != 0 ? "complete" : "not complete",
                  (s & 0x8000) != 0 ? "energy on the cable (link partner sends)" : "no energy on the cable");
}

/// Both ports alike, whatever the straps say: autonegotiation advertising only 100BASE-TX full
/// duplex (EtherCAT), Auto-MDIX. On revision 1 the 100FD_B strap (ERRLED) sets PHY B to fixed
/// 100 Mbit/s full duplex without Auto-MDIX (HW_CFG bits 24/26) - a negotiating PC adapter then
/// gets no stable link. Through the PHY registers, since MI link detection and enhanced link
/// detection are off (HW_CFG bit 24 description).
void configure_phy(std::uint8_t a) noexcept {
    (void)mii_write(a, phy_an_advertisement, 0x0101);           // 100BASE-TX full duplex, selector 802.3
    (void)mii_write(a, phy_special_control_status_ind, 0xC000);  // AMDIXCTRL, AMDIXEN: Auto-MDIX (table 10-14)
    (void)mii_write(a, phy_basic_control, 0x3300);               // 100 Mbit/s, autonegotiation on and restarted, full duplex
}

void scan_phys() noexcept {
    if (!mii_claim()) {
        RTOS_LOG_WARN("ethercat: MII management not available to the PDI (0x0510 0x%04lx, 0x0516 0x%02lx)",
                      static_cast<unsigned long>(esc_read(mii_control, 2)), static_cast<unsigned long>(esc_read(0x0516, 1)));
        return;
    }
    RTOS_LOG_INFO("ethercat: MII management 0x%04lx (PHY address offset %lu, MI link detection %s)",
                  static_cast<unsigned long>(esc_read(mii_control, 2)), static_cast<unsigned long>((esc_read(mii_control, 2) >> 3) & 0x1F),
                  (esc_read(mii_control, 2) & 0x4) != 0 ? "on" : "off");
    for (std::uint8_t a = 0; a < 4; ++a) {
        const std::int32_t id1 = mii_read(a, phy_id_msb);
        if (id1 < 0 || id1 == 0xFFFF) continue;
        const std::int32_t bmcr = mii_read(a, phy_basic_control);
        RTOS_LOG_INFO("ethercat: PHY %u - ID 0x%04lx%04lx, basic control 0x%04lx (%s%s%s), status 0x%04lx", a, static_cast<unsigned long>(id1),
                      static_cast<unsigned long>(mii_read(a, phy_id_lsb)), static_cast<unsigned long>(bmcr),
                      (bmcr & (1 << 11)) != 0 ? "POWER DOWN, " : "", (bmcr & (1 << 14)) != 0 ? "LOOPBACK, " : "",
                      (bmcr & (1 << 12)) != 0 ? "autonegotiation" : "fixed", static_cast<unsigned long>(mii_read(a, phy_basic_status)));
        const std::int32_t modes = mii_read(a, phy_special_modes);
        RTOS_LOG_INFO("ethercat: PHY %u - mode control/status 0x%04lx, special modes 0x%04lx (MODE %lu, address %lu), "
                      "special control/status ind. 0x%04lx, special control/status 0x%04lx",
                      a, static_cast<unsigned long>(mii_read(a, phy_mode_control_status)), static_cast<unsigned long>(modes),
                      static_cast<unsigned long>((modes >> 5) & 7), static_cast<unsigned long>(modes & 0x1F),
                      static_cast<unsigned long>(mii_read(a, phy_special_control_status_ind)),
                      static_cast<unsigned long>(mii_read(a, phy_special_control_status)));
        configure_phy(a);
        if (phy_count < phys.size()) {
            const auto s = phy_state(a);
            phys[phy_count++] = Phy{a, s};
            log_phy_state(a, s);
        }
    }
    if (phy_count == 0) RTOS_LOG_WARN("ethercat: no PHY answers on the MII management (addresses 0..3)");
}

/// Once a second: link, autonegotiation and energy of the PHYs, logged on change.
void watch_phys(std::uint32_t now) noexcept {
    if (phy_count == 0 || now - last_phy_poll < 1000) return;
    last_phy_poll = now;
    for (std::size_t i = 0; i < phy_count; ++i) {
        const auto s = phy_state(phys[i].address);
        if (s != phys[i].status) {
            log_phy_state(phys[i].address, s);
            phys[i].status = s;
        }
    }
}

// ---- the emulated EEPROM: the slave information interface (SII, ethercat_sii.hpp) ----------------
sii::Image image{};  ///< written by the master stays until the reset

/// RUN LED off by override (11.16.23): with the RUNLED/EE_EMUL0 strap at 0 the pin is an active-high
/// open-source output; on revision 1 it is wired straight to GND for the strap, and "off" keeps it
/// from driving high into that wire. A valid AL status write ends the override: it is set again
/// after each one (serve_al()).
constexpr std::uint32_t run_led_off = 0x10;  // override enabled, code 0 (off)

// ---- state -----------------------------------------------------------------------------------------
bool present = false;
bool ready = false;  ///< HW_CFG READY: configuration loaded; before it only the EEPROM emulation
std::uint32_t since = 0;  ///< time of setup() (cycle's `now`), for the READY timeout
bool ready_late_logged = false;
std::uint16_t last_dl = 0xFFFF, last_al = 0xFFFF;
std::uint32_t eeprom_commands = 0;

const char* state_name(unsigned s) noexcept {
    switch (s & 0x0F) {
        case 1: return "INIT";
        case 2: return "PRE-OP";
        case 3: return "BOOT";
        case 4: return "SAFE-OP";
        case 8: return "OP";
        default: return "?";
    }
}

void serve_eeprom() noexcept {
    const auto status = static_cast<std::uint16_t>(esc_read(eeprom_status, 2));
    if ((status & eeprom_busy) == 0 || (status & eeprom_emulation) == 0) return;
    const unsigned cmd = (status >> 8) & 0x7;
    const std::uint32_t address = esc_read(eeprom_address, 4);  // word address
    const unsigned words = (status & eeprom_8_bytes) != 0 ? 4 : 2;
    auto word = [](std::uint32_t a) -> std::uint32_t { return a < image.size() ? image[a] : 0xFFFF; };
    if (cmd == cmd_read || cmd == cmd_reload) {
        esc_write(eeprom_data, 4, word(address) | (word(address + 1) << 16));
        if (words == 4) esc_write(eeprom_data + 4, 4, word(address + 2) | (word(address + 3) << 16));
    } else if (cmd == cmd_write) {
        if (address < image.size()) image[address] = static_cast<std::uint16_t>(esc_read(eeprom_data, 2));
    }
    esc_write(eeprom_status, 2, cmd << 8);  // acknowledge: the command executed (note 24)
    if (eeprom_commands++ < 40 || cmd != cmd_read) {
        RTOS_LOG_INFO("ethercat: EEPROM %s word 0x%04lx", cmd == cmd_read ? "read" : cmd == cmd_write ? "write" : cmd == cmd_reload ? "reload" : "?",
                      static_cast<unsigned long>(address));
    }
}

void serve_al() noexcept {
    const auto control = static_cast<std::uint16_t>(esc_read(al_control, 2));  // reading clears the AL control event
    const auto status = static_cast<std::uint16_t>(esc_read(al_status, 2));
    const unsigned wanted = control & 0x0F;
    if (wanted == 0 || (wanted == (status & 0x0F) && (status & 0x10) == 0)) return;
    if (wanted == 3) {  // BOOT: no firmware update here
        esc_write(al_status_code, 2, 0x0011);  // invalid requested state change
        esc_write(al_status, 2, (status & 0x0F) | 0x10);
        RTOS_LOG_WARN("ethercat: master requests BOOT - refused");
        return;
    }
    esc_write(al_status_code, 2, 0);
    esc_write(al_status, 2, wanted);  // no checks: a test without mailbox and process data
    esc_write(run_led_override, 1, run_led_off);
    RTOS_LOG_INFO("ethercat: AL state %s -> %s", state_name(status), state_name(wanted));
}

// ---- register access over EtherCAT (ethercat_registers.hpp): the Modbus register map -------------
namespace ra = register_access;
std::uint16_t last_sequence = 0;
std::uint32_t register_requests = 0;

void esc_read_bytes(std::uint16_t address, std::span<std::uint8_t> out) noexcept {
    for (std::size_t i = 0; i < out.size(); i += 4) {
        const auto n = static_cast<unsigned>(std::min<std::size_t>(4, out.size() - i));
        const std::uint32_t v = esc_read(static_cast<std::uint16_t>(address + i), n);
        for (unsigned j = 0; j < n; ++j) out[i + j] = static_cast<std::uint8_t>(v >> (8 * j));
    }
}

void esc_write_bytes(std::uint16_t address, std::span<const std::uint8_t> in) noexcept {
    for (std::size_t i = 0; i < in.size(); i += 4) {
        const auto n = static_cast<unsigned>(std::min<std::size_t>(4, in.size() - i));
        std::uint32_t v = 0;
        for (unsigned j = 0; j < n; ++j) v |= std::uint32_t{in[i + j]} << (8 * j);
        esc_write(static_cast<std::uint16_t>(address + i), n, v);
    }
}

/// One request per I/O cycle: a new sequence number at 0x1000 triggers it; the response is written
/// with its sequence number last. Only EtherCAT Direct Mapped (process RAM by address).
void serve_registers() noexcept {
    if (!direct_mapped) return;
    const auto sequence = static_cast<std::uint16_t>(esc_read(ra::request_address, 2));
    if (sequence == 0 || sequence == last_sequence) return;
    ra::Area request{};
    esc_read_bytes(ra::request_address, std::span(request).first(ra::header_bytes));
    const std::size_t values = ra::request_value_bytes(request);
    esc_read_bytes(ra::request_address + ra::header_bytes, std::span(request).subspan(ra::header_bytes, values));
    ra::Area response{};
    ra::execute(request, ::registers, response);
    esc_write_bytes(ra::response_address + 2, std::span<const std::uint8_t>(response).subspan(2, ra::response_bytes(response) - 2));
    esc_write(ra::response_address, 2, sequence);
    last_sequence = sequence;
    if (register_requests++ == 0) {
        RTOS_LOG_INFO("ethercat: first register request from the master - function 0x%02x, start %u, count %u, exception %u",
                      response[2], ra::get16(response, 4), ra::get16(response, 6), response[3]);
    }
}

void watch_links() noexcept {
    const auto dl = static_cast<std::uint16_t>(esc_read(dl_status, 2));
    if (dl != last_dl) {
        // bit 0 PDI operational (EEPROM loaded), 4/5 physical link port 0/1, 8.. loop/communication per port
        RTOS_LOG_INFO("ethercat: DL status 0x%04x - PDI %s, port 0 link %s%s, port 1 link %s%s", dl, (dl & 1) != 0 ? "ok" : "not operational",
                      (dl & 0x10) != 0 ? "up" : "down", (dl & 0x0200) != 0 ? " (communication)" : "",
                      (dl & 0x20) != 0 ? "up" : "down", (dl & 0x0800) != 0 ? " (communication)" : "");
        last_dl = dl;
    }
    const auto al = static_cast<std::uint16_t>(esc_read(al_status, 2));
    if (al != last_al) {
        RTOS_LOG_INFO("ethercat: AL status 0x%04x (%s%s)", al, state_name(al), (al & 0x10) != 0 ? ", error" : "");
        last_al = al;
    }
}

}  // namespace

void setup() noexcept {
    if constexpr (!pins::ethercat.has_value()) {
        return;
    } else {
        write(pins::ethercat_reset, false);
        kernel.sleep(2);
        write(pins::ethercat_reset, true);  // out of reset
        // the SPI answers once the device is up: BYTE_TEST reads 87654321h (the first reads may not) -
        // at 0x064 (LAN9252 compatible) or at 0x3064 (EtherCAT Direct Mapped)
        std::uint32_t test = 0, test_direct = 0;
        for (int i = 0; i < 100 && test != byte_test_value && test_direct != byte_test_value; ++i) {
            kernel.sleep(2);
            system_base = 0;
            test = read32(byte_test);
            system_base = direct_system_base;
            test_direct = read32(byte_test);
        }
        direct_mapped = test_direct == byte_test_value;
        system_base = direct_mapped ? direct_system_base : 0;
        if (test != byte_test_value && !direct_mapped) {
            RTOS_LOG_ERROR("ethercat: no answer on SPI (BYTE_TEST 0x%08lx at 0x064, 0x%08lx at 0x3064, expected 0x87654321) - "
                           "PDI not SPI (EE_EMUL straps), wiring, or the LAN9253 has no clock/supply",
                           static_cast<unsigned long>(test), static_cast<unsigned long>(test_direct));
            return;
        }
        RTOS_LOG_INFO("ethercat: SPI %s", direct_mapped ? "EtherCAT Direct Mapped (system CSRs at 0x3000)"
                                                        : "LAN9252 compatible (core registers through ECAT_CSR_CMD)");
        const std::uint32_t cfg = read32(hw_cfg);
        RTOS_LOG_INFO("ethercat: LAN9253 on SPI - ID_REV 0x%08lx, HW_CFG 0x%08lx (%s)", static_cast<unsigned long>(read32(id_rev)),
                      static_cast<unsigned long>(cfg), (cfg & hw_cfg_ready) != 0 ? "ready" : "not ready, waiting for the EEPROM reload");
        // the PDI type of the straps, not read from 0x0140: see ethercat_sii.hpp
        image = sii::make(direct_mapped);
        const auto eeprom = static_cast<std::uint16_t>(esc_read(eeprom_status, 2));
        RTOS_LOG_INFO("ethercat: EEPROM status 0x%04x - %s, %s, %u bytes per read", eeprom,
                      (eeprom & eeprom_emulation) != 0 ? "emulated by the PDI" : "I2C EEPROM",
                      (eeprom & (1u << 12)) != 0 ? "not loaded" : "loaded", (eeprom & eeprom_8_bytes) != 0 ? 8u : 4u);
        present = true;
    }
}

namespace {

/// Once READY is set (the configuration area is loaded): the ESC's identity and settings - reads of
/// these registers before READY are forbidden (DS00003421 6.x, HW_CFG), writes are lost.
void log_configured() noexcept {
    RTOS_LOG_INFO("ethercat: ready - HW_CFG 0x%08lx", static_cast<unsigned long>(read32(hw_cfg)));
    RTOS_LOG_INFO("ethercat: ESC type 0x%02lx rev %lu build 0x%04lx, %lu FMMUs, %lu SyncManagers, %lu KB RAM, ports 0x%02lx, features 0x%04lx",
                  static_cast<unsigned long>(esc_read(esc_type, 1)), static_cast<unsigned long>(esc_read(esc_revision, 1)),
                  static_cast<unsigned long>(esc_read(esc_build, 2)), static_cast<unsigned long>(esc_read(esc_fmmus, 1)),
                  static_cast<unsigned long>(esc_read(esc_syncmanagers, 1)), static_cast<unsigned long>(esc_read(esc_ram_kb, 1)),
                  static_cast<unsigned long>(esc_read(esc_ports, 1)), static_cast<unsigned long>(esc_read(esc_features, 2)));
    const auto eeprom = static_cast<std::uint16_t>(esc_read(eeprom_status, 2));
    RTOS_LOG_INFO("ethercat: PDI control 0x%02lx (0x80/0x82: SPI), ESC config 0x%02lx, PDI config 0x%02lx, SYNC/latch 0x%02lx, ext PDI 0x%04lx, EEPROM %s%s",
                  static_cast<unsigned long>(esc_read(pdi_control, 1)), static_cast<unsigned long>(esc_read(esc_config, 1)),
                  static_cast<unsigned long>(esc_read(pdi_config, 1)), static_cast<unsigned long>(esc_read(sync_latch_config, 1)),
                  static_cast<unsigned long>(esc_read(ext_pdi_config, 2)), (eeprom & (1u << 12)) != 0 ? "not loaded" : "loaded",
                  (eeprom & (1u << 11)) != 0 ? ", checksum error" : "");
}

}  // namespace

void cycle(std::uint32_t now) noexcept {
    if (!present || !csr_ok) return;
    if (since == 0) since = now == 0 ? 1 : now;
    for (int i = 0; i < 8; ++i) {  // a master reads the SII in bursts: several commands per cycle
        const auto status = static_cast<std::uint16_t>(esc_read(eeprom_status, 2));
        if ((status & eeprom_busy) == 0) break;
        serve_eeprom();
    }
    if (!ready) {
        const std::uint32_t cfg = read32(hw_cfg);
        if ((cfg & hw_cfg_ready) == 0) {
            if (!ready_late_logged && now - since > 2000) {
                const auto eeprom = static_cast<std::uint16_t>(esc_read(eeprom_status, 2));
                RTOS_LOG_WARN("ethercat: still not ready after 2 s - HW_CFG 0x%08lx, EEPROM status 0x%04x (%s%s), %lu EEPROM commands served",
                              static_cast<unsigned long>(cfg), eeprom, (eeprom & (1u << 12)) != 0 ? "not loaded" : "loaded",
                              (eeprom & (1u << 11)) != 0 ? ", checksum error" : "", static_cast<unsigned long>(eeprom_commands));
                ready_late_logged = true;
            }
            return;  // before READY: no other register (DS00003421 HW_CFG note)
        }
        ready = true;
        wait_ns(1500);  // 13.4: the new configuration takes effect 1.5 us after the last emulation write
        log_configured();
        esc_write(run_led_override, 1, run_led_off);
        scan_phys();
        last_sequence = static_cast<std::uint16_t>(esc_read(register_access::request_address, 2));  // no stale request
        last_phy_poll = now;
    }
    serve_al();
    serve_registers();
    watch_links();
    watch_phys(now);
}

}  // namespace fcu::ethercat
