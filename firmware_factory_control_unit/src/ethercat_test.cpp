// LAN9253 bring-up test (see ethercat_test.hpp). Register addresses and bits: LAN9253 data sheet
// DS00003421B (system CSRs: table 5-1, LAN9252-compatible indirect mode; EtherCAT core CSRs:
// 11.16; EEPROM emulation: 12.2, 13.4) and the EtherCAT slave information (ETG.2010).
#include "ethercat_test.hpp"

#include <array>
#include <cstring>

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

void select(bool on) noexcept {
    write(pins::ethercat->cs, !on);
    pause();
}

// System CSRs (byte address, 14 bits; bits 15:14 = 00: the bytes of the DWORD in order)
constexpr std::uint16_t id_rev = 0x050, byte_test = 0x064, hw_cfg = 0x074, ecat_csr_data = 0x300, ecat_csr_cmd = 0x304;
constexpr std::uint32_t byte_test_value = 0x8765'4321, hw_cfg_ready = 1u << 27;

std::uint32_t read32(std::uint16_t address) noexcept {
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

void write32(std::uint16_t address, std::uint32_t value) noexcept {
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

std::uint32_t esc_read(std::uint16_t address, unsigned size) noexcept {
    write32(ecat_csr_cmd, csr_busy | csr_read | (std::uint32_t{size} << 16) | address);
    if (!csr_wait()) return 0;
    const std::uint32_t v = read32(ecat_csr_data);
    return size == 4 ? v : v & ((1u << (8 * size)) - 1u);
}

void esc_write(std::uint16_t address, unsigned size, std::uint32_t value) noexcept {
    write32(ecat_csr_data, value);
    write32(ecat_csr_cmd, csr_busy | (std::uint32_t{size} << 16) | address);
    (void)csr_wait();
}

// EtherCAT core registers (11.16)
constexpr std::uint16_t esc_type = 0x0000, esc_revision = 0x0001, esc_build = 0x0002, esc_fmmus = 0x0004,
                        esc_syncmanagers = 0x0005, esc_ram_kb = 0x0006, esc_ports = 0x0007, esc_features = 0x0008,
                        dl_status = 0x0110, al_control = 0x0120, al_status = 0x0130, al_status_code = 0x0134,
                        pdi_control = 0x0140, esc_config = 0x0141, pdi_config = 0x0150, sync_latch_config = 0x0151,
                        ext_pdi_config = 0x0152, eeprom_status = 0x0502, eeprom_address = 0x0504, eeprom_data = 0x0508;
constexpr std::uint16_t eeprom_busy = 1u << 15, eeprom_emulation = 1u << 5, eeprom_8_bytes = 1u << 6;
constexpr unsigned cmd_read = 1, cmd_write = 2, cmd_reload = 4;

// ---- the emulated EEPROM: the slave information interface (SII, ETG.2010) ------------------------
// Test identity: no ETG vendor ID of our own yet - vendor 0 marks it as a lab device.
constexpr std::uint32_t vendor_id = 0x0000'0000, product_code = 0x0000'9253, revision_no = 1;
constexpr char device_name[] = "FCU EtherCAT test";

std::array<std::uint16_t, 128> sii{};  ///< 2 Kbit; written by the master stays until the reset

/// CRC-8 of the ESC configuration area (words 0..6): polynomial x^8+x^2+x+1, initial FFh.
std::uint8_t config_crc(const std::array<std::uint16_t, 128>& w) noexcept {
    std::uint8_t crc = 0xFF;
    for (int i = 0; i < 14; ++i) {
        crc ^= static_cast<std::uint8_t>(w[static_cast<std::size_t>(i / 2)] >> (8 * (i % 2)));
        for (int b = 0; b < 8; ++b) crc = static_cast<std::uint8_t>((crc & 0x80) != 0 ? (crc << 1) ^ 0x07 : crc << 1);
    }
    return crc;
}

/// The image: configuration area with the PDI settings the straps selected (EEPROM emulation must
/// not change the PDI type or the SPI select polarity, 13.4), identity, a name and a general category.
void build_sii(std::uint8_t pdi, std::uint8_t pdi_cfg) noexcept {
    sii.fill(0);
    sii[0] = pdi;                // PDI control (0x0140); ESC configuration (0x0141) 0: the PDI acknowledges AL states
    sii[1] = pdi_cfg;            // PDI configuration (0x0150); SYNC/latch configuration (0x0151) 0
    sii[7] = config_crc(sii);
    sii[0x08] = static_cast<std::uint16_t>(vendor_id), sii[0x09] = static_cast<std::uint16_t>(vendor_id >> 16);
    sii[0x0A] = static_cast<std::uint16_t>(product_code), sii[0x0B] = static_cast<std::uint16_t>(product_code >> 16);
    sii[0x0C] = static_cast<std::uint16_t>(revision_no), sii[0x0D] = static_cast<std::uint16_t>(revision_no >> 16);
    sii[0x3E] = 1;               // size: (2 Kbit / 1 Kbit) - 1
    sii[0x3F] = 1;               // SII version
    // category STRINGS (10): one string
    std::size_t w = 0x40;
    const std::size_t name_len = sizeof device_name - 1;
    const std::size_t string_bytes = 2 + name_len;           // count, length, characters
    const std::size_t string_words = (string_bytes + 1) / 2;
    sii[w++] = 10;
    sii[w++] = static_cast<std::uint16_t>(string_words);
    std::array<std::uint8_t, 64> bytes{};
    bytes[0] = 1;
    bytes[1] = static_cast<std::uint8_t>(name_len);
    std::memcpy(&bytes[2], device_name, name_len);
    for (std::size_t i = 0; i < string_words; ++i) sii[w++] = static_cast<std::uint16_t>(bytes[2 * i] | (bytes[2 * i + 1] << 8));
    // category GENERAL (30): group, image, order and name: string 1
    sii[w++] = 30;
    sii[w++] = 16;
    sii[w] = 0x0001;             // group index 1, image index 0
    sii[w + 1] = 0x0101;         // order index 1, name index 1
    w += 16;
    sii[w] = 0xFFFF;             // end
}

// ---- state -----------------------------------------------------------------------------------------
bool present = false;
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
    auto word = [](std::uint32_t a) -> std::uint32_t { return a < sii.size() ? sii[a] : 0xFFFF; };
    if (cmd == cmd_read || cmd == cmd_reload) {
        esc_write(eeprom_data, 4, word(address) | (word(address + 1) << 16));
        if (words == 4) esc_write(eeprom_data + 4, 4, word(address + 2) | (word(address + 3) << 16));
    } else if (cmd == cmd_write) {
        if (address < sii.size()) sii[address] = static_cast<std::uint16_t>(esc_read(eeprom_data, 2));
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
    RTOS_LOG_INFO("ethercat: AL state %s -> %s", state_name(status), state_name(wanted));
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
        // the SPI answers once the device is up: BYTE_TEST reads 87654321h (the first reads may not)
        std::uint32_t test = 0;
        for (int i = 0; i < 100 && test != byte_test_value; ++i) {
            kernel.sleep(2);
            test = read32(byte_test);
        }
        if (test != byte_test_value) {
            RTOS_LOG_ERROR("ethercat: no answer on SPI (BYTE_TEST 0x%08lx, expected 0x87654321) - PDI not SPI (straps), "
                           "wiring, or the LAN9253 has no clock/supply", static_cast<unsigned long>(test));
            return;
        }
        const std::uint32_t cfg = read32(hw_cfg);
        RTOS_LOG_INFO("ethercat: LAN9253 on SPI - ID_REV 0x%08lx, HW_CFG 0x%08lx (%s)", static_cast<unsigned long>(read32(id_rev)),
                      static_cast<unsigned long>(cfg), (cfg & hw_cfg_ready) != 0 ? "ready" : "not ready");
        RTOS_LOG_INFO("ethercat: ESC type 0x%02lx rev %lu build 0x%04lx, %lu FMMUs, %lu SyncManagers, %lu KB RAM, ports 0x%02lx, features 0x%04lx",
                      static_cast<unsigned long>(esc_read(esc_type, 1)), static_cast<unsigned long>(esc_read(esc_revision, 1)),
                      static_cast<unsigned long>(esc_read(esc_build, 2)), static_cast<unsigned long>(esc_read(esc_fmmus, 1)),
                      static_cast<unsigned long>(esc_read(esc_syncmanagers, 1)), static_cast<unsigned long>(esc_read(esc_ram_kb, 1)),
                      static_cast<unsigned long>(esc_read(esc_ports, 1)), static_cast<unsigned long>(esc_read(esc_features, 2)));
        const auto pdi = static_cast<std::uint8_t>(esc_read(pdi_control, 1));
        const auto pdi_cfg = static_cast<std::uint8_t>(esc_read(pdi_config, 1));
        const auto eeprom = static_cast<std::uint16_t>(esc_read(eeprom_status, 2));
        RTOS_LOG_INFO("ethercat: PDI control 0x%02x (0x80: SPI), ESC config 0x%02lx, PDI config 0x%02x, SYNC/latch 0x%02lx, ext PDI 0x%04lx",
                      pdi, static_cast<unsigned long>(esc_read(esc_config, 1)), pdi_cfg,
                      static_cast<unsigned long>(esc_read(sync_latch_config, 1)), static_cast<unsigned long>(esc_read(ext_pdi_config, 2)));
        RTOS_LOG_INFO("ethercat: EEPROM status 0x%04x - %s, %s, %u bytes per read", eeprom,
                      (eeprom & eeprom_emulation) != 0 ? "emulated by the PDI" : "I2C EEPROM",
                      (eeprom & (1u << 12)) != 0 ? "not loaded" : "loaded", (eeprom & eeprom_8_bytes) != 0 ? 8u : 4u);
        build_sii(pdi, pdi_cfg);
        present = true;
    }
}

void cycle(std::uint32_t) noexcept {
    if (!present || !csr_ok) return;
    for (int i = 0; i < 8; ++i) {  // a master reads the SII in bursts: several commands per cycle
        const auto status = static_cast<std::uint16_t>(esc_read(eeprom_status, 2));
        if ((status & eeprom_busy) == 0) break;
        serve_eeprom();
    }
    serve_al();
    watch_links();
}

}  // namespace fcu::ethercat
