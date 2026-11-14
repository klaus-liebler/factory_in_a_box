// Start-up of the FCU (see boot.hpp).
#include "boot.hpp"

#include <array>

#include "board/fcu.hpp"
#include "device_ids.hh"
#include "firmware_constants.hh"
#include "gitconstants.hh"
#include "hardware.hpp"
#include "power_delivery.hpp"
#include "system.hpp"

namespace fcu::boot {
namespace {

using namespace mcu;  // RCC_RSR, its flags, uid_base: the revision's MCU (mcu.h)

ResetCause g_reset_cause = ResetCause::Unknown;

std::array<std::uint32_t, 3> chip_uid() noexcept {
    const auto* uid = reinterpret_cast<const volatile std::uint32_t*>(uid_base);
    return {uid[0], uid[1], uid[2]};
}

// ---- supply gate ----------------------------------------------------------------------------------
constexpr std::int32_t supply_min_mv = 18'000;
constexpr int pd_request_mv = 20'000;  // revisions 2/3: the CH224K's/CH224Q's request (CFG1)

}  // namespace

void read_reset_cause() noexcept {
    const std::uint32_t rsr = RCC_RSR::read();
    g_reset_cause = (rsr & rsr_iwdgrstf) != 0   ? ResetCause::IndependentWatchdog
                    : (rsr & rsr_wwdgrstf) != 0 ? ResetCause::WindowWatchdog
                    : (rsr & rsr_sftrstf) != 0  ? ResetCause::Software
                    : (rsr & rsr_lpwrrstf) != 0 ? ResetCause::LowPower
                    : (rsr & rsr_borrstf) != 0  ? ResetCause::BrownOut
                    : (rsr & rsr_pinrstf) != 0  ? ResetCause::PinOrPowerOn
                                                : ResetCause::Unknown;
    RCC_RSR::write(rsr | rsr_rmvf);
}

ResetCause reset_cause() noexcept { return g_reset_cause; }

bool identity_matches() noexcept {
    const auto uid = chip_uid();
    return uid[0] == DEVICE_CHIP_UID_W0 && uid[1] == DEVICE_CHIP_UID_W1 && uid[2] == DEVICE_CHIP_UID_W2;
}

void announce() noexcept {
    static constexpr std::array<const char*, 7> causes{"unknown",         "independent watchdog", "window watchdog",
                                                       "software",        "low power",            "brown-out",
                                                       "pin / power-on"};
    const auto uid = chip_uid();
    {
        auto b = RTOS_LOG_BLOCK(Info);
        b.add("=== FactoryInABox Control Unit (rtoos) ===");
        b.add(" board     %.*s, hardware revision %u", static_cast<int>(BOARD_NAME.size()), BOARD_NAME.data(),
               board::revision);
        b.add(" hostname  %s.local", DEVICE_HOSTNAME);
        b.add(" version   %.*s, commit %.*s (%.*s)%s", static_cast<int>(git::VERSION.size()), git::VERSION.data(),
               static_cast<int>(git::COMMIT_HASH.size()), git::COMMIT_HASH.data(), static_cast<int>(git::BRANCH.size()),
               git::BRANCH.data(), git::IS_DIRTY ? ", dirty" : "");
        b.add(" clocks    %.*s, CPU %u MHz, PCLK1 %u MHz", static_cast<int>(board::mcu_name.size()),
               board::mcu_name.data(), static_cast<unsigned>(board::sysclk_hz / 1'000'000),
               static_cast<unsigned>(hw::clocks.pclk1_hz() / 1'000'000));
        b.add(" chip      %08lX-%08lX-%08lX", static_cast<unsigned long>(uid[0]), static_cast<unsigned long>(uid[1]),
               static_cast<unsigned long>(uid[2]));
        b.add(" reset     %s", causes[static_cast<std::size_t>(g_reset_cause)]);
    }
    register_map::set_fw_version_major(registers, FW_VERSION_MAJOR);
    register_map::set_fw_version_minor(registers, FW_VERSION_MINOR);
    register_map::set_fw_version_patch(registers, FW_VERSION_PATCH);
    register_map::set_chip_id(registers, register_map::Uid96{uid[0], uid[1], uid[2]});
}

void update_pd_registers() noexcept {
    if (const auto s = pd::status()) {  // the MCU's own sink (revision 1)
        const std::uint16_t status = !s->connected ? 2 : (s->active_mv < pd_request_mv ? 1 : 0);
        register_map::set_pwr_pd_voltage_mv(registers, static_cast<std::uint16_t>(s->active_mv));
        register_map::set_pwr_pd_current_ma(registers, static_cast<std::uint16_t>(s->active_ma));
        register_map::set_pwr_pd_status(registers, status);
        return;
    }
    // A CH224K/CH224Q reports nothing the firmware reads: the measured bus voltage is all there
    // is. At 18 V or more the 20 V contract (or another supply) holds.
    const auto m = hw::Supply::read();
    const bool up = m && m->bus_mv >= supply_min_mv;
    register_map::set_pwr_pd_voltage_mv(registers, up ? static_cast<std::uint16_t>(m->bus_mv) : 0);
    register_map::set_pwr_pd_current_ma(registers, 0);
    register_map::set_pwr_pd_status(registers, up ? 0 : 2);
}

void poll_pd_events() noexcept {
    pd::poll();
    update_pd_registers();
}

bool wait_for_supply() noexcept {
    chip::gpio::configure<pins::supply_pins>();
    hw::I2cSupply::init();
    hw::enable_i2c_irqs<hw::I2cSupply>();
    if (!hw::Supply::probe() || !hw::Supply::init()) {
        RTOS_LOG_ERROR("supply: INA226 (I2C%u, 0x40) not found, supply voltage unknown", hw::i2c_supply_bus);
        return false;
    }
    kernel.sleep(40);  // first measurement: 16 x (1.1 ms + 1.1 ms) = 35.2 ms (hw::Supply)
    auto bus_mv = [] {
        for (int attempt = 0; attempt < 5; ++attempt) {
            if (const auto m = hw::Supply::read()) {
                return m->bus_mv;
            }
            kernel.sleep(1);
        }
        return std::int32_t{-1};
    };
    std::int32_t mv = bus_mv();
    if (mv < 0) {
        RTOS_LOG_ERROR("supply: INA226 does not answer");
        return false;
    }
    if (mv >= supply_min_mv) {
        RTOS_LOG_INFO("supply: %ld mV present%s", static_cast<long>(mv), hw::mcu_power_delivery ? ", USB-PD not needed" : "");
        update_pd_registers();
        return true;
    }
    if constexpr (hw::mcu_power_delivery) {
        RTOS_LOG_INFO("supply: only %ld mV, asking USB-PD (%.*s) for %d mV", static_cast<long>(mv),
                      static_cast<int>(hw::pd_controller.size()), hw::pd_controller.data(), pd_request_mv);
        pd::request(pd_request_mv);
    } else {
        RTOS_LOG_INFO("supply: only %ld mV, waiting for the %.*s's %d mV", static_cast<long>(mv),
                      static_cast<int>(hw::pd_controller.size()), hw::pd_controller.data(), pd_request_mv);
    }
    rtos::Tick last_log = kernel.now();
    while (mv < supply_min_mv) {
        poll_pd_events();
        kernel.sleep(10);
        mv = bus_mv();
        if (kernel.now() - last_log >= 5'000) {
            last_log = kernel.now();
            RTOS_LOG_WARN("supply: still waiting for %ld mV, now %ld mV", static_cast<long>(supply_min_mv),
                          static_cast<long>(mv));
        }
    }
    poll_pd_events();
    update_pd_registers();
    RTOS_LOG_INFO("supply: %ld mV reached", static_cast<long>(mv));
    return true;
}

}  // namespace fcu::boot
