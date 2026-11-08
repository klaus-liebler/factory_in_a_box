// Start-up of the FCU (see boot.hpp).
#include "boot.hpp"

#include <array>

#include "board/fcu.hpp"
#include "device_ids.hh"
#include "firmware_constants.hh"
#include "gitconstants.hh"
#include "hardware.hpp"
#include "system.hpp"

namespace fcu::boot {
namespace {

namespace chip = board::chip;
namespace pd = rtos::usb_pd;

ResetCause g_reset_cause = ResetCause::Unknown;

#if RTOS_FCU_REVISION == 3
// RCC_RSR (stm32h743xx.h: RCC_BASE 0x58024400, RSR at 0xD0, RCC_RSR_*_Pos)
using RCC_RSR = chip::Reg<0x5802'4400 + 0xD0>;
constexpr std::uint32_t rsr_rmvf = 1u << 16, rsr_borrstf = 1u << 21, rsr_pinrstf = 1u << 22, rsr_sftrstf = 1u << 24,
                        rsr_iwdgrstf = 1u << 26, rsr_wwdgrstf = 1u << 28, rsr_lpwrrstf = 1u << 30;
constexpr std::uintptr_t uid_base = 0x1FF1'E800;  // stm32h743xx.h: UID_BASE
#else
// RCC_RSR (RM0481, SVD v1.8: RCC base 0x44020C00 + 0xF4)
using RCC_RSR = chip::PReg<0x0402'0C00 + 0xF4>;
constexpr std::uint32_t rsr_rmvf = 1u << 23, rsr_pinrstf = 1u << 26, rsr_borrstf = 1u << 27, rsr_sftrstf = 1u << 28,
                        rsr_iwdgrstf = 1u << 29, rsr_wwdgrstf = 1u << 30, rsr_lpwrrstf = 1u << 31;
constexpr std::uintptr_t uid_base = 0x08FF'F800;  // RM0481: UID_BASE
#endif

std::array<std::uint32_t, 3> chip_uid() noexcept {
    const auto* uid = reinterpret_cast<const volatile std::uint32_t*>(uid_base);
    return {uid[0], uid[1], uid[2]};
}

// ---- supply gate ----------------------------------------------------------------------------------
constexpr std::int32_t supply_min_mv = 18'000;
constexpr int pd_request_mv = 20'000;  // revisions 2/3: the CH224K's/CH224Q's request (CFG1)

#if RTOS_FCU_REVISION != 3
// UCPD1 sink, revision 1 (board::has_ucpd); unused on revision 2
hw::PdClock pd_clock;
RTOS_DMA_BUFFER hw::Ucpd ucpd;
pd::Sink<hw::Ucpd, hw::PdClock, Kernel::Lock> pd_sink{ucpd, pd_clock};
bool pd_started = false;
#endif

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
#if RTOS_FCU_REVISION != 3
    if constexpr (!board::has_ucpd)
#endif
    {
        // The CH224K/CH224Q reports nothing the firmware reads: the measured bus voltage is all
        // there is. At 18 V or more the 20 V contract (or another supply) holds.
        const auto m = hw::Supply::read();
        const bool up = m && m->bus_mv >= supply_min_mv;
        register_map::set_pwr_pd_voltage_mv(registers, up ? static_cast<std::uint16_t>(m->bus_mv) : 0);
        register_map::set_pwr_pd_current_ma(registers, 0);
        register_map::set_pwr_pd_status(registers, up ? 0 : 2);
        return;
    }
#if RTOS_FCU_REVISION != 3
    const pd::SinkStatus s = pd_sink.status();
    const std::uint16_t status = !s.connected ? 2 : (s.active_mv < pd_request_mv ? 1 : 0);
    register_map::set_pwr_pd_voltage_mv(registers, static_cast<std::uint16_t>(s.active_mv));
    register_map::set_pwr_pd_current_ma(registers, static_cast<std::uint16_t>(s.active_ma));
    register_map::set_pwr_pd_status(registers, status);
#endif
}

void poll_pd_events() noexcept {
#if RTOS_FCU_REVISION != 3
    if (!pd_started) {
        return;
    }
    pd_sink.poll([](pd::SinkEvent e) {
        const pd::SinkStatus s = pd_sink.status();
        switch (e) {
            case pd::SinkEvent::SourceCapabilitiesChanged: {
                auto b = RTOS_LOG_BLOCK(Info);
                b.add("USB-PD: %u source capabilities", static_cast<unsigned>(s.capability_count));
                for (std::size_t i = 0; i < s.capability_count; ++i) {
                    const auto& c = s.capabilities[i];
                    b.add("  [%u] %-8s %5u-%5u mV, max %4u mA", static_cast<unsigned>(i), pd::supply_type_name(c.type),
                           c.min_mv, c.max_mv, c.max_ma);
                }
                break;
            }
            case pd::SinkEvent::VoltageChanged:
                RTOS_LOG_INFO("USB-PD: supply now %d mV / %d mA", s.active_mv, s.active_ma);
                break;
            case pd::SinkEvent::PowerRejected: RTOS_LOG_WARN("USB-PD: request rejected by the source"); break;
        }
    });
    update_pd_registers();
#endif
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
        RTOS_LOG_INFO("supply: %ld mV present%s", static_cast<long>(mv), board::has_ucpd ? ", USB-PD not needed" : "");
        update_pd_registers();
        return true;
    }
#if RTOS_FCU_REVISION != 3
    if constexpr (board::has_ucpd) {
        RTOS_LOG_INFO("supply: only %ld mV, asking USB-PD for %d mV", static_cast<long>(mv), pd_request_mv);
        hw::PdClock::init();
        (void)pd_sink.request_power(pd_request_mv);
        ucpd.start();
        pd_started = true;
        enable_irq(hw::Ucpd::irq, irq_priority_pd);
        enable_irq(hw::PdClock::irq, irq_priority_pd);
    } else
#endif
    {
        RTOS_LOG_INFO("supply: only %ld mV, waiting for the %s's %d mV", static_cast<long>(mv),
                      board::revision == 3 ? "CH224Q" : "CH224K", pd_request_mv);
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

#if RTOS_FCU_REVISION != 3
extern "C" void UCPD1_IRQHandler() noexcept { fcu::boot::ucpd.on_irq(fcu::boot::pd_sink); }

extern "C" void TIM5_IRQHandler() noexcept {
    if (fcu::hw::PdClock::take()) {
        fcu::boot::pd_sink.on_timer();
    }
}
#endif
