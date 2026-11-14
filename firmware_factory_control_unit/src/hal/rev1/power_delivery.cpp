// USB-PD of FCU revision 1: the MCU's UCPD1 sink (rtos::usb_pd::Sink) with TIM5 for its protocol
// deadlines; both interrupts at irq_priority_pd. Started only when the supply is below 18 V
// (boot.cpp) - another supply needs no contract.
#include "power_delivery.hpp"

#include <cstddef>

#include "hardware.hpp"
#include "rtos/usb_pd/sink.hpp"
#include "system.hpp"

namespace fcu::pd {
namespace {

namespace usb_pd = rtos::usb_pd;

hw::PdClock pd_clock;
RTOS_DMA_BUFFER hw::Ucpd ucpd;
usb_pd::Sink<hw::Ucpd, hw::PdClock, Kernel::Lock> sink{ucpd, pd_clock};
bool started = false;

}  // namespace

void request(int mv) noexcept {
    hw::PdClock::init();
    (void)sink.request_power(mv);
    ucpd.start();
    started = true;
    enable_irq(hw::Ucpd::irq, irq_priority_pd);
    enable_irq(hw::PdClock::irq, irq_priority_pd);
}

void poll() noexcept {
    if (!started) {
        return;
    }
    sink.poll([](usb_pd::SinkEvent e) {
        const usb_pd::SinkStatus s = sink.status();
        switch (e) {
            case usb_pd::SinkEvent::SourceCapabilitiesChanged: {
                auto b = RTOS_LOG_BLOCK(Info);
                b.add("USB-PD: %u source capabilities", static_cast<unsigned>(s.capability_count));
                for (std::size_t i = 0; i < s.capability_count; ++i) {
                    const auto& c = s.capabilities[i];
                    b.add("  [%u] %-8s %5u-%5u mV, max %4u mA", static_cast<unsigned>(i), usb_pd::supply_type_name(c.type),
                          c.min_mv, c.max_mv, c.max_ma);
                }
                break;
            }
            case usb_pd::SinkEvent::VoltageChanged:
                RTOS_LOG_INFO("USB-PD: supply now %d mV / %d mA", s.active_mv, s.active_ma);
                break;
            case usb_pd::SinkEvent::PowerRejected: RTOS_LOG_WARN("USB-PD: request rejected by the source"); break;
        }
    });
}

std::optional<Status> status() noexcept {
    const usb_pd::SinkStatus s = sink.status();
    return Status{.connected = s.connected, .active_mv = s.active_mv, .active_ma = s.active_ma};
}

}  // namespace fcu::pd

extern "C" void UCPD1_IRQHandler() noexcept { fcu::pd::ucpd.on_irq(fcu::pd::sink); }

extern "C" void TIM5_IRQHandler() noexcept {
    if (fcu::hw::PdClock::take()) {
        fcu::pd::sink.on_timer();
    }
}
