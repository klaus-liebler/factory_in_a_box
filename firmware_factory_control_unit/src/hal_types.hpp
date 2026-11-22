// Building blocks of the board revisions' HAL layers (src/hal/<revision>/): what every revision
// describes the same way. A revision's HAL (selected by the CMake preset, its name = the
// directory name) consists of
//
//   mcu.h            the MCU's peripheral headers (rtoos port) and the chip facts read directly
//   pins.hpp         the I/O pins of the revision (fcu::pins)
//   peripherals.hpp  buses, U(S)ARTs, DMA channels, ADC, the USB-PD controller, the interrupt
//                    vector names of the firmware's handlers (fcu::hw)
//   *.cpp            what only that revision has (USB-PD by the MCU's UCPD on revision 1)
//   hal.cmake        chip, revision number for rtoos's board package, the revision's sources
//
// Common code includes "hardware.hpp" (which includes the three headers) and asks the HAL what is
// there (std::optional pins, constexpr flags) - no preprocessor switch on the revision.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "board/chip.hpp"
#include "rtos/rtos.hpp"
#include "system.hpp"

namespace fcu {

namespace hal {
/// The elements of `a` and then those of `b` (pin lists, resource lists).
template <class T, std::size_t A, std::size_t B>
[[nodiscard]] constexpr std::array<T, A + B> join(const std::array<T, A>& a, const std::array<T, B>& b) {
    std::array<T, A + B> r{};
    for (std::size_t i = 0; i < A; ++i) r[i] = a[i];
    for (std::size_t i = 0; i < B; ++i) r[A + i] = b[i];
    return r;
}
template <class T, std::size_t A, class... More>
[[nodiscard]] constexpr auto join(const std::array<T, A>& a, const More&... more) {
    if constexpr (sizeof...(More) == 0) {
        return a;
    } else {
        return join(a, join(more...));
    }
}
}  // namespace hal

namespace pins {
namespace gpio = board::chip::gpio;
using gpio::GpioPort;
using gpio::Pin;
using S = gpio::Signal;

/// The LAN9253's host interface (SPI/SQI) and its other lines. src/ethercat.cpp drives SPI by GPIO
/// (SIO0 = SI, SIO1 = SO; SIO2/SIO3 held high), no OCTOSPI/QUADSPI driver yet.
struct EthercatPins {
    Pin sck, cs, sio0, sio1, sio2, sio3, irq, sync0, sync1;
};
/// Their configuration: select inactive, clock low, SYNC/IRQ inputs (IRQ: open drain, pulled up;
/// used where pins::ethercat_irq_wired, else the EtherCAT thread polls).
[[nodiscard]] constexpr std::array<gpio::PinConfig, 9> ethercat_pin_config(const EthercatPins& e) {
    return {gpio::output(e.cs, /*high = deselected*/ true, gpio::Speed::High), gpio::output(e.sck, false, gpio::Speed::High),
            gpio::output(e.sio0, false, gpio::Speed::High), gpio::input(e.sio1, gpio::Pull::Up),
            gpio::output(e.sio2, true), gpio::output(e.sio3, true), gpio::input(e.irq, gpio::Pull::Up),
            gpio::input(e.sync0, gpio::Pull::Down), gpio::input(e.sync1, gpio::Pull::Down)};
}
}  // namespace pins

namespace hw {
namespace chip = board::chip;
inline constexpr auto& clocks = board::clocks;

/// Interrupt-driven I2C: the calling thread sleeps on a semaphore while the event interrupt moves
/// the bytes (one per bus; the interrupts at irq_priority_net, kernel-aware).
template <unsigned Bus>
struct I2cWait {
    static inline rtos::Semaphore<Kernel> done{kernel, 0, 1};
    static void reset() noexcept {
        while (done.take(rtos::no_wait)) {
        }
    }
    static bool wait(std::uint32_t timeout_us) noexcept { return done.take(timeout_us / 1000 + 2); }  // 1 ms ticks
    static void signal() noexcept { (void)done.give_from_isr(); }
};
}  // namespace hw

}  // namespace fcu
