// The FCU's peripherals and external chips, every configuration a compile-time value
// (rtoos: dividers, timings and calibrations are computed and checked by the compiler).
//
//   I2C1        colour sensor TCS34725 (0x29), ToF 1 VL53L0X (0x30)          100 kHz
//   ToF 2 bus   ToF 2 VL53L0X (0x29): I2C2 (rev. 1/2), I2C4 (rev. 3)         100 kHz
//   supply bus  INA226 (0x40), ToF 3 VL53L0X (0x29): I2C4 (rev. 1/2),        100 kHz
//               I2C3 (rev. 3, also the CH224Q - not queried)
//   SPI2        HX711 load cell (MOSI clocks, MISO data)                       781 kHz
//   UART5       TMC2209 x2 (single wire with echo, nodes 0b10, 0b00)           115200
//   servo bus   Feetech servos of the RoArm-M3: UART7 (rev. 1/2), USART1 (3)  1 Mbit/s
//   TIM4        conveyor (CH3) and compressor (CH4) PWM                        1 kHz
//   TIM15       WS2812 strip (CH1, 8 LEDs) and single LED (CH2), DMA           800 kHz
//   TIM2        stepper 1 (sorter), TIM3 stepper 2 (elevator): STEP = compare output, 1 MHz
//               ticks, periods by DMA (rtos TimerStepper: no interrupt per step)
//   TIM5        USB-PD deadlines (1 MHz, 32 bit; revision 1)
//   ADC1        pressure sensor (INP18), FDCAN1 500 kbit/s, CPU temperature (DTS on the
//               STM32H563, ADC3's sensor on the STM32H743)
//   SDMMC1      SD card: the RoArm missions (FAT, FileX)
#pragma once

#include <array>
#include <cstddef>
#include <utility>

#include "board/chip.hpp"
#include "board/fcu.hpp"
#include "rtos/devices/hx711.hpp"
#include "rtos/devices/ina226.hpp"
#include "rtos/devices/tcs34725.hpp"
#include "rtos/devices/vl53l0x.hpp"
#include "rtos/devices/ws2812.hpp"
#include "rtos/distinct.hpp"
#include "rtos/motion/timer_stepper.hpp"
#if RTOS_FCU_REVISION == 3
#include "rtos/port/stm32h7/adc.hpp"
#include "rtos/port/stm32h7/dma.hpp"
#include "rtos/port/stm32h7/exti.hpp"
#include "rtos/port/stm32h7/fdcan.hpp"
#include "rtos/port/stm32h7/i2c.hpp"
#include "rtos/port/stm32h7/sdmmc.hpp"
#include "rtos/port/stm32h7/spi.hpp"
#include "rtos/port/stm32h7/timer.hpp"
#include "rtos/port/stm32h7/usart.hpp"
#else
#include "rtos/port/stm32h5/adc.hpp"
#include "rtos/port/stm32h5/dts.hpp"
#include "rtos/port/stm32h5/exti.hpp"
#include "rtos/port/stm32h5/fdcan.hpp"
#include "rtos/port/stm32h5/gpdma.hpp"
#include "rtos/port/stm32h5/i2c.hpp"
#include "rtos/port/stm32h5/sdmmc.hpp"
#include "rtos/port/stm32h5/spi.hpp"
#include "rtos/port/stm32h5/timer.hpp"
#include "rtos/port/stm32h5/ucpd.hpp"
#include "rtos/port/stm32h5/usart.hpp"
#endif
#include "pins.hpp"
#include "system.hpp"

namespace fcu::hw {

namespace chip = board::chip;
namespace dev = rtos::devices;
inline constexpr auto& clocks = board::clocks;

// ---- buses ------------------------------------------------------------------------------------
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

#if RTOS_FCU_REVISION == 3
inline constexpr unsigned i2c_tof2_bus = 4, i2c_supply_bus = 3;  ///< the buses' numbers (logs, terminal)
using I2c1 = chip::I2cMaster<chip::I2cId::I2c1, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<1>>;
using I2cTof2 = chip::I2cMaster<chip::I2cId::I2c4, {.kernel_hz = clocks.pclk4_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<4>>;
using I2cSupply = chip::I2cMaster<chip::I2cId::I2c3, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<3>>;
using ServoUart = chip::BufferedUart<chip::UsartId::Usart1, {.kernel_hz = clocks.pclk2_hz(), .baud = 1'000'000}, 256, 256>;
#else
inline constexpr unsigned i2c_tof2_bus = 2, i2c_supply_bus = 4;
using I2c1 = chip::I2cMaster<chip::I2cId::I2c1, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<1>>;
using I2cTof2 = chip::I2cMaster<chip::I2cId::I2c2, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<2>>;
using I2cSupply = chip::I2cMaster<chip::I2cId::I2c4, {.kernel_hz = clocks.pclk3_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<4>>;
using ServoUart = chip::BufferedUart<chip::UsartId::Uart7, {.kernel_hz = clocks.pclk1_hz(), .baud = 1'000'000}, 256, 256>;
#endif

/// The bus's event and error interrupts on (after init()).
template <class Bus>
void enable_i2c_irqs() noexcept {
    enable_irq(Bus::irq_event, irq_priority_net);
    enable_irq(Bus::irq_error, irq_priority_net);
}
using Spi2 = chip::SpiMaster<chip::SpiId::Spi2, {.kernel_hz = board::pll1_q_hz, .max_bus_hz = 1'000'000,
                                                 .cpu_hz = board::sysclk_hz}>;
/// SD card slot (J3): SDMMC1, 1-bit bus (board::init_sdcard()); the RoArm missions live there.
using SdHost = chip::Sdmmc<chip::SdmmcId::Sdmmc1, {.kernel_hz = board::pll1_q_hz, .cpu_hz = board::sysclk_hz}>;
using StepperUart = chip::BufferedUart<chip::UsartId::Uart5, {.kernel_hz = clocks.pclk1_hz(), .baud = 115'200}, 64, 64>;

// ---- sensors ------------------------------------------------------------------------------------
/// 10 mOhm shunt, up to 8 A; 16 averages of 1.1 ms bus + 1.1 ms shunt: a measurement every
/// 35.2 ms, announced on ALERT (conversion ready).
using Supply = dev::Ina226<I2cSupply, {.shunt_micro_ohm = 10'000, .max_current_ma = 8'000, .config = 0x4527}>;
using Tof1 = dev::Vl53l0x<I2c1, 0x30>;  ///< address changed from 0x29 (the colour sensor has it)
using Tof2 = dev::Vl53l0x<I2cTof2>;
using Tof3 = dev::Vl53l0x<I2cSupply>;
/// The former firmware's setting: ATIME 0xC0 (153.6 ms), gain 4.
using Color = dev::Tcs34725<I2c1, {.integration_ms = 154, .gain = dev::Tcs34725Gain::X4}>;
inline bool scale_ready() noexcept { return !chip::gpio::read(pins::hx711_dout); }  // DOUT low: conversion ready
using Scale = dev::Hx711<Spi2, &scale_ready, dev::Hx711Gain::A128, 8>;

// ---- GPIO edge interrupts (EXTI) -----------------------------------------------------------------
/// HX711 DOUT low: a conversion is ready (the reading then runs by SPI interrupt alone).
using ScaleReady = chip::ExtiLine<pins::hx711_dout, chip::ExtiEdge::Falling>;
/// INA226 ALERT low: a measurement is ready.
using SupplyAlert = chip::ExtiLine<pins::current_alert, chip::ExtiEdge::Falling>;
/// Light barriers: every interruption is latched, however short (both edges counted).
using Barrier1 = chip::ExtiLine<pins::light_barrier1, chip::ExtiEdge::Both>;
using Barrier2 = chip::ExtiLine<pins::light_barrier2, chip::ExtiEdge::Both>;
using Barrier3 = chip::ExtiLine<pins::light_barrier3.value_or(pins::light_barrier2), chip::ExtiEdge::Both>;  ///< revision 1
/// TMC2209 DIAG rising: a stall during the homing (pins::stepper_diag; the fallback pins are never used).
using Diag1 = chip::ExtiLine<pins::stepper1_diag.value_or(pins::light_barrier1), chip::ExtiEdge::Rising>;
using Diag2 = chip::ExtiLine<pins::stepper2_diag.value_or(pins::light_barrier2), chip::ExtiEdge::Rising>;

#if RTOS_FCU_REVISION == 3
using Pressure = chip::Adc<chip::AdcId::Adc1, {.cpu_hz = board::sysclk_hz, .channel = 18}>;
/// The STM32H743's ADC converts with 16 bits; the register map keeps the 12 bits of the H563.
inline constexpr unsigned pressure_shift = 4;
using CpuTemperature = chip::CpuTemperature<{.cpu_hz = board::sysclk_hz}>;
#else
using Pressure = chip::Adc<chip::AdcId::Adc1, {.kernel_hz = clocks.hclk_hz(), .cpu_hz = board::sysclk_hz, .channel = 18,
                                               .sample = chip::AdcSample::Cycles2_5}>;
inline constexpr unsigned pressure_shift = 0;
using CpuTemperature = chip::Dts<{.pclk_hz = clocks.pclk1_hz()}>;
#endif
/// The pressure sensor's last conversion, 12 bits (0..4095) on every chip.
inline std::uint16_t pressure_raw() noexcept { return static_cast<std::uint16_t>(Pressure::read() >> pressure_shift); }
using Can = chip::Fdcan<{.kernel_hz = board::pll1_q_hz, .cpu_hz = board::sysclk_hz, .bitrate = 500'000}>;

// ---- actuators ----------------------------------------------------------------------------------
/// Conveyor and compressor: 1 kHz PWM (requirement: at most 1 kHz), duty in 1/1000.
using MotorPwm = chip::Timer<chip::TimerId::Tim4, {.kernel_hz = clocks.timer1_hz(), .update_hz = 1'000, .min_steps = 1000}>;

/// 100 -> 1000 steps/s in 0.5 s (the FCU's sorter and elevator; rtoos motion_test).
inline constexpr rtos::motion::SigmoidRamp stepper_ramp{.tick_hz = 1'000'000, .start_hz = 100, .cruise_hz = 1000,
                                                        .ramp_us = 500'000};
/// The timers make every STEP edge (toggle on CCR, TMC2209 DEDGE: one microstep per edge), the
/// DMA writes each step's period into ARR; STEP 1 = PA15 (TIM2_CH1), STEP 2 = PB4 (TIM3_CH1).
using Stepper1Timer = chip::Timer<chip::TimerId::Tim2, {.kernel_hz = clocks.timer1_hz(), .tick_hz = 1'000'000}>;
using Stepper2Timer = chip::Timer<chip::TimerId::Tim3, {.kernel_hz = clocks.timer1_hz(), .tick_hz = 1'000'000}>;

/// WS2812 at 800 kHz: TIM15 counts at the timer clock, one period per bit; the compare values
/// from the period the timer really has.
using LedTimer = chip::Timer<chip::TimerId::Tim15, {.kernel_hz = clocks.timer2_hz(), .update_hz = 800'000}>;
inline constexpr dev::Ws2812Timing ws2812{LedTimer::period_ticks, static_cast<std::uint16_t>(LedTimer::period_ticks * 35 / 100),
                                          static_cast<std::uint16_t>(LedTimer::period_ticks * 70 / 100)};

#if RTOS_FCU_REVISION == 3
// DMA1 streams 0-3 (DMAMUX1). The STM32H743's DMA does not widen: memory and peripheral width
// are equal - the steppers' periods are 32-bit items (ARR is written as a 32-bit register; TIM3's
// upper 16 bits are reserved and get 0).
/// CH1: compare DMA request; CH2 has none of its own: the update request writes CCR2 through DMAR.
using LedStripDma = chip::DmaChannel<{.controller = 1, .stream = 0, .request = chip::DmaRequest::Tim15Ch1,
                                      .direction = chip::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                                      .peripheral_bytes = 2}>;
using LedSingleDma = chip::DmaChannel<{.controller = 1, .stream = 1, .request = chip::DmaRequest::Tim15Up,
                                       .direction = chip::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                                       .peripheral_bytes = 2}>;
using Stepper1Dma = chip::DmaChannel<{.controller = 1, .stream = 2, .request = chip::DmaRequest::Tim2Up,
                                      .direction = chip::DmaDirection::MemoryToPeripheral, .memory_bytes = 4,
                                      .peripheral_bytes = 4, .priority = 3}>;
using Stepper2Dma = chip::DmaChannel<{.controller = 1, .stream = 3, .request = chip::DmaRequest::Tim3Up,
                                      .direction = chip::DmaDirection::MemoryToPeripheral, .memory_bytes = 4,
                                      .peripheral_bytes = 4, .priority = 3}>;
/// TIM15: DMAR -> CCR2 at each update (DCR: base CCR2, one transfer).
inline void configure_led_burst() noexcept { LedTimer::configure_dma_burst(0x38, 1); }

/// PC2_C/PC3_C (HX711 on SPI2): the pads reach the digital PC2/PC3 through analog switches,
/// SYSCFG_PMCR.PC2SO/PC3SO = 0 closes them (ST HAL: SYSCFG_SWITCH_PC2_CLOSE = 0). The SVD's
/// reset value is 0 already; written anyway, so the HX711 never depends on it.
inline void close_analog_switches() noexcept {
    using PMCR = chip::Reg<0x5800'0400 + 0x04>;  // SYSCFG base (stm32h743xx.h) + PMCR
    chip::rcc::enable(chip::rcc::gate::syscfg);
    PMCR::write(PMCR::read() & ~((1u << 26) | (1u << 27)));  // PC2SO, PC3SO
}
#else
// GPDMA1 channels 2-5 (0 and 1: UCPD). Memory items of 16 bits widened to the 32-bit ARR.
using LedStripDma = chip::DmaChannel<{.channel = 2, .request = chip::DmaRequest::Tim15Ch1,
                                      .direction = chip::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                                      .peripheral_bytes = 2}>;
using LedSingleDma = chip::DmaChannel<{.channel = 3, .request = chip::DmaRequest::Tim15Up,
                                       .direction = chip::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                                       .peripheral_bytes = 2}>;
using Stepper1Dma = chip::DmaChannel<{.channel = 4, .request = chip::DmaRequest::Tim2Up,
                                      .direction = chip::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                                      .peripheral_bytes = 4, .priority = 3}>;
using Stepper2Dma = chip::DmaChannel<{.channel = 5, .request = chip::DmaRequest::Tim3Up,
                                      .direction = chip::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                                      .peripheral_bytes = 4, .priority = 3}>;
/// TIM15: DMAR -> CCR2 at each update (DCR: base CCR2, one transfer, update source).
inline void configure_led_burst() noexcept { LedTimer::configure_dma_burst(0x38, 1, 1); }
inline void close_analog_switches() noexcept {}  // the STM32H563 has none

// ---- USB Power Delivery (revision 1) ---------------------------------------------------------------
using PdClock = chip::MicrosecondTimer<chip::TimerId::Tim5, clocks.timer1_hz()>;
using Ucpd = chip::Ucpd<{.kernel_hz = clocks.ucpd_hz()}>;  // GPDMA channels 0 (RX), 1 (TX)
#endif
using Stepper1 = rtos::motion::TimerStepper<Stepper1Timer, 1, Stepper1Dma, stepper_ramp>;
using Stepper2 = rtos::motion::TimerStepper<Stepper2Timer, 1, Stepper2Dma, stepper_ramp>;

// ---- resources: nothing used twice (rtos/distinct.hpp, rtoos docs/integration.md) ----------------
// Every pin, EXTI line, timer, DMA channel, U(S)ART, I2C bus and I2C address of the firmware, the
// board package's included; one owner each. What a revision does not have is left out (Barrier3:
// revision 1; DIAG: revisions 2/3; UCPD with its clock and DMA channels: revision 1).
namespace resources {
template <class T, std::size_t A, std::size_t B>
consteval std::array<T, A + B> join(const std::array<T, A>& a, const std::array<T, B>& b) {
    std::array<T, A + B> r{};
    for (std::size_t i = 0; i < A; ++i) r[i] = a[i];
    for (std::size_t i = 0; i < B; ++i) r[A + i] = b[i];
    return r;
}

inline constexpr auto exti_lines = [] {
    constexpr std::array common{ScaleReady::line, SupplyAlert::line, Barrier1::line, Barrier2::line};
    constexpr auto barrier3 = [] {
        if constexpr (pins::light_barrier3.has_value()) return std::array{Barrier3::line};
        else return std::array<unsigned, 0>{};
    }();
    constexpr auto diag = [] {
        if constexpr (pins::stepper_diag) return std::array{Diag1::line, Diag2::line};
        else return std::array<unsigned, 0>{};
    }();
    return join(join(common, barrier3), diag);
}();
static_assert(rtos::distinct(exti_lines), "two EXTI interrupts on one line (one pin per line number)");

inline constexpr auto timers = [] {
    constexpr std::array common{MotorPwm::base, Stepper1Timer::base, Stepper2Timer::base, LedTimer::base};
#if RTOS_FCU_REVISION != 3
    if constexpr (board::has_ucpd) return join(common, std::array{PdClock::base});
    else return common;
#else
    return common;
#endif
}();
static_assert(rtos::distinct(timers), "a timer used twice");

inline constexpr auto dma_channels = [] {
    constexpr std::array common{LedStripDma::id, LedSingleDma::id, Stepper1Dma::id, Stepper2Dma::id};
#if RTOS_FCU_REVISION != 3
    if constexpr (board::has_ucpd) return join(common, std::array{Ucpd::rx_dma::id, Ucpd::tx_dma::id});
    else return common;
#else
    return common;
#endif
}();
static_assert(rtos::distinct(dma_channels), "a DMA channel used twice");

static_assert(rtos::distinct(std::array{board::Console::base, ServoUart::base, StepperUart::base}), "a U(S)ART used twice");
static_assert(rtos::distinct(std::array{I2c1::base, I2cTof2::base, I2cSupply::base}), "an I2C bus used twice");

using BusAddress = std::pair<std::uintptr_t, std::uint8_t>;
template <class Device>
inline constexpr BusAddress at{Device::bus::base, Device::address};
static_assert(rtos::distinct(std::array{at<Supply>, at<Tof1>, at<Tof2>, at<Tof3>, at<Color>}), "two I2C devices at one address");

#if RTOS_FCU_REVISION == 3
static_assert(chip::gpio::pins_distinct(board::pins::base, board::pins::ethernet, board::pins::sdcard, board::pins::usb,
                                        pins::supply_pins, pins::io_pins),
              "a pin configured twice");
#else
static_assert(chip::gpio::pins_distinct(board::pins::base, board::pins::ethernet, board::pins::sdcard, pins::supply_pins,
                                        pins::io_pins),
              "a pin configured twice");
#endif
}  // namespace resources

}  // namespace fcu::hw
