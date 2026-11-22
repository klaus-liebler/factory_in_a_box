// The FCU's peripherals and external chips, every configuration a compile-time value
// (rtoos: dividers, timings and calibrations are computed and checked by the compiler).
//
// What differs between the board revisions - the MCU, the pins, which I2C/U(S)ART/DMA instance
// does what, the USB-PD controller, the interrupt vector names - is the revision's HAL
// (src/hal/<revision>/: mcu.h, pins.hpp, peripherals.hpp; hal_types.hpp). This file is the same
// for every revision:
//
//   I2C1        colour sensor TCS34725 (0x29), ToF 1 VL53L0X (0x30)          100 kHz
//   ToF 2 bus   ToF 2 VL53L0X (0x29)                                         100 kHz
//   supply bus  INA226 (0x40), ToF 3 VL53L0X (0x29)                          100 kHz
//   SPI2        HX711 load cell (MOSI clocks, MISO data)                       781 kHz
//   UART5       TMC2209 x2 (single wire with echo, nodes 0b10, 0b00)           115200
//   TIM4        conveyor (CH3) and compressor (CH4) PWM                        1 kHz
//   TIM15       WS2812 strip (CH1, 8 LEDs) and single LED (CH2), DMA           800 kHz
//   TIM2        stepper 1 (sorter), TIM3 stepper 2 (elevator): STEP = compare output, 1 MHz
//               ticks, periods by DMA (rtos TimerStepper: no interrupt per step)
//   ADC1        pressure sensor (INP18), FDCAN1 500 kbit/s
//   SDMMC1      SD card: the RoArm missions (FAT, FileX)
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>

#include "board/chip.hpp"
#include "board/fcu.hpp"
#include "hal_types.hpp"
#include "peripherals.hpp"  // the revision's HAL: mcu.h, pins.hpp, peripherals.hpp
#include "rtos/devices/hx711.hpp"
#include "rtos/devices/ina226.hpp"
#include "rtos/devices/tcs34725.hpp"
#include "rtos/devices/vl53l0x.hpp"
#include "rtos/devices/ws2812.hpp"
#include "rtos/distinct.hpp"
#include "rtos/motion/timer_stepper.hpp"
#include "system.hpp"

namespace fcu::hw {

namespace dev = rtos::devices;

// ---- buses ------------------------------------------------------------------------------------
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
/// The pressure sensor's last conversion, 12 bits (0..4095) on every chip.
inline std::uint16_t pressure_raw() noexcept { return static_cast<std::uint16_t>(Pressure::read() >> pressure_shift); }
using Can = chip::Fdcan<{.kernel_hz = board::pll1_q_hz, .cpu_hz = board::sysclk_hz, .bitrate = 500'000}>;

// ---- GPIO edge interrupts (EXTI) -----------------------------------------------------------------
/// HX711 DOUT low: a conversion is ready (the reading then runs by SPI interrupt alone).
using ScaleReady = chip::ExtiLine<pins::hx711_dout, chip::ExtiEdge::Falling>;
/// INA226 ALERT low: a measurement is ready.
using SupplyAlert = chip::ExtiLine<pins::current_alert, chip::ExtiEdge::Falling>;
/// Light barriers: every interruption is latched, however short (both edges counted).
using Barrier1 = chip::ExtiLine<pins::light_barrier1, chip::ExtiEdge::Both>;
using Barrier2 = chip::ExtiLine<pins::light_barrier2, chip::ExtiEdge::Both>;
/// Only where pins::light_barrier3 exists (the fallback pin is never used).
using Barrier3 = chip::ExtiLine<pins::light_barrier3.value_or(pins::light_barrier2), chip::ExtiEdge::Both>;
inline constexpr bool has_barrier3 = pins::light_barrier3.has_value();
/// TMC2209 DIAG rising: a stall during the homing (only where both DIAG pins exist).
using Diag1 = chip::ExtiLine<pins::stepper1_diag.value_or(pins::light_barrier1), chip::ExtiEdge::Rising>;
using Diag2 = chip::ExtiLine<pins::stepper2_diag.value_or(pins::light_barrier2), chip::ExtiEdge::Rising>;
inline constexpr bool has_stepper_diag = pins::stepper1_diag.has_value() && pins::stepper2_diag.has_value();
/// LAN9253 IRQ low: an AL event is pending (only where the line is wired; the fallback pin is never used).
inline constexpr bool has_ethercat_irq = pins::ethercat.has_value() && pins::ethercat_irq_wired;
using EscIrq = chip::ExtiLine<has_ethercat_irq ? pins::ethercat->irq : pins::light_barrier1, chip::ExtiEdge::Falling>;

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
/// The speed profile is chosen at run time (the stepper test bench of the web UI, io.cpp).
using Stepper1 = rtos::motion::ProfileTimerStepper<Stepper1Timer, 1, Stepper1Dma>;
using Stepper2 = rtos::motion::ProfileTimerStepper<Stepper2Timer, 1, Stepper2Dma>;

/// WS2812 at 800 kHz: TIM15 counts at the timer clock, one period per bit; the compare values
/// from the period the timer really has.
using LedTimer = chip::Timer<chip::TimerId::Tim15, {.kernel_hz = clocks.timer2_hz(), .update_hz = 800'000}>;
inline constexpr dev::Ws2812Timing ws2812{LedTimer::period_ticks, static_cast<std::uint16_t>(LedTimer::period_ticks * 35 / 100),
                                          static_cast<std::uint16_t>(LedTimer::period_ticks * 70 / 100)};

// ---- resources: nothing used twice (rtos/distinct.hpp, rtoos docs/integration.md) ----------------
// Every pin, EXTI line, timer, DMA channel, U(S)ART, I2C bus and I2C address of the firmware, the
// board package's and the revision's included; one owner each. What a revision does not have is
// left out (Barrier3, DIAG: the optional pins).
namespace resources {
using hal::join;

inline constexpr auto exti_lines = [] {
    constexpr std::array common{ScaleReady::line, SupplyAlert::line, Barrier1::line, Barrier2::line};
    constexpr auto barrier3 = [] {
        if constexpr (has_barrier3) return std::array{Barrier3::line};
        else return std::array<unsigned, 0>{};
    }();
    constexpr auto diag = [] {
        if constexpr (has_stepper_diag) return std::array{Diag1::line, Diag2::line};
        else return std::array<unsigned, 0>{};
    }();
    constexpr auto esc = [] {
        if constexpr (has_ethercat_irq) return std::array{EscIrq::line};
        else return std::array<unsigned, 0>{};
    }();
    return join(common, barrier3, diag, esc);
}();
static_assert(rtos::distinct(exti_lines), "two EXTI interrupts on one line (one pin per line number)");

/// The IRQ numbers of the EXTI vectors the revision's HAL routes to the I/O code (FCU_EXTI_HANDLERS).
#define FCU_EXTI_IRQ_NUMBER(handler, irq) irq,
inline constexpr std::array exti_vector_irqs{FCU_EXTI_HANDLERS(FCU_EXTI_IRQ_NUMBER)};
#undef FCU_EXTI_IRQ_NUMBER
consteval bool exti_vector_routed(unsigned irq) {
    return std::find(exti_vector_irqs.begin(), exti_vector_irqs.end(), irq) != exti_vector_irqs.end();
}
static_assert(exti_vector_routed(ScaleReady::irq) && exti_vector_routed(SupplyAlert::irq) &&
                  exti_vector_routed(Barrier1::irq) && exti_vector_routed(Barrier2::irq) &&
                  (!has_barrier3 || exti_vector_routed(Barrier3::irq)) &&
                  (!has_stepper_diag || (exti_vector_routed(Diag1::irq) && exti_vector_routed(Diag2::irq))) &&
                  (!has_ethercat_irq || exti_vector_routed(EscIrq::irq)),
              "an EXTI line without its vector in the revision's FCU_EXTI_HANDLERS (peripherals.hpp)");

inline constexpr auto timers =
    join(std::array{MotorPwm::base, Stepper1Timer::base, Stepper2Timer::base, LedTimer::base}, revision_timers);
static_assert(rtos::distinct(timers), "a timer used twice");

inline constexpr auto dma_channels =
    join(std::array{LedStripDma::id, LedSingleDma::id, Stepper1Dma::id, Stepper2Dma::id}, revision_dma_channels);
static_assert(rtos::distinct(dma_channels), "a DMA channel used twice");

static_assert(rtos::distinct(join(std::array{board::Console::base, ServoUart::base, StepperUart::base}, revision_uarts)),
              "a U(S)ART used twice");
static_assert(rtos::distinct(std::array{I2c1::base, I2cTof2::base, I2cSupply::base}), "an I2C bus used twice");

using BusAddress = std::pair<std::uintptr_t, std::uint8_t>;
template <class Device>
inline constexpr BusAddress at{Device::bus::base, Device::address};
static_assert(rtos::distinct(std::array{at<Supply>, at<Tof1>, at<Tof2>, at<Tof3>, at<Color>}), "two I2C devices at one address");

static_assert(chip::gpio::pins_distinct(board_package_pins, pins::supply_pins, pins::io_pins), "a pin configured twice");
}  // namespace resources

}  // namespace fcu::hw
