// The FCU's peripherals and external chips, every configuration a compile-time value
// (rtoos: dividers, timings and calibrations are computed and checked by the compiler).
//
//   I2C1  colour sensor TCS34725 (0x29), ToF 1 VL53L0X (0x30)     100 kHz
//   I2C2  ToF 2 VL53L0X (0x29)                                     100 kHz
//   I2C4  INA226 supply monitor (0x40), ToF 3 VL53L0X (0x29)        100 kHz
//   SPI2  HX711 load cell (MOSI clocks, MISO data)                  781 kHz
//   UART5 TMC2209 x2 (single wire with echo, nodes 0b10, 0b00)      115200
//   UART7 Feetech servo bus of the RoArm-M3                         1 Mbit/s
//   TIM4  conveyor (CH3) and compressor (CH4) PWM                   1 kHz
//   TIM15 WS2812 strip (CH1, 8 LEDs) and single LED (CH2), DMA      800 kHz
//   TIM2  stepper 1 (sorter), TIM3 stepper 2 (elevator): STEP = compare output, 1 MHz ticks,
//         periods by GPDMA1 channels 4 and 5 (rtos TimerStepper: no interrupt per step)
//   TIM5  USB-PD deadlines (1 MHz, 32 bit)
//   ADC1  pressure sensor (INP18), FDCAN1 500 kbit/s, DTS CPU temperature
#pragma once

#include "board/fcu.hpp"
#include "rtos/devices/hx711.hpp"
#include "rtos/devices/ina226.hpp"
#include "rtos/devices/tcs34725.hpp"
#include "rtos/devices/vl53l0x.hpp"
#include "rtos/devices/ws2812.hpp"
#include "rtos/motion/timer_stepper.hpp"
#include "rtos/port/stm32h5/adc.hpp"
#include "rtos/port/stm32h5/dts.hpp"
#include "rtos/port/stm32h5/fdcan.hpp"
#include "rtos/port/stm32h5/gpdma.hpp"
#include "rtos/port/stm32h5/i2c.hpp"
#include "rtos/port/stm32h5/spi.hpp"
#include "rtos/port/stm32h5/timer.hpp"
#include "rtos/port/stm32h5/ucpd.hpp"
#include "rtos/port/stm32h5/usart.hpp"
#include "pins.hpp"

namespace fcu::hw {

namespace h5 = rtos::port::stm32h5;
namespace dev = rtos::devices;
inline constexpr auto& clocks = board::clocks;

// ---- buses ------------------------------------------------------------------------------------
using I2c1 = h5::I2cMaster<h5::I2cId::I2c1, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz}>;
using I2c2 = h5::I2cMaster<h5::I2cId::I2c2, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz}>;
using I2c4 = h5::I2cMaster<h5::I2cId::I2c4, {.kernel_hz = clocks.pclk3_hz(), .cpu_hz = board::sysclk_hz}>;
using Spi2 = h5::SpiMaster<h5::SpiId::Spi2, {.kernel_hz = board::pll1_q_hz, .max_bus_hz = 1'000'000,
                                             .cpu_hz = board::sysclk_hz}>;
using StepperUart = h5::BufferedUart<h5::UsartId::Uart5, {.kernel_hz = clocks.pclk1_hz(), .baud = 115'200}, 64, 64>;
using ServoUart = h5::BufferedUart<h5::UsartId::Uart7, {.kernel_hz = clocks.pclk1_hz(), .baud = 1'000'000}, 256, 256>;

// ---- sensors ------------------------------------------------------------------------------------
/// 10 mOhm shunt, up to 8 A.
using Supply = dev::Ina226<I2c4, {.shunt_micro_ohm = 10'000, .max_current_ma = 8'000}>;
using Tof1 = dev::Vl53l0x<I2c1, 0x30>;  ///< address changed from 0x29 (the colour sensor has it)
using Tof2 = dev::Vl53l0x<I2c2>;
using Tof3 = dev::Vl53l0x<I2c4>;
/// The former firmware's setting: ATIME 0xC0 (153.6 ms), gain 4.
using Color = dev::Tcs34725<I2c1, {.integration_ms = 154, .gain = dev::Tcs34725Gain::X4}>;
inline bool scale_ready() noexcept { return !h5::gpio::read(pins::hx711_dout); }  // DOUT low: conversion ready
using Scale = dev::Hx711<Spi2, &scale_ready, dev::Hx711Gain::A128, 8>;
using Pressure = h5::Adc<h5::AdcId::Adc1, {.kernel_hz = clocks.hclk_hz(), .cpu_hz = board::sysclk_hz, .channel = 18,
                                           .sample = h5::AdcSample::Cycles2_5}>;
using CpuTemperature = h5::Dts<{.pclk_hz = clocks.pclk1_hz()}>;
using Can = h5::Fdcan<{.kernel_hz = board::pll1_q_hz, .cpu_hz = board::sysclk_hz, .bitrate = 500'000}>;

// ---- actuators ----------------------------------------------------------------------------------
/// Conveyor and compressor: 1 kHz PWM (requirement: at most 1 kHz), duty in 1/1000.
using MotorPwm = h5::Timer<h5::TimerId::Tim4, {.kernel_hz = clocks.timer1_hz(), .update_hz = 1'000, .min_steps = 1000}>;

/// 100 -> 1000 steps/s in 0.5 s (the FCU's sorter and elevator; rtoos motion_test).
inline constexpr rtos::motion::SigmoidRamp stepper_ramp{.tick_hz = 1'000'000, .start_hz = 100, .cruise_hz = 1000,
                                                        .ramp_us = 500'000};
/// The timers make every STEP edge (toggle on CCR, TMC2209 DEDGE: one microstep per edge), the
/// GPDMA writes each step's period into ARR; STEP 1 = PA15 (TIM2_CH1), STEP 2 = PB4 (TIM3_CH1).
using Stepper1Timer = h5::Timer<h5::TimerId::Tim2, {.kernel_hz = clocks.timer1_hz(), .tick_hz = 1'000'000}>;
using Stepper2Timer = h5::Timer<h5::TimerId::Tim3, {.kernel_hz = clocks.timer1_hz(), .tick_hz = 1'000'000}>;
using Stepper1Dma = h5::DmaChannel<{.channel = 4, .request = h5::DmaRequest::Tim2Up,
                                    .direction = h5::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                                    .peripheral_bytes = 4, .priority = 3}>;
using Stepper2Dma = h5::DmaChannel<{.channel = 5, .request = h5::DmaRequest::Tim3Up,
                                    .direction = h5::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                                    .peripheral_bytes = 4, .priority = 3}>;
using Stepper1 = rtos::motion::TimerStepper<Stepper1Timer, 1, Stepper1Dma, stepper_ramp>;
using Stepper2 = rtos::motion::TimerStepper<Stepper2Timer, 1, Stepper2Dma, stepper_ramp>;

/// WS2812 at 800 kHz: TIM15 counts at the timer clock, one period per bit; the compare values
/// from the period the timer really has.
using LedTimer = h5::Timer<h5::TimerId::Tim15, {.kernel_hz = clocks.timer2_hz(), .update_hz = 800'000}>;
inline constexpr dev::Ws2812Timing ws2812{LedTimer::period_ticks, static_cast<std::uint16_t>(LedTimer::period_ticks * 35 / 100),
                                          static_cast<std::uint16_t>(LedTimer::period_ticks * 70 / 100)};
/// CH1: compare DMA request; CH2 has none of its own: the update request writes CCR2 through DMAR.
using LedStripDma = h5::DmaChannel<{.channel = 2, .request = h5::DmaRequest::Tim15Ch1,
                                    .direction = h5::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                                    .peripheral_bytes = 2}>;
using LedSingleDma = h5::DmaChannel<{.channel = 3, .request = h5::DmaRequest::Tim15Up,
                                     .direction = h5::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                                     .peripheral_bytes = 2}>;

// ---- USB Power Delivery ----------------------------------------------------------------------------
using PdClock = h5::MicrosecondTimer<h5::TimerId::Tim5, clocks.timer1_hz()>;
using Ucpd = h5::Ucpd<{.kernel_hz = clocks.ucpd_hz()}>;  // GPDMA channels 0 (RX), 1 (TX)

}  // namespace fcu::hw
