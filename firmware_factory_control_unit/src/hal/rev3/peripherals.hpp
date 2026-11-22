// Peripherals of FCU revision 3 (STM32H743), every configuration a compile-time value:
//
//   ToF 2 bus   I2C4 (ToF 2 VL53L0X)                                  100 kHz
//   supply bus  I2C3 (INA226, ToF 3 VL53L0X, the CH224Q - not queried) 100 kHz
//   servo bus   USART1 (Feetech servos of the RoArm-M3)               1 Mbit/s
//   Malibotti   USART6 (J16)                                          115200
//   DMA         DMA1 streams 0-3 via DMAMUX1 (LED strip, single LED, steppers 1/2)
//   ADC1        pressure sensor (INP18, 16 bits, shifted to 12); CPU temperature: ADC3's sensor
//   USB-PD      CH224Q (20 V by CFG1): nothing for the MCU to do
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "hal_types.hpp"
#include "mcu.h"
#include "pins.hpp"

// Interrupt vectors of the firmware's handlers (names: the chip's vector table).
#define FCU_I2C_TOF2_EV_IRQ_HANDLER I2C4_EV_IRQHandler
#define FCU_I2C_TOF2_ER_IRQ_HANDLER I2C4_ER_IRQHandler
#define FCU_I2C_SUPPLY_EV_IRQ_HANDLER I2C3_EV_IRQHandler
#define FCU_I2C_SUPPLY_ER_IRQ_HANDLER I2C3_ER_IRQHandler
#define FCU_STEPPER1_DMA_IRQ_HANDLER DMA1_Stream2_IRQHandler
#define FCU_STEPPER2_DMA_IRQ_HANDLER DMA1_Stream3_IRQHandler
#define FCU_SERVO_UART_IRQ_HANDLER USART1_IRQHandler
#define FCU_MALIBOTTI_UART_IRQ_HANDLER USART6_IRQHandler
/// The EXTI vectors the firmware's lines use, with their IRQ numbers (stm32h743xx.h: EXTI0..4 =
/// IRQ 6..10, EXTI9_5 = 23, EXTI15_10 = 40). hardware.hpp checks that every line used has its vector.
#define FCU_EXTI_HANDLERS(X) \
    X(EXTI2_IRQHandler, 8)       /* HX711 DOUT (PC2) */ \
    X(EXTI4_IRQHandler, 10)      /* DIAG 2 (PE4) */ \
    X(EXTI9_5_IRQHandler, 23)    /* DIAG 1 (PD6), INA226 ALERT (PA9) */ \
    X(EXTI15_10_IRQHandler, 40)  /* light barriers 1, 2 (PB10, PD11), LAN9253 IRQ (PC14) */

namespace fcu::hw {

// ---- buses ------------------------------------------------------------------------------------
inline constexpr unsigned i2c_tof2_bus = 4, i2c_supply_bus = 3;  ///< the buses' numbers (logs, terminal)
using I2c1 = chip::I2cMaster<chip::I2cId::I2c1, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<1>>;
using I2cTof2 = chip::I2cMaster<chip::I2cId::I2c4, {.kernel_hz = clocks.pclk4_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<4>>;
using I2cSupply = chip::I2cMaster<chip::I2cId::I2c3, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<3>>;
using ServoUart = chip::BufferedUart<chip::UsartId::Usart1, {.kernel_hz = clocks.pclk2_hz(), .baud = 1'000'000}, 256, 256>;
/// The Malibotti robot (J16, docs/malibotti-uart.md): frames of a few dozen bytes.
using MalibottiUart = chip::BufferedUart<chip::UsartId::Usart6, {.kernel_hz = clocks.pclk2_hz(), .baud = 115'200}, 128, 128>;

// ---- analog -----------------------------------------------------------------------------------
using Pressure = chip::Adc<chip::AdcId::Adc1, {.cpu_hz = board::sysclk_hz, .channel = 18}>;
/// The STM32H743's ADC converts with 16 bits; the register map keeps the 12 bits of the H563.
inline constexpr unsigned pressure_shift = 4;
using CpuTemperature = chip::CpuTemperature<{.cpu_hz = board::sysclk_hz}>;

// ---- DMA: DMA1 streams 0-3 (DMAMUX1) ------------------------------------------------------------
// The STM32H743's DMA does not widen: memory and peripheral width are equal - the steppers'
// periods are 32-bit items (ARR is written as a 32-bit register; TIM3's upper 16 bits are reserved
// and get 0).
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
/// TIM15 (`LedTimer`): DMAR -> CCR2 at each update (DCR: base CCR2, one transfer).
template <class LedTimer>
void configure_led_burst() noexcept {
    LedTimer::configure_dma_burst(0x38, 1);
}

/// PC2_C/PC3_C (HX711 on SPI2): the pads reach the digital PC2/PC3 through analog switches,
/// SYSCFG_PMCR.PC2SO/PC3SO = 0 closes them (ST HAL: SYSCFG_SWITCH_PC2_CLOSE = 0). The SVD's
/// reset value is 0 already; written anyway, so the HX711 never depends on it.
inline void close_analog_switches() noexcept {
    using PMCR = chip::Reg<0x5800'0400 + 0x04>;  // SYSCFG base (stm32h743xx.h) + PMCR
    chip::rcc::enable(chip::rcc::gate::syscfg);
    PMCR::write(PMCR::read() & ~((1u << 26) | (1u << 27)));  // PC2SO, PC3SO
}

// ---- USB Power Delivery -------------------------------------------------------------------------
/// The CH224Q negotiates alone; the firmware sees the result on the INA226 only.
inline constexpr bool mcu_power_delivery = false;
inline constexpr std::string_view pd_controller = "CH224Q";

// ---- what only this revision adds to the resource checks (hardware.hpp) ----------------------------
inline constexpr std::array<std::uintptr_t, 0> revision_timers{};
inline constexpr std::array<unsigned, 0> revision_dma_channels{};
inline constexpr std::array revision_uarts{MalibottiUart::base};
/// Every pin the board package configures (revision 3: the USB OTG pins too).
inline constexpr auto board_package_pins =
    hal::join(board::pins::base, board::pins::ethernet, board::pins::sdcard, board::pins::usb);

}  // namespace fcu::hw
