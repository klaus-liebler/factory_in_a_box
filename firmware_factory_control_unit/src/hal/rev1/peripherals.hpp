// Peripherals of FCU revision 1 (STM32H563, first boards), every configuration a compile-time
// value:
//
//   ToF 2 bus   I2C2 (ToF 2 VL53L0X)                                  100 kHz
//   supply bus  I2C4 (INA226, ToF 3 VL53L0X)                          100 kHz
//   servo bus   UART7 (Feetech servos of the RoArm-M3)                1 Mbit/s
//   DMA         GPDMA1 channels 0/1 (UCPD RX/TX), 2-5 (LED strip, single LED, steppers 1/2)
//   ADC1        pressure sensor (INP18, 12 bits); CPU temperature: the DTS
//   USB-PD      the MCU's UCPD1 sink, TIM5 for its deadlines (1 MHz, 32 bit): power_delivery.cpp
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "hal_types.hpp"
#include "mcu.h"
#include "pins.hpp"

// Interrupt vectors of the firmware's handlers (names: the chip's vector table).
#define FCU_I2C_TOF2_EV_IRQ_HANDLER I2C2_EV_IRQHandler
#define FCU_I2C_TOF2_ER_IRQ_HANDLER I2C2_ER_IRQHandler
#define FCU_I2C_SUPPLY_EV_IRQ_HANDLER I2C4_EV_IRQHandler
#define FCU_I2C_SUPPLY_ER_IRQ_HANDLER I2C4_ER_IRQHandler
#define FCU_STEPPER1_DMA_IRQ_HANDLER GPDMA1_CH4_IRQHandler
#define FCU_STEPPER2_DMA_IRQ_HANDLER GPDMA1_CH5_IRQHandler
#define FCU_SERVO_UART_IRQ_HANDLER UART7_IRQHandler
/// The EXTI vectors the firmware's lines use, with their IRQ numbers (one vector per line on the
/// H563: EXTIn = IRQ 11 + n). hardware.hpp checks that every line used has its vector here.
#define FCU_EXTI_HANDLERS(X) \
    X(EXTI2_IRQHandler, 13)  /* HX711 DOUT (PC2) */ \
    X(EXTI4_IRQHandler, 15)  /* INA226 ALERT (PD4) */ \
    X(EXTI6_IRQHandler, 17)  /* light barrier 1 (PC6) */ \
    X(EXTI7_IRQHandler, 18)  /* light barrier 2 (PC7) */ \
    X(EXTI8_IRQHandler, 19)  /* light barrier 3 (PC8) */

namespace fcu::hw {

// ---- buses ------------------------------------------------------------------------------------
inline constexpr unsigned i2c_tof2_bus = 2, i2c_supply_bus = 4;  ///< the buses' numbers (logs, terminal)
using I2c1 = chip::I2cMaster<chip::I2cId::I2c1, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<1>>;
using I2cTof2 = chip::I2cMaster<chip::I2cId::I2c2, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<2>>;
using I2cSupply = chip::I2cMaster<chip::I2cId::I2c4, {.kernel_hz = clocks.pclk3_hz(), .cpu_hz = board::sysclk_hz}, I2cWait<4>>;
using ServoUart = chip::BufferedUart<chip::UsartId::Uart7, {.kernel_hz = clocks.pclk1_hz(), .baud = 1'000'000}, 256, 256>;

// ---- analog -----------------------------------------------------------------------------------
using Pressure = chip::Adc<chip::AdcId::Adc1, {.kernel_hz = clocks.hclk_hz(), .cpu_hz = board::sysclk_hz, .channel = 18,
                                               .sample = chip::AdcSample::Cycles2_5}>;
inline constexpr unsigned pressure_shift = 0;  ///< 12-bit conversions already
using CpuTemperature = chip::Dts<{.pclk_hz = clocks.pclk1_hz()}>;

// ---- DMA: GPDMA1 channels 2-5 (0 and 1: UCPD); memory items of 16 bits widened to the 32-bit ARR -
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
/// TIM15 (`LedTimer`): DMAR -> CCR2 at each update (DCR: base CCR2, one transfer, update source).
template <class LedTimer>
void configure_led_burst() noexcept {
    LedTimer::configure_dma_burst(0x38, 1, 1);
}
/// The STM32H563 has no PC2_C/PC3_C analog switches.
inline void close_analog_switches() noexcept {}

// ---- USB Power Delivery: the MCU's UCPD1 sink (power_delivery.cpp) ------------------------------
inline constexpr bool mcu_power_delivery = true;
inline constexpr std::string_view pd_controller = "UCPD1";
using PdClock = chip::MicrosecondTimer<chip::TimerId::Tim5, clocks.timer1_hz()>;
using Ucpd = chip::Ucpd<{.kernel_hz = clocks.ucpd_hz()}>;  // GPDMA channels 0 (RX), 1 (TX)

// ---- what only this revision adds to the resource checks (hardware.hpp) ----------------------------
inline constexpr std::array revision_timers{PdClock::base};
inline constexpr std::array revision_dma_channels{Ucpd::rx_dma::id, Ucpd::tx_dma::id};
inline constexpr std::array<std::uintptr_t, 0> revision_uarts{};
/// Every pin the board package configures.
inline constexpr auto board_package_pins = hal::join(board::pins::base, board::pins::ethernet, board::pins::sdcard);

}  // namespace fcu::hw
