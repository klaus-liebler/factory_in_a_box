// I/O pins of FCU revision 3: STM32H743VIT6 (pcb_factory_control_unit, MCU U17). USB-PD by a
// CH224Q (20 V by CFG1, on I2C3 too but not queried), POWER_GOOD from the eFuse TPS26631 (U14
// PGOOD: open drain, high = output good), robot bus on USART1, I2C3 instead of I2C2, LAN9253
// EtherCAT on the QUADSPI pins (driven as plain SPI by GPIO: ethercat_test.cpp).
//
// Console, LED, Ethernet, SD card and USB pins belong to the board package (rtoos boards/fcu_rev3).
// Pins are named by signal: rtoos checks each against ST's pin database at compile time. What this
// revision lacks is std::nullopt.
#pragma once

#include <array>
#include <optional>

#include "hal_types.hpp"

namespace fcu::pins {

// ---- pneumatics -----------------------------------------------------------------------------
inline constexpr Pin valve1{GpioPort::D, 10};
inline constexpr Pin valve2{GpioPort::E, 11};
inline constexpr Pin valve3{GpioPort::A, 3};
inline constexpr std::optional<Pin> vacuum_valve = Pin{GpioPort::C, 10};
inline constexpr std::optional<Pin> vacuum_pump = Pin{GpioPort::B, 7};
inline constexpr Pin pressure_sensor{GpioPort::A, 4};  ///< ADC1 INP18

// ---- light barriers (NPN, active low; EXTI lines 10 and 11) ------------------------------------
inline constexpr Pin light_barrier1{GpioPort::B, 10};
inline constexpr Pin light_barrier2{GpioPort::D, 11};
inline constexpr std::optional<Pin> light_barrier3 = std::nullopt;

// ---- motors -----------------------------------------------------------------------------------
inline constexpr Pin conveyor_pwm{GpioPort::D, 14};    ///< TIM4_CH3
inline constexpr Pin compressor_pwm{GpioPort::D, 15};  ///< TIM4_CH4
inline constexpr Pin stepper1_step{GpioPort::A, 15};   ///< TIM2_CH1
inline constexpr Pin stepper1_dir{GpioPort::D, 5};
inline constexpr Pin stepper2_step{GpioPort::B, 4};    ///< TIM3_CH1
inline constexpr Pin stepper2_dir{GpioPort::D, 4};
inline constexpr Pin stepper_enable{GpioPort::D, 3};   ///< TMC2209 EN, active low
/// TMC2209 DIAG outputs (high on a stall or a driver error; EXTI lines 6 and 4).
inline constexpr std::optional<Pin> stepper1_diag = Pin{GpioPort::D, 6};
inline constexpr std::optional<Pin> stepper2_diag = Pin{GpioPort::E, 4};
inline constexpr Pin stepper_rx{GpioPort::B, 5};       ///< UART5
inline constexpr Pin stepper_tx{GpioPort::B, 6};
inline constexpr Pin robot_rx{GpioPort::B, 15};        ///< USART1 (RoArm-M3 servo bus)
inline constexpr Pin robot_tx{GpioPort::B, 14};

// ---- sensors ----------------------------------------------------------------------------------
inline constexpr Pin hx711_dout{GpioPort::C, 2};   ///< PC2_C pad, SPI2_MISO (HX711 DOUT)
inline constexpr Pin hx711_clock{GpioPort::C, 3};  ///< PC3_C pad, SPI2_MOSI (bit patterns clock the HX711)
inline constexpr Pin tof1_irq{GpioPort::D, 7};
inline constexpr Pin tof2_irq{GpioPort::E, 15};
inline constexpr Pin tof3_irq{GpioPort::A, 5};
inline constexpr Pin color_irq{GpioPort::C, 13};  ///< RGB_IRQ
/// INA226 ALERT, open drain (EXTI line 9)
inline constexpr Pin current_alert{GpioPort::A, 9};
/// TPS26631 PGOOD, open drain, high: the eFuse's output is up (TI TPS2663 datasheet: "Active High");
/// no external pull-up - the internal one. Polled.
inline constexpr std::optional<Pin> power_good = Pin{GpioPort::A, 10};
inline constexpr bool power_good_active_low = false;

// ---- other ------------------------------------------------------------------------------------
inline constexpr Pin ws2812_1{GpioPort::E, 5};  ///< TIM15_CH1
inline constexpr Pin ws2812_2{GpioPort::E, 6};  ///< TIM15_CH2
inline constexpr Pin can_rx{GpioPort::D, 0}, can_tx{GpioPort::D, 1};  ///< FDCAN1
inline constexpr Pin usb_vsense{GpioPort::C, 0};
inline constexpr Pin ethercat_reset{GpioPort::A, 6};  ///< LAN9253 RST# (CAT_RESET)
/// UEXT connector (J15) SPI chip select, active low. The SPI itself (SPI4, PE12-PE14) is unused
/// so far: only the select is driven, inactive.
inline constexpr std::optional<Pin> uext_cs = Pin{GpioPort::E, 0};
/// Malibotti robot (J16): USART6, 3.3 V levels, no series resistors on the board.
inline constexpr Pin malibotti_tx{GpioPort::C, 6};  ///< J16 pin 2 (MALIBOTTI_TX: to the robot's RX)
inline constexpr Pin malibotti_rx{GpioPort::C, 7};  ///< J16 pin 1 (MALIBOTTI_RX)
/// GPIO 1/2 (J30, J31: +3V3, GND, signal), binary inputs, active high: the internal pull-down keeps
/// an open input at 0. Direct to the MCU (no series resistor, no ESD protection on the board).
inline constexpr std::optional<Pin> gpio1 = Pin{GpioPort::E, 2};
inline constexpr std::optional<Pin> gpio2 = Pin{GpioPort::E, 3};
/// LAN9253 EtherCAT (U13): its QUADSPI lines driven as plain SPI by GPIO (ethercat.cpp).
inline constexpr std::optional<EthercatPins> ethercat = EthercatPins{
    .sck = {GpioPort::B, 2}, .cs = {GpioPort::C, 11}, .sio0 = {GpioPort::E, 7}, .sio1 = {GpioPort::E, 8},
    .sio2 = {GpioPort::E, 9}, .sio3 = {GpioPort::E, 10}, .irq = {GpioPort::C, 14}, .sync0 = {GpioPort::B, 0},
    .sync1 = {GpioPort::B, 1}};
/// IRQ (PC14, open drain, active low) wakes the EtherCAT thread through EXTI.
inline constexpr bool ethercat_irq_wired = true;

/// I2C buses: the supply bus carries the INA226, ToF 3 and the CH224Q; the second ToF bus ToF 2.
inline constexpr Pin i2c1_scl{GpioPort::B, 8}, i2c1_sda{GpioPort::B, 9};             ///< colour, ToF 1
inline constexpr Pin i2c_tof2_scl{GpioPort::D, 12}, i2c_tof2_sda{GpioPort::D, 13};   ///< I2C4: ToF 2
inline constexpr Pin i2c_supply_scl{GpioPort::A, 8}, i2c_supply_sda{GpioPort::C, 9}; ///< I2C3: INA226

/// Pins needed before the supply is up: the supply bus (INA226), PGOOD, the UEXT select released.
inline constexpr auto supply_pins = std::array{
    gpio::alternate(i2c_supply_scl, S::I2C3_SCL, gpio::Speed::Low, gpio::Pull::None, /*open_drain*/ true),
    gpio::alternate(i2c_supply_sda, S::I2C3_SDA, gpio::Speed::Low, gpio::Pull::None, true),
    gpio::input(current_alert, gpio::Pull::Up),
    gpio::input(*power_good, gpio::Pull::Up),
    gpio::output(*uext_cs, /*high = deselected*/ true),
};

/// Everything else, configured once the supply is up (power consumers). Outputs start inactive:
/// valves closed, motors off, steppers disabled, ESC in reset.
inline constexpr auto io_pins = hal::join(
    std::array{
        gpio::output(valve1), gpio::output(valve2), gpio::output(valve3),
        gpio::output(*vacuum_valve), gpio::output(*vacuum_pump),
        gpio::alternate(conveyor_pwm, S::TIM4_CH3, gpio::Speed::Low), gpio::alternate(compressor_pwm, S::TIM4_CH4, gpio::Speed::Low),
        gpio::analog(pressure_sensor, S::ADC1_INP18),
        gpio::input(light_barrier1), gpio::input(light_barrier2),
        gpio::alternate(stepper1_step, S::TIM2_CH1, gpio::Speed::Medium), gpio::output(stepper1_dir),
        gpio::alternate(stepper2_step, S::TIM3_CH1, gpio::Speed::Medium), gpio::output(stepper2_dir),
        gpio::output(stepper_enable, /*high = disabled*/ true),
        gpio::input(*stepper1_diag, gpio::Pull::Down), gpio::input(*stepper2_diag, gpio::Pull::Down),
        gpio::alternate(stepper_rx, S::UART5_RX, gpio::Speed::Low, gpio::Pull::Up), gpio::alternate(stepper_tx, S::UART5_TX, gpio::Speed::Low),
        gpio::alternate(robot_rx, S::USART1_RX, gpio::Speed::Low, gpio::Pull::Up), gpio::alternate(robot_tx, S::USART1_TX, gpio::Speed::Low),
        gpio::alternate(malibotti_rx, S::USART6_RX, gpio::Speed::Low, gpio::Pull::Up), gpio::alternate(malibotti_tx, S::USART6_TX, gpio::Speed::Low),
        gpio::input(*gpio1, gpio::Pull::Down), gpio::input(*gpio2, gpio::Pull::Down),
        gpio::alternate(i2c1_scl, S::I2C1_SCL, gpio::Speed::Low, gpio::Pull::None, true),
        gpio::alternate(i2c1_sda, S::I2C1_SDA, gpio::Speed::Low, gpio::Pull::None, true),
        gpio::alternate(i2c_tof2_scl, S::I2C4_SCL, gpio::Speed::Low, gpio::Pull::None, true),
        gpio::alternate(i2c_tof2_sda, S::I2C4_SDA, gpio::Speed::Low, gpio::Pull::None, true),
        gpio::alternate(hx711_dout, S::SPI2_MISO, gpio::Speed::Low), gpio::alternate(hx711_clock, S::SPI2_MOSI, gpio::Speed::Low),
        gpio::input(tof1_irq), gpio::input(tof2_irq), gpio::input(tof3_irq), gpio::input(color_irq),
        gpio::alternate(ws2812_1, S::TIM15_CH1, gpio::Speed::Medium), gpio::alternate(ws2812_2, S::TIM15_CH2, gpio::Speed::Medium),
        gpio::alternate(can_rx, S::FDCAN1_RX, gpio::Speed::Low), gpio::alternate(can_tx, S::FDCAN1_TX, gpio::Speed::Low),
        gpio::input(usb_vsense),
        gpio::output(ethercat_reset),
    },
    ethercat_pin_config(*ethercat));

}  // namespace fcu::pins
