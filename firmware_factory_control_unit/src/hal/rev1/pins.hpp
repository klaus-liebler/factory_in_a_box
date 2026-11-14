// I/O pins of FCU revision 1 (first boards): STM32H563, the pin map of the former CubeMX project.
// USB-PD by the MCU's own UCPD1 sink (CC lines PB13/PB14); three light barriers, no TMC2209 DIAG
// lines, no vacuum valve/pump, the LAN9253 EtherCAT on the OCTOSPI1 pins.
//
// Console, LED, Ethernet and SD card pins belong to the board package (rtoos boards/fcu). Pins
// are named by signal: rtoos checks each against ST's pin database at compile time. What this
// revision lacks is std::nullopt.
#pragma once

#include <array>
#include <optional>

#include "hal_types.hpp"

namespace fcu::pins {

// ---- pneumatics -----------------------------------------------------------------------------
inline constexpr Pin valve1{GpioPort::D, 10};
inline constexpr Pin valve2{GpioPort::D, 5};
inline constexpr Pin valve3{GpioPort::D, 11};
inline constexpr std::optional<Pin> vacuum_valve = std::nullopt;
inline constexpr std::optional<Pin> vacuum_pump = std::nullopt;
inline constexpr Pin pressure_sensor{GpioPort::A, 4};  ///< ADC1 INP18

// ---- light barriers (NPN, active low) --------------------------------------------------------
inline constexpr Pin light_barrier1{GpioPort::C, 6};
inline constexpr Pin light_barrier2{GpioPort::C, 7};
inline constexpr std::optional<Pin> light_barrier3 = Pin{GpioPort::C, 8};

// ---- motors -----------------------------------------------------------------------------------
inline constexpr Pin conveyor_pwm{GpioPort::D, 14};    ///< TIM4_CH3
inline constexpr Pin compressor_pwm{GpioPort::D, 15};  ///< TIM4_CH4
inline constexpr Pin stepper1_step{GpioPort::A, 15};   ///< TIM2_CH1
inline constexpr Pin stepper1_dir{GpioPort::D, 6};
inline constexpr Pin stepper2_step{GpioPort::B, 4};    ///< TIM3_CH1
inline constexpr Pin stepper2_dir{GpioPort::D, 7};
inline constexpr Pin stepper_enable{GpioPort::B, 7};   ///< TMC2209 EN, active low
inline constexpr std::optional<Pin> stepper1_diag = std::nullopt;
inline constexpr std::optional<Pin> stepper2_diag = std::nullopt;
inline constexpr Pin stepper_rx{GpioPort::B, 5};       ///< UART5
inline constexpr Pin stepper_tx{GpioPort::B, 6};
inline constexpr Pin robot_rx{GpioPort::E, 7};         ///< UART7 (RoArm-M3 servo bus)
inline constexpr Pin robot_tx{GpioPort::E, 8};

// ---- sensors ----------------------------------------------------------------------------------
inline constexpr Pin i2c1_scl{GpioPort::B, 8}, i2c1_sda{GpioPort::B, 9};                ///< colour, ToF 1
inline constexpr Pin i2c_tof2_scl{GpioPort::B, 10}, i2c_tof2_sda{GpioPort::B, 12};      ///< I2C2: ToF 2
inline constexpr Pin i2c_supply_scl{GpioPort::D, 12}, i2c_supply_sda{GpioPort::D, 13};  ///< I2C4: INA226, ToF 3
inline constexpr Pin hx711_dout{GpioPort::C, 2};   ///< SPI2_MISO (HX711 DOUT)
inline constexpr Pin hx711_clock{GpioPort::C, 3};  ///< SPI2_MOSI (bit patterns clock the HX711)
inline constexpr Pin tof1_irq{GpioPort::C, 15};
inline constexpr Pin tof2_irq{GpioPort::H, 1};
inline constexpr Pin tof3_irq{GpioPort::A, 3};
inline constexpr Pin color_irq{GpioPort::H, 0};  ///< INT_RGB
/// INA226 ALERT, open drain
inline constexpr Pin current_alert{GpioPort::D, 4};
inline constexpr std::optional<Pin> power_good = std::nullopt;
inline constexpr bool power_good_active_low = true;

// ---- other ------------------------------------------------------------------------------------
inline constexpr Pin ws2812_1{GpioPort::E, 5};  ///< TIM15_CH1
inline constexpr Pin ws2812_2{GpioPort::E, 6};  ///< TIM15_CH2
inline constexpr Pin can_rx{GpioPort::D, 0}, can_tx{GpioPort::D, 1};  ///< FDCAN1
inline constexpr Pin usb_vsense{GpioPort::C, 0};
/// UCPD1 sink: the USB-C connector's CC lines.
inline constexpr Pin usb_pd_cc1{GpioPort::B, 13}, usb_pd_cc2{GpioPort::B, 14};
inline constexpr Pin ethercat_reset{GpioPort::C, 14};  ///< ESC held in reset
inline constexpr std::optional<Pin> uext_cs = std::nullopt;
/// LAN9253 EtherCAT on OCTOSPI1 (former CubeMX project: ETHERCAT_*), driven as plain SPI by GPIO
/// (ethercat_test.cpp).
inline constexpr std::optional<EthercatPins> ethercat =
    EthercatPins{.sck = {GpioPort::B, 2}, .cs = {GpioPort::E, 11}, .sio0 = {GpioPort::B, 1}, .sio1 = {GpioPort::B, 0},
                 .sio2 = {GpioPort::E, 2}, .sio3 = {GpioPort::A, 6}, .irq = {GpioPort::C, 13}, .sync0 = {GpioPort::E, 4},
                 .sync1 = {GpioPort::E, 3}};
/// GPIO 1/2 inputs, the Malibotti connector: revision 3 only.
inline constexpr std::optional<Pin> gpio1 = std::nullopt;
inline constexpr std::optional<Pin> gpio2 = std::nullopt;

/// Pins needed before the supply is up: the supply bus (I2C4, INA226) and the USB-PD CC lines.
inline constexpr auto supply_pins = std::array{
    gpio::alternate(i2c_supply_scl, S::I2C4_SCL, gpio::Speed::Low, gpio::Pull::None, /*open_drain*/ true),
    gpio::alternate(i2c_supply_sda, S::I2C4_SDA, gpio::Speed::Low, gpio::Pull::None, true),
    gpio::input(current_alert, gpio::Pull::Up),
    gpio::analog(usb_pd_cc1, S::UCPD1_CC1),
    gpio::analog(usb_pd_cc2, S::UCPD1_CC2),
};

/// Everything else, configured once the supply is up (power consumers). Outputs start inactive:
/// valves closed, motors off, steppers disabled, ESC in reset.
inline constexpr auto io_pins = hal::join(
    std::array{
        gpio::output(valve1), gpio::output(valve2), gpio::output(valve3),
        gpio::alternate(conveyor_pwm, S::TIM4_CH3, gpio::Speed::Low), gpio::alternate(compressor_pwm, S::TIM4_CH4, gpio::Speed::Low),
        gpio::analog(pressure_sensor, S::ADC1_INP18),
        gpio::input(light_barrier1), gpio::input(light_barrier2), gpio::input(*light_barrier3),
        gpio::alternate(stepper1_step, S::TIM2_CH1, gpio::Speed::Medium), gpio::output(stepper1_dir),
        gpio::alternate(stepper2_step, S::TIM3_CH1, gpio::Speed::Medium), gpio::output(stepper2_dir),
        gpio::output(stepper_enable, /*high = disabled*/ true),
        gpio::alternate(stepper_rx, S::UART5_RX, gpio::Speed::Low, gpio::Pull::Up), gpio::alternate(stepper_tx, S::UART5_TX, gpio::Speed::Low),
        gpio::alternate(robot_rx, S::UART7_RX, gpio::Speed::Low, gpio::Pull::Up), gpio::alternate(robot_tx, S::UART7_TX, gpio::Speed::Low),
        gpio::alternate(i2c1_scl, S::I2C1_SCL, gpio::Speed::Low, gpio::Pull::None, true),
        gpio::alternate(i2c1_sda, S::I2C1_SDA, gpio::Speed::Low, gpio::Pull::None, true),
        gpio::alternate(i2c_tof2_scl, S::I2C2_SCL, gpio::Speed::Low, gpio::Pull::None, true),
        gpio::alternate(i2c_tof2_sda, S::I2C2_SDA, gpio::Speed::Low, gpio::Pull::None, true),
        gpio::alternate(hx711_dout, S::SPI2_MISO, gpio::Speed::Low), gpio::alternate(hx711_clock, S::SPI2_MOSI, gpio::Speed::Low),
        gpio::input(tof1_irq), gpio::input(tof2_irq), gpio::input(tof3_irq), gpio::input(color_irq),
        gpio::alternate(ws2812_1, S::TIM15_CH1, gpio::Speed::Medium), gpio::alternate(ws2812_2, S::TIM15_CH2, gpio::Speed::Medium),
        gpio::alternate(can_rx, S::FDCAN1_RX, gpio::Speed::Low), gpio::alternate(can_tx, S::FDCAN1_TX, gpio::Speed::Low),
        gpio::input(usb_vsense),
        gpio::output(ethercat_reset),
    },
    ethercat_pin_config(*ethercat));

}  // namespace fcu::pins
