// I/O pins of the FCU firmware, per hardware revision (board::revision, CMake RTOS_FCU_REVISION).
//
//   revision 1: first boards, pin map of the former CubeMX project (firmware_factory_control_unit.ioc)
//   revision 2: current PCB (pcb_factory_control_unit, MCU U1) - USB-PD by a CH224K (requests
//               20 V, CFG1 open, PGOOD on PA9), the MCU's UCPD unused (dead-battery pull-downs
//               off in board_early_init()): the former CC/DB pins carry UEXT_CS (PB13),
//               VACUUM_VALVE (PB14), POWER_GOOD (PA9) and CURRENT ALERT (PC9)
//
// Console, LED, Ethernet and SD card pins belong to the board package (rtoos boards/fcu).
// Pins are named by signal: rtoos checks each against ST's pin database at compile time.
#pragma once

#include <array>
#include <optional>

#include "board/fcu.hpp"
#include "rtos/port/stm32h5/gpio.hpp"

namespace fcu::pins {

namespace gpio = rtos::port::stm32h5::gpio;
using gpio::GpioPort;
using gpio::Pin;
using S = gpio::Signal;

inline constexpr bool rev1 = board::revision == 1;

// ---- pneumatics -----------------------------------------------------------------------------
inline constexpr Pin valve1 = rev1 ? Pin{GpioPort::D, 10} : Pin{GpioPort::A, 8};
inline constexpr Pin valve2 = rev1 ? Pin{GpioPort::D, 5} : Pin{GpioPort::C, 6};
inline constexpr Pin valve3{GpioPort::D, 11};
/// Revision 2 only.
inline constexpr std::optional<Pin> vacuum_valve = rev1 ? std::nullopt : std::optional{Pin{GpioPort::B, 14}};  ///< UCPD1_CC2 pin
inline constexpr std::optional<Pin> vacuum_pump = rev1 ? std::nullopt : std::optional{Pin{GpioPort::C, 11}};
inline constexpr Pin pressure_sensor{GpioPort::A, 4};  ///< ADC1 INP18

// ---- light barriers (NPN, active low) --------------------------------------------------------
inline constexpr Pin light_barrier1 = rev1 ? Pin{GpioPort::C, 6} : Pin{GpioPort::C, 7};
inline constexpr Pin light_barrier2 = rev1 ? Pin{GpioPort::C, 7} : Pin{GpioPort::C, 8};
/// Revision 1 only.
inline constexpr std::optional<Pin> light_barrier3 = rev1 ? std::optional{Pin{GpioPort::C, 8}} : std::nullopt;

// ---- motors -----------------------------------------------------------------------------------
inline constexpr Pin conveyor_pwm{GpioPort::D, 14};    ///< TIM4_CH3
inline constexpr Pin compressor_pwm{GpioPort::D, 15};  ///< TIM4_CH4
inline constexpr Pin stepper1_step{GpioPort::A, 15};  ///< TIM2_CH1
inline constexpr Pin stepper1_dir = rev1 ? Pin{GpioPort::D, 6} : Pin{GpioPort::D, 5};
inline constexpr Pin stepper2_step{GpioPort::B, 4};   ///< TIM3_CH1
inline constexpr Pin stepper2_dir{GpioPort::D, 7};
inline constexpr Pin stepper_enable{GpioPort::B, 7};   ///< TMC2209 EN, active low
/// TMC2209 DIAG outputs (high on a stall or a driver error), revision 2 only (EXTI lines 6 and 0).
inline constexpr std::optional<Pin> stepper1_diag = rev1 ? std::nullopt : std::optional{Pin{GpioPort::D, 6}};
inline constexpr std::optional<Pin> stepper2_diag = rev1 ? std::nullopt : std::optional{Pin{GpioPort::E, 0}};
inline constexpr bool stepper_diag = stepper1_diag.has_value() && stepper2_diag.has_value();
inline constexpr Pin stepper_rx{GpioPort::B, 5};       ///< UART5
inline constexpr Pin stepper_tx{GpioPort::B, 6};
inline constexpr Pin robot_rx{GpioPort::E, 7};         ///< UART7 (RoArm-M3 servo bus)
inline constexpr Pin robot_tx{GpioPort::E, 8};

// ---- sensors ----------------------------------------------------------------------------------
inline constexpr Pin i2c1_scl{GpioPort::B, 8}, i2c1_sda{GpioPort::B, 9};    ///< color, ToF
inline constexpr Pin i2c2_scl{GpioPort::B, 10}, i2c2_sda{GpioPort::B, 12};  ///< ToF
inline constexpr Pin i2c4_scl{GpioPort::D, 12}, i2c4_sda{GpioPort::D, 13};  ///< INA226
inline constexpr Pin hx711_dout{GpioPort::C, 2};  ///< SPI2_MISO (HX711 DOUT)
inline constexpr Pin hx711_clock{GpioPort::C, 3}; ///< SPI2_MOSI (bit patterns clock the HX711)
inline constexpr Pin tof1_irq = rev1 ? Pin{GpioPort::C, 15} : Pin{GpioPort::C, 14};
inline constexpr Pin tof2_irq = rev1 ? Pin{GpioPort::H, 1} : Pin{GpioPort::E, 15};
inline constexpr Pin tof3_irq = rev1 ? Pin{GpioPort::A, 3} : Pin{GpioPort::D, 10};
inline constexpr Pin color_irq = rev1 ? Pin{GpioPort::H, 0} : Pin{GpioPort::H, 1};  ///< INT_RGB
/// INA226 ALERT, open drain; revision 2: PC9 (UCPD1_DB2 pin)
inline constexpr Pin current_alert = rev1 ? Pin{GpioPort::D, 4} : Pin{GpioPort::C, 9};
/// CH224K PGOOD, open drain (low: power good), no external pull-up - the internal one; revision 2
/// only (PA9, UCPD1_DB1 pin). Polled: EXTI line 9 belongs to CURRENT ALERT.
inline constexpr std::optional<Pin> power_good = rev1 ? std::nullopt : std::optional{Pin{GpioPort::A, 9}};

// ---- other ------------------------------------------------------------------------------------
inline constexpr Pin ws2812_1{GpioPort::E, 5};  ///< TIM15_CH1
inline constexpr Pin ws2812_2{GpioPort::E, 6};  ///< TIM15_CH2
inline constexpr Pin can_rx{GpioPort::D, 0}, can_tx{GpioPort::D, 1};  ///< FDCAN1
inline constexpr Pin usb_vsense{GpioPort::C, 0};
/// UCPD1 sink (revision 1 only; board::has_ucpd)
inline constexpr Pin usb_pd_cc1{GpioPort::B, 13}, usb_pd_cc2{GpioPort::B, 14};
inline constexpr Pin ethercat_reset = rev1 ? Pin{GpioPort::C, 14} : Pin{GpioPort::D, 3};  ///< ESC held in reset
/// UEXT connector (J15) SPI chip select, active low; revision 2 only (PB13, UCPD1_CC1 pin). The
/// SPI itself (SPI4, PE12-PE14) is unused so far: only the select is driven, inactive.
inline constexpr std::optional<Pin> uext_cs = rev1 ? std::nullopt : std::optional{Pin{GpioPort::B, 13}};

/// Pins needed before the supply is up: I2C4 (INA226); with UCPD the USB-PD CC lines, with the
/// CH224K its PGOOD and (not supply, but as early as possible) the UEXT select released.
inline constexpr auto supply_pins = [] {
    constexpr std::array common{
        gpio::alternate(i2c4_scl, S::I2C4_SCL, gpio::Speed::Low, gpio::Pull::None, /*open_drain*/ true),
        gpio::alternate(i2c4_sda, S::I2C4_SDA, gpio::Speed::Low, gpio::Pull::None, true),
        gpio::input(current_alert, gpio::Pull::Up),
    };
    if constexpr (board::has_ucpd) {
        return std::array{common[0], common[1], common[2], gpio::analog(usb_pd_cc1, S::UCPD1_CC1),
                          gpio::analog(usb_pd_cc2, S::UCPD1_CC2)};
    } else {
        return std::array{common[0], common[1], common[2], gpio::input(*power_good, gpio::Pull::Up),
                          gpio::output(*uext_cs, /*high = deselected*/ true)};
    }
}();

namespace detail {
constexpr auto common_io_pins = std::array{
    // outputs start inactive: valves closed, motors off, steppers disabled, ESC in reset
    gpio::output(valve1), gpio::output(valve2), gpio::output(valve3),
    gpio::alternate(conveyor_pwm, S::TIM4_CH3, gpio::Speed::Low), gpio::alternate(compressor_pwm, S::TIM4_CH4, gpio::Speed::Low),
    gpio::analog(pressure_sensor, S::ADC1_INP18),
    gpio::input(light_barrier1), gpio::input(light_barrier2),
    gpio::alternate(stepper1_step, S::TIM2_CH1, gpio::Speed::Medium), gpio::output(stepper1_dir),
    gpio::alternate(stepper2_step, S::TIM3_CH1, gpio::Speed::Medium), gpio::output(stepper2_dir),
    gpio::output(stepper_enable, /*high = disabled*/ true),
    gpio::alternate(stepper_rx, S::UART5_RX, gpio::Speed::Low, gpio::Pull::Up), gpio::alternate(stepper_tx, S::UART5_TX, gpio::Speed::Low),
    gpio::alternate(robot_rx, S::UART7_RX, gpio::Speed::Low, gpio::Pull::Up), gpio::alternate(robot_tx, S::UART7_TX, gpio::Speed::Low),
    gpio::alternate(i2c1_scl, S::I2C1_SCL, gpio::Speed::Low, gpio::Pull::None, true),
    gpio::alternate(i2c1_sda, S::I2C1_SDA, gpio::Speed::Low, gpio::Pull::None, true),
    gpio::alternate(i2c2_scl, S::I2C2_SCL, gpio::Speed::Low, gpio::Pull::None, true),
    gpio::alternate(i2c2_sda, S::I2C2_SDA, gpio::Speed::Low, gpio::Pull::None, true),
    gpio::alternate(hx711_dout, S::SPI2_MISO, gpio::Speed::Low), gpio::alternate(hx711_clock, S::SPI2_MOSI, gpio::Speed::Low),
    gpio::input(tof1_irq), gpio::input(tof2_irq), gpio::input(tof3_irq), gpio::input(color_irq),
    gpio::alternate(ws2812_1, S::TIM15_CH1, gpio::Speed::Medium), gpio::alternate(ws2812_2, S::TIM15_CH2, gpio::Speed::Medium),
    gpio::alternate(can_rx, S::FDCAN1_RX, gpio::Speed::Low), gpio::alternate(can_tx, S::FDCAN1_TX, gpio::Speed::Low),
    gpio::input(usb_vsense),
    gpio::output(ethercat_reset),
};
template <std::size_t A, std::size_t B>
constexpr auto join(const std::array<gpio::PinConfig, A>& a, const std::array<gpio::PinConfig, B>& b) {
    std::array<gpio::PinConfig, A + B> r{};
    for (std::size_t i = 0; i < A; ++i) r[i] = a[i];
    for (std::size_t i = 0; i < B; ++i) r[A + i] = b[i];
    return r;
}
}  // namespace detail

/// Everything else, configured once the supply is up (power consumers).
inline constexpr auto io_pins = [] {
    if constexpr (rev1) {
        return detail::join(detail::common_io_pins, std::array{gpio::input(*light_barrier3)});
    } else {
        return detail::join(detail::common_io_pins,
                            std::array{gpio::output(*vacuum_valve), gpio::output(*vacuum_pump),
                                       gpio::input(*stepper1_diag, gpio::Pull::Down),
                                       gpio::input(*stepper2_diag, gpio::Pull::Down)});
    }
}();

}  // namespace fcu::pins
