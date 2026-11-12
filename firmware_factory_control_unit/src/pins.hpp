// I/O pins of the FCU firmware, per hardware revision (board::revision, CMake RTOS_FCU_REVISION).
//
//   revision 1: first boards, STM32H563, pin map of the former CubeMX project
//   revision 2: STM32H563 (pcb_factory_control_unit, MCU U1) - USB-PD by a CH224K (requests
//               20 V, CFG1 open, PGOOD on PA9), the MCU's UCPD unused (dead-battery pull-downs
//               off in board_early_init()): the former CC/DB pins carry UEXT_CS (PB13),
//               VACUUM_VALVE (PB14), POWER_GOOD (PA9) and CURRENT ALERT (PC9)
//   revision 3: STM32H743VIT6 (pcb_factory_control_unit, MCU U17) - USB-PD by a CH224Q (20 V by
//               CFG1, on I2C3 too but not queried), POWER_GOOD from the eFuse TPS26631 (U14
//               PGOOD: open drain, high = output good), robot bus on USART1, I2C3 instead of
//               I2C2, LAN9253 EtherCAT on QUADSPI (held in reset, not used yet)
//
// Console, LED, Ethernet and SD card pins belong to the board package (rtoos boards/fcu,
// boards/fcu_rev3). Pins are named by signal: rtoos checks each against ST's pin database at
// compile time.
#pragma once

#include <array>
#include <optional>

#include "board/chip.hpp"
#include "board/fcu.hpp"

namespace fcu::pins {

namespace gpio = board::chip::gpio;
using gpio::GpioPort;
using gpio::Pin;
using S = gpio::Signal;

inline constexpr bool rev1 = board::revision == 1;
inline constexpr bool rev3 = board::revision == 3;

/// The LAN9253's host interface (SPI/SQI) and its other lines. The test code drives SPI by GPIO
/// (SIO0 = SI, SIO1 = SO; SIO2/SIO3 held high), no OCTOSPI/QUADSPI driver yet.
struct EthercatPins {
    Pin sck, cs, sio0, sio1, sio2, sio3, irq, sync0, sync1;
};
template <std::size_t A, std::size_t B>
[[nodiscard]] constexpr std::array<gpio::PinConfig, A + B> join_pins(const std::array<gpio::PinConfig, A>& a,
                                                                     const std::array<gpio::PinConfig, B>& b) {
    std::array<gpio::PinConfig, A + B> r{};
    for (std::size_t i = 0; i < A; ++i) r[i] = a[i];
    for (std::size_t i = 0; i < B; ++i) r[A + i] = b[i];
    return r;
}
/// Their configuration: select inactive, clock low, SYNC/IRQ inputs (IRQ: open drain, pulled up -
/// not used by the software, which polls; on revision 1 it is not connected).
[[nodiscard]] constexpr std::array<gpio::PinConfig, 9> ethercat_pin_config(const EthercatPins& e) {
    return {gpio::output(e.cs, /*high = deselected*/ true, gpio::Speed::High), gpio::output(e.sck, false, gpio::Speed::High),
            gpio::output(e.sio0, false, gpio::Speed::High), gpio::input(e.sio1, gpio::Pull::Up),
            gpio::output(e.sio2, true), gpio::output(e.sio3, true), gpio::input(e.irq, gpio::Pull::Up),
            gpio::input(e.sync0, gpio::Pull::Down), gpio::input(e.sync1, gpio::Pull::Down)};
}

#if RTOS_FCU_REVISION == 3
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
inline constexpr Pin ethercat_reset{GpioPort::A, 6};  ///< LAN9253 held in reset (CAT_RESET)
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
/// LAN9253 EtherCAT (U13): its QUADSPI lines driven as plain SPI by GPIO (ethercat_test.cpp).
inline constexpr std::optional<EthercatPins> ethercat = EthercatPins{
    .sck = {GpioPort::B, 2}, .cs = {GpioPort::C, 11}, .sio0 = {GpioPort::E, 7}, .sio1 = {GpioPort::E, 8},
    .sio2 = {GpioPort::E, 9}, .sio3 = {GpioPort::E, 10}, .irq = {GpioPort::C, 14}, .sync0 = {GpioPort::B, 0},
    .sync1 = {GpioPort::B, 1}};

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

namespace detail {
inline constexpr auto base_io_pins = std::array{
    // outputs start inactive: valves closed, motors off, steppers disabled, ESC in reset
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
};
}  // namespace detail
/// Everything else, configured once the supply is up (power consumers).
inline constexpr auto io_pins = join_pins(detail::base_io_pins, ethercat_pin_config(*ethercat));

#else  // revisions 1 and 2: STM32H563
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
inline constexpr Pin stepper_rx{GpioPort::B, 5};       ///< UART5
inline constexpr Pin stepper_tx{GpioPort::B, 6};
inline constexpr Pin robot_rx{GpioPort::E, 7};         ///< UART7 (RoArm-M3 servo bus)
inline constexpr Pin robot_tx{GpioPort::E, 8};

// ---- sensors ----------------------------------------------------------------------------------
inline constexpr Pin i2c1_scl{GpioPort::B, 8}, i2c1_sda{GpioPort::B, 9};                ///< color, ToF 1
inline constexpr Pin i2c_tof2_scl{GpioPort::B, 10}, i2c_tof2_sda{GpioPort::B, 12};      ///< I2C2: ToF 2
inline constexpr Pin i2c_supply_scl{GpioPort::D, 12}, i2c_supply_sda{GpioPort::D, 13};  ///< I2C4: INA226, ToF 3
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
inline constexpr bool power_good_active_low = true;

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
/// GPIO 1/2 inputs: revision 3 only.
/// LAN9253 EtherCAT on OCTOSPI1 (former CubeMX project: ETHERCAT_*), driven as plain SPI by GPIO
/// (ethercat_test.cpp); revision 1 only - revision 2's wiring is not known here.
inline constexpr std::optional<EthercatPins> ethercat =
    rev1 ? std::optional{EthercatPins{.sck = {GpioPort::B, 2}, .cs = {GpioPort::E, 11}, .sio0 = {GpioPort::B, 1},
                                      .sio1 = {GpioPort::B, 0}, .sio2 = {GpioPort::E, 2}, .sio3 = {GpioPort::A, 6},
                                      .irq = {GpioPort::C, 13}, .sync0 = {GpioPort::E, 4}, .sync1 = {GpioPort::E, 3}}}
         : std::nullopt;
inline constexpr std::optional<Pin> gpio1 = std::nullopt;
inline constexpr std::optional<Pin> gpio2 = std::nullopt;

/// Pins needed before the supply is up: the supply bus (I2C4, INA226); with UCPD the USB-PD CC
/// lines, with the CH224K its PGOOD and (not supply, but as early as possible) the UEXT select
/// released.
inline constexpr auto supply_pins = [] {
    constexpr std::array common{
        gpio::alternate(i2c_supply_scl, S::I2C4_SCL, gpio::Speed::Low, gpio::Pull::None, /*open_drain*/ true),
        gpio::alternate(i2c_supply_sda, S::I2C4_SDA, gpio::Speed::Low, gpio::Pull::None, true),
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
    gpio::alternate(i2c_tof2_scl, S::I2C2_SCL, gpio::Speed::Low, gpio::Pull::None, true),
    gpio::alternate(i2c_tof2_sda, S::I2C2_SDA, gpio::Speed::Low, gpio::Pull::None, true),
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
        return join_pins(detail::join(detail::common_io_pins, std::array{gpio::input(*light_barrier3)}),
                         ethercat_pin_config(*ethercat));
    } else {
        return detail::join(detail::common_io_pins,
                            std::array{gpio::output(*vacuum_valve), gpio::output(*vacuum_pump),
                                       gpio::input(*stepper1_diag, gpio::Pull::Down),
                                       gpio::input(*stepper2_diag, gpio::Pull::Down)});
    }
}();
#endif

inline constexpr bool stepper_diag = stepper1_diag.has_value() && stepper2_diag.has_value();

}  // namespace fcu::pins
