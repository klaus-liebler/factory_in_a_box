// The I/O cycle (see io.hpp).
#include "io.hpp"

#include <array>
#include <cstring>

#include "boot.hpp"
#include "hardware.hpp"
#include "rtos/port/stm32h5/cache.hpp"
#include "network.hpp"
#include "rtos/devices/tmc2209.hpp"
#include "rtos/port/cm33/port.hpp"
#include "roarm/roarm.hpp"
#include "system.hpp"
#include "web.hpp"
#include "ws_protocol.hh"

namespace fcu::io {
namespace {

namespace h5 = rtos::port::stm32h5;
namespace dev = rtos::devices;
namespace cm33 = rtos::port::cm33;
namespace rm = register_map;
using h5::gpio::read;
using h5::gpio::write;

// ---- the buses' interrupts (byte channels) ------------------------------------------------------
hw::StepperUart stepper_uart;

struct KernelClock {
    static std::uint32_t now_ms() noexcept { return kernel.now(); }
    static void sleep_ms(std::uint32_t ms) noexcept { kernel.sleep(ms); }
};

// ---- I2C boot scans ---------------------------------------------------------------------------
std::array<std::array<std::uint8_t, 16>, 3> scans{};

template <class Bus>
void scan(unsigned bus, std::array<std::uint8_t, 16>& out) noexcept {
    out = {};
    unsigned found = 0;
    auto b = RTOS_LOG_BLOCK(Info);
    for (std::uint8_t a = 0x08; a < 0x78; ++a) {  // 7-bit addresses without the reserved ones
        if (Bus::probe(a)) {
            out[a / 8] |= static_cast<std::uint8_t>(1u << (a % 8));
            b.add("I2C%u: device at 0x%02X", bus, a);
            ++found;
        }
    }
    b.add("I2C%u: %u device(s)", bus, found);
}

// ---- sensors ------------------------------------------------------------------------------------
bool supply_present = false, tof_present[3] = {}, color_present = false;
unsigned supply_failures = 0;

void power() noexcept {
    rm::set_pwr_status(registers, supply_present ? 0 : 1);
    if (!supply_present) {
        return;
    }
    if (const auto m = hw::Supply::read()) {
        supply_failures = 0;
        rm::set_pwr_bus_voltage_mv(registers, static_cast<std::uint16_t>(m->bus_mv));
        rm::set_pwr_shunt_voltage_uv(registers, static_cast<std::uint16_t>(m->shunt_uv));
        rm::set_pwr_current_ma(registers, static_cast<std::uint16_t>(m->current_ma));
        rm::set_pwr_power_mw(registers, static_cast<std::uint16_t>(m->power_mw));
    } else if (++supply_failures <= 3 || supply_failures % 20 == 0) {
        RTOS_LOG_WARN("INA226: read failed (%u in a row)", supply_failures);
        hw::I2c4::recover();
    }
}

template <class Tof>
void tof(std::size_t i, std::uint16_t distance_reg, std::uint16_t status_reg) noexcept {
    if (!tof_present[i]) {
        registers.set_input(status_reg, 1);
        return;
    }
    if (const auto mm = Tof::poll()) {  // the measurement started one cycle ago
        registers.set_input(distance_reg, *mm);
        registers.set_input(status_reg, 0);
    }
    (void)Tof::start();
}

void color() noexcept {
    if (!color_present) {
        return;
    }
    if (const auto c = hw::Color::read()) {
        rm::set_color_clear(registers, c->clear);
        rm::set_color_red(registers, c->red);
        rm::set_color_green(registers, c->green);
        rm::set_color_blue(registers, c->blue);
    }
}

void scale() noexcept {
    (void)hw::Scale::poll();
    if (const auto w = hw::Scale::average()) {
        rm::set_scale_a_weight(registers, *w);
        rm::set_scale_a_status(registers, 0);
    } else {
        rm::set_scale_a_status(registers, 1);
    }
}

void cpu_temperature() noexcept {
    if (const auto t = hw::CpuTemperature::centi_celsius()) {
        rm::set_cpu_temperature_c(registers, static_cast<std::int16_t>(*t / 100));
    }
    if (hw::CpuTemperature::ready()) {
        hw::CpuTemperature::start();  // the value for the next cycle
    }
}

std::uint32_t can_rx = 0;
void can() noexcept {
    h5::CanFrame f;
    while (hw::Can::receive(f)) {
        ++can_rx;
    }
    const h5::CanStatus s = hw::Can::status();
    rm::set_can_rx_count(registers, can_rx);
    rm::set_can_error_count(registers, std::uint32_t{s.tx_errors} + s.rx_errors);
    rm::set_can_last_error(registers, s.last_error);
    rm::set_can_bus_state(registers, s.bus_off ? 2 : (s.error_passive ? 1 : 0));
}

// ---- pneumatics, motors ------------------------------------------------------------------------
void outputs() noexcept {
    write(pins::valve1, rm::valve1(registers) != 0);
    write(pins::valve2, rm::valve2(registers) != 0);
    write(pins::valve3, rm::valve3(registers) != 0);
    if constexpr (!pins::rev1) {
        write(*pins::vacuum_valve, rm::vacuum_valve(registers) != 0);
        write(*pins::vacuum_pump, rm::vacuum_pump(registers) != 0);
    }
    hw::MotorPwm::set_duty_permille<3>(rm::conveyor_pwm(registers));
    hw::MotorPwm::set_duty_permille<4>(rm::compressor_pwm(registers));
}

void inputs() noexcept {
    rm::set_lightbarrier1(registers, read(pins::light_barrier1) ? 0 : 1);  // NPN: active low
    rm::set_lightbarrier2(registers, read(pins::light_barrier2) ? 0 : 1);
    if constexpr (pins::rev1) {
        rm::set_lightbarrier3(registers, read(*pins::light_barrier3) ? 0 : 1);
    }
    rm::set_pressure_raw(registers, hw::Pressure::read());  // continuous conversion
}

// ---- steppers: TMC2209 on UART5, step timing in TIM17 (1) and TIM16 (2) -----------------------------
hw::StepperMotion stepper_motion[2];
dev::Tmc2209<hw::StepperUart, KernelClock> tmc[2]{{stepper_uart, 0b10}, {stepper_uart, 0b00}};
bool homing_active[2] = {};

template <class Timer>
void step_irq(hw::StepperMotion& m, rtos::port::stm32h5::gpio::Pin step) noexcept {
    if (!Timer::take_update()) {
        return;
    }
    h5::gpio::toggle(step);
    if (const std::uint32_t ticks = m.next(); ticks != 0) {
        Timer::set_period(ticks);
    } else {
        Timer::stop();
    }
}

template <class Timer>
void move(hw::StepperMotion& m, rtos::port::stm32h5::gpio::Pin dir, std::int32_t steps) noexcept {
    const std::uint32_t first = m.begin(steps);
    if (first == 0) {
        return;
    }
    write(dir, m.forward());
    Timer::set_period(first);
    Timer::set_counter(0);
    Timer::start();
}

std::uint16_t stepper_status(bool enabled, const hw::StepperMotion& m, std::int32_t target) noexcept {
    return static_cast<std::uint16_t>((enabled ? 1u : 0u) | (m.moving() ? 2u : 0u) |
                                      (!m.moving() && m.position() == target ? 8u : 0u));
}

void homing(std::size_t i, std::uint16_t start_reg, std::uint32_t now) noexcept {
    if (!homing_active[i]) {
        if (registers.holding(start_reg) == 1 && !stepper_motion[i].moving()) {
            homing_active[i] = tmc[i].start_homing({.speed = -200}, now);
        }
        return;
    }
    using R = dev::Tmc2209<hw::StepperUart, KernelClock>::HomingResult;
    switch (tmc[i].tick_homing(now)) {
        case R::InProgress: return;
        case R::Done:
            stepper_motion[i].set_position(0);
            registers.set_holding(start_reg, 0);
            break;
        case R::Timeout: registers.set_holding(start_reg, 2); break;
        case R::Error: registers.set_holding(start_reg, 3); break;
    }
    homing_active[i] = false;
}

void steppers(std::uint32_t now) noexcept {
    homing(0, rm::holding::stepper1_start_homing, now);
    homing(1, rm::holding::stepper2_start_homing, now);
    const bool enabled = rm::stepper_enable(registers) != 0 || homing_active[0] || homing_active[1];
    write(pins::stepper_enable, !enabled);  // active low
    const std::int32_t target[2] = {rm::stepper1_target(registers), rm::stepper2_target(registers)};
    // a new target starts when the current move is done (no change of plan mid-ramp)
    if (enabled && !homing_active[0] && !stepper_motion[0].moving() && stepper_motion[0].position() != target[0]) {
        move<hw::Stepper1Timer>(stepper_motion[0], pins::stepper1_dir, target[0] - stepper_motion[0].position());
    }
    if (enabled && !homing_active[1] && !stepper_motion[1].moving() && stepper_motion[1].position() != target[1]) {
        move<hw::Stepper2Timer>(stepper_motion[1], pins::stepper2_dir, target[1] - stepper_motion[1].position());
    }
    rm::set_stepper1_position(registers, stepper_motion[0].position());
    rm::set_stepper2_position(registers, stepper_motion[1].position());
    rm::set_stepper1_status(registers, stepper_status(enabled, stepper_motion[0], target[0]));
    rm::set_stepper2_status(registers, stepper_status(enabled, stepper_motion[1], target[1]));
}

// ---- WS2812: CH1 strip of 8 LEDs (patterns), CH2 one LED (palette) --------------------------------
constexpr std::size_t strip_leds = 8;
RTOS_DMA_BUFFER std::array<std::uint16_t, dev::ws2812_slots(strip_leds)> strip_slots;
RTOS_DMA_BUFFER std::array<std::uint16_t, dev::ws2812_slots(1)> single_slots;
std::uint16_t last_pattern[2] = {0xFFFF, 0xFFFF};
constexpr std::array<std::uint32_t, 8> palette{0x000000, 0xFF0000, 0x00FF00, 0x0000FF,
                                               0xFFFFFF, 0xFFFF00, 0x00FFFF, 0xFF00FF};

std::array<std::uint32_t, strip_leds> strip_pattern(std::uint16_t p) noexcept {
    std::array<std::uint32_t, strip_leds> c{};
    switch (p % 5) {
        case 1: c.fill(0xFF0000); break;  // red
        case 2: c.fill(0xFFFF00); break;  // yellow
        case 3: c.fill(0x00FF00); break;  // green
        case 4:
            for (std::size_t i = 0; i < c.size(); ++i) c[i] = i % 2 == 0 ? 0x0000FF : 0xFFFFFF;  // blue/white
            break;
        default: break;  // off
    }
    return c;
}

void leds() noexcept {
    const std::uint16_t p1 = rm::ws2812_ch1_pattern(registers), p2 = rm::ws2812_ch2_pattern(registers);
    if (p1 != last_pattern[0] && !hw::LedStripDma::busy()) {
        last_pattern[0] = p1;
        const auto colours = strip_pattern(p1);
        dev::ws2812_encode(colours, strip_slots, hw::ws2812);
        hw::LedStripDma::stop();
        (void)hw::LedStripDma::start(hw::LedTimer::compare_address<1>(), strip_slots.data(), strip_slots.size());
    }
    if (p2 != last_pattern[1] && !hw::LedSingleDma::busy()) {
        last_pattern[1] = p2;
        const std::array<std::uint32_t, 1> colour{palette[p2 % palette.size()]};
        dev::ws2812_encode(colour, single_slots, hw::ws2812);
        hw::LedSingleDma::stop();
        (void)hw::LedSingleDma::start(hw::LedTimer::dmar_address(), single_slots.data(), single_slots.size());
    }
}

// ---- health, diagnostics ---------------------------------------------------------------------------
void health(std::uint32_t now) noexcept {
    const net::Status s = net::status();
    rm::set_eth_link_status(registers, s.ethernet_link ? 1 : 0);
    rm::set_eth_link_speed(registers, s.ethernet_mbps);
    rm::set_eth_link_duplex(registers, s.ethernet_full_duplex ? 1 : 0);
    rm::set_eth_rx_count(registers, s.rx_frames);
    rm::set_eth_tx_count(registers, s.tx_frames);
    namespace bit = rm::input::health_state_bit;
    std::uint16_t h = 0;
    if (!s.ethernet_link) h |= 1u << bit::eth_link_down;
    if (!supply_present) h |= 1u << bit::power_fault;
    if (rm::can_bus_state(registers) == 2) h |= 1u << bit::can_error;
    if (rm::cpu_temperature_c(registers) > 85) h |= 1u << bit::overtemperature;
    if (h == 0) h = 1u << bit::ok;
    rm::set_health_state(registers, h);
    rm::set_timer_tick(registers, static_cast<std::uint16_t>(now));
    rm::set_free_heap_kib(registers, 0);  // no heap
}

std::uint32_t next_heartbeat = 0, next_pneumatics = 0;

void heartbeat(std::uint32_t now) noexcept {
    if (static_cast<std::int32_t>(now - next_heartbeat) < 0) {
        return;
    }
    next_heartbeat = now + 3'000;
    if (supply_present) {
        RTOS_LOG_INFO("heartbeat: CPU %d C, bus %u mV, %d mA", rm::cpu_temperature_c(registers),
                      rm::pwr_bus_voltage_mv(registers), static_cast<std::int16_t>(rm::pwr_current_ma(registers)));
    } else {
        RTOS_LOG_INFO("heartbeat: CPU %d C, supply monitor missing", rm::cpu_temperature_c(registers));
    }
}

void pneumatics_broadcast(std::uint32_t now) noexcept {
    if (static_cast<std::int32_t>(now - next_pneumatics) < 0) {
        return;
    }
    next_pneumatics = now + 500;
    const WsProtocol::pneumatics::PressureControlFeedback::Payload p{
        .pressureRaw = rm::pressure_raw(registers),
        .compressorPwmPermille = rm::compressor_pwm(registers),
        .valve1Open = rm::valve1(registers) != 0,
        .valve2Open = rm::valve2(registers) != 0,
        .valve3Open = rm::valve3(registers) != 0};
    std::array<std::uint8_t, 16> buf{};
    if (const std::size_t n = WsProtocol::pneumatics::PressureControlFeedback::Encode(p, buf.data(), buf.size()); n > 0) {
        web::broadcast_from_io(std::span(buf).first(n));
    }
}

void enable_irq(unsigned irq, std::uint8_t priority) noexcept {
    cm33::nvic::ipr(irq) = static_cast<std::uint8_t>(priority << (8 - RTOS_CM33_PRIO_BITS));
    cm33::nvic::enable(irq);
}

}  // namespace

void setup() noexcept {
    h5::gpio::configure<pins::io_pins>();
    hw::I2c1::init();
    hw::I2c2::init();
    // the scans first: the sensors' own probing (NACKs) must not colour them
    scan<hw::I2c1>(1, scans[0]);
    scan<hw::I2c2>(2, scans[1]);
    scan<hw::I2c4>(4, scans[2]);

    supply_present = hw::Supply::probe() && hw::Supply::init();
    tof_present[0] = hw::Tof1::probe();
    tof_present[1] = hw::Tof2::probe();
    tof_present[2] = hw::Tof3::probe();
    color_present = hw::Color::probe() && hw::Color::power_on();
    kernel.sleep(3);  // PON before AEN (2.4 ms)
    color_present = color_present && hw::Color::enable();
    for (std::size_t i = 0; i < 3; ++i) {
        if (!tof_present[i]) {
            RTOS_LOG_WARN("ToF %u (VL53L0X) not found", static_cast<unsigned>(i + 1));
        }
    }
    if (!color_present) {
        RTOS_LOG_WARN("colour sensor (TCS34725, I2C1) not found");
    }

    hw::Spi2::init();
    (void)hw::Scale::init();
    if (!hw::Pressure::init()) {
        RTOS_LOG_WARN("ADC1: calibration failed");
    }
    hw::Pressure::start_continuous();
    hw::CpuTemperature::init();
    if (!hw::Can::init()) {
        RTOS_LOG_WARN("FDCAN1: init failed");
    }

    hw::MotorPwm::init();
    hw::MotorPwm::configure_pwm<3>();
    hw::MotorPwm::configure_pwm<4>();
    hw::MotorPwm::start();

    stepper_uart.start();
    enable_irq(hw::StepperUart::irq, irq_priority_net);
    for (std::size_t i = 0; i < 2; ++i) {
        if (!tmc[i].init(rtos::devices::Tmc2209Microsteps::M16)) {
            RTOS_LOG_WARN("stepper %u: TMC2209 does not answer (steps still go out)", static_cast<unsigned>(i + 1));
        }
        (void)tmc[i].set_current(4, 12, 6);
    }
    hw::Stepper1Timer::init();
    hw::Stepper2Timer::init();
    hw::Stepper1Timer::enable_update_irq();
    hw::Stepper2Timer::enable_update_irq();
    enable_irq(hw::Stepper1Timer::irq, irq_priority_motion);
    enable_irq(hw::Stepper2Timer::irq, irq_priority_motion);

    hw::LedTimer::init();
    hw::LedTimer::configure_pwm<1>();
    hw::LedTimer::configure_pwm<2>();
    hw::LedTimer::enable_compare_dma<1>();
    hw::LedTimer::enable_update_dma();
    hw::LedTimer::configure_dma_burst(0x38, 1, 1);  // DMAR -> CCR2 at each update
    hw::LedTimer::start();
    hw::LedStripDma::init();
    hw::LedSingleDma::init();

    roarm::setup();
}

void cycle(std::uint32_t now) noexcept {
    inputs();
    power();
    tof<hw::Tof1>(0, rm::input::tof1_distance_mm, rm::input::tof1_status);
    tof<hw::Tof2>(1, rm::input::tof2_distance_mm, rm::input::tof2_status);
    tof<hw::Tof3>(2, rm::input::tof3_distance_mm, rm::input::tof3_status);
    color();
    scale();
    cpu_temperature();
    can();
    boot::poll_pd_events();
    steppers(now);
    roarm::cycle(now);
    leds();
    outputs();
    health(now);
    heartbeat(now);
    pneumatics_broadcast(now);
}

std::span<const std::uint8_t, 16> i2c_scan(unsigned bus) noexcept {
    return scans[bus == 1 ? 0 : (bus == 2 ? 1 : 2)];
}

}  // namespace fcu::io

extern "C" void UART5_IRQHandler() noexcept { fcu::io::stepper_uart.on_irq(); }
extern "C" void TIM17_IRQHandler() noexcept {
    fcu::io::step_irq<fcu::hw::Stepper1Timer>(fcu::io::stepper_motion[0], fcu::pins::stepper1_step);
}
extern "C" void TIM16_IRQHandler() noexcept {
    fcu::io::step_irq<fcu::hw::Stepper2Timer>(fcu::io::stepper_motion[1], fcu::pins::stepper2_step);
}
