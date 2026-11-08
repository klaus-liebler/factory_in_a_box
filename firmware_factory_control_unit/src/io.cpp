// The I/O cycle (see io.hpp).
#include "io.hpp"

#include <array>
#include <atomic>
#include <cstring>

#include "boot.hpp"
#include "hardware.hpp"
#include "network.hpp"
#include "rtos/devices/tmc2209.hpp"
#include "roarm/roarm.hpp"
#include "system.hpp"
#include "terminal.hpp"
#include "web.hpp"
#include "ws_protocol.hh"

namespace fcu::io {
namespace {

namespace chip = board::chip;
namespace dev = rtos::devices;
namespace rm = register_map;
using chip::gpio::read;
using chip::gpio::write;

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

// ---- events from interrupts -----------------------------------------------------------------------
Kernel::Thread* event_thread = nullptr;
std::atomic<bool> barrier_interrupted[3] = {};  // latched by EXTI, taken by inputs()
std::atomic<std::uint32_t> scale_readings_seen{0};

// ---- sensors ------------------------------------------------------------------------------------
bool supply_present = false, tof_present[3] = {}, color_present = false;
unsigned supply_failures = 0;
std::uint32_t last_supply_read = 0;

/// The INA226 measurement into the registers; acknowledge() releases ALERT for the next one.
void read_supply(std::uint32_t now) noexcept {
    last_supply_read = now;
    const auto m = hw::Supply::read();
    (void)hw::Supply::acknowledge();
    if (m) {
        supply_failures = 0;
        rm::set_pwr_bus_voltage_mv(registers, static_cast<std::uint16_t>(m->bus_mv));
        rm::set_pwr_shunt_voltage_uv(registers, static_cast<std::uint16_t>(m->shunt_uv));
        rm::set_pwr_current_ma(registers, static_cast<std::uint16_t>(m->current_ma));
        rm::set_pwr_power_mw(registers, static_cast<std::uint16_t>(m->power_mw));
    } else if (++supply_failures <= 3 || supply_failures % 20 == 0) {
        RTOS_LOG_WARN("INA226: read failed (%u in a row)", supply_failures);
        hw::I2cSupply::recover();
    }
}

void power(std::uint32_t now) noexcept {
    rm::set_pwr_status(registers, supply_present ? 0 : 1);
    // normally ALERT wakes the thread (events()); without one for 200 ms - a lost edge, a failed
    // acknowledge - read anyway, which also releases ALERT
    if (supply_present && now - last_supply_read >= 200) {
        read_supply(now);
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

/// The HX711 is read by interrupts alone (DOUT edge -> SPI -> end of transfer); here only the
/// average goes into the registers, and status 1 when no reading came for a second.
std::uint32_t scale_last_count = 0, scale_last_change = 0;
void scale(std::uint32_t now) noexcept {
    const std::uint32_t n = hw::Scale::readings();
    if (n != scale_last_count) {
        scale_last_count = n;
        scale_last_change = now;
    }
    const auto w = hw::Scale::average();
    if (w && now - scale_last_change < 1'000) {
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
    chip::CanFrame f;
    while (hw::Can::receive(f)) {
        ++can_rx;
    }
    const chip::CanStatus s = hw::Can::status();
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
    // NPN: active low. An interruption since the last cycle counts too, however short it was
    // (EXTI latches it): one cycle of 1 for an object that passed between two looks.
    rm::set_lightbarrier1(registers, !read(pins::light_barrier1) || barrier_interrupted[0].exchange(false) ? 1 : 0);
    rm::set_lightbarrier2(registers, !read(pins::light_barrier2) || barrier_interrupted[1].exchange(false) ? 1 : 0);
    if constexpr (pins::rev1) {
        rm::set_lightbarrier3(registers, !read(*pins::light_barrier3) || barrier_interrupted[2].exchange(false) ? 1 : 0);
    }
    rm::set_pressure_raw(registers, hw::pressure_raw());  // continuous conversion
}

// ---- steppers: TMC2209 on UART5, steps by TIM2 (1) and TIM3 (2) with DMA ----------------------------
RTOS_DMA_BUFFER hw::Stepper1 stepper1;  // the period buffers are DMA sources
RTOS_DMA_BUFFER hw::Stepper2 stepper2;
dev::Tmc2209<hw::StepperUart, KernelClock> tmc[2]{{stepper_uart, 0b10}, {stepper_uart, 0b00}};
bool homing_active[2] = {};
bool homed[2] = {};  ///< the position is referenced: the last homing found the end stop
std::atomic<bool> diag_stall[2] = {};  ///< DIAG rose (EXTI), taken by homing()

template <class Stepper>
void move(Stepper& s, pins::Pin dir, std::int32_t steps) noexcept {
    write(dir, steps > 0);
    (void)s.start(steps);
}

/// Bit0 enabled, Bit1 moving, Bit2 homing failed, Bit3 in position, Bit4 homing, Bit5 homed.
template <class Stepper>
std::uint16_t stepper_status(std::size_t i, bool enabled, const Stepper& m, std::int32_t target,
                             std::uint16_t start_reg) noexcept {
    return static_cast<std::uint16_t>((enabled ? 1u : 0u) | (m.moving() ? 2u : 0u) |
                                      (registers.holding(start_reg) >= 2 ? 4u : 0u) |
                                      (!m.moving() && m.position() == target ? 8u : 0u) |
                                      (homing_active[i] ? 16u : 0u) | (homed[i] ? 32u : 0u));
}

template <class Stepper, class SetTarget>
void homing(std::size_t i, Stepper& stepper, std::uint16_t start_reg, SetTarget set_target, std::uint32_t now) noexcept {
    if (!homing_active[i]) {
        if (registers.holding(start_reg) == 1 && !stepper.moving()) {
            diag_stall[i] = false;
            homing_active[i] = tmc[i].start_homing({.speed = -200}, now);
            homed[i] = false;
            if (!homing_active[i]) {
                registers.set_holding(start_reg, 3);
                RTOS_LOG_WARN("stepper %u: homing not started (TMC2209 does not answer)", static_cast<unsigned>(i + 1));
            }
        }
        return;
    }
    using R = dev::Tmc2209<hw::StepperUart, KernelClock>::HomingResult;
    const unsigned n = static_cast<unsigned>(i + 1);
    switch (tmc[i].tick_homing(now, diag_stall[i].exchange(false))) {
        case R::InProgress: return;
        case R::Done:
            stepper.set_position(0);
            registers.set_holding(start_reg, 0);
            set_target(registers, 0);  // no move right after it to a target written before
            homed[i] = true;
            RTOS_LOG_INFO("stepper %u: homed (StallGuard at the end stop, seen %s), position 0", n,
                          tmc[i].stall_by_signal() ? "on DIAG" : "over UART");
            if (pins::stepper_diag && !tmc[i].stall_by_signal()) {
                RTOS_LOG_WARN("stepper %u: the stall did not show on DIAG - is the DIAG wire there?", n);
            }
            break;
        case R::Timeout:
            registers.set_holding(start_reg, 2);
            RTOS_LOG_WARN("stepper %u: homing timed out, no end stop found", n);
            break;
        case R::Error:
            registers.set_holding(start_reg, 3);
            RTOS_LOG_WARN("stepper %u: homing failed (%s)", n,
                          tmc[i].stall_by_signal() ? "DIAG rose from a driver error" : "TMC2209 UART");
            break;
    }
    homing_active[i] = false;
}

void homing_both(std::uint32_t now) noexcept {
    homing(0, stepper1, rm::holding::stepper1_start_homing, [](auto& b, std::int32_t v) { rm::set_stepper1_target(b, v); }, now);
    homing(1, stepper2, rm::holding::stepper2_start_homing, [](auto& b, std::int32_t v) { rm::set_stepper2_target(b, v); }, now);
}

void steppers(std::uint32_t now) noexcept {
    homing_both(now);
    const bool enabled = rm::stepper_enable(registers) != 0 || homing_active[0] || homing_active[1];
    write(pins::stepper_enable, !enabled);  // active low
    const std::int32_t target[2] = {rm::stepper1_target(registers), rm::stepper2_target(registers)};
    // a new target starts when the current move is done (no change of plan mid-ramp); only on a
    // referenced axis: without homing the position means nothing, the target waits
    if (enabled && homed[0] && !homing_active[0] && !stepper1.moving() && stepper1.position() != target[0]) {
        move(stepper1, pins::stepper1_dir, target[0] - stepper1.position());
    }
    if (enabled && homed[1] && !homing_active[1] && !stepper2.moving() && stepper2.position() != target[1]) {
        move(stepper2, pins::stepper2_dir, target[1] - stepper2.position());
    }
    rm::set_stepper1_position(registers, stepper1.position());
    rm::set_stepper2_position(registers, stepper2.position());
    rm::set_stepper1_status(registers, stepper_status(0, enabled, stepper1, target[0], rm::holding::stepper1_start_homing));
    rm::set_stepper2_status(registers, stepper_status(1, enabled, stepper2, target[1], rm::holding::stepper2_start_homing));
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

}  // namespace

void setup(bool supply_up) noexcept {
    hw::close_analog_switches();
    chip::gpio::configure<pins::io_pins>();
    hw::I2c1::init();
    hw::I2cTof2::init();
    hw::enable_i2c_irqs<hw::I2c1>();
    hw::enable_i2c_irqs<hw::I2cTof2>();
    // the scans first: the sensors' own probing (NACKs) must not colour them
    scan<hw::I2c1>(1, scans[0]);
    scan<hw::I2cTof2>(hw::i2c_tof2_bus, scans[1]);
    scan<hw::I2cSupply>(hw::i2c_supply_bus, scans[2]);

    supply_present = hw::Supply::probe() && hw::Supply::init() && hw::Supply::enable_conversion_ready_alert();
    hw::SupplyAlert::init();
    hw::SupplyAlert::enable();
    enable_irq(hw::SupplyAlert::irq, irq_priority_net);  // kernel-aware: notifies the I/O thread
    hw::Barrier1::init();
    hw::Barrier2::init();
    hw::Barrier1::enable();
    hw::Barrier2::enable();
    enable_irq(hw::Barrier1::irq, irq_priority_net);
    enable_irq(hw::Barrier2::irq, irq_priority_net);
    if constexpr (pins::rev1) {
        hw::Barrier3::init();
        hw::Barrier3::enable();
        enable_irq(hw::Barrier3::irq, irq_priority_net);
    }
    if constexpr (pins::stepper_diag) {
        hw::Diag1::init();
        hw::Diag2::init();
        hw::Diag1::enable();
        hw::Diag2::enable();
        enable_irq(hw::Diag1::irq, irq_priority_net);  // kernel-aware: wakes the I/O thread
        enable_irq(hw::Diag2::irq, irq_priority_net);
    }
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
    hw::ScaleReady::init();
    enable_irq(hw::ScaleReady::irq, irq_priority_net);
    enable_irq(hw::Spi2::irq, irq_priority_net);
    hw::ScaleReady::enable();
    hw::ScaleReady::trigger();  // DOUT may be low already: the handler looks at the level
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
    bool tmc_ok[2] = {};
    for (std::size_t i = 0; i < 2; ++i) {
        tmc_ok[i] = tmc[i].init(rtos::devices::Tmc2209Microsteps::M16);
        if (!tmc_ok[i]) {
            RTOS_LOG_WARN("stepper %u: TMC2209 does not answer (steps still go out)", static_cast<unsigned>(i + 1));
        }
        (void)tmc[i].set_current(4, 12, 6);
    }
    // Sensorless homing at the start, both at once: the I/O cycle runs it (homing()). Not without
    // the motor supply: StallGuard could then report an end stop that is not there. A stepper that
    // is not homed says so in its status (Bit5 clear); homing again: write 1 to StartHoming.
    const std::uint16_t start_homing[2] = {rm::holding::stepper1_start_homing, rm::holding::stepper2_start_homing};
    for (std::size_t i = 0; i < 2; ++i) {
        if (supply_up && tmc_ok[i]) {
            registers.set_holding(start_homing[i], 1);
        }
    }
    if (!supply_up) {
        RTOS_LOG_WARN("steppers: no homing at the start (motor supply not up)");
    }
    stepper1.init();
    stepper2.init();
    for (const unsigned irq : {unsigned{hw::Stepper1Timer::irq}, unsigned{hw::Stepper2Timer::irq},
                               unsigned{hw::Stepper1Dma::irq}, unsigned{hw::Stepper2Dma::irq}}) {
        enable_irq(irq, irq_priority_motion);
    }

    hw::LedTimer::init();
    hw::LedTimer::configure_pwm<1>();
    hw::LedTimer::configure_pwm<2>();
    hw::LedTimer::enable_compare_dma<1>();
    hw::LedTimer::enable_update_dma();
    hw::configure_led_burst();  // DMAR -> CCR2 at each update
    hw::LedTimer::start();
    hw::LedStripDma::init();
    hw::LedSingleDma::init();

    roarm::setup();
}

void set_event_thread(Kernel::Thread& thread) noexcept { event_thread = &thread; }

std::uint32_t homing_poll_ms() noexcept { return homing_active[0] || homing_active[1] ? homing_poll_period_ms : 0; }

void homing_poll(std::uint32_t now) noexcept { homing_both(now); }

void events(std::uint32_t bits) noexcept {
    if ((bits & event_stall) != 0) {
        homing_both(kernel.now());  // the stop at once, not at the next poll
    }
    if ((bits & event_supply) != 0 && supply_present) {
        read_supply(kernel.now());
    }
}

void cycle(std::uint32_t now) noexcept {
    inputs();
    power(now);
    tof<hw::Tof1>(0, rm::input::tof1_distance_mm, rm::input::tof1_status);
    tof<hw::Tof2>(1, rm::input::tof2_distance_mm, rm::input::tof2_status);
    tof<hw::Tof3>(2, rm::input::tof3_distance_mm, rm::input::tof3_status);
    color();
    scale(now);
    cpu_temperature();
    can();
    boot::poll_pd_events();
    steppers(now);
    roarm::cycle(now);
    terminal::cycle();
    leds();
    outputs();
    health(now);
    heartbeat(now);
    pneumatics_broadcast(now);
}

std::span<const std::uint8_t, 16> i2c_scan(I2cBus bus) noexcept { return scans[static_cast<std::size_t>(bus)]; }

unsigned i2c_bus_number(I2cBus bus) noexcept {
    switch (bus) {
        case I2cBus::Color: return 1;
        case I2cBus::Tof2: return hw::i2c_tof2_bus;
        case I2cBus::Supply: return hw::i2c_supply_bus;
    }
    return 0;
}

}  // namespace fcu::io

extern "C" void UART5_IRQHandler() noexcept { fcu::io::stepper_uart.on_irq(); }
namespace fcu::io {
namespace {
/// DIAG of stepper `i` rose: latched for homing(), which the I/O thread runs at once.
void stall_from_isr(std::size_t i) noexcept {
    diag_stall[i] = true;
    if (event_thread != nullptr) {
        kernel.notify_from_isr(*event_thread, event_slot, event_stall, rtos::NotifyAction::SetBits);
    }
}

/// The EXTI interrupt `Irq`: the lines of this revision that use it (STM32H563: one interrupt per
/// line, EXTI0..15 = 11..26; STM32H743: EXTI0..4 = 6..10, lines 5-9 share 23, lines 10-15 40).
template <unsigned Irq>
void exti_irq() noexcept {
    if constexpr (hw::ScaleReady::irq == Irq) {
        if (hw::ScaleReady::take()) {
            hw::ScaleReady::disable();  // the data bits toggle DOUT: no edges until the reading is done
            if (!hw::Scale::start_read()) {
                hw::ScaleReady::enable();  // not ready (the software trigger with DOUT high)
            }
        }
    }
    if constexpr (hw::SupplyAlert::irq == Irq) {
        if (hw::SupplyAlert::take() && event_thread != nullptr) {
            kernel.notify_from_isr(*event_thread, event_slot, event_supply, rtos::NotifyAction::SetBits);
        }
    }
    if constexpr (hw::Barrier1::irq == Irq) {
        if (hw::Barrier1::take().falling) barrier_interrupted[0] = true;
    }
    if constexpr (hw::Barrier2::irq == Irq) {
        if (hw::Barrier2::take().falling) barrier_interrupted[1] = true;
    }
    if constexpr (pins::rev1 && hw::Barrier3::irq == Irq) {
        if (hw::Barrier3::take().falling) barrier_interrupted[2] = true;
    }
    if constexpr (pins::stepper_diag && hw::Diag1::irq == Irq) {
        if (hw::Diag1::take().rising) stall_from_isr(0);
    }
    if constexpr (pins::stepper_diag && hw::Diag2::irq == Irq) {
        if (hw::Diag2::take().rising) stall_from_isr(1);
    }
}
}  // namespace
}  // namespace fcu::io

#if RTOS_FCU_REVISION == 3
extern "C" void EXTI0_IRQHandler() noexcept { fcu::io::exti_irq<6>(); }
extern "C" void EXTI1_IRQHandler() noexcept { fcu::io::exti_irq<7>(); }
extern "C" void EXTI2_IRQHandler() noexcept { fcu::io::exti_irq<8>(); }
extern "C" void EXTI3_IRQHandler() noexcept { fcu::io::exti_irq<9>(); }
extern "C" void EXTI4_IRQHandler() noexcept { fcu::io::exti_irq<10>(); }
extern "C" void EXTI9_5_IRQHandler() noexcept { fcu::io::exti_irq<23>(); }
extern "C" void EXTI15_10_IRQHandler() noexcept { fcu::io::exti_irq<40>(); }
#else
extern "C" void EXTI0_IRQHandler() noexcept { fcu::io::exti_irq<11>(); }
extern "C" void EXTI2_IRQHandler() noexcept { fcu::io::exti_irq<13>(); }
extern "C" void EXTI4_IRQHandler() noexcept { fcu::io::exti_irq<15>(); }
extern "C" void EXTI6_IRQHandler() noexcept { fcu::io::exti_irq<17>(); }
extern "C" void EXTI7_IRQHandler() noexcept { fcu::io::exti_irq<18>(); }
extern "C" void EXTI8_IRQHandler() noexcept { fcu::io::exti_irq<19>(); }
extern "C" void EXTI9_IRQHandler() noexcept { fcu::io::exti_irq<20>(); }
#endif

/// HX711: the SPI's end of transfer - the reading that the DOUT edge started.
extern "C" void SPI2_IRQHandler() noexcept {
    (void)fcu::hw::Scale::complete();
    (void)fcu::hw::ScaleReady::take();  // the edges of the data bits
    fcu::hw::ScaleReady::enable();
}

extern "C" void I2C1_EV_IRQHandler() noexcept { fcu::hw::I2c1::on_event_irq(); }
extern "C" void I2C1_ER_IRQHandler() noexcept { fcu::hw::I2c1::on_error_irq(); }
#if RTOS_FCU_REVISION == 3
extern "C" void I2C3_EV_IRQHandler() noexcept { fcu::hw::I2cSupply::on_event_irq(); }
extern "C" void I2C3_ER_IRQHandler() noexcept { fcu::hw::I2cSupply::on_error_irq(); }
extern "C" void I2C4_EV_IRQHandler() noexcept { fcu::hw::I2cTof2::on_event_irq(); }
extern "C" void I2C4_ER_IRQHandler() noexcept { fcu::hw::I2cTof2::on_error_irq(); }
extern "C" void DMA1_Stream2_IRQHandler() noexcept { fcu::io::stepper1.on_dma_irq(); }
extern "C" void DMA1_Stream3_IRQHandler() noexcept { fcu::io::stepper2.on_dma_irq(); }
#else
extern "C" void I2C2_EV_IRQHandler() noexcept { fcu::hw::I2cTof2::on_event_irq(); }
extern "C" void I2C2_ER_IRQHandler() noexcept { fcu::hw::I2cTof2::on_error_irq(); }
extern "C" void I2C4_EV_IRQHandler() noexcept { fcu::hw::I2cSupply::on_event_irq(); }
extern "C" void I2C4_ER_IRQHandler() noexcept { fcu::hw::I2cSupply::on_error_irq(); }
extern "C" void GPDMA1_CH4_IRQHandler() noexcept { fcu::io::stepper1.on_dma_irq(); }
extern "C" void GPDMA1_CH5_IRQHandler() noexcept { fcu::io::stepper2.on_dma_irq(); }
#endif
extern "C" void TIM2_IRQHandler() noexcept { (void)fcu::io::stepper1.on_timer_irq(); }
extern "C" void TIM3_IRQHandler() noexcept { (void)fcu::io::stepper2.on_timer_irq(); }
