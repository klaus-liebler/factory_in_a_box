// The I/O cycle (see io.hpp).
#include "io.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>

#include "boot.hpp"
#include "hardware.hpp"
#include "network.hpp"
#include "plc_project.hpp"
#include "ethercat.hpp"
#include "malibotti/malibotti.hpp"
#include "roarm/roarm.hpp"
#include "steppers/steppers.hpp"
#include "rtos/time.hpp"
#include "system.hpp"
#include "terminal.hpp"
#include "web.hpp"
#include "ws2812.hpp"
#include "ws_protocol.hh"

namespace fcu::io {
namespace {

namespace chip = board::chip;
namespace dev = rtos::devices;
namespace rm = register_map;
using chip::gpio::read;
using chip::gpio::write;

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
    if constexpr (pins::vacuum_valve.has_value() && pins::vacuum_pump.has_value()) {
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
    if constexpr (hw::has_barrier3) {
        rm::set_lightbarrier3(registers, !read(*pins::light_barrier3) || barrier_interrupted[2].exchange(false) ? 1 : 0);
    }
    if constexpr (pins::gpio1.has_value()) {
        rm::set_gpio1(registers, read(*pins::gpio1) ? 1 : 0);  // active high
        rm::set_gpio2(registers, read(*pins::gpio2) ? 1 : 0);
    }
    rm::set_pressure_raw(registers, hw::pressure_raw());  // continuous conversion
}

// ---- WS2812: CH1 strip of 8 LEDs, CH2 one LED (ws2812.hpp: patterns and single LEDs) ---------------
RTOS_DMA_BUFFER std::array<std::uint16_t, dev::ws2812_slots(ws2812::strip_leds)> strip_slots;
RTOS_DMA_BUFFER std::array<std::uint16_t, dev::ws2812_slots(1)> single_slots;
ws2812::Strip<Kernel::Lock> strip;
ws2812::Single<Kernel::Lock> single;

void set_pixel_ready(std::uint16_t pixel_reg) noexcept {
    constexpr std::array<std::uint16_t, 2> ready{ws2812::pixel_ready >> 16, ws2812::pixel_ready & 0xFFFF};
    registers.set_holdings(pixel_reg, ready);
}

/// A write that covers both words of a pixel register (one write: never half a value - a write of one
/// word alone is no command).
template <class Channel>
void pixel_written(Channel& channel, std::uint16_t pattern_reg, std::uint16_t pixel_reg, std::uint16_t start,
                   std::uint16_t count) noexcept {
    if (start > pixel_reg || start + count < pixel_reg + 2) return;
    const std::uint32_t value = (std::uint32_t{registers.holding(pixel_reg)} << 16) | registers.holding(pixel_reg + 1);
    if (channel.pixel(registers.holding(pattern_reg), value)) {
        registers.set_holding(pattern_reg, ws2812::individual);
    } else {
        RTOS_LOG_WARN("ws2812: LED %u does not exist (register %u) - ignored", ws2812::pixel_index(value), pixel_reg);
    }
    set_pixel_ready(pixel_reg);
}

/// Starts the DMA for a changed frame (when the previous transfer is done; else the next cycle).
template <class Dma, class Channel, std::size_t Slots>
void show(Channel& channel, std::uint16_t pattern, std::array<std::uint16_t, Slots>& slots, std::uint32_t target) noexcept {
    channel.sync(pattern);
    if (Dma::busy()) return;
    const auto frame = channel.take();
    if (!frame) return;
    dev::ws2812_encode(*frame, slots, hw::ws2812);
    Dma::stop();
    (void)Dma::start(target, slots.data(), slots.size());
}

void leds() noexcept {
    show<hw::LedStripDma>(strip, rm::ws2812_ch1_pattern(registers), strip_slots, hw::LedTimer::compare_address<1>());
    show<hw::LedSingleDma>(single, rm::ws2812_ch2_pattern(registers), single_slots, hw::LedTimer::dmar_address());
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
}

std::uint32_t next_heartbeat = 0, next_pneumatics = 0;

void heartbeat(std::uint32_t now) noexcept {
    if (!rtos::reached(now, next_heartbeat)) {
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
    if (!rtos::reached(now, next_pneumatics)) {
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
    if constexpr (hw::has_barrier3) {
        hw::Barrier3::init();
        hw::Barrier3::enable();
        enable_irq(hw::Barrier3::irq, irq_priority_net);
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

    steppers::setup(supply_up);

    hw::LedTimer::init();
    hw::LedTimer::configure_pwm<1>();
    hw::LedTimer::configure_pwm<2>();
    hw::LedTimer::enable_compare_dma<1>();
    hw::LedTimer::enable_update_dma();
    hw::configure_led_burst<hw::LedTimer>();  // DMAR -> CCR2 at each update
    hw::LedTimer::start();
    set_pixel_ready(rm::holding::ws2812_ch1_pixel);  // before the network starts: no write seen yet
    set_pixel_ready(rm::holding::ws2812_ch2_pixel);
    hw::LedStripDma::init();
    hw::LedSingleDma::init();

    roarm::setup();
    malibotti::setup();
    ethercat::start();  // its own thread: releases the ESC from reset (until now it was held there)
}

void set_event_thread(Kernel::Thread& thread) noexcept { event_thread = &thread; }

void holding_written(void*, std::uint16_t start, std::uint16_t count) noexcept {
    pixel_written(strip, rm::holding::ws2812_ch1_pattern, rm::holding::ws2812_ch1_pixel, start, count);
    pixel_written(single, rm::holding::ws2812_ch2_pattern, rm::holding::ws2812_ch2_pixel, start, count);
}

void events(std::uint32_t bits) noexcept {
    if ((bits & event_stall) != 0) {
        steppers::homing_poll(kernel.now());  // the stop at once, not at the next poll
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
    steppers::cycle(now);
    roarm::cycle(now);
    malibotti::cycle(now);
    plc::project::cycle();
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

namespace fcu::io {
namespace {
/// DIAG of stepper `i` rose: latched for its homing, which the I/O thread runs at once.
void stall_from_isr(std::size_t i) noexcept {
    steppers::stall_from_isr(i);
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
    if constexpr (hw::has_barrier3 && hw::Barrier3::irq == Irq) {
        if (hw::Barrier3::take().falling) barrier_interrupted[2] = true;
    }
    if constexpr (hw::has_stepper_diag && hw::Diag1::irq == Irq) {
        if (hw::Diag1::take().rising) stall_from_isr(0);
    }
    if constexpr (hw::has_stepper_diag && hw::Diag2::irq == Irq) {
        if (hw::Diag2::take().rising) stall_from_isr(1);
    }
    if constexpr (hw::has_ethercat_irq && hw::EscIrq::irq == Irq) {
        if (hw::EscIrq::take()) ethercat::irq_from_isr();
    }
}
}  // namespace
}  // namespace fcu::io

// The EXTI vectors of the revision (FCU_EXTI_HANDLERS, its peripherals.hpp), each to exti_irq<IRQ>
// (hardware.hpp checks that every line used is among them).
#define FCU_DEFINE_EXTI_HANDLER(handler, irq) \
    extern "C" void handler() noexcept { fcu::io::exti_irq<irq>(); }
FCU_EXTI_HANDLERS(FCU_DEFINE_EXTI_HANDLER)
#undef FCU_DEFINE_EXTI_HANDLER

/// HX711: the SPI's end of transfer - the reading that the DOUT edge started.
extern "C" void SPI2_IRQHandler() noexcept {
    (void)fcu::hw::Scale::complete();
    (void)fcu::hw::ScaleReady::take();  // the edges of the data bits
    fcu::hw::ScaleReady::enable();
}

extern "C" void I2C1_EV_IRQHandler() noexcept { fcu::hw::I2c1::on_event_irq(); }
extern "C" void I2C1_ER_IRQHandler() noexcept { fcu::hw::I2c1::on_error_irq(); }
extern "C" void FCU_I2C_TOF2_EV_IRQ_HANDLER() noexcept { fcu::hw::I2cTof2::on_event_irq(); }
extern "C" void FCU_I2C_TOF2_ER_IRQ_HANDLER() noexcept { fcu::hw::I2cTof2::on_error_irq(); }
extern "C" void FCU_I2C_SUPPLY_EV_IRQ_HANDLER() noexcept { fcu::hw::I2cSupply::on_event_irq(); }
extern "C" void FCU_I2C_SUPPLY_ER_IRQ_HANDLER() noexcept { fcu::hw::I2cSupply::on_error_irq(); }

