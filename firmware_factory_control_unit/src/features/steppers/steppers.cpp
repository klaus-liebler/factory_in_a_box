// The steppers (see steppers.hpp).
#include "steppers/steppers.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <type_traits>

#include "hardware.hpp"
#include "rtos/devices/tmc2209.hpp"
#include "system.hpp"
#include "web.hpp"

namespace fcu::steppers {
namespace {

namespace chip = board::chip;
namespace dev = rtos::devices;
namespace tm = dev::tmc2209;
namespace rm = register_map;
namespace wst = WsProtocol::stepper;
using chip::gpio::write;

struct KernelClock {
    static std::uint32_t now_ms() noexcept { return kernel.now(); }
    static void sleep_ms(std::uint32_t ms) noexcept { kernel.sleep(ms); }
};
using Tmc = dev::Tmc2209<hw::StepperUart, KernelClock>;
using HomingResult = Tmc::HomingResult;

// TMC2209 x2 on UART5 (single wire with echo, nodes 0b10 and 0b00)
hw::StepperUart uart;

// The firmware keeps the TMC2209 settings (most of these registers cannot be read back) and the speed
// profile of each axis; the test bench reads and changes them.
constexpr wst::TmcSettings default_settings{
    .irun = 12, .ihold = 4, .iholdDelay = 6, .tpowerdown = 20, .spreadCycle = false, .tpwmThrs = 0, .pwmAutoscale = true,
    .pwmAutograd = true, .microsteps = 16, .sgThrs = 100, .tcoolThrs = 0, .coolStep = false, .semin = 5, .semax = 2,
    .seup = 0, .sedn = 0, .seimin = false};
/// 100 -> 1000 micro steps/s, sigmoid (about 0.5 s)
constexpr wst::Profile default_profile{.startHz = 100, .cruiseHz = 1000, .accel = 1800, .shape = wst::RampShape::Sigmoid};

/// The last homing (the test bench shows it).
enum class Homed : std::uint8_t { none = 0, done = 1, timeout = 2, error = 3 };

/// One axis: its step generator (timer + DMA), DIR pin, TMC2209 and registers, and its state.
template <class Stepper, class Timer>
struct Axis {
    const std::size_t index;
    Stepper& stepper;
    const pins::Pin dir;
    const std::uint16_t start_homing_reg;
    std::int32_t (*const target)(const Registers&) noexcept;
    void (*const set_target)(Registers&, std::int32_t) noexcept;
    void (*const set_position)(Registers&, std::int32_t) noexcept;
    void (*const set_status)(Registers&, std::uint16_t) noexcept;
    Tmc tmc;

    wst::TmcSettings settings = default_settings;
    wst::Profile profile = default_profile;
    dev::Tmc2209Homing home_params{};
    bool manual = false;         ///< the test bench controls the axis (the register target rests)
    bool homing = false;         ///< a homing runs
    bool homed = false;          ///< the position is referenced: the last homing found the end stop
    Homed last_homing = Homed::none;
    std::atomic<bool> diag_stall{false};  ///< DIAG rose (EXTI), taken by tick_homing()

    [[nodiscard]] unsigned number() const noexcept { return static_cast<unsigned>(index + 1); }

    void move(std::int32_t steps) noexcept {
        write(dir, steps > 0);
        (void)stepper.start(steps);
    }

    const char* configure_profile(const wst::Profile& p) noexcept {
        auto* m = stepper.motion();
        if (m == nullptr || homing) return "the axis is moving";
        const rtos::motion::StepProfile sp{
            .start_hz = p.startHz, .cruise_hz = p.cruiseHz, .accel = p.accel,
            .shape = p.shape == wst::RampShape::Linear ? rtos::motion::RampShape::Linear : rtos::motion::RampShape::Sigmoid};
        if (const char* e = m->configure(sp, Timer::tick_hz, Stepper::max_ticks)) return e;
        profile = p;
        return nullptr;
    }

    /// CoolStep and TCOOLTHRS as set (a homing switches CoolStep off and TCOOLTHRS to 450).
    void apply_cool() noexcept {
        const wst::TmcSettings& s = settings;
        const std::uint32_t coolconf =
            !s.coolStep ? 0u  // semin 0: CoolStep off
                        : (s.semin & 0xFu) | ((s.seup & 3u) << 5) | ((s.semax & 0xFu) << 8) | ((s.sedn & 3u) << 13) |
                              (s.seimin ? 1u << 15 : 0u);
        (void)tmc.write(tm::tcoolthrs, s.tcoolThrs & 0xFFFFFu);
        (void)tmc.write(tm::coolconf, coolconf);
    }

    /// All settings into the TMC2209 (kept also if it does not answer: written when it does).
    const char* apply_settings(const wst::TmcSettings& s) noexcept {
        const unsigned ms = s.microsteps;
        if (ms == 0 || ms > 256 || (ms & (ms - 1)) != 0) return "micro steps: 1, 2, 4 ... 256";
        if (s.irun > 31 || s.ihold > 31 || s.iholdDelay > 15 || s.semin > 15 || s.semax > 15 || s.seup > 3 || s.sedn > 3) {
            return "a value is out of its range";
        }
        unsigned mres = 0;
        while ((256u >> mres) != ms) ++mres;
        settings = s;
        home_params.stall_threshold = s.sgThrs;
        std::uint32_t pwm = tmc.read(tm::pwmconf).value_or(tm::pwmconf_reset);
        pwm = s.pwmAutoscale ? pwm | tm::pwmconf_autoscale : pwm & ~tm::pwmconf_autoscale;
        pwm = s.pwmAutograd ? pwm | tm::pwmconf_autograd : pwm & ~tm::pwmconf_autograd;
        (void)tmc.set_current(s.ihold, s.irun, s.iholdDelay);
        (void)tmc.write(tm::tpowerdown, s.tpowerdown);
        (void)tmc.set_spreadcycle(s.spreadCycle);
        (void)tmc.write(tm::tpwmthrs, s.tpwmThrs & 0xFFFFFu);
        (void)tmc.write(tm::pwmconf, pwm);
        (void)tmc.set_microsteps(static_cast<dev::Tmc2209Microsteps>(mres));
        (void)tmc.write(tm::sgthrs, s.sgThrs);
        apply_cool();
        // the writes are not answered: one read shows whether the chip is there
        return tmc.read(tm::gconf) ? nullptr : "the TMC2209 does not answer (the settings are kept)";
    }

    /// StartHoming: 1 starts a homing (Modbus, the test bench, the boot), the result 0 done, 2 timeout, 3 error.
    void tick_homing(std::uint32_t now) noexcept {
        if (!homing) {
            if (registers.holding(start_homing_reg) == 1 && !stepper.moving()) {
                diag_stall = false;
                homing = tmc.start_homing(home_params, now);
                homed = false;
                last_homing = Homed::none;
                if (!homing) {
                    last_homing = Homed::error;
                    registers.set_holding(start_homing_reg, 3);
                    RTOS_LOG_WARN("stepper %u: homing not started (TMC2209 does not answer)", number());
                }
            }
            return;
        }
        switch (tmc.tick_homing(now, diag_stall.exchange(false))) {
            case HomingResult::InProgress: return;
            case HomingResult::Done:
                last_homing = Homed::done;
                stepper.set_position(0);
                registers.set_holding(start_homing_reg, 0);
                set_target(registers, 0);  // no move right after it to a target written before
                homed = true;
                RTOS_LOG_INFO("stepper %u: homed (StallGuard at the end stop, seen %s), position 0", number(),
                              tmc.stall_by_signal() ? "on DIAG" : "over UART");
                if (hw::has_stepper_diag && !tmc.stall_by_signal()) {
                    RTOS_LOG_WARN("stepper %u: the stall did not show on DIAG - is the DIAG wire there?", number());
                }
                break;
            case HomingResult::Timeout:
                last_homing = Homed::timeout;
                registers.set_holding(start_homing_reg, 2);
                RTOS_LOG_WARN("stepper %u: homing timed out, no end stop found", number());
                break;
            case HomingResult::Error:
                last_homing = Homed::error;
                registers.set_holding(start_homing_reg, 3);
                RTOS_LOG_WARN("stepper %u: homing failed (%s)", number(),
                              tmc.stall_by_signal() ? "DIAG rose from a driver error" : "TMC2209 UART");
                break;
        }
        apply_cool();  // the homing switched CoolStep off
        homing = false;
    }

    void abort_homing(const char* by) noexcept {
        stepper.stop();
        if (!homing) return;
        tmc.abort_homing();
        homing = false;
        last_homing = Homed::error;
        registers.set_holding(start_homing_reg, 3);
        apply_cool();
        RTOS_LOG_WARN("stepper %u: homing aborted (%s)", number(), by);
    }

    /// A new register target starts when the current move is done (no change of plan mid-ramp); only on
    /// a referenced axis: without homing the position means nothing, the target waits.
    void follow_target(bool enabled) noexcept {
        const std::int32_t t = target(registers);
        if (enabled && homed && !manual && !homing && !stepper.moving() && stepper.position() != t) {
            move(t - stepper.position());
        }
        set_position(registers, stepper.position());
        // Bit0 enabled, Bit1 moving, Bit2 homing failed, Bit3 in position, Bit4 homing, Bit5 homed
        set_status(registers, static_cast<std::uint16_t>((enabled ? 1u : 0u) | (stepper.moving() ? 2u : 0u) |
                                                         (registers.holding(start_homing_reg) >= 2 ? 4u : 0u) |
                                                         (!stepper.moving() && stepper.position() == t ? 8u : 0u) |
                                                         (homing ? 16u : 0u) | (homed ? 32u : 0u)));
    }
};

RTOS_DMA_BUFFER hw::Stepper1 stepper1;  // the period buffers are DMA sources
RTOS_DMA_BUFFER hw::Stepper2 stepper2;

Axis<hw::Stepper1, hw::Stepper1Timer> axis1{
    .index = 0, .stepper = stepper1, .dir = pins::stepper1_dir, .start_homing_reg = rm::holding::stepper1_start_homing,
    .target = [](const Registers& r) noexcept { return rm::stepper1_target(r); },
    .set_target = [](Registers& r, std::int32_t v) noexcept { rm::set_stepper1_target(r, v); },
    .set_position = [](Registers& r, std::int32_t v) noexcept { rm::set_stepper1_position(r, v); },
    .set_status = [](Registers& r, std::uint16_t v) noexcept { rm::set_stepper1_status(r, v); },
    .tmc = {uart, 0b10}};
Axis<hw::Stepper2, hw::Stepper2Timer> axis2{
    .index = 1, .stepper = stepper2, .dir = pins::stepper2_dir, .start_homing_reg = rm::holding::stepper2_start_homing,
    .target = [](const Registers& r) noexcept { return rm::stepper2_target(r); },
    .set_target = [](Registers& r, std::int32_t v) noexcept { rm::set_stepper2_target(r, v); },
    .set_position = [](Registers& r, std::int32_t v) noexcept { rm::set_stepper2_position(r, v); },
    .set_status = [](Registers& r, std::uint16_t v) noexcept { rm::set_stepper2_status(r, v); },
    .tmc = {uart, 0b00}};

template <class F>
void for_each_axis(F f) noexcept {
    f(axis1);
    f(axis2);
}
/// `f(axis)` for axis `i` (0 or 1).
template <class F>
auto with_axis(std::size_t i, F f) noexcept {
    return i == 0 ? f(axis1) : f(axis2);
}

/// The drivers are on (ENN low): the register says so, or an axis homes or is under manual control.
bool drivers_enabled() noexcept {
    bool any = rm::stepper_enable(registers) != 0;
    for_each_axis([&](const auto& a) { any = any || a.homing || a.manual; });
    return any;
}

void tick_homings(std::uint32_t now) noexcept {
    for_each_axis([&](auto& a) { a.tick_homing(now); });
}

// ---- the test bench: WebSocket requests (queued by the network thread) -----------------------------
constexpr std::size_t max_request = std::max({wst::SettingsRequest::SettingsRequest_MAX_SIZE, wst::ManualRequest::ManualRequest_MAX_SIZE,
                                              wst::MoveRequest::MoveRequest_MAX_SIZE, wst::StopRequest::StopRequest_MAX_SIZE,
                                              wst::HomeRequest::HomeRequest_MAX_SIZE, wst::StatusRequest::StatusRequest_MAX_SIZE});
web::FrameQueue<max_request, 4> requests{"steppers"};
std::array<std::uint8_t, 256> answer;  // every response of ws::stepper is smaller

using WsProtocol::Responder;

/// Decodes the request and calls `handler(respond, axis, request)` for the axis it names.
template <class Payload, class Handler>
void dispatch(Responder& reply, std::span<const std::uint8_t> frame, bool (*decode)(const std::uint8_t*, std::size_t, Payload&),
              Handler handler) noexcept {
    WsProtocol::Dispatch(reply, frame.data(), frame.size(), decode, [handler](Responder& respond, const Payload& q) {
                             if (q.axis > 1) return respond.Error(WsProtocol::ErrorCode::Malformed, "no such axis");
                             with_axis(q.axis, [&](auto& a) { handler(respond, a, q); });
                         });
}

void on_settings(Responder& respond, auto& a, const wst::SettingsRequest::Payload& q) noexcept {
    const char* e = nullptr;
    if (q.write) {
        e = a.configure_profile(q.profile);
        if (e == nullptr) e = a.apply_settings(q.settings);
    }
    respond.Send(wst::SettingsResponse::Encode, wst::SettingsResponse::Payload{.ok = e == nullptr, .message = e ? e : "",
                                                                               .settings = a.settings, .profile = a.profile});
}

void on_manual(Responder& respond, auto& a, const wst::ManualRequest::Payload& q) noexcept {
    if (!q.on && a.manual) {
        a.set_target(registers, a.stepper.position());  // the register control again: from where the axis is
    }
    a.manual = q.on;
    RTOS_LOG_INFO("stepper %u: %s", a.number(), q.on ? "manual control (web page)" : "register control again");
    respond.Send(wst::ManualResponse::Encode, wst::ManualResponse::Payload{.ok = true, .message = ""});
}

void on_move(Responder& respond, auto& a, const wst::MoveRequest::Payload& q) noexcept {
    const char* e = !a.manual                          ? "manual control is off"
                    : a.stepper.moving() || a.homing ? "the axis is moving"
                    : q.steps == 0                     ? "0 steps"
                                                       : nullptr;
    if (e == nullptr) a.move(q.steps);
    respond.Send(wst::MoveResponse::Encode, wst::MoveResponse::Payload{.ok = e == nullptr, .message = e ? e : ""});
}

void on_stop(Responder& respond, auto& a, const wst::StopRequest::Payload&) noexcept {
    a.abort_homing("web page");
    respond.Send(wst::StopResponse::Encode, wst::StopResponse::Payload{.ok = true});
}

void on_home(Responder& respond, auto& a, const wst::HomeRequest::Payload& q) noexcept {
    const char* e = a.stepper.moving() || a.homing ? "the axis is moving" : q.speed == 0 ? "speed 0" : nullptr;
    if (e == nullptr) {
        a.home_params = {.speed = q.speed, .stall_threshold = q.sgThrs, .timeout_ms = q.timeoutMs};
        a.last_homing = Homed::none;                   // the result of this one, not of the last
        registers.set_holding(a.start_homing_reg, 1);  // tick_homing() starts it, as a write by Modbus would
    }
    respond.Send(wst::HomeResponse::Encode, wst::HomeResponse::Payload{.ok = e == nullptr, .message = e ? e : ""});
}

void on_status(Responder& respond, auto& a, const wst::StatusRequest::Payload&) noexcept {
    const auto drv = a.tmc.read(tm::drv_status);
    const auto sg = a.tmc.read(tm::sg_result);
    const auto ts = a.tmc.read(tm::tstep);
    const auto pwm = a.tmc.read(tm::pwm_scale);
    const auto gs = a.tmc.read(tm::gstat);
    wst::StatusResponse::Payload p{};
    p.tmcOk = drv && sg && ts && pwm && gs;
    p.position = a.stepper.position();
    p.moving = a.stepper.moving();
    p.manual = a.manual;
    p.homing = a.homing;
    p.homed = a.homed;
    p.homingResult = static_cast<std::uint8_t>(a.last_homing);
    p.stallByDiag = a.tmc.stall_by_signal();
    p.hasDiag = hw::has_stepper_diag;
    p.enabled = drivers_enabled();
    p.drvStatus = drv.value_or(0);
    p.sgResult = static_cast<std::uint16_t>(sg.value_or(0) & 0x3FFu);
    p.tstep = ts.value_or(0);
    p.pwmScale = pwm.value_or(0);
    p.gstat = static_cast<std::uint8_t>(gs.value_or(0));
    p.uptimeMs = kernel.now();
    respond.Send(wst::StatusResponse::Encode, p);
}

void handle(Responder& reply, const WsProtocol::FrameHeader& header, std::span<const std::uint8_t> frame) noexcept {
    switch (header.messageTypeId) {
        case wst::SettingsRequest::TYPE_ID:
            return dispatch(reply, frame, wst::SettingsRequest::Decode, [](auto& r, auto& a, const auto& q) { on_settings(r, a, q); });
        case wst::ManualRequest::TYPE_ID:
            return dispatch(reply, frame, wst::ManualRequest::Decode, [](auto& r, auto& a, const auto& q) { on_manual(r, a, q); });
        case wst::MoveRequest::TYPE_ID:
            return dispatch(reply, frame, wst::MoveRequest::Decode, [](auto& r, auto& a, const auto& q) { on_move(r, a, q); });
        case wst::StopRequest::TYPE_ID:
            return dispatch(reply, frame, wst::StopRequest::Decode, [](auto& r, auto& a, const auto& q) { on_stop(r, a, q); });
        case wst::HomeRequest::TYPE_ID:
            return dispatch(reply, frame, wst::HomeRequest::Decode, [](auto& r, auto& a, const auto& q) { on_home(r, a, q); });
        case wst::StatusRequest::TYPE_ID:
            return dispatch(reply, frame, wst::StatusRequest::Decode, [](auto& r, auto& a, const auto& q) { on_status(r, a, q); });
        default: return reply.Error(WsProtocol::ErrorCode::UnknownMessage, "unknown message");
    }
}

}  // namespace

void setup(bool supply_up) noexcept {
    if constexpr (hw::has_stepper_diag) {
        hw::Diag1::init();
        hw::Diag2::init();
        hw::Diag1::enable();
        hw::Diag2::enable();
        enable_irq(hw::Diag1::irq, irq_priority_net);  // kernel-aware: wakes the I/O thread
        enable_irq(hw::Diag2::irq, irq_priority_net);
    }
    uart.start();
    enable_irq(hw::StepperUart::irq, irq_priority_net);
    // Sensorless homing at the start, both at once: the I/O cycle runs it (tick_homing()). Not without
    // the motor supply: StallGuard could then report an end stop that is not there. An axis that is
    // not homed says so in its status (Bit5 clear); homing again: write 1 to StartHoming.
    for_each_axis([&](auto& a) {
        const bool ok = a.tmc.init(dev::Tmc2209Microsteps::M16);
        if (!ok) {
            RTOS_LOG_WARN("stepper %u: TMC2209 does not answer (steps still go out)", a.number());
        } else {
            (void)a.apply_settings(a.settings);
            if (supply_up) registers.set_holding(a.start_homing_reg, 1);
        }
        a.stepper.init();
        (void)a.configure_profile(a.profile);
    });
    if (!supply_up) {
        RTOS_LOG_WARN("steppers: no homing at the start (motor supply not up)");
    }
    for (const unsigned irq : {unsigned{hw::Stepper1Timer::irq}, unsigned{hw::Stepper2Timer::irq},
                               unsigned{hw::Stepper1Dma::irq}, unsigned{hw::Stepper2Dma::irq}}) {
        enable_irq(irq, irq_priority_motion);
    }
}

void cycle(std::uint32_t now) noexcept {
    requests.drain(answer, handle);
    tick_homings(now);
    const bool enabled = drivers_enabled();
    write(pins::stepper_enable, !enabled);  // active low
    for_each_axis([&](auto& a) { a.follow_target(enabled); });
}

std::uint32_t homing_poll_ms() noexcept { return axis1.homing || axis2.homing ? homing_poll_period_ms : 0; }

void homing_poll(std::uint32_t now) noexcept { tick_homings(now); }

void stall_from_isr(std::size_t axis) noexcept {
    with_axis(axis, [](auto& a) { a.diag_stall = true; });
}

void post(net::Server::SessionId session, WsProtocol::Responder& reply, std::span<const std::uint8_t> message) noexcept {
    requests.post(session, reply, message);
}

}  // namespace fcu::steppers

extern "C" void UART5_IRQHandler() noexcept { fcu::steppers::uart.on_irq(); }
extern "C" void FCU_STEPPER1_DMA_IRQ_HANDLER() noexcept { fcu::steppers::stepper1.on_dma_irq(); }
extern "C" void FCU_STEPPER2_DMA_IRQ_HANDLER() noexcept { fcu::steppers::stepper2.on_dma_irq(); }
extern "C" void TIM2_IRQHandler() noexcept { (void)fcu::steppers::stepper1.on_timer_irq(); }
extern "C" void TIM3_IRQHandler() noexcept { (void)fcu::steppers::stepper2.on_timer_irq(); }
