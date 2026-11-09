// RoArm-M3 (see roarm.hpp).
#include "plc.hpp"
#include "roarm/roarm.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <numbers>
#include <type_traits>

#include "hardware.hpp"
#include "rtos/devices/feetech_sts.hpp"
#include "rtos/sync/message_queue.hpp"
#include "roarm/kinematics.hpp"
#include "roarm/missions.hpp"
#include "roarm/motion.hpp"
#include "system.hpp"
#include "web.hpp"
#include "ws_protocol.hh"

namespace fcu::roarm {
namespace {

namespace ws = WsProtocol::roarm;
namespace rm = register_map;
using std::numbers::pi;

struct KernelClock {
    static std::uint32_t now_ms() noexcept { return kernel.now(); }
    static void sleep_ms(std::uint32_t ms) noexcept { kernel.sleep(ms); }
};

hw::ServoUart servo_uart;
rtos::devices::FeetechBus<hw::ServoUart, KernelClock> servos{servo_uart};

// ---- servos ---------------------------------------------------------------------------------
constexpr std::array<std::uint8_t, 7> servo_ids{11, 12, 13, 14, 15, 16, 17};  // base, shoulder (2), elbow, wrist, roll, gripper
constexpr std::int16_t middle = 2047;
constexpr double counts_per_rad = 4096.0 / (2.0 * pi);

std::int16_t counts(double rad) { return static_cast<std::int16_t>(std::lround(rad * counts_per_rad)); }

/// Servo positions for the joint angles (the RoArm-M3 reference firmware's mapping).
std::array<std::int16_t, 7> servo_positions(const Joints& j) {
    return {static_cast<std::int16_t>(middle + counts(-std::clamp(j[Base], -pi, pi))),
            static_cast<std::int16_t>(middle + counts(std::clamp(j[Shoulder], -pi / 2, pi / 2))),
            static_cast<std::int16_t>(middle - counts(std::clamp(j[Shoulder], -pi / 2, pi / 2))),
            std::clamp<std::int16_t>(static_cast<std::int16_t>(counts(j[Elbow]) + 1024), 1024, 3071),
            static_cast<std::int16_t>(middle + counts(std::clamp(j[Wrist], -pi / 2, pi / 2))),
            static_cast<std::int16_t>(middle - counts(std::clamp(j[Roll], -pi, pi))),
            std::clamp<std::int16_t>(counts(j[Gripper]), 700, 3396)};
}

constexpr double max_velocity = 90.0 * pi / 180.0;
constexpr double max_acceleration = 180.0 * pi / 180.0;
ArmTracker tracker{Joints{}, max_velocity, max_acceleration};
std::array<ws::ServoStatusBits, 7> servo_status{};
bool teach_mode = false;
std::uint32_t last_tick = 0;

// ---- mission GPIOs: none wired yet (the former roarm_mission_gpio.hh) ---------------------------------
constexpr std::array<const char*, 4> mission_gpio_names{"Mission GPIO 1", "Mission GPIO 2", "Mission GPIO 3",
                                                        "Mission GPIO 4"};
bool set_mission_gpio(std::uint8_t id, bool) {
    return id < mission_gpio_names.size() && false;  // no pin assigned
}

// ---- missions ---------------------------------------------------------------------------------
enum class StepKind : std::uint8_t { JointMove, Gpio, Delay };
struct Step {
    StepKind kind = StepKind::Delay;
    Joints joints{};
    std::uint8_t gpio = 0;
    bool state = false;
    std::uint32_t delay_ms = 0;
};
constexpr std::size_t max_steps = 64;
std::array<Step, max_steps> steps{};
std::size_t step_count = 0, step_index = 0;
std::uint32_t step_start = 0;
bool running = false;
std::int16_t running_mission = 0;
std::array<std::uint8_t, ws::Mission::Mission_MAX_SIZE> mission_buffer;

void begin_step(std::uint32_t now) {
    const Step& s = steps[step_index];
    step_start = now;
    if (s.kind == StepKind::JointMove) {
        tracker.set_targets(s.joints);
    } else if (s.kind == StepKind::Gpio && !set_mission_gpio(s.gpio, s.state)) {
        RTOS_LOG_WARN("RoArm: mission GPIO %u has no pin, step skipped", s.gpio);
    }
}

void start_mission(std::int16_t index, std::uint32_t now) {
    const std::size_t size = missions::load(static_cast<std::uint16_t>(index), mission_buffer);
    if (size == 0) {
        RTOS_LOG_WARN("RoArm: mission %d not found", index);
        rm::set_ro_arm_mission_control(registers, -1);
        return;
    }
    ws::Mission::Payload m{};
    step_count = 0;
    const bool ok = ws::Mission::Decode(mission_buffer.data(), size, m) &&
                    ws::DecodeMissionStepsElements(m.stepsData, m.stepsDataSize, m.stepsCount, [](auto&& item) {
                        if (step_count >= max_steps) {
                            return;  // longer missions are cut (as before)
                        }
                        using T = std::decay_t<decltype(item)>;
                        Step& s = steps[step_count++];
                        if constexpr (std::is_same_v<T, ws::JointMoveStep::Payload>) {
                            s.kind = StepKind::JointMove;
                            for (int i = 0; i < joint_count; ++i) s.joints[i] = centideg_to_rad(item.jointAnglesCentiDeg[i]);
                        } else if constexpr (std::is_same_v<T, ws::GpioStep::Payload>) {
                            s.kind = StepKind::Gpio;
                            s.gpio = item.gpioId;
                            s.state = item.state;
                        } else if constexpr (std::is_same_v<T, ws::DelayStep::Payload>) {
                            s.kind = StepKind::Delay;
                            s.delay_ms = item.durationMs;
                        }
                    });
    if (!ok || step_count == 0) {
        RTOS_LOG_ERROR("RoArm: mission %d has no readable steps", index);
        rm::set_ro_arm_mission_control(registers, -3);
        return;
    }
    running = true;
    running_mission = index;
    step_index = 0;
    begin_step(now);
    rm::set_ro_arm_active_mission(registers, index);
    RTOS_LOG_INFO("RoArm: mission %d started, %u steps", index, static_cast<unsigned>(step_count));
}

void mission_control(std::uint32_t now) {
    const std::int16_t control = rm::ro_arm_mission_control(registers);
    if (running) {
        if (control != running_mission) {
            rm::set_ro_arm_mission_control(registers, running_mission);  // a running mission is not interrupted
        }
        return;
    }
    if (control <= 0) {
        return;
    }
    if (teach_mode) {
        RTOS_LOG_WARN("RoArm: mission %d refused, teach mode active", control);
        rm::set_ro_arm_mission_control(registers, -2);
        return;
    }
    start_mission(control, now);
}

void tick_mission(std::uint32_t now) {
    const Step& s = steps[step_index];
    const bool done = s.kind == StepKind::JointMove ? !tracker.moving()
                      : s.kind == StepKind::Gpio    ? true
                                                    : now - step_start >= s.delay_ms;
    if (!done) {
        return;
    }
    if (++step_index < step_count) {
        begin_step(now);
        return;
    }
    running = false;
    rm::set_ro_arm_mission_control(registers, 0);
    rm::set_ro_arm_active_mission(registers, 0);
    RTOS_LOG_INFO("RoArm: mission %d done", running_mission);
}

// ---- servo bus: positions out, feedback in -----------------------------------------------------------
void servo_io() {
    const auto positions = servo_positions(tracker.current());
    constexpr std::array<std::uint16_t, 7> speeds{};  // 0: the servos' default; the tracker shapes the motion
    (void)servos.sync_write_positions(servo_ids, positions, speeds, 20);
    bool any_error = false;
    for (std::size_t i = 0; i < servo_ids.size(); ++i) {
        const auto f = servos.feedback(servo_ids[i]);
        servo_status[i] = !f                               ? ws::ServoStatusBits::CommError
                          : f->temperature_c > 70          ? ws::ServoStatusBits::Overheat
                          : std::abs(f->load) > 900        ? ws::ServoStatusBits::Overload
                          : f->voltage_dv < 60             ? ws::ServoStatusBits::Undervoltage
                                                           : ws::ServoStatusBits::Ok;
        any_error = any_error || !f;
    }
    const Joints j = tracker.current();
    rm::set_ro_arm_joint1_angle_centi_deg(registers, rad_to_centideg(j[0]));
    rm::set_ro_arm_joint2_angle_centi_deg(registers, rad_to_centideg(j[1]));
    rm::set_ro_arm_joint3_angle_centi_deg(registers, rad_to_centideg(j[2]));
    rm::set_ro_arm_joint4_angle_centi_deg(registers, rad_to_centideg(j[3]));
    rm::set_ro_arm_joint5_angle_centi_deg(registers, rad_to_centideg(j[4]));
    rm::set_ro_arm_joint6_angle_centi_deg(registers, rad_to_centideg(j[5]));
    // bit 0 torque on (always), 1 moving, 2 a servo does not answer, 3 teach mode
    rm::set_status(registers, static_cast<std::uint16_t>(1u | (tracker.moving() ? 2u : 0u) | (any_error ? 4u : 0u) |
                                                         (teach_mode ? 8u : 0u)));
}

// ---- WebSocket requests (queued by the network thread) -------------------------------------------------
struct Request {
    net::Server::SessionId session = 0;
    std::uint16_t size = 0;
    std::array<std::uint8_t, net::max_ws_message> data{};
};
rtos::MessageQueue<Kernel, Request, 3> requests{kernel};

std::array<std::uint8_t, net::max_ws_message> reply;
std::array<std::uint8_t, net::max_ws_message> build;

void send(net::Server::SessionId id, std::size_t n) {
    if (n > 0) {
        (void)net::outbox.send(id, std::as_bytes(std::span(reply).first(n)));
    }
}

void handle(const Request& r) {
    const std::uint8_t* d = r.data.data();
    const std::size_t len = r.size;
    const auto type = static_cast<std::uint16_t>(d[2] | (d[3] << 8));
    switch (type) {
        case ws::StartTeachModeRequest::TYPE_ID: {
            ws::StartTeachModeRequest::Payload q{};
            if (!ws::StartTeachModeRequest::Decode(d, len, q)) return;
            const bool ok = !running && !plc::iec_controls();  // IEC mode: the program moves the arm
            teach_mode = teach_mode || ok;
            send(r.session, ws::StartTeachModeResponse::Encode({.requestId = q.requestId, .success = ok}, reply.data(), reply.size()));
            return;
        }
        case ws::StopTeachModeRequest::TYPE_ID: {
            ws::StopTeachModeRequest::Payload q{};
            if (!ws::StopTeachModeRequest::Decode(d, len, q)) return;
            teach_mode = false;
            send(r.session, ws::StopTeachModeResponse::Encode({.requestId = q.requestId, .success = true}, reply.data(), reply.size()));
            return;
        }
        case ws::JointJogTarget::TYPE_ID: {
            ws::JointJogTarget::Payload q{};
            if (!ws::JointJogTarget::Decode(d, len, q) || !teach_mode || running) return;
            Joints t{};
            for (int i = 0; i < joint_count; ++i) t[i] = clamp_joint(i, centideg_to_rad(q.jointAnglesCentiDeg[i]));
            tracker.set_targets(t);
            return;
        }
        case ws::CartesianJogTarget::TYPE_ID: {
            ws::CartesianJogTarget::Payload q{};
            if (!ws::CartesianJogTarget::Decode(d, len, q) || !teach_mode || running) return;
            auto ik = inverse({static_cast<double>(q.xMm), static_cast<double>(q.yMm), static_cast<double>(q.zMm),
                               centideg_to_rad(q.pitchCentiDeg), centideg_to_rad(q.rollCentiDeg),
                               centideg_to_rad(q.gripperCentiDeg)});
            if (!ik.reachable) return;
            for (int i = 0; i < joint_count; ++i) ik.joints[i] = clamp_joint(i, ik.joints[i]);
            tracker.set_targets(ik.joints);
            return;
        }
        case ws::ListMissionsRequest::TYPE_ID: {
            ws::ListMissionsRequest::Payload q{};
            if (!ws::ListMissionsRequest::Decode(d, len, q)) return;
            std::array<MissionSummary, 32> list{};
            const std::size_t count = missions::list(list);
            std::size_t pos = 0, n = 0;
            for (; n < count; ++n) {
                const ws::MissionSummary::Payload item{.missionIndex = list[n].index, .name = list[n].name.data()};
                const std::size_t next = ws::AppendListMissionsResponseMissionsMissionSummaryElement(item, build.data(), pos, build.size());
                if (next == 0) break;  // full: the rest of the list is cut
                pos = next;
            }
            send(r.session, ws::ListMissionsResponse::Encode({.requestId = q.requestId, .missionsData = build.data(),
                                                              .missionsCount = n, .missionsDataSize = pos},
                                                             reply.data(), reply.size()));
            return;
        }
        case ws::GetMissionRequest::TYPE_ID: {
            ws::GetMissionRequest::Payload q{};
            if (!ws::GetMissionRequest::Decode(d, len, q)) return;
            ws::GetMissionResponse::Payload p{.requestId = q.requestId, .found = false, .missionIndex = q.missionIndex,
                                              .name = "", .stepsData = nullptr, .stepsCount = 0, .stepsDataSize = 0};
            ws::Mission::Payload m{};
            const std::size_t size = running ? 0 : missions::load(q.missionIndex, mission_buffer);
            if (size != 0 && ws::Mission::Decode(mission_buffer.data(), size, m)) {
                p.found = true;
                p.name = m.name;
                p.stepsData = m.stepsData;
                p.stepsCount = m.stepsCount;
                p.stepsDataSize = m.stepsDataSize;
            }
            send(r.session, ws::GetMissionResponse::Encode(p, reply.data(), reply.size()));
            return;
        }
        case ws::SaveMissionRequest::TYPE_ID: {
            ws::SaveMissionRequest::Payload q{};
            if (!ws::SaveMissionRequest::Decode(d, len, q)) return;
            const bool ok = missions::save(q.missionIndex, q.name, {q.stepsData, q.stepsDataSize}, q.stepsCount);
            send(r.session, ws::SaveMissionResponse::Encode({.requestId = q.requestId, .success = ok,
                                                             .errorCode = static_cast<std::int16_t>(ok ? 0 : -1)},
                                                            reply.data(), reply.size()));
            return;
        }
        case ws::DeleteMissionRequest::TYPE_ID: {
            ws::DeleteMissionRequest::Payload q{};
            if (!ws::DeleteMissionRequest::Decode(d, len, q)) return;
            const bool ok = missions::remove(q.missionIndex);
            send(r.session, ws::DeleteMissionResponse::Encode({.requestId = q.requestId, .success = ok}, reply.data(), reply.size()));
            return;
        }
        case ws::GetMissionGpioListRequest::TYPE_ID: {
            ws::GetMissionGpioListRequest::Payload q{};
            if (!ws::GetMissionGpioListRequest::Decode(d, len, q)) return;
            std::size_t pos = 0;
            for (const char* name : mission_gpio_names) {
                const std::size_t l = std::strlen(name) + 1;
                std::memcpy(build.data() + pos, name, l);
                pos += l;
            }
            send(r.session, ws::GetMissionGpioListResponse::Encode({.requestId = q.requestId, .namesData = build.data(),
                                                                    .namesCount = mission_gpio_names.size(),
                                                                    .namesDataSize = pos},
                                                                   reply.data(), reply.size()));
            return;
        }
        default: return;
    }
}

std::uint32_t next_pose = 0;

void pose_broadcast(std::uint32_t now) {
    if (!teach_mode || static_cast<std::int32_t>(now - next_pose) < 0) {
        return;
    }
    next_pose = now + 100;
    const Joints j = tracker.current();
    const Pose p = forward(j);
    ws::PoseFeedback::Payload f{};
    for (int i = 0; i < joint_count; ++i) f.jointAnglesCentiDeg[i] = rad_to_centideg(j[i]);
    f.xMm = static_cast<std::int16_t>(std::lround(p.x_mm));
    f.yMm = static_cast<std::int16_t>(std::lround(p.y_mm));
    f.zMm = static_cast<std::int16_t>(std::lround(p.z_mm));
    f.pitchCentiDeg = rad_to_centideg(p.pitch);
    f.rollCentiDeg = rad_to_centideg(p.roll);
    std::copy(servo_status.begin(), servo_status.end(), f.servoStatus);
    std::array<std::uint8_t, 64> buf{};
    if (const std::size_t n = ws::PoseFeedback::Encode(f, buf.data(), buf.size()); n > 0) {
        web::broadcast_from_io(std::span(buf).first(n));
    }
}

}  // namespace

void setup() noexcept {
    servo_uart.start();
    enable_irq(hw::ServoUart::irq, irq_priority_net);
    (void)missions::mount();  // the SD card; without one the missions come when it does (missions.hpp)
}

void cycle(std::uint32_t now) noexcept {
    if (teach_mode && plc::iec_controls()) {
        teach_mode = false;  // switched to IEC: jogging ends, the arm holds its position
        RTOS_LOG_INFO("roarm: teach mode ended: the IEC program is the control source");
    }
    for (int i = 0; i < 3; ++i) {
        const auto r = requests.try_receive();
        if (!r) break;
        handle(*r);
    }
    double dt = last_tick == 0 ? 0.05 : (now - last_tick) / 1000.0;
    last_tick = now;
    if (!(dt > 0.0) || dt > 1.0) dt = 0.05;
    mission_control(now);
    if (running) {
        tick_mission(now);
    }
    tracker.tick(dt);
    servo_io();
    pose_broadcast(now);
}

void post(net::Server::SessionId id, std::span<const std::uint8_t> message) noexcept {
    if (message.size() < 4 || message.size() > net::max_ws_message) {
        return;
    }
    Request r{.session = id, .size = static_cast<std::uint16_t>(message.size())};
    std::memcpy(r.data.data(), message.data(), message.size());
    if (!requests.try_send(r)) {
        RTOS_LOG_WARN("RoArm: request queue full, message dropped");
    }
}

}  // namespace fcu::roarm

#if RTOS_FCU_REVISION == 3
extern "C" void USART1_IRQHandler() noexcept { fcu::roarm::servo_uart.on_irq(); }
#else
extern "C" void UART7_IRQHandler() noexcept { fcu::roarm::servo_uart.on_irq(); }
#endif
