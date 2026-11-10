// The Malibotti robot (see malibotti.hpp, docs/malibotti-uart.md).
#include "malibotti.hpp"

#if RTOS_FCU_REVISION == 3

#include <array>
#include <cstddef>
#include <span>

#include "hardware.hpp"
#include "rtos/serial/frame.hpp"
#include "system.hpp"
#include "ws_protocol.hh"

namespace fcu::malibotti {
namespace {

namespace ws = WsProtocol::malibotti;
namespace rm = register_map;

constexpr std::uint32_t lost_after_ms = 1000;   ///< no Status for this long: not connected
constexpr std::uint32_t resend_ms = 200;        ///< a command without acknowledgement goes out again
constexpr std::uint32_t give_up_ms = 1000;      ///< ... until this long: then "no answer"

/// MalibottiMissionControl values of the FCU itself (the robot's results are -MissionResult).
constexpr std::int16_t no_answer = -static_cast<std::int16_t>(ws::MissionResult::NoAnswer);
constexpr std::int16_t aborted = -static_cast<std::int16_t>(ws::MissionResult::Aborted);

hw::MalibottiUart uart;
rtos::serial::FrameDecoder<rtos::serial::max_frame_size(ws::Status::Status_MAX_SIZE) + 16> decoder;

ws::Status::Payload status{};
bool have_status = false;
std::uint32_t last_status_ms = 0;

/// The command being delivered (until the robot acknowledges its seq).
struct Pending {
    bool active = false;
    ws::Command::Payload command{};
    std::uint32_t first_ms = 0, last_ms = 0;
};
Pending pending;
std::uint8_t next_seq = 1;

/// The mission the FCU started and waits for (0: none). acked: the robot took the start.
std::int16_t running = 0;
bool running_acked = false;

void send(const ws::Command::Payload& c) noexcept {
    std::array<std::uint8_t, ws::Command::Command_MAX_SIZE> msg{};
    const std::size_t n = ws::Command::Encode(c, msg.data(), msg.size());
    std::array<std::uint8_t, rtos::serial::max_frame_size(ws::Command::Command_MAX_SIZE)> frame{};
    const std::size_t f = rtos::serial::encode_frame(std::span(msg.data(), n), frame);
    (void)uart.write(std::as_bytes(std::span(frame.data(), f)));
}

void command(ws::CommandKind kind, std::uint16_t mission, std::uint32_t now) noexcept {
    pending = {.active = true, .command = {.seq = next_seq, .kind = kind, .mission = mission}, .first_ms = now, .last_ms = now};
    next_seq = next_seq == 255 ? 1 : static_cast<std::uint8_t>(next_seq + 1);  // 0 is "none"
    send(pending.command);
}

void receive(std::uint32_t now) noexcept {
    std::array<std::byte, 64> chunk;
    for (std::size_t n; (n = uart.read(chunk)) > 0;) {
        for (std::size_t i = 0; i < n; ++i) {
            const auto frame = decoder.push(static_cast<std::uint8_t>(chunk[i]));
            if (!frame || frame->size() < 4) {
                continue;
            }
            const auto* d = frame->data();
            const auto ns = static_cast<std::uint16_t>(d[0] | (d[1] << 8));
            const auto type = static_cast<std::uint16_t>(d[2] | (d[3] << 8));
            ws::Status::Payload s{};
            if (ns != ws::NAMESPACE_ID || type != ws::Status::TYPE_ID || !ws::Status::Decode(d, frame->size(), s)) {
                continue;
            }
            status = s;
            have_status = true;
            last_status_ms = now;
            if (pending.active && s.ackSeq == pending.command.seq) {
                pending.active = false;
                if (pending.command.kind == ws::CommandKind::StartMission) {
                    running_acked = true;
                }
            }
        }
    }
}

}  // namespace

void setup() noexcept {
    uart.start();
    enable_irq(hw::MalibottiUart::irq, irq_priority_net);
}

void cycle(std::uint32_t now) noexcept {
    receive(now);
    const bool connected = have_status && now - last_status_ms < lost_after_ms;

    // the command being delivered: again, or given up
    if (pending.active) {
        if (now - pending.first_ms >= give_up_ms) {
            pending.active = false;
            if (running != 0) {
                running = 0;
                rm::set_malibotti_mission_control(registers, no_answer);
            }
        } else if (now - pending.last_ms >= resend_ms) {
            pending.last_ms = now;
            send(pending.command);
        }
    }

    // the control register (as RoArmMissionControl): > 0 starts, a running mission is not replaced;
    // -3 (aborted) written while it runs stops it
    const std::int16_t control = rm::malibotti_mission_control(registers);
    if (running != 0) {
        if (!connected && running_acked) {
            running = 0;
            rm::set_malibotti_mission_control(registers, no_answer);
        } else if (running_acked && status.state != ws::RobotState::Busy && status.lastMission == static_cast<std::uint16_t>(running)) {
            // done: 0, or the robot's result as a negative code
            running = 0;
            rm::set_malibotti_mission_control(registers, -static_cast<std::int16_t>(status.lastResult));
        } else if (control == aborted) {
            if (!pending.active || pending.command.kind != ws::CommandKind::Abort) {
                command(ws::CommandKind::Abort, 0, now);
            }
        } else if (control != running) {
            rm::set_malibotti_mission_control(registers, running);
        }
    } else if (control > 0) {
        if (!connected) {
            rm::set_malibotti_mission_control(registers, no_answer);
        } else {
            running = control;
            running_acked = false;
            command(ws::CommandKind::StartMission, static_cast<std::uint16_t>(control), now);
        }
    }

    std::uint16_t bits = 0;
    if (connected) {
        bits |= 1u << rm::input::malibotti_status_bit::connected;
        if (status.state == ws::RobotState::Busy) bits |= 1u << rm::input::malibotti_status_bit::busy;
        if (status.state == ws::RobotState::Error) bits |= 1u << rm::input::malibotti_status_bit::error;
    }
    rm::set_malibotti_status(registers, bits);
    rm::set_malibotti_active_mission(registers, connected ? static_cast<std::int16_t>(status.activeMission) : std::int16_t{0});
}

}  // namespace fcu::malibotti

extern "C" void USART6_IRQHandler() noexcept { fcu::malibotti::uart.on_irq(); }

#else  // revision 1: no Malibotti connector

namespace fcu::malibotti {
void setup() noexcept {}
void cycle(std::uint32_t) noexcept {}
}  // namespace fcu::malibotti

#endif
