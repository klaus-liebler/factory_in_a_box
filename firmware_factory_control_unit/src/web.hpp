// WebSocket application protocol (wss://<hostname>.local/ws, ws_protocol.hh from
// schema/best_binary_buffers/*.cs): every message starts with the 8-byte header namespace id, type id,
// correlation id, flags (u16 LE). A response carries the correlation id of its request; a request that
// cannot be served gets a generic error reply (FLAG_ERROR, WsProtocol::ErrorCode) instead of silence,
// if its sender waits for one (correlation id != 0). Handlers never touch the header: they answer
// through a WsProtocol::Responder (Send / Error) and get their payload from WsProtocol::Dispatch, which
// answers Malformed itself.
//
//   modbus  GetRegisters / WriteHolding: the register map for the web UI's register panel
//   ethercat Status request: the EtherCAT slave's ESC registers, PHYs and counters (ethercat.hpp)
//   system  SystemInfo request; LogMessage events: every log record to every client at once
//           (the device keeps no history; the browser's Terminal page does)
//   terminal command list and commands of the browser terminal (terminal.hpp)
//   roarm   teach mode, jogging, missions (roarm.hpp)
//   tasks   thread and pool list (task_monitor.hpp)
//
// Handlers run in the network thread. Events from the I/O thread (pose feedback, pneumatics)
// go through net::outbox.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <span>

#include "network.hpp"
#include "rtos/sync/message_queue.hpp"
#include "rtos/net/service.hpp"
#include "ws_protocol.hh"

namespace fcu::web {

void on_opened(void*, net::Server::SessionId id) noexcept;
void on_message(void*, net::Server::SessionId id, std::span<const std::byte> data, bool binary) noexcept;
void on_closed(void*, net::Server::SessionId id) noexcept;

/// Network-thread service: sends new log records to every WebSocket client.
struct LogMirror {
    void service() noexcept;
    [[nodiscard]] std::uint32_t next_deadline() const noexcept;
};
LogMirror& log_mirror() noexcept;

/// Send an encoded message (from the I/O thread: through the outbox).
void broadcast_from_io(std::span<const std::uint8_t> message) noexcept;

/// The answer side of a request for a thread other than the network thread (I/O, PLC): sends through the
/// outbox. `request`: the header of the queued frame; `buffer` holds the answer, so it must outlive the
/// responder and fit the largest response of the handler. The network thread's handlers get theirs from
/// on_message() (it sends directly).
WsProtocol::Responder responder_from_io(net::Server::SessionId id, const WsProtocol::FrameHeader& request,
                                        std::span<std::uint8_t> buffer) noexcept;

/// WebSocket requests for a thread other than the network thread (the I/O thread owns the UARTs and
/// the SD card): the network thread queues the raw frame (post), the owner takes the frames in its
/// cycle (drain) and answers through the outbox. `Bytes`: the largest request of the namespace.
template <std::size_t Bytes, std::size_t Depth>
class FrameQueue {
public:
    /// `name` for the log ("RoArm", ...).
    explicit FrameQueue(const char* name) noexcept : name_(name) {}

    /// Network thread: queues the frame; a frame that is too large or finds the queue full is answered
    /// here (Malformed, Busy).
    void post(net::Server::SessionId id, WsProtocol::Responder& reply, std::span<const std::uint8_t> frame) noexcept {
        if (frame.size() > Bytes) {
            reply.Error(WsProtocol::ErrorCode::Malformed, "frame too large");
            return;
        }
        Frame f{.session = id, .size = static_cast<std::uint16_t>(frame.size())};
        std::ranges::copy(frame, f.data.begin());
        if (!queue_.try_send(f)) {
            RTOS_LOG_WARN("%s: request queue full, message dropped", name_);
            reply.Error(WsProtocol::ErrorCode::Busy, "request queue full");
        }
    }

    /// Owner thread: `handle(reply, header, frame)` for each queued frame, at most `max` (new ones wait
    /// for the next cycle); the responses are encoded into `answer`.
    template <class Handle>
    void drain(std::span<std::uint8_t> answer, Handle handle, std::size_t max = Depth) noexcept {
        for (std::size_t i = 0; i < max; ++i) {
            const auto f = queue_.try_receive();
            if (!f) return;
            const std::span<const std::uint8_t> frame{f->data.data(), f->size};
            WsProtocol::FrameHeader header{};
            if (!WsProtocol::ReadHeader(frame.data(), frame.size(), header)) continue;
            WsProtocol::Responder reply = responder_from_io(f->session, header, answer);
            handle(reply, header, frame);
        }
    }

private:
    struct Frame {
        net::Server::SessionId session = 0;
        std::uint16_t size = 0;
        std::array<std::uint8_t, Bytes> data{};
    };
    static_assert(Bytes <= 0xFFFF);
    rtos::MessageQueue<Kernel, Frame, Depth> queue_{kernel};
    const char* name_;
};

}  // namespace fcu::web
