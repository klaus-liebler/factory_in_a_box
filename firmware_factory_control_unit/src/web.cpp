// WebSocket application protocol (see web.hpp).
#include "web.hpp"

#include <array>
#include <cstring>

#include "boot.hpp"
#include "io.hpp"
#include "plc.hpp"
#include "plc_project.hpp"
#include "roarm/roarm.hpp"
#include "task_monitor.hpp"
#include "terminal.hpp"
#include "ws_protocol.hh"

namespace fcu::web {
namespace {

namespace ws = WsProtocol;
using SessionId = net::Server::SessionId;
using ws::Responder;

// ---- answering: directly (network thread) or through the outbox (other threads) ----------------------

void* as_context(SessionId id) { return reinterpret_cast<void*>(static_cast<std::uintptr_t>(id)); }
SessionId session_of(void* context) { return static_cast<SessionId>(reinterpret_cast<std::uintptr_t>(context)); }

void log_too_small(SessionId id) {
    // Encode() found its buffer too small: the client waits in vain - say so
    RTOS_LOG_ERROR("ws: answer could not be encoded (buffer too small), client %u gets none", static_cast<unsigned>(id));
}

void send_direct(void* context, const std::uint8_t* data, std::size_t len) {
    if (len == 0) return log_too_small(session_of(context));
    (void)net::server.send(session_of(context), {reinterpret_cast<const std::byte*>(data), len});
}

void send_outbox(void* context, const std::uint8_t* data, std::size_t len) {
    if (len == 0) return log_too_small(session_of(context));
    (void)net::outbox.send(session_of(context), {reinterpret_cast<const std::byte*>(data), len});
}

// The network thread answers one request at a time: one buffer for every answer.
std::array<std::uint8_t, net::max_ws_message> answer;

// ---- modbus ---------------------------------------------------------------------------------

// The register answer carries every register: its schema bounds (schema/best_binary_buffers/modbus.cs)
// must hold the register map, and the message must fit a WebSocket message. Header, two array lengths
// of 2 each, 2 bytes per register.
static_assert(ws::modbus::RegistersMessage::RegistersMessage_MAX_SIZE >=
                  ws::HEADER_SIZE + 2 * 2 + (register_map::holding_count + register_map::input_count) * 2,
              "RegistersMessage too small for the register map: raise the bounds in modbus.cs");
static_assert(ws::modbus::RegistersMessage::RegistersMessage_MAX_SIZE <= net::max_ws_message,
              "RegistersMessage larger than a WebSocket message");

void get_registers(Responder& reply, const ws::modbus::GetRegistersRequest::Payload&) {
    constexpr std::size_t holding_count = register_map::holding_count, input_count = register_map::input_count;
    std::array<std::uint16_t, holding_count> holding{};
    std::array<std::uint16_t, input_count> input{};
    registers.read_holdings(0, holding);
    registers.read_inputs(0, input);
    std::array<std::uint8_t, holding_count * ws::modbus::RegistersMessageHolding_ELEMENT_SIZE> holding_bytes{};
    std::array<std::uint8_t, input_count * ws::modbus::RegistersMessageInput_ELEMENT_SIZE> input_bytes{};
    std::size_t pos = 0;
    for (const std::uint16_t v : holding) {
        pos += ws::modbus::EncodeRegistersMessageHoldingElement(v, holding_bytes.data() + pos, holding_bytes.size() - pos);
    }
    pos = 0;
    for (const std::uint16_t v : input) {
        pos += ws::modbus::EncodeRegistersMessageInputElement(v, input_bytes.data() + pos, input_bytes.size() - pos);
    }
    reply.Send(ws::modbus::RegistersMessage::Encode,
               ws::modbus::RegistersMessage::Payload{.holdingData = holding_bytes.data(), .holdingCount = holding_count,
                                                     .inputData = input_bytes.data(), .inputCount = input_count});
}

void write_holding(Responder& reply, const ws::modbus::WriteHoldingRequest::Payload& req) {
    // one write of 1..8 registers (a 32-bit value never arrives half); the decoder does not bound the
    // count, so it is checked here. The protocol path: the write filter refuses the program's outputs
    // in IEC mode (plc.hpp), the write hook applies commands (io::holding_written).
    constexpr std::size_t max_values = 8;
    std::array<std::uint16_t, max_values> values{};
    bool ok = req.valuesCount >= 1 && req.valuesCount <= max_values &&
              std::size_t{req.address} + req.valuesCount <= register_map::holding_count;
    for (std::size_t i = 0; ok && i < req.valuesCount; ++i) {
        ok = ws::modbus::DecodeWriteHoldingRequestValuesElementAt(req.valuesData, i, values[i]);
    }
    ok = ok && registers.write(req.address, {values.data(), req.valuesCount}) == rtos::modbus::Exception::None;
    reply.Send(ws::modbus::WriteHoldingResponse::Encode, ws::modbus::WriteHoldingResponse::Payload{.success = ok});
}

// ---- system ---------------------------------------------------------------------------------

void system_info(Responder& reply, const ws::system::SystemInfoRequest::Payload&) {
    ws::system::SystemInfoMessage::Payload resp{};
    resp.uptimeSeconds = kernel.now() / 1000;
    resp.freeHeapBytes = 0;  // no heap
    const net::Status s = net::status();
    const std::uint32_t ip = s.ethernet_address.value;
    for (int i = 0; i < 4; ++i) {
        resp.ipAddress[i] = static_cast<std::uint8_t>(ip >> (24 - 8 * i));
    }
    const std::uint32_t mask = s.ethernet_address.value != 0 ? s.ethernet_netmask.value : 0;
    for (int i = 0; i < 4; ++i) {
        resp.netMask[i] = static_cast<std::uint8_t>(mask >> (24 - 8 * i));
    }
    resp.resetCauseCode = static_cast<std::uint8_t>(boot::reset_cause());
    const net::PhyDiagnostics phy = net::phy_diagnostics();
    resp.phy = {.readOk = phy.read_ok, .tdrAvailable = phy.tdr_available, .bsr = phy.bsr, .physcsr = phy.physcsr,
                .mcsr = phy.mcsr, .secr = phy.secr, .scsir = phy.scsir, .tcsr = phy.tcsr, .clr = phy.clr};
    // every bus with its number on this revision (the page labels them I2Cn)
    std::array<std::uint8_t, io::i2c_bus_count * ws::system::SystemInfoMessageI2cScans_ELEMENT_SIZE> scans{};
    std::size_t scan_pos = 0;
    for (const auto bus : {io::I2cBus::Color, io::I2cBus::Tof2, io::I2cBus::Supply}) {
        ws::system::I2cBusScan s{};
        s.bus = static_cast<std::uint8_t>(io::i2c_bus_number(bus));
        std::memcpy(s.devices, io::i2c_scan(bus).data(), sizeof s.devices);
        scan_pos += ws::system::EncodeSystemInfoMessageI2cScansElement(s, scans.data() + scan_pos, scans.size() - scan_pos);
    }
    resp.i2cScansData = scans.data();
    resp.i2cScansCount = io::i2c_bus_count;
    reply.Send(ws::system::SystemInfoMessage::Encode, resp);
}

// ---- terminal: queued for the I/O thread, which answers ------------------------------------------------

void terminal_lookup(SessionId id, Responder& reply, const ws::terminal::LookupRequest::Payload& q) {
    switch (terminal::post_lookup(id, reply.Request().correlationId, q.lookup)) {
        case terminal::Posted::queued: break;
        case terminal::Posted::full: reply.Error(ws::ErrorCode::Busy, "terminal busy"); break;
        case terminal::Posted::invalid:
            reply.Send(ws::terminal::LookupResponse::Encode, ws::terminal::LookupResponse::Payload{.ok = false, .values = ""});
            break;
    }
}

void terminal_command(SessionId id, Responder& reply, const ws::terminal::CommandRequest::Payload& q) {
    if (terminal::post(id, reply.Request().correlationId, {q.line, std::strlen(q.line)}) != terminal::Posted::queued) {
        reply.Error(ws::ErrorCode::Busy, "terminal busy");
    }
}

void unknown(Responder& reply) { reply.Error(ws::ErrorCode::UnknownMessage, "unknown message"); }

// ---- log: every record to every browser, at once ------------------------------------------------------

LogMirror mirror;
rtos::log::Cursor log_cursor;
rtos::Tick next_log_poll = 0;
constexpr rtos::Tick log_poll_ms = 20;
constexpr std::size_t log_records_per_pass = 16;  // bounded foreground work (service.hpp)
std::uint32_t log_sequence = 0;

/// "io.cpp" -> "io"
std::string_view logger_name(const char* file) {
    std::string_view f(file);
    return f.substr(0, std::min<std::size_t>(f.find('.'), 16));
}

}  // namespace

void LogMirror::service() noexcept {
    const rtos::Tick now = kernel.now();
    if (static_cast<std::int32_t>(now - next_log_poll) < 0) {
        return;
    }
    next_log_poll = now + log_poll_ms;
    std::array<char, 120> text{};
    rtos::log::Record record;
    std::uint32_t lost = 0;
    for (std::size_t i = 0; i < log_records_per_pass && journal.read(log_cursor, record, text, lost); ++i) {
        ++log_sequence;
        if (net::server.websocket_clients() == 0) {
            continue;  // nobody watches: the record is gone (no history in the device)
        }
        std::array<char, 121> line{};
        std::memcpy(line.data(), record.text.data(), record.text.size());
        std::array<char, 17> logger{};
        const std::string_view name = logger_name(record.file);
        std::memcpy(logger.data(), name.data(), name.size());
        const std::uint64_t unix_now = net::unix_ms();
        const ws::system::LogMessage::Payload p{
            .level = static_cast<ws::system::LogLevel>(record.level),
            .sequence = log_sequence,
            .uptimeMs = record.time_ms,
            .unixMs = unix_now != 0 ? unix_now - (now - record.time_ms) : 0,
            .lost = static_cast<std::uint16_t>(std::min<std::uint32_t>(lost, 0xFFFF)),
            .logger = logger.data(),
            .text = line.data()};
        std::array<std::uint8_t, ws::system::LogMessage::LogMessage_MAX_SIZE> buf{};
        if (const std::size_t n = ws::system::LogMessage::Encode(p, buf.data(), buf.size()); n > 0) {
            (void)net::server.broadcast({reinterpret_cast<const std::byte*>(buf.data()), n});
        }
        next_log_poll = now;  // more may be waiting
    }
}

std::uint32_t LogMirror::next_deadline() const noexcept { return next_log_poll; }

LogMirror& log_mirror() noexcept { return mirror; }

void on_opened(void*, SessionId id) noexcept { RTOS_LOG_INFO("ws: client %u connected", static_cast<unsigned>(id)); }
void on_closed(void*, SessionId id) noexcept {
    plc::session_closed(id);  // its debug lease ends
    // a session the server reset because its TLS failed (e.g. MEMORY_E from the static pool) says so
    static std::uint32_t tls_failures = 0;
    if (const std::uint32_t f = net::server.tls_failures(); f != tls_failures) {
        tls_failures = f;
        RTOS_LOG_WARN("ws: client %u closed: TLS failed (wolfSSL error %d)", static_cast<unsigned>(id), net::server.last_tls_error());
        return;
    }
    RTOS_LOG_INFO("ws: client %u closed", static_cast<unsigned>(id));
}

void on_message(void*, SessionId id, std::span<const std::byte> message, bool binary) noexcept {
    const std::span<const std::uint8_t> frame{reinterpret_cast<const std::uint8_t*>(message.data()), message.size()};
    ws::FrameHeader header{};
    if (!binary || !ws::ReadHeader(frame.data(), frame.size(), header)) {
        return;
    }
    Responder reply(header, answer.data(), answer.size(), send_direct, as_context(id));
    const auto dispatch = [&](auto decode, auto handler) { ws::Dispatch(reply, frame.data(), frame.size(), decode, handler); };
    const std::uint16_t type = header.messageTypeId;
    switch (header.namespaceId) {
        case ws::modbus::NAMESPACE_ID:
            if (type == ws::modbus::GetRegistersRequest::TYPE_ID) return dispatch(ws::modbus::GetRegistersRequest::Decode, get_registers);
            if (type == ws::modbus::WriteHoldingRequest::TYPE_ID) return dispatch(ws::modbus::WriteHoldingRequest::Decode, write_holding);
            return unknown(reply);
        case ws::system::NAMESPACE_ID:
            if (type == ws::system::SystemInfoRequest::TYPE_ID) return dispatch(ws::system::SystemInfoRequest::Decode, system_info);
            return unknown(reply);
        case ws::terminal::NAMESPACE_ID: {
            const auto with_session = [id](auto handler) { return [id, handler](Responder& r, const auto& q) { handler(id, r, q); }; };
            if (type == ws::terminal::LookupRequest::TYPE_ID) return dispatch(ws::terminal::LookupRequest::Decode, with_session(terminal_lookup));
            if (type == ws::terminal::CommandRequest::TYPE_ID) return dispatch(ws::terminal::CommandRequest::Decode, with_session(terminal_command));
            return unknown(reply);
        }
        case ws::plc::NAMESPACE_ID:
            // the project (SD card): the I/O thread; everything else: the PLC module
            if (!plc::project::post(id, reply, frame)) plc::on_message(id, reply, frame);
            return;
        case ws::roarm::NAMESPACE_ID: return roarm::post(id, reply, frame);        // the I/O thread answers
        case ws::stepper::NAMESPACE_ID: return io::stepper_post(id, reply, frame);  // the I/O thread owns the TMC2209 UART
        case ws::tasks::NAMESPACE_ID: return task_monitor::handle_message(reply, frame);
        default: return unknown(reply);
    }
}

void broadcast_from_io(std::span<const std::uint8_t> message) noexcept {
    (void)net::outbox.broadcast({reinterpret_cast<const std::byte*>(message.data()), message.size()});
}

Responder responder_from_io(SessionId id, const ws::FrameHeader& request, std::span<std::uint8_t> buffer) noexcept {
    return Responder(request, buffer.data(), buffer.size(), send_outbox, as_context(id));
}

}  // namespace fcu::web
