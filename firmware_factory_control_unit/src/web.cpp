// WebSocket application protocol (see web.hpp).
#include "web.hpp"

#include <array>
#include <cstring>

#include "boot.hpp"
#include "io.hpp"
#include "plc.hpp"
#include "roarm/roarm.hpp"
#include "task_monitor.hpp"
#include "terminal.hpp"
#include "ws_protocol.hh"

namespace fcu::web {
namespace {

namespace ws = WsProtocol;
using SessionId = net::Server::SessionId;

void send(SessionId id, const std::uint8_t* data, std::size_t size) {
    if (size > 0) {
        (void)net::server.send(id, {reinterpret_cast<const std::byte*>(data), size});
    }
}

// ---- modbus ---------------------------------------------------------------------------------

void get_registers(SessionId id, const std::uint8_t* data, std::size_t len) {
    ws::modbus::GetRegistersRequest::Payload req{};
    if (!ws::modbus::GetRegistersRequest::Decode(data, len, req)) {
        return;
    }
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
    ws::modbus::RegistersMessage::Payload resp{};
    resp.requestId = req.requestId;
    resp.holdingData = holding_bytes.data();
    resp.holdingCount = holding_count;
    resp.inputData = input_bytes.data();
    resp.inputCount = input_count;
    std::array<std::uint8_t, ws::modbus::RegistersMessage::RegistersMessage_MAX_SIZE> buf{};
    send(id, buf.data(), ws::modbus::RegistersMessage::Encode(resp, buf.data(), buf.size()));
}

void write_holding(SessionId id, const std::uint8_t* data, std::size_t len) {
    ws::modbus::WriteHoldingRequest::Payload req{};
    if (!ws::modbus::WriteHoldingRequest::Decode(data, len, req)) {
        return;
    }
    ws::modbus::WriteHoldingResponse::Payload resp{};
    resp.requestId = req.requestId;
    // the protocol path: the write filter refuses the program's outputs in IEC mode (plc.hpp)
    resp.success = req.address < register_map::holding_count &&
                   registers.write(req.address, {&req.value, 1}) == rtos::modbus::Exception::None;
    std::array<std::uint8_t, ws::modbus::WriteHoldingResponse::WriteHoldingResponse_MAX_SIZE> buf{};
    send(id, buf.data(), ws::modbus::WriteHoldingResponse::Encode(resp, buf.data(), buf.size()));
}

// ---- system ---------------------------------------------------------------------------------

void system_info(SessionId id, const std::uint8_t* data, std::size_t len) {
    ws::system::SystemInfoRequest::Payload req{};
    if (!ws::system::SystemInfoRequest::Decode(data, len, req)) {
        return;
    }
    ws::system::SystemInfoMessage::Payload resp{};
    resp.requestId = req.requestId;
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
    std::array<std::uint8_t, ws::system::SystemInfoMessage::SystemInfoMessage_MAX_SIZE> buf{};
    send(id, buf.data(), ws::system::SystemInfoMessage::Encode(resp, buf.data(), buf.size()));
}

// ---- terminal ---------------------------------------------------------------------------------

void terminal_message(SessionId id, std::uint16_t type, const std::uint8_t* data, std::size_t len) {
    namespace t = ws::terminal;
    if (type == t::LookupRequest::TYPE_ID) {
        t::LookupRequest::Payload q{};
        if (!t::LookupRequest::Decode(data, len, q)) return;
        if (!terminal::post_lookup(id, q.requestId, q.lookup)) {  // else the I/O thread answers
            std::array<std::uint8_t, t::LookupResponse::LookupResponse_MAX_SIZE> buf{};
            send(id, buf.data(), t::LookupResponse::Encode({.requestId = q.requestId, .ok = false, .values = ""}, buf.data(), buf.size()));
        }
    } else if (type == t::CommandRequest::TYPE_ID) {
        t::CommandRequest::Payload q{};
        if (!t::CommandRequest::Decode(data, len, q)) return;
        terminal::post(id, q.requestId, {q.line, std::strlen(q.line)});  // the I/O thread answers
    }
}

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
void on_closed(void*, SessionId id) noexcept { RTOS_LOG_INFO("ws: client %u closed", static_cast<unsigned>(id)); }

void on_message(void*, SessionId id, std::span<const std::byte> message, bool binary) noexcept {
    if (!binary || message.size() < 4) {
        return;
    }
    const auto* data = reinterpret_cast<const std::uint8_t*>(message.data());
    const std::size_t len = message.size();
    const auto ns = static_cast<std::uint16_t>(data[0] | (data[1] << 8));
    const auto type = static_cast<std::uint16_t>(data[2] | (data[3] << 8));
    switch (ns) {
        case ws::modbus::NAMESPACE_ID:
            if (type == ws::modbus::GetRegistersRequest::TYPE_ID) {
                get_registers(id, data, len);
            } else if (type == ws::modbus::WriteHoldingRequest::TYPE_ID) {
                write_holding(id, data, len);
            }
            break;
        case ws::plc::NAMESPACE_ID:
            plc::on_message(id, type, data, len);
            break;
        case ws::system::NAMESPACE_ID:
            if (type == ws::system::SystemInfoRequest::TYPE_ID) {
                system_info(id, data, len);
            }
            break;
        case ws::terminal::NAMESPACE_ID:
            terminal_message(id, type, data, len);
            break;
        case ws::roarm::NAMESPACE_ID:
            roarm::post(id, {data, len});  // the I/O thread answers through the outbox
            break;
        case ws::tasks::NAMESPACE_ID: {
            std::array<std::uint8_t, net::max_ws_message> reply{};
            const std::size_t n = task_monitor::handle_message(type, {data, len}, reply);
            send(id, reply.data(), n);
            break;
        }
        default: break;
    }
}

void broadcast_from_io(std::span<const std::uint8_t> message) noexcept {
    (void)net::outbox.broadcast({reinterpret_cast<const std::byte*>(message.data()), message.size()});
}

}  // namespace fcu::web
