// WebSocket application protocol (see web.hpp).
#include "web.hpp"

#include <array>
#include <cstring>

#include "boot.hpp"
#include "io.hpp"
#include "roarm/roarm.hpp"
#include "task_monitor.hpp"
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
    resp.success = req.address < register_map::holding_count;
    if (resp.success) {
        registers.set_holding(req.address, req.value);
    }
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
    std::memcpy(resp.i2c1Scan, io::i2c_scan(1).data(), 16);
    std::memcpy(resp.i2c2Scan, io::i2c_scan(2).data(), 16);
    std::memcpy(resp.i2c4Scan, io::i2c_scan(4).data(), 16);
    std::array<std::uint8_t, ws::system::SystemInfoMessage::SystemInfoMessage_MAX_SIZE> buf{};
    send(id, buf.data(), ws::system::SystemInfoMessage::Encode(resp, buf.data(), buf.size()));
}

}  // namespace

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
        case ws::system::NAMESPACE_ID:
            if (type == ws::system::SystemInfoRequest::TYPE_ID) {
                system_info(id, data, len);
            }
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
