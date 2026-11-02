// The network thread: one rtos::net::NetRuntime with every network service.
//
//   Ethernet  DHCP client (host name DEVICE_HOSTNAME), mDNS <hostname>.local, SNTP
//   USB       composite device: CDC-NCM (Ethernet to the PC, 192.168.173.1, DHCP server
//             leasing 192.168.173.2) and CDC-ACM (Modbus RTU, unit 1); connected while
//             USB_VSENSE sees VBUS
//   HTTPS     https://<hostname>.local/ web app (every path: the single-page app),
//             wss://<hostname>.local/ws binary protocol (ws_protocol.hh, see web.cpp)
//   Modbus    TCP port 502, RTU over CDC-ACM: the register map (register_map.hpp)
//   OPC UA    opc.tcp://<hostname>.local:4840, Objects/FactoryControlUnit (register_map_opcua.hpp)
//
// Services reach every interface; the certificates come from the builder (build/assets).
#pragma once

#include <cstdint>
#include <span>

#include "rtos/net/address.hpp"
#include "rtos/net/https_server.hpp"
#include "rtos/net/stack.hpp"
#include "rtos/net/websocket_outbox.hpp"
#include "system.hpp"

namespace fcu::net {

using Pool = rtos::net::PacketPool<36>;  // the 64 KB DMA region also holds USB, UCPD and LED buffers
struct StackConfig : rtos::net::DefaultStackConfig {
    static constexpr std::size_t udp_sockets = 8;      // DHCP client+server, mDNS, SNTP, DNS, spare
    static constexpr std::size_t tcp_connections = 12;  // browsers, Modbus TCP, OPC UA, TIME_WAIT
    static constexpr std::size_t tcp_listeners = 3;     // HTTPS, Modbus TCP, OPC UA
    static constexpr std::size_t tcp_bulk_connections = 2;
};
using Stack = rtos::net::Stack<Pool, StackConfig>;
inline constexpr std::size_t max_https_sessions = 6;
/// WebSocket messages up to 1280 bytes: the largest of ws_protocol.hh is 1160 (ListMissionsResponse).
inline constexpr std::size_t max_ws_message = 1280;
using Server = rtos::net::HttpsServer<Stack, max_https_sessions, max_ws_message>;
/// WebSocket messages from the I/O thread (broadcasts).
using Outbox = rtos::net::WebSocketOutbox<Server, 4096, Kernel::Lock>;

extern Server server;
extern Outbox outbox;

/// Starts the network thread (after the supply is up: the PHY and USB are power consumers).
void start() noexcept;

struct Status {
    bool ethernet_link = false;
    std::uint16_t ethernet_mbps = 0;
    bool ethernet_full_duplex = false;
    rtos::net::Ipv4Address ethernet_address{}, ethernet_netmask{};
    bool usb_configured = false;
    std::uint32_t rx_frames = 0, tx_frames = 0;
};
/// Snapshot for the I/O thread (registers, health bits).
[[nodiscard]] Status status() noexcept;

/// LAN8742 registers for the system page; the TDR cable test runs on every call (about 2 ms).
/// Network thread only (WebSocket handlers).
struct PhyDiagnostics {
    bool read_ok = false, tdr_available = false;
    std::uint16_t bsr = 0, physcsr = 0, mcsr = 0, secr = 0, scsir = 0, tcsr = 0, clr = 0;
};
[[nodiscard]] PhyDiagnostics phy_diagnostics() noexcept;

/// Packet buffers: total and free now (task manager).
struct PoolUse {
    std::uint32_t capacity = 0, available = 0;
};
[[nodiscard]] PoolUse packet_pool_use() noexcept;

/// Unix time in ms from SNTP, 0 while not synchronised.
[[nodiscard]] std::uint64_t unix_ms() noexcept;

}  // namespace fcu::net
