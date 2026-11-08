// Network thread of the FCU firmware (see network.hpp).
#include "network.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <optional>
#include <string_view>

#include "board/chip.hpp"
#include "board/fcu.hpp"
#include "device_ids.hh"
#include "pins.hpp"
#include "register_map_opcua.hpp"
#include "rtos/modbus/rtu_service.hpp"
#include "rtos/net/dhcp.hpp"
#include "rtos/net/dhcp_server.hpp"
#include "rtos/net/dns_resolver.hpp"
#include "rtos/net/mdns.hpp"
#include "rtos/net/modbus_tcp_server.hpp"
#include "rtos/net/opcua_server.hpp"
#include "rtos/net/service.hpp"
#include "rtos/net/sntp.hpp"
#include "rtos/net/tls.hpp"
#include "rtos/net/tls_memory.hpp"
#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/ns0_base.hpp"
#include "rtos/opcua/registers.hpp"
#if FCU_USB_DEVICE
#include "rtos/port/stm32h5/usb_device.hpp"
#include "rtos/usb/cdc_acm.hpp"
#include "rtos/usb/cdc_ncm.hpp"
#include "rtos/usb/descriptors.hpp"
#include "rtos/usb/device.hpp"
#endif
#include "task_monitor.hpp"
#include "web.hpp"

namespace fcu::net {
namespace {

namespace rn = rtos::net;
namespace ua = rtos::opcua;

// ---- assets from the builder (build/assets, #embed) -------------------------------------------
constexpr unsigned char index_html_br[] = {
#embed "index.html.br"
};
constexpr unsigned char https_certificate[] = {
#embed "device_https_certificate.der"
};
constexpr unsigned char https_private_key[] = {
#embed "device_https_key.der"
};
constexpr unsigned char opcua_certificate[] = {
#embed "device_certificate.der"
};
constexpr unsigned char opcua_private_key[] = {
#embed "device_key.der"
};
constexpr unsigned char ca_certificate[] = {
#embed "root_ca.der"
};

template <std::size_t N>
std::span<const std::byte> as_span(const unsigned char (&a)[N]) {
    return {reinterpret_cast<const std::byte*>(a), N};
}

constexpr rn::MacAddress mac(const std::uint8_t (&m)[6]) {
    return rn::MacAddress::from(m[0], m[1], m[2], m[3], m[4], m[5]);
}

// ---- stack and drivers ------------------------------------------------------------------------
constexpr std::size_t eth_rx_descriptors = 8;
using Ethernet = board::Ethernet<Pool, eth_rx_descriptors>;
static_assert(Pool::capacity() >= eth_rx_descriptors + StackConfig::rx_reserve + StackConfig::tcp_connections,
              "packet pool too small for RX ring + receive reserve + one segment per connection");

RTOS_DMA_BUFFER Pool packet_pool;
RTOS_DMA_BUFFER Ethernet ethernet{packet_pool, mac(DEVICE_ETH_MAC), board::phy_address};
Stack stack{packet_pool};
constexpr rn::InterfaceId eth_if = 0;  // first attach(): unconfigured until DHCP

#if FCU_USB_DEVICE
// ---- USB: CDC-NCM (interfaces 0, 1) + CDC-ACM for Modbus RTU (interfaces 2, 3) -----------------
namespace usb = rtos::usb;
// The PC's NIC gets DEVICE_USB_NCM_MAC (iMACAddress); this end uses the same address with the
// lowest bit of the last octet flipped, so the two ends of the link differ.
constexpr rn::MacAddress usb_device_mac = rn::MacAddress::from(
    DEVICE_USB_NCM_MAC[0], DEVICE_USB_NCM_MAC[1], DEVICE_USB_NCM_MAC[2], DEVICE_USB_NCM_MAC[3],
    DEVICE_USB_NCM_MAC[4], static_cast<std::uint8_t>(DEVICE_USB_NCM_MAC[5] ^ 0x01));
constexpr usb::CdcNcmInfo ncm_info{.interface = 0, .notify_ep = 0x81, .out_ep = 0x02, .in_ep = 0x82,
                                   .string = 4, .mac_string = 5};
constexpr usb::CdcAcmInfo acm_info{.interface = 2, .notify_ep = 0x83, .out_ep = 0x04, .in_ep = 0x84, .string = 6};
constexpr auto usb_device_descriptor = usb::device_descriptor({});
constexpr auto usb_configuration = usb::configuration_descriptor(
    {.max_power_ma = 100, .self_powered = true}, usb::cdc_ncm_function(ncm_info), usb::cdc_acm_function(acm_info));
constexpr std::array<std::string_view, 6> usb_strings{
    "Klaus Liebler", "FactoryControl Unit", DEVICE_USB_SERIAL_STRING, "FactoryControl Network",
    DEVICE_USB_NCM_MAC_STRING, "FactoryControl Modbus RTU"};
constexpr usb::DeviceDescriptors usb_descriptors{usb_device_descriptor, usb_configuration, usb_strings};

using UsbController = rtos::port::stm32h5::UsbDrdFs<>;
UsbController usb_fs;
RTOS_DMA_BUFFER usb::CdcNcm<Pool> ncm{ncm_info, packet_pool, usb_device_mac};
usb::CdcAcm<> acm{acm_info};
std::array<usb::Function*, 2> usb_functions{&ncm, &acm};
usb::Device<UsbController> usb_device{usb_fs, usb_descriptors, usb_functions};

constexpr rn::Ipv4Config usb_address{.address = rn::Ipv4Address::from(192, 168, 173, 1),
                                     .netmask = rn::Ipv4Address::from(255, 255, 255, 0)};
rn::DhcpServer<Stack> usb_dhcp{stack, 1, {.first = rn::Ipv4Address::from(192, 168, 173, 2), .count = 1}};
rtos::modbus::RtuService<usb::CdcAcm<>, Stack> modbus_rtu{acm, stack, registers, 1, 5};
#endif

// ---- name services, time ------------------------------------------------------------------------
rn::MdnsResponder<Stack> mdns{stack, DEVICE_HOSTNAME};
rn::SntpClient<Stack> sntp{stack, rn::Ipv4Address::any(), 123, 3'600'000};
rn::DnsResolver<Stack> dns{stack};
constexpr std::string_view sntp_host = "pool.ntp.org";

// ---- TLS + HTTPS ------------------------------------------------------------------------------
// wolfSSL static pool of TLS and OPC UA: the rtoos fcu_net sizing (measured with 6 sessions,
// 2 concurrent handshakes and the OPC UA RSA opens), see rtoos apps/net/main.cpp.
constexpr std::array<std::uint32_t, 7> tls_bucket_sizes{64, 128, 256, 512, 1024, 2176, 5632};
constexpr std::array<std::uint32_t, 7> tls_bucket_counts{64, 24, 36, 8, 6, 10, 3};
alignas(16) std::array<std::byte, 72 * 1024> tls_memory;
rn::TlsServerContext tls_context;

// Every path is the single-page app (it routes by window.location.pathname).
const std::array<rn::HttpResource, 1> resources{{
    {"*", "text/html; charset=utf-8", "br", as_span(index_html_br)},
}};

const Server::WebSocketHandler ws_handler{nullptr, &web::on_opened, &web::on_message, &web::on_closed};

// ---- Modbus TCP, OPC UA ------------------------------------------------------------------------
rn::ModbusTcpServer<Stack, 2> modbus_tcp{stack, registers};

OPCUA_RODATA constexpr auto ua_nodes = ua::JoinNodes(ua::kBaseNodes, register_map::opcua::nodes);
constexpr auto ua_references = ua::JoinReferences(ua::kBaseReferences, register_map::opcua::references);
static_assert(ua::AllTypedNodesHaveDataType(ua_nodes));
static_assert(ua::AllDataTypesResolve(ua_nodes));
static_assert(ua::AllReferencesResolve(ua_nodes, ua_references));
constexpr ua::AddressSpace ua_space(ua_nodes, ua_references);

constexpr auto opcua_endpoint = [] {
    std::array<char, 64> url{};
    constexpr std::string_view prefix = "opc.tcp://", suffix = ".local:4840";
    std::size_t n = 0;
    for (const std::string_view part : {prefix, std::string_view(DEVICE_HOSTNAME), suffix}) {
        for (const char c : part) {
            url[n++] = c;
        }
    }
    return url;
}();
rn::OpcUaServer<Stack, 2> opcua{stack, ua_space, std::string_view(opcua_endpoint.data())};

// ---- runtime, thread ---------------------------------------------------------------------------
rn::NetRuntime<Stack> runtime{stack};

Kernel::Thread net_tcb;
// Deepest path: an OPC UA OpenSecureChannel step with the RSA-2048 key, ~13.5 KB (rtoos fcu_net).
alignas(8) std::array<std::byte, 20 * 1024> net_stack;
constexpr std::size_t slot_wake = 0;

std::atomic<bool> link_up{false};
std::atomic<std::uint32_t> eth_address{0}, eth_netmask{0};
std::atomic<bool> usb_configured{false};

void on_sntp_host(void*, std::string_view name, std::optional<rn::Ipv4Address> address) noexcept {
    if (!address) {
        RTOS_LOG_WARN("sntp: %.*s not resolved", static_cast<int>(name.size()), name.data());
        return;
    }
    sntp.set_server(*address, 123);
}

void on_dhcp_bound(void*, const rn::DhcpLease& lease, bool renewed) noexcept {
    const unsigned a = lease.config.address.value;
    eth_address = a;
    eth_netmask = lease.config.netmask.value;
    RTOS_LOG_INFO("dhcp: %s %u.%u.%u.%u, lease %u s", renewed ? "renewed" : "bound", a >> 24, (a >> 16) & 0xFF,
                  (a >> 8) & 0xFF, a & 0xFF, static_cast<unsigned>(lease.lease_s));
    if (!renewed) {
        dns.set_servers(lease.dns);
        if (lease.ntp[0] != rn::Ipv4Address::any()) {
            sntp.set_server(lease.ntp[0], 123);
        } else if (!dns.resolve(sntp_host, &on_sntp_host, nullptr)) {
            RTOS_LOG_WARN("sntp: no DNS server for %.*s", static_cast<int>(sntp_host.size()), sntp_host.data());
        }
    }
}
void on_dhcp_lost(void*) noexcept {
    eth_address = 0;
    RTOS_LOG_WARN("dhcp: lease lost");
}
void on_dhcp_declined(void*, rn::Ipv4Address) noexcept { RTOS_LOG_WARN("dhcp: offered address in use, declined"); }
void on_mdns_renamed(void*, std::string_view name) noexcept {
    RTOS_LOG_WARN("mdns: name conflict, now %.*s", static_cast<int>(name.size()), name.data());
}

const rn::DhcpClient<Stack>::Handler dhcp_handler{nullptr, &on_dhcp_bound, &on_dhcp_lost, &on_dhcp_declined};
rn::DhcpClient<Stack> dhcp{stack, eth_if, DEVICE_HOSTNAME, dhcp_handler};

/// Link and VBUS polling (USB device: connected while VBUS is there), SNTP log line.
struct LinkTasks {
    static constexpr rtos::Tick poll_ms = 250;
    rtos::Tick next_poll = 0;
    bool link = false, vbus = false, synced = false;

    void service() noexcept {
        const rtos::Tick now = stack.now_ms();
        if (static_cast<std::int32_t>(now - next_poll) < 0) {
            return;
        }
        next_poll = now + poll_ms;
        if (ethernet.poll_link() != link) {  // the stack sees it at its next poll
            link = !link;
            link_up = link;
            RTOS_LOG_INFO("net: ethernet link %s", link ? "up" : "down");
        }
        if (const bool v = pins::gpio::read(pins::usb_vsense); v != vbus) {
            vbus = v;
#if FCU_USB_DEVICE
            if (v) {
                usb_device.connect();
            } else {
                usb_device.disconnect();
            }
            RTOS_LOG_INFO("usb: VBUS %s", v ? "present, connected" : "gone, disconnected");
#else
            RTOS_LOG_INFO("usb: VBUS %s (no USB device on this revision yet)", v ? "present" : "gone");
#endif
        }
#if FCU_USB_DEVICE
        usb_configured = usb_device.state() == usb::DeviceState::Configured;
#endif
        if (sntp.synchronized() && !synced) {
            synced = true;
            RTOS_LOG_INFO("net: sntp synchronized");
        }
    }
    [[nodiscard]] std::uint32_t next_deadline() const noexcept { return next_poll; }
} link_tasks;

bool rng_fill(std::span<std::byte> out) noexcept {
    for (std::size_t i = 0; i < out.size(); i += 4) {
        const std::uint32_t r = board::Rng::next();
        for (std::size_t j = 0; j < 4 && i + j < out.size(); ++j) {
            out[i + j] = static_cast<std::byte>(r >> (8 * j));
        }
    }
    return true;
}

void net_main(void*) {
    board::init_ethernet();
    if (!ethernet.init(board::hclk_hz)) {
        RTOS_LOG_ERROR("net: ETH DMA reset timed out (no RMII reference clock?)");
    }
    board::Rng::init();
    [[maybe_unused]] const rn::InterfaceId e = stack.attach(ethernet);
    contract_assert(e == eth_if);
#if FCU_USB_DEVICE
    [[maybe_unused]] const rn::InterfaceId u = stack.attach(ncm, usb_address);
    contract_assert(u == 1);
#endif
    stack.set_random([](void*) noexcept { return board::Rng::next(); }, nullptr);

#if FCU_USB_DEVICE
    rtos::port::stm32h5::UsbHardware::enable_clocks();
    usb_fs.start();
    enable_irq(UsbController::irq, irq_priority_net);
#endif
    enable_irq(Ethernet::irq_number, irq_priority_net);

    if (!rn::TlsServerContext::initialize_library(
            &rng_fill, []() noexcept -> std::uint32_t { return kernel.now() / 1000; },
            []() noexcept { return board::CycleCounter::now(); })) {
        RTOS_LOG_ERROR("tls: library init failed");
        return;
    }
    rn::tls_pool_tracking_start();
    if (const auto r = tls_context.init(tls_memory, {as_span(https_certificate), as_span(https_private_key)},
                                        static_cast<int>(max_https_sessions), {tls_bucket_sizes, tls_bucket_counts});
        r != rn::TlsInitResult::Ok) {
        RTOS_LOG_ERROR("tls: context init failed, step %u", static_cast<unsigned>(r));
        return;
    }
    ua::SetClock([]() noexcept { return sntp.unix_ms(kernel.now()); }, []() noexcept { return kernel.now64(); });
    if (const auto r = opcua.init({as_span(opcua_certificate), as_span(opcua_private_key), as_span(ca_certificate)}, {});
        r != rn::OpcUaInitResult::Ok) {
        RTOS_LOG_ERROR("opcua: init failed, step %u", static_cast<unsigned>(r));
        return;
    }
    ua::registers::Bind(registers);

    mdns.start(rn::InterfaceSet::all(), &on_mdns_renamed);
    dhcp.start();
#if FCU_USB_DEVICE
    usb_dhcp.start();
#endif
    sntp.start();
    server.start(443);
    if (!modbus_tcp.start()) {
        RTOS_LOG_ERROR("modbus: listen failed");
    }
    if (!opcua.start()) {
        RTOS_LOG_ERROR("opcua: listen failed");
    }
    outbox.set_wake([](void*) noexcept { kernel.notify(net_tcb, slot_wake, 1, rtos::NotifyAction::SetBits); },
                    nullptr);

#if FCU_USB_DEVICE
    runtime.add(usb_device, "usb");  // first: drains the controller's events
#endif
    runtime.add(link_tasks, "link");
    runtime.add(dhcp, "dhcp");
#if FCU_USB_DEVICE
    runtime.add(usb_dhcp, "usb-dhcp");
#endif
    runtime.add(dns, "dns");
    runtime.add(mdns, "mdns");
    runtime.add(sntp, "sntp");
    runtime.add(server, "https");
    runtime.add(outbox, "ws-outbox");
    runtime.add(web::log_mirror(), "log-mirror");
    runtime.add(modbus_tcp, "modbus");
#if FCU_USB_DEVICE
    runtime.add(modbus_rtu, "modbus-rtu");
#endif
    runtime.add(opcua, "opcua");
    runtime.set_cycle_counter([]() noexcept { return board::CycleCounter::now(); });
    RTOS_LOG_INFO("net: https://%s.local/, opc.tcp://%s.local:4840, modbus tcp 502", DEVICE_HOSTNAME, DEVICE_HOSTNAME);
    for (;;) {
        const std::uint32_t wake = runtime.run_once(kernel.now());
        (void)kernel.notify_wait(slot_wake, runtime.sleep_ms(wake, kernel.now()));
    }
}

}  // namespace

constexpr rn::HttpsServerConfig https_config{};
Server server{stack, tls_context, resources, "/ws", ws_handler, https_config};
Outbox outbox{server};

void start() noexcept {
    std::ranges::fill(net_stack, task_monitor::paint);
    task_monitor::add(net_tcb, net_stack);
    kernel.create_thread(net_tcb, {.name = "net", .entry = &net_main, .stack = net_stack, .priority = net_priority});
}

Status status() noexcept {
    return {.ethernet_link = link_up,
            .ethernet_mbps = ethernet.speed_mbps(),
            .ethernet_full_duplex = ethernet.full_duplex(),
            .ethernet_address = rn::Ipv4Address{eth_address},
            .ethernet_netmask = rn::Ipv4Address{eth_netmask},
            .usb_configured = usb_configured,
            .rx_frames = stack.stats().rx_frames,
            .tx_frames = stack.stats().tx_frames};
}

PoolUse packet_pool_use() noexcept {
    return {static_cast<std::uint32_t>(Pool::capacity()), static_cast<std::uint32_t>(packet_pool.available())};
}

std::uint64_t unix_ms() noexcept { return sntp.unix_ms(kernel.now()); }

PhyDiagnostics phy_diagnostics() noexcept {
    // LAN8742A registers (datasheet section 4.2; ST's lan8742.h)
    constexpr unsigned bsr = 0x01, mcsr = 0x11, tcsr = 0x19, secr = 0x1A, scsir = 0x1B, clr = 0x1C, physcsr = 0x1F;
    constexpr std::uint16_t tcsr_tdr_enable = 0x8000;
    PhyDiagnostics d;
    d.bsr = ethernet.phy_read(bsr);
    d.physcsr = ethernet.phy_read(physcsr);
    d.mcsr = ethernet.phy_read(mcsr);
    d.secr = ethernet.phy_read(secr);  // clears on read: errors since the last call
    d.scsir = ethernet.phy_read(scsir);
    d.read_ok = d.bsr != 0xFFFF;       // no PHY answers: the bus floats high
    if (!d.read_ok) {
        return d;
    }
    ethernet.phy_write(tcsr, tcsr_tdr_enable);
    const std::uint32_t start = board::CycleCounter::now();
    while (board::CycleCounter::now() - start < board::sysclk_hz / 500) {  // 2 ms for the reflection measurement
    }
    d.tcsr = ethernet.phy_read(tcsr);
    d.clr = ethernet.phy_read(clr);
    d.tdr_available = true;
    return d;
}

}  // namespace fcu::net

extern "C" void ETH_IRQHandler() noexcept {
    if (fcu::net::ethernet.irq()) {
        kernel.notify_from_isr(fcu::net::net_tcb, fcu::net::slot_wake, 1, rtos::NotifyAction::SetBits);
    }
}

#if FCU_USB_DEVICE
extern "C" void USB_DRD_FS_IRQHandler() noexcept {
    fcu::net::usb_fs.on_irq();
    kernel.notify_from_isr(fcu::net::net_tcb, fcu::net::slot_wake, 1, rtos::NotifyAction::SetBits);
}
#endif
