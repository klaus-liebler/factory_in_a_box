#include "opcua_setup.hpp"

#include <cstdio>

#include "opcua/net_transport.hpp"
#include "generated/opcua_registers_generated.hh"

#include "app.hh"
#include "common_macros.hh"
#include "log.h"
#include "main.h"

namespace {

constexpr UINT OPCUA_PORT = 4840;
constexpr UINT OPCUA_SESSION_TIMEOUT_SECONDS = 60;
constexpr uint32_t OPCUA_PACKET_SIZE = 1536;
constexpr uint32_t OPCUA_POOL_SIZE = OPCUA_PACKET_SIZE * 8;
// The NX_TCPSERVER worker thread this stack belongs to is where ALL request processing
// happens (OnReceiveData calls straight into Connection::HandleMessage(), synchronously) --
// matches the open62541 branch's identically-sized stack for the same reason (see its
// opcua_setup.cpp comment).
constexpr uint32_t OPCUA_THREAD_STACK_SIZE = 8192;
// Same band as the Modbus TCP server thread (DEFAULT_PRIORITY in app.cc) -- reacts to client
// requests, doesn't need to preempt the I/O thread's sensor/actuator cycle.
constexpr UINT OPCUA_THREAD_PRIORITY = 6;

// nx_packet_pool_create()/OpcUaTcpServer both keep pointers into these for as long as the
// server runs (the whole program's lifetime here) -- must not be stack-locals.
NX_PACKET_POOL g_opcuaPacketPool;
opcua::OpcUaTcpServer g_server;
char g_endpointUrl[64];

} // namespace

void OpcUaServerSetup(App *app) {
    // Wires the compile-time-fixed generated Node table's Read/WriteCallbacks (see
    // opcua_registers_generated.hh -- one shared callback per base type, driven by a per-register
    // RegisterContext) to the runtime register storage. Must happen before AddressSpaceInstance()
    // is handed to the server below; app->register_model already exists at this point (created in
    // App::SetupBeforeThreadX(), long before AppThread()/OpcUaServerSetup() run).
    GeneratedOpcUa::SetRegisterModel(app->register_model);

    void *ptr = nullptr;
    XASSERT(tx_byte_allocate(&app->byte_pool, &ptr, OPCUA_POOL_SIZE, TX_NO_WAIT),
            "OPC UA packet pool allocate failed");
    XASSERT(nx_packet_pool_create(&g_opcuaPacketPool, _C("OPC UA Packet Pool"),
                                  OPCUA_PACKET_SIZE, ptr, OPCUA_POOL_SIZE),
            "OPC UA packet pool create failed");

    XASSERT(tx_byte_allocate(&app->byte_pool, &ptr, OPCUA_THREAD_STACK_SIZE, TX_NO_WAIT),
            "OPC UA TCP server thread stack allocate failed");

    // No stable hostname is available yet at this point in the boot sequence (DHCP hasn't
    // resolved); "opc.tcp://:<port>" (empty host) is valid per Part 6 7.1.2.3 and is what
    // clients connect to by IP/port anyway.
    snprintf(g_endpointUrl, sizeof(g_endpointUrl), "opc.tcp://:%u", static_cast<unsigned>(OPCUA_PORT));

    XASSERT(g_server.Create(&app->ip_instance, &g_opcuaPacketPool, ptr, OPCUA_THREAD_STACK_SIZE,
                            OPCUA_THREAD_PRIORITY, OPCUA_SESSION_TIMEOUT_SECONDS,
                            &GeneratedOpcUa::AddressSpaceInstance(), g_endpointUrl),
            "OPC UA TCP server create failed");
    XASSERT(g_server.Start(OPCUA_PORT, opcua::OpcUaTcpServer::MAX_SESSIONS * 2),
            "OPC UA TCP server start failed");

    log_info("OPC UA Server started on %s (SecurityPolicy#None)", g_endpointUrl);
}
