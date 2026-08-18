#include "opcua_setup.hpp"

#include "open62541_netxduo_arch.h"
#include <open62541/server.h>
#include <open62541/server_config_default.h>
#include <open62541/util.h>

#include "app.hh"
#include "common_macros.hh"
#include "log.h"
#include "main.h"

#include <cstring>

namespace {

constexpr UINT OPCUA_PORT = 4840;
// Small handwritten test address space (no large arrays/strings) -- 8KB chunks are plenty
// and keep the per-channel receive-buffer allocation (see NXTCP_allocNetworkBuffer, plain
// heap malloc) modest across up to UA_NX_TCP_MAX_SESSIONS connections at once. Left at the
// open62541 default (64kB) this would be 64kB * 2 (send+recv) * up to 4 sessions = 512kB,
// most of this project's "huge RAM headroom" (project notes) gone to buffers that are never
// filled anywhere near that size by this tiny test model.
constexpr UA_UInt32 OPCUA_BUFFER_SIZE = 8192;
constexpr UINT OPCUA_MAX_SESSIONS = 4;
constexpr UINT OPCUA_SESSION_TIMEOUT_SECONDS = 60;
constexpr uint32_t OPCUA_PACKET_SIZE = 1536;
constexpr uint32_t OPCUA_POOL_SIZE = OPCUA_PACKET_SIZE * 8;
// The NX_TCPSERVER worker thread this stack belongs to is where ALL actual OPC UA message
// processing happens (NXTCP_onReceiveData calls straight into the core's binary-chunk
// decode/secure-channel/service-dispatch/encode chain, synchronously) -- this project's
// existing HTTPS server needed 16KB for its TLS handshake alone (SERVER_STACK, net_setup.cpp)
// and noted plain HTTP fit in 4KB; OPC UA binary processing (no TLS here, but a deeper
// service-dispatch call chain than plain HTTP) gets a comparable safety margin.
constexpr uint32_t OPCUA_TCPSERVER_STACK_SIZE = 8192;
// Same priority band as the Modbus TCP server (App::modbus_tcp_server_thread, DEFAULT_PRIORITY
// in app.cc) -- reacts to client requests, but doesn't need to preempt the I/O thread's
// sensor/actuator cycle.
constexpr UINT OPCUA_TCPSERVER_THREAD_PRIORITY = 6;

// nx_packet_pool_create() keeps this alive for as long as the pool exists (the whole
// program's lifetime here) -- must not be a stack-local of OpcUaServerThread().
NX_PACKET_POOL g_opcuaPacketPool;

UA_NodeId g_uptimeNodeId;

void
BuildAddressSpace(UA_Server *server) {
    // Objects/FactoryControlUnit folder
    UA_NodeId folderId;
    UA_ObjectAttributes folderAttr = UA_ObjectAttributes_default;
    folderAttr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)"FactoryControlUnit");
    UA_Server_addObjectNode(server, UA_NODEID_NULL,
                            UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                            UA_QUALIFIEDNAME(1, (char*)"FactoryControlUnit"),
                            UA_NODEID_NUMERIC(0, UA_NS0ID_FOLDERTYPE),
                            folderAttr, NULL, &folderId);

    // .../Greeting (static string)
    UA_VariableAttributes greetingAttr = UA_VariableAttributes_default;
    UA_String greetingValue = UA_STRING((char*)"Hello from Factory Control Unit");
    UA_Variant_setScalar(&greetingAttr.value, &greetingValue, &UA_TYPES[UA_TYPES_STRING]);
    greetingAttr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)"Greeting");
    greetingAttr.dataType = UA_TYPES[UA_TYPES_STRING].typeId;
    greetingAttr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_Server_addVariableNode(server, UA_NODEID_NULL, folderId,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, (char*)"Greeting"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              greetingAttr, NULL, NULL);

    // .../UptimeSeconds (dynamic, updated by the repeated callback below)
    UA_VariableAttributes uptimeAttr = UA_VariableAttributes_default;
    UA_UInt32 uptimeInitial = 0;
    UA_Variant_setScalar(&uptimeAttr.value, &uptimeInitial, &UA_TYPES[UA_TYPES_UINT32]);
    uptimeAttr.displayName = UA_LOCALIZEDTEXT((char*)"en-US", (char*)"UptimeSeconds");
    uptimeAttr.dataType = UA_TYPES[UA_TYPES_UINT32].typeId;
    uptimeAttr.accessLevel = UA_ACCESSLEVELMASK_READ;
    UA_Server_addVariableNode(server, UA_NODEID_NULL, folderId,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                              UA_QUALIFIEDNAME(1, (char*)"UptimeSeconds"),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              uptimeAttr, NULL, &g_uptimeNodeId);
}

void
UpdateUptime(UA_Server *server, void *data) {
    (void)data;
    UA_UInt32 uptimeSeconds = (UA_UInt32)(tx_time_get() / TX_TIMER_TICKS_PER_SECOND);
    UA_Variant value;
    UA_Variant_setScalar(&value, &uptimeSeconds, &UA_TYPES[UA_TYPES_UINT32]);
    UA_Server_writeValue(server, g_uptimeNodeId, value);
}

} // namespace

[[noreturn]] void
OpcUaServerThread(App *app) {
    void *ptr = nullptr;
    XASSERT(tx_byte_allocate(&app->byte_pool, &ptr, OPCUA_POOL_SIZE, TX_NO_WAIT),
            "OPC UA packet pool allocate failed");
    XASSERT(nx_packet_pool_create(&g_opcuaPacketPool, _C("OPC UA Packet Pool"),
                                  OPCUA_PACKET_SIZE, ptr, OPCUA_POOL_SIZE),
            "OPC UA packet pool create failed");

    XASSERT(tx_byte_allocate(&app->byte_pool, &ptr, OPCUA_TCPSERVER_STACK_SIZE, TX_NO_WAIT),
            "OPC UA TCP server thread stack allocate failed");
    void *tcpServerStack = ptr;

    UA_ServerConfig config;
    memset(&config, 0, sizeof(config));
    config.logging = OpcUa_Log_Project();

    config.eventLoop = UA_EventLoop_new_NetXDuo(config.logging);
    if(!config.eventLoop) {
        log_error("OPC UA EventLoop create failed");
        Error_Handler();
    }

    UA_ConnectionManager *tcpCM = UA_ConnectionManager_new_NetXDuo_TCP(
        UA_STRING((char*)"opcua tcp connection manager"), &app->ip_instance, &g_opcuaPacketPool,
        tcpServerStack, OPCUA_TCPSERVER_STACK_SIZE, OPCUA_TCPSERVER_THREAD_PRIORITY,
        OPCUA_SESSION_TIMEOUT_SECONDS);
    if(!tcpCM) {
        log_error("OPC UA ConnectionManager create failed");
        Error_Handler();
    }
    XASSERT(config.eventLoop->registerEventSource(config.eventLoop, (UA_EventSource*)tcpCM),
            "OPC UA ConnectionManager register failed");

    XASSERT(UA_ServerConfig_setMinimalCustomBuffer(&config, OPCUA_PORT, NULL,
                                                   OPCUA_BUFFER_SIZE, OPCUA_BUFFER_SIZE),
            "OPC UA ServerConfig setup failed");
    // Matches UA_NX_TCP_MAX_SESSIONS (connectionmanager_tcp_netxduo.c) -- no point allowing
    // more secure channels/sessions than the transport can ever actually carry.
    config.maxSecureChannels = OPCUA_MAX_SESSIONS;
    config.maxSessions = OPCUA_MAX_SESSIONS;

    XASSERT(UA_ServerConfig_addSecurityPolicyNone(&config, NULL),
            "OPC UA SecurityPolicy#None add failed");
    XASSERT(UA_ServerConfig_addAllEndpoints(&config),
            "OPC UA endpoint add failed");

    UA_Server *server = UA_Server_newWithConfig(&config);
    if(!server) {
        log_error("OPC UA Server create failed");
        Error_Handler();
    }

    BuildAddressSpace(server);
    UA_Server_addRepeatedCallback(server, UpdateUptime, NULL, 1000.0, NULL);

    XASSERT(UA_Server_run_startup(server), "OPC UA Server startup failed");
    log_info("OPC UA Server started on opc.tcp://:%u (SecurityPolicy#None)", (unsigned)OPCUA_PORT);

    while(true) {
        UA_Server_run_iterate(server, true);
    }
}
