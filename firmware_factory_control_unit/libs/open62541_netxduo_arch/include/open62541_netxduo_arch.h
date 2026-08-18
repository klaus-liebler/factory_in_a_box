#pragma once
// NetX Duo / ThreadX port of open62541's EventLoop + TCP ConnectionManager (open62541 v1.4.18,
// vendored de-gitted under libs/open62541/, built with UA_ARCHITECTURE=none -- see root
// CMakeLists.txt for the cache-variable block). Deliberately NOT named "*_POSIX" like the
// functions this replaces (UA_EventLoop_new_POSIX/UA_ConnectionManager_new_POSIX_TCP): the
// caller (opcua_setup.cpp) always pre-populates UA_ServerConfig.eventLoop with the types
// declared here BEFORE calling any UA_ServerConfig_set*() function, so open62541's own
// (never-taken at runtime, but still linked -- see posix_stubs.c) POSIX-named constructors
// are never actually invoked.
//
// Built on top of the existing NX_TCPSERVER addon (libs/ST/netxduo/addons/web/nx_tcpserver.h)
// instead of a raw NX_TCP_SOCKET/select-based reactor: reuses the same accept/session/timeout
// machinery already proven by Core/Src/http_websocket_server.hpp, running entirely on
// NX_TCPSERVER's own single worker thread (its 4 callbacks call directly into the
// ConnectionManager's stored connectionCallback -- no separate network-polling thread).
//
// Threading model: the EventLoop is guarded by one recursive TX_MUTEX (UA_EventLoop::lock/
// unlock, wired to open62541's own lockServer()/unlockServer() -- see eventloop_netxduo.c for
// why this must be a REAL mutex here, unlike most single-threaded UA_ARCHITECTURE=none ports).
// Two ThreadX threads touch the server: the NX_TCPSERVER thread (network events, via the
// ConnectionManager below) and a dedicated "OPC UA pump" thread that the application must run,
// looping UA_Server_run_startup() once followed by UA_Server_run_iterate(server, true) forever
// (exactly the usual open62541 usage pattern) -- this drives cyclic/timed callbacks (secure
// channel housekeeping, subscriptions, ...) even while no network activity occurs. Both threads
// serialize against each other via that one mutex.
#include <open62541/plugin/eventloop.h>
#include "nx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

// One EventLoop instance per application. Never polls a socket itself -- all network I/O
// arrives out-of-band via the NX_TCPSERVER-backed ConnectionManager below; run() only
// services due timers/delayed callbacks, then sleeps (WITHOUT holding the lock, see
// eventloop_netxduo.c) until the next one is due.
UA_EventLoop *
UA_EventLoop_new_NetXDuo(const UA_Logger *logger);

// Must still be registered by the caller via eventLoop->registerEventSource(). Only ONE
// listening connection is supported (this application never opens more than one
// "opc.tcp://" server URL) -- a second openConnection(listen=true) call fails.
//
// ipPtr: the NX_IP instance to listen on. All of its attached interfaces (e.g. both the
// real Ethernet and the USB-NCM interface in this project) are reachable transparently,
// same as the existing Http::WebServer.
// packetPool: pool used for outgoing (send) packet allocation -- a dedicated pool, not
// necessarily ipPtr's own, matching the project convention of a per-subsystem pool (see
// net_setup.cpp "HTTP Server Pool").
// threadStackPtr/threadStackSize/threadPriority: caller-allocated stack + priority for the
// NX_TCPSERVER's own worker thread (identical convention to Http::WebServer::Create()).
// sessionTimeoutSeconds: idle-connection timeout, forwarded to nx_tcpserver_create().
UA_ConnectionManager *
UA_ConnectionManager_new_NetXDuo_TCP(const UA_String eventSourceName, NX_IP *ipPtr,
                                     NX_PACKET_POOL *packetPool,
                                     void *threadStackPtr, UINT threadStackSize,
                                     UINT threadPriority, UINT sessionTimeoutSeconds);

// UA_Logger that forwards to this project's own log.h (log_info()/log_warn()/...) instead of
// open62541's default UA_Log_Stdout (this board never retargets stdio to anything useful).
// Returns a pointer to a static instance -- do not free. Non-const to match
// UA_ServerConfig.logging's declared type (see log_bridge.c); nothing actually mutates it.
UA_Logger *
OpcUa_Log_Project(void);

#ifdef __cplusplus
}
#endif
