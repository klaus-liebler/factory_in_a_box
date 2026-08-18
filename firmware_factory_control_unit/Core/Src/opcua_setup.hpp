#pragma once
// OPC UA server (open62541 v1.4.18, vendored under libs/open62541/, NetX Duo/ThreadX port
// under libs/open62541_netxduo_arch/ -- see there for the design notes). First working
// version: SecurityPolicy#None only (no TLS/signing, per project decision), a small
// handwritten test address space (NOT yet generated from register-map.json).
//
// Entirely self-contained in its own ThreadX thread (App::opcua_thread, see app.cc) --
// builds the UA_ServerConfig, our own NetX Duo EventLoop + TCP ConnectionManager (opc.tcp://
// port 4840), the address space, then runs UA_Server_run_startup() followed by
// UA_Server_run_iterate() forever (the usual open62541 usage pattern). See
// open62541_netxduo_arch.h for why this must be a dedicated thread rather than folded into
// an existing one: it holds the EventLoop's recursive mutex only briefly per iteration,
// deliberately never across its internal sleep, so it can't stall the NX_TCPSERVER worker
// thread that delivers network events.
class App;

[[noreturn]] void OpcUaServerThread(App *app);
