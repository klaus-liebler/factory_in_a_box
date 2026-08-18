#pragma once
// From-scratch OPC UA Binary server (see libs/opcua_native/ for the protocol/transport
// implementation) -- SecurityPolicy#None only, small handwritten test address space (see
// opcua_test_address_space.hpp/.cpp). Unlike the open62541-based port on the sibling branch,
// no dedicated ThreadX thread is needed here: nx_tcpserver_create() (called from
// OpcUaTcpServer::Create(), see libs/opcua_native/src/net_transport.cpp) spawns and
// auto-starts its own worker thread, and this server has no separate timer/subscription
// engine that would need a second thread pumping it (Subscriptions are "later", see the
// project's effort-estimation discussion) -- so this setup function just builds everything
// and returns, same as ModbusTcpServer's construction in App::AppThread().
class App;

void OpcUaServerSetup(App *app);
