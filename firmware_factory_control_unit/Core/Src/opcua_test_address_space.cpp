#include "opcua_test_address_space.hpp"

#include "opcua/node_ids.hpp"
#include "tx_api.h"

using namespace opcua;

namespace {

// Computed fresh on every Read -- no periodic/cyclic callback engine exists yet in this
// from-scratch server (Subscriptions are "later", see the project's effort-estimation
// discussion), so "live" simply means "read straight from tx_time_get() at request time"
// instead of caching a value that something else refreshes on a timer.
Variant ReadUptimeSeconds(void * /*context*/) {
    return Variant::Of(static_cast<UInt32>(tx_time_get() / TX_TIMER_TICKS_PER_SECOND));
}

} // namespace

void BuildOpcUaTestAddressSpace(AddressSpace &as) {
    // Namespace 0's ObjectsFolder doesn't exist for free the way it would with a real NS0
    // nodeset compiled in (see the open62541 branch) -- this server has no NS0 machinery at
    // all yet, so the one NS0 node an OPC UA client actually needs to start Browsing from
    // (i=85) is registered by hand here.
    as.AddObjectNode(NodeId(0, ns0::ObjectsFolder), QualifiedName(0, "Objects"),
                     LocalizedText("en-US", "Objects"));

    const NodeId folderId(1, 1000);
    as.AddObjectNode(folderId, QualifiedName(1, "FactoryControlUnit"),
                     LocalizedText("en-US", "FactoryControlUnit"));
    as.AddReference(NodeId(0, ns0::ObjectsFolder), NodeId(0, ns0::Organizes), folderId);
    as.AddReference(folderId, NodeId(0, ns0::HasTypeDefinition), NodeId(0, ns0::FolderType), false);

    const NodeId greetingId(1, 1001);
    as.AddVariableNode(greetingId, QualifiedName(1, "Greeting"), LocalizedText("en-US", "Greeting"),
                       NodeId(0, ns0::DataType_String), ns0::ACCESS_LEVEL_CURRENT_READ,
                       Variant::Of(String("Hello from Factory Control Unit")));
    as.AddReference(folderId, NodeId(0, ns0::HasComponent), greetingId);
    as.AddReference(greetingId, NodeId(0, ns0::HasTypeDefinition), NodeId(0, ns0::BaseDataVariableType), false);

    const NodeId uptimeId(1, 1002);
    as.AddVariableNodeWithCallback(uptimeId, QualifiedName(1, "UptimeSeconds"),
                                   LocalizedText("en-US", "UptimeSeconds"),
                                   NodeId(0, ns0::DataType_UInt32), ns0::ACCESS_LEVEL_CURRENT_READ,
                                   ReadUptimeSeconds, nullptr, nullptr);
    as.AddReference(folderId, NodeId(0, ns0::HasComponent), uptimeId);
    as.AddReference(uptimeId, NodeId(0, ns0::HasTypeDefinition), NodeId(0, ns0::BaseDataVariableType), false);
}
