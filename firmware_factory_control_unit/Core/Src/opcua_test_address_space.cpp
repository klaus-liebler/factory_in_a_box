#include "opcua_test_address_space.hpp"

#include "opcua/node_ids.hpp"
#include "tx_api.h"

using namespace opcua;

namespace {

// Computed fresh on every Read -- no periodic/cyclic callback engine exists yet in this
// from-scratch server (Subscriptions are "later", see the project's effort-estimation
// discussion), so "live" simply means "read straight from tx_time_get() at request time"
// instead of caching a value that something else refreshes on a timer. Not constexpr
// (tx_time_get() is a runtime RTOS call), but its ADDRESS is a compile-time constant, which is
// all a constexpr Node's ReadCallback field needs (see kNodes below).
Variant ReadUptimeSeconds(void * /*context*/) {
    return Variant::Of(static_cast<UInt32>(tx_time_get() / TX_TIMER_TICKS_PER_SECOND));
}

// Namespace 0's ObjectsFolder doesn't exist for free the way it would with a real NS0 nodeset
// compiled in (see the open62541 branch) -- this server has no NS0 machinery at all yet, so the
// one NS0 node an OPC UA client actually needs to start Browsing from (i=85) is listed here by
// hand, alongside this server's own three test nodes under namespace 1.
constexpr NodeId kFolderId(1, 1000);
constexpr NodeId kGreetingId(1, 1001);
constexpr NodeId kUptimeId(1, 1002);

// Entirely compile-time: no runtime "build the address space" step exists (project
// requirement, 2026-08-19, see address_space.hpp). Greeting has no WriteCallback (read-only,
// see its accessLevel) so its Value can be a plain constexpr Variant, placed in Flash with the
// rest of this table -- zero RAM. UptimeSeconds is the one genuinely dynamic node here, backed
// by ReadUptimeSeconds() above instead of a staticValue.
OPCUA_RODATA constexpr Node kNodes[] = {
    Node{
        .nodeId = NodeId(0, ns0::ObjectsFolder),
        .nodeClass = NodeClass::Object,
        .browseName = QualifiedName(0, "Objects"),
        .displayName = LocalizedText("en-US", "Objects"),
    },
    Node{
        .nodeId = kFolderId,
        .nodeClass = NodeClass::Object,
        .browseName = QualifiedName(1, "FactoryControlUnit"),
        .displayName = LocalizedText("en-US", "FactoryControlUnit"),
    },
    Node{
        .nodeId = kGreetingId,
        .nodeClass = NodeClass::Variable,
        .browseName = QualifiedName(1, "Greeting"),
        .displayName = LocalizedText("en-US", "Greeting"),
        .dataType = NodeId(0, ns0::DataType_String),
        .accessLevel = ns0::ACCESS_LEVEL_CURRENT_READ,
        .staticValue = Variant::Of(String("Hello from Factory Control Unit")),
    },
    Node{
        .nodeId = kUptimeId,
        .nodeClass = NodeClass::Variable,
        .browseName = QualifiedName(1, "UptimeSeconds"),
        .displayName = LocalizedText("en-US", "UptimeSeconds"),
        .dataType = NodeId(0, ns0::DataType_UInt32),
        .accessLevel = ns0::ACCESS_LEVEL_CURRENT_READ,
        .readCallback = &ReadUptimeSeconds,
    },
};

// Both directions listed explicitly (there is no runtime AddReference(..., addInverse=true)
// helper anymore, see address_space.hpp) -- HasTypeDefinition is forward-only, this server
// never needs to Browse "what has this TypeDefinition" in reverse.
constexpr ReferenceEntry kReferences[] = {
    {NodeId(0, ns0::ObjectsFolder), NodeId(0, ns0::Organizes), true, kFolderId},
    {kFolderId, NodeId(0, ns0::Organizes), false, NodeId(0, ns0::ObjectsFolder)},
    {kFolderId, NodeId(0, ns0::HasTypeDefinition), true, NodeId(0, ns0::FolderType)},

    {kFolderId, NodeId(0, ns0::HasComponent), true, kGreetingId},
    {kGreetingId, NodeId(0, ns0::HasComponent), false, kFolderId},
    {kGreetingId, NodeId(0, ns0::HasTypeDefinition), true, NodeId(0, ns0::BaseDataVariableType)},

    {kFolderId, NodeId(0, ns0::HasComponent), true, kUptimeId},
    {kUptimeId, NodeId(0, ns0::HasComponent), false, kFolderId},
    {kUptimeId, NodeId(0, ns0::HasTypeDefinition), true, NodeId(0, ns0::BaseDataVariableType)},
};

} // namespace

const AddressSpace &OpcUaTestAddressSpace() {
    static constexpr AddressSpace instance(kNodes, kReferences);
    return instance;
}
