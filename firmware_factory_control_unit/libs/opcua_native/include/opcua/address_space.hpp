#pragma once
// Minimal, statically-allocated address space (Part 3): a flat array of Nodes plus a flat
// array of References between them -- no hash map, matches this project's static-allocation
// convention (see e.g. NX_TCP_SESSION arrays elsewhere) and comfortably covers the eventual
// ~70 register-map.json-derived Variable nodes plus a handful of NS0 nodes. MAX_NODES/
// MAX_REFERENCES are sized for that later step already, not just today's small test model.
//
// Two ways to back a Variable's Value attribute:
//  - No ReadCallback: the Value attribute IS staticValue, and Write() (if AccessLevel allows)
//    assigns directly into it -- for genuinely static or free-standing values (this server's
//    "Greeting"/"UptimeSeconds" test nodes).
//  - ReadCallback set (and optionally WriteCallback): Value is computed/forwarded on every
//    access -- the shape a future register-map.json-derived node uses to read/write through to
//    ModbusRegisterModel instead of duplicating storage.
#include <array>
#include <optional>

#include "opcua/node_ids.hpp"
#include "opcua/types.hpp"
#include "opcua/variant.hpp"

namespace opcua {

class AddressSpace;

using ReadCallback = Variant (*)(void *context);
// Returns false if the value's type/range is unacceptable (-> BadTypeMismatch/BadOutOfRange
// upstream in the Write service handler).
using WriteCallback = bool (*)(void *context, const Variant &value);

struct Node {
    NodeId nodeId;
    NodeClass nodeClass = NodeClass::Object;
    QualifiedName browseName;
    LocalizedText displayName;

    // --- Variable-only fields (ignored for Object nodes) ---
    NodeId dataType;
    Byte accessLevel = 0;
    Variant staticValue;
    ReadCallback readCallback = nullptr;
    WriteCallback writeCallback = nullptr;
    void *userContext = nullptr;

    Variant ReadValue() const { return readCallback ? readCallback(userContext) : staticValue; }
};

struct ReferenceEntry {
    NodeId sourceId;
    NodeId referenceTypeId;
    bool isForward = true; // true: sourceId --refType--> targetId; false: the inverse direction
    NodeId targetId;
};

class AddressSpace {
public:
    static constexpr size_t MAX_NODES = 128;
    static constexpr size_t MAX_REFERENCES = 256;

    // --- Registration (call during startup only, before the server accepts connections;
    // none of this is safe to call concurrently with lookups from a network thread). ---
    bool AddObjectNode(const NodeId &id, const QualifiedName &browseName, const LocalizedText &displayName);
    // Static Variable: Value attribute is "initialValue" itself, mutable via Write() in place
    // if accessLevel allows (no ReadCallback/WriteCallback).
    bool AddVariableNode(const NodeId &id, const QualifiedName &browseName, const LocalizedText &displayName,
                        const NodeId &dataType, Byte accessLevel, Variant initialValue);
    // Callback-backed Variable: Value attribute is computed/forwarded through read/write.
    bool AddVariableNodeWithCallback(const NodeId &id, const QualifiedName &browseName,
                                     const LocalizedText &displayName, const NodeId &dataType,
                                     Byte accessLevel, ReadCallback read, WriteCallback write,
                                     void *context);

    // Adds a forward reference sourceId --referenceTypeId--> targetId, and (unless
    // addInverse=false) the matching inverse entry from targetId's perspective -- Browse
    // (connection.cpp) needs to walk both directions (e.g. HasComponent forward from a folder
    // to find its children, but also inverse from a Variable to find its parent for
    // TranslateBrowsePathsToNodeIds later).
    bool AddReference(const NodeId &sourceId, const NodeId &referenceTypeId, const NodeId &targetId,
                      bool addInverse = true);

    const Node *FindNode(const NodeId &id) const;
    Node *FindNodeMutable(const NodeId &id);

    // Invokes fn(const ReferenceEntry&) for every reference where sourceId matches "nodeId" in
    // the requested direction (isForward == wantForward) -- the core of the Browse service.
    template <typename F>
    void ForEachReference(const NodeId &nodeId, bool wantForward, F &&fn) const {
        for(size_t i = 0; i < referenceCount_; i++) {
            const ReferenceEntry &ref = references_[i];
            if(ref.sourceId == nodeId && ref.isForward == wantForward)
                fn(ref);
        }
    }

private:
    Node *AddNodeCommon(const NodeId &id, NodeClass nodeClass, const QualifiedName &browseName,
                        const LocalizedText &displayName);

    std::array<Node, MAX_NODES> nodes_;
    size_t nodeCount_ = 0;
    std::array<ReferenceEntry, MAX_REFERENCES> references_;
    size_t referenceCount_ = 0;
};

} // namespace opcua
