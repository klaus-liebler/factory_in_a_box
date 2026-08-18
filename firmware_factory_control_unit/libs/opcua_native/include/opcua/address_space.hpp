#pragma once
// The address space is a compile-time-fixed, non-owning view over externally-defined constexpr
// tables (project requirement, 2026-08-19) -- NOT built at runtime via AddObjectNode()/
// AddVariableNode()/AddReference()-style function calls. AddressSpace itself is just two spans
// plus lookup helpers; the actual Node/ReferenceEntry data lives wherever the caller defines it
// (e.g. opcua_test_address_space.cpp today, a register-map.json-generated table later) as a
// `constexpr Node kNodes[] = {...}` array with designated initializers.
//
// Nodes are immutable by design: a genuinely writable Variable node MUST go through a
// WriteCallback pointing at externally-owned, explicitly-mutable RAM (e.g. a future
// ModbusRegisterModel-backed register) -- there is no "static value the server quietly mutates
// in place" path. This keeps every constexpr node table trivially placeable in Flash (.rodata)
// with zero RAM cost, and makes "is this node's storage mutable" a question answered entirely
// by whether it has a WriteCallback, not by inspecting AddressSpace internals.
#include <span>

#include "opcua/node_ids.hpp"
#include "opcua/types.hpp"
#include "opcua/variant.hpp"

// Apply to every `constexpr Node kNodes[] = {...}` table that contains a ReadCallback/
// WriteCallback (a function pointer): GCC's default section-selection heuristic treats any
// const object holding a pointer-to-code as needing load-time relocation (correct for
// PIC/shared-library targets) and silently places it in .data instead of .rodata -- meaning
// it would still cost RAM plus a startup copy, exactly what constexpr was meant to avoid. On
// this bare-metal, statically-linked target there is no runtime relocation step at all (the
// linker resolves every function address to its final Flash location at link time), so it's
// always safe to force true .rodata placement here. Without this, `nm`/`size` on the object
// file would show the table as `d` (.data) instead of `r` (.rodata) -- verify with those tools
// if in doubt, `constexpr` alone does not guarantee Flash placement once function pointers are
// involved.
#define OPCUA_RODATA __attribute__((section(".rodata")))

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
    // Constant Value for a node with no ReadCallback (e.g. a fixed test/status string) --
    // ignored if readCallback is set. Never mutated at runtime; see the file header comment
    // for why a truly dynamic value must be a ReadCallback instead.
    Variant staticValue;
    ReadCallback readCallback = nullptr;
    // Set together with a WriteCallback for a genuinely writable Variable -- a node with the
    // Write access bit set but no WriteCallback is a node-definition bug and is rejected at
    // Write time (see services.cpp WriteNodeAttribute), not silently allowed to "work" via
    // staticValue mutation.
    WriteCallback writeCallback = nullptr;
    void *userContext = nullptr;

    constexpr Variant ReadValue() const { return readCallback ? readCallback(userContext) : staticValue; }
};

struct ReferenceEntry {
    NodeId sourceId;
    NodeId referenceTypeId;
    bool isForward = true; // true: sourceId --refType--> targetId; false: the inverse direction
    NodeId targetId;
};

class AddressSpace {
public:
    constexpr AddressSpace(std::span<const Node> nodes, std::span<const ReferenceEntry> references)
        : nodes_(nodes), references_(references) {}

    const Node *FindNode(const NodeId &id) const {
        for(const Node &n : nodes_) {
            if(n.nodeId == id) return &n;
        }
        return nullptr;
    }

    // Invokes fn(const ReferenceEntry&) for every reference where sourceId matches "nodeId" in
    // the requested direction (isForward == wantForward) -- the core of the Browse service.
    template <typename F>
    void ForEachReference(const NodeId &nodeId, bool wantForward, F &&fn) const {
        for(const ReferenceEntry &ref : references_) {
            if(ref.sourceId == nodeId && ref.isForward == wantForward)
                fn(ref);
        }
    }

private:
    std::span<const Node> nodes_;
    std::span<const ReferenceEntry> references_;
};

} // namespace opcua
