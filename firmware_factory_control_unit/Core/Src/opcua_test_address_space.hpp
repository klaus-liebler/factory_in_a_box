#pragma once
// Small handwritten test address space (NOT yet generated from register-map.json -- that's
// the planned next step once this transport/protocol layer is proven against a real client).
// Mirrors the same "Greeting" (constexpr String, Flash-resident) + "UptimeSeconds" (live,
// computed-on-read UInt32 via ReadCallback) shape used by the open62541-based port on the
// sibling branch, for an easy side-by-side comparison with the same OPC UA client.
//
// The whole table is a compile-time constant (project requirement, 2026-08-19) -- see
// opcua_test_address_space.cpp and opcua/address_space.hpp's header comment. This accessor is
// the only way to reach it; there is no runtime "build" step to call.
#include "opcua/address_space.hpp"

const opcua::AddressSpace &OpcUaTestAddressSpace();
