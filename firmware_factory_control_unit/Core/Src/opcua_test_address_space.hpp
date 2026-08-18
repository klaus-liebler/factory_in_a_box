#pragma once
// Small handwritten test address space (NOT yet generated from register-map.json -- that's
// the planned next step once this transport/protocol layer is proven against a real client).
// Mirrors the same "Greeting" (static String) + "UptimeSeconds" (live, computed-on-read
// UInt32) shape used by the open62541-based port on the sibling branch, for an easy
// side-by-side comparison with the same OPC UA client.
#include "opcua/address_space.hpp"

void BuildOpcUaTestAddressSpace(opcua::AddressSpace &addressSpace);
