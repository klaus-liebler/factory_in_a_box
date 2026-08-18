#pragma once
// Fundamental OPC UA Binary (Part 6) built-in types -- deliberately only what this server
// actually needs today (see the project's own effort-estimation discussion: the "full type
// canon" is added later purely by extending the Variant alternative list + TypeTable entries
// in variant.hpp/.cpp, no rework of the codec itself). Header-only where trivial; encode/decode
// logic lives in codec.hpp/.cpp (needs ByteReader/ByteWriter, which would create an include
// cycle if pulled in here).
//
// No exceptions/RTTI on this target (-fno-exceptions -fno-rtti, see cmake/gcc-arm-none-eabi.cmake)
// -- every fallible operation in this whole server returns bool/StatusCode, never throws.
#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <string>
#include <string_view>
#include <variant>

namespace opcua {

using Boolean = bool;
using SByte = int8_t;
using Byte = uint8_t;
using Int16 = int16_t;
using UInt16 = uint16_t;
using Int32 = int32_t;
using UInt32 = uint32_t;
using Int64 = int64_t;
using UInt64 = uint64_t;
using Float = float;
using Double = double;

// 100ns ticks since 1601-01-01 (the OPC UA / Windows FILETIME epoch) -- same convention used
// project-wide for any future RTC/SNTP integration, kept consistent here even though this
// board has neither (see DateTimeNow() in clock.cpp, tx_time_get()-based like the open62541
// port's arch layer was).
using DateTime = int64_t;
constexpr int64_t DATETIME_UNIX_EPOCH = 11644473600LL * 10000000LL;
constexpr int64_t DATETIME_SEC = 10000000LL;

// OPC UA Part 6 6.2.5: length -1 (0xFFFFFFFF on the wire) is the "null string", distinct from
// a present-but-empty string. std::string cannot represent that distinction on its own, so it
// carries alongside a bool -- default-constructed String() is null (matches a not-yet-set
// optional attribute), String("") is the empty-but-present string.
struct String {
    std::string value;
    bool isNull = true;

    String() = default;
    String(std::string_view v) : value(v), isNull(false) {}
    static String Null() { return String{}; }
};

// Part 6 5.1.3: StatusCode is a plain UInt32, top 16 bits are the severity+code, low 16 bits
// are (mostly-unused-by-us) sub-flags. Only the handful of codes this server actually returns
// are named here -- add more as needed, exactly like the Variant type table.
enum class StatusCode : UInt32 {
    Good = 0x00000000,
    BadInternalError = 0x80020000,
    BadOutOfMemory = 0x80030000,
    BadNotImplemented = 0x80040000,
    BadNotSupported = 0x80100000,
    BadInvalidArgument = 0x80AB0000,
    BadOutOfRange = 0x803D0000,
    BadNodeIdUnknown = 0x80330000,
    BadNodeIdInvalid = 0x80320000,
    BadAttributeIdInvalid = 0x80350000,
    BadNotReadable = 0x803A0000,
    BadNotWritable = 0x803B0000,
    BadTypeMismatch = 0x80740000,
    BadUserAccessDenied = 0x801F0000,
    BadSecurityChecksFailed = 0x80130000,
    BadSecurityPolicyRejected = 0x80550000,
    BadSecurityModeRejected = 0x80620000,
    BadSecureChannelIdInvalid = 0x80300000,
    BadSecureChannelClosed = 0x80560000,
    BadRequestTypeInvalid = 0x80B00000,
    BadRequestHeaderInvalid = 0x802A0000,
    BadTimestampsToReturnInvalid = 0x80DB0000,
    BadSessionIdInvalid = 0x80250000,
    BadSessionClosed = 0x80260000,
    BadSessionNotActivated = 0x80270000,
    BadIdentityTokenInvalid = 0x80230000,
    BadIdentityTokenRejected = 0x80240000,
    BadTooManySessions = 0x80690000,
    BadServiceUnsupported = 0x800B0000,
    BadDecodingError = 0x80070000,
    BadEncodingError = 0x80060000,
    BadEncodingLimitsExceeded = 0x80080000,
    BadNothingToDo = 0x80140000,
    BadNoMatch = 0x80390000,
    BadTcpMessageTypeInvalid = 0x807C0000,
    BadTcpMessageTooLarge = 0x807E0000,
    BadConnectionClosed = 0x80AE0000,
    BadUnexpectedError = 0x80010000,
    UncertainInitialValue = 0x40920000,
};

constexpr bool IsGood(StatusCode s) { return (static_cast<UInt32>(s) & 0x80000000u) == 0; }
constexpr bool IsBad(StatusCode s) { return !IsGood(s); }

// Part 6 5.2.2: three encodings actually used on the wire by this server -- Numeric (all our
// own NodeIds, spec-optimized into Two-Byte/Four-Byte/Numeric sub-forms on ENCODE) plus String/
// Guid/Opaque on DECODE only (a client could in principle echo one back, even though we never
// hand one out -- see codec.cpp DecodeNodeId()). NamespaceIndex 0 is reserved for the standard
// OPC UA namespace (NS0); this server's own address space lives in namespace 1.
enum class NodeIdType : Byte { Numeric, String, Guid, Opaque };

struct Guid {
    UInt32 data1 = 0;
    UInt16 data2 = 0;
    UInt16 data3 = 0;
    Byte data4[8] = {};
};

struct NodeId {
    UInt16 namespaceIndex = 0;
    NodeIdType type = NodeIdType::Numeric;
    UInt32 numeric = 0;
    std::string string;   // used for String and (raw bytes) Opaque
    Guid guid;

    NodeId() = default;
    constexpr NodeId(UInt16 ns, UInt32 id) : namespaceIndex(ns), type(NodeIdType::Numeric), numeric(id) {}

    friend bool operator==(const NodeId &a, const NodeId &b) {
        if(a.namespaceIndex != b.namespaceIndex || a.type != b.type)
            return false;
        switch(a.type) {
        case NodeIdType::Numeric: return a.numeric == b.numeric;
        case NodeIdType::String:  return a.string == b.string;
        case NodeIdType::Opaque:  return a.string == b.string;
        case NodeIdType::Guid:
            return a.guid.data1 == b.guid.data1 && a.guid.data2 == b.guid.data2 &&
                   a.guid.data3 == b.guid.data3 &&
                   std::equal(std::begin(a.guid.data4), std::end(a.guid.data4), std::begin(b.guid.data4));
        }
        return false;
    }
    friend bool operator!=(const NodeId &a, const NodeId &b) { return !(a == b); }

    bool IsNull() const { return namespaceIndex == 0 && type == NodeIdType::Numeric && numeric == 0; }
};

// ExpandedNodeId (Part 6 5.2.3): NodeId + optional namespaceUri/serverIndex. This server never
// references another server or a namespace-by-URI, so it's kept to "just a NodeId" -- the
// encoder always writes the two optional fields absent (matches how open62541 and most minimal
// servers behave for local-only address spaces).
using ExpandedNodeId = NodeId;

struct QualifiedName {
    UInt16 namespaceIndex = 0;
    std::string name;

    QualifiedName() = default;
    QualifiedName(UInt16 ns, std::string_view n) : namespaceIndex(ns), name(n) {}
};

struct LocalizedText {
    std::string locale;
    std::string text;

    LocalizedText() = default;
    LocalizedText(std::string_view loc, std::string_view t) : locale(loc), text(t) {}
};

// NodeClass (Part 3 5.2.2) -- only the two classes this server's address space actually uses.
enum class NodeClass : Int32 {
    Unspecified = 0,
    Object = 1,
    Variable = 2,
};

} // namespace opcua
