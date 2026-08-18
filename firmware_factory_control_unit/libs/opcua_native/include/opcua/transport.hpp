#pragma once
// Abstraction the protocol/service layer (connection.hpp, secure_channel.hpp, session.hpp)
// sends framed UA TCP messages through -- deliberately NOT NetX Duo-specific, so none of that
// code needs to know about NX_TCP_SESSION/NX_PACKET at all. The one concrete implementation
// (net_transport.hpp/.cpp, added once the NX_TCPSERVER wiring is built) just does
// nx_tcp_socket_send()/nx_tcp_socket_disconnect() underneath.
#include <span>

#include "opcua/types.hpp"

namespace opcua {

class ITransport {
public:
    virtual ~ITransport() = default;

    // Sends exactly one complete, already-framed UA TCP message (its own 8-byte
    // MessageType/ChunkType/MessageSize header included). May be split across multiple
    // underlying packets by the implementation, but must be delivered as one ordered byte
    // range on the TCP stream -- callers never call Send() twice for what must arrive as a
    // single message.
    virtual bool Send(std::span<const Byte> data) = 0;

    // Requests the underlying TCP connection be torn down (e.g. after CloseSecureChannel, or
    // a fatal decode/protocol error). Asynchronous: no further Send() calls will be made
    // afterward, but the connection object itself is only reset once the transport's own
    // disconnect callback fires.
    virtual void RequestClose() = 0;
};

} // namespace opcua
