#pragma once
// NetX Duo/ThreadX transport: wires Connection (connection.hpp, pure protocol logic, knows
// nothing about NetX Duo) onto the existing NX_TCPSERVER addon
// (libs/ST/netxduo/addons/web/nx_tcpserver.h) -- same building block already used by
// Core/Src/http_websocket_server.hpp, and by the open62541-based port on the sibling branch's
// connectionmanager_tcp_netxduo.c (whose design notes on the NX_TCPSERVER integration mostly
// carry over here too, single-threaded callback model included).
//
// Everything (accept, receive, timeout, disconnect) runs on NX_TCPSERVER's own single worker
// thread -- there is no separate "pump" thread here, unlike the open62541 port, because this
// server has no EventLoop-style timer/subscription engine to drive yet (Subscriptions are
// explicitly "later", see the project's effort-estimation discussion).
#include <array>
#include <cstdint>
#include <string_view>

#include "opcua/address_space.hpp"
#include "opcua/connection.hpp"
#include "opcua/transport.hpp"

extern "C" {
#include "nx_api.h"
// nx_tcpserver.h has no extern "C" guard of its own (unlike essentially every other NetX/
// ThreadX header) -- without wrapping it here, its declarations get C++ name-mangled and
// won't link against the C-compiled nx_tcpserver.c symbols (identical issue already solved
// once in Core/Src/http_websocket_server.hpp, see its comment for the full explanation).
#include "nx_tcpserver.h"
}

namespace opcua {

class OpcUaTcpServer {
public:
    static constexpr UINT MAX_SESSIONS = 4;
    // Matches Connection::SEND_BUFFER_SIZE and the buffer sizes offered in Acknowledge --
    // must stay large enough for this server's biggest single response (currently: a Browse
    // result over the whole address space, comfortably under a few KB).
    static constexpr size_t RAW_BUFFER_SIZE = 8192;

    // endpointUrl must outlive this object (a server-owned, unchanging string, e.g. a
    // function-local static in opcua_setup.cpp).
    UINT Create(NX_IP *ipPtr, NX_PACKET_POOL *packetPool, void *threadStackPtr, UINT threadStackSize,
               UINT threadPriority, ULONG sessionTimeoutSeconds, const AddressSpace *addressSpace,
               std::string_view endpointUrl);
    UINT Start(UINT port, UINT listenBacklog);

private:
    class SessionTransport : public ITransport {
    public:
        void Bind(NX_TCP_SESSION *session, NX_PACKET_POOL *pool) {
            session_ = session;
            pool_ = pool;
            bound_ = true;
        }
        // Inverted (bound_, default false) instead of a directly-stored closed_=true default:
        // an all-zero default lets the whole OpcUaTcpServer (including every array element of
        // sessionStates_) live in .bss instead of .data -- a non-zero in-class default member
        // initializer anywhere in this object graph forces the compiler to emit an explicit
        // (non-zero) static initializer image for the WHOLE object instead. Same reasoning
        // applied to SecureChannel::serverSequenceNumber_ (secure_channel.hpp).
        bool Closed() const { return !bound_; }

        bool Send(std::span<const Byte> data) override;
        void RequestClose() override;

    private:
        NX_TCP_SESSION *session_ = nullptr;
        NX_PACKET_POOL *pool_ = nullptr;
        bool bound_ = false; // stays false until Bind() runs from OnNewConnection
    };

    struct SessionState {
        std::array<Byte, RAW_BUFFER_SIZE> recvBuffer{};
        size_t recvLength = 0;
        SessionTransport transport;
        Connection connection;
    };

    static void OnNewConnectionTrampoline(NX_TCPSERVER *server, NX_TCP_SESSION *session);
    static void OnReceiveDataTrampoline(NX_TCPSERVER *server, NX_TCP_SESSION *session);
    static void OnConnectionEndTrampoline(NX_TCPSERVER *server, NX_TCP_SESSION *session);

    void OnNewConnection(NX_TCP_SESSION *session);
    void OnReceiveData(NX_TCP_SESSION *session);
    void OnConnectionEnd(NX_TCP_SESSION *session);

    UINT SessionIndex(NX_TCP_SESSION *session) const {
        return static_cast<UINT>(session - tcpServer_.nx_tcpserver_sessions);
    }

    NX_TCPSERVER tcpServer_{};
    NX_PACKET_POOL *packetPool_ = nullptr;
    const AddressSpace *addressSpace_ = nullptr;
    std::string_view endpointUrl_;
    std::array<NX_TCP_SESSION, MAX_SESSIONS> sessions_{};
    std::array<SessionState, MAX_SESSIONS> sessionStates_{};
};

} // namespace opcua
