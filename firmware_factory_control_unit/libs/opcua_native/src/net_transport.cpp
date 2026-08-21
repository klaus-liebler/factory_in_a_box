#include "opcua/net_transport.hpp"

#include <cstring>

#include "opcua/tcp_message.hpp"
#include "log.h"

namespace opcua {

bool OpcUaTcpServer::SessionTransport::Send(std::span<const Byte> data) {
    if(!bound_ || !session_) return false;

    NX_PACKET *packet = NX_NULL;
    if(nx_packet_allocate(pool_, &packet, NX_TCP_PACKET, NX_WAIT_FOREVER) != NX_SUCCESS)
        return false;
    if(nx_packet_data_append(packet, const_cast<Byte*>(data.data()), static_cast<ULONG>(data.size()),
                             pool_, NX_WAIT_FOREVER) != NX_SUCCESS) {
        nx_packet_release(packet);
        return false;
    }
    if(nx_tcp_socket_send(&session_->nx_tcp_session_socket, packet, NX_WAIT_FOREVER) != NX_SUCCESS) {
        nx_packet_release(packet);
        return false;
    }
    return true;
}

void OpcUaTcpServer::SessionTransport::RequestClose() {
    if(!bound_ || !session_) return;
    bound_ = false;
    nx_tcp_socket_disconnect(&session_->nx_tcp_session_socket, NX_NO_WAIT);
    nx_tcp_server_socket_unaccept(&session_->nx_tcp_session_socket);
}

UINT OpcUaTcpServer::Create(NX_IP *ipPtr, NX_PACKET_POOL *packetPool, void *threadStackPtr,
                           UINT threadStackSize, UINT threadPriority, ULONG sessionTimeoutSeconds,
                           const AddressSpace *addressSpace, std::string_view endpointUrl) {
    packetPool_ = packetPool;
    addressSpace_ = addressSpace;
    endpointUrl_ = endpointUrl;

    UINT status = nx_tcpserver_create(
        ipPtr, &tcpServer_, const_cast<CHAR*>("OPC UA TCP Server"),
        NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 8192,
        OnNewConnectionTrampoline, OnReceiveDataTrampoline,
        OnConnectionEndTrampoline, OnConnectionEndTrampoline,
        sessionTimeoutSeconds, threadStackPtr, threadStackSize,
        sessions_.data(), static_cast<UINT>(sessions_.size() * sizeof(NX_TCP_SESSION)),
        threadPriority, NX_IP_PERIODIC_RATE);
    if(status != NX_SUCCESS)
        return status;

    // Back-pointer so the static NX_TCPSERVER callback trampolines can recover "this" from the
    // bare NX_TCPSERVER* they are handed -- nx_tcpserver_reserved exists in the struct
    // specifically for application use (see nx_tcpserver.h).
    tcpServer_.nx_tcpserver_reserved = reinterpret_cast<ULONG>(this);
    return NX_SUCCESS;
}

UINT OpcUaTcpServer::Start(UINT port, UINT listenBacklog) {
    return nx_tcpserver_start(&tcpServer_, port, listenBacklog);
}

void OpcUaTcpServer::OnNewConnectionTrampoline(NX_TCPSERVER *server, NX_TCP_SESSION *session) {
    reinterpret_cast<OpcUaTcpServer*>(server->nx_tcpserver_reserved)->OnNewConnection(session);
}
void OpcUaTcpServer::OnReceiveDataTrampoline(NX_TCPSERVER *server, NX_TCP_SESSION *session) {
    reinterpret_cast<OpcUaTcpServer*>(server->nx_tcpserver_reserved)->OnReceiveData(session);
}
void OpcUaTcpServer::OnConnectionEndTrampoline(NX_TCPSERVER *server, NX_TCP_SESSION *session) {
    reinterpret_cast<OpcUaTcpServer*>(server->nx_tcpserver_reserved)->OnConnectionEnd(session);
}

void OpcUaTcpServer::OnNewConnection(NX_TCP_SESSION *session) {
    UINT index = SessionIndex(session);
    SessionState &state = sessionStates_[index];
    state.recvLength = 0;
    state.transport.Bind(session, packetPool_);
    state.connection.Reset(index, &state.transport, addressSpace_, endpointUrl_);
    log_debug("OPC UA: slot %u new TCP connection", index);
}

void OpcUaTcpServer::OnConnectionEnd(NX_TCP_SESSION *session) {
    UINT index = SessionIndex(session);
    SessionState &state = sessionStates_[index];
    log_debug("OPC UA: slot %u connection end (state=%u)", index,
             static_cast<unsigned>(session->nx_tcp_session_socket.nx_tcp_socket_state));
    state.transport.RequestClose(); // idempotent -- also the path a remote-initiated close takes
    state.recvLength = 0;
}

void OpcUaTcpServer::OnReceiveData(NX_TCP_SESSION *session) {
    UINT index = SessionIndex(session);
    SessionState &state = sessionStates_[index];
    if(state.transport.Closed())
        return;

    for(;;) {
        NX_PACKET *packet = NX_NULL;
        if(nx_tcp_socket_receive(&session->nx_tcp_session_socket, &packet, NX_NO_WAIT) != NX_SUCCESS)
            break;

        for(NX_PACKET *p = packet; p != NX_NULL; p = p->nx_packet_next) {
            size_t fragmentLength = static_cast<size_t>(p->nx_packet_append_ptr - p->nx_packet_prepend_ptr);
            if(state.recvLength + fragmentLength > RAW_BUFFER_SIZE) {
                nx_packet_release(packet);
                state.transport.RequestClose();
                return;
            }
            std::memcpy(state.recvBuffer.data() + state.recvLength, p->nx_packet_prepend_ptr, fragmentLength);
            state.recvLength += fragmentLength;
        }
        nx_packet_release(packet);
    }

    // Extract and process as many complete messages as are currently buffered -- a client may
    // pipeline several small requests into one TCP segment.
    for(;;) {
        if(state.recvLength < TCP_HEADER_SIZE)
            break;

        TcpHeader header;
        std::span<const Byte> buffered(state.recvBuffer.data(), state.recvLength);
        if(!PeekTcpHeader(buffered, header)) {
            log_debug("OPC UA: slot %u malformed TCP header, closing (recvLength=%u)",
                     index, static_cast<unsigned>(state.recvLength));
            state.transport.RequestClose();
            return;
        }
        // MaxMessageSize offered in Acknowledge (see connection.cpp) equals RAW_BUFFER_SIZE --
        // a compliant client never exceeds it; a header claiming otherwise is either corrupt
        // or a non-compliant client, either way not something to buffer for.
        if(header.messageSize < TCP_HEADER_SIZE || header.messageSize > RAW_BUFFER_SIZE) {
            log_debug("OPC UA: slot %u '%c%c%c' claims oversized messageSize=%u, closing",
                     index, header.messageType[0], header.messageType[1], header.messageType[2],
                     static_cast<unsigned>(header.messageSize));
            state.transport.RequestClose();
            return;
        }
        if(state.recvLength < header.messageSize)
            break; // wait for the rest of this message to arrive

        log_debug("OPC UA: slot %u recv '%c%c%c' size=%u", index, header.messageType[0],
                 header.messageType[1], header.messageType[2],
                 static_cast<unsigned>(header.messageSize));
        state.connection.HandleMessage(buffered.subspan(0, header.messageSize));
        if(state.transport.Closed()) {
            log_debug("OPC UA: slot %u closed by HandleMessage", index);
            return;
        }

        size_t remaining = state.recvLength - header.messageSize;
        std::memmove(state.recvBuffer.data(), state.recvBuffer.data() + header.messageSize, remaining);
        state.recvLength = remaining;
    }
}

} // namespace opcua
