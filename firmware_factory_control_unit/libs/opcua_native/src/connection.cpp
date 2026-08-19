#include "opcua/connection.hpp"

#include "opcua/clock.hpp"
#include "opcua/codec.hpp"
#include "opcua/node_ids.hpp"
#include "opcua/services.hpp"
#include "opcua/tcp_message.hpp"
#include "log.h"

namespace opcua {

void Connection::Reset(UInt32 connectionSlot, ITransport *transport, const AddressSpace *addressSpace,
                       std::string_view endpointUrl) {
    connectionSlot_ = connectionSlot;
    transport_ = transport;
    addressSpace_ = addressSpace;
    endpointUrl_ = endpointUrl;
    helloReceived_ = false;
    secureChannel_ = SecureChannel{};
    session_ = Session{};
}

void Connection::SendErrorAndClose(StatusCode error, std::string_view reason) {
    log_debug("OPC UA: slot %u closing: %.*s (0x%08X)", connectionSlot_,
             static_cast<int>(reason.size()), reason.data(), static_cast<unsigned>(error));
    ByteWriter w(sendBuffer_);
    if(EncodeErrorMessage(w, error, reason) && w.Ok())
        transport_->Send(w.Written());
    transport_->RequestClose();
}

void Connection::HandleMessage(std::span<const Byte> msg) {
    TcpHeader header;
    if(!PeekTcpHeader(msg, header)) {
        SendErrorAndClose(StatusCode::BadDecodingError, "malformed message header");
        return;
    }

    if(!helloReceived_) {
        if(header.Is("HEL")) { HandleHello(msg); return; }
        SendErrorAndClose(StatusCode::BadTcpMessageTypeInvalid, "expected Hello");
        return;
    }
    if(header.Is("HEL")) {
        SendErrorAndClose(StatusCode::BadTcpMessageTypeInvalid, "duplicate Hello");
        return;
    }
    if(header.Is("OPN")) { HandleOpenSecureChannel(msg); return; }
    if(header.Is("CLO")) { HandleCloseSecureChannel(msg); return; }
    if(header.Is("MSG")) {
        if(!secureChannel_.IsOpen()) {
            SendErrorAndClose(StatusCode::BadSecureChannelClosed, "no open SecureChannel");
            return;
        }
        HandleServiceMessage(msg);
        return;
    }
    SendErrorAndClose(StatusCode::BadTcpMessageTypeInvalid, "unknown message type");
}

void Connection::HandleHello(std::span<const Byte> msg) {
    ByteReader r(msg);
    TcpHeader header;
    if(!DecodeTcpHeader(r, header)) {
        SendErrorAndClose(StatusCode::BadDecodingError, "malformed Hello header");
        return;
    }
    HelloMessage hello;
    if(!DecodeHelloBody(r, hello) || !r.Ok()) {
        SendErrorAndClose(StatusCode::BadDecodingError, "malformed Hello body");
        return;
    }

    helloReceived_ = true;

    AcknowledgeMessage ack;
    ack.protocolVersion = PROTOCOL_VERSION;
    ack.receiveBufferSize = static_cast<UInt32>(SEND_BUFFER_SIZE);
    ack.sendBufferSize = static_cast<UInt32>(SEND_BUFFER_SIZE);
    ack.maxMessageSize = static_cast<UInt32>(SEND_BUFFER_SIZE);
    // Single-chunk-only transport (see the project's scope notes / secure_channel.hpp) -- a
    // compliant client is required to honor this and never send a multi-chunk request.
    ack.maxChunkCount = 1;

    ByteWriter w(sendBuffer_);
    if(!EncodeAcknowledge(w, ack) || !w.Ok() || !transport_->Send(w.Written()))
        transport_->RequestClose();
}

void Connection::HandleOpenSecureChannel(std::span<const Byte> msg) {
    ParsedSecureMessage parsed;
    if(!ParseOpenSecureChannelMessage(msg, parsed)) {
        SendErrorAndClose(StatusCode::BadSecurityPolicyRejected, "OpenSecureChannel parse failed");
        return;
    }

    ByteReader body(parsed.body);
    NodeId typeId;
    if(!DecodeNodeId(body, typeId) || typeId != NodeId(0, ns0::OpenSecureChannelRequest)) {
        SendErrorAndClose(StatusCode::BadTcpMessageTypeInvalid, "expected OpenSecureChannelRequest");
        return;
    }

    // Assigns a fresh ChannelId on first Open, keeps the existing one across a Renew --
    // (connectionSlot_ + 1) is guaranteed unique among this server's small, fixed connection
    // table (see the future net_transport.hpp wiring) without needing a separate allocator.
    UInt32 channelId = secureChannel_.IsOpen() ? secureChannel_.ChannelId() : (connectionSlot_ + 1);

    ByteWriter w(sendBuffer_);
    if(!secureChannel_.HandleOpen(body, channelId, parsed.sequenceNumber, parsed.requestId, w) || !w.Ok()) {
        SendErrorAndClose(StatusCode::BadInternalError, "OpenSecureChannel handling failed");
        return;
    }
    if(!transport_->Send(w.Written()))
        transport_->RequestClose();
}

void Connection::HandleCloseSecureChannel(std::span<const Byte> msg) {
    ParsedSecureMessage parsed;
    bool ok = ParseSymmetricMessage(msg, parsed) && secureChannel_.IsOpen() &&
             parsed.channelId == secureChannel_.ChannelId() && parsed.tokenId == secureChannel_.TokenId();
    if(ok) {
        ByteReader body(parsed.body);
        NodeId typeId;
        // Best-effort: a CloseSecureChannelRequest with a malformed body still results in the
        // connection closing (Part 4 5.5.3: there is no response either way).
        if(DecodeNodeId(body, typeId))
            secureChannel_.HandleClose(body);
    }
    transport_->RequestClose();
}

void Connection::HandleServiceMessage(std::span<const Byte> msg) {
    ParsedSecureMessage parsed;
    if(!ParseSymmetricMessage(msg, parsed)) {
        SendErrorAndClose(StatusCode::BadDecodingError, "malformed service message");
        return;
    }
    if(parsed.channelId != secureChannel_.ChannelId() || parsed.tokenId != secureChannel_.TokenId()) {
        log_debug("OPC UA: slot %u channel/token mismatch: got channel=%u token=%u, expected channel=%u token=%u",
                 connectionSlot_, static_cast<unsigned>(parsed.channelId), static_cast<unsigned>(parsed.tokenId),
                 static_cast<unsigned>(secureChannel_.ChannelId()), static_cast<unsigned>(secureChannel_.TokenId()));
        SendErrorAndClose(StatusCode::BadSecureChannelIdInvalid, "channel/token mismatch");
        return;
    }

    ByteReader body(parsed.body);
    NodeId typeId;
    if(!DecodeNodeId(body, typeId)) {
        SendErrorAndClose(StatusCode::BadDecodingError, "malformed service TypeId");
        return;
    }
    log_debug("OPC UA: slot %u service typeId=(ns=%u,i=%u)", connectionSlot_,
             static_cast<unsigned>(typeId.namespaceIndex), static_cast<unsigned>(typeId.numeric));

    ByteWriter w(sendBuffer_);
    size_t startPos = 0;
    if(!BeginSymmetricMessage(w, "MSG", secureChannel_.ChannelId(), secureChannel_.TokenId(),
                              secureChannel_.NextServerSequenceNumber(), parsed.requestId, startPos)) {
        transport_->RequestClose();
        return;
    }

    bool handled;
    if(typeId == NodeId(0, ns0::CreateSessionRequest)) {
        handled = session_.HandleCreate(body, connectionSlot_, endpointUrl_, w);
    } else if(typeId == NodeId(0, ns0::ActivateSessionRequest)) {
        handled = session_.HandleActivate(body, w);
    } else if(typeId == NodeId(0, ns0::CloseSessionRequest)) {
        handled = session_.HandleClose(body, w);
    } else if(typeId == NodeId(0, ns0::GetEndpointsRequest)) {
        handled = HandleGetEndpoints(body, endpointUrl_, w);
    } else if(typeId == NodeId(0, ns0::ReadRequest)) {
        handled = HandleRead(*addressSpace_, session_.IsActive(), body, w);
    } else if(typeId == NodeId(0, ns0::WriteRequest)) {
        handled = HandleWrite(*addressSpace_, session_.IsActive(), body, w);
    } else if(typeId == NodeId(0, ns0::BrowseRequest)) {
        handled = HandleBrowse(*addressSpace_, session_.IsActive(), body, w);
    } else {
        // Unimplemented service (e.g. CreateSubscription -- "later", see project notes) --
        // ServiceFault with requestHandle echoed as 0 since we deliberately don't decode a
        // RequestHeader for a service we don't even recognize the body shape of.
        ResponseHeader fault;
        fault.timestamp = Now();
        fault.requestHandle = 0;
        fault.serviceResult = StatusCode::BadServiceUnsupported;
        handled = EncodeNodeId(w, NodeId(0, ns0::ServiceFault)) && EncodeResponseHeader(w, fault);
    }

    if(!handled || !w.Ok() || !FinishSecureMessage(w, startPos)) {
        log_debug("OPC UA: slot %u handler failed for typeId=(ns=%u,i=%u) handled=%d w.Ok=%d",
                 connectionSlot_, static_cast<unsigned>(typeId.namespaceIndex),
                 static_cast<unsigned>(typeId.numeric), handled ? 1 : 0, w.Ok() ? 1 : 0);
        transport_->RequestClose();
        return;
    }
    if(!transport_->Send(w.Written())) {
        log_debug("OPC UA: slot %u Send() failed for typeId=(ns=%u,i=%u)", connectionSlot_,
                 static_cast<unsigned>(typeId.namespaceIndex), static_cast<unsigned>(typeId.numeric));
        transport_->RequestClose();
    }
}

} // namespace opcua
