#include "opcua/secure_channel.hpp"

#include "opcua/clock.hpp"
#include "opcua/codec.hpp"
#include "opcua/node_ids.hpp"

namespace opcua {

namespace {
// This server never rolls over/renews a token mid-flight (see SecureChannel::HandleOpen's
// comment) -- 1 hour is a generous default for a RequestedLifetime of 0, well past any
// realistic idle timeout a client would notice.
constexpr UInt32 DEFAULT_CHANNEL_LIFETIME_MS = 3600000;
} // namespace

bool BeginOpenSecureChannelMessage(ByteWriter &w, UInt32 channelId, UInt32 sequenceNumber,
                                   UInt32 requestId, size_t &startPosOut) {
    startPosOut = w.Position();
    if(!EncodeTcpHeader(w, "OPN", ChunkType::Final, 0)) return false;
    if(!w.WriteUInt32(channelId)) return false;
    // AsymmetricAlgorithmSecurityHeader (SecurityPolicy#None: no certificates).
    if(!w.WriteString(SECURITY_POLICY_NONE_URI)) return false;
    if(!w.WriteByteString(std::string_view{}, true)) return false; // SenderCertificate = null
    if(!w.WriteByteString(std::string_view{}, true)) return false; // ReceiverCertificateThumbprint = null
    if(!w.WriteUInt32(sequenceNumber)) return false;
    return w.WriteUInt32(requestId);
}

bool BeginSymmetricMessage(ByteWriter &w, const char messageType[3], UInt32 channelId,
                           UInt32 tokenId, UInt32 sequenceNumber, UInt32 requestId,
                           size_t &startPosOut) {
    startPosOut = w.Position();
    if(!EncodeTcpHeader(w, messageType, ChunkType::Final, 0)) return false;
    if(!w.WriteUInt32(channelId)) return false;
    if(!w.WriteUInt32(tokenId)) return false; // SymmetricAlgorithmSecurityHeader
    if(!w.WriteUInt32(sequenceNumber)) return false;
    return w.WriteUInt32(requestId);
}

bool FinishSecureMessage(ByteWriter &w, size_t startPos) {
    if(!w.Ok()) return false;
    UInt32 totalSize = static_cast<UInt32>(w.Position() - startPos);
    return w.PatchUInt32(startPos + 4, totalSize);
}

bool ParseOpenSecureChannelMessage(std::span<const Byte> msg, ParsedSecureMessage &out) {
    ByteReader r(msg);
    if(!DecodeTcpHeader(r, out.header)) return false;
    if(!out.header.Is("OPN")) return false;
    if(out.header.chunkType != ChunkType::Final) return false;
    if(out.header.messageSize != msg.size()) return false;
    if(!r.ReadUInt32(out.channelId)) return false;

    String policyUri;
    std::string_view senderCert, receiverThumbprint;
    bool senderCertIsNull = true, receiverThumbprintIsNull = true;
    if(!r.ReadString(policyUri)) return false;
    if(!r.ReadByteString(senderCert, senderCertIsNull)) return false;
    if(!r.ReadByteString(receiverThumbprint, receiverThumbprintIsNull)) return false;
    if(policyUri.isNull || policyUri.value != SECURITY_POLICY_NONE_URI)
        return false; // any policy other than None -- reject (BadSecurityPolicyRejected upstream)

    if(!r.ReadUInt32(out.sequenceNumber)) return false;
    if(!r.ReadUInt32(out.requestId)) return false;
    if(!r.Ok()) return false;

    out.tokenId = 0;
    out.body = r.RemainingSpan();
    return true;
}

bool ParseSymmetricMessage(std::span<const Byte> msg, ParsedSecureMessage &out) {
    ByteReader r(msg);
    if(!DecodeTcpHeader(r, out.header)) return false;
    if(!(out.header.Is("MSG") || out.header.Is("CLO"))) return false;
    if(out.header.chunkType != ChunkType::Final) return false;
    if(out.header.messageSize != msg.size()) return false;
    if(!r.ReadUInt32(out.channelId)) return false;
    if(!r.ReadUInt32(out.tokenId)) return false;
    if(!r.ReadUInt32(out.sequenceNumber)) return false;
    if(!r.ReadUInt32(out.requestId)) return false;
    if(!r.Ok()) return false;

    out.body = r.RemainingSpan();
    return true;
}

bool SecureChannel::HandleOpen(ByteReader &requestBody, UInt32 assignedChannelId,
                               UInt32 /*requestSequenceNumber*/, UInt32 requestId, ByteWriter &w) {
    RequestHeader reqHeader;
    if(!DecodeRequestHeader(requestBody, reqHeader)) return false;

    UInt32 clientProtocolVersion = 0;
    Int32 requestType = 0;
    Int32 securityMode = 0;
    std::string_view clientNonce;
    bool clientNonceIsNull = true;
    UInt32 requestedLifetimeMs = 0;
    if(!requestBody.ReadUInt32(clientProtocolVersion)) return false;
    if(!requestBody.ReadInt32(requestType)) return false;
    if(!requestBody.ReadInt32(securityMode)) return false;
    if(!requestBody.ReadByteString(clientNonce, clientNonceIsNull)) return false;
    if(!requestBody.ReadUInt32(requestedLifetimeMs)) return false;
    if(!requestBody.Ok()) return false;
    (void)clientProtocolVersion;
    (void)requestType; // Issue vs. Renew handled identically, see header comment

    constexpr Int32 SECURITY_MODE_NONE = 1;
    StatusCode result = (securityMode == SECURITY_MODE_NONE)
        ? StatusCode::Good : StatusCode::BadSecurityModeRejected;

    if(IsGood(result)) {
        isOpen_ = true;
        channelId_ = assignedChannelId;
        tokenId_ += 1;
        createdAt_ = Now();
        revisedLifetimeMs_ = (requestedLifetimeMs != 0) ? requestedLifetimeMs : DEFAULT_CHANNEL_LIFETIME_MS;
    }

    size_t startPos = 0;
    if(!BeginOpenSecureChannelMessage(w, IsGood(result) ? channelId_ : assignedChannelId,
                                      NextServerSequenceNumber(), requestId, startPos))
        return false;
    if(!EncodeNodeId(w, NodeId(0, ns0::OpenSecureChannelResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(reqHeader, result))) return false;
    if(!w.WriteUInt32(0)) return false; // ServerProtocolVersion
    if(!w.WriteUInt32(channelId_)) return false;
    if(!w.WriteUInt32(tokenId_)) return false;
    if(!w.WriteDateTime(createdAt_)) return false;
    if(!w.WriteUInt32(revisedLifetimeMs_)) return false;
    if(!w.WriteByteString(std::string_view{}, true)) return false; // ServerNonce = null (no crypto)
    return FinishSecureMessage(w, startPos);
}

bool SecureChannel::HandleClose(ByteReader &requestBody) {
    RequestHeader reqHeader;
    if(!DecodeRequestHeader(requestBody, reqHeader)) return false;
    isOpen_ = false;
    return true;
}

} // namespace opcua
