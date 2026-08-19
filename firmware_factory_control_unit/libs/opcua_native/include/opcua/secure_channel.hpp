#pragma once
// SecureChannel layer (Part 6 7.1.2/7.2, Part 4 5.5): the OPN/MSG/CLO message framing
// (AsymmetricAlgorithmSecurityHeader for OPN, SymmetricAlgorithmSecurityHeader for MSG/CLO,
// SequenceHeader for both) plus the OpenSecureChannel/CloseSecureChannel service handlers.
//
// SecurityPolicy#None only (project decision): no signing, no encryption, no certificates --
// SenderCertificate/ReceiverCertificateThumbprint are always null ByteStrings, and there is no
// signature or padding appended to any message. This is what makes single-chunk-only transport
// (see connection.hpp) plus this comparatively small framing layer sufficient; adding a real
// SecurityPolicy later means: (1) a pluggable "SecurityPolicy" interface slot here for
// sign/verify/encrypt/decrypt, (2) padding + signature fields in the frame, (3) certificate
// handling in the OPN handshake -- the message-framing SHAPE below doesn't change.
#include <span>

#include "opcua/byte_stream.hpp"
#include "opcua/service_header.hpp"
#include "opcua/tcp_message.hpp"
#include "opcua/types.hpp"

namespace opcua {

constexpr char SECURITY_POLICY_NONE_URI[] = "http://opcfoundation.org/UA/SecurityPolicy#None";

// A fully parsed OPN/MSG/CLO frame, minus the still-encoded service body (TypeId + struct
// fields), which the caller (connection.cpp) decodes based on the TypeId itself.
struct ParsedSecureMessage {
    TcpHeader header;
    UInt32 channelId = 0;
    UInt32 tokenId = 0;         // only meaningful for MSG/CLO (SymmetricAlgorithmSecurityHeader)
    UInt32 sequenceNumber = 0;
    UInt32 requestId = 0;
    std::span<const Byte> body; // TypeId (NodeId) + the service struct's own encoded fields
};

// "msg" must be exactly one complete OPN message (header.messageSize == msg.size()).
// Validates ChunkType==Final and SecurityPolicyUri=="...#None" (anything else -> false, which
// the caller turns into a BadSecurityPolicyRejected ERR message and connection close).
bool ParseOpenSecureChannelMessage(std::span<const Byte> msg, ParsedSecureMessage &out);

// "msg" must be exactly one complete MSG or CLO message.
bool ParseSymmetricMessage(std::span<const Byte> msg, ParsedSecureMessage &out);

// Writes a complete OPN response frame (header + AsymmetricAlgorithmSecurityHeader(None) +
// SequenceHeader + whatever the caller already appended to w as the body) -- call
// BeginOpenSecureChannelMessage(), then encode TypeId+response struct directly into w, then
// FinishSecureMessage() to backfill MessageSize.
bool BeginOpenSecureChannelMessage(ByteWriter &w, UInt32 channelId, UInt32 sequenceNumber,
                                   UInt32 requestId, size_t &startPosOut);
// Writes a complete MSG or CLO response frame header; same Begin/encode-body/Finish pattern.
bool BeginSymmetricMessage(ByteWriter &w, const char messageType[3], UInt32 channelId,
                           UInt32 tokenId, UInt32 sequenceNumber, UInt32 requestId,
                           size_t &startPosOut);
bool FinishSecureMessage(ByteWriter &w, size_t startPos);

// Per-connection SecureChannel state (Part 4 5.5.2/5.5.3). One instance per Connection -- this
// server ties exactly one SecureChannel to one TCP connection (no channel migration/renewal
// across connections, see connection.hpp).
class SecureChannel {
public:
    bool IsOpen() const { return isOpen_; }
    UInt32 ChannelId() const { return channelId_; }
    UInt32 TokenId() const { return tokenId_; }

    // Allocates the next SequenceNumber this server will use for an outgoing MSG/CLO/OPN on
    // this channel (Part 6 6.7.2.3: each side maintains its own independently-incrementing
    // counter starting at an arbitrary value; 1 is as good as any for a minimal server).
    UInt32 NextServerSequenceNumber() { return serverSequenceNumber_++; }

    // Handles an OpenSecureChannelRequest (already TypeId-dispatched by the caller, "body" is
    // positioned right after the TypeId NodeId). Assigns a fresh ChannelId+TokenId on first
    // call (RequestType Issue), or revises the token on a later call for the same channel
    // (RequestType Renew) -- Renew is accepted but currently just re-issues the same
    // ChannelId/TokenId with a refreshed CreatedAt/Lifetime, since this server has no
    // multi-token overlap/rollover logic yet (fine under SecurityPolicy#None: there's nothing
    // a stale token would let an attacker forge anyway).
    // Writes the full OPN response frame into "w". requestSequenceNumber/requestId come from
    // the just-parsed ParsedSecureMessage.
    bool HandleOpen(ByteReader &requestBody, UInt32 assignedChannelId,
                    UInt32 requestSequenceNumber, UInt32 requestId, ByteWriter &w);

    // Handles a CloseSecureChannelRequest -- there is no response message per Part 4 5.5.3 (the
    // server just closes the underlying connection); returns false if the request didn't even
    // decode, true otherwise (the caller closes the connection regardless).
    bool HandleClose(ByteReader &requestBody);

private:
    bool isOpen_ = false;
    UInt32 channelId_ = 0;
    UInt32 tokenId_ = 0;
    DateTime createdAt_ = 0;
    UInt32 revisedLifetimeMs_ = 0;
    UInt32 serverSequenceNumber_ = 1;
};

} // namespace opcua
