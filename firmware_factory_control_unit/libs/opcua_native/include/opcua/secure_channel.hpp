#pragma once
// SecureChannel layer (Part 6 7.1.2/7.2, Part 4 5.5): OPN/MSG/CLO message framing plus the
// OpenSecureChannel/CloseSecureChannel service handlers.
//
// SecurityPolicy#Basic256Sha256, SecurityMode Sign only (project decision, 2026-08-19 -- see
// libs/opcua_native/README or project memory for the discussion): every OPN message is BOTH
// signed (RSASSA-PKCS1-v1_5-SHA256) and encrypted (RSAES-OAEP-SHA1) with asymmetric keys,
// regardless of the negotiated MessageSecurityMode -- Part 6 6.7.4 mandates this for the channel
// handshake itself, independent of what mode later MSG traffic uses. MSG/CLO messages are only
// HMAC-SHA256 signed (never AES-encrypted) -- SecurityMode Sign, not SignAndEncrypt.
//
// Client certificates are accepted for signature verification WITHOUT trust-list/CA-chain
// validation (transport integrity, not client authentication -- matches this server's unchanged
// anonymous Session model and its general minimal-server scope).
//
// One narrow exception, added after live UAExpert testing (2026-08-19): a channel may also be
// opened with SecurityPolicy#None, but ONLY GetEndpoints is servable on it afterward (Part 4
// 5.4.4 -- GetEndpoints is deliberately the one service usable without security, so a client can
// discover which SecurityPolicy to actually use before committing to one; real clients,
// UAExpert included, try exactly this as their default connection flow). Such a "discovery-only"
// channel carries no crypto at all (no cert, no signature, no encryption, matching how this
// server behaved before Basic256Sha256 existed) -- see SecureChannel::IsDiscoveryOnly().
#include <span>

#include "opcua/byte_stream.hpp"
#include "opcua/security_crypto.hpp"
#include "opcua/service_header.hpp"
#include "opcua/tcp_message.hpp"
#include "opcua/types.hpp"

namespace opcua {

constexpr char SECURITY_POLICY_BASIC256SHA256_URI[] = "http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256";
constexpr char SECURITY_POLICY_NONE_URI[] = "http://opcfoundation.org/UA/SecurityPolicy#None";
constexpr Int32 SECURITY_MODE_NONE = 1; // MessageSecurityMode, Part 4 7.15
constexpr Int32 SECURITY_MODE_SIGN = 2;

// This server's own certificate + private key -- set ONCE at startup (opcua_setup.cpp) and
// shared by every Connection/SecureChannel (one identity for the whole process; the SAME
// certificate the HTTPS webserver uses, see net_setup.cpp). Neither the spans nor the key data
// are owned here -- they point at the embedded device-certificate binary blob, which outlives
// the program.
struct ServerIdentity {
    std::span<const Byte> certificateDer; // leaf only -- used to extract this server's RSA key
    // Root CA cert (DER), appended after the leaf whenever the certificate is sent over the wire
    // (SenderCertificate/serverCertificate), so a client (e.g. UAExpert) can build a complete
    // trust chain without a separate out-of-band CA fetch -- found necessary via live UAExpert
    // testing 2026-08-19 (BadCertificateChainIncomplete when only the leaf was sent).
    std::span<const Byte> caCertificateDer;
    security::RsaPrivateKey privateKey;
};
void SetServerIdentity(const ServerIdentity &identity);
const ServerIdentity &GetServerIdentity();

// Writes the server's certificate CHAIN (leaf + root CA) as a single ByteString, matching how
// serverCertificate/SenderCertificate is encoded on the wire (Part 6 7.10) -- built from two
// separately-embedded DER blobs without needing to physically concatenate them in memory first.
bool WriteServerCertificateChain(ByteWriter &w);

// A fully verified/decrypted MSG or CLO service body, ready for TypeId dispatch.
struct ParsedSecureMessage {
    TcpHeader header;
    UInt32 channelId = 0;
    UInt32 tokenId = 0;
    UInt32 sequenceNumber = 0;
    UInt32 requestId = 0;
    std::span<const Byte> body; // plaintext: TypeId (NodeId) + the service struct's own fields
};

// Writes a SymmetricAlgorithmSecurityHeader'd MSG/CLO frame header, leaving "w" positioned for
// the caller to encode the service body -- pair with SecureChannel::SignAndFinishSymmetric().
bool BeginSymmetricMessage(ByteWriter &w, const char messageType[3], UInt32 channelId,
                           UInt32 tokenId, UInt32 sequenceNumber, UInt32 requestId,
                           size_t &startPosOut);

// Per-connection SecureChannel state (Part 4 5.5.2/5.5.3). One instance per Connection -- this
// server ties exactly one SecureChannel to one TCP connection (no channel migration/renewal
// across connections, see connection.hpp).
class SecureChannel {
public:
    bool IsOpen() const { return isOpen_; }
    // True if this channel was opened with SecurityPolicy#None -- only GetEndpoints may be
    // served on it (Connection::HandleServiceMessage enforces this), see the file header comment.
    bool IsDiscoveryOnly() const { return isDiscoveryOnly_; }
    UInt32 ChannelId() const { return channelId_; }
    UInt32 TokenId() const { return tokenId_; }
    UInt32 NextServerSequenceNumber() { return serverSequenceNumber_++; }

    // "fullOpnMessage" is exactly one complete OPN message (header.messageSize == size()).
    // Accepts SecurityPolicy#Basic256Sha256 (parses+decrypts+verifies the request using the
    // client certificate embedded in the message itself, plus this server's own identity,
    // derives fresh HMAC-SHA256 signing keys from the ClientNonce/ServerNonce exchange per Part 6
    // 6.7.5, and writes a fully signed+encrypted OPN response) or SecurityPolicy#None (no crypto
    // at all, IsDiscoveryOnly() becomes true) into "w". Assigns a fresh ChannelId+TokenId on
    // first call (Issue), or revises the token on a later call for the same channel (Renew) --
    // Renew re-derives keys from a fresh nonce exchange but otherwise just re-issues the same
    // ChannelId/TokenId, since this server has no multi-token overlap/rollover logic yet (an
    // accepted simplification, see the project's OPC UA design notes).
    bool HandleOpen(std::span<const Byte> fullOpnMessage, UInt32 assignedChannelId, ByteWriter &w);

    // "fullCloMessage" is exactly one complete CLO message. There is no response either way
    // (Part 4 5.5.3); returns false only if the message didn't even verify/decode (caller closes
    // the connection regardless).
    bool HandleClose(std::span<const Byte> fullCloMessage);

    // Verifies the HMAC-SHA256 signature and strips padding from a complete MSG/CLO message,
    // exposing the plaintext service body via "out". Requires IsOpen().
    bool VerifyAndUnwrapSymmetric(std::span<const Byte> fullMessage, ParsedSecureMessage &out);

    // Pairs with BeginSymmetricMessage(): appends PaddingSize+Padding+HMAC-SHA256 signature to
    // the response service body already written into "w" from "startPos", then backfills the
    // message-size field. Replaces the old policy-agnostic FinishSecureMessage for MSG/CLO.
    bool SignAndFinishSymmetric(ByteWriter &w, size_t startPos);

private:
    bool isOpen_ = false;
    bool isDiscoveryOnly_ = false;
    UInt32 channelId_ = 0;
    UInt32 tokenId_ = 0;
    DateTime createdAt_ = 0;
    UInt32 revisedLifetimeMs_ = 0;
    UInt32 serverSequenceNumber_ = 1;

    // HMAC-SHA256 signing keys derived from the OPN nonce exchange (Part 6 6.7.5) -- Sign mode
    // only needs these (no AES encrypting key/IV derived; see the file header comment).
    Byte clientSigningKey_[security::SHA256_DIGEST_SIZE] = {};
    Byte serverSigningKey_[security::SHA256_DIGEST_SIZE] = {};
};

} // namespace opcua
