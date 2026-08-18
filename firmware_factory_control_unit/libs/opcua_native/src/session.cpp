#include "opcua/session.hpp"

#include "opcua/codec.hpp"
#include "opcua/node_ids.hpp"
#include "opcua/service_header.hpp"

namespace opcua {

namespace {
// AnonymousIdentityToken's DefaultBinary encoding id (Part 4 7.36.4 / Opc.Ua.NodeIds.csv).
constexpr UInt32 ANONYMOUS_IDENTITY_TOKEN_TYPEID = 321;

bool SkipSignatureData(ByteReader &r) {
    String algorithm;
    std::string_view signature;
    bool signatureIsNull = true;
    if(!r.ReadString(algorithm)) return false;
    return r.ReadByteString(signature, signatureIsNull);
}

// ClientSoftwareCertificates (Part 4 7.32): array of {CertificateData: ByteString,
// SignatureData}. Real anonymous-auth clients send an empty array; skipped either way since
// this server has no software certificate policy.
bool SkipSoftwareCertificateArray(ByteReader &r) {
    Int32 count = 0;
    if(!r.ReadInt32(count)) return false;
    for(Int32 i = 0; i < count; i++) {
        std::string_view cert;
        bool certIsNull = true;
        if(!r.ReadByteString(cert, certIsNull)) return false;
        if(!SkipSignatureData(r)) return false;
    }
    return true;
}
} // namespace

bool Session::HandleCreate(ByteReader &requestBody, UInt32 connectionSlot,
                           std::string_view endpointUrl, ByteWriter &w) {
    RequestHeader reqHeader;
    if(!DecodeRequestHeader(requestBody, reqHeader)) return false;

    ApplicationDescription clientDescription;
    if(!DecodeApplicationDescription(requestBody, clientDescription)) return false;

    String serverUri, requestedEndpointUrl, sessionName;
    std::string_view clientNonce, clientCertificate;
    bool clientNonceIsNull = true, clientCertificateIsNull = true;
    Double requestedSessionTimeout = 0;
    UInt32 maxResponseMessageSize = 0;
    if(!requestBody.ReadString(serverUri)) return false;
    if(!requestBody.ReadString(requestedEndpointUrl)) return false;
    if(!requestBody.ReadString(sessionName)) return false;
    if(!requestBody.ReadByteString(clientNonce, clientNonceIsNull)) return false;
    if(!requestBody.ReadByteString(clientCertificate, clientCertificateIsNull)) return false;
    if(!requestBody.ReadDouble(requestedSessionTimeout)) return false;
    if(!requestBody.ReadUInt32(maxResponseMessageSize)) return false;
    if(!requestBody.Ok()) return false;
    (void)maxResponseMessageSize;

    // Derived from the fixed connection-table slot, not a counter -- unique for as long as
    // this connection lives, and namespace 1 keeps it clearly separate from this server's own
    // NS0 usage (see node_ids.hpp) and its address-space nodes.
    sessionId_ = NodeId(1, 10000 + connectionSlot);
    authenticationToken_ = NodeId(1, 20000 + connectionSlot);
    isCreated_ = true;
    isActive_ = false;

    Double revisedTimeout = (requestedSessionTimeout > 0.0) ? requestedSessionTimeout : 60000.0;

    if(!EncodeNodeId(w, NodeId(0, ns0::CreateSessionResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(reqHeader, StatusCode::Good))) return false;
    if(!EncodeNodeId(w, sessionId_)) return false;
    if(!EncodeNodeId(w, authenticationToken_)) return false;
    if(!w.WriteDouble(revisedTimeout)) return false;
    if(!w.WriteByteString(std::string_view{}, true)) return false; // ServerNonce = null (no crypto)
    if(!w.WriteByteString(std::string_view{}, true)) return false; // ServerCertificate = null
    if(!w.WriteInt32(1)) return false;                        // ServerEndpoints: 1 entry
    if(!EncodeEndpointDescription(w, BuildEndpoint(endpointUrl))) return false;
    if(!w.WriteInt32(-1)) return false;                        // ServerSoftwareCertificates: null
    if(!w.WriteString(String::Null())) return false;           // ServerSignature.Algorithm
    if(!w.WriteByteString(std::string_view{}, true)) return false;  // ServerSignature.Signature
    return w.WriteUInt32(8192);                                 // MaxRequestMessageSize
}

bool Session::HandleActivate(ByteReader &requestBody, ByteWriter &w) {
    RequestHeader reqHeader;
    if(!DecodeRequestHeader(requestBody, reqHeader)) return false;
    if(!SkipSignatureData(requestBody)) return false;          // ClientSignature
    if(!SkipSoftwareCertificateArray(requestBody)) return false; // ClientSoftwareCertificates
    if(!SkipStringArray(requestBody)) return false;            // LocaleIds

    // UserIdentityToken (ExtensionObject) -- only its TypeId matters here (only Anonymous is
    // accepted, per project decision); the body (just a PolicyId string for
    // AnonymousIdentityToken) is skipped unread since this server offers exactly one policy.
    NodeId tokenTypeId;
    if(!DecodeNodeId(requestBody, tokenTypeId)) return false;
    Byte tokenEncoding = 0;
    if(!requestBody.ReadByte(tokenEncoding)) return false;
    if(tokenEncoding & 0x01) {
        std::string_view body;
        bool bodyIsNull = true;
        if(!requestBody.ReadByteString(body, bodyIsNull)) return false;
    } else if(tokenEncoding & 0x02) {
        String xml;
        if(!requestBody.ReadString(xml)) return false;
    }
    if(!SkipSignatureData(requestBody)) return false; // UserTokenSignature
    if(!requestBody.Ok()) return false;

    bool isAnonymous = tokenTypeId.namespaceIndex == 0 && tokenTypeId.type == NodeIdType::Numeric &&
                       tokenTypeId.numeric == ANONYMOUS_IDENTITY_TOKEN_TYPEID;
    bool isNullToken = tokenTypeId.IsNull(); // some clients omit the token entirely
    StatusCode result;
    if(!isCreated_)
        result = StatusCode::BadSessionIdInvalid; // no CreateSession happened yet on this connection
    else
        result = (isAnonymous || isNullToken) ? StatusCode::Good : StatusCode::BadIdentityTokenRejected;

    if(IsGood(result))
        isActive_ = true;

    if(!EncodeNodeId(w, NodeId(0, ns0::ActivateSessionResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(reqHeader, result))) return false;
    if(!w.WriteByteString(std::string_view{}, true)) return false; // ServerNonce
    if(!w.WriteInt32(-1)) return false;                        // Results: null array
    return w.WriteInt32(-1);                                    // DiagnosticInfos: null array
}

bool Session::HandleClose(ByteReader &requestBody, ByteWriter &w) {
    RequestHeader reqHeader;
    if(!DecodeRequestHeader(requestBody, reqHeader)) return false;
    Boolean deleteSubscriptions = false;
    if(!requestBody.ReadBoolean(deleteSubscriptions)) return false;
    if(!requestBody.Ok()) return false;
    (void)deleteSubscriptions; // no subscriptions implemented yet (project decision: "later")

    StatusCode result = isCreated_ ? StatusCode::Good : StatusCode::BadSessionIdInvalid;
    isActive_ = false;
    isCreated_ = false;

    if(!EncodeNodeId(w, NodeId(0, ns0::CloseSessionResponse))) return false;
    return EncodeResponseHeader(w, MakeResponseHeader(reqHeader, result));
}

} // namespace opcua
