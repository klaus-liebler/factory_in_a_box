#!/usr/bin/env python3
"""Hand-rolled OPC UA Binary test client for the Control Unit's from-scratch OPC UA server
(libs/opcua_native/, feature/opcua-native-server branch) -- SecurityPolicy#None only, single
TCP chunk per message, anonymous auth. Deliberately NOT built on a third-party OPC UA library
(e.g. python-opcua/asyncua): those implement the FULL spec far more strictly than this minimal
server does, so a real client library would likely reject/mis-diagnose things that are fine
for this server's intentionally-reduced scope. No pip install needed -- stdlib only.

Exercises the whole handshake end to end: Hello/Acknowledge -> OpenSecureChannel ->
CreateSession -> ActivateSession (anonymous) -> Browse (Objects -> FactoryControlUnit ->
Greeting/UptimeSeconds) -> Read both values -> attempt Write on the read-only Greeting node
(expects BadNotWritable, confirming access-level enforcement actually works) -> CloseSession ->
CloseSecureChannel.

Examples:
    python test_opcua.py --host factory-box-363836.local
    python test_opcua.py --host 192.168.1.50
"""
import argparse
import socket
import struct
import sys

OPCUA_PORT = 4840
SECURITY_POLICY_NONE_URI = b"http://opcfoundation.org/UA/SecurityPolicy#None"

# --- Well-known NodeIds/TypeIds this server actually uses (see libs/opcua_native/include/
# opcua/node_ids.hpp -- kept in sync by hand, this is a test tool, not generated code). ---
NS0_OBJECTS_FOLDER = 85
NS0_ORGANIZES = 35
NS0_HAS_COMPONENT = 47

NS0_OPEN_SECURE_CHANNEL_REQUEST = 446
NS0_OPEN_SECURE_CHANNEL_RESPONSE = 449
NS0_CLOSE_SECURE_CHANNEL_REQUEST = 452
NS0_CREATE_SESSION_REQUEST = 461
NS0_CREATE_SESSION_RESPONSE = 464
NS0_ACTIVATE_SESSION_REQUEST = 467
NS0_ACTIVATE_SESSION_RESPONSE = 470
NS0_CLOSE_SESSION_REQUEST = 473
NS0_CLOSE_SESSION_RESPONSE = 476
NS0_READ_REQUEST = 631
NS0_READ_RESPONSE = 634
NS0_WRITE_REQUEST = 673
NS0_WRITE_RESPONSE = 676
NS0_BROWSE_REQUEST = 527
NS0_BROWSE_RESPONSE = 530
NS0_SERVICE_FAULT = 397

ATTR_VALUE = 13
ATTR_BROWSE_NAME = 3

BUILTIN_TYPE_NAMES = {
    1: "Boolean", 2: "SByte", 3: "Byte", 4: "Int16", 5: "UInt16", 6: "Int32", 7: "UInt32",
    8: "Int64", 9: "UInt64", 10: "Float", 11: "Double", 12: "String", 13: "DateTime",
    14: "Guid", 15: "ByteString", 16: "XmlElement", 17: "NodeId", 18: "ExpandedNodeId",
    19: "StatusCode", 20: "QualifiedName", 21: "LocalizedText", 22: "ExtensionObject",
    23: "DataValue", 24: "Variant", 25: "DiagnosticInfo",
}


class DecodeError(Exception):
    pass


class Writer:
    """Accumulates one message BODY (everything after the transport frame header, which
    OpcUaClient.send_* prepends once the body's total length is known)."""

    def __init__(self):
        self.buf = bytearray()

    def byte(self, v):
        self.buf += struct.pack("<B", v)

    def boolean(self, v):
        self.byte(1 if v else 0)

    def int16(self, v):
        self.buf += struct.pack("<h", v)

    def uint16(self, v):
        self.buf += struct.pack("<H", v)

    def int32(self, v):
        self.buf += struct.pack("<i", v)

    def uint32(self, v):
        self.buf += struct.pack("<I", v)

    def double(self, v):
        self.buf += struct.pack("<d", v)

    def raw(self, b: bytes):
        self.buf += b

    def string(self, s):
        """None -> null string (length -1); str/bytes -> length-prefixed UTF-8."""
        if s is None:
            self.int32(-1)
            return
        b = s.encode("utf-8") if isinstance(s, str) else bytes(s)
        self.int32(len(b))
        self.raw(b)

    def byte_string(self, b):
        self.string(b)  # identical wire format (Part 6 5.2.1.2/5.2.1.3)

    def node_id_numeric(self, ns, identifier):
        """Always emits the compact two-/four-byte form when possible, like the server does."""
        if ns == 0 and identifier <= 0xFF:
            self.byte(0x00)
            self.byte(identifier)
        elif ns <= 0xFF and identifier <= 0xFFFF:
            self.byte(0x01)
            self.byte(ns)
            self.uint16(identifier)
        else:
            self.byte(0x02)
            self.uint16(ns)
            self.uint32(identifier)

    def qualified_name(self, ns, name):
        self.uint16(ns)
        self.string(name)

    def request_header(self, auth_token_ns=0, auth_token_id=0, request_handle=1, timeout_hint_ms=5000):
        self.node_id_numeric(auth_token_ns, auth_token_id)
        self.raw(struct.pack("<q", 0))  # timestamp (UtcTime) -- server doesn't check this
        self.uint32(request_handle)
        self.uint32(0)  # returnDiagnostics
        self.string(None)  # auditEntryId
        self.uint32(timeout_hint_ms)
        self.node_id_numeric(0, 0)  # additionalHeader: null ExtensionObject
        self.byte(0x00)  # ExtensionObject encoding: no body


class Reader:
    def __init__(self, data: bytes):
        self.data = data
        self.pos = 0

    def _take(self, n):
        if self.pos + n > len(self.data):
            raise DecodeError(f"truncated: need {n} bytes at {self.pos}, have {len(self.data) - self.pos}")
        b = self.data[self.pos:self.pos + n]
        self.pos += n
        return b

    def byte(self):
        return struct.unpack("<B", self._take(1))[0]

    def boolean(self):
        return self.byte() != 0

    def int16(self):
        return struct.unpack("<h", self._take(2))[0]

    def uint16(self):
        return struct.unpack("<H", self._take(2))[0]

    def int32(self):
        return struct.unpack("<i", self._take(4))[0]

    def uint32(self):
        return struct.unpack("<I", self._take(4))[0]

    def int64(self):
        return struct.unpack("<q", self._take(8))[0]

    def uint64(self):
        return struct.unpack("<Q", self._take(8))[0]

    def float32(self):
        return struct.unpack("<f", self._take(4))[0]

    def double(self):
        return struct.unpack("<d", self._take(8))[0]

    def string(self):
        length = self.int32()
        if length < 0:
            return None
        return self._take(length).decode("utf-8", errors="replace")

    def byte_string(self):
        length = self.int32()
        if length < 0:
            return None
        return bytes(self._take(length))

    def node_id(self):
        encoding = self.byte()
        if encoding == 0x00:
            return (0, self.byte())
        if encoding == 0x01:
            ns = self.byte()
            return (ns, self.uint16())
        if encoding == 0x02:
            ns = self.uint16()
            return (ns, self.uint32())
        if encoding == 0x03:
            ns = self.uint16()
            return (ns, self.string())
        if encoding == 0x04:
            ns = self.uint16()
            return (ns, self._take(16))
        if encoding == 0x05:
            ns = self.uint16()
            return (ns, self.byte_string())
        raise DecodeError(f"unknown NodeId encoding byte 0x{encoding:02x}")

    def expanded_node_id(self):
        # This server never sets the NamespaceUri/ServerIndex flag bits, but decode generically
        # in case that ever changes.
        save = self.pos
        first = self.data[self.pos]
        base = self.node_id()
        if first & 0x80:
            self.string()
        if first & 0x40:
            self.uint32()
        _ = save
        return base

    def qualified_name(self):
        ns = self.uint16()
        name = self.string()
        return (ns, name)

    def localized_text(self):
        encoding = self.byte()
        locale = self.string() if (encoding & 0x01) else None
        text = self.string() if (encoding & 0x02) else None
        return (locale, text)

    def status_code(self):
        return self.uint32()

    def response_header(self):
        timestamp = self.int64()
        request_handle = self.uint32()
        service_result = self.status_code()
        self.byte()  # serviceDiagnostics: DiagnosticInfo encoding byte (assumed 0x00 -- this
        # server always encodes "no fields present"; a real generic client would need to fully
        # decode this per its own encoding byte)
        n = self.int32()  # stringTable
        for _ in range(max(n, 0)):
            self.string()
        # additionalHeader: ExtensionObject
        self.node_id()
        ext_encoding = self.byte()
        if ext_encoding & 0x01:
            self.byte_string()
        elif ext_encoding & 0x02:
            self.string()
        return {"timestamp": timestamp, "request_handle": request_handle, "service_result": service_result}

    def variant(self):
        encoding = self.byte()
        if encoding == 0:
            return None
        if encoding & 0xC0:
            raise DecodeError("array-valued Variant not supported by this test client (or the server)")
        type_id = encoding & 0x3F
        if type_id == 1:
            return self.boolean()
        if type_id == 2:
            return struct.unpack("<b", self._take(1))[0]
        if type_id == 3:
            return self.byte()
        if type_id == 4:
            return self.int16()
        if type_id == 5:
            return self.uint16()
        if type_id == 6:
            return self.int32()
        if type_id == 7:
            return self.uint32()
        if type_id == 8:
            return self.int64()
        if type_id == 9:
            return self.uint64()
        if type_id == 10:
            return self.float32()
        if type_id == 11:
            return self.double()
        if type_id == 12:
            return self.string()
        if type_id == 17:
            return self.node_id()
        if type_id == 19:
            return self.status_code()
        if type_id == 20:
            return self.qualified_name()
        if type_id == 21:
            return self.localized_text()
        raise DecodeError(f"Variant type {type_id} ({BUILTIN_TYPE_NAMES.get(type_id, '?')}) not "
                          f"decoded by this test client")

    def data_value(self):
        mask = self.byte()
        value = self.variant() if (mask & 0x01) else None
        status = self.status_code() if (mask & 0x02) else 0
        if mask & 0x04:
            self.int64()  # SourceTimestamp
        if mask & 0x08:
            self.int64()  # ServerTimestamp
        if mask & 0x10:
            self.uint16()  # SourcePicoseconds
        if mask & 0x20:
            self.uint16()  # ServerPicoseconds
        return {"value": value, "status": status}


def status_name(code: int) -> str:
    return "Good" if (code & 0x80000000) == 0 else f"Bad/Uncertain(0x{code:08X})"


class OpcUaTestClient:
    def __init__(self, host, port=OPCUA_PORT, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.channel_id = 0
        self.token_id = 0
        self.seq = 1
        self.auth_token = (0, 0)

    def close(self):
        self.sock.close()

    def _next_seq(self):
        n = self.seq
        self.seq += 1
        return n

    def _send_frame(self, msg_type: bytes, chunk_type: bytes, header_extra: bytes, body: bytes):
        size = 8 + len(header_extra) + len(body)
        frame = msg_type + chunk_type + struct.pack("<I", size) + header_extra + body
        self.sock.sendall(frame)

    def _recv_exact(self, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise DecodeError("connection closed while reading")
            buf += chunk
        return bytes(buf)

    def _recv_message(self):
        header = self._recv_exact(8)
        msg_type = header[0:3]
        chunk_type = header[3:4]
        size = struct.unpack("<I", header[4:8])[0]
        rest = self._recv_exact(size - 8)
        if chunk_type != b"F":
            raise DecodeError(f"unexpected ChunkType {chunk_type!r} (this server only sends final chunks)")
        return msg_type, rest

    # --- Hello / Acknowledge ---
    def hello(self, endpoint_url: str):
        w = Writer()
        w.uint32(0)      # protocolVersion
        w.uint32(65536)  # receiveBufferSize
        w.uint32(65536)  # sendBufferSize
        w.uint32(65536)  # maxMessageSize
        w.uint32(0)      # maxChunkCount
        w.string(endpoint_url)
        self._send_frame(b"HEL", b"F", b"", bytes(w.buf))

        msg_type, body = self._recv_message()
        if msg_type != b"ACK":
            raise DecodeError(f"expected ACK, got {msg_type!r}: {body!r}")
        r = Reader(body)
        proto, recv_buf, send_buf, max_msg, max_chunks = r.uint32(), r.uint32(), r.uint32(), r.uint32(), r.uint32()
        print(f"  Acknowledge: protocolVersion={proto} recvBuf={recv_buf} sendBuf={send_buf} "
              f"maxMsg={max_msg} maxChunkCount={max_chunks}")

    # --- OpenSecureChannel ---
    def open_secure_channel(self):
        w = Writer()
        w.request_header()
        w.uint32(0)  # clientProtocolVersion
        w.int32(0)   # requestType: Issue
        w.int32(1)   # securityMode: None
        w.byte_string(None)  # clientNonce
        w.uint32(3600000)    # requestedLifetime (ms)
        body = w.buf

        # OPN body = TypeId + struct fields
        payload = Writer()
        payload.node_id_numeric(0, NS0_OPEN_SECURE_CHANNEL_REQUEST)
        payload.raw(body)

        header_extra = struct.pack("<I", 0)  # SecureChannelId (0 = "assign one", per spec)
        sec_header = Writer()
        sec_header.string(SECURITY_POLICY_NONE_URI)
        sec_header.byte_string(None)  # SenderCertificate
        sec_header.byte_string(None)  # ReceiverCertificateThumbprint
        seq_header = struct.pack("<II", self._next_seq(), 1)  # SequenceNumber, RequestId

        self._send_frame(b"OPN", b"F", header_extra + bytes(sec_header.buf) + seq_header, bytes(payload.buf))

        msg_type, resp = self._recv_message()
        if msg_type != b"OPN":
            raise DecodeError(f"expected OPN response, got {msg_type!r}")
        r = Reader(resp)
        self.channel_id = r.uint32()
        policy_uri = r.string()
        r.byte_string()  # SenderCertificate
        r.byte_string()  # ReceiverCertificateThumbprint
        seq_num = r.uint32()
        req_id = r.uint32()
        type_id = r.node_id()
        rh = r.response_header()
        server_proto = r.uint32()
        chan_id2 = r.uint32()
        self.token_id = r.uint32()
        created_at = r.int64()
        revised_lifetime = r.uint32()
        r.byte_string()  # ServerNonce
        print(f"  OpenSecureChannel: channelId={self.channel_id} tokenId={self.token_id} "
              f"policy={policy_uri} serviceResult={status_name(rh['service_result'])}")
        if rh["service_result"] != 0:
            raise DecodeError("OpenSecureChannel rejected")
        _ = (seq_num, req_id, type_id, server_proto, chan_id2, created_at, revised_lifetime)

    def _send_service(self, type_id_num: int, body: bytes, request_id: int):
        payload = Writer()
        payload.node_id_numeric(0, type_id_num)
        payload.raw(body)
        header_extra = struct.pack("<III", self.channel_id, self.token_id, self._next_seq())
        header_extra += struct.pack("<I", request_id)
        self._send_frame(b"MSG", b"F", header_extra, bytes(payload.buf))

    def _recv_service(self):
        msg_type, resp = self._recv_message()
        if msg_type != b"MSG":
            raise DecodeError(f"expected MSG response, got {msg_type!r}")
        r = Reader(resp)
        chan_id = r.uint32()
        token_id = r.uint32()
        seq_num = r.uint32()
        req_id = r.uint32()
        type_ns, type_num = r.node_id()
        _ = (chan_id, token_id, seq_num, req_id, type_ns)
        return type_num, r

    # --- Session ---
    def create_session(self, endpoint_url: str, session_name="opcua-test-client"):
        w = Writer()
        w.request_header()
        # ClientDescription (ApplicationDescription)
        w.string("urn:test-client:opcua")
        w.string("urn:test-client:opcua:product")
        w.byte(0x02)  # LocalizedText encoding: text present
        w.string("Python OPC UA Test Client")
        w.int32(1)  # applicationType: Client
        w.string(None)  # gatewayServerUri
        w.string(None)  # discoveryProfileUri
        w.int32(-1)  # discoveryUrls: null array
        w.string(None)  # serverUri
        w.string(endpoint_url)
        w.string(session_name)
        w.byte_string(None)  # clientNonce
        w.byte_string(None)  # clientCertificate
        w.double(60000.0)  # requestedSessionTimeout
        w.uint32(65536)  # maxResponseMessageSize

        self._send_service(NS0_CREATE_SESSION_REQUEST, bytes(w.buf), request_id=10)
        type_num, r = self._recv_service()
        if type_num == NS0_SERVICE_FAULT:
            rh = r.response_header()
            raise DecodeError(f"CreateSession -> ServiceFault: {status_name(rh['service_result'])}")
        if type_num != NS0_CREATE_SESSION_RESPONSE:
            raise DecodeError(f"unexpected response TypeId {type_num}")
        rh = r.response_header()
        session_id = r.node_id()
        self.auth_token = r.node_id()
        revised_timeout = r.double()
        r.byte_string()  # ServerNonce
        r.byte_string()  # ServerCertificate
        n_endpoints = r.int32()
        for _ in range(max(n_endpoints, 0)):
            self._skip_endpoint_description(r)
        print(f"  CreateSession: sessionId={session_id} authToken={self.auth_token} "
              f"revisedTimeout={revised_timeout} serviceResult={status_name(rh['service_result'])}")
        if rh["service_result"] != 0:
            raise DecodeError("CreateSession rejected")

    def _skip_endpoint_description(self, r: Reader):
        r.string()  # endpointUrl
        # ApplicationDescription
        r.string(); r.string(); r.localized_text(); r.int32(); r.string(); r.string()
        n = r.int32()
        for _ in range(max(n, 0)):
            r.string()
        r.byte_string()  # serverCertificate
        r.int32()  # securityMode
        r.string()  # securityPolicyUri
        n_tokens = r.int32()
        for _ in range(max(n_tokens, 0)):
            r.string(); r.int32(); r.string(); r.string(); r.string()
        r.string()  # transportProfileUri
        r.byte()  # securityLevel

    def activate_session(self):
        w = Writer()
        w.request_header(auth_token_ns=self.auth_token[0], auth_token_id=self.auth_token[1])
        w.string(None); w.byte_string(None)  # ClientSignature: Algorithm, Signature
        w.int32(-1)  # ClientSoftwareCertificates: null array
        w.int32(-1)  # LocaleIds: null array
        # UserIdentityToken: AnonymousIdentityToken (ns=0;i=321), body = {PolicyId: String}
        w.node_id_numeric(0, 321)
        w.byte(0x01)  # ExtensionObject encoding: binary body present
        inner = Writer()
        inner.string("anonymous")
        w.byte_string(bytes(inner.buf))
        w.string(None); w.byte_string(None)  # UserTokenSignature

        self._send_service(NS0_ACTIVATE_SESSION_REQUEST, bytes(w.buf), request_id=11)
        type_num, r = self._recv_service()
        if type_num == NS0_SERVICE_FAULT:
            rh = r.response_header()
            raise DecodeError(f"ActivateSession -> ServiceFault: {status_name(rh['service_result'])}")
        if type_num != NS0_ACTIVATE_SESSION_RESPONSE:
            raise DecodeError(f"unexpected response TypeId {type_num}")
        rh = r.response_header()
        print(f"  ActivateSession: serviceResult={status_name(rh['service_result'])}")
        if rh["service_result"] != 0:
            raise DecodeError("ActivateSession rejected")

    def close_session(self):
        w = Writer()
        w.request_header(auth_token_ns=self.auth_token[0], auth_token_id=self.auth_token[1])
        w.boolean(False)  # deleteSubscriptions
        self._send_service(NS0_CLOSE_SESSION_REQUEST, bytes(w.buf), request_id=99)
        type_num, r = self._recv_service()
        rh = r.response_header()
        print(f"  CloseSession: serviceResult={status_name(rh['service_result'])}")

    def close_secure_channel(self):
        w = Writer()
        w.request_header(auth_token_ns=self.auth_token[0], auth_token_id=self.auth_token[1])
        payload = Writer()
        payload.node_id_numeric(0, NS0_CLOSE_SECURE_CHANNEL_REQUEST)
        payload.raw(w.buf)
        header_extra = struct.pack("<III", self.channel_id, self.token_id, self._next_seq())
        header_extra += struct.pack("<I", 100)
        self._send_frame(b"CLO", b"F", header_extra, bytes(payload.buf))
        print("  CloseSecureChannel sent (no response expected)")

    # --- Browse ---
    def browse(self, node_ns: int, node_id: int, direction=0):
        """direction: 0=Forward, 1=Inverse, 2=Both. Returns list of
        (referenceTypeId, isForward, targetNodeId, browseName, displayName, nodeClass)."""
        w = Writer()
        w.request_header(auth_token_ns=self.auth_token[0], auth_token_id=self.auth_token[1])
        w.node_id_numeric(0, 0)  # View.ViewId = null
        w.raw(struct.pack("<q", 0))  # View.Timestamp
        w.uint32(0)  # View.ViewVersion
        w.uint32(0)  # RequestedMaxReferencesPerNode (unlimited -- server ignores this anyway)
        w.int32(1)  # NodesToBrowse: 1 entry
        w.node_id_numeric(node_ns, node_id)
        w.int32(direction)
        w.node_id_numeric(0, 0)  # ReferenceTypeId: null = all types
        w.boolean(True)  # IncludeSubtypes
        w.uint32(0)  # NodeClassMask: 0 = all
        w.uint32(0x3F)  # ResultMask: all fields

        self._send_service(NS0_BROWSE_REQUEST, bytes(w.buf), request_id=20)
        type_num, r = self._recv_service()
        if type_num == NS0_SERVICE_FAULT:
            rh = r.response_header()
            raise DecodeError(f"Browse -> ServiceFault: {status_name(rh['service_result'])}")
        if type_num != NS0_BROWSE_RESPONSE:
            raise DecodeError(f"unexpected response TypeId {type_num}")
        r.response_header()
        n_results = r.int32()
        assert n_results == 1
        status = r.status_code()
        r.byte_string()  # ContinuationPoint
        n_refs = r.int32()
        results = []
        for _ in range(max(n_refs, 0)):
            ref_type = r.node_id()
            is_forward = r.boolean()
            target = r.expanded_node_id()
            browse_name = r.qualified_name()
            display_name = r.localized_text()
            node_class = r.int32()
            r.expanded_node_id()  # TypeDefinition
            results.append((ref_type, is_forward, target, browse_name, display_name, node_class))
        if status != 0:
            raise DecodeError(f"Browse result status: {status_name(status)}")
        return results

    # --- Read / Write ---
    def read_value(self, node_ns: int, node_id: int, attribute_id=ATTR_VALUE):
        w = Writer()
        w.request_header(auth_token_ns=self.auth_token[0], auth_token_id=self.auth_token[1])
        w.double(0)  # MaxAge
        w.int32(0)  # TimestampsToReturn: Source
        w.int32(1)  # NodesToRead: 1 entry
        w.node_id_numeric(node_ns, node_id)
        w.uint32(attribute_id)
        w.string(None)  # IndexRange
        w.qualified_name(0, None)  # DataEncoding

        self._send_service(NS0_READ_REQUEST, bytes(w.buf), request_id=30)
        type_num, r = self._recv_service()
        if type_num == NS0_SERVICE_FAULT:
            rh = r.response_header()
            raise DecodeError(f"Read -> ServiceFault: {status_name(rh['service_result'])}")
        if type_num != NS0_READ_RESPONSE:
            raise DecodeError(f"unexpected response TypeId {type_num}")
        r.response_header()
        n = r.int32()
        assert n == 1
        return r.data_value()

    def write_value(self, node_ns: int, node_id: int, py_value, ua_type_id: int, attribute_id=ATTR_VALUE):
        w = Writer()
        w.request_header(auth_token_ns=self.auth_token[0], auth_token_id=self.auth_token[1])
        w.int32(1)  # NodesToWrite: 1 entry
        w.node_id_numeric(node_ns, node_id)
        w.uint32(attribute_id)
        w.string(None)  # IndexRange
        # DataValue: Value only
        w.byte(0x01)
        w.byte(ua_type_id)
        if ua_type_id == 12:
            w.string(py_value)
        elif ua_type_id == 7:
            w.uint32(py_value)
        elif ua_type_id == 5:
            w.uint16(py_value)
        else:
            raise NotImplementedError(f"write_value: type {ua_type_id} not supported by this test client")

        self._send_service(NS0_WRITE_REQUEST, bytes(w.buf), request_id=40)
        type_num, r = self._recv_service()
        if type_num == NS0_SERVICE_FAULT:
            rh = r.response_header()
            raise DecodeError(f"Write -> ServiceFault: {status_name(rh['service_result'])}")
        if type_num != NS0_WRITE_RESPONSE:
            raise DecodeError(f"unexpected response TypeId {type_num}")
        r.response_header()
        n = r.int32()
        assert n == 1
        return r.status_code()


def run(host: str, port: int):
    endpoint_url = f"opc.tcp://{host}:{port}"
    print(f"Connecting to {endpoint_url} ...")
    client = OpcUaTestClient(host, port)
    try:
        print("Hello/Acknowledge:")
        client.hello(endpoint_url)

        print("OpenSecureChannel:")
        client.open_secure_channel()

        print("CreateSession:")
        client.create_session(endpoint_url)

        print("ActivateSession:")
        client.activate_session()

        print("Browse Objects (i=85):")
        refs = client.browse(0, NS0_OBJECTS_FOLDER)
        folder_id = None
        for ref_type, is_forward, target, browse_name, display_name, node_class in refs:
            print(f"  -> {target} browseName={browse_name} displayName={display_name} nodeClass={node_class}")
            if browse_name[1] == "FactoryControlUnit":
                folder_id = target
        if folder_id is None:
            print("FAIL: FactoryControlUnit folder not found under Objects")
            sys.exit(1)

        print(f"Browse FactoryControlUnit ({folder_id}):")
        refs = client.browse(folder_id[0], folder_id[1])
        nodes = {}
        for ref_type, is_forward, target, browse_name, display_name, node_class in refs:
            print(f"  -> {target} browseName={browse_name} displayName={display_name} nodeClass={node_class}")
            nodes[browse_name[1]] = target

        for name in ("Greeting", "UptimeSeconds"):
            if name not in nodes:
                print(f"FAIL: {name} node not found")
                sys.exit(1)
            ns, nid = nodes[name]
            dv = client.read_value(ns, nid)
            print(f"Read {name} ({ns},{nid}): value={dv['value']!r} status={status_name(dv['status'])}")

        # Greeting is read-only (see opcua_test_address_space.cpp) -- this Write must be
        # rejected. A Good result here would mean access-level enforcement is broken.
        ns, nid = nodes["Greeting"]
        result = client.write_value(ns, nid, "hacked", ua_type_id=12)
        print(f"Write Greeting (expect BadNotWritable): {status_name(result)}")
        if result == 0:
            print("FAIL: server accepted a write to a read-only node")
            sys.exit(1)

        print("CloseSession:")
        client.close_session()
        print("CloseSecureChannel:")
        client.close_secure_channel()

        print("\nAll checks passed.")
    finally:
        client.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", required=True, help="Device hostname or IP, e.g. factory-box-XXXXXX.local")
    parser.add_argument("--port", type=int, default=OPCUA_PORT)
    args = parser.parse_args()

    try:
        run(args.host, args.port)
    except (DecodeError, OSError) as e:
        print(f"\nFEHLER: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
