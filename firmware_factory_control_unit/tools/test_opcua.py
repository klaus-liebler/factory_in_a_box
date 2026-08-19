#!/usr/bin/env python3
"""Hand-rolled OPC UA Binary test client for the Control Unit's from-scratch OPC UA server
(libs/opcua_native/, feature/opcua-native-server branch) -- SecurityPolicy#Basic256Sha256,
SecurityMode Sign, single TCP chunk per message, anonymous session auth. Deliberately NOT built
on a third-party OPC UA library (e.g. python-opcua/asyncua): those implement the FULL spec far
more strictly than this minimal server does, so a real client library would likely reject/mis-
diagnose things that are fine for this server's intentionally-reduced scope.

Needs the 'cryptography' package for real RSA sign/verify/OAEP (stdlib alone doesn't provide
this) -- see tools/requirements.txt (pip install -r tools/requirements.txt).

This server offers Basic256Sha256 ONLY (no unsecured discovery channel, see the project's
security-policy notes) -- a client must already know/trust the server's certificate out of band
before it can open a channel at all, exactly like this project's HTTPS trust model. Pass it via
--server-cert (the same *.pem.crt the board's HTTPS server uses, e.g. from the board archive).
This test client generates (or reuses a cached) self-signed RSA client certificate on first run.

Exercises the whole handshake end to end: Hello/Acknowledge -> OpenSecureChannel (RSA-OAEP
encrypted + RSA-PKCS1v15-SHA256 signed, per Part 6 6.7.4 -- always both, regardless of
SecurityMode) -> CreateSession -> ActivateSession (anonymous) -> Browse (Objects ->
FactoryControlUnit -> Diagnostics/Ws2812) -> Read Diagnostics.CpuTemperatureC/HealthState ->
attempt Write on the read-only HealthState node (expects BadNotWritable, confirming access-level
enforcement still works) -> Write + read back Ws2812.Ws2812Ch1Pattern (a genuinely writable
Holding-backed node, via UniversalRegisterAccess's generated address space) -> CloseSession ->
CloseSecureChannel. Every MSG/CLO frame after the handshake is HMAC-SHA256 signed (SecurityMode
Sign, no AES encryption of ongoing traffic -- see the project's security-policy notes).

Examples:
    python test_opcua.py --host factory-box-363836.local --server-cert path\\to\\factory-box-363836.pem.crt
    python test_opcua.py --host 192.168.1.50 --server-cert server.pem.crt
"""
import argparse
import datetime
import hashlib
import hmac
import os
import socket
import struct
import sys
from pathlib import Path

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding, rsa
from cryptography.x509.oid import NameOID

OPCUA_PORT = 4840
SECURITY_POLICY_BASIC256SHA256_URI = b"http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256"
SECURITY_MODE_SIGN = 2
NONCE_SIZE = 32
SIGNING_KEY_SIZE = 32  # HMAC-SHA256

# Cached client identity (self-signed, generated once) -- gitignored, never committed, mirrors
# how this project already treats device certificates (see .gitignore).
CLIENT_IDENTITY_DIR = Path(__file__).parent / ".opcua-test-client"


def _load_or_create_client_identity():
    key_path = CLIENT_IDENTITY_DIR / "client_key.pem"
    cert_path = CLIENT_IDENTITY_DIR / "client_cert.der"
    if key_path.exists() and cert_path.exists():
        key = serialization.load_pem_private_key(key_path.read_bytes(), password=None)
        cert_der = cert_path.read_bytes()
        return key, cert_der

    CLIENT_IDENTITY_DIR.mkdir(parents=True, exist_ok=True)
    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = issuer = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, "opcua-test-client")])
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now - datetime.timedelta(days=1))
        .not_valid_after(now + datetime.timedelta(days=3650))
        .sign(key, hashes.SHA256())
    )
    cert_der = cert.public_bytes(serialization.Encoding.DER)
    key_path.write_bytes(key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                            serialization.NoEncryption()))
    cert_path.write_bytes(cert_der)
    print(f"  Generated new test client identity in {CLIENT_IDENTITY_DIR}")
    return key, cert_der


def _rsa_public_key_from_der(cert_der: bytes):
    return x509.load_der_x509_certificate(cert_der).public_key()


def _rsa_sha256_sign(private_key, message: bytes) -> bytes:
    return private_key.sign(message, padding.PKCS1v15(), hashes.SHA256())


def _rsa_sha256_verify(public_key, message: bytes, signature: bytes) -> bool:
    try:
        public_key.verify(signature, message, padding.PKCS1v15(), hashes.SHA256())
        return True
    except Exception:
        return False


def _rsa_oaep_max_plaintext(modulus_bytes: int) -> int:
    return modulus_bytes - 2 * 20 - 2  # SHA-1-based OAEP overhead, Basic256Sha256's AsymmetricEncryptionAlgorithm


def _rsa_oaep_encrypt_blocks(public_key, plaintext: bytes) -> bytes:
    modulus_bytes = (public_key.key_size + 7) // 8
    block = _rsa_oaep_max_plaintext(modulus_bytes)
    assert len(plaintext) % block == 0, "caller must pad to an exact block multiple first"
    oaep = padding.OAEP(mgf=padding.MGF1(algorithm=hashes.SHA1()), algorithm=hashes.SHA1(), label=None)
    out = bytearray()
    for i in range(0, len(plaintext), block):
        out += public_key.encrypt(plaintext[i:i + block], oaep)
    return bytes(out)


def _rsa_oaep_decrypt_blocks(private_key, ciphertext: bytes) -> bytes:
    modulus_bytes = (private_key.key_size + 7) // 8
    block = _rsa_oaep_max_plaintext(modulus_bytes)
    assert len(ciphertext) % modulus_bytes == 0
    oaep = padding.OAEP(mgf=padding.MGF1(algorithm=hashes.SHA1()), algorithm=hashes.SHA1(), label=None)
    out = bytearray()
    for i in range(0, len(ciphertext), modulus_bytes):
        plain = private_key.decrypt(ciphertext[i:i + modulus_bytes], oaep)
        assert len(plain) == block, "peer did not pad to an exact block multiple"
        out += plain
    return bytes(out)


def _hmac_sha256(key: bytes, data: bytes) -> bytes:
    return hmac.new(key, data, hashlib.sha256).digest()


def _p_sha256(secret: bytes, seed: bytes, length: int) -> bytes:
    """RFC 5246 5. / OPC UA Part 6 6.7.5 -- same construction the firmware side implements in
    libs/opcua_native/src/security_crypto.cpp's PSha256()."""
    result = bytearray()
    a = _hmac_sha256(secret, seed)
    while len(result) < length:
        result += _hmac_sha256(secret, a + seed)
        a = _hmac_sha256(secret, a)
    return bytes(result[:length])


def _asymmetric_padding(plaintext_len: int, signature_len: int, plain_block: int) -> int:
    """Padding count N such that (plaintext_len + 1 + N + signature_len) is a multiple of
    plain_block -- mirrors AppendAsymmetricPadding's caller in secure_channel.cpp."""
    base = plaintext_len + 1 + signature_len
    return (plain_block - (base % plain_block)) % plain_block
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
    def __init__(self, host, server_cert_der: bytes, port=OPCUA_PORT, timeout=5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.channel_id = 0
        self.token_id = 0
        self.seq = 1
        self.auth_token = (0, 0)
        self.client_key, self.client_cert_der = _load_or_create_client_identity()
        self.server_cert_der = server_cert_der
        self.server_public_key = _rsa_public_key_from_der(server_cert_der)
        # Filled in by open_secure_channel() from the ClientNonce/ServerNonce exchange -- see
        # its docstring for the secret/seed role mapping (Part 6 6.7.5).
        self.my_signing_key = b""
        self.peer_signing_key = b""

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
        """Part 6 6.7.4: OPN messages are ALWAYS both signed AND encrypted whenever
        SecurityPolicy != None, independent of the requested SecurityMode -- mirrors
        SecureChannel::HandleOpen in libs/opcua_native/src/secure_channel.cpp exactly (see its
        comments for the full byte-layout reasoning)."""
        client_nonce = os.urandom(NONCE_SIZE)

        seq_header = struct.pack("<II", self._next_seq(), 1)  # SequenceNumber, RequestId
        body = Writer()
        body.node_id_numeric(0, NS0_OPEN_SECURE_CHANNEL_REQUEST)
        body.request_header()
        body.uint32(0)  # clientProtocolVersion
        body.int32(0)   # requestType: Issue
        body.int32(SECURITY_MODE_SIGN)
        body.byte_string(client_nonce)
        body.uint32(3600000)  # requestedLifetime (ms)
        seq_header_plus_body = seq_header + bytes(body.buf)

        client_modulus_bytes = 2048 // 8  # this client's own key, fixed at generation above
        server_modulus_bytes = (self.server_public_key.key_size + 7) // 8
        plain_block = _rsa_oaep_max_plaintext(server_modulus_bytes)  # encrypting TO the server
        pad = _asymmetric_padding(len(seq_header_plus_body), client_modulus_bytes, plain_block)
        plaintext_no_sig = seq_header_plus_body + bytes([pad]) + bytes([pad]) * pad
        plaintext_len = len(plaintext_no_sig) + client_modulus_bytes  # +signature
        cipher_len = (plaintext_len // plain_block) * server_modulus_bytes

        sec_header = Writer()
        sec_header.string(SECURITY_POLICY_BASIC256SHA256_URI)
        sec_header.byte_string(self.client_cert_der)
        sec_header.byte_string(hashlib.sha1(self.server_cert_der).digest())
        # The signature must cover the EXACT bytes actually sent, including the final messageSize
        # field -- so messageSize has to be correct BEFORE signing, not patched in afterward (a
        # patch-after-sign would leave the signature covering a different messageSize than the
        # one on the wire, failing verification 100% of the time). Every length needed is already
        # known before building the header, mirroring SecureChannel::HandleOpen's fix in
        # secure_channel.cpp for the exact same bug on the response side.
        header_and_secheader_len = 8 + 4 + len(sec_header.buf)  # TCP header + ChannelId + secheader
        message_size = header_and_secheader_len + cipher_len
        header_and_secheader = b"OPN" + b"F" + struct.pack("<I", message_size) + struct.pack("<I", 0) + bytes(sec_header.buf)
        assert len(header_and_secheader) == header_and_secheader_len

        signature = _rsa_sha256_sign(self.client_key, header_and_secheader + plaintext_no_sig)
        plaintext = plaintext_no_sig + signature
        assert len(plaintext) == plaintext_len
        ciphertext = _rsa_oaep_encrypt_blocks(self.server_public_key, plaintext)
        assert len(ciphertext) == cipher_len
        self.sock.sendall(header_and_secheader + ciphertext)

        msg_type, resp = self._recv_message()
        if msg_type != b"OPN":
            raise DecodeError(f"expected OPN response, got {msg_type!r}")
        r = Reader(resp)
        self.channel_id = r.uint32()
        policy_uri = r.string()
        sender_cert = r.byte_string()  # server's own certificate (not re-verified against
        # --server-cert here beyond using it to check the signature below -- this test client's
        # trust decision is the --server-cert argument itself, same out-of-band model as the
        # server's own "accept any client cert for signature purposes" choice)
        r.byte_string()  # ReceiverCertificateThumbprint
        cleartext_len = r.pos
        ciphertext_resp = resp[cleartext_len:]
        if len(ciphertext_resp) % client_modulus_bytes != 0:
            raise DecodeError("OPN response ciphertext is not a multiple of our modulus size")
        plain_resp = _rsa_oaep_decrypt_blocks(self.client_key, ciphertext_resp)

        server_sig_len = server_modulus_bytes
        signature_start = len(plain_resp) - server_sig_len
        padding_count = plain_resp[signature_start - 1]
        padding_size_byte_idx = signature_start - 1 - padding_count
        # _recv_message() already stripped the 8-byte TCP header (message type + chunk type +
        # size) from "resp" -- but the firmware signs those bytes too (the whole on-wire message
        # up to the signature itself), so they have to be reconstructed here for verification to
        # match (same fix as _recv_service()'s "full" reconstruction below).
        full_cleartext = b"OPN" + b"F" + struct.pack("<I", 8 + len(resp)) + resp[:cleartext_len]
        signed_part = full_cleartext + plain_resp[:signature_start]
        if not _rsa_sha256_verify(self.server_public_key, signed_part, plain_resp[signature_start:]):
            raise DecodeError("OPN response signature verification failed")

        pr = Reader(plain_resp[:padding_size_byte_idx])
        seq_num = pr.uint32()
        req_id = pr.uint32()
        type_id = pr.node_id()
        rh = pr.response_header()
        server_proto = pr.uint32()
        chan_id2 = pr.uint32()
        self.token_id = pr.uint32()
        created_at = pr.int64()
        revised_lifetime = pr.uint32()
        server_nonce = pr.byte_string()
        print(f"  OpenSecureChannel: channelId={self.channel_id} tokenId={self.token_id} "
              f"policy={policy_uri} serviceResult={status_name(rh['service_result'])}")
        if rh["service_result"] != 0:
            raise DecodeError("OpenSecureChannel rejected")
        if not server_nonce or len(server_nonce) != NONCE_SIZE:
            raise DecodeError("OpenSecureChannel: missing/wrong-size ServerNonce")

        # Part 6 6.7.5: "Client*" keys (used to sign what WE send / the server uses to verify us)
        # derive from (secret=ServerNonce, seed=ClientNonce); "Server*" keys (used to verify what
        # the server sends us) from (secret=ClientNonce, seed=ServerNonce) -- same mapping as
        # SecureChannel::HandleOpen's comment in secure_channel.cpp.
        self.my_signing_key = _p_sha256(server_nonce, client_nonce, SIGNING_KEY_SIZE)
        self.peer_signing_key = _p_sha256(client_nonce, server_nonce, SIGNING_KEY_SIZE)
        _ = (seq_num, req_id, type_id, server_proto, chan_id2, created_at, revised_lifetime, sender_cert)

    def _send_service(self, type_id_num: int, body: bytes, request_id: int):
        payload = Writer()
        payload.node_id_numeric(0, type_id_num)
        payload.raw(body)
        header_extra = struct.pack("<III", self.channel_id, self.token_id, self._next_seq())
        header_extra += struct.pack("<I", request_id)
        message_no_sig = b"MSG" + b"F" + struct.pack("<I", 0) + header_extra + bytes(payload.buf)
        # Message-size placeholder above is wrong on purpose -- SignAndFinishSymmetric-equivalent:
        # patch it to the REAL final size (including the signature) before signing, exactly like
        # the firmware side does (SecureChannel::SignAndFinishSymmetric).
        final_size = len(message_no_sig) + SIGNING_KEY_SIZE
        message_no_sig = message_no_sig[:4] + struct.pack("<I", final_size) + message_no_sig[8:]
        signature = _hmac_sha256(self.my_signing_key, message_no_sig)
        self.sock.sendall(message_no_sig + signature)

    def _recv_service(self):
        msg_type, resp_body = self._recv_message()
        if msg_type != b"MSG":
            raise DecodeError(f"expected MSG response, got {msg_type!r}")
        full = b"MSG" + b"F" + struct.pack("<I", 8 + len(resp_body)) + resp_body
        if len(full) < SIGNING_KEY_SIZE:
            raise DecodeError("MSG response too short to contain a signature")
        signature_start = len(full) - SIGNING_KEY_SIZE
        signed_part, signature = full[:signature_start], full[signature_start:]
        expected = _hmac_sha256(self.peer_signing_key, signed_part)
        if not hmac.compare_digest(expected, signature):
            raise DecodeError("MSG response signature verification failed")

        r = Reader(resp_body[:signature_start - 8])
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
        w.byte_string(os.urandom(NONCE_SIZE))  # clientNonce
        w.byte_string(self.client_cert_der)    # clientCertificate -- lets the server compute a
        # real ServerSignature (proof-of-possession, Part 4 5.6.2.2) in its response.
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
        # CLO is HMAC-signed like MSG (Part 6 6.7.2 Table 62 covers both) -- same pattern as
        # _send_service, just with a different message type and no response expected.
        message_no_sig = b"CLO" + b"F" + struct.pack("<I", 0) + header_extra + bytes(payload.buf)
        final_size = len(message_no_sig) + SIGNING_KEY_SIZE
        message_no_sig = message_no_sig[:4] + struct.pack("<I", final_size) + message_no_sig[8:]
        signature = _hmac_sha256(self.my_signing_key, message_no_sig)
        self.sock.sendall(message_no_sig + signature)
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


def run(host: str, port: int, server_cert_der: bytes):
    endpoint_url = f"opc.tcp://{host}:{port}"
    print(f"Connecting to {endpoint_url} ...")
    client = OpcUaTestClient(host, server_cert_der, port)
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
        regions = {}
        for ref_type, is_forward, target, browse_name, display_name, node_class in refs:
            print(f"  -> {target} browseName={browse_name} displayName={display_name} nodeClass={node_class}")
            regions[browse_name[1]] = target

        for name in ("Diagnostics", "Ws2812"):
            if name not in regions:
                print(f"FAIL: {name} region object not found under FactoryControlUnit")
                sys.exit(1)

        print(f"Browse Diagnostics ({regions['Diagnostics']}):")
        refs = client.browse(regions["Diagnostics"][0], regions["Diagnostics"][1])
        diag_nodes = {}
        for ref_type, is_forward, target, browse_name, display_name, node_class in refs:
            print(f"  -> {target} browseName={browse_name} displayName={display_name} nodeClass={node_class}")
            diag_nodes[browse_name[1]] = target

        for name in ("CpuTemperatureC", "HealthState"):
            if name not in diag_nodes:
                print(f"FAIL: Diagnostics.{name} node not found")
                sys.exit(1)
            ns, nid = diag_nodes[name]
            dv = client.read_value(ns, nid)
            print(f"Read Diagnostics.{name} ({ns},{nid}): value={dv['value']!r} status={status_name(dv['status'])}")

        # HealthState is Access.ReadOnly (Input register) in register_map_schema/Diagnostics.cs --
        # this Write must be rejected. A Good result here would mean access-level enforcement is
        # broken (or the schema's Access got flipped without the OPC UA node following it).
        ns, nid = diag_nodes["HealthState"]
        result = client.write_value(ns, nid, 0, ua_type_id=5)
        print(f"Write Diagnostics.HealthState (expect BadNotWritable): {status_name(result)}")
        if result == 0:
            print("FAIL: server accepted a write to a read-only (Access.ReadOnly) node")
            sys.exit(1)

        print(f"Browse Ws2812 ({regions['Ws2812']}):")
        refs = client.browse(regions["Ws2812"][0], regions["Ws2812"][1])
        ws_nodes = {}
        for ref_type, is_forward, target, browse_name, display_name, node_class in refs:
            print(f"  -> {target} browseName={browse_name} displayName={display_name} nodeClass={node_class}")
            ws_nodes[browse_name[1]] = target
        if "Ws2812Ch1Pattern" not in ws_nodes:
            print("FAIL: Ws2812.Ws2812Ch1Pattern node not found")
            sys.exit(1)
        ns, nid = ws_nodes["Ws2812Ch1Pattern"]
        dv = client.read_value(ns, nid)
        print(f"Read Ws2812.Ws2812Ch1Pattern before write: value={dv['value']!r} status={status_name(dv['status'])}")

        # Access.ReadWrite (Holding register) -- this is the first genuinely writable node this
        # server has ever had (the old hand-written test address space had none). A BadNotWritable
        # here would mean the OPC UA generator's Access.ReadWrite -> accessLevel/writeCallback
        # wiring is broken.
        new_value = (dv["value"] + 1) % 5
        result = client.write_value(ns, nid, new_value, ua_type_id=5)
        print(f"Write Ws2812.Ws2812Ch1Pattern={new_value} (expect Good): {status_name(result)}")
        if result != 0:
            print("FAIL: server rejected a write to a Access.ReadWrite node")
            sys.exit(1)
        dv = client.read_value(ns, nid)
        print(f"Read Ws2812.Ws2812Ch1Pattern after write: value={dv['value']!r} status={status_name(dv['status'])}")
        if dv["value"] != new_value:
            print(f"FAIL: read-back value {dv['value']!r} does not match written value {new_value!r} -- "
                  "OPC UA write and the underlying ModbusRegisterModel storage are not consistent")
            sys.exit(1)

        print("CloseSession:")
        client.close_session()
        print("CloseSecureChannel:")
        client.close_secure_channel()

        print("\nAll checks passed.")
    finally:
        client.close()


def _load_server_cert_der(path: str) -> bytes:
    raw = Path(path).read_bytes()
    if raw.lstrip().startswith(b"-----BEGIN"):
        return x509.load_pem_x509_certificate(raw).public_bytes(serialization.Encoding.DER)
    return raw


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", required=True, help="Device hostname or IP, e.g. factory-box-XXXXXX.local")
    parser.add_argument("--port", type=int, default=OPCUA_PORT)
    parser.add_argument("--server-cert", required=True,
                        help="Path to the server's certificate (PEM or DER) -- this server offers "
                             "Basic256Sha256 only, so the client must already trust it out of band "
                             "(e.g. the board archive's factory-box-<id>.pem.crt).")
    args = parser.parse_args()

    try:
        server_cert_der = _load_server_cert_der(args.server_cert)
        run(args.host, args.port, server_cert_der)
    except (DecodeError, OSError) as e:
        print(f"\nFEHLER: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
