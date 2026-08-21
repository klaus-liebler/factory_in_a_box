#include "opcua_setup.hpp"

#include <cstdio>
#include <span>

#include "nx_secure_x509.h"

#include "opcua/net_transport.hpp"
#include "opcua/secure_channel.hpp"
#include "generated/opcua_registers_generated.hh"

#include "app.hh"
#include "assets.h"
#include "common_macros.hh"
#include "log.h"
#include "main.h"

namespace {

constexpr UINT OPCUA_PORT = 4840;
constexpr UINT OPCUA_SESSION_TIMEOUT_SECONDS = 60;
constexpr uint32_t OPCUA_PACKET_SIZE = 1536;
constexpr uint32_t OPCUA_POOL_SIZE = OPCUA_PACKET_SIZE * 8;
// The NX_TCPSERVER worker thread this stack belongs to is where ALL request processing
// happens (OnReceiveData calls straight into Connection::HandleMessage(), synchronously) --
// matches the open62541 branch's identically-sized stack for the same reason (see its
// opcua_setup.cpp comment). Bumped from the original 8192 for SecurityPolicy#Basic256Sha256
// (secure_channel.cpp's OpenSecureChannel handling keeps several ~1.5-2KB buffers live at once
// -- decrypted request plaintext, the request/response signing concatenation, the response
// plaintext-to-encrypt -- plus whatever NetX Crypto's own RSA scratch buffer needs on top while
// a sign/verify/encrypt/decrypt call is active); RAM has ample headroom (see build output).
constexpr uint32_t OPCUA_THREAD_STACK_SIZE = 24576;
// Same band as the Modbus TCP server thread (DEFAULT_PRIORITY in app.cc) -- reacts to client
// requests, doesn't need to preempt the I/O thread's sensor/actuator cycle.
constexpr UINT OPCUA_THREAD_PRIORITY = 6;

char g_endpointUrl[64];

// This server's own identity for SecurityPolicy#Basic256Sha256 (secure_channel.hpp) -- the SAME
// RSA device certificate/key net_setup.cpp's HTTPS TLS setup uses (one certificate, one key,
// shared by both servers, per the project decision). Parsed once here (own NX_SECURE_X509_CERT
// instance, separate from net_setup.cpp's -- each consumer owns its own parse rather than
// sharing a struct across independently-initialized subsystems) purely to extract the RSA key
// material into opcua::ServerIdentity; the cert object itself isn't otherwise used afterward.
NX_SECURE_X509_CERT g_opcuaServerCertificate;
opcua::ServerIdentity g_opcuaServerIdentity;

} // namespace

void OpcUaServerSetup(App *app) {
    // Wires the compile-time-fixed generated Node table's Read/WriteCallbacks (see
    // opcua_registers_generated.hh -- one shared callback per base type, driven by a per-register
    // RegisterContext) to the runtime register storage. Must happen before AddressSpaceInstance()
    // is handed to the server below; app->register_model already exists at this point (created in
    // App::SetupBeforeThreadX(), long before AppThread()/OpcUaServerSetup() run).
    GeneratedOpcUa::SetRegisterModel(app->register_model);

    const USHORT device_cert_der_len =
        (USHORT)(_binary_device_certificate_der_end - _binary_device_certificate_der_start);
    const USHORT device_key_der_len = (USHORT)(_binary_device_key_der_end - _binary_device_key_der_start);
    // nx_secure_x509_certificate_initialize() parses exactly ONE certificate's info/key out of
    // the DER it's given -- must stay pointed at the LEAF alone, not the leaf+CA chain below
    // (which is only ever assembled on the wire, never re-parsed by this server itself).
    XASSERT(nx_secure_x509_certificate_initialize(
        &g_opcuaServerCertificate, (UCHAR *)_binary_device_certificate_der_start, device_cert_der_len,
        NX_NULL, 0, (UCHAR *)_binary_device_key_der_start, device_key_der_len,
        NX_SECURE_X509_KEY_TYPE_RSA_PKCS1_DER),
        "OPC UA server certificate initialize failed");
    const auto &rsaKey = g_opcuaServerCertificate.nx_secure_x509_private_key.rsa_private_key;
    g_opcuaServerIdentity.certificateDer =
        std::span<const opcua::Byte>((const opcua::Byte *)_binary_device_certificate_der_start, device_cert_der_len);
    // Root CA cert (DER), sent alongside the leaf wherever this server presents its identity over
    // the wire (WriteServerCertificateChain()) so clients (e.g. UAExpert) can build a complete
    // trust chain -- see secure_channel.hpp's ServerIdentity::caCertificateDer comment.
    const USHORT root_ca_der_len = (USHORT)(_binary_root_ca_der_end - _binary_root_ca_der_start);
    g_opcuaServerIdentity.caCertificateDer =
        std::span<const opcua::Byte>((const opcua::Byte *)_binary_root_ca_der_start, root_ca_der_len);
    g_opcuaServerIdentity.privateKey.modulus =
        std::span<const opcua::Byte>(rsaKey.nx_secure_rsa_public_modulus, rsaKey.nx_secure_rsa_public_modulus_length);
    g_opcuaServerIdentity.privateKey.privateExponent = std::span<const opcua::Byte>(
        rsaKey.nx_secure_rsa_private_exponent, rsaKey.nx_secure_rsa_private_exponent_length);
    g_opcuaServerIdentity.privateKey.primeP = std::span<const opcua::Byte>(
        rsaKey.nx_secure_rsa_private_prime_p, rsaKey.nx_secure_rsa_private_prime_p_length);
    g_opcuaServerIdentity.privateKey.primeQ = std::span<const opcua::Byte>(
        rsaKey.nx_secure_rsa_private_prime_q, rsaKey.nx_secure_rsa_private_prime_q_length);
    opcua::SetServerIdentity(g_opcuaServerIdentity);

    void *ptr = nullptr;
    XASSERT(tx_byte_allocate(&app->byte_pool, &ptr, OPCUA_POOL_SIZE, TX_NO_WAIT),
            "OPC UA packet pool allocate failed");
    XASSERT(nx_packet_pool_create(&app->opcua_packet_pool, _C("OPC UA Packet Pool"),
                                  OPCUA_PACKET_SIZE, ptr, OPCUA_POOL_SIZE),
            "OPC UA packet pool create failed");

    XASSERT(tx_byte_allocate(&app->byte_pool, &ptr, OPCUA_THREAD_STACK_SIZE, TX_NO_WAIT),
            "OPC UA TCP server thread stack allocate failed");

    // No stable hostname is available yet at this point in the boot sequence (DHCP hasn't
    // resolved); "opc.tcp://:<port>" (empty host) is valid per Part 6 7.1.2.3 and is what
    // clients connect to by IP/port anyway.
    snprintf(g_endpointUrl, sizeof(g_endpointUrl), "opc.tcp://:%u", static_cast<unsigned>(OPCUA_PORT));

    XASSERT(app->opcua_server.Create(&app->ip_instance, &app->opcua_packet_pool, ptr, OPCUA_THREAD_STACK_SIZE,
                            OPCUA_THREAD_PRIORITY, OPCUA_SESSION_TIMEOUT_SECONDS,
                            &GeneratedOpcUa::AddressSpaceInstance(), g_endpointUrl),
            "OPC UA TCP server create failed");
    XASSERT(app->opcua_server.Start(OPCUA_PORT, opcua::OpcUaTcpServer::MAX_SESSIONS * 2),
            "OPC UA TCP server start failed");

    log_info("OPC UA Server started on %s (SecurityPolicy#Basic256Sha256, SecurityMode Sign)", g_endpointUrl);
}
