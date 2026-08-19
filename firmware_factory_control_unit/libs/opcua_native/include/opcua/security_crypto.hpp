#pragma once
// Cryptographic primitives for SecurityPolicy#Basic256Sha256 (Part 6 6.7.3/6.7.5): RSASSA-
// PKCS1-v1_5 signing/verification with SHA-256, HMAC-SHA256, and the P_SHA256 key-derivation
// function (the same HMAC-SHA256-based PRF construction as TLS's PRF, RFC 5246 5.). Built
// directly on NetX Crypto's low-level primitives (nx_crypto_rsa.h's _nx_crypto_rsa_operation,
// nx_crypto_sha2.h's raw SHA-256) rather than the NX_CRYPTO_METHOD-wrapped versions TLS itself
// uses -- those are tightly coupled to TLS session/metadata plumbing this server has no use for.
//
// Fixed to RSA-2048 (this project's chosen device-certificate key size, see
// firmware_builder_common's Certificates.KeyAlgorithm) -- MAX_RSA_MODULUS_BYTES below is sized
// exactly for that, no dynamic allocation.
#include <cstddef>
#include <span>

#include "opcua/types.hpp"

namespace opcua::security {

constexpr size_t SHA256_DIGEST_SIZE = 32;
constexpr size_t SHA1_DIGEST_SIZE = 20;
constexpr size_t MAX_RSA_MODULUS_BYTES = 256; // RSA-2048

bool Sha256(std::span<const Byte> data, Byte out[SHA256_DIGEST_SIZE]);
// Only used internally for OAEP (RsaOaepEncrypt/Decrypt's MGF1) -- exposed too since
// secure_channel.cpp separately needs it for the CertificateThumbprint field (Part 6 7.10, a
// SHA-1 digest of the peer's DER certificate), the one other place this server touches SHA-1.
bool Sha1(std::span<const Byte> data, Byte out[SHA1_DIGEST_SIZE]);
bool HmacSha256(std::span<const Byte> key, std::span<const Byte> data, Byte out[SHA256_DIGEST_SIZE]);

// RFC 5246 5. / OPC UA Part 6 6.7.5 -- expands (secret, seed) into exactly out.size() bytes.
bool PSha256(std::span<const Byte> secret, std::span<const Byte> seed, std::span<Byte> out);

// Public key straight from an X.509 certificate's SubjectPublicKeyInfo (big-endian, as NetX
// Secure's own cert parser already exposes it -- see NX_SECURE_X509_CERT's
// nx_secure_rsa_public_modulus/nx_secure_rsa_public_exponent, secure_channel.cpp).
struct RsaPublicKey {
    std::span<const Byte> modulus;
    std::span<const Byte> exponent;
};

// Private key as parsed by NetX Secure's _nx_secure_x509_pkcs1_rsa_private_key_parse() (the same
// PKCS#1 DER format the device's embedded key blob already uses, see net_setup.cpp).
struct RsaPrivateKey {
    std::span<const Byte> modulus;
    std::span<const Byte> privateExponent;
    std::span<const Byte> primeP;
    std::span<const Byte> primeQ;
};

// RSASSA-PKCS1-v1_5 with SHA-256 (the "http://www.w3.org/2001/04/xmldsig-more#rsa-sha256"
// algorithm Basic256Sha256 uses for both the asymmetric OPN signature and CreateSession's ServerSignature
// proof-of-possession). "signatureOut" must be exactly key.modulus.size() bytes -- callers know
// this size upfront (it's the server's own certificate's modulus, fixed at startup).
bool RsaSha256Sign(const RsaPrivateKey &key, std::span<const Byte> message, std::span<Byte> signatureOut);
bool RsaSha256Verify(const RsaPublicKey &key, std::span<const Byte> message, std::span<const Byte> signature);

// RSAES-OAEP (RFC 3447 7.1) with SHA-1 for both the label hash and MGF1 -- Basic256Sha256's
// AsymmetricEncryptionAlgorithm (http://www.w3.org/2001/04/xmlenc#rsa-oaep), empty label. Per
// Part 6 6.7.4, OpenSecureChannel messages are ALWAYS both signed and encrypted with this
// whenever SecurityPolicy != None -- independent of the negotiated MessageSecurityMode (Sign
// only affects later symmetric MSG traffic, not the OPN handshake itself).
constexpr size_t OAEP_SHA1_OVERHEAD_BYTES = 2 * 20 + 2; // 2*hLen + 2
constexpr size_t RsaOaepMaxPlaintext(size_t modulusLen) { return modulusLen - OAEP_SHA1_OVERHEAD_BYTES; }

// "cipherOut" must be exactly key.modulus.size() bytes.
bool RsaOaepEncrypt(const RsaPublicKey &key, std::span<const Byte> plaintext, std::span<Byte> cipherOut);
// "ciphertext" must be exactly key.modulus.size() bytes; "plaintextOut" must be at least
// RsaOaepMaxPlaintext(key.modulus.size()) bytes. plaintextLenOut receives the actual (variable)
// decoded length.
bool RsaOaepDecrypt(const RsaPrivateKey &key, std::span<const Byte> ciphertext, std::span<Byte> plaintextOut,
                    size_t &plaintextLenOut);

// Multi-block form (Part 6 6.7.2): a whole OPN message body is padded by the CALLER to an exact
// multiple of RsaOaepMaxPlaintext(key.modulus.size()) ("PlainTextBlockSize" in the spec), then
// each block is OAEP-encrypted independently and the ciphertext blocks concatenated --
// "cipherOut" must be exactly (plaintext.size() / RsaOaepMaxPlaintext(...)) * key.modulus.size()
// bytes, and plaintext.size() must already be an exact multiple of that block size.
bool RsaOaepEncryptBlocks(const RsaPublicKey &key, std::span<const Byte> plaintext, std::span<Byte> cipherOut);
// Inverse: "ciphertext" must be a non-zero exact multiple of key.modulus.size(). Every block must
// decode to EXACTLY RsaOaepMaxPlaintext(key.modulus.size()) plaintext bytes -- a peer that built
// its blocks correctly always produces that (see RsaOaepEncryptBlocks); anything else is treated
// as a protocol/security violation, not silently accepted with a shorter last block.
// "plaintextOut" must be exactly (ciphertext.size()/key.modulus.size()) * RsaOaepMaxPlaintext(...)
// bytes.
bool RsaOaepDecryptBlocks(const RsaPrivateKey &key, std::span<const Byte> ciphertext, std::span<Byte> plaintextOut);

} // namespace opcua::security
