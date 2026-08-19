#include "opcua/security_crypto.hpp"

#include <cstring>

#include "nx_crypto.h"
#include "nx_crypto_huge_number.h"
#include "nx_crypto_rsa.h"
#include "nx_crypto_sha1.h"
#include "nx_crypto_sha2.h"

namespace opcua::security {

namespace {

// RFC 3447 9.2 Note 1 -- the DER encoding of DigestInfo's AlgorithmIdentifier for SHA-256,
// prepended to the raw 32-byte hash before RSA signing/verification. Same 19-byte constant NetX
// Secure's own TLS CertificateVerify code uses (_NX_SECURE_OID_SHA256,
// nx_secure_tls_send_certificate_verify.c) -- reproduced here since that file doesn't expose it.
constexpr Byte SHA256_DIGESTINFO_PREFIX[] = {
    0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01, 0x05,
    0x00, 0x04, 0x20,
};
constexpr size_t DIGESTINFO_SIZE = sizeof(SHA256_DIGESTINFO_PREFIX) + SHA256_DIGEST_SIZE; // 51

// Builds the PKCS#1 v1.5 encoded message EM (RFC 3447 9.2): 0x00 0x01 PS 0x00 DigestInfo(hash),
// PS = 0xFF padding, total length == modulusLen. Shared by sign (build EM, then RSA-private-op
// it) and verify (RSA-public-op the signature, then compare against a freshly built EM).
bool BuildPkcs1Sha256Em(std::span<const Byte> message, size_t modulusLen, Byte *emOut) {
    if(modulusLen > MAX_RSA_MODULUS_BYTES || modulusLen < DIGESTINFO_SIZE + 11) return false;
    Byte hash[SHA256_DIGEST_SIZE];
    if(!Sha256(message, hash)) return false;

    size_t psLen = modulusLen - 3 - DIGESTINFO_SIZE;
    emOut[0] = 0x00;
    emOut[1] = 0x01;
    std::memset(emOut + 2, 0xFF, psLen);
    emOut[2 + psLen] = 0x00;
    Byte *t = emOut + 3 + psLen;
    std::memcpy(t, SHA256_DIGESTINFO_PREFIX, sizeof(SHA256_DIGESTINFO_PREFIX));
    std::memcpy(t + sizeof(SHA256_DIGESTINFO_PREFIX), hash, SHA256_DIGEST_SIZE);
    return true;
}

// RFC 3447 B.2.1. seed is at most MAX_RSA_MODULUS_BYTES bytes in both call sites below (the
// 20-byte OAEP seed, or the masked-DB block which is at most MAX_RSA_MODULUS_BYTES -
// SHA1_DIGEST_SIZE - 1 bytes), so a MAX_RSA_MODULUS_BYTES + 4 scratch buffer covers both.
bool Mgf1Sha1(std::span<const Byte> seed, std::span<Byte> maskOut) {
    if(seed.size() > MAX_RSA_MODULUS_BYTES) return false;
    Byte combined[MAX_RSA_MODULUS_BYTES + 4];
    std::memcpy(combined, seed.data(), seed.size());
    size_t filled = 0;
    UInt32 counter = 0;
    while(filled < maskOut.size()) {
        combined[seed.size() + 0] = (Byte)(counter >> 24);
        combined[seed.size() + 1] = (Byte)(counter >> 16);
        combined[seed.size() + 2] = (Byte)(counter >> 8);
        combined[seed.size() + 3] = (Byte)counter;
        Byte digest[SHA1_DIGEST_SIZE];
        if(!Sha1(std::span<const Byte>(combined, seed.size() + 4), digest)) return false;
        size_t n = std::min(SHA1_DIGEST_SIZE, maskOut.size() - filled);
        std::memcpy(maskOut.data() + filled, digest, n);
        filled += n;
        counter++;
    }
    return true;
}

} // namespace

bool Sha1(std::span<const Byte> data, Byte out[SHA1_DIGEST_SIZE]) {
    NX_CRYPTO_SHA1 ctx{};
    if(_nx_crypto_sha1_initialize(&ctx, NX_CRYPTO_HASH_SHA1) != NX_CRYPTO_SUCCESS) return false;
    if(data.size() > 0 &&
       _nx_crypto_sha1_update(&ctx, const_cast<UCHAR *>(data.data()), (UINT)data.size()) != NX_CRYPTO_SUCCESS)
        return false;
    return _nx_crypto_sha1_digest_calculate(&ctx, out, NX_CRYPTO_HASH_SHA1) == NX_CRYPTO_SUCCESS;
}

bool Sha256(std::span<const Byte> data, Byte out[SHA256_DIGEST_SIZE]) {
    NX_CRYPTO_SHA256 ctx{};
    if(_nx_crypto_sha256_initialize(&ctx, NX_CRYPTO_HASH_SHA256) != NX_CRYPTO_SUCCESS) return false;
    if(data.size() > 0 &&
       _nx_crypto_sha256_update(&ctx, const_cast<UCHAR *>(data.data()), (UINT)data.size()) != NX_CRYPTO_SUCCESS)
        return false;
    return _nx_crypto_sha256_digest_calculate(&ctx, out, NX_CRYPTO_HASH_SHA256) == NX_CRYPTO_SUCCESS;
}

bool HmacSha256(std::span<const Byte> key, std::span<const Byte> data, Byte out[SHA256_DIGEST_SIZE]) {
    constexpr size_t BLOCK = 64;
    Byte keyBlock[BLOCK] = {};
    if(key.size() <= BLOCK) {
        std::memcpy(keyBlock, key.data(), key.size());
    } else if(!Sha256(key, keyBlock)) {
        return false; // remaining bytes stay zero -- correct per RFC 2104 (hashed key < BLOCK)
    }

    Byte ipad[BLOCK], opad[BLOCK];
    for(size_t i = 0; i < BLOCK; i++) {
        ipad[i] = (Byte)(keyBlock[i] ^ 0x36);
        opad[i] = (Byte)(keyBlock[i] ^ 0x5c);
    }

    Byte inner[SHA256_DIGEST_SIZE];
    NX_CRYPTO_SHA256 ctx{};
    if(_nx_crypto_sha256_initialize(&ctx, NX_CRYPTO_HASH_SHA256) != NX_CRYPTO_SUCCESS) return false;
    if(_nx_crypto_sha256_update(&ctx, ipad, BLOCK) != NX_CRYPTO_SUCCESS) return false;
    if(data.size() > 0 &&
       _nx_crypto_sha256_update(&ctx, const_cast<UCHAR *>(data.data()), (UINT)data.size()) != NX_CRYPTO_SUCCESS)
        return false;
    if(_nx_crypto_sha256_digest_calculate(&ctx, inner, NX_CRYPTO_HASH_SHA256) != NX_CRYPTO_SUCCESS) return false;

    if(_nx_crypto_sha256_initialize(&ctx, NX_CRYPTO_HASH_SHA256) != NX_CRYPTO_SUCCESS) return false;
    if(_nx_crypto_sha256_update(&ctx, opad, BLOCK) != NX_CRYPTO_SUCCESS) return false;
    if(_nx_crypto_sha256_update(&ctx, inner, SHA256_DIGEST_SIZE) != NX_CRYPTO_SUCCESS) return false;
    return _nx_crypto_sha256_digest_calculate(&ctx, out, NX_CRYPTO_HASH_SHA256) == NX_CRYPTO_SUCCESS;
}

bool PSha256(std::span<const Byte> secret, std::span<const Byte> seed, std::span<Byte> out) {
    // Seeds used by this server are always small (nonce concatenations, <= ~128 bytes) --
    // MAX_SEED bounds the scratch buffer without dynamic allocation.
    constexpr size_t MAX_SEED = 256;
    if(seed.size() > MAX_SEED) return false;

    Byte a[SHA256_DIGEST_SIZE];
    if(!HmacSha256(secret, seed, a)) return false; // A(1) = HMAC(secret, seed)

    Byte combined[SHA256_DIGEST_SIZE + MAX_SEED];
    size_t filled = 0;
    while(filled < out.size()) {
        std::memcpy(combined, a, SHA256_DIGEST_SIZE);
        std::memcpy(combined + SHA256_DIGEST_SIZE, seed.data(), seed.size());
        Byte chunk[SHA256_DIGEST_SIZE];
        if(!HmacSha256(secret, std::span<const Byte>(combined, SHA256_DIGEST_SIZE + seed.size()), chunk))
            return false;
        size_t n = std::min(SHA256_DIGEST_SIZE, out.size() - filled);
        std::memcpy(out.data() + filled, chunk, n);
        filled += n;
        if(filled < out.size()) {
            if(!HmacSha256(secret, std::span<const Byte>(a, SHA256_DIGEST_SIZE), a)) return false; // A(i+1)
        }
    }
    return true;
}

bool RsaSha256Sign(const RsaPrivateKey &key, std::span<const Byte> message, std::span<Byte> signatureOut) {
    size_t modulusLen = key.modulus.size();
    if(signatureOut.size() != modulusLen || modulusLen > MAX_RSA_MODULUS_BYTES) return false;

    Byte em[MAX_RSA_MODULUS_BYTES];
    if(!BuildPkcs1Sha256Em(message, modulusLen, em)) return false;

    USHORT scratch[NX_CRYPTO_RSA_SCRATCH_BUFFER_SIZE];
    UINT status = _nx_crypto_rsa_operation(
        key.privateExponent.data(), (UINT)key.privateExponent.size(),
        key.modulus.data(), (UINT)modulusLen,
        key.primeP.data(), (UINT)key.primeP.size(),
        const_cast<UCHAR *>(key.primeQ.data()), (UINT)key.primeQ.size(),
        em, (UINT)modulusLen, signatureOut.data(),
        scratch, sizeof(scratch));
    return status == NX_CRYPTO_SUCCESS;
}

bool RsaSha256Verify(const RsaPublicKey &key, std::span<const Byte> message, std::span<const Byte> signature) {
    size_t modulusLen = key.modulus.size();
    if(signature.size() != modulusLen || modulusLen > MAX_RSA_MODULUS_BYTES) return false;

    Byte expectedEm[MAX_RSA_MODULUS_BYTES];
    if(!BuildPkcs1Sha256Em(message, modulusLen, expectedEm)) return false;

    Byte decrypted[MAX_RSA_MODULUS_BYTES];
    USHORT scratch[NX_CRYPTO_RSA_SCRATCH_BUFFER_SIZE];
    UINT status = _nx_crypto_rsa_operation(
        key.exponent.data(), (UINT)key.exponent.size(),
        key.modulus.data(), (UINT)modulusLen,
        NX_NULL, 0, NX_NULL, 0,
        const_cast<UCHAR *>(signature.data()), (UINT)modulusLen, decrypted,
        scratch, sizeof(scratch));
    if(status != NX_CRYPTO_SUCCESS) return false;
    return std::memcmp(decrypted, expectedEm, modulusLen) == 0;
}

bool RsaOaepEncrypt(const RsaPublicKey &key, std::span<const Byte> plaintext, std::span<Byte> cipherOut) {
    size_t k = key.modulus.size();
    if(k > MAX_RSA_MODULUS_BYTES || cipherOut.size() != k || k < OAEP_SHA1_OVERHEAD_BYTES) return false;
    if(plaintext.size() > RsaOaepMaxPlaintext(k)) return false;

    constexpr size_t hLen = 20;
    Byte lHash[hLen];
    if(!Sha1(std::span<const Byte>{}, lHash)) return false; // SHA1("") -- empty label

    size_t dbLen = k - hLen - 1;
    size_t psLen = dbLen - hLen - 1 - plaintext.size();
    Byte db[MAX_RSA_MODULUS_BYTES];
    std::memcpy(db, lHash, hLen);
    std::memset(db + hLen, 0, psLen);
    db[hLen + psLen] = 0x01;
    std::memcpy(db + hLen + psLen + 1, plaintext.data(), plaintext.size());

    Byte seed[hLen];
    if(_nx_crypto_huge_number_rbg(hLen * 8, seed) != NX_CRYPTO_SUCCESS) return false;

    Byte dbMask[MAX_RSA_MODULUS_BYTES];
    if(!Mgf1Sha1(std::span<const Byte>(seed, hLen), std::span<Byte>(dbMask, dbLen))) return false;
    Byte maskedDb[MAX_RSA_MODULUS_BYTES];
    for(size_t i = 0; i < dbLen; i++) maskedDb[i] = (Byte)(db[i] ^ dbMask[i]);

    Byte seedMask[hLen];
    if(!Mgf1Sha1(std::span<const Byte>(maskedDb, dbLen), std::span<Byte>(seedMask, hLen))) return false;
    Byte maskedSeed[hLen];
    for(size_t i = 0; i < hLen; i++) maskedSeed[i] = (Byte)(seed[i] ^ seedMask[i]);

    Byte em[MAX_RSA_MODULUS_BYTES];
    em[0] = 0x00;
    std::memcpy(em + 1, maskedSeed, hLen);
    std::memcpy(em + 1 + hLen, maskedDb, dbLen);

    USHORT scratch[NX_CRYPTO_RSA_SCRATCH_BUFFER_SIZE];
    UINT status = _nx_crypto_rsa_operation(
        key.exponent.data(), (UINT)key.exponent.size(),
        key.modulus.data(), (UINT)k,
        NX_NULL, 0, NX_NULL, 0,
        em, (UINT)k, cipherOut.data(),
        scratch, sizeof(scratch));
    return status == NX_CRYPTO_SUCCESS;
}

bool RsaOaepDecrypt(const RsaPrivateKey &key, std::span<const Byte> ciphertext, std::span<Byte> plaintextOut,
                    size_t &plaintextLenOut) {
    size_t k = key.modulus.size();
    constexpr size_t hLen = 20;
    if(k > MAX_RSA_MODULUS_BYTES || ciphertext.size() != k || k < OAEP_SHA1_OVERHEAD_BYTES) return false;

    Byte em[MAX_RSA_MODULUS_BYTES];
    USHORT scratch[NX_CRYPTO_RSA_SCRATCH_BUFFER_SIZE];
    UINT status = _nx_crypto_rsa_operation(
        key.privateExponent.data(), (UINT)key.privateExponent.size(),
        key.modulus.data(), (UINT)k,
        key.primeP.data(), (UINT)key.primeP.size(),
        const_cast<UCHAR *>(key.primeQ.data()), (UINT)key.primeQ.size(),
        const_cast<UCHAR *>(ciphertext.data()), (UINT)k, em,
        scratch, sizeof(scratch));
    if(status != NX_CRYPTO_SUCCESS) return false;
    if(em[0] != 0x00) return false; // malformed -- see note below on not distinguishing WHICH check failed

    size_t dbLen = k - hLen - 1;
    Byte maskedSeed[hLen];
    std::memcpy(maskedSeed, em + 1, hLen);
    Byte maskedDb[MAX_RSA_MODULUS_BYTES];
    std::memcpy(maskedDb, em + 1 + hLen, dbLen);

    Byte seedMask[hLen];
    if(!Mgf1Sha1(std::span<const Byte>(maskedDb, dbLen), std::span<Byte>(seedMask, hLen))) return false;
    Byte seed[hLen];
    for(size_t i = 0; i < hLen; i++) seed[i] = (Byte)(maskedSeed[i] ^ seedMask[i]);

    Byte dbMask[MAX_RSA_MODULUS_BYTES];
    if(!Mgf1Sha1(std::span<const Byte>(seed, hLen), std::span<Byte>(dbMask, dbLen))) return false;
    Byte db[MAX_RSA_MODULUS_BYTES];
    for(size_t i = 0; i < dbLen; i++) db[i] = (Byte)(maskedDb[i] ^ dbMask[i]);

    Byte lHash[hLen];
    if(!Sha1(std::span<const Byte>{}, lHash)) return false;
    // Deliberately a single generic failure path from here on (no distinct error per check) --
    // RFC 3447 7.1.2 explicitly warns against leaking WHICH validation step failed (a padding-
    // oracle risk); this server doesn't attempt full constant-time countermeasures beyond that,
    // consistent with its stated minimal-server threat model (see plan/secure_channel.hpp).
    if(std::memcmp(db, lHash, hLen) != 0) return false;

    size_t i = hLen;
    while(i < dbLen && db[i] == 0x00) i++;
    if(i >= dbLen || db[i] != 0x01) return false;
    size_t msgStart = i + 1;
    size_t msgLen = dbLen - msgStart;
    if(msgLen > plaintextOut.size()) return false;
    std::memcpy(plaintextOut.data(), db + msgStart, msgLen);
    plaintextLenOut = msgLen;
    return true;
}

bool RsaOaepEncryptBlocks(const RsaPublicKey &key, std::span<const Byte> plaintext, std::span<Byte> cipherOut) {
    size_t k = key.modulus.size();
    size_t blockPlain = RsaOaepMaxPlaintext(k);
    if(blockPlain == 0 || plaintext.size() % blockPlain != 0) return false;
    size_t numBlocks = plaintext.size() / blockPlain;
    if(cipherOut.size() != numBlocks * k) return false;

    for(size_t i = 0; i < numBlocks; i++) {
        if(!RsaOaepEncrypt(key, plaintext.subspan(i * blockPlain, blockPlain), cipherOut.subspan(i * k, k)))
            return false;
    }
    return true;
}

bool RsaOaepDecryptBlocks(const RsaPrivateKey &key, std::span<const Byte> ciphertext, std::span<Byte> plaintextOut) {
    size_t k = key.modulus.size();
    size_t blockPlain = RsaOaepMaxPlaintext(k);
    if(k == 0 || ciphertext.size() == 0 || ciphertext.size() % k != 0) return false;
    size_t numBlocks = ciphertext.size() / k;
    if(plaintextOut.size() != numBlocks * blockPlain) return false;

    for(size_t i = 0; i < numBlocks; i++) {
        size_t decodedLen = 0;
        if(!RsaOaepDecrypt(key, ciphertext.subspan(i * k, k), plaintextOut.subspan(i * blockPlain, blockPlain),
                           decodedLen))
            return false;
        if(decodedLen != blockPlain) return false; // see header comment -- not a partial-block format
    }
    return true;
}

} // namespace opcua::security
