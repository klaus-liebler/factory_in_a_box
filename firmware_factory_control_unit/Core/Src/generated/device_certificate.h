#pragma once
// Generiert von tools/provision-certificate.mjs -- nicht von Hand editieren. Neu erzeugen
// per "node tools/provision-certificate.mjs" (liest/erzeugt das Zertifikat fuer das aktuell
// am ST-Link angeschlossene Board).
#include <stddef.h>

extern const unsigned char DEVICE_CERT_DER[];
extern const size_t DEVICE_CERT_DER_LEN;
extern const unsigned char DEVICE_KEY_DER[];
extern const size_t DEVICE_KEY_DER_LEN;
extern const char DEVICE_HOSTNAME[];
