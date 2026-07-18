#pragma once
// Generiert von tools/provision-certificate.mjs -- nicht von Hand editieren. Neu erzeugen
// per "node tools/provision-certificate.mjs" (liest/erzeugt das Zertifikat fuer das aktuell
// am ST-Link angeschlossene Board). Nur eine kurze String-Konstante -- anders als
// Zertifikat/privater Schluessel (assets/device_certificate.der/device_key.der) kein
// Binary, deshalb hier als gewoehnliche C++-Konstante statt per objcopy/Linker-Section.
constexpr char DEVICE_HOSTNAME[] = "factory-box-303537";
