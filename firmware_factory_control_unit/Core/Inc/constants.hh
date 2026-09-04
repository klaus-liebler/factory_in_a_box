#pragma once
#include <cstdint>
#include <string_view>

// BOARD_NAME/FW_VERSION_MAJOR/MINOR/PATCH: generiert (nicht mehr von Hand gepflegt), s.
// builder/Phases/ReadGitStatus.cs. BOARD_NAME kommt aus einem Board-Archiv-Nachschlag
// (board_info.json, s. builder/BoardStore.cs) statt aus einem #ifdef BOARD_NUCLEO_H563ZI-Umschalter (Fallback:
// konfigurierter Default, s. docs/build-process.md Abschnitt 5); FW_VERSION_* aus
// firmware-version.json (Repo-Root).
#include "generated/firmware_constants.hh"

//Memory Pool Sizes for ThreadX, FileX, and NetXDuo
constexpr uint32_t TX_APP_MEM_POOL_SIZE = 10 * 1024;
constexpr uint32_t FX_APP_MEM_POOL_SIZE = 10 * 1024;
// Die Summe aller tx_byte_allocate(&nx_app_byte_pool, ...)-Aufrufe unten (v.a. NetXDuo-
// Paketpool mit ~80 KB, HTTP-Server-Stack 16 KB fuer RSA/ECDHE-Handshakes, plus die Stacks
// von app_main/modbus_tcp_server/io_thread) liegt bei ca. 130 KB vor Byte-Pool-Verwaltungsoverhead.
// Von 192 KB auf 256 KB angehoben, als net_setup.cpp's SERVER_POOL_SIZE (HTTP/WebSocket-
// Paket-Pool) von 8 auf 32 Pakete (+36 KB) vergroessert wurde -- mit nur 192 KB reichte die
// verbleibende Reserve nicht mehr fuer opcua_setup.cpp's spaeteren 24-KB-Thread-Stack-Alloc
// ("OPC UA TCP server thread stack allocate failed", live beobachtet 2026-08-19). RAM hat nach
// wie vor reichlich Platz (STM32H563: 640 KiB gesamt), daher hier wieder grosszuegige statt
// exakt bemessene Reserve.
constexpr uint32_t NX_APP_MEM_POOL_SIZE = 256 * 1024;

