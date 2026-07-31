#pragma once
#include <cstdint>
#include <string_view>

// BOARD_NAME/FW_VERSION_MAJOR/MINOR/PATCH: generiert (nicht mehr von Hand gepflegt), s.
// builder/Phases/ReadGitStatus.cs. BOARD_NAME kommt aus einem Board-Archiv-Nachschlag (board.json,
// s. builder/BoardStore.cs) statt aus einem #ifdef BOARD_NUCLEO_H563ZI-Umschalter (Fallback:
// konfigurierter Default, s. docs/build-process.md Abschnitt 5); FW_VERSION_* aus
// firmware-version.json (Repo-Root).
#include "generated/firmware_constants.hh"

//Memory Pool Sizes for ThreadX, FileX, and NetXDuo
constexpr uint32_t TX_APP_MEM_POOL_SIZE = 10 * 1024;
constexpr uint32_t FX_APP_MEM_POOL_SIZE = 10 * 1024;
// Die Summe aller tx_byte_allocate(&nx_app_byte_pool, ...)-Aufrufe unten (v.a. NetXDuo-
// Paketpool mit ~80 KB, HTTP-Server-Stack 16 KB fuer RSA/ECDHE-Handshakes, plus die Stacks
// von app_main/modbus_tcp_server/io_thread) liegt bei ca. 130 KB vor Byte-Pool-Verwaltungsoverhead
// -- deutliche Reserve statt auf Kante genau genug.
constexpr uint32_t NX_APP_MEM_POOL_SIZE = 192 * 1024;

