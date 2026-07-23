#pragma once
#include <cstdint>
#include <string_view>

//Name & Version
#ifdef BOARD_NUCLEO_H563ZI
constexpr std::string_view BOARD_NAME = "Factory Control Unit [TEST RIG: Nucleo-H563ZI] (STM32H563)";
#else
constexpr std::string_view BOARD_NAME = "Factory Control Unit (STM32H563)";
#endif
constexpr uint16_t FW_VERSION_MAJOR = 0;
constexpr uint16_t FW_VERSION_MINOR = 1;
constexpr uint16_t FW_VERSION_PATCH = 0;

//Memory Pool Sizes for ThreadX, FileX, and NetXDuo
constexpr uint32_t TX_APP_MEM_POOL_SIZE = 10 * 1024;
constexpr uint32_t FX_APP_MEM_POOL_SIZE = 10 * 1024;
// Die Summe aller tx_byte_allocate(&nx_app_byte_pool, ...)-Aufrufe unten (v.a. NetXDuo-
// Paketpool mit ~80 KB, HTTP-Server-Stack 16 KB fuer RSA/ECDHE-Handshakes, plus die Stacks
// von app_main/modbus_server/io_thread) liegt bei ca. 130 KB vor Byte-Pool-Verwaltungsoverhead
// -- deutliche Reserve statt auf Kante genau genug.
constexpr uint32_t NX_APP_MEM_POOL_SIZE = 192 * 1024;

