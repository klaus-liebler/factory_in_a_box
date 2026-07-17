#pragma once
// Gemeinsamer Anwendungszustand -- von allen Modulen (net_setup.cpp, webserver.cpp,
// io_thread.cpp und den davon aufgerufenen *_control.cpp-Dateien) referenziert. Definiert
// (nicht nur deklariert) wird g_app_state einmalig in app.cpp.

#include "nx_api.h"
#include "fx_api.h"
#include "tx_api.h"
#include "nxd_dhcp_client.h"
#include "nx_web_http_server.h"
#include "nxd_mdns.h"

#include "main.h" // Error_Handler(), von XASSERT unten verwendet
#include "modbus_tcp_server.hpp"

typedef struct {
    NX_IP ip_instance;
    NX_PACKET_POOL packet_pool;
    NX_DHCP dhcp_client;
    NX_WEB_HTTP_SERVER http_server;
    NX_MDNS mdns;
    ModbusTcpServer* modbus_server = nullptr;

    FX_MEDIA sd_media;

    ULONG ip_address;
    ULONG net_mask;

    TX_THREAD app_main_thread;
    TX_THREAD modbus_server_thread;
    TX_THREAD io_thread;
} AppState;

extern AppState g_app_state;

// printf-Format samt passender Argumentliste fuer eine NetX-ULONG-IPv4-Adresse
// (Network-Byte-Order-Oktette aus dem 32-Bit-Wert maskiert/geshiftet), z.B.:
//   log_info("IP Address: " IP_ADDR_FMT, IP_ADDR_FMT_ARGS(g_app_state.ip_address));
#define IP_ADDR_FMT "%lu.%lu.%lu.%lu"
#define IP_ADDR_FMT_ARGS(addr) \
    (((addr) >> 24) & 0xff), (((addr) >> 16) & 0xff), (((addr) >> 8) & 0xff), ((addr) & 0xff)

#define NX_CHAR_LITERAL(str) const_cast<CHAR *>(str)

// Success-Makro fuer NX_SUCCESS-liefernde Aufrufe -- loggt Kontext (Datei/Zeile/Statuscode) und
// haelt in Error_Handler() an. Von allen Modulen gemeinsam genutzt.
#define XASSERT(status_expr, message_on_fail) \
do { \
    UINT assure_status_ = (status_expr); \
    if (assure_status_ != NX_SUCCESS) { \
        log_error("Error %s: %s:%d, status: 0x%x", message_on_fail, __FILE__, __LINE__, assure_status_); \
        Error_Handler(); \
    } \
} while (0)
