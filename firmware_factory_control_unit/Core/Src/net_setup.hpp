#pragma once
// NetX Duo IP/TCP/UDP/HTTP(S)/mDNS/DHCP-Bring-up. Zweigeteilt, weil einige NetX-Aufrufe
// (nx_secure_x509_certificate_initialize(), nx_tcp_server_socket_listen() ueber
// ModbusTcpServer::initialize()) NX_THREADS_ONLY_CALLER_CHECKING-beschraenkt sind und daher
// erst aus einem laufenden Thread heraus aufgerufen werden duerfen:
//   - net_setup_create(): laeuft in tx_application_define() (kein Thread aktiv), erzeugt nur
//     Objekte/Pools/Server-Handles, ohne sie zu starten.
//   - net_setup_start(): laeuft in app_main_thread_entry() (App Main Thread, der einzige mit
//     TX_AUTO_START), startet FileX/TLS/HTTP/DHCP tatsaechlich.
#include "tx_api.h"

constexpr ULONG NX_APP_THREAD_STACK_SIZE = 8 * 1024;
constexpr UINT NX_APP_THREAD_PRIORITY = 10;

void net_setup_create(TX_BYTE_POOL *nx_app_byte_pool);
void net_setup_start();
