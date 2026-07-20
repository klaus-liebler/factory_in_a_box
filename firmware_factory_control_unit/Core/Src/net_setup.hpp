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

class App;

void net_setup_create(App *app, TX_BYTE_POOL *nx_app_byte_pool);
void net_setup_start(App *app);

// Oeffnet (bzw. re-oeffnet) das FileX-Medium der microSD-Karte auf app->sd_media. Card optional:
// gibt bei fehlender/defekter Karte false zurueck, statt zu blockieren/abzustuerzen -- Aufrufer
// laesst app->sd_media in diesem Fall einfach ungenutzt. Wiederverwendet von net_setup_start()
// (initiales Oeffnen beim Boot) und vom SD/USB-Arbiter (Rueckgabe der Karte an die Firmware nach
// einem USB-MSC-Zugriffsfenster, s. sd_usb_arbiter.hh).
bool sd_media_try_open(App *app);
