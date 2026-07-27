#pragma once
// HTTP-Request-Callback fuer den NetX Duo HTTP(S)-Server -- liefert die unter web/ gebaute
// Single-File-UI (gzip, siehe Core/Src/generated/modbus_ui_page.c) unter "/" aus, sowie
// schlanke Text-/Query-Endpunkte zum Lesen/Schreiben aller Register. Siehe webserver.cpp fuer
// Details zum Antwortformat.

#include "nx_web_http_server.h"

UINT webserver_request_callback(NX_WEB_HTTP_SERVER *server_ptr, UINT request_type,
                                 CHAR *resource, NX_PACKET *packet_ptr);

// Einmaliger I2C-Geraete-Scan aller drei Busse beim Boot -- aufgerufen aus Io::Setup() (nicht per
// HTTP-Anfrage: ein per Browser ausgeloester Scan schlug wiederholt fehl, vermutlich durch
// Netzwerk-Interruptlast waehrend der laufenden Anfrage, s. Commit-Historie zu
// "cross-peripheral I2C/network starvation"; ein Scan direkt beim Hochfahren -- wie das
// urspruengliche main()-Testcode-Vorbild, das zuverlaessig funktionierte -- umgeht das komplett).
// Speichert das Ergebnis in App::Instance().i2c1_scan/i2c2_scan/i2c4_scan, unveraendert
// ausgeliefert ueber /api/system (Core/Src/webserver.cpp fill_system_info()).
void PerformBootI2cScans();
