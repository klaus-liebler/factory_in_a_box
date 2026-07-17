#pragma once
// HTTP-Request-Callback fuer den NetX Duo HTTP(S)-Server -- liefert die unter web/ gebaute
// Single-File-UI (gzip, siehe Core/Src/generated/modbus_ui_page.c) unter "/" aus, sowie
// schlanke Text-/Query-Endpunkte zum Lesen/Schreiben aller Register. Siehe webserver.cpp fuer
// Details zum Antwortformat.

#include "nx_web_http_server.h"

UINT webserver_request_callback(NX_WEB_HTTP_SERVER *server_ptr, UINT request_type,
                                 CHAR *resource, NX_PACKET *packet_ptr);
