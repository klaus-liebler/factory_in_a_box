#pragma once
// Spiegelt jede Logger-Ausgabe (log_info/log_warn/... aus stm32_libs/common_stm32/log.h) als
// "system.LogMessage"-WebSocket-Event (s. ws-protocol/system.json/docs/websocket-protocol.md) an alle
// aktuell verbundenen WebSocket-Clients -- rein additiv zur bestehenden UART-Ausgabe, die
// weiterhin die primaere, garantierte Log-Senke bleibt (s. Klassenkommentar in ws_log_bridge.cpp).

namespace Http {
class WebServer;
}

// Registriert den Log-Sink (log_set_extra_sink(), s. log.h) fuer die Lebensdauer des Programms.
// 'server' muss bereits erzeugt (Create()) sein, aber noch nicht zwingend gestartet -- der Sink
// greift erst auf Broadcast() zu, sobald tatsaechlich WebSocket-Clients verbunden sind.
void WsLogBridge_Install(Http::WebServer *server);
