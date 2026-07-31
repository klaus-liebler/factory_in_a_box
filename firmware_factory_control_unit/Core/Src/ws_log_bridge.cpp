// Siehe ws_log_bridge.hpp. Der Sink wird SYNCHRON und mit gehaltenem Log-Mutex aus log_log()
// heraus aufgerufen (s. log.h log_ExtraSinkFn-Kommentar) -- deshalb ruft er ausschliesslich
// Http::WebServer::Broadcast() auf, das durchgehend NX_NO_WAIT nutzt und nie blockiert (s.
// dortiger Kommentar), und NIEMALS log_*() selbst (Deadlock auf dem Log-Mutex).
//
// timestampMs kommt aus einem zweiten HAL_GetTick()-Aufruf HIER (nicht aus log.c uebernommen) --
// der generische log_ExtraSinkFn-Hook (log.h) reicht bewusst nur Level+Text durch, keinen
// Zeitstempel, um die Log-Bibliothek nicht mit Wire-Format-Details dieses einen Projekts zu
// belasten. Die Abweichung zum tatsaechlichen Log-Zeitpunkt liegt im Mikrosekundenbereich
// (derselbe log_log()-Aufruf, nur wenige Anweisungen spaeter) und ist fuer Debug-Zwecke
// unerheblich.
#include "ws_log_bridge.hpp"

#include "http_websocket_server.hpp"
#include "generated/ws_protocol.hh"
#include "log.h"
#include "main.h"

static Http::WebServer *g_ws_log_bridge_server = nullptr;

static void WsLogBridgeSink(int level, char const *text, size_t len) {
    if (!g_ws_log_bridge_server) return;

    WsProtocol::system::LogMessage::Payload payload{};
    // level (int, s. enum in log.h: LOG_TRACE..LOG_FATAL) und LogLevel (generiert aus
    // ws-protocol/system.json) sind bewusst zwei getrennte, aber wertgleiche Aufzaehlungen --
    // log.h soll kein Wissen ueber das WebSocket-Wire-Format haben (s. log_ExtraSinkFn-Kommentar).
    payload.level = (WsProtocol::system::LogMessage::LogLevel)level;
    payload.timestampMs = HAL_GetTick();
    payload.text = text;
    payload.textLength = len;

    uint8_t buffer[300];
    size_t encoded = WsProtocol::system::LogMessage::Encode(payload, buffer, sizeof(buffer));
    if (encoded > 0) {
        g_ws_log_bridge_server->Broadcast(buffer, encoded);
    }
}

void WsLogBridge_Install(Http::WebServer *server) {
    g_ws_log_bridge_server = server;
    log_set_extra_sink(&WsLogBridgeSink);
}
