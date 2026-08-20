#pragma once
// Routen fuer den HTTPS/WebSocket-Server (s. http_websocket_server.hpp) -- liefert die unter
// web/ gebaute, minifizierte und Brotli-komprimierte Single-File-UI (per objcopy ins Flash
// einkompiliert, siehe assets/index.html.br und CMakeLists.txt BINARY_ASSETS) fuer JEDEN Pfad
// AUSSER "/api/*" aus (SPA-Fallback fuer den History-API-Router der UI, s.
// web/src/shell/router.ts), schlanke Endpunkte zum Lesen/Schreiben aller Register und zum
// Abfragen von System-/I2C-Bus-Informationen, sowie einen WebSocket-Endpunkt unter "/ws"
// (s. Klassenkommentar in http_websocket_server.hpp -- Anwendungsprotokoll darueber ist noch
// nicht festgelegt, aktuell nur ein Echo zur Verifikation der Transportschicht).
#include "http_websocket_server.hpp"

void webserver_register_routes(Http::WebServer &server);

// Einmaliger I2C-Geraete-Scan aller drei Busse beim Boot -- aufgerufen aus Io::Setup() (nicht per
// HTTP-Anfrage: ein per Browser ausgeloester Scan schlug wiederholt fehl, vermutlich durch
// Netzwerk-Interruptlast waehrend der laufenden Anfrage, s. Commit-Historie zu
// "cross-peripheral I2C/network starvation"; ein Scan direkt beim Hochfahren -- wie das
// urspruengliche main()-Testcode-Vorbild, das zuverlaessig funktionierte -- umgeht das komplett).
// Speichert das Ergebnis in App::Instance().i2c1_scan/i2c2_scan/i2c4_scan, unveraendert
// ausgeliefert ueber /api/system (Core/Src/webserver.cpp fill_system_info()).
void PerformBootI2cScans();

// Sendet, falls Teach-Modus aktiv ist und seit dem letzten Aufruf genug Zeit vergangen ist
// (s. Definition in webserver.cpp), ein roarm.PoseFeedback-Event an alle verbundenen WebSocket-
// Clients. Aufgerufen aus Io::processMembers() (io.cpp) nach roarm_.Loop(now) -- bewusst NICHT
// aus RoArmSetupAndLoop selbst heraus (setup_and_loops/*.hh bleiben absichtlich frei von
// Web-/Networking-Wissen, s. roarm.hh-Kommentar), analog zu PerformBootI2cScans() oben, das aus
// demselben Grund schon jetzt hier statt in setup_and_loops/ lebt.
void RoArmBroadcastPoseFeedbackIfDue(uint32_t now);
