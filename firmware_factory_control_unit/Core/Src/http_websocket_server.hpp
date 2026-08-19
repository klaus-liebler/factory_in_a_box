#pragma once
// Generischer HTTPS/WebSocket-Server auf NetXDuo/ThreadX -- ersetzt den bisherigen
// nx_web_http_server (addons/web/nx_web_http_server.c), der keine WebSockets kann. Baut auf
// dem darunterliegenden, protokollunabhaengigen NX_TCPSERVER-Modul auf (addons/web/
// nx_tcpserver.c/.h) -- exakt derselbe Baustein, auf dem nx_web_http_server selbst intern
// steht (siehe dortige nx_tcpserver_tls_setup()-Aufrufe: identische Parameterliste wie
// nx_web_http_server_secure_configure()). NX_TCPSERVER liefert bereits fertig: einen
// Event-Thread, der beliebig viele TCP/TLS-Sessions gleichzeitig bedient (accept/Handshake/
// Empfang/Timeout/Disconnect), inkl. TLS-Handshake pro Session -- hier kommt nur noch die
// eigentliche Anwendungsschicht dazu: HTTP/1.1-Parsing + Routing, und RFC6455-WebSocket-Framing
// oberhalb einer erfolgreichen Upgrade-Verhandlung.
//
// Architektur: EIN NX_TCPSERVER-Thread bedient alle Sessions (kein Thread pro Verbindung).
// Jede Session ist entweder eine gewoehnliche HTTP-Request/Response-Verbindung (mit
// Keep-Alive, also mehrere Requests nacheinander auf derselben TCP/TLS-Verbindung) oder --
// nach einem erfolgreichen Upgrade -- eine dauerhafte WebSocket-Verbindung. Ueber
// MAX_WEBSOCKET_CONNECTIONS wird die Zahl gleichzeitiger WebSocket-Verbindungen begrenzt;
// weitere Upgrade-Versuche bekommen 503 statt 101.
//
// Bewusst KEINE Fragmentierung beim SENDEN (SendText()/SendBinary() erzeugen immer ein
// einzelnes Frame mit FIN=1) -- fuer die hier vorgesehenen kleinen Anwendungsnachrichten
// ausreichend. Beim EMPFANGEN wird Fragmentierung (RFC6455 Abschnitt 5.4) dagegen vollstaendig
// unterstuetzt, da Browser-Clients das jederzeit verwenden duerfen.
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdlib>
#include <cstdio>

#include "tx_api.h"
#include "nx_api.h"
// nx_tcpserver.h hat (anders als praktisch jeder andere NetX-/ThreadX-Header) KEIN eigenes
// "#ifdef __cplusplus extern \"C\" {"-Guard -- sein einziger bisheriger Verwender war das rein
// in C geschriebene nx_web_http_server.c. Ohne dieses manuelle extern "C" wuerden die
// _nx_tcpserver_*()-Deklarationen hier C++-Name-Mangling bekommen und beim Linken nicht zu den
// (in C compilierten, also unmangled) Symbolen aus nx_tcpserver.c passen.
extern "C" {
#include "nx_tcpserver.h"
}
#include "nx_secure_tls_api.h"
#include "nx_crypto_sha1.h"
#include "log.h"

namespace Http {

// Kein <cctype>/tolower(): newlib-Ports (auch dieser arm-none-eabi-Toolchain) definieren in
// ctype.h intern ein Makro namens "_C" (Bitmaske fuer isalpha & Co.) -- kollidiert mit dem
// projektweiten "_C(str)"-String-Cast-Makro (Core/Inc/common_macros.hh) und ueberschreibt es
// fuer den Rest der Uebersetzungseinheit, sobald common_macros.hh bereits (per #pragma once
// bereits einmalig) eingebunden wurde. HTTP-Methoden-/Header-Namen sind ohnehin reines ASCII,
// eine einfache eigene Kleinschreibung genuegt.
inline char ToLowerAscii(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c; }

enum class Method : uint8_t { GET, POST, PUT, PATCH, DELETE_, HEAD, UNKNOWN };

struct Header {
    const char *name;
    const char *value;
};

// Sicht auf einen vollstaendig empfangenen Request -- alle Zeiger zeigen in den
// Session-eigenen Empfangspuffer (WebServer::SessionState::buffer) und sind nur waehrend des
// Handler-Aufrufs gueltig (der Puffer wird direkt danach fuer den naechsten Request wiederverwendet).
struct Request {
    Method method = Method::UNKNOWN;
    const char *path = "";
    const char *query = nullptr;
    const Header *headers = nullptr;
    size_t header_count = 0;
    const uint8_t *body = nullptr;
    size_t body_length = 0;

    const char *FindHeader(const char *name) const {
        for (size_t i = 0; i < header_count; i++) {
            const char *a = headers[i].name;
            const char *b = name;
            while (*a && *b && ToLowerAscii(*a) == ToLowerAscii(*b)) { a++; b++; }
            if (*a == '\0' && *b == '\0') return headers[i].value;
        }
        return nullptr;
    }
};

// Baut/sendet die HTTP-Antwort direkt auf der (immer TLS-) Session des Requests. Zwei
// Varianten wie zuvor in webserver.cpp: SendFixed() fuer kleine, komplett im Speicher
// vorliegende Antworten, SendStreamed() fuer grosse Antworten (SPA-Blob, Register-Dump), die
// haeppchenweise per ChunkWriter-Callback erzeugt werden, ohne die Gesamtgroesse jemals im
// Speicher zu halten (identisches Muster wie die alte send_streamed_response()).
class Response {
public:
    using ChunkWriter = void (*)(void *context, char *dest, size_t want);

    Response(NX_TCP_SESSION *session, NX_PACKET_POOL *pool, bool keep_alive)
        : session_(session), pool_(pool), keep_alive_(keep_alive) {}

    void SendFixed(UINT status, const char *status_text, const char *content_type,
                   const char *extra_headers, const uint8_t *body, size_t body_len) {
        char header[320];
        int header_len = snprintf(header, sizeof(header),
            "HTTP/1.1 %u %s\r\nContent-Type: %s\r\nContent-Length: %u\r\nConnection: %s\r\n%s\r\n",
            (unsigned)status, status_text, content_type, (unsigned)body_len,
            keep_alive_ ? "keep-alive" : "close", extra_headers ? extra_headers : "");
        if (header_len < 0) return;

        NX_PACKET *packet;
        if (Allocate(&packet) != NX_SUCCESS) return;
        if (nx_packet_data_append(packet, header, (ULONG)header_len, pool_, NX_WAIT_FOREVER) != NX_SUCCESS) {
            nx_packet_release(packet);
            return;
        }
        if (body_len > 0 &&
            nx_packet_data_append(packet, (VOID *)body, (ULONG)body_len, pool_, NX_WAIT_FOREVER) != NX_SUCCESS) {
            nx_packet_release(packet);
            return;
        }
        if (Send(packet) != NX_SUCCESS) {
            log_warn("HTTP response send failed (status=%u)", (unsigned)status);
        }
    }

    void SendStreamed(UINT status, const char *content_type, const char *extra_headers,
                       size_t total_length, ChunkWriter writer, void *context) {
        static constexpr size_t CHUNK_SIZE = 512;

        char header[320];
        int header_len = snprintf(header, sizeof(header),
            "HTTP/1.1 %u OK\r\nContent-Type: %s\r\nContent-Length: %u\r\nConnection: %s\r\n%s\r\n",
            (unsigned)status, content_type, (unsigned)total_length,
            keep_alive_ ? "keep-alive" : "close", extra_headers ? extra_headers : "");
        if (header_len < 0) return;

        NX_PACKET *packet;
        if (Allocate(&packet) != NX_SUCCESS) return;
        if (nx_packet_data_append(packet, header, (ULONG)header_len, pool_, NX_WAIT_FOREVER) != NX_SUCCESS) {
            nx_packet_release(packet);
            return;
        }

        char chunk[CHUNK_SIZE];
        size_t sent = 0;
        while (sent < total_length) {
            size_t want = total_length - sent;
            if (want > CHUNK_SIZE) want = CHUNK_SIZE;
            writer(context, chunk, want);

            if (sent > 0) {
                if (Allocate(&packet) != NX_SUCCESS) return;
            }
            if (nx_packet_data_append(packet, chunk, (ULONG)want, pool_, NX_WAIT_FOREVER) != NX_SUCCESS) {
                nx_packet_release(packet);
                return;
            }
            if (Send(packet) != NX_SUCCESS) {
                log_warn("HTTP streamed response send failed (status=%u)", (unsigned)status);
                return;
            }
            sent += want;
        }
        if (total_length == 0) {
            Send(packet);
        }
    }

private:
    UINT Allocate(NX_PACKET **packet) {
        return nx_secure_tls_packet_allocate(&session_->nx_tcp_session_tls_session, pool_, packet, NX_WAIT_FOREVER);
    }
    UINT Send(NX_PACKET *packet) {
        return nx_secure_tls_session_send(&session_->nx_tcp_session_tls_session, packet, NX_WAIT_FOREVER);
    }

    NX_TCP_SESSION *session_;
    NX_PACKET_POOL *pool_;
    bool keep_alive_;
};

// Reassemblierungs-/Sendepuffer fuer WebSocket-Anwendungsnachrichten -- ein Frame (Fragment
// oder unfragmentierte Nachricht) muss vollstaendig hineinpassen. Fuer die hier vorgesehene
// generische Transportschicht (kleine JSON-/Text-Nachrichten) grosszuegig bemessen.
constexpr size_t WS_MAX_MESSAGE_SIZE = 4096;

// Baut ein einzelnes, unfragmentiertes RFC6455-Frame (FIN=1, unmaskiert -- Server->Client-Frames
// duerfen laut Spec nicht maskiert sein) und sendet es ueber die TLS-Session. Von
// WebSocketConnection::Send*() UND von WebServer selbst (Pong-/Close-Antworten) genutzt.
inline bool SendWebSocketFrame(NX_TCP_SESSION *session, NX_PACKET_POOL *pool,
                                uint8_t opcode, const uint8_t *payload, size_t len,
                                ULONG wait_option = NX_WAIT_FOREVER) {
    if (len > WS_MAX_MESSAGE_SIZE) return false;

    uint8_t header[10];
    size_t header_len;
    header[0] = (uint8_t)(0x80 | (opcode & 0x0F));
    if (len <= 125) {
        header[1] = (uint8_t)len;
        header_len = 2;
    } else if (len <= 0xFFFF) {
        header[1] = 126;
        header[2] = (uint8_t)(len >> 8);
        header[3] = (uint8_t)(len & 0xFF);
        header_len = 4;
    } else {
        header[1] = 127;
        for (int i = 0; i < 8; i++) {
            header[2 + i] = (uint8_t)((uint64_t)len >> (8 * (7 - i)));
        }
        header_len = 10;
    }

    NX_PACKET *packet;
    if (nx_secure_tls_packet_allocate(&session->nx_tcp_session_tls_session, pool, &packet, wait_option) != NX_SUCCESS) {
        return false;
    }
    if (nx_packet_data_append(packet, header, (ULONG)header_len, pool, wait_option) != NX_SUCCESS) {
        nx_packet_release(packet);
        return false;
    }
    if (len > 0 && nx_packet_data_append(packet, (VOID *)payload, (ULONG)len, pool, wait_option) != NX_SUCCESS) {
        nx_packet_release(packet);
        return false;
    }
    return nx_secure_tls_session_send(&session->nx_tcp_session_tls_session, packet, wait_option) == NX_SUCCESS;
}

class WebServer;

// Griff auf eine aktive WebSocket-Verbindung, wird dem Anwendungscode bei jedem Connect-/
// Message-/Close-Callback frisch (auf dem Stack) uebergeben -- kein eigener Lebenszyklus,
// nur ein duenner Wrapper um Session-Zeiger + Server-Rueckkanal fuer Close().
class WebSocketConnection {
public:
    bool SendText(const char *data, size_t len) { return SendWebSocketFrame(session_, pool_, 0x1, (const uint8_t *)data, len); }
    bool SendBinary(const uint8_t *data, size_t len) { return SendWebSocketFrame(session_, pool_, 0x2, data, len); }
    // Sendet ein Close-Frame und stoesst das Schliessen der TCP/TLS-Verbindung an. RFC6455
    // 7.1.1 erlaubt der Seite, die den Abbau eingeleitet hat (hier: der Server-Anwendungscode
    // ueber diesen Aufruf), die Verbindung zu schliessen, ohne zwingend auf das
    // Client-Echo zu warten -- fuer die hier vorgesehene generische Transportschicht genuegt das.
    void Close(uint16_t code = 1000);

private:
    friend class WebServer;
    WebSocketConnection(WebServer *server, NX_TCP_SESSION *session, NX_PACKET_POOL *pool)
        : server_(server), session_(session), pool_(pool) {}

    WebServer *server_;
    NX_TCP_SESSION *session_;
    NX_PACKET_POOL *pool_;
};

using RequestHandler = void (*)(void *context, const Request &request, Response &response);
using WebSocketConnectHandler = void (*)(void *context, WebSocketConnection &connection);
using WebSocketMessageHandler = void (*)(void *context, WebSocketConnection &connection, bool is_binary,
                                          const uint8_t *data, size_t len);
using WebSocketCloseHandler = void (*)(void *context, WebSocketConnection &connection);

class WebServer {
public:
    // War 3 (1 serielle HTTP-Verbindung + 2 gleichzeitige WebSocket-Verbindungen, s.
    // Projektanforderung) -- zu knapp fuer echten Betrieb: ein einzelner Browser-Tab belegt
    // bereits 2 der 3 Sessions (1 HTTP-Keep-Alive fuers Registerpolling + 1 WebSocket fuer den
    // Log-Spiegel), sodass JEDE weitere gleichzeitige Verbindung (Reload waehrend die alte
    // Session noch abbaut, zweiter Tab, ...) sofort scheitert. Live beobachtet 2026-08-19:
    // wiederholte Testverbindungen (curl/openssl/Python) fuehrten zu haeufigen Connect-Timeouts
    // und kaputten Antworten, konsistent mit einer erschoepften/evtl. leckenden Session-Tabelle
    // im vendorierten nx_tcpserver (addons/web/nx_tcpserver.c, nicht projekteigen). Auf 6 erhoeht
    // als guenstige Absicherung (RAM hat reichlich Reserve) -- behebt eine tatsaechliche
    // Leckage in der Drittanbieter-Bibliothek nicht grundsaetzlich, vergroessert aber deutlich
    // den Puffer, bevor sie sich bemerkbar macht.
    static constexpr UINT MAX_SESSIONS = 6;
    static constexpr UINT MAX_WEBSOCKET_CONNECTIONS = 2;
    static constexpr size_t RAW_BUFFER_SIZE = 4096;
    static constexpr size_t MAX_HEADERS = 24;
    static constexpr size_t MAX_ROUTES = 16;

    WebServer() { memset(&tcp_server_, 0, sizeof(tcp_server_)); }

    // --- Konfiguration (vor Start()) ---
    void SetDefaultHandler(RequestHandler handler, void *context) {
        default_handler_ = handler;
        default_context_ = context;
    }
    void SetNotFoundHandler(RequestHandler handler, void *context) {
        not_found_handler_ = handler;
        not_found_context_ = context;
    }
    // Nur zur Entscheidung "404 statt Default-Handler" bei unbekannten Pfaden gebraucht (s.
    // DispatchRoute()) -- Default "/api", analog zum bisherigen webserver_request_callback().
    void SetApiPrefix(const char *prefix) { api_prefix_ = prefix; }

    bool RegisterRoute(Method method, const char *path, bool exact_match, RequestHandler handler, void *context) {
        if (route_count_ >= MAX_ROUTES) return false;
        routes_[route_count_++] = Route{method, path, exact_match, handler, context};
        return true;
    }

    // Pfad, unter dem WebSocket-Upgrades akzeptiert werden (z.B. "/ws"). Ohne Aufruf (leerer
    // String) werden keine Upgrades akzeptiert, auch wenn Handler gesetzt sind.
    void SetWebSocketPath(const char *path) { ws_path_ = path; }
    void SetWebSocketHandlers(WebSocketConnectHandler on_open, WebSocketMessageHandler on_message,
                               WebSocketCloseHandler on_close, void *context) {
        ws_on_open_ = on_open;
        ws_on_message_ = on_message;
        ws_on_close_ = on_close;
        ws_context_ = context;
    }

    // --- Lebenszyklus -- Signaturen bewusst nah an nx_web_http_server_create()/
    // _secure_configure()/_secure_ecc_configure()/_start(), um den Umbau von net_setup.cpp
    // klein zu halten. sessions_[] (NX_TCP_SESSION-Array) liegt fest als Klassenmember, statt
    // wie beim alten Server extern uebergeben zu werden -- MAX_SESSIONS ist Compile-Zeit-Konstante.
    UINT Create(NX_IP *ip_ptr, NX_PACKET_POOL *pool_ptr, VOID *stack_ptr, ULONG stack_size,
                UINT thread_priority, ULONG timeout_seconds) {
        packet_pool_ = pool_ptr;
        // Schuetzt sessions_state_[]s mode/cleanup_done-Uebergaenge sowie Broadcast()s Iteration
        // darueber -- Broadcast() ist bewusst von JEDEM Thread aufrufbar (z.B. aus dem Log-Sink,
        // s. ws_log_bridge.cpp), nicht nur vom eigenen nx_tcpserver-Thread, der diese Felder
        // sonst exklusiv besitzt.
        tx_mutex_create(&ws_registry_mutex_, const_cast<CHAR *>("WS Registry Mutex"), TX_NO_INHERIT);
        return nx_tcpserver_create(ip_ptr, &tcp_server_, const_cast<CHAR *>("HTTPS/WS Server"),
                                    NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE, 8192,
                                    nullptr, &WebServer::OnReceiveDataTrampoline,
                                    &WebServer::OnConnectionEndTrampoline, &WebServer::OnConnectionTimeoutTrampoline,
                                    timeout_seconds, stack_ptr, stack_size,
                                    sessions_, sizeof(sessions_), thread_priority, NX_IP_PERIODIC_RATE);
    }

    UINT SecureConfigure(const NX_SECURE_TLS_CRYPTO *crypto_table, UCHAR *metadata_buffer, ULONG metadata_size,
                          UCHAR *packet_buffer, ULONG packet_buffer_size, NX_SECURE_X509_CERT *identity_certificate) {
        return nx_tcpserver_tls_setup(&tcp_server_, crypto_table, metadata_buffer, metadata_size,
                                       packet_buffer, packet_buffer_size, identity_certificate,
                                       nullptr, 0, nullptr, 0, nullptr, 0);
    }

    UINT SecureEccConfigure(const USHORT *supported_groups, USHORT supported_group_count,
                             const NX_CRYPTO_METHOD **curves) {
        return nx_tcpserver_tls_ecc_setup(&tcp_server_, supported_groups, supported_group_count, curves);
    }

    UINT Start(UINT port, UINT listen_backlog) { return nx_tcpserver_start(&tcp_server_, port, listen_backlog); }

    // Oeffentlich (statt privat+friend), da sowohl WebSocketConnection::Close() als auch die
    // internen Protokollfehler-Pfade denselben "jetzt abbauen"-Einstieg brauchen. Idempotent:
    // mehrfacher Aufruf fuer dieselbe Session (z.B. einmal durch uns selbst, einmal durch das
    // asynchrone TCP-Disconnect-Ereignis, das unser eigener nx_tcp_socket_disconnect() ausloest)
    // ist unschaedlich -- Disconnect/Session-Ende auf einer bereits geschlossenen Verbindung
    // liefert lediglich einen (hier ignorierten) Fehlercode zurueck.
    void RequestClose(NX_TCP_SESSION *session) {
        SessionState &s = StateFor(session);
        bool notify_close = false;
        tx_mutex_get(&ws_registry_mutex_, TX_WAIT_FOREVER);
        if (!s.cleanup_done) {
            s.cleanup_done = true;
            if (s.mode == SessionState::Mode::WebSocket) {
                if (active_websocket_count_ > 0) active_websocket_count_--;
                notify_close = true;
            }
        }
        tx_mutex_put(&ws_registry_mutex_);
        // Der Anwendungs-Callback laeuft bewusst AUSSERHALB der Sperre (koennte beliebig lange
        // dauern bzw. seinerseits wieder in den Server hineinrufen, z.B. Broadcast()).
        if (notify_close && ws_on_close_) {
            WebSocketConnection conn(this, session, packet_pool_);
            ws_on_close_(ws_context_, conn);
        }
        nx_secure_tls_session_end(&session->nx_tcp_session_tls_session, NX_NO_WAIT);
        nx_tcp_socket_disconnect(&session->nx_tcp_session_socket, NX_NO_WAIT);
    }

    // Sendet 'data' als einzelnes binaeres WebSocket-Frame an ALLE aktuell verbundenen
    // WebSocket-Clients. Von JEDEM Thread aufrufbar (nicht auf den nx_tcpserver-Thread
    // beschraenkt) -- Hauptzweck ist das Log-Spiegeln (s. ws_log_bridge.cpp), das aus praktisch
    // jedem Thread im System heraus ausgeloest werden kann. Durchgehend NX_NO_WAIT: darf niemals
    // blockieren, da der Aufrufer (z.B. log_log()) dabei ein globales Mutex haelt -- eine nicht
    // zustellbare Nachricht (voller Paket-Pool, langsamer Client) wird dafuer stillschweigend
    // uebersprungen statt den Aufrufer aufzuhalten.
    void Broadcast(const uint8_t *data, size_t len) { BroadcastFrame(0x2, data, len); }

    // Sendet ein leeres Ping-Frame an ALLE aktuell verbundenen WebSocket-Clients -- muss
    // regelmaessig (deutlich unter dem an Create() uebergebenen Session-Timeout, s.
    // net_setup.cpp) aufgerufen werden, z.B. aus einem sowieso periodisch laufenden Thread
    // (s. App::HeartbeatThread()). Grund: nx_tcpserver (addons/web/nx_tcpserver.c) setzt den
    // Idle-Timeout ausschliesslich anhand EMPFANGENER Bytes zurueck -- Broadcast() (Server->Client,
    // z.B. die Log-Spiegelung) zaehlt dafuer NICHT, da rein ausgehend. Ohne jede eingehende
    // Aktivitaet (bei diesem generischen Transport gibt es aktuell keine Client->Server-Nachrichten)
    // wuerde eine ansonsten gesunde WebSocket-Verbindung nach Ablauf des Timeouts einfach getrennt
    // -- sichtbar im Log als "WebSocket: Verbindung geschlossen"/"...geoeffnet" im Minutentakt.
    // Jeder Standard-Browser beantwortet ein Ping-Frame auf WebSocket-Protokollebene automatisch
    // mit einem Pong (RFC6455 Abschnitt 5.5.2/5.5.3), OHNE dass die Web-UI dafuer JS-Code braucht --
    // dieses Pong zaehlt dann wieder als empfangene Aktivitaet und haelt die Verbindung am Leben.
    void SendKeepalivePings() { BroadcastFrame(0x9, nullptr, 0); }

private:
    enum class StepResult { NeedMoreData, Progressed, Closed };

    struct Route {
        Method method;
        const char *path;
        bool exact;
        RequestHandler handler;
        void *context;
    };

    struct SessionState {
        enum class Mode : uint8_t { Http, WebSocket } mode = Mode::Http;

        uint8_t buffer[RAW_BUFFER_SIZE];
        size_t length = 0;

        bool headers_complete = false;
        size_t header_block_length = 0;
        size_t body_length_expected = 0;
        Method method = Method::UNKNOWN;
        char *path = nullptr;
        char *query = nullptr;
        Header headers[MAX_HEADERS];
        size_t header_count = 0;
        bool keep_alive = true;
        bool is_websocket_upgrade = false;
        char sec_websocket_key[64] = {0};

        bool ws_fragment_in_progress = false;
        uint8_t ws_fragment_opcode = 0;
        uint8_t ws_message_buffer[WS_MAX_MESSAGE_SIZE];
        size_t ws_message_length = 0;

        bool cleanup_done = false;
    };

    SessionState &StateFor(NX_TCP_SESSION *session) {
        size_t index = (size_t)(session - tcp_server_.nx_tcpserver_sessions);
        return sessions_state_[index];
    }

    // --- nx_tcpserver-Trampoline: die einzige WebServer-Instanz dieses Projekts wird ueber den
    // Umstand rekonstruiert, dass tcp_server_ das ERSTE Member ist (Adresse von tcp_server_ ==
    // Adresse von *this). Kein Klassentemplate/kein globaler Zeiger noetig, da nx_tcpserver_create()
    // ausschliesslich mit einem NX_TCPSERVER* arbeitet.
    static void OnReceiveDataTrampoline(NX_TCPSERVER *server_ptr, NX_TCP_SESSION *session_ptr) {
        reinterpret_cast<WebServer *>(server_ptr)->OnReceiveData(session_ptr);
    }
    static void OnConnectionEndTrampoline(NX_TCPSERVER *server_ptr, NX_TCP_SESSION *session_ptr) {
        reinterpret_cast<WebServer *>(server_ptr)->OnConnectionEnd(session_ptr);
    }
    static void OnConnectionTimeoutTrampoline(NX_TCPSERVER *server_ptr, NX_TCP_SESSION *session_ptr) {
        reinterpret_cast<WebServer *>(server_ptr)->OnConnectionEnd(session_ptr);
    }

    void OnReceiveData(NX_TCP_SESSION *session) {
        SessionState &s = StateFor(session);

        for (;;) {
            NX_PACKET *packet = nullptr;
            if (nx_secure_tls_session_receive(&session->nx_tcp_session_tls_session, &packet, NX_NO_WAIT) != NX_SUCCESS) {
                break;
            }
            for (NX_PACKET *p = packet; p != nullptr; p = p->nx_packet_next) {
                size_t fragment_len = (size_t)(p->nx_packet_append_ptr - p->nx_packet_prepend_ptr);
                if (s.length + fragment_len > RAW_BUFFER_SIZE) {
                    nx_packet_release(packet);
                    RequestClose(session);
                    return;
                }
                memcpy(s.buffer + s.length, p->nx_packet_prepend_ptr, fragment_len);
                s.length += fragment_len;
            }
            nx_packet_release(packet);
        }

        for (;;) {
            StepResult result = (s.mode == SessionState::Mode::Http) ? TryProcessOneHttpRequest(session, s)
                                                                      : TryProcessOneWebSocketFrame(session, s);
            if (result != StepResult::Progressed) break;
        }
    }

    void OnConnectionEnd(NX_TCP_SESSION *session) {
        RequestClose(session);
        nx_tcp_server_socket_unaccept(&session->nx_tcp_session_socket);
        nx_secure_tls_session_reset(&session->nx_tcp_session_tls_session);
        ResetSessionState(StateFor(session));
    }

    // Setzt nur die Zaehler/Flags zurueck, NICHT die grossen Puffer-Arrays (buffer/
    // ws_message_buffer/headers) -- deren Inhalt ist ausserhalb der jeweils gueltigen
    // Laenge (s.length/s.header_block_length/...) ohnehin nie gelesen. Vermeidet, ein
    // ~8 KB grosses SessionState-Temporaer-Objekt auf dem Stack des (einzigen) nx_tcpserver-
    // Threads anzulegen, nur um es per Zuweisung wieder zu verwerfen.
    static void ResetSessionState(SessionState &s) {
        s.mode = SessionState::Mode::Http;
        s.length = 0;
        s.headers_complete = false;
        s.header_block_length = 0;
        s.body_length_expected = 0;
        s.method = Method::UNKNOWN;
        s.path = nullptr;
        s.query = nullptr;
        s.header_count = 0;
        s.keep_alive = true;
        s.is_websocket_upgrade = false;
        s.sec_websocket_key[0] = '\0';
        s.ws_fragment_in_progress = false;
        s.ws_fragment_opcode = 0;
        s.ws_message_length = 0;
        s.cleanup_done = false;
    }

    static uint8_t *FindHeaderTerminator(uint8_t *buf, size_t len) {
        if (len < 4) return nullptr;
        for (size_t i = 0; i + 3 < len; i++) {
            if (buf[i] == '\r' && buf[i + 1] == '\n' && buf[i + 2] == '\r' && buf[i + 3] == '\n') {
                return buf + i + 4;
            }
        }
        return nullptr;
    }

    static char *FindCrlf(char *buf, size_t len) {
        for (size_t i = 0; i + 1 < len; i++) {
            if (buf[i] == '\r' && buf[i + 1] == '\n') return buf + i;
        }
        return nullptr;
    }

    static bool EqualsIgnoreCase(const char *a, const char *b) {
        while (*a && *b) {
            if (ToLowerAscii(*a) != ToLowerAscii(*b)) return false;
            a++;
            b++;
        }
        return *a == *b;
    }

    static bool ContainsIgnoreCase(const char *haystack, const char *needle) {
        size_t needle_len = strlen(needle);
        for (const char *p = haystack; *p; p++) {
            size_t i = 0;
            while (i < needle_len && p[i] && ToLowerAscii(p[i]) == ToLowerAscii(needle[i])) i++;
            if (i == needle_len) return true;
        }
        return false;
    }

    static Method ParseMethod(const char *token) {
        if (strcmp(token, "GET") == 0) return Method::GET;
        if (strcmp(token, "POST") == 0) return Method::POST;
        if (strcmp(token, "PUT") == 0) return Method::PUT;
        if (strcmp(token, "PATCH") == 0) return Method::PATCH;
        if (strcmp(token, "DELETE") == 0) return Method::DELETE_;
        if (strcmp(token, "HEAD") == 0) return Method::HEAD;
        return Method::UNKNOWN;
    }

    // Zerlegt die bereits als vollstaendig erkannten Kopfzeilen (s.buffer[0..header_block_length))
    // per In-Place-Mutation (Trennzeichen werden durch '\0' ersetzt) in Methode/Pfad/Query/
    // Kopfzeilen-Tabelle. Nur EINMAL pro Request aufgerufen (nicht inkrementell/resumable) --
    // der Aufrufer stellt vorher sicher, dass der komplette Kopfzeilenblock schon da ist, das
    // spart einen inkrementellen Parser fuer einen Fall, der bei Requests dieser Groessenordnung
    // (Browser-Header, wenige hundert Byte) ohnehin immer in einem Rutsch ankommt.
    static bool ParseRequestHeaders(SessionState &s) {
        char *buf = (char *)s.buffer;
        char *line_end = FindCrlf(buf, s.header_block_length);
        if (!line_end) return false;
        *line_end = '\0';

        char *sp1 = strchr(buf, ' ');
        if (!sp1) return false;
        *sp1 = '\0';
        s.method = ParseMethod(buf);

        char *target = sp1 + 1;
        char *sp2 = strchr(target, ' ');
        if (!sp2) return false;
        *sp2 = '\0';

        char *qmark = strchr(target, '?');
        if (qmark) {
            *qmark = '\0';
            s.query = qmark + 1;
        } else {
            s.query = nullptr;
        }
        s.path = target;

        char *cursor = line_end + 2;
        char *buf_end = buf + s.header_block_length;
        s.header_count = 0;
        s.body_length_expected = 0;
        s.keep_alive = true;
        bool has_upgrade_header = false;
        bool has_connection_upgrade = false;
        s.sec_websocket_key[0] = '\0';

        while (cursor < buf_end) {
            if (cursor[0] == '\r' && cursor[1] == '\n') break;
            char *eol = FindCrlf(cursor, (size_t)(buf_end - cursor));
            if (!eol) break;
            *eol = '\0';

            char *colon = strchr(cursor, ':');
            if (colon) {
                *colon = '\0';
                char *value = colon + 1;
                while (*value == ' ' || *value == '\t') value++;

                if (s.header_count < MAX_HEADERS) {
                    s.headers[s.header_count].name = cursor;
                    s.headers[s.header_count].value = value;
                    s.header_count++;
                }

                if (EqualsIgnoreCase(cursor, "Content-Length")) {
                    s.body_length_expected = (size_t)strtoul(value, nullptr, 10);
                } else if (EqualsIgnoreCase(cursor, "Connection")) {
                    if (ContainsIgnoreCase(value, "close")) s.keep_alive = false;
                    if (ContainsIgnoreCase(value, "upgrade")) has_connection_upgrade = true;
                } else if (EqualsIgnoreCase(cursor, "Upgrade")) {
                    if (ContainsIgnoreCase(value, "websocket")) has_upgrade_header = true;
                } else if (EqualsIgnoreCase(cursor, "Sec-WebSocket-Key")) {
                    strncpy(s.sec_websocket_key, value, sizeof(s.sec_websocket_key) - 1);
                    s.sec_websocket_key[sizeof(s.sec_websocket_key) - 1] = '\0';
                }
            }
            cursor = eol + 2;
        }

        s.is_websocket_upgrade = has_upgrade_header && has_connection_upgrade && s.method == Method::GET;
        return true;
    }

    static void ComputeWebSocketAccept(const char *client_key, char *out, size_t out_size) {
        static constexpr char WS_GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
        NX_CRYPTO_SHA1 ctx;
        UCHAR digest[20];
        _nx_crypto_sha1_initialize(&ctx, NX_CRYPTO_HASH_SHA1);
        _nx_crypto_sha1_update(&ctx, (UCHAR *)client_key, (UINT)strlen(client_key));
        _nx_crypto_sha1_update(&ctx, (UCHAR *)WS_GUID, (UINT)(sizeof(WS_GUID) - 1));
        _nx_crypto_sha1_digest_calculate(&ctx, digest, NX_CRYPTO_HASH_SHA1);

        UINT bytes_copied = 0;
        _nx_utility_base64_encode(digest, sizeof(digest), (UCHAR *)out, (UINT)out_size, &bytes_copied);
        if (bytes_copied < out_size) out[bytes_copied] = '\0';
    }

    void SendFixedErrorAndClose(NX_TCP_SESSION *session, UINT status, const char *text) {
        Response resp(session, packet_pool_, false);
        resp.SendFixed(status, text, "text/plain", nullptr, (const uint8_t *)text, strlen(text));
        RequestClose(session);
    }

    // NotApplicable: dieser Request zielt gar nicht auf den konfigurierten WS-Pfad -- der
    // Aufrufer faellt auf normales Routing zurueck (i.d.R. 404, da der WS-Pfad sonst keine Route
    // hat), OHNE dass hier schon eine Antwort gesendet wurde. HandledAsError/HandledAsUpgrade:
    // es wurde bereits eine komplette Antwort gesendet (400/503 bzw. 101) -- der Aufrufer darf
    // in diesem Fall NICHT zusaetzlich DispatchRoute() aufrufen, sonst gingen zwei HTTP-Antworten
    // auf derselben Verbindung raus.
    enum class UpgradeResult { NotApplicable, HandledAsError, HandledAsUpgrade };

    UpgradeResult TryCompleteWebSocketUpgrade(NX_TCP_SESSION *session, SessionState &s) {
        if (!ws_on_message_ || ws_path_[0] == '\0') return UpgradeResult::NotApplicable;
        if (strcmp(s.path, ws_path_) != 0) return UpgradeResult::NotApplicable;

        if (s.sec_websocket_key[0] == '\0') {
            Response resp(session, packet_pool_, s.keep_alive);
            resp.SendFixed(400, "Bad Request", "text/plain", nullptr, (const uint8_t *)"Bad Request", 11);
            return UpgradeResult::HandledAsError;
        }

        if (active_websocket_count_ >= MAX_WEBSOCKET_CONNECTIONS) {
            static const char *msg = "Maximum number of WebSocket connections reached";
            Response resp(session, packet_pool_, s.keep_alive);
            resp.SendFixed(503, "Service Unavailable", "text/plain", nullptr, (const uint8_t *)msg, strlen(msg));
            return UpgradeResult::HandledAsError;
        }

        char accept[32];
        ComputeWebSocketAccept(s.sec_websocket_key, accept, sizeof(accept));

        char header[256];
        int n = snprintf(header, sizeof(header),
            "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
            "Sec-WebSocket-Accept: %s\r\n\r\n", accept);
        if (n < 0) return UpgradeResult::HandledAsError;

        NX_PACKET *packet;
        if (nx_secure_tls_packet_allocate(&session->nx_tcp_session_tls_session, packet_pool_, &packet, NX_WAIT_FOREVER) != NX_SUCCESS) {
            return UpgradeResult::HandledAsError;
        }
        if (nx_packet_data_append(packet, header, (ULONG)n, packet_pool_, NX_WAIT_FOREVER) != NX_SUCCESS) {
            nx_packet_release(packet);
            return UpgradeResult::HandledAsError;
        }
        if (nx_secure_tls_session_send(&session->nx_tcp_session_tls_session, packet, NX_WAIT_FOREVER) != NX_SUCCESS) {
            return UpgradeResult::HandledAsError;
        }
        return UpgradeResult::HandledAsUpgrade;
    }

    void DispatchRoute(const Request &req, Response &resp) {
        for (size_t i = 0; i < route_count_; i++) {
            Route &r = routes_[i];
            if (r.method != req.method) continue;
            bool match = r.exact ? (strcmp(req.path, r.path) == 0)
                                  : (strncmp(req.path, r.path, strlen(r.path)) == 0);
            if (match) {
                r.handler(r.context, req, resp);
                return;
            }
        }

        bool is_api = strncmp(req.path, api_prefix_, strlen(api_prefix_)) == 0;
        if (is_api) {
            if (not_found_handler_) {
                not_found_handler_(not_found_context_, req, resp);
            } else {
                resp.SendFixed(404, "Not Found", "text/plain", nullptr, (const uint8_t *)"Not Found", 9);
            }
        } else if (default_handler_) {
            default_handler_(default_context_, req, resp);
        } else {
            resp.SendFixed(404, "Not Found", "text/plain", nullptr, (const uint8_t *)"Not Found", 9);
        }
    }

    StepResult TryProcessOneHttpRequest(NX_TCP_SESSION *session, SessionState &s) {
        if (!s.headers_complete) {
            uint8_t *header_end = FindHeaderTerminator(s.buffer, s.length);
            if (!header_end) {
                if (s.length >= RAW_BUFFER_SIZE) {
                    SendFixedErrorAndClose(session, 431, "Request Header Fields Too Large");
                    return StepResult::Closed;
                }
                return StepResult::NeedMoreData;
            }
            s.header_block_length = (size_t)(header_end - s.buffer);
            if (!ParseRequestHeaders(s)) {
                SendFixedErrorAndClose(session, 400, "Bad Request");
                return StepResult::Closed;
            }
            s.headers_complete = true;
        }

        size_t total_needed = s.header_block_length + s.body_length_expected;
        if (total_needed > RAW_BUFFER_SIZE) {
            SendFixedErrorAndClose(session, 413, "Payload Too Large");
            return StepResult::Closed;
        }
        if (s.length < total_needed) return StepResult::NeedMoreData;

        Request req;
        req.method = s.method;
        req.path = s.path;
        req.query = s.query;
        req.headers = s.headers;
        req.header_count = s.header_count;
        req.body = s.buffer + s.header_block_length;
        req.body_length = s.body_length_expected;

        UpgradeResult ws_result = s.is_websocket_upgrade ? TryCompleteWebSocketUpgrade(session, s)
                                                          : UpgradeResult::NotApplicable;
        if (ws_result == UpgradeResult::NotApplicable) {
            Response resp(session, packet_pool_, s.keep_alive);
            DispatchRoute(req, resp);
        }
        bool did_upgrade = (ws_result == UpgradeResult::HandledAsUpgrade);

        memmove(s.buffer, s.buffer + total_needed, s.length - total_needed);
        s.length -= total_needed;
        s.headers_complete = false;
        s.header_count = 0;

        if (did_upgrade) {
            tx_mutex_get(&ws_registry_mutex_, TX_WAIT_FOREVER);
            s.mode = SessionState::Mode::WebSocket;
            active_websocket_count_++;
            tx_mutex_put(&ws_registry_mutex_);
            if (ws_on_open_) {
                WebSocketConnection conn(this, session, packet_pool_);
                ws_on_open_(ws_context_, conn);
            }
            return StepResult::Progressed;
        }

        if (!s.keep_alive) {
            RequestClose(session);
            return StepResult::Closed;
        }
        return StepResult::Progressed;
    }

    void ConsumeWsBytes(SessionState &s, size_t n) {
        memmove(s.buffer, s.buffer + n, s.length - n);
        s.length -= n;
    }

    void DeliverWebSocketMessage(NX_TCP_SESSION *session, uint8_t opcode, const uint8_t *data, size_t len) {
        if (!ws_on_message_) return;
        WebSocketConnection conn(this, session, packet_pool_);
        ws_on_message_(ws_context_, conn, opcode == 0x2, data, len);
    }

    void SendWsCloseAndDisconnect(NX_TCP_SESSION *session, uint16_t code) {
        uint8_t payload[2] = {(uint8_t)(code >> 8), (uint8_t)(code & 0xFF)};
        SendWebSocketFrame(session, packet_pool_, 0x8, payload, sizeof(payload));
        RequestClose(session);
    }

    // Gemeinsame Iteration ueber alle aktuell verbundenen WebSocket-Sessions fuer Broadcast()
    // (Daten-Frame) und SendKeepalivePings() (Ping-Frame) -- s. dortige Kommentare.
    void BroadcastFrame(uint8_t opcode, const uint8_t *data, size_t len) {
        tx_mutex_get(&ws_registry_mutex_, TX_WAIT_FOREVER);
        for (UINT i = 0; i < MAX_SESSIONS; i++) {
            if (sessions_state_[i].mode != SessionState::Mode::WebSocket || sessions_state_[i].cleanup_done) {
                continue;
            }
            SendWebSocketFrame(&sessions_[i], packet_pool_, opcode, data, len, NX_NO_WAIT);
        }
        tx_mutex_put(&ws_registry_mutex_);
    }

    // RFC6455 Abschnitt 5.2: liest genau EIN Frame (Basis-Header, ggf. erweiterte
    // Laenge/Maskierungsschluessel, maskierte Nutzlast) aus s.buffer, sobald es vollstaendig
    // vorliegt; sonst NeedMoreData (der Rest bleibt unangetastet im Puffer liegen, bis der
    // naechste Empfangs-Event mehr Bytes nachliefert). Fragmentierte Text-/Binaernachrichten
    // (FIN=0, dann 0..n Continuation-Frames) werden in s.ws_message_buffer zusammengefuehrt.
    StepResult TryProcessOneWebSocketFrame(NX_TCP_SESSION *session, SessionState &s) {
        if (s.length < 2) return StepResult::NeedMoreData;

        uint8_t b0 = s.buffer[0];
        uint8_t b1 = s.buffer[1];
        bool fin = (b0 & 0x80) != 0;
        uint8_t rsv = b0 & 0x70;
        uint8_t opcode = b0 & 0x0F;
        bool masked = (b1 & 0x80) != 0;
        bool is_control = (opcode & 0x08) != 0;
        uint64_t base_len = b1 & 0x7F;

        if (rsv != 0) { SendWsCloseAndDisconnect(session, 1002); return StepResult::Closed; }
        if (!masked) { SendWsCloseAndDisconnect(session, 1002); return StepResult::Closed; }
        if (is_control && (!fin || base_len > 125)) { SendWsCloseAndDisconnect(session, 1002); return StepResult::Closed; }

        size_t pos = 2;
        uint64_t payload_len = base_len;
        if (payload_len == 126) {
            if (s.length < pos + 2) return StepResult::NeedMoreData;
            payload_len = ((uint16_t)s.buffer[pos] << 8) | s.buffer[pos + 1];
            pos += 2;
        } else if (payload_len == 127) {
            if (s.length < pos + 8) return StepResult::NeedMoreData;
            payload_len = 0;
            for (int i = 0; i < 8; i++) payload_len = (payload_len << 8) | s.buffer[pos + i];
            pos += 8;
        }

        // Vor jeder weiteren Zeigerarithmetik pruefen (payload_len ist bis hier ein 64-Bit-Wert
        // direkt aus Client-Daten) -- verhindert sowohl einen Ueberlauf von "pos + 4 + payload_len"
        // bei einer boesartig grossen Laengenangabe als auch ein stilles Haengenbleiben in
        // NeedMoreData: der Vergleich muss das bereits um die (je nach 126-/127-Kodierung
        // unterschiedlich grosse) erweiterte Laengenangabe vorgerueckte "pos" mit einbeziehen --
        // ein fixer Puffer waere fuer die 127-Kodierung (pos bereits 10 statt 2) zu grosszuegig
        // und wuerde Frames zulassen, die selbst nach vollstaendigem Empfang nie in RAW_BUFFER_SIZE
        // passen (s.length koennte "s.length < pos + 4 + payload_len" dann nie mehr unterschreiten).
        if (payload_len > (uint64_t)(RAW_BUFFER_SIZE - pos - 4)) {
            SendWsCloseAndDisconnect(session, 1009);
            return StepResult::Closed;
        }

        if (s.length < pos + 4) return StepResult::NeedMoreData;
        uint8_t mask_key[4];
        memcpy(mask_key, s.buffer + pos, 4);
        pos += 4;

        if (s.length < pos + (size_t)payload_len) return StepResult::NeedMoreData;

        uint8_t *payload = s.buffer + pos;
        for (uint64_t i = 0; i < payload_len; i++) payload[i] ^= mask_key[i % 4];

        size_t frame_total = pos + (size_t)payload_len;

        switch (opcode) {
        case 0x0:
            if (!s.ws_fragment_in_progress) { SendWsCloseAndDisconnect(session, 1002); return StepResult::Closed; }
            if (s.ws_message_length + payload_len > WS_MAX_MESSAGE_SIZE) {
                SendWsCloseAndDisconnect(session, 1009);
                return StepResult::Closed;
            }
            memcpy(s.ws_message_buffer + s.ws_message_length, payload, (size_t)payload_len);
            s.ws_message_length += (size_t)payload_len;
            if (fin) {
                DeliverWebSocketMessage(session, s.ws_fragment_opcode, s.ws_message_buffer, s.ws_message_length);
                s.ws_fragment_in_progress = false;
                s.ws_message_length = 0;
            }
            break;

        case 0x1:
        case 0x2:
            if (s.ws_fragment_in_progress) { SendWsCloseAndDisconnect(session, 1002); return StepResult::Closed; }
            if (!fin) {
                s.ws_fragment_in_progress = true;
                s.ws_fragment_opcode = opcode;
                memcpy(s.ws_message_buffer, payload, (size_t)payload_len);
                s.ws_message_length = (size_t)payload_len;
            } else {
                DeliverWebSocketMessage(session, opcode, payload, (size_t)payload_len);
            }
            break;

        case 0x8: {
            uint16_t code = 1000;
            if (payload_len >= 2) code = ((uint16_t)payload[0] << 8) | payload[1];
            ConsumeWsBytes(s, frame_total);
            SendWsCloseAndDisconnect(session, code);
            return StepResult::Closed;
        }

        case 0x9:
            SendWebSocketFrame(session, packet_pool_, 0xA, payload, (size_t)payload_len);
            break;

        case 0xA:
            break;

        default:
            SendWsCloseAndDisconnect(session, 1002);
            return StepResult::Closed;
        }

        ConsumeWsBytes(s, frame_total);
        return StepResult::Progressed;
    }

    NX_TCPSERVER tcp_server_;  // MUSS das erste Member bleiben (s. Trampoline-Kommentar oben).
    NX_TCP_SESSION sessions_[MAX_SESSIONS];
    SessionState sessions_state_[MAX_SESSIONS];
    NX_PACKET_POOL *packet_pool_ = nullptr;

    Route routes_[MAX_ROUTES];
    size_t route_count_ = 0;
    const char *api_prefix_ = "/api";
    RequestHandler default_handler_ = nullptr;
    void *default_context_ = nullptr;
    RequestHandler not_found_handler_ = nullptr;
    void *not_found_context_ = nullptr;

    const char *ws_path_ = "";
    WebSocketConnectHandler ws_on_open_ = nullptr;
    WebSocketMessageHandler ws_on_message_ = nullptr;
    WebSocketCloseHandler ws_on_close_ = nullptr;
    void *ws_context_ = nullptr;
    UINT active_websocket_count_ = 0;
    TX_MUTEX ws_registry_mutex_;
};

inline void WebSocketConnection::Close(uint16_t code) {
    uint8_t payload[2] = {(uint8_t)(code >> 8), (uint8_t)(code & 0xFF)};
    SendWebSocketFrame(session_, pool_, 0x8, payload, sizeof(payload));
    server_->RequestClose(session_);
}

}  // namespace Http
