#include <cstdint>
#include <new>

#include "net_setup.hpp"
#include "app_state.hpp"
#include "webserver.hpp"
#include "log.h"

#include "nx_stm32_eth_driver.h"
#include "fx_stm32_sd_driver.h"
#include "generated/device_certificate.h"

#include "nx_crypto.h"
#include "nx_secure_tls_api.h"
#include "nx_secure_x509.h"

// Ciphersuite-/ECC-Kurven-Tabellen aus crypto_libraries/src/nx_crypto_generic_ciphersuites.c
// -- kein eigener Header deklariert sie (Referenz: netx_web_basic_ecc_test.c aus dem
// STM32Cube-Paket). "_ecc"-Tabelle statt der Basis-Tabelle: bietet TLS_ECDHE_RSA_WITH_AES_*
// (ECDHE-Keyexchange + RSA-Auth, passt zum vorhandenen RSA-2048-Zertifikat) -- ohne ECDHE
// bieten alle gaengigen Browser keine gemeinsame Ciphersuite mehr an
// (ERR_SSL_VERSION_OR_CIPHER_MISMATCH), da sie die reinen TLS_RSA_*-Suiten (kein Forward
// Secrecy) laengst abgelehnt haben.
extern "C" const NX_SECURE_TLS_CRYPTO nx_crypto_tls_ciphers_ecc;
extern "C" const USHORT nx_crypto_ecc_supported_groups[];
extern "C" const NX_CRYPTO_METHOD *nx_crypto_ecc_curves[];
extern "C" const UINT nx_crypto_ecc_supported_groups_size;

constexpr uint32_t DEFAULT_PAYLOAD_SIZE = 1536;
constexpr uint32_t DEFAULT_ARP_CACHE_SIZE = 1024;
constexpr uint32_t Nx_IP_INSTANCE_THREAD_SIZE = 2 * 1024;
constexpr uint32_t NX_APP_INSTANCE_PRIORITY = 10;
constexpr UINT HTTPS_PORT = 443;
constexpr uint32_t SERVER_PACKET_SIZE = 2 * NX_WEB_HTTP_SERVER_MIN_PACKET_SIZE;
// RSA-2048-Signierung waehrend des ECDHE-Handshakes (nx_crypto_huge_number Montgomery-
// Exponentiation) laeuft auf diesem Thread und braucht deutlich mehr Stack als reine
// HTTP-Verarbeitung -- 4096 Byte reichten fuer Plain-HTTP, fuehrten mit HTTPS aber zu einem
// Stack-Overflow mitten im Handshake. 16000 Byte matched demo_netxduo_https.c aus dem
// STM32Cube-Referenzpaket (dort exakt fuer denselben Zweck dimensioniert).
constexpr uint32_t SERVER_STACK = 16384;
constexpr uint32_t SERVER_POOL_SIZE = SERVER_PACKET_SIZE * 4;
constexpr uint32_t NX_APP_PACKET_POOL_SIZE = (DEFAULT_PAYLOAD_SIZE + sizeof(NX_PACKET)) * 50;
constexpr uint32_t NX_APP_DEFAULT_IP_ADDRESS = 0;
constexpr uint32_t NX_APP_DEFAULT_NET_MASK = 0;

// TLS-Empfangspuffer fuer die Reassemblierung von TLS-Records (Handshake-Nachrichten wie die
// Zertifikatskette, aber auch normale Application-Data-Records) -- ein einzelner TLS-Record
// kann laut Spezifikation bis zu ~16 KB gross sein, 8 KB reichte also im ungebuenstigsten Fall
// nicht. ST's eigenes demo_netxduo_https.c nutzt 40 KB (grosszuegig gerundet, nicht errechnet);
// wir bleiben knapp ueber der 16-KB-Grenze, da der Puffer per new[] vom freien Heap kommt (nicht
// vom knappen nx_app_byte_pool) und ausreichend SRAM ausserhalb des Pools frei ist.
constexpr uint32_t TLS_PACKET_BUFFER_SIZE = 17 * 1024;
constexpr UINT MDNS_THREAD_PRIORITY = 14;
constexpr uint32_t MDNS_STACK_SIZE = 2 * 1024;
constexpr uint32_t MDNS_LOCAL_CACHE_SIZE = 1024;
constexpr uint32_t MDNS_PEER_CACHE_SIZE = 1024;

// Muss fuer die Lebensdauer des HTTPS-Servers bestehen bleiben (die TLS-Schicht haelt
// intern einen Pointer darauf), daher statisch statt lokal in net_setup_start().
static NX_SECURE_X509_CERT g_device_certificate;

// IP address change callback
static void ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr) {
    (void)ptr;
    // nx_ip_address_get() kann hier nicht fehlschlagen: ip_instance ist bereits eine gueltige,
    // erzeugte IP-Instanz (der Callback haengt an ihr), und die Zielpointer sind statische
    // Globals - NX_PTR_ERROR/NX_CALLER_ERROR (siehe _nxe_ip_address_get) sind damit ausgeschlossen.
    nx_ip_address_get(ip_instance, &g_app_state.ip_address, &g_app_state.net_mask);
    if (g_app_state.ip_address != 0) {
        log_info("IP Address: " IP_ADDR_FMT, IP_ADDR_FMT_ARGS(g_app_state.ip_address));
    }
}

// mDNS-Probing-Callback -- wird bei Namenskonflikten/-bestaetigung aufgerufen, hier nur
// geloggt (siehe demo_netx_duo_mdns.c fuer das Referenz-Verhalten).
static void mdns_probing_notify(NX_MDNS *mdns_ptr, UCHAR *name, UINT state) {
    (void)mdns_ptr;
    log_info("mDNS: %s probing state=%u", name, state);
}

void net_setup_create(TX_BYTE_POOL *nx_app_byte_pool) {
    void *ptr = 0;
    XASSERT(tx_byte_allocate(nx_app_byte_pool, (VOID **)&ptr, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT), "NetXDuo App Pool allocate failed");
    XASSERT(nx_packet_pool_create(&g_app_state.packet_pool, NX_CHAR_LITERAL("NetXDuo App Pool"),
                                DEFAULT_PAYLOAD_SIZE, ptr, NX_APP_PACKET_POOL_SIZE), "NetXDuo App Pool create failed");

    XASSERT(tx_byte_allocate(nx_app_byte_pool, (VOID **)&ptr, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT), "IP instance memory allocate failed");
    XASSERT(nx_ip_create(&g_app_state.ip_instance, NX_CHAR_LITERAL("NetX IP"),
                       NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK,
                       &g_app_state.packet_pool, nx_stm32_eth_driver,
                       (UCHAR *)ptr, Nx_IP_INSTANCE_THREAD_SIZE,
                       NX_APP_INSTANCE_PRIORITY), "IP create failed");

    XASSERT(tx_byte_allocate(nx_app_byte_pool, (VOID **)&ptr, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT), "ARP cache allocate failed");
    XASSERT(nx_arp_enable(&g_app_state.ip_instance, (VOID *)ptr, DEFAULT_ARP_CACHE_SIZE), "ARP enable failed");

    XASSERT(nx_icmp_enable(&g_app_state.ip_instance), "ICMP enable failed");
    XASSERT(nx_tcp_enable(&g_app_state.ip_instance), "TCP enable failed");
    XASSERT(nx_udp_enable(&g_app_state.ip_instance), "UDP enable failed");

    XASSERT(tx_byte_allocate(nx_app_byte_pool, (VOID **)&ptr, SERVER_POOL_SIZE, TX_NO_WAIT), "HTTP Server Pool allocate failed");
    NX_PACKET_POOL *server_pool = (NX_PACKET_POOL *)ptr;
    XASSERT(nx_packet_pool_create(server_pool, NX_CHAR_LITERAL("HTTP Server Pool"),
                                SERVER_PACKET_SIZE, (VOID *)(server_pool + 1),
                                SERVER_POOL_SIZE - sizeof(NX_PACKET_POOL)), "HTTP Server Pool create failed");

    XASSERT(tx_byte_allocate(nx_app_byte_pool, (VOID **)&ptr, SERVER_STACK, TX_NO_WAIT), "HTTP Server stack allocate failed");
    XASSERT(nx_web_http_server_create(&g_app_state.http_server, NX_CHAR_LITERAL("HTTP Server"),
                                    &g_app_state.ip_instance, HTTPS_PORT,
                                    &g_app_state.sd_media, (VOID *)ptr, SERVER_STACK,
                                    server_pool, NX_NULL,
                                    webserver_request_callback), "HTTP Server create failed");

    // --- HTTPS (NetX Secure TLS) Setup ---
    // nx_secure_x509_certificate_initialize()/nx_secure_tls_metadata_size_calculate() sind
    // NX_THREADS_ONLY_CALLER_CHECKING-beschraenkt (kein laufender Thread hier in
    // tx_application_define()). Deshalb passiert die eigentliche TLS-Konfiguration erst in
    // net_setup_start(), s. dort.

    // --- mDNS Setup ---
    // Publiziert DEVICE_HOSTNAME (identisch zum Zertifikats-CN, siehe device_certificate.h)
    // als "<hostname>.local" im lokalen Netz. DEVICE_HOSTNAME besteht nur aus
    // Buchstaben/Ziffern/Bindestrich ("factory-box-<6 Hex-Ziffern>"), erfuellt also bereits
    // die von nx_mdns_create() erzwungenen RFC-1035-Zeichenregeln direkt.
    XASSERT(tx_byte_allocate(nx_app_byte_pool, (VOID **)&ptr, MDNS_STACK_SIZE, TX_NO_WAIT), "mDNS stack allocate failed");
    VOID *mdns_stack = ptr;
    XASSERT(tx_byte_allocate(nx_app_byte_pool, (VOID **)&ptr, MDNS_LOCAL_CACHE_SIZE, TX_NO_WAIT), "mDNS local cache allocate failed");
    VOID *mdns_local_cache = ptr;
    XASSERT(tx_byte_allocate(nx_app_byte_pool, (VOID **)&ptr, MDNS_PEER_CACHE_SIZE, TX_NO_WAIT), "mDNS peer cache allocate failed");
    VOID *mdns_peer_cache = ptr;

    XASSERT(nx_mdns_create(&g_app_state.mdns, &g_app_state.ip_instance, &g_app_state.packet_pool,
                                  MDNS_THREAD_PRIORITY, mdns_stack, MDNS_STACK_SIZE,
                                  (UCHAR *)DEVICE_HOSTNAME,
                                  mdns_local_cache, MDNS_LOCAL_CACHE_SIZE,
                                  mdns_peer_cache, MDNS_PEER_CACHE_SIZE,
                                  mdns_probing_notify), "mDNS create failed");
    XASSERT(nx_mdns_enable(&g_app_state.mdns, 0), "mDNS enable failed");

    XASSERT(nx_dhcp_create(&g_app_state.dhcp_client, &g_app_state.ip_instance, NX_CHAR_LITERAL("DHCP Client")), "DHCP Client create failed");
}

void net_setup_start() {
    // static: FileX haelt diesen Zeiger als Sektor-Cache fuer die gesamte Lebensdauer des
    // geoeffneten Mediums, nicht nur waehrend fx_media_open() selbst -- als einfache lokale
    // Stack-Variable waere der Speicher ab dem Return aus dieser Funktion (net_setup_start()
    // ist kein Thread-Entry-Point mehr, der nie zurueckkehrt, sondern eine normale, aus
    // app_main_thread_entry() aufgerufene Funktion) sofort wieder ueberschrieben worden --
    // stille Speicherkorruption des FileX-Caches bei jedem weiteren Funktionsaufruf danach.
    static uint32_t sd_media_sector_cache[512];

    // Karte optional: keine gesteckte/funktionierende microSD darf den Rest des Boots
    // (Netzwerk/Modbus/alles) nicht blockieren -- HAL_SD_Init() in main.c haelt bei
    // fehlender Karte inzwischen ebenfalls nicht mehr an (s. dortiger Kommentar). Ohne
    // erfolgreich geoeffnetes Medium bleibt g_app_state.sd_media einfach ungenutzt; der
    // HTTP-Server wurde bereits mit dem (dann inhaltsleeren) FX_MEDIA-Zeiger erstellt und
    // faellt fuer alle von webserver_request_callback() nicht selbst beantworteten URLs auf
    // NetX Duo's eigene FileX-basierte Dateiauslieferung zurueck, die ein nicht geoeffnetes
    // Medium ihrerseits mit einem HTTP-Fehler statt eines Absturzes quittiert.
    UINT sd_status = fx_media_open(&g_app_state.sd_media, NX_CHAR_LITERAL("STM32_SDIO_DISK"),
                                    fx_stm32_sd_driver, 0,
                                    (VOID *)sd_media_sector_cache, sizeof(sd_media_sector_cache));
    if (sd_status == FX_SUCCESS) {
        log_info("FileX media opened");
    } else {
        log_warn("FileX media open failed (status=0x%x) - continuing without microSD card", sd_status);
    }

    // --- HTTPS (NetX Secure TLS) Setup ---
    // Zertifikat + privater Schluessel kommen aus dem generierten device_certificate.c
    // (siehe tools/provision-certificate.mjs) -- individuell pro Board ueber die
    // STM32-Unique-ID provisioniert und von der privaten CA signiert. TLS 1.2/ECDSA-P256,
    // kein Client-Zertifikat/Mutual-TLS (letzte drei Parameterpaare NX_NULL/0).
    // nx_secure_x509_certificate_initialize()/nx_secure_tls_metadata_size_calculate() sind
    // NX_THREADS_ONLY_CALLER_CHECKING-beschraenkt, muessen also von einem laufenden Thread
    // aus aufgerufen werden -- dieser Thread ist der einzige mit TX_AUTO_START (siehe
    // tx_application_define()), daher hier statt in net_setup_create(). Puffer kommen bewusst
    // vom Heap (new[]) statt aus nx_app_byte_pool: dieser Pool ist als lokale Variable auf
    // tx_application_define() beschraenkt, waehrend der Heap seit malloc_lock_init() (s.
    // dort) threadsicher ist -- selbes Muster wie beim ModbusTcpServer.
    XASSERT(nx_secure_x509_certificate_initialize(
        &g_device_certificate, (UCHAR *)DEVICE_CERT_DER, (USHORT)DEVICE_CERT_DER_LEN,
        NX_NULL, 0, (UCHAR *)DEVICE_KEY_DER, (USHORT)DEVICE_KEY_DER_LEN,
        NX_SECURE_X509_KEY_TYPE_EC_DER), "Device certificate initialize failed");

    // nx_secure_x509_not_before/_not_after sind die rohen ASN.1-UTCTime/GeneralizedTime-Bytes
    // aus dem X.509-Zertifikat (von _nx_secure_x509_certificate_parse() innerhalb obigem
    // certificate_initialize() befuellt) -- Format "YYMMDDHHMMSSZ" bzw. "YYYYMMDDHHMMSSZ",
    // bewusst roh statt geparst ausgegeben (kein Kalenderdatum-Parsing/-Umrechnen hier noetig,
    // nur zur Sichtkontrolle "laeuft das Zertifikat bald ab").
    log_info("Device certificate valid: %.*s .. %.*s",
              (int)g_device_certificate.nx_secure_x509_not_before_length,
              g_device_certificate.nx_secure_x509_not_before,
              (int)g_device_certificate.nx_secure_x509_not_after_length,
              g_device_certificate.nx_secure_x509_not_after);

    ULONG tls_metadata_size = 0;
    XASSERT(nx_secure_tls_metadata_size_calculate(&nx_crypto_tls_ciphers_ecc, &tls_metadata_size),
            "TLS metadata size calculate failed");
    UCHAR *tls_metadata = new UCHAR[tls_metadata_size * NX_WEB_HTTP_SERVER_SESSION_MAX];
    UCHAR *tls_packet_buffer = new UCHAR[TLS_PACKET_BUFFER_SIZE];

    XASSERT(nx_web_http_server_secure_configure(
        &g_app_state.http_server, &nx_crypto_tls_ciphers_ecc,
        tls_metadata, tls_metadata_size * NX_WEB_HTTP_SERVER_SESSION_MAX,
        tls_packet_buffer, TLS_PACKET_BUFFER_SIZE,
        &g_device_certificate, NX_NULL, 0, NX_NULL, 0, NX_NULL, 0), "HTTPS TLS configure failed");

    // ECDHE braucht zusaetzlich die unterstuetzten Kurven (secp256r1/384r1/521r1, s.
    // nx_crypto_ecc_curves) -- ohne diesen Aufruf schlaegt der Handshake fehl, sobald der
    // Client eine ECDHE-Ciphersuite waehlt.
    XASSERT(nx_web_http_server_secure_ecc_configure(
        &g_app_state.http_server, nx_crypto_ecc_supported_groups,
        (USHORT)nx_crypto_ecc_supported_groups_size, nx_crypto_ecc_curves), "HTTPS ECC configure failed");

    NX_WEB_HTTP_SERVER_MIME_MAP mime_maps[] = {
        {NX_CHAR_LITERAL("css"), NX_CHAR_LITERAL("text/css")},
        {NX_CHAR_LITERAL("svg"), NX_CHAR_LITERAL("image/svg+xml")},
        {NX_CHAR_LITERAL("png"), NX_CHAR_LITERAL("image/png")},
        {NX_CHAR_LITERAL("jpg"), NX_CHAR_LITERAL("image/jpg")}
    };
    nx_web_http_server_mime_maps_additional_set(&g_app_state.http_server, mime_maps, 4);

    XASSERT(nx_web_http_server_start(&g_app_state.http_server), "HTTP Server start failed");
    log_info("HTTP Server started");

    XASSERT(nx_ip_address_change_notify(&g_app_state.ip_instance,
                                      ip_address_change_notify_callback,
                                      NULL), "IP address change notify failed");

    XASSERT(nx_dhcp_start(&g_app_state.dhcp_client), "DHCP start failed");
}
