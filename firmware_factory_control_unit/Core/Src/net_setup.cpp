#include <cstdint>
#include <new>

#include "net_setup.hpp"
#include "app.hh"
#include "webserver.hpp"
#include "ws_log_bridge.hpp"
#include "log.h"

#include "nx_stm32_eth_driver.h"
#include "fx_stm32_sd_driver.h"
#include "generated/device_ids.hh"
#include "assets.h"
// _ux_network_driver_entry: USBX' eigener, generischer NX_IP_DRIVER (libs/ST/usbx/common/
// usbx_network/inc/ux_network_driver.h) -- unsere eigene CDC-NCM-Geraeteklasse (s. usbd_ncm.h/.c)
// bindet sich darueber selbststaendig an NetX Duo an (eigene Ethernet-Framing-/Warteschlangen-/
// Thread-Logik, s. dortiger Klassenkommentar), exakt wie USBX' eigene CDC-ECM/RNDIS-Klassen --
// kein projekteigener Treiber-Code mehr noetig. Kompatibel zu dieser C++-Uebersetzungseinheit
// (extern "C"-gekapselt, zieht nur tx_api.h/nx_api.h nach -- beide hier ohnehin schon
// eingebunden).
#include "ux_network_driver.h"
// Per objcopy eingebettete Binaerdaten (assets/device_certificate.der, assets/device_key.der,
// s. CMakeLists.txt BINARY_ASSETS/generated/assets.h) -- Laenge per Zeigerdifferenz Start/Ende
// statt eines separaten LEN-Symbols (objcopy liefert nur _start/_end).

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
// Ehemals von NX_WEB_HTTP_SERVER_MIN_PACKET_SIZE (nx_web_http_server.h) abgeleitet -- jetzt
// eigener, von diesem Addon unabhaengiger Wert fuer den Paket-Pool des neuen Http::WebServer
// (s. http_websocket_server.hpp). Ein NX_PACKET dieser Groesse reicht fuer die meisten
// Antwort-/WebSocket-Frame-Haeppchen; groessere Antworten (SPA-Blob, Register-Dump) verketten
// automatisch mehrere Pakete aus demselben Pool (nx_packet_data_append()).
constexpr uint32_t SERVER_PACKET_SIZE = 1536;
// RSA-2048-Signierung waehrend des ECDHE-Handshakes (nx_crypto_huge_number Montgomery-
// Exponentiation) laeuft auf diesem Thread und braucht deutlich mehr Stack als reine
// HTTP-Verarbeitung -- 4096 Byte reichten fuer Plain-HTTP, fuehrten mit HTTPS aber zu einem
// Stack-Overflow mitten im Handshake. 16000 Byte matched demo_netxduo_https.c aus dem
// STM32Cube-Referenzpaket (dort exakt fuer denselben Zweck dimensioniert).
constexpr uint32_t SERVER_STACK = 16384;
// 8 Pakete Reserve statt der alten 4 -- der neue Server bedient bis zu drei gleichzeitige
// Sessions (1 HTTP + 2 WebSocket, s. Http::WebServer::MAX_SESSIONS) statt vormals zwei rein
// serieller HTTP-Sessions.
constexpr uint32_t SERVER_POOL_SIZE = SERVER_PACKET_SIZE * 8;
constexpr uint32_t NX_APP_PACKET_POOL_SIZE = (DEFAULT_PAYLOAD_SIZE + sizeof(NX_PACKET)) * 50;
constexpr uint32_t NX_APP_DEFAULT_IP_ADDRESS = 0;
constexpr uint32_t NX_APP_DEFAULT_NET_MASK = 0;

// TLS-Empfangspuffer fuer die Reassemblierung von TLS-Records (Handshake-Nachrichten wie die
// Zertifikatskette, aber auch normale Application-Data-Records) -- ein einzelner TLS-Record
// kann laut Spezifikation bis zu ~16 KB gross sein. nx_tcpserver_tls_setup() (s.
// Http::WebServer::SecureConfigure()) teilt diesen Gesamtpuffer gleichmaessig auf alle
// Http::WebServer::MAX_SESSIONS Sessions auf -- 24 KB / 3 Sessions = 8 KB je Session, etwas
// grosszuegiger als der alte, auf eine Session bezogene 17-KB-Wert (der intern bereits durch
// NX_WEB_HTTP_SERVER_SESSION_MAX=2 geteilt wurde). Kommt per new[] vom freien Heap (nicht vom
// knappen nx_app_byte_pool), ausreichend SRAM ist vorhanden (STM32H563: 640 KiB gesamt).
constexpr uint32_t TLS_PACKET_BUFFER_SIZE = 24 * 1024;
constexpr UINT MDNS_THREAD_PRIORITY = 14;
constexpr uint32_t MDNS_STACK_SIZE = 2 * 1024;
constexpr uint32_t MDNS_LOCAL_CACHE_SIZE = 1024;
constexpr uint32_t MDNS_PEER_CACHE_SIZE = 1024;

// --- USB-CDC-NCM: virtuelle NIC (192.168.173.1) + DHCP-Server (vergibt 192.168.173.2 an den
// Host) + bestehende mDNS-Instanz zusaetzlich auf diesem Interface aktiviert. NCM statt des
// zwischenzeitlich versuchten RNDIS (Windows-Bluescreen, s. Commit-Historie) bzw. CDC-ECM (kein
// freier Windows-Treiber) -- der urspruenglich vorgesehene feste Name "factory-box.local" ueber
// eine zweite NX_MDNS-Instanz scheiterte an NX_PORT_UNAVAILABLE, s. Kommentar bei
// nx_mdns_enable() weiter unten. Alles nur auf dem per nx_ip_interface_attach() angehaengten
// zweiten Interface aktiv -- Interface 0 bleibt der bestehende Ethernet-Port unveraendert.
//
// nx_ip_interface_attach() sucht sich den ersten freien Interface-Slot selbst (nicht von aussen
// waehlbar) -- da Interface 0 bereits durch nx_ip_create() belegt ist (s. net_setup_create()
// oben) und dies der einzige weitere attach()-Aufruf im ganzen Projekt ist, ist der resultierende
// Index deterministisch 1. Die DHCP-Server-/mDNS-Konfigurationsaufrufe unten verlangen den Index
// trotzdem explizit als Parameter.
constexpr UINT USB_NCM_INTERFACE_INDEX = 1;
constexpr ULONG USB_NCM_IP_ADDRESS = IP_ADDRESS(192, 168, 173, 1);
constexpr ULONG USB_NCM_NET_MASK = IP_ADDRESS(255, 255, 255, 0);
constexpr ULONG USB_NCM_DHCP_LEASE_ADDRESS = IP_ADDRESS(192, 168, 173, 2);
constexpr uint32_t USB_NCM_DHCP_SERVER_STACK_SIZE = 2 * 1024;

// Muss fuer die Lebensdauer des HTTPS-Servers bestehen bleiben (die TLS-Schicht haelt
// intern einen Pointer darauf), daher statisch statt lokal in net_setup_start().
static NX_SECURE_X509_CERT g_device_certificate;

// FileX haelt diesen Zeiger als Sektor-Cache fuer die gesamte Lebensdauer des geoeffneten
// Mediums, nicht nur waehrend fx_media_open() selbst -- muss deshalb Dateiscope-static statt
// lokale Variable in sd_media_try_open() sein (sonst stille Speicherkorruption des FileX-Caches
// nach Verlassen der Funktion). Wird sowohl vom initialen Oeffnen in net_setup_start() als auch
// vom SD/USB-Arbiter beim Zurueckgeben der Karte an die Firmware wiederverwendet.
static uint32_t sd_media_sector_cache[512];

// IP address change callback
static void ip_address_change_notify_callback(NX_IP *ip_instance, VOID *ptr) {
    App* app = static_cast<App*>(ptr);
    // nx_ip_address_get() kann hier nicht fehlschlagen: ip_instance ist bereits eine gueltige,
    // erzeugte IP-Instanz (der Callback haengt an ihr), und die Zielpointer sind statische
    // Globals - NX_PTR_ERROR/NX_CALLER_ERROR (siehe _nxe_ip_address_get) sind damit ausgeschlossen.
    nx_ip_address_get(ip_instance, &app->ip_address, &app->net_mask);
    if (app->ip_address != 0) {
        log_info("Das Geraet ist jetzt unter %s.local oder " IP_ADDR_FMT " erreichbar",
                 DEVICE_HOSTNAME, IP_ADDR_FMT_ARGS(app->ip_address));
    }
}

// mDNS-Probing-Callback -- wird bei Namenskonflikten/-bestaetigung aufgerufen, hier nur
// geloggt (siehe demo_netx_duo_mdns.c fuer das Referenz-Verhalten).
static void mdns_probing_notify(NX_MDNS *mdns_ptr, UCHAR *name, UINT state) {
    (void)mdns_ptr;
    log_info("mDNS: %s probing state=%u", name, state);
}

void net_setup_create(App *app, TX_BYTE_POOL *nx_app_byte_pool) {
    // MUSS vor dem nx_ip_interface_attach()-Aufruf weiter unten laufen: dieser ruft synchron
    // (noch hier in tx_application_define(), also VOR Scheduler-Start) den generischen
    // ux_network_driver mit NX_LINK_INTERFACE_ATTACH/_INITIALIZE/_ENABLE auf, die dessen internes
    // usb_network_devices[]-Tabellenfeld (ip_instance/interface_ptr) befuellen. usbd_device_setup()
    // (in usbd_device.c) ruft ux_network_driver_init() ebenfalls auf -- aber ERST spaeter aus
    // UsbdDeviceThread heraus, einem Thread, der erst NACH Scheduler-Start laeuft. Waere
    // ux_network_driver_init() nicht bereits hier (idempotent, s. dortige Guard-Variable)
    // vorab aufgerufen worden, wuerde der SPAETERE Aufruf die gesamte Tabelle -- inklusive des
    // von ATTACH bereits befuellten Eintrags -- auf 0 zuruecksetzen (memset), OHNE dass ATTACH
    // je erneut liefe, um ip_instance/interface_ptr nachzutragen: das Netzwerk-Interface haette
    // dauerhaft einen NULL-IP-Instanz-Zeiger (der Paket-Pool liesse sich nie aufloesen, s.
    // Debugging-Sitzung -- DHCP lief nie durch, NTBs wurden empfangen, aber jedes Datagramm
    // verworfen) und ausserdem _ux_network_driver_link_up()s "if (interface_ptr)"-Zweig
    // still ueberspringen (Link-Status bliebe fuer immer FALSE).
    // _ux_network_driver_init() statt des sonst ueblichen ux_network_driver_init()-Alias-Makros:
    // letzteres steckt in ux_api.h (s. usbd_device.c, das ohnehin schon "ux_api.h" einbindet),
    // diese Uebersetzungseinheit bindet aber bewusst nur das schlanke ux_network_driver.h ein,
    // das dieses Alias-Makro nicht mitbringt -- der zugrunde liegende Funktionsname ist trotz
    // des fuehrenden Unterstrichs eine ganz normale, in ux_network_driver.h deklarierte
    // extern-Funktion.
    XASSERT(_ux_network_driver_init(), "USBX network driver init failed");

    void *ptr = 0;
    XASSERT(tx_byte_allocate(nx_app_byte_pool, &ptr, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT), "NetXDuo App Pool allocate failed");
    XASSERT(nx_packet_pool_create(&app->packet_pool, _C("NetXDuo App Pool"),
                                DEFAULT_PAYLOAD_SIZE, ptr, NX_APP_PACKET_POOL_SIZE), "NetXDuo App Pool create failed");

    XASSERT(tx_byte_allocate(nx_app_byte_pool, &ptr, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT), "IP instance memory allocate failed");
    XASSERT(nx_ip_create(&app->ip_instance, _C("NetX IP"),
                       NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK,
                       &app->packet_pool, nx_stm32_eth_driver,
                       (UCHAR *)ptr, Nx_IP_INSTANCE_THREAD_SIZE,
                       NX_APP_INSTANCE_PRIORITY), "IP create failed");

    XASSERT(tx_byte_allocate(nx_app_byte_pool, &ptr, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT), "ARP cache allocate failed");
    XASSERT(nx_arp_enable(&app->ip_instance, ptr, DEFAULT_ARP_CACHE_SIZE), "ARP enable failed");

    XASSERT(nx_icmp_enable(&app->ip_instance), "ICMP enable failed");
    XASSERT(nx_tcp_enable(&app->ip_instance), "TCP enable failed");
    XASSERT(nx_udp_enable(&app->ip_instance), "UDP enable failed");

    XASSERT(tx_byte_allocate(nx_app_byte_pool, &ptr, SERVER_POOL_SIZE, TX_NO_WAIT), "HTTP Server Pool allocate failed");
    NX_PACKET_POOL *server_pool = (NX_PACKET_POOL *)ptr;
    XASSERT(nx_packet_pool_create(server_pool, _C("HTTP Server Pool"),
                                SERVER_PACKET_SIZE, (VOID *)(server_pool + 1),
                                SERVER_POOL_SIZE - sizeof(NX_PACKET_POOL)), "HTTP Server Pool create failed");

    XASSERT(tx_byte_allocate(nx_app_byte_pool, &ptr, SERVER_STACK, TX_NO_WAIT), "HTTP Server stack allocate failed");
    // Priority 4: dieselbe erhoehte Prioritaet, die zuvor NX_WEB_HTTP_SERVER_PRIORITY (nx_user.h)
    // dem alten nx_web_http_server-Thread gab -- reagiert schneller auf eingehende Requests/
    // WebSocket-Frames als die App-Threads (Prioritaet 10, s. NX_APP_INSTANCE_PRIORITY oben).
    // 30s Session-Timeout: Kompromiss zwischen zuegigem Aufraeumen liegengelassener HTTP-
    // Keep-Alive-Verbindungen (vormals 10s) und genug Toleranz fuer WebSocket-Verbindungen, die
    // laut Anforderung laengere Ruhephasen haben duerfen, bis die Anwendung ihr eigenes
    // Nachrichtenprotokoll (inkl. etwaiger Heartbeats) festlegt.
    XASSERT(app->web_server.Create(&app->ip_instance, server_pool, ptr, SERVER_STACK, 4, 30),
            "HTTP/WebSocket Server create failed");
    webserver_register_routes(app->web_server);

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
    XASSERT(tx_byte_allocate(nx_app_byte_pool, &ptr, MDNS_STACK_SIZE, TX_NO_WAIT), "mDNS stack allocate failed");
    VOID *mdns_stack = ptr;
    XASSERT(tx_byte_allocate(nx_app_byte_pool, &ptr, MDNS_LOCAL_CACHE_SIZE, TX_NO_WAIT), "mDNS local cache allocate failed");
    VOID *mdns_local_cache = ptr;
    XASSERT(tx_byte_allocate(nx_app_byte_pool, &ptr, MDNS_PEER_CACHE_SIZE, TX_NO_WAIT), "mDNS peer cache allocate failed");
    VOID *mdns_peer_cache = ptr;

    XASSERT(nx_mdns_create(&app->mdns, &app->ip_instance, &app->packet_pool,
                                  MDNS_THREAD_PRIORITY, mdns_stack, MDNS_STACK_SIZE,
                                  (UCHAR *)DEVICE_HOSTNAME,
                                  mdns_local_cache, MDNS_LOCAL_CACHE_SIZE,
                                  mdns_peer_cache, MDNS_PEER_CACHE_SIZE,
                                  mdns_probing_notify), "mDNS create failed");
    XASSERT(nx_mdns_enable(&app->mdns, 0), "mDNS enable failed");

    XASSERT(nx_dhcp_create(&app->dhcp_client, &app->ip_instance, _C("DHCP Client")), "DHCP Client create failed");

    // --- USB-CDC-NCM: virtuelle NIC + DHCP-Server + zweite mDNS-Instanz ---
    // Kein projekteigener Treiber-Init-Aufruf mehr noetig: die CDC-NCM-Geraeteklasse (s.
    // usbd_ncm.c, ux_device_stack_class_register() dort) verwaltet ihre NetX-Duo-Anbindung ueber
    // USBX' eigenen generischen ux_network_driver komplett selbst (inkl. Paket-Pool-Aufloesung
    // aus der NX_IP-Instanz) -- das hier verwendete app->ip_instance/app->packet_pool ist
    // ausschliesslich fuer Interface 0 (echtes Ethernet) sowie den DHCP-Server unten relevant.

    // nx_ip_interface_attach() (anders als die x509/TLS-Aufrufe oben) ist HIER, in
    // tx_application_define(), zulaessig: wird vor Start des IP-Threads aufgerufen, holt die
    // eigentliche Treiber-Initialisierung/-Freischaltung automatisch waehrend dessen eigenem
    // Boot-Vorgang nach (s. _nx_ip_interface_attach()-Doku). _ux_network_driver_entry (s.
    // ux_network_driver.h-Include oben) ist USBX' eigener, generischer NX_IP_DRIVER -- dieselbe
    // Funktion wird spaeter von der CDC-NCM-Klasse intern wiederverwendet, um sich an genau
    // dieses Interface zu binden (ueber deren ncm_activate()/_ux_network_driver_activate(),
    // s. usbd_ncm.c).
    XASSERT(nx_ip_interface_attach(&app->ip_instance, _C("USB-NCM"), USB_NCM_IP_ADDRESS,
                                    USB_NCM_NET_MASK, _ux_network_driver_entry), "USB-NCM interface attach failed");

    // Zur Diagnose/Absicherung: nicht blind auf USB_NCM_IP_ADDRESS/_NET_MASK vertrauen, sondern
    // das tatsaechlich im NX_IP-Interface hinterlegte Adress-/Masken-Paar zurueckholen und fuer
    // die DHCP-Server-Konfiguration unten verwenden -- falls nx_ip_interface_attach() (z.B. durch
    // Timing/Interface-Index-Ueberraschungen) etwas anderes committed hat als erwartet, faellt
    // das hier sofort auf statt erst in der spaeteren Subnetz-Pruefung von
    // nx_dhcp_set_interface_network_parameters() mit einem schwer zuzuordnenden Fehlercode.
    ULONG usb_ncm_committed_ip = 0;
    ULONG usb_ncm_committed_mask = 0;
    XASSERT(nx_ip_interface_address_get(&app->ip_instance, USB_NCM_INTERFACE_INDEX,
                                         &usb_ncm_committed_ip, &usb_ncm_committed_mask),
            "USB-NCM interface address get failed");
    log_info("USB-NCM interface: committed ip=" IP_ADDR_FMT " mask=" IP_ADDR_FMT,
              IP_ADDR_FMT_ARGS(usb_ncm_committed_ip), IP_ADDR_FMT_ARGS(usb_ncm_committed_mask));

    UINT dhcp_addresses_added = 0;
    XASSERT(tx_byte_allocate(nx_app_byte_pool, &ptr, USB_NCM_DHCP_SERVER_STACK_SIZE, TX_NO_WAIT), "DHCP server stack allocate failed");
    XASSERT(nx_dhcp_server_create(&app->dhcp_server, &app->ip_instance, ptr, USB_NCM_DHCP_SERVER_STACK_SIZE,
                                   _C("DHCP Server"), &app->packet_pool), "DHCP Server create failed");
    // Minimaler Einzel-Lease-Pool: nur die eine Adresse, die der per USB verbundene Host bekommen
    // soll (s. Implementierungsplan) -- start==end.
    XASSERT(nx_dhcp_create_server_ip_address_list(&app->dhcp_server, USB_NCM_INTERFACE_INDEX,
                                                   USB_NCM_DHCP_LEASE_ADDRESS, USB_NCM_DHCP_LEASE_ADDRESS,
                                                   &dhcp_addresses_added), "DHCP Server IP address list create failed");
    // Kein eigener DNS-Server in dieser Firmware (nur mDNS) -- die Fehlerpruefung von
    // nx_dhcp_set_interface_network_parameters() (_nxe_dhcp_set_interface_network_parameters)
    // lehnt dns_server_address==0 aber explizit als "invalid non pointer input" ab
    // (NX_DHCP_INVALID_NETWORK_PARAMETERS, 0xA3 -- das war der eigentliche Fehler, NICHT die
    // Subnetz-Konsistenzpruefung weiter unten in der Funktion, die nie erreicht wurde). Da echte
    // DNS-Aufloesung ohnehin nicht angeboten wird, hier ersatzweise die eigene Adresse eintragen
    // -- unkritisch, der Host bekaeme auf eine echte DNS-Anfrage dorthin einfach keine Antwort.
    // Gateway/Subnetzmaske bewusst aus usb_ncm_committed_ip/_mask (s. oben) statt erneut aus
    // USB_NCM_IP_ADDRESS/_NET_MASK -- muss exakt zu dem passen, was
    // nx_dhcp_create_server_ip_address_list() intern bereits aus dem Interface uebernommen hat
    // (dessen eigene Pruefung liest ebenfalls live vom Interface, nicht die Konstanten).
    XASSERT(nx_dhcp_set_interface_network_parameters(&app->dhcp_server, USB_NCM_INTERFACE_INDEX,
                                                      usb_ncm_committed_mask, usb_ncm_committed_ip,
                                                      usb_ncm_committed_ip),
            "DHCP Server network parameters set failed");

    // Kein mDNS auf dem USB-NCM-Interface (bewusst NICHT nx_mdns_enable(&app->mdns,
    // USB_NCM_INTERFACE_INDEX) wie in einer frueheren Version dieser Datei): eine zweite
    // NX_MDNS-Instanz scheiterte an NX_PORT_UNAVAILABLE (0x23, s. Commit-Historie -- zwei
    // Instanzen koennen nicht beide UDP-Port 5353 auf derselben NX_IP binden), und die
    // bestehende Instanz zusaetzlich auf Interface 1 zu aktivieren loeste denselben
    // board-spezifischen "factory-box-<hex>.local"-Namen auf wie auf dem Ethernet-Interface --
    // je nachdem, welches Interface der Host gerade tatsaechlich erreichen kann, fuehrte das zu
    // nicht erreichbaren Adressen. Die IP dieses Interfaces ist ohnehin fest (192.168.173.1,
    // s. USB_NCM_IP_ADDRESS), daher stattdessen einfach per direkter IP erreichbar --
    // tools/provision_board_individual_data_and_files.mjs traegt diese Adresse zusaetzlich als IP-SAN ins
    // Geraetezertifikat ein, damit https://192.168.173.1/ ohne Zertifikatsfehler funktioniert.
}

// Karte optional: keine gesteckte/funktionierende microSD darf den Rest des Boots
// (Netzwerk/Modbus/alles) nicht blockieren -- HAL_SD_Init() in main.c haelt bei
// fehlender Karte inzwischen ebenfalls nicht mehr an (s. dortiger Kommentar). Ohne
// erfolgreich geoeffnetes Medium bleibt app->sd_media einfach ungenutzt; der
// HTTP-Server wurde bereits mit dem (dann inhaltsleeren) FX_MEDIA-Zeiger erstellt und
// faellt fuer alle von webserver_request_callback() nicht selbst beantworteten URLs auf
// NetX Duo's eigene FileX-basierte Dateiauslieferung zurueck, die ein nicht geoeffnetes
// Medium ihrerseits mit einem HTTP-Fehler statt eines Absturzes quittiert.
bool sd_media_try_open(App *app) {
    UINT sd_status = fx_media_open(&app->sd_media, _C("STM32_SDIO_DISK"),
                                    fx_stm32_sd_driver, 0,
                                    (VOID *)sd_media_sector_cache, sizeof(sd_media_sector_cache));
    if (sd_status == FX_SUCCESS) {
        log_info("FileX media opened");
        return true;
    }
    log_warn("FileX media open failed (status=0x%x) - continuing without microSD card", sd_status);
    return false;
}

void net_setup_start(App *app) {
    sd_media_try_open(app);

    // --- HTTPS (NetX Secure TLS) Setup ---
    // Zertifikat + privater Schluessel kommen aus dem generierten device_certificate.c
    // (siehe tools/provision_board_individual_data_and_files.mjs) -- individuell pro Board ueber die
    // STM32-Unique-ID provisioniert und von der privaten CA signiert. TLS 1.2/ECDSA-P256,
    // kein Client-Zertifikat/Mutual-TLS (letzte drei Parameterpaare NX_NULL/0).
    // nx_secure_x509_certificate_initialize()/nx_secure_tls_metadata_size_calculate() sind
    // NX_THREADS_ONLY_CALLER_CHECKING-beschraenkt, muessen also von einem laufenden Thread
    // aus aufgerufen werden -- dieser Thread ist der einzige mit TX_AUTO_START (siehe
    // tx_application_define()), daher hier statt in net_setup_create(). Puffer kommen bewusst
    // vom Heap (new[]) statt aus nx_app_byte_pool: dieser Pool ist als lokale Variable auf
    // tx_application_define() beschraenkt, waehrend der Heap seit malloc_lock_init() (s.
    // dort) threadsicher ist -- selbes Muster wie beim ModbusTcpServer.
    const USHORT device_cert_der_len = (USHORT)(_binary_device_certificate_der_end - _binary_device_certificate_der_start);
    const USHORT device_key_der_len = (USHORT)(_binary_device_key_der_end - _binary_device_key_der_start);
    XASSERT(nx_secure_x509_certificate_initialize(
        &g_device_certificate, (UCHAR *)_binary_device_certificate_der_start, device_cert_der_len,
        NX_NULL, 0, (UCHAR *)_binary_device_key_der_start, device_key_der_len,
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
    UCHAR *tls_metadata = new UCHAR[tls_metadata_size * Http::WebServer::MAX_SESSIONS];
    UCHAR *tls_packet_buffer = new UCHAR[TLS_PACKET_BUFFER_SIZE];

    XASSERT(app->web_server.SecureConfigure(
        &nx_crypto_tls_ciphers_ecc,
        tls_metadata, tls_metadata_size * Http::WebServer::MAX_SESSIONS,
        tls_packet_buffer, TLS_PACKET_BUFFER_SIZE,
        &g_device_certificate), "HTTPS TLS configure failed");

    // ECDHE braucht zusaetzlich die unterstuetzten Kurven (secp256r1/384r1/521r1, s.
    // nx_crypto_ecc_curves) -- ohne diesen Aufruf schlaegt der Handshake fehl, sobald der
    // Client eine ECDHE-Ciphersuite waehlt.
    XASSERT(app->web_server.SecureEccConfigure(
        nx_crypto_ecc_supported_groups,
        (USHORT)nx_crypto_ecc_supported_groups_size, nx_crypto_ecc_curves), "HTTPS ECC configure failed");

    // Listen-Backlog 2x MAX_SESSIONS (analog zum alten NX_WEB_HTTP_SERVER_MAX_PENDING =
    // NX_WEB_HTTP_SERVER_SESSION_MAX<<1): erlaubt kurzzeitig etwas mehr wartende SYN-Verbindungen,
    // als gleichzeitig bedient werden koennen, statt sie sofort abzulehnen.
    XASSERT(app->web_server.Start(HTTPS_PORT, Http::WebServer::MAX_SESSIONS * 2), "HTTP Server start failed");
    log_info("HTTP/WebSocket Server started");
    // Ab hier wird jede weitere Logger-Ausgabe zusaetzlich per WebSocket an verbundene Clients
    // gespiegelt (s. ws_log_bridge.hpp) -- unschaedlich, dass noch keine Verbindungen bestehen
    // (Broadcast() findet einfach keine aktiven Sessions).
    WsLogBridge_Install(&app->web_server);

    XASSERT(nx_ip_address_change_notify(&app->ip_instance,
                                      ip_address_change_notify_callback,
                                      app), "IP address change notify failed");

    XASSERT(nx_dhcp_start(&app->dhcp_client), "DHCP start failed");

    XASSERT(nx_dhcp_server_start(&app->dhcp_server), "DHCP Server start failed");
}
