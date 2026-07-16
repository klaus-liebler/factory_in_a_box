#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <new>
#include <vector>
#include <utility>
#include "nx_api.h"
#include "fx_api.h"
#include "tx_api.h"
#include "nxd_dhcp_client.h"
#include "nx_web_http_server.h"
#include "nx_stm32_eth_driver.h"
#include "nx_stm32_phy_driver.h"
#include "fx_stm32_sd_driver.h"

#include "modbus_tcp_server.hpp"
#include "modbus_register_map.hpp"
#include "diagnostics.hpp"
#include "PDSink.h"
#include "log.h"

#include "main.h"

#include "gpio.hh"
#include "timer.hh"

#include "generated/modbus_ui_page.h"
#include "generated/device_certificate.h"

#include "nx_crypto.h"
#include "nx_secure_tls_api.h"
#include "nx_secure_x509.h"
#include "nxd_mdns.h"

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

#define NX_CHAR_LITERAL(str) const_cast<CHAR *>(str)

// printf-Format samt passender Argumentliste fuer eine NetX-ULONG-IPv4-Adresse
// (Network-Byte-Order-Oktette aus dem 32-Bit-Wert maskiert/geshiftet), z.B.:
//   log_info("IP Address: " IP_ADDR_FMT, IP_ADDR_FMT_ARGS(g_app_state.ip_address));
#define IP_ADDR_FMT "%lu.%lu.%lu.%lu"
#define IP_ADDR_FMT_ARGS(addr) \
    (((addr) >> 24) & 0xff), (((addr) >> 16) & 0xff), (((addr) >> 8) & 0xff), ((addr) & 0xff)

// ADC1-Handle wird in main.c erzeugt (MX_ADC1_Init), hier nur referenziert.
extern "C" ADC_HandleTypeDef hadc1;

// ============================================================================
// Configuration Constants
// ============================================================================

#define TX_APP_MEM_POOL_SIZE        (10 * 1024)
#define FX_APP_MEM_POOL_SIZE        (10 * 1024)
// War 128 KB, reichte mit dem urspruenglich kleinen SERVER_STACK (4 KB) knapp. Die Summe
// aller tx_byte_allocate(&nx_app_byte_pool, ...)-Aufrufe unten (v.a. NX_APP_PACKET_POOL_SIZE
// mit ~80 KB, plus SERVER_STACK jetzt 16 KB fuer RSA/ECDHE-Handshakes, plus mDNS-Puffer)
// liegt inzwischen bei ~128,6 KB VOR Byte-Pool-Verwaltungsoverhead -- lief also am Limit und
// eine der spaeteren Allocations (z.B. USB-PD-Thread-Stack) schlug vermutlich still fehl
// (ASSURE_SUCCESS -> Error_Handler(), kein HTTP-Server mehr erreichbar). Mit deutlicher
// Reserve statt auf Kante genau genug.
#define NX_APP_MEM_POOL_SIZE        (192 * 1024)

#define DEFAULT_PAYLOAD_SIZE        1536
#define DEFAULT_ARP_CACHE_SIZE      1024
#define DEFAULT_MEMORY_SIZE         1024
#define TOGGLE_LED_PRIORITY         15
#define DEFAULT_PRIORITY            5
#define LINK_PRIORITY               11
#define IO_PRIORITY                 12
#define USB_PD_PRIORITY             13
#define USB_PD_THREAD_SLEEP_TICKS   10
#define IO_THREAD_SLEEP_TICKS       50
#define NX_APP_THREAD_PRIORITY      10
#define NX_APP_INSTANCE_PRIORITY    10
#define NX_APP_THREAD_STACK_SIZE    (8 * 1024)
#define Nx_IP_INSTANCE_THREAD_SIZE  (2 * 1024)
#define HTTPS_PORT                  443
#define SERVER_PACKET_SIZE          (2 * NX_WEB_HTTP_SERVER_MIN_PACKET_SIZE)
// RSA-2048-Signierung waehrend des ECDHE-Handshakes (nx_crypto_huge_number Montgomery-
// Exponentiation) laeuft auf diesem Thread und braucht deutlich mehr Stack als reine
// HTTP-Verarbeitung -- 4096 Byte reichten fuer Plain-HTTP, fuehrten mit HTTPS aber zu einem
// Stack-Overflow mitten im Handshake. 16000 Byte matched demo_netxduo_https.c aus dem
// STM32Cube-Referenzpaket (dort exakt fuer denselben Zweck dimensioniert).
#define SERVER_STACK                16384
#define SERVER_POOL_SIZE            (SERVER_PACKET_SIZE * 4)
#define NX_APP_PACKET_POOL_SIZE     ((DEFAULT_PAYLOAD_SIZE + sizeof(NX_PACKET)) * 50)
#define NX_APP_DEFAULT_IP_ADDRESS   0
#define NX_APP_DEFAULT_NET_MASK     0

// TLS-Empfangspuffer fuer die Reassemblierung von TLS-Records (Handshake-Nachrichten wie die
// Zertifikatskette, aber auch normale Application-Data-Records) -- ein einzelner TLS-Record
// kann laut Spezifikation bis zu ~16 KB gross sein, 8 KB reichte also im ungebuenstigsten Fall
// nicht. ST's eigenes demo_netxduo_https.c nutzt 40 KB (grosszuegig gerundet, nicht errechnet);
// wir bleiben knapp ueber der 16-KB-Grenze, da der Puffer per new[] vom freien Heap kommt (nicht
// vom knappen nx_app_byte_pool) und ~390 KB SRAM ausserhalb des Pools frei sind.
#define TLS_PACKET_BUFFER_SIZE      (17 * 1024)
#define MDNS_THREAD_PRIORITY        14
#define MDNS_STACK_SIZE             (2 * 1024)
#define MDNS_LOCAL_CACHE_SIZE       1024
#define MDNS_PEER_CACHE_SIZE        1024

// ============================================================================
// Application State
// ============================================================================

typedef struct {
    NX_IP ip_instance;
    bool blink_led = false;
    NX_PACKET_POOL packet_pool;
    NX_DHCP dhcp_client;
    NX_WEB_HTTP_SERVER http_server;
    NX_MDNS mdns;
    ModbusTcpServer* modbus_server = nullptr;

    FX_MEDIA sd_media;

    ULONG ip_address;
    ULONG net_mask;

    TX_THREAD app_main_thread;
    TX_THREAD led_thread;
    TX_THREAD link_thread;
    TX_THREAD modbus_server_thread;
    TX_THREAD io_thread;
    TX_THREAD usb_pd_thread;
} AppState;

static AppState g_app_state = {0};

// Muss fuer die Lebensdauer des HTTPS-Servers bestehen bleiben (die TLS-Schicht haelt
// intern einen Pointer darauf), daher statisch statt lokal in tx_application_define().
static NX_SECURE_X509_CERT g_device_certificate;

// mDNS-Probing-Callback -- wird bei Namenskonflikten/-bestaetigung aufgerufen, hier nur
// geloggt (siehe demo_netx_duo_mdns.c fuer das Referenz-Verhalten).
static void mdns_probing_notify(NX_MDNS *mdns_ptr, UCHAR *name, UINT state) {
    (void)mdns_ptr;
    log_info("mDNS: %s probing state=%u", name, state);
}


// ============================================================================
// Success Macro
// ============================================================================

#define XASSERT(status_expr, message_on_fail) \
do { \
    UINT assure_status_ = (status_expr); \
    if (assure_status_ != NX_SUCCESS) { \
        log_error("Error %s: %s:%d, status: 0x%x", message_on_fail, __FILE__, __LINE__, assure_status_); \
        Error_Handler(); \
    } \
} while (0)



// ============================================================================
// Thread Entry Points
// ============================================================================

static void led_thread_entry(ULONG arg);
static void link_thread_entry(ULONG arg);
static void app_main_thread_entry(ULONG arg);
static void io_thread_entry(ULONG arg);
static void usb_pd_thread_entry(ULONG arg);

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

// ============================================================================
// Modbus Server Thread
// ============================================================================
static void modbus_server_thread_entry(ULONG thread_input)
{
    // thread_input ist hier immer gesetzt: tx_thread_create() bekommt
    // (ULONG)g_app_state.modbus_server direkt beim Erzeugen dieses Threads uebergeben,
    // unmittelbar nachdem der Pointer per new ModbusTcpServer(...) zugewiesen wurde.
    ModbusTcpServer* server = (ModbusTcpServer*)thread_input;
    log_info("Modbus TCP Server started on port 502");
    server->run();  // Infinite loop, akzeptiert Clients
    log_info("Modbus TCP Server stopped");
}

// ============================================================================
// USB-PD Thread -- fuer's Erste nur Ausgabe der vom Netzteil gemeldeten
// Capabilities auf der Konsole, keine aktive Spannungsanforderung (der Sink
// fordert selbstaendig 5V an, siehe PDSink::onSourceCapabilities()).
// ============================================================================

static void usb_pd_event_callback(PDSinkEventType eventType) {
    switch (eventType) {
        case PDSinkEventType::sourceCapabilitiesChanged:
            {
                std::unique_ptr<char[]> buf = PowerSink.printCapabilitiesToBuf(25);
                log_info("%s", buf.get());
            }
            break;
        case PDSinkEventType::voltageChanged:
            log_info("USB-PD: active supply now %d mV / %d mA", PowerSink.activeVoltage, PowerSink.activeCurrent);
            // g_app_state.modbus_server ist hier immer gesetzt: der Callback feuert nur aus
            // PowerSink.Loop() (siehe usb_pd_thread_entry), und dieser Thread wird als letzter
            // in app_main_thread_entry gestartet - lange nachdem modbus_server in
            // tx_application_define() zugewiesen wurde.
            g_app_state.modbus_server->write_input_register(ModbusRegisters::Input::PWR_PD_VOLTAGE_MV, (uint16_t)PowerSink.activeVoltage);
            g_app_state.modbus_server->write_input_register(ModbusRegisters::Input::PWR_PD_CURRENT_MA, (uint16_t)PowerSink.activeCurrent);
            g_app_state.modbus_server->write_input_register(ModbusRegisters::Input::PWR_PD_STATUS, PowerSink.isConnected() ? 1 : 0);
            break;
        case PDSinkEventType::powerRejected:
            log_warn("USB-PD: power request rejected by source");
            break;
    }
}

static void usb_pd_thread_entry(ULONG arg) {
    (void)arg;
    while (1) {
        PowerSink.Loop();
        tx_thread_sleep(USB_PD_THREAD_SLEEP_TICKS);
    }
}

// ============================================================================
// LED Thread
// ============================================================================

static void led_thread_entry(ULONG arg) {
    (void)arg;
    while (1) {
        if (g_app_state.blink_led) {
            HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
        }else{
            HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        }
        tx_thread_sleep(50);
    }
}

// ============================================================================
// Link Thread (Ethernet cable detection)
// ============================================================================

// Schreibt Link-Status + (bei Link-Up) die von der LAN8742-PHY tatsächlich
// ausgehandelte Speed/Duplex ins Registermodell. Wird nur bei einer erkannten
// Verbindungsänderung aufgerufen (siehe link_thread_entry), nicht periodisch --
// die Werte aendern sich nur bei einer neuen Auto-Negotiation.
static void update_eth_link_registers(bool link_up) {
    // g_app_state.modbus_server ist hier immer gesetzt: link_thread wird wie io_thread/
    // usb_pd_thread erst am Ende von app_main_thread_entry per tx_thread_resume() gestartet,
    // lange nachdem modbus_server in tx_application_define() zugewiesen wurde.
    ModbusTcpServer& server = *g_app_state.modbus_server;

    uint16_t speed = 0;   // 0=10M, 1=100M, 2=1000M
    uint16_t duplex = 0;  // 0=half, 1=full

    if (link_up) {
        switch (nx_eth_phy_get_link_state()) {
            case ETH_PHY_STATUS_100MBITS_FULLDUPLEX:
                speed = 1; duplex = 1; break;
            case ETH_PHY_STATUS_100MBITS_HALFDUPLEX:
                speed = 1; duplex = 0; break;
            case ETH_PHY_STATUS_10MBITS_FULLDUPLEX:
                speed = 0; duplex = 1; break;
            case ETH_PHY_STATUS_10MBITS_HALFDUPLEX:
            default:
                speed = 0; duplex = 0; break;
        }
    }

    server.write_input_register(ModbusRegisters::Input::ETH_LINK_STATUS, link_up ? 1 : 0);
    server.write_input_register(ModbusRegisters::Input::ETH_LINK_SPEED, speed);
    server.write_input_register(ModbusRegisters::Input::ETH_LINK_DUPLEX, duplex);
}

static void link_thread_entry(ULONG arg) {
    (void)arg;
    ULONG actual_status;
    // -1 = Zustand beim Boot noch unbekannt -- erzwingt bei der ersten Pruefung
    // unten einen Durchlauf des passenden Zweigs (verbunden oder getrennt),
    // egal ob das Kabel beim Start bereits steckt oder nicht. Mit Startwert 0
    // ("nehmen Verbindung an") wuerde der Up-Zweig nie feuern, wenn das Kabel
    // schon beim Boot steckt, und update_eth_link_registers(true) bliebe aus.
    int linkdown = -1;

    while (1) {
        UINT status = nx_ip_interface_status_check(&g_app_state.ip_instance, 0,
                                                    NX_IP_LINK_ENABLED,
                                                    &actual_status, 10);

        if (status == NX_SUCCESS) {
            if (linkdown != 0) {
                linkdown = 0;
                log_info("Network cable connected");
                nx_ip_driver_direct_command(&g_app_state.ip_instance, NX_LINK_ENABLE, &actual_status);

                status = nx_ip_interface_status_check(&g_app_state.ip_instance, 0,
                                                      NX_IP_ADDRESS_RESOLVED,
                                                      &actual_status, 10);
                if (status == NX_SUCCESS) {
                    nx_dhcp_stop(&g_app_state.dhcp_client);
                    nx_dhcp_reinitialize(&g_app_state.dhcp_client);
                    nx_dhcp_start(&g_app_state.dhcp_client);
                }

                update_eth_link_registers(true);
            }
        } else {
            if (linkdown != 1) {
                linkdown = 1;
                log_info("Network cable disconnected");
                nx_ip_driver_direct_command(&g_app_state.ip_instance, NX_LINK_DISABLE, &actual_status);

                update_eth_link_registers(false);
            }
        }

        tx_thread_sleep(NX_IP_PERIODIC_RATE);
    }
}

// ============================================================================
// Eval-Board-Standalone-I/O -- STM32H573I-DK ohne angeschlossene Control-Unit-
// Hardware. Ersetzt die per Header verdrahteten Sensoren/Aktoren (PC6/PD5/PD10/...)
// durch den onboard User-Button und LEDs, damit sich das Register-Modell auch ohne
// externe Verkabelung demonstrieren laesst. Kompressor/Foerderband brauchen echtes
// PWM, das aber laut AF-Tabelle auf keiner der vier onboard-LEDs (PF4/PF1/PI8/PI9)
// verfuegbar ist -- daher an TIM1_CH1/PA8 bzw. TIM2_CH4/PA3 (externe LEDs) statt an
// eine der vier onboard-LEDs verlegt. PI9/PF1 bleiben bewusst LED_GREEN/LED_RED
// (Netzwerk-Blink-Demo bzw. Error_Handler) und werden hier nicht angefasst.
// ============================================================================
#if defined(STM32H573xx)
namespace EvalBoardIO {
    using namespace gpio;

    constexpr Pin LIGHTBARRIER1_BUTTON = Pin::PC13; // User-Button B1, aktiv=HIGH
    constexpr Pin VALVE1_LED = Pin::PI08; // aktiv=LOW
    constexpr Pin VALVE2_LED = Pin::PF04; // aktiv=LOW

    // TimerClockSourceHz = 250 MHz: SYSCLK = HSE(25MHz)/PLLM(5)*PLLN(100)/PLLP(2), APB1-/APB2-
    // Prescaler stehen beide auf /1 (siehe SystemClock_Config in main.c), Timer-Takt = PCLKx also
    // ohne die sonst uebliche Verdopplung bei Prescaler != 1.
    using CompressorPwm = timer::PwmOutput<Pin::PA08, Peripheral::TIM1_, Signal::CH1_, 250000000>;
    using ConveyorPwm   = timer::PwmOutput<Pin::PA03, Peripheral::TIM2_, Signal::CH4_, 250000000>;

    static void init() {
        Gpio::ConfigureGPIOInput(LIGHTBARRIER1_BUTTON, PullDirection::DOWN);
        // Initialwert true = Pin HIGH = LED (aktiv=LOW) aus.
        Gpio::ConfigureGPIOOutput(VALVE1_LED, true);
        Gpio::ConfigureGPIOOutput(VALVE2_LED, true);
        CompressorPwm::Init();
        ConveyorPwm::Init();
    }
}
#endif

// ============================================================================
// I/O Thread (digitale Eingaenge, Drucksensor, Ventile, Diagnostik) -- Stage 1
// ============================================================================

static void io_thread_entry(ULONG arg) {
    (void)arg;
#if defined(STM32H573xx)
    EvalBoardIO::init();
#endif

    while (1) {
        // g_app_state.modbus_server ist hier immer gesetzt: io_thread wird wie usb_pd_thread
        // erst ganz am Ende von app_main_thread_entry per tx_thread_resume() gestartet, lange
        // nachdem modbus_server in tx_application_define() zugewiesen wurde.
        ModbusTcpServer& server = *g_app_state.modbus_server;

        // Lichtschranken (NPN, idle-high -> aktiv = LOW). LS3/PC8 auf dem
        // Eval-Board nicht verdrahtet (Konflikt mit SDMMC1/FileX, s. Plan).
#if defined(STM32H573xx)
        // Standalone-Demo: User-Button B1/PC13 statt LS1/PC6 (s. EvalBoardIO oben).
        // Anders als die NPN-Lichtschranken oben ist der Button aktiv=HIGH, also ohne Negierung.
        bool ls1_active = gpio::Gpio::Get(EvalBoardIO::LIGHTBARRIER1_BUTTON);
#else
        bool ls1_active = (HAL_GPIO_ReadPin(LIGHTBARRIER1_GPIO_Port, LIGHTBARRIER1_Pin) == GPIO_PIN_RESET);
#endif
        bool ls2_active = (HAL_GPIO_ReadPin(LIGHTBARRIER2_GPIO_Port, LIGHTBARRIER2_Pin) == GPIO_PIN_RESET);
        server.write_input_register(ModbusRegisters::Input::LIGHTBARRIER1, ls1_active ? 1 : 0);
        server.write_input_register(ModbusRegisters::Input::LIGHTBARRIER2, ls2_active ? 1 : 0);

        // Analoger Drucksensor -- Rohwert, physikalische Skalierung noch offen.
        // ADC1 laeuft im Continuous-Conversion-Modus (einmalig per HAL_ADC_Start()
        // in main.c gestartet, siehe dort) -- hier also kein Start/Poll/Stop je
        // Durchlauf mehr, einfach den zuletzt gewandelten Wert abholen.
        uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
        server.write_input_register(ModbusRegisters::Input::PRESSURE_RAW, raw);

        // Pneumatikventile aus Holding-Registern setzen
        bool valve1 = server.read_holding_register(ModbusRegisters::Holding::VALVE1) != 0;
        bool valve2 = server.read_holding_register(ModbusRegisters::Holding::VALVE2) != 0;
        bool valve3 = server.read_holding_register(ModbusRegisters::Holding::VALVE3) != 0;
#if defined(STM32H573xx)
        // Standalone-Demo: onboard-LEDs PI8/PF4 statt Valve1/2 an PD10/PD5.
        // LEDs sind aktiv=LOW, daher negiert (valve=true -> Pin LOW -> LED an).
        gpio::Gpio::Set(EvalBoardIO::VALVE1_LED, !valve1);
        gpio::Gpio::Set(EvalBoardIO::VALVE2_LED, !valve2);
#else
        HAL_GPIO_WritePin(VALVE1_GPIO_Port, VALVE1_Pin, valve1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(VALVE2_GPIO_Port, VALVE2_Pin, valve2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
        HAL_GPIO_WritePin(VALVE3_GPIO_Port, VALVE3_Pin, valve3 ? GPIO_PIN_SET : GPIO_PIN_RESET);

#if defined(STM32H573xx)
        // Kompressor/Foerderband -- Holding-Register sind 0..1000 Promille Duty,
        // SetDutyCycle_0_10000 erwartet 0..10000 Basispunkte (Faktor 10). Die externen
        uint16_t compressor_permille = server.read_holding_register(ModbusRegisters::Holding::COMPRESSOR_PWM);
        uint16_t conveyor_permille = server.read_holding_register(ModbusRegisters::Holding::CONVEYOR_PWM);
        EvalBoardIO::CompressorPwm::SetDutyCycle_0_10000(compressor_permille * 10);
        EvalBoardIO::ConveyorPwm::SetDutyCycle_0_10000(conveyor_permille * 10);
#endif

        // HealthState-Aggregation -- ETH_LINK_STATUS/SPEED/DUPLEX selbst werden
        // von link_thread_entry() edge-getriggert geschrieben (siehe update_eth_link_registers),
        // hier nur der aktuelle Status fuer die Health-Bit-Berechnung.
        ULONG actual_status;
        bool eth_link_up = (nx_ip_interface_status_check(&g_app_state.ip_instance, 0,
                                                          NX_IP_LINK_ENABLED, &actual_status, 10) == NX_SUCCESS);
        Diagnostics::update_health_state(server, eth_link_up);
        Diagnostics::update_timer_tick(server, tx_time_get());

        tx_thread_sleep(IO_THREAD_SLEEP_TICKS);
    }
}

// ============================================================================
// Main App Thread -- einziger Thread mit TX_AUTO_START. Fuehrt alle
// Initialisierungen durch, die einen laufenden Thread-Kontext brauchen
// (FileX/HTTP/DHCP/Modbus), und startet danach alle uebrigen (mit
// TX_DONT_START angelegten) Threads explizit per tx_thread_resume().
// ============================================================================

static void app_main_thread_entry(ULONG arg) {
    (void)arg;

    uint32_t data_buffer[512];

    XASSERT(fx_media_open(&g_app_state.sd_media, NX_CHAR_LITERAL("STM32_SDIO_DISK"),
                           fx_stm32_sd_driver, 0,
                           (VOID *)data_buffer, sizeof(data_buffer)),
                           "FileX media open failed");

    log_info("FileX media opened");

    // --- HTTPS (NetX Secure TLS) Setup ---
    // Zertifikat + privater Schluessel kommen aus dem generierten device_certificate.c
    // (siehe tools/provision-certificate.mjs) -- individuell pro Board ueber die
    // STM32-Unique-ID provisioniert und von der privaten CA signiert. TLS 1.2/RSA-2048,
    // kein Client-Zertifikat/Mutual-TLS (letzte drei Parameterpaare NX_NULL/0).
    // nx_secure_x509_certificate_initialize()/nx_secure_tls_metadata_size_calculate() sind
    // NX_THREADS_ONLY_CALLER_CHECKING-beschraenkt, muessen also von einem laufenden Thread
    // aus aufgerufen werden -- dieser Thread ist der einzige mit TX_AUTO_START (siehe
    // tx_application_define()), daher hier statt dort. Puffer kommen bewusst vom Heap
    // (new[]) statt aus nx_app_byte_pool: dieser Pool ist als lokale Variable auf
    // tx_application_define() beschraenkt, waehrend der Heap seit malloc_lock_init() (s.
    // dort) threadsicher ist -- selbes Muster wie beim ModbusTcpServer weiter unten.
    XASSERT(nx_secure_x509_certificate_initialize(
        &g_device_certificate, (UCHAR *)DEVICE_CERT_DER, (USHORT)DEVICE_CERT_DER_LEN,
        NX_NULL, 0, (UCHAR *)DEVICE_KEY_DER, (USHORT)DEVICE_KEY_DER_LEN,
        NX_SECURE_X509_KEY_TYPE_EC_DER), "Device certificate initialize failed");

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

    // Modbus-Server-Initialisierung braucht wie der HTTP-Server-Start oben einen
    // laufenden Thread-Kontext (nx_tcp_server_socket_listen() lehnt Aufrufe aus
    // tx_application_define() sonst mit NX_CALLER_ERROR ab).
    XASSERT(g_app_state.modbus_server->initialize(), "Modbus TCP Server initialization failed");
    Diagnostics::write_startup_registers(*g_app_state.modbus_server);

    // Alle uebrigen Threads sind mit TX_DONT_START angelegt und werden erst hier,
    // ganz am Ende der Initialisierung, gestartet -- so haengt die Boot-Reihenfolge
    // nicht von Thread-Prioritaeten ab, sondern ist explizit festgelegt.
    XASSERT(tx_thread_resume(&g_app_state.led_thread), "LED Thread resume failed");
    XASSERT(tx_thread_resume(&g_app_state.link_thread), "Link Thread resume failed");
    XASSERT(tx_thread_resume(&g_app_state.modbus_server_thread), "Modbus Server Thread resume failed");
    XASSERT(tx_thread_resume(&g_app_state.io_thread), "IO Thread resume failed");
    XASSERT(tx_thread_resume(&g_app_state.usb_pd_thread), "USB-PD Thread resume failed");
}

// ============================================================================
// Modbus-Register-Weboberflaeche -- liefert die unter web/ gebaute Single-File-UI (gzip,
// ins Flash einkompiliert, siehe Core/Src/generated/modbus_ui_page.c) unter "/" aus, sowie
// zwei schlanke Text-Endpunkte zum Lesen/Schreiben aller Register. Bewusst kein JSON auf
// C++-Seite (weder Parser noch Encoder) -- /api/registers liefert zwei kommagetrennte, fest
// 5-stellige Zahlenlisten, /api/write-holding ist ein GET mit Query-Parametern statt
// POST+JSON-Body (erspart nx_web_http_server_content_get()+Parser fuer diese Demo-UI).
// ============================================================================

// Liefert genau 'want' Bytes ab dem aktuellen Zustand von *context nach dest -- wird von
// send_streamed_response() wiederholt aufgerufen, bis die komplette Antwort geschrieben ist.
typedef void (*ChunkWriter)(void *context, char *dest, size_t want);

// Sendet eine Antwort bekannter Gesamtlaenge in 512-Byte-Haeppchen: pro Haeppchen ein frisches
// NX_PACKET allozieren, befuellen, sofort senden -- nie mehr als ein/zwei Pakete gleichzeitig
// in Arbeit, unabhaengig von total_length. Genau das Muster, mit dem die FileX-GET-Verarbeitung
// dieser Middleware selbst grosse Dateien haeppchenweise sendet (nx_web_http_server.c,
// GET-Verarbeitung: Paket allozieren/befuellen/senden/wiederholen) -- braucht daher keinen
// groesseren Server-Paket-Pool als den bestehenden (SERVER_POOL_SIZE, s.u.).
static UINT send_streamed_response(NX_WEB_HTTP_SERVER *server_ptr, const char *content_type,
                                   const char *additional_header, size_t total_length,
                                   ChunkWriter write_chunk, void *context) {
    static constexpr size_t CHUNK_SIZE = 512;

    NX_PACKET *packet_ptr;
    if (nx_web_http_server_callback_generate_response_header(
            server_ptr, &packet_ptr, NX_CHAR_LITERAL(NX_WEB_HTTP_STATUS_OK),
            (UINT)total_length, NX_CHAR_LITERAL(content_type),
            NX_CHAR_LITERAL(additional_header)) != NX_SUCCESS) {
        return NX_NOT_SUCCESSFUL;
    }

    char chunk[CHUNK_SIZE];
    size_t sent = 0;
    while (sent < total_length) {
        size_t want = total_length - sent;
        if (want > CHUNK_SIZE) {
            want = CHUNK_SIZE;
        }
        write_chunk(context, chunk, want);

        if (sent > 0) {
            // Erstes Haeppchen landet im Header-Paket von oben, jedes weitere braucht ein
            // frisches Paket -- das vorherige wurde bereits per packet_send() verschickt.
            // Bewusst nx_web_http_server_response_packet_allocate() statt des rohen
            // nx_packet_allocate(...,0,...): nur diese Variante reserviert am Paketanfang
            // den Platz, den der IP/TCP-Stack beim Senden fuer die Header braucht (siehe
            // _nx_web_http_server_generate_response_header, das dieselbe Funktion fuer das
            // allererste Paket verwendet). Mit rohem nx_packet_allocate(...,0,...) bekommt
            // _nx_ip_header_add() beim Senden ein falsch ausgerichtetes nx_packet_prepend_ptr
            // und loest einen UsageFault (UNALIGNED) aus.
            if (nx_web_http_server_response_packet_allocate(server_ptr, &packet_ptr,
                                                            NX_WAIT_FOREVER) != NX_SUCCESS) {
                return NX_NOT_SUCCESSFUL;
            }
        }

        if (nx_packet_data_append(packet_ptr, chunk, (ULONG)want,
                                  server_ptr->nx_web_http_server_packet_pool_ptr, NX_WAIT_FOREVER) != NX_SUCCESS) {
            nx_packet_release(packet_ptr);
            return NX_NOT_SUCCESSFUL;
        }
        if (nx_web_http_server_callback_packet_send(server_ptr, packet_ptr) != NX_SUCCESS) {
            return NX_NOT_SUCCESSFUL;
        }
        sent += want;
    }

    if (total_length == 0) {
        // Leerer Body: das oben allozierte Header-Paket wurde nie befuellt/gesendet.
        return (nx_web_http_server_callback_packet_send(server_ptr, packet_ptr) == NX_SUCCESS)
                   ? NX_SUCCESS : NX_NOT_SUCCESSFUL;
    }
    return NX_SUCCESS;
}

// Quelle fuer send_streamed_response(): der eincompilierte gzip-Blob, context = Cursor (Byte-
// Offset), der zwischen Aufrufen weiterlaeuft.
static void write_from_flash_blob(void *context, char *dest, size_t want) {
    size_t *cursor = (size_t *)context;
    memcpy(dest, MODBUS_UI_HTML_GZ + *cursor, want);
    *cursor += want;
}

// Quelle fuer send_streamed_response(): alle Holding- dann alle Input-Register als fest
// 6 Byte breite Felder "NNNNN," (5-stellig nullgepolstert + Trennzeichen) -- das letzte
// Holding-Feld nutzt statt "," ein "|" als Bank-Trenner, alle anderen (auch das letzte
// Input-Feld) enden auf ",". register-map.ts/api.ts auf der JS-Seite spiegeln dieses Format
// (splitten auf "|", dann auf ",", leere Tokens durch das feste Format werden ignoriert).
struct RegisterTextState {
    ModbusTcpServer *server;
    uint16_t holding_i = 0;
    uint16_t input_i = 0;
    char cell[8] = {0};
    uint8_t cell_pos = 0;
    uint8_t cell_len = 0;
};

static size_t register_text_total_length() {
    return (size_t)((ModbusRegisters::HOLDING_REGISTER_MAX_INDEX + 1) +
                    (ModbusRegisters::INPUT_REGISTER_MAX_INDEX + 1)) * 6;
}

static void refill_register_text_cell(RegisterTextState &st) {
    uint16_t value;
    char sep;
    if (st.holding_i <= ModbusRegisters::HOLDING_REGISTER_MAX_INDEX) {
        value = st.server->read_holding_register(st.holding_i);
        sep = (st.holding_i == ModbusRegisters::HOLDING_REGISTER_MAX_INDEX) ? '|' : ',';
        st.holding_i++;
    } else {
        value = st.server->read_input_register(st.input_i);
        sep = ',';
        st.input_i++;
    }
    st.cell_len = (uint8_t)snprintf(st.cell, sizeof(st.cell), "%05u%c", value, sep);
    st.cell_pos = 0;
}

static void write_register_text_chunk(void *context, char *dest, size_t want) {
    RegisterTextState *st = (RegisterTextState *)context;
    size_t written = 0;
    while (written < want) {
        if (st->cell_pos == st->cell_len) {
            refill_register_text_cell(*st);
        }
        dest[written++] = st->cell[st->cell_pos++];
    }
}

// ============================================================================
// HTTP Request Callback
// ============================================================================

static UINT webserver_request_callback(NX_WEB_HTTP_SERVER *server_ptr, UINT request_type,
                                       CHAR *resource, NX_PACKET *packet_ptr) {
    (void)request_type;

    if (strcmp(resource, "/") == 0) {
        size_t cursor = 0;
        UINT status = send_streamed_response(server_ptr, "text/html", "Content-Encoding: gzip\r\n",
                                             MODBUS_UI_HTML_GZ_LEN, write_from_flash_blob, &cursor);
        return (status == NX_SUCCESS) ? NX_WEB_HTTP_CALLBACK_COMPLETED : NX_NOT_SUCCESSFUL;
    }

    if (strcmp(resource, "/api/registers") == 0) {
        RegisterTextState state;
        state.server = g_app_state.modbus_server;
        UINT status = send_streamed_response(server_ptr, "text/plain", NX_NULL,
                                             register_text_total_length(), write_register_text_chunk, &state);
        return (status == NX_SUCCESS) ? NX_WEB_HTTP_CALLBACK_COMPLETED : NX_NOT_SUCCESSFUL;
    }

    char response_data[512] = {0};
    char response_type[30] = {0};

    if (strcmp(resource, "/api/write-holding") == 0) {
        char addr_query[24] = {0};
        char value_query[24] = {0};
        UINT addr_query_size, value_query_size;
        unsigned int address = 0, value = 0;

        if (nx_web_http_server_query_get(packet_ptr, 0, addr_query, &addr_query_size, sizeof(addr_query) - 1) == NX_SUCCESS &&
            nx_web_http_server_query_get(packet_ptr, 1, value_query, &value_query_size, sizeof(value_query) - 1) == NX_SUCCESS &&
            sscanf(addr_query, "address=%u", &address) == 1 &&
            sscanf(value_query, "value=%u", &value) == 1 &&
            address <= ModbusRegisters::HOLDING_REGISTER_MAX_INDEX && value <= UINT16_MAX) {
            g_app_state.modbus_server->write_holding_register((uint16_t)address, (uint16_t)value);
            sprintf(response_data, "OK");
        } else {
            sprintf(response_data, "ERROR");
        }
    } else if (strcmp(resource, "/GetNetInfo") == 0) {
        snprintf(response_data, sizeof(response_data), IP_ADDR_FMT ",443", IP_ADDR_FMT_ARGS(g_app_state.ip_address));
    } else if (strcmp(resource, "/LedOn") == 0) {
        log_info("LED On");
        g_app_state.blink_led = true;
        sprintf(response_data, "OK");
    } else if (strcmp(resource, "/LedOff") == 0) {
        log_info("LED Off");
        g_app_state.blink_led = false;
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        sprintf(response_data, "OK");
    } else {
        return NX_SUCCESS;
    }

    UINT string_length;
    nx_web_http_server_type_get(server_ptr, resource, response_type, &string_length);
    response_type[string_length] = '\0';

    NX_PACKET *resp_packet;
    if (nx_web_http_server_callback_generate_response_header(
            server_ptr, &resp_packet, NX_CHAR_LITERAL(NX_WEB_HTTP_STATUS_OK),
            strlen(response_data), response_type, NX_NULL) != NX_SUCCESS) {
        return NX_NOT_SUCCESSFUL;
    }

    if (nx_packet_data_append(resp_packet, response_data, strlen(response_data),
                              server_ptr->nx_web_http_server_packet_pool_ptr,
                              NX_WAIT_FOREVER) != NX_SUCCESS) {
        nx_packet_release(resp_packet);
        return NX_NOT_SUCCESSFUL;
    }

    if (nx_web_http_server_callback_packet_send(server_ptr, resp_packet) != NX_SUCCESS) {
        nx_packet_release(resp_packet);
        return NX_NOT_SUCCESSFUL;
    }

    return NX_WEB_HTTP_CALLBACK_COMPLETED;
}


// ============================================================================
// ThreadX Application Entry Point
// ============================================================================
extern "C" void malloc_lock_init(void);
extern "C" int TCPP0203_WakeupForSink(void);

extern "C" void tx_application_define(void *first_unused_memory) {
    malloc_lock_init();

    log_info("001 Application initialization starting...");

    static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE] __attribute__((aligned(4)));
    static TX_BYTE_POOL tx_app_byte_pool;

    static UCHAR fx_byte_pool_buffer[FX_APP_MEM_POOL_SIZE] __attribute__((aligned(4)));
    static TX_BYTE_POOL fx_app_byte_pool;

    static UCHAR nx_byte_pool_buffer[NX_APP_MEM_POOL_SIZE] __attribute__((aligned(4)));
    static TX_BYTE_POOL nx_app_byte_pool;

    XASSERT(tx_byte_pool_create(&tx_app_byte_pool, NX_CHAR_LITERAL("Tx App Pool"),
                                 tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE), "Tx App Pool create failed");


    XASSERT(tx_byte_pool_create(&fx_app_byte_pool, NX_CHAR_LITERAL("Fx App Pool"),
                                 fx_byte_pool_buffer, FX_APP_MEM_POOL_SIZE), "Fx App Pool create failed");

    XASSERT(tx_byte_pool_create(&nx_app_byte_pool, NX_CHAR_LITERAL("Nx App Pool"),
                                 nx_byte_pool_buffer, NX_APP_MEM_POOL_SIZE), "Nx App Pool create failed");

    fx_system_initialize();
    nx_system_initialize();

    void *ptr=0;
    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, NX_APP_PACKET_POOL_SIZE, TX_NO_WAIT), "NetXDuo App Pool allocate failed");
    XASSERT(nx_packet_pool_create(&g_app_state.packet_pool, NX_CHAR_LITERAL("NetXDuo App Pool"),
                                DEFAULT_PAYLOAD_SIZE, ptr, NX_APP_PACKET_POOL_SIZE), "NetXDuo App Pool create failed");

    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, Nx_IP_INSTANCE_THREAD_SIZE, TX_NO_WAIT), "IP instance memory allocate failed");
    XASSERT(nx_ip_create(&g_app_state.ip_instance, NX_CHAR_LITERAL("NetX IP"),
                       NX_APP_DEFAULT_IP_ADDRESS, NX_APP_DEFAULT_NET_MASK,
                       &g_app_state.packet_pool, nx_stm32_eth_driver,
                       (UCHAR *)ptr, Nx_IP_INSTANCE_THREAD_SIZE,
                       NX_APP_INSTANCE_PRIORITY), "IP create failed");

    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, DEFAULT_ARP_CACHE_SIZE, TX_NO_WAIT), "ARP cache allocate failed");
    XASSERT(nx_arp_enable(&g_app_state.ip_instance, (VOID *)ptr, DEFAULT_ARP_CACHE_SIZE), "ARP enable failed");

    XASSERT(nx_icmp_enable(&g_app_state.ip_instance), "ICMP enable failed");

    XASSERT(nx_tcp_enable(&g_app_state.ip_instance), "TCP enable failed");

    XASSERT(nx_udp_enable(&g_app_state.ip_instance), "UDP enable failed");

    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT), "App Main Thread stack allocate failed");
    tx_thread_create(&g_app_state.app_main_thread, NX_CHAR_LITERAL("App Main Thread"),
                     app_main_thread_entry, 0,
                     (VOID *)ptr, NX_APP_THREAD_STACK_SIZE,
                     NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "LED Thread stack allocate failed");
    tx_thread_create(&g_app_state.led_thread, NX_CHAR_LITERAL("LED Thread"),
                     led_thread_entry, 0,
                     (VOID *)ptr, DEFAULT_MEMORY_SIZE,
                     TOGGLE_LED_PRIORITY, TOGGLE_LED_PRIORITY,
                     TX_NO_TIME_SLICE, TX_DONT_START);

    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "Link Thread stack allocate failed");
    tx_thread_create(&g_app_state.link_thread, NX_CHAR_LITERAL("Link Thread"),
                     link_thread_entry, 0,
                     (VOID *)ptr, 2 * DEFAULT_MEMORY_SIZE,
                     LINK_PRIORITY, LINK_PRIORITY,
                     TX_NO_TIME_SLICE, TX_DONT_START);

    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, SERVER_POOL_SIZE, TX_NO_WAIT), "HTTP Server Pool allocate failed");
    NX_PACKET_POOL *server_pool = (NX_PACKET_POOL *)ptr;
    XASSERT(nx_packet_pool_create(server_pool, NX_CHAR_LITERAL("HTTP Server Pool"),
                                SERVER_PACKET_SIZE, (VOID *)(server_pool + 1),
                                SERVER_POOL_SIZE - sizeof(NX_PACKET_POOL)), "HTTP Server Pool create failed");

    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, SERVER_STACK, TX_NO_WAIT), "HTTP Server stack allocate failed");
    XASSERT(nx_web_http_server_create(&g_app_state.http_server, NX_CHAR_LITERAL("HTTP Server"),
                                    &g_app_state.ip_instance, HTTPS_PORT,
                                    &g_app_state.sd_media, (VOID *)ptr, SERVER_STACK,
                                    server_pool, NX_NULL,
                                    webserver_request_callback), "HTTP Server create failed");

    // --- HTTPS (NetX Secure TLS) Setup ---
    // nx_secure_x509_certificate_initialize()/nx_secure_tls_metadata_size_calculate() sind
    // NX_THREADS_ONLY_CALLER_CHECKING-beschraenkt (kein laufender Thread hier in
    // tx_application_define() -- gleicher Grund, aus dem modbus_server->initialize() weiter
    // unten erst in app_main_thread_entry() laeuft, s. dortigen Kommentar). Deshalb passiert
    // die eigentliche TLS-Konfiguration dort, s. app_main_thread_entry().

    // --- mDNS Setup ---
    // Publiziert DEVICE_HOSTNAME (identisch zum Zertifikats-CN, siehe device_certificate.h)
    // als "<hostname>.local" im lokalen Netz. DEVICE_HOSTNAME besteht nur aus
    // Buchstaben/Ziffern/Bindestrich ("factory-box-<6 Hex-Ziffern>"), erfuellt also bereits
    // die von nx_mdns_create() erzwungenen RFC-1035-Zeichenregeln direkt.
    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, MDNS_STACK_SIZE, TX_NO_WAIT), "mDNS stack allocate failed");
    VOID *mdns_stack = ptr;
    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, MDNS_LOCAL_CACHE_SIZE, TX_NO_WAIT), "mDNS local cache allocate failed");
    VOID *mdns_local_cache = ptr;
    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, MDNS_PEER_CACHE_SIZE, TX_NO_WAIT), "mDNS peer cache allocate failed");
    VOID *mdns_peer_cache = ptr;

    XASSERT(nx_mdns_create(&g_app_state.mdns, &g_app_state.ip_instance, &g_app_state.packet_pool,
                                  MDNS_THREAD_PRIORITY, mdns_stack, MDNS_STACK_SIZE,
                                  (UCHAR *)DEVICE_HOSTNAME,
                                  mdns_local_cache, MDNS_LOCAL_CACHE_SIZE,
                                  mdns_peer_cache, MDNS_PEER_CACHE_SIZE,
                                  mdns_probing_notify), "mDNS create failed");
    XASSERT(nx_mdns_enable(&g_app_state.mdns, 0), "mDNS enable failed");

    XASSERT(nx_dhcp_create(&g_app_state.dhcp_client, &g_app_state.ip_instance, NX_CHAR_LITERAL("DHCP Client")), "DHCP Client create failed");

        // --- Modbus TCP Server Setup ---
    // Server-Instanz und Registerspeicher (std::vector) kommen jetzt aus dem
    // newlib-Heap statt aus dem nx_app_byte_pool -- sicher seit malloc_lock_init()
    // (siehe Core/Src/malloc_lock.c) in tx_application_define() aufgerufen wird.
    ModbusTcpServer::RegisterModel modbus_registers{
        std::vector<uint16_t>(ModbusRegisters::HOLDING_REGISTER_MAX_INDEX + 1, 0),
        std::vector<uint16_t>(ModbusRegisters::INPUT_REGISTER_MAX_INDEX + 1, 0)
    };
    g_app_state.modbus_server = new ModbusTcpServer(&g_app_state.ip_instance, &g_app_state.packet_pool, std::move(modbus_registers));

    // initialize() ruft u.a. nx_tcp_server_socket_listen() auf, das NetX nur aus
    // einem laufenden Thread heraus erlaubt (NX_THREADS_ONLY_CALLER_CHECKING) --
    // hier in tx_application_define() laeuft noch kein Thread. Der eigentliche
    // initialize()-Aufruf passiert daher im app_main_thread_entry() (App Main
    // Thread), der als einziger Thread mit TX_AUTO_START laeuft und danach alle
    // anderen (mit TX_DONT_START angelegten) Threads explizit per
    // tx_thread_resume() startet.

    // Allocate the Modbus server thread stack
    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "Modbus server thread stack allocate failed");

    XASSERT(tx_thread_create(
        &g_app_state.modbus_server_thread,
        NX_CHAR_LITERAL("Modbus Server Thread"),
        modbus_server_thread_entry,
        (ULONG)g_app_state.modbus_server,  // thread_input = pointer to server
        ptr,
        2 * DEFAULT_MEMORY_SIZE,
        DEFAULT_PRIORITY,
        DEFAULT_PRIORITY,
        TX_NO_TIME_SLICE,
        TX_DONT_START
    ), "Modbus server thread create failed");

    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, 2 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "IO Thread stack allocate failed");
    XASSERT(tx_thread_create(&g_app_state.io_thread, NX_CHAR_LITERAL("IO Thread"),
                     io_thread_entry, 0,
                     (VOID *)ptr, 2 * DEFAULT_MEMORY_SIZE,
                     IO_PRIORITY, IO_PRIORITY,
                     TX_NO_TIME_SLICE, TX_DONT_START), "IO thread create failed");

    // --- USB-PD Sink Setup ---
    // The TCPP03-M20 port protection IC sitting between UCPD1 and the USB-C
    // receptacle (STM32H573I-DK) boots into HIBERNATE and won't present CC1/CC2
    // correctly until woken up over I2C4 -- must happen before PowerSink.start()
    // touches UCPD1, or the sink will never see anything on the CC lines.
    if (TCPP0203_WakeupForSink() != 0) {
        log_warn("TCPP0203_WakeupForSink() failed - USB-PD CC lines may not work");
    }

    // PowerSink.start() enables the scheduler timer (TIM7) and initializes the UCPD1
    // PHY (clocks/GPIO/DMA/NVIC) -- plain register-level setup, unlike the NetX calls
    // above it has no running-thread requirement, so it's safe to call here directly.
    // Event delivery (printing capabilities) happens from usb_pd_thread_entry() via
    // PowerSink.Loop(), since the callback must not run from IRQ context.
    PowerSink.start(usb_pd_event_callback);

    // 4x DEFAULT_MEMORY_SIZE: this thread hard-faulted (UFSR.STKOF set) at the plain
    // DEFAULT_MEMORY_SIZE (1024 bytes). The capabilities buffer itself is heap-allocated
    // (PDSink::printCapabilitiesToBuf() returns a std::unique_ptr<char[]>, not a stack
    // array) - the pressure comes from newlib's vfprintf()/stdio formatting in log_info()
    // plus the PD scheduler/Loop() call chain itself.
    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, 4 * DEFAULT_MEMORY_SIZE, TX_NO_WAIT), "USB-PD Thread stack allocate failed");
    XASSERT(tx_thread_create(&g_app_state.usb_pd_thread, NX_CHAR_LITERAL("USB-PD Thread"),
                     usb_pd_thread_entry, 0,
                     (VOID *)ptr, 4 * DEFAULT_MEMORY_SIZE,
                     USB_PD_PRIORITY, USB_PD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_DONT_START), "USB-PD thread create failed");

    log_info("Application initialization complete");
}
