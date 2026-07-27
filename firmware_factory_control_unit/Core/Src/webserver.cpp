// ============================================================================
// Modbus-Register-Weboberflaeche -- liefert die unter web/ gebaute, minifizierte und Brotli-
// komprimierte Single-File-UI (per objcopy ins Flash einkompiliert, siehe assets/index.html.br
// und CMakeLists.txt BINARY_ASSETS) fuer JEDEN Pfad AUSSER "/api/*" aus (SPA-Fallback fuer den
// History-API-Router der UI, s. web/src/shell/router.ts -- "/system"/"/power" muessen serverseitig
// dieselbe index.html.br liefern wie "/"), sowie schlanke Endpunkte zum Lesen/Schreiben aller
// Register und zum Abfragen von System-/I2C-Bus-Informationen. Bewusst kein JSON auf C++-Seite
// (weder Parser noch Encoder) -- /api/registers liefert alle Register voll binaer (2 Byte
// Little-Endian je Register, s. register_binary_total_length()), /api/system liefert ein
// kompaktes festes Binaer-Struct (s. fill_system_info()), das u.a. auch das Ergebnis des
// einmaligen I2C-Geraete-Scans beim Boot enthaelt (s. PerformBootI2cScans(), aufgerufen aus
// Io::Setup()), /api/write-holding ist ein GET mit Query-Parametern statt POST+JSON-Body (erspart
// nx_web_http_server_content_get()+Parser fuer diese Demo-UI). Unbekannte /api/*-Pfade bekommen
// 404 statt der UI, s. webserver_request_callback()/handle_api_request().
// ============================================================================
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "webserver.hpp"
#include "app.hh"
#include "modbus_register_model.hh"
#include "assets.h"
#include "lan8742.h"

// Aus sysmem.c -- gleiche Deklaration wie in io.cpp (FREE_HEAP_KIB-Register) genutzt, hier fuer
// den free_heap_bytes-Wert in /api/system.
extern "C" size_t GetFreeHeapBytes(void);

// Fuer PerformBootI2cScans() (s. weiter unten) -- dieselben drei Busse, die
// setup_and_loops/tof_color.hh und setup_and_loops/power.hh bereits nutzen (main.c
// MX_I2C{1,2,4}_Init()).
extern "C" I2C_HandleTypeDef hi2c1;
extern "C" I2C_HandleTypeDef hi2c2;
extern "C" I2C_HandleTypeDef hi2c4;
// Per objcopy eingebettetes Binary (assets/index.html.br, s. CMakeLists.txt BINARY_ASSETS/
// generated/assets.h) -- Laenge per Zeigerdifferenz Start/Ende statt eines separaten
// LEN-Symbols (objcopy liefert nur _start/_end).

// Liefert genau 'want' Bytes ab dem aktuellen Zustand von *context nach dest -- wird von
// send_streamed_response() wiederholt aufgerufen, bis die komplette Antwort geschrieben ist.
typedef void (*ChunkWriter)(void *context, char *dest, size_t want);

// Sendet eine Antwort bekannter Gesamtlaenge in 512-Byte-Haeppchen: pro Haeppchen ein frisches
// NX_PACKET allozieren, befuellen, sofort senden -- nie mehr als ein/zwei Pakete gleichzeitig
// in Arbeit, unabhaengig von total_length. Genau das Muster, mit dem die FileX-GET-Verarbeitung
// dieser Middleware selbst grosse Dateien haeppchenweise sendet (nx_web_http_server.c,
// GET-Verarbeitung: Paket allozieren/befuellen/senden/wiederholen) -- braucht daher keinen
// groesseren Server-Paket-Pool als den bestehenden (SERVER_POOL_SIZE, s. net_setup.cpp).
static UINT send_streamed_response(NX_WEB_HTTP_SERVER *server_ptr, const char *content_type,
                                   const char *additional_header, size_t total_length,
                                   ChunkWriter write_chunk, void *context) {
    static constexpr size_t CHUNK_SIZE = 512;

    NX_PACKET *packet_ptr;
    if (nx_web_http_server_callback_generate_response_header(
            server_ptr, &packet_ptr, _C(NX_WEB_HTTP_STATUS_OK),
            (UINT)total_length, _C(content_type),
            _C(additional_header)) != NX_SUCCESS) {
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

// Quelle fuer send_streamed_response(): der eincompilierte Brotli-Blob, context = Cursor (Byte-
// Offset), der zwischen Aufrufen weiterlaeuft.
static void write_from_flash_blob(void *context, char *dest, size_t want) {
    size_t *cursor = (size_t *)context;
    memcpy(dest, _binary_index_html_br_start + *cursor, want);
    *cursor += want;
}

// Quelle fuer send_streamed_response(): alle Holding- (Index 0..HOLDING_REGISTER_MAX_INDEX)
// dann alle Input-Register (0..INPUT_REGISTER_MAX_INDEX) als rohe 2-Byte-Little-Endian-Werte,
// ohne jedes Trenn- oder Fuellzeichen -- die Anzahl je Bank ist auf beiden Seiten fix bekannt
// (register-map.ts exportiert HOLDING_REGISTER_COUNT/INPUT_REGISTER_COUNT dafuer), ein
// Laengenpraefix oder Trennzeichen erspart sich damit. Little-Endian ist explizit (Low-Byte
// zuerst geschrieben) statt sich auf die Host-Byte-Order zu verlassen, damit api.ts auf der
// JS-Seite mit DataView.getUint16(offset, /*littleEndian=*/true) exakt dazu passt.
struct RegisterBinaryState {
    Modbus::IModbusRegisterModel *register_model;
    uint16_t holding_i = 0;
    uint16_t input_i = 0;
    uint8_t cell[2] = {0};
    uint8_t cell_pos = 2;
};

static size_t register_binary_total_length() {
    return (size_t)((ModbusRegisters::HOLDING_REGISTER_MAX_INDEX + 1) +
                    (ModbusRegisters::INPUT_REGISTER_MAX_INDEX + 1)) * 2;
}

static void refill_register_binary_cell(RegisterBinaryState &st) {
    uint16_t value;
    if (st.holding_i <= ModbusRegisters::HOLDING_REGISTER_MAX_INDEX) {
        value = st.register_model->GetHoldingRegister(st.holding_i);
        st.holding_i++;
    } else {
        value = st.register_model->GetInputRegister(st.input_i);
        st.input_i++;
    }
    st.cell[0] = (uint8_t)(value & 0xFF);
    st.cell[1] = (uint8_t)(value >> 8);
    st.cell_pos = 0;
}

static void write_register_binary_chunk(void *context, char *dest, size_t want) {
    RegisterBinaryState *st = (RegisterBinaryState *)context;
    size_t written = 0;
    while (written < want) {
        if (st->cell_pos == sizeof(st->cell)) {
            refill_register_binary_cell(*st);
        }
        dest[written++] = (char)st->cell[st->cell_pos++];
    }
}

// LAN8742-PHY-Register fuer /api/system (s. SystemInfoState-Layout unten) -- UNVERAENDERT roh
// weitergereicht, ohne jede Bit-Interpretation in C++: das Parsen/Interpretieren (Link-Status,
// Speed/Duplex, ENERGYON, Auto-MDIX/Polaritaet, TDR-Kabeltyp/-Laenge) passiert im Web-UI
// (web/src/api.ts/system-info-app.ts), analog zum PWR_*-Registermuster. Passive Reads (BSR/
// PHYSCSR/MCSR/SECR/SCSIR) PLUS eine aktiv ausgeloeste TDR-Kabeldiagnose (TCSR/CLR).
//
// ETH_PHY_ADDRESS = 0: nicht per CubeMX generiert (dieses Projekt nutzt keinen ST-BSP-ethernetif.c-
// Layer, s. Datei-Kommentar oben) -- Standardadresse fuer LAN8742 auf den meisten STM32-Nucleo-144-
// Boards (Hardware-Strap). Bislang las kein Code-Pfad in diesem Projekt ueberhaupt MDIO-Register
// (Ethernet lief rein MAC-seitig ohne PHY-Statusabfrage) -- falls diese Adresse fuer die reale
// Factory-Control-Unit-Platine nicht stimmt, liefert HAL_ETH_ReadPHYRegister() lediglich einen
// Timeout (kein Hardware-Risiko), sichtbar an read_ok == false.
constexpr uint32_t ETH_PHY_ADDRESS = 0;

struct PhyRegisters {
    bool read_ok = false;      // false: schon die passiven Reads (BSR etc.) schlugen fehl
    bool tdr_available = false; // TDR-Schreib-/Lesezugriff erfolgreich (unabhaengig von read_ok)
    uint16_t bsr = 0;
    uint16_t physcsr = 0;
    uint16_t mcsr = 0;
    uint16_t secr = 0;
    uint16_t scsir = 0;
    uint16_t tcsr = 0;
    uint16_t clr = 0;
};

// AKTIVE Messung: schreibt TCSR.TDR_ENABLE, wartet kurz auf die Pulsreflexions-Messung und liest
// TCSR/CLR roh zurueck (Kabeltyp/-Status/-Laenge stecken darin als Bitfelder, s.
// stm32_libs/.../lan8742.h LAN8742_TCSR_*/LAN8742_CLR_* -- Interpretation im Web-UI).
// HAL_Delay(2): empirisch gewaehlte kurze Wartezeit (analog zum TMC2209-Inter-Transaction-Delay,
// s. Projekt-Historie) -- falls Messwerte in der Praxis noch "stale" wirken, hier zuerst ansetzen.
static void read_tdr_registers(PhyRegisters &r) {
    if (HAL_ETH_WritePHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_TCSR, LAN8742_TCSR_TDR_ENABLE) != HAL_OK) {
        return;
    }
    HAL_Delay(2);

    uint32_t tcsr = 0;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_TCSR, &tcsr) != HAL_OK) {
        return;
    }
    r.tdr_available = true;
    r.tcsr = (uint16_t)tcsr;

    uint32_t clr = 0;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_CLR, &clr) == HAL_OK) {
        r.clr = (uint16_t)clr;
    }
}

// SECR (Symbol Error Counter) loescht sich laut LAN8742-Datenblatt beim Lesen selbst -- der hier
// gelesene Wert ist deshalb der Zaehlerstand SEIT DEM LETZTEN GET /api/system, nicht seit
// Systemstart.
static PhyRegisters read_phy_registers() {
    PhyRegisters r;
    uint32_t bsr = 0, physcsr = 0, mcsr = 0, secr = 0, scsir = 0;

    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_BSR, &bsr) != HAL_OK) return r;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_PHYSCSR, &physcsr) != HAL_OK) return r;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_MCSR, &mcsr) != HAL_OK) return r;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_SECR, &secr) != HAL_OK) return r;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_SCSIR, &scsir) != HAL_OK) return r;

    r.read_ok = true;
    r.bsr = (uint16_t)bsr;
    r.physcsr = (uint16_t)physcsr;
    r.mcsr = (uint16_t)mcsr;
    r.secr = (uint16_t)secr;
    r.scsir = (uint16_t)scsir;

    read_tdr_registers(r);
    return r;
}

// Quelle fuer send_streamed_response(): ein einmalig (bei GET /api/system) zusammengestelltes,
// festes Binaer-Struct mit Systeminformationen -- klein genug, um komplett in einen lokalen
// Stack-Puffer geschrieben und in einem einzigen Chunk gesendet zu werden. Enthaelt bewusst NUR
// echte Laufzeitwerte -- Firmware-Version, Board-Name, Hostname, Chip-UID und MAC-Adressen sind
// Compile-Zeit-Konstanten (s. Core/Inc/constants.hh bzw. Core/generated/device_ids.hh) und werden
// stattdessen einmalig beim Web-Build nach web/generated/build-info.ts eincompiliert (s.
// builder/src/phases/read-git-status.ts) -- keine unnoetige Wire-Uebertragung bei jedem
// Seitenaufruf. Layout (alle Mehrbyte-Felder Little-Endian, s. RegisterBinaryState-Kommentar
// oben), von system-info-app.ts per DataView an exakt denselben Offsets wieder ausgelesen:
//   Offset  Bytes  Feld
//        0      4  uptime_seconds          (uint32)
//        4      4  free_heap_bytes         (uint32)
//        8      4  ip_address              (4x uint8, MSB-Oktett zuerst, wie IP_ADDR_FMT_ARGS)
//       12      4  net_mask                (4x uint8, MSB-Oktett zuerst)
//       16      1  reset_cause_code        (uint8, s. App::ResetCauseCode())
//       17      1  phy_read_ok             (uint8 0/1, s. PhyRegisters::read_ok -- false: alle
//                                            phy_*-Felder unten bedeutungslos)
//       18      1  phy_tdr_available       (uint8 0/1, s. PhyRegisters::tdr_available -- false:
//                                            phy_tcsr/phy_clr bedeutungslos)
//       19      2  phy_bsr                 (uint16, LAN8742_BSR -- roh, Interpretation im Web-UI)
//       21      2  phy_physcsr             (uint16, LAN8742_PHYSCSR -- roh)
//       23      2  phy_mcsr                (uint16, LAN8742_MCSR -- roh)
//       25      2  phy_secr                (uint16, LAN8742_SECR -- roh, s. read_phy_registers()-
//                                            Kommentar zum Selbst-Loeschen beim Lesen)
//       27      2  phy_scsir               (uint16, LAN8742_SCSIR -- roh)
//       29      2  phy_tcsr                (uint16, LAN8742_TCSR nach TDR-Trigger -- roh)
//       31      2  phy_clr                 (uint16, LAN8742_CLR -- roh)
//       33     16  i2c1_scan               (Bitfeld, s. App::i2c1_scan)
//       49     16  i2c2_scan               (Bitfeld, s. App::i2c2_scan)
//       65     16  i2c4_scan               (Bitfeld, s. App::i2c4_scan)
//       81          Gesamtlaenge
struct SystemInfoState {
    uint8_t buffer[81];
    size_t pos = 0;
};

static void put_u16(uint8_t *dest, uint16_t v) {
    dest[0] = (uint8_t)(v & 0xFF);
    dest[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *dest, uint32_t v) {
    dest[0] = (uint8_t)(v & 0xFF);
    dest[1] = (uint8_t)((v >> 8) & 0xFF);
    dest[2] = (uint8_t)((v >> 16) & 0xFF);
    dest[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void put_ip_octets(uint8_t *dest, ULONG addr) {
    dest[0] = (uint8_t)((addr >> 24) & 0xFF);
    dest[1] = (uint8_t)((addr >> 16) & 0xFF);
    dest[2] = (uint8_t)((addr >> 8) & 0xFF);
    dest[3] = (uint8_t)(addr & 0xFF);
}

static void fill_system_info(SystemInfoState &st) {
    App &app = App::Instance();
    uint8_t *p = st.buffer;

    put_u32(p + 0, (uint32_t)(tx_time_get() / TX_TIMER_TICKS_PER_SECOND));
    put_u32(p + 4, (uint32_t)GetFreeHeapBytes());
    put_ip_octets(p + 8, app.ip_address);
    put_ip_octets(p + 12, app.net_mask);
    p[16] = app.ResetCauseCode();

    PhyRegisters phy = read_phy_registers();
    p[17] = phy.read_ok ? 1 : 0;
    p[18] = phy.tdr_available ? 1 : 0;
    put_u16(p + 19, phy.bsr);
    put_u16(p + 21, phy.physcsr);
    put_u16(p + 23, phy.mcsr);
    put_u16(p + 25, phy.secr);
    put_u16(p + 27, phy.scsir);
    put_u16(p + 29, phy.tcsr);
    put_u16(p + 31, phy.clr);

    memcpy(p + 33, app.i2c1_scan, 16);
    memcpy(p + 49, app.i2c2_scan, 16);
    memcpy(p + 65, app.i2c4_scan, 16);

    st.pos = 0;
}

static void write_system_info_chunk(void *context, char *dest, size_t want) {
    SystemInfoState *st = (SystemInfoState *)context;
    memcpy(dest, st->buffer + st->pos, want);
    st->pos += want;
}

// Einmaliger I2C-Geraete-Scan beim Boot (s. webserver.hpp PerformBootI2cScans() fuer die
// Begruendung: ein per HTTP ausgeloester Scan schlug wiederholt fehl -- Verdacht auf
// Netzwerk-Interruptlast waehrend der laufenden Anfrage, s. Commit 72033d9 "cross-peripheral
// I2C/network starvation" --, waehrend derselbe Scan-Code direkt in main() vor jeder
// Netzwerk-/ThreadX-Aktivitaet zuverlaessig funktionierte). BEWUSST OHNE HAL_I2C_DeInit()+Init()
// vor dem Scan: ein erster Versuch dieser Portierung tat genau das ("billige Vorsichtsmassnahme")
// und lieferte danach einen falschen Treffer (Phantom-Geraet bei einer Adresse, an der real
// nichts haengt) statt des tatsaechlich verbauten BME280 -- der Re-Init-Zyklus selbst hat den
// Bus offenbar kurzzeitig in einen Zustand versetzt, der einzelne Adressen faelschlich ACKen
// laesst. main.c MX_I2C{1,2,4}_Init() (laeuft VOR tx_kernel_enter(), also lange vor diesem Scan)
// hat die Peripherie bereits sauber initialisiert -- das reicht, wie der main()-Testcode ohne
// jeglichen Re-Init beweist. Trials=2/Timeout=2ms wie im main()-Testcode-Vorbild (nicht Trials=1
// wie im urspruenglichen, hier verworfenen HTTP-Endpunkt).
//
// log_debug() (nicht log_warn(): auf Bussen ohne jegliches Geraet/ohne Pull-ups, z.B. I2C2/I2C4
// auf dem Test-Rig, kippen mehrere Adressen am Scan-Anfang erwartungsgemaess kurz auf HAL_BUSY
// statt HAL_ERROR, bevor sich der Bus-Idle-Zustand einpendelt -- das ist Rauschen, kein
// Diagnosefall, und ueberschwemmte bei LOG_INFO-Pegel unnoetig das Boot-Log) bei jeder Adresse
// mit einem Status ausser HAL_OK/HAL_ERROR, damit sich ein nicht gefundenes Geraet bei Bedarf per
// log_set_level(LOG_DEBUG) trotzdem ueber das Boot-Log nachvollziehen laesst.
static void scan_i2c_bus(unsigned int bus_number, I2C_HandleTypeDef *hi2c, uint8_t out_bitfield[16]) {
    log_info("I2C%u: Boot-Scan gestartet (State=%d, ErrorCode=0x%08lX)",
              bus_number, (int)HAL_I2C_GetState(hi2c), (unsigned long)hi2c->ErrorCode);

    memset(out_bitfield, 0, 16);
    uint16_t found_count = 0;
    for (uint16_t addr = 0; addr < 128; addr++) {
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 2);
        if (status == HAL_OK) {
            out_bitfield[addr / 8] |= (uint8_t)(1u << (addr % 8));
            found_count++;
            log_info("I2C%u: Geraet gefunden bei Adresse 0x%02X", bus_number, (unsigned int)addr);
        } else if (status != HAL_ERROR) {
            log_debug("I2C%u: Adresse 0x%02X -- ungewoehnlicher Status %d (ErrorCode=0x%08lX)",
                       bus_number, (unsigned int)addr, (int)status, (unsigned long)hi2c->ErrorCode);
        }
        // Ohne diese Pause schlagen zurueckliegende Adressen (u.a. die echte 0x76) fehl, sobald
        // die 128 IsDeviceReady()-Aufrufe direkt aufeinanderfolgen -- reproduzierbar beobachtet:
        // bei log_set_level(LOG_DEBUG) (viele zusaetzliche, zeitraubende UART-Log-Ausgaben
        // zwischen den Aufrufen durch die dann sichtbaren Status-Anomalien) wird der Sensor
        // zuverlaessig gefunden, bei log_set_level(LOG_INFO) (kein Pausieren) nicht. Gleiches
        // Bugmuster wie beim TMC2209-Timeout (s. Commit-Historie: fehlende
        // Inter-Transaktions-Pause) -- deshalb hier eine explizite, von der Log-Ausgabe
        // unabhaengige Pause statt uns auf einen Zufallseffekt der Log-Verbosity zu verlassen.
        HAL_Delay(2);
    }

    log_info("I2C%u: Boot-Scan fertig -- %u Geraet(e) gefunden", bus_number, (unsigned int)found_count);
}

void PerformBootI2cScans() {
    App &app = App::Instance();
    scan_i2c_bus(1, &hi2c1, app.i2c1_scan);
    scan_i2c_bus(2, &hi2c2, app.i2c2_scan);
    scan_i2c_bus(4, &hi2c4, app.i2c4_scan);
}

// 404-Antwort fuer unbekannte /api/*-Pfade (s. handle_api_request()) -- ein Client-Bug oder
// veralteter API-Aufruf, kein Fall fuer den SPA-Fallback in send_spa_shell_response().
static UINT send_not_found_response(NX_WEB_HTTP_SERVER *server_ptr) {
    NX_PACKET *packet_ptr;
    if (nx_web_http_server_callback_generate_response_header(
            server_ptr, &packet_ptr, _C(NX_WEB_HTTP_STATUS_NOT_FOUND),
            0, _C("text/plain"), NX_NULL) != NX_SUCCESS) {
        return NX_NOT_SUCCESSFUL;
    }
    return (nx_web_http_server_callback_packet_send(server_ptr, packet_ptr) == NX_SUCCESS)
               ? NX_WEB_HTTP_CALLBACK_COMPLETED : NX_NOT_SUCCESSFUL;
}

// Alle Endpunkte unter "/api" -- fester, bekannter Satz. Eine unbekannte /api/*-URL bekommt
// bewusst ein 404 statt der Weboberflaeche (anders als jeder Nicht-/api-Pfad, s.
// webserver_request_callback()).
static UINT handle_api_request(NX_WEB_HTTP_SERVER *server_ptr, CHAR *resource, NX_PACKET *packet_ptr) {
    if (strcmp(resource, "/api/registers") == 0) {
        RegisterBinaryState state;
        state.register_model = App::Instance().register_model;
        UINT status = send_streamed_response(server_ptr, "application/octet-stream", NX_NULL,
                                             register_binary_total_length(), write_register_binary_chunk, &state);
        return (status == NX_SUCCESS) ? NX_WEB_HTTP_CALLBACK_COMPLETED : NX_NOT_SUCCESSFUL;
    }

    if (strcmp(resource, "/api/system") == 0) {
        SystemInfoState state;
        fill_system_info(state);
        UINT status = send_streamed_response(server_ptr, "application/octet-stream", NX_NULL,
                                             sizeof(state.buffer), write_system_info_chunk, &state);
        return (status == NX_SUCCESS) ? NX_WEB_HTTP_CALLBACK_COMPLETED : NX_NOT_SUCCESSFUL;
    }

    if (strcmp(resource, "/api/write-holding") == 0) {
        char response_data[512] = {0};
        char response_type[30] = {0};
        char addr_query[24] = {0};
        char value_query[24] = {0};
        UINT addr_query_size, value_query_size;
        unsigned int address = 0, value = 0;

        if (nx_web_http_server_query_get(packet_ptr, 0, addr_query, &addr_query_size, sizeof(addr_query) - 1) == NX_SUCCESS &&
            nx_web_http_server_query_get(packet_ptr, 1, value_query, &value_query_size, sizeof(value_query) - 1) == NX_SUCCESS &&
            sscanf(addr_query, "address=%u", &address) == 1 &&
            sscanf(value_query, "value=%u", &value) == 1 &&
            address <= ModbusRegisters::HOLDING_REGISTER_MAX_INDEX && value <= UINT16_MAX) {
            App::Instance().register_model->SetHoldingRegister((uint16_t)address, (uint16_t)value);
            sprintf(response_data, "OK");
        } else {
            sprintf(response_data, "ERROR");
        }

        UINT string_length;
        nx_web_http_server_type_get(server_ptr, resource, response_type, &string_length);
        response_type[string_length] = '\0';

        NX_PACKET *resp_packet;
        if (nx_web_http_server_callback_generate_response_header(
                server_ptr, &resp_packet, _C(NX_WEB_HTTP_STATUS_OK),
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

    return send_not_found_response(server_ptr);
}

// Liefert fuer "/" UND jeden Client-seitigen Router-Pfad (s. web/src/shell/router.ts -- History-
// API-Routing, z.B. "/system"/"/power", kein Hash-Routing) dieselbe Single-File-UI aus. Ein
// direkter Aufruf oder Reload einer solchen URL landet damit trotzdem in der App, die anhand von
// window.location.pathname selbst entscheidet, was sie anzeigt (klassischer SPA-Fallback).
static UINT send_spa_shell_response(NX_WEB_HTTP_SERVER *server_ptr) {
    size_t cursor = 0;
    size_t html_br_len = (size_t)(_binary_index_html_br_end - _binary_index_html_br_start);
    UINT status = send_streamed_response(server_ptr, "text/html", "Content-Encoding: br\r\n",
                                         html_br_len, write_from_flash_blob, &cursor);
    return (status == NX_SUCCESS) ? NX_WEB_HTTP_CALLBACK_COMPLETED : NX_NOT_SUCCESSFUL;
}

UINT webserver_request_callback(NX_WEB_HTTP_SERVER *server_ptr, UINT request_type,
                                 CHAR *resource, NX_PACKET *packet_ptr) {
    (void)request_type;

    if (strncmp(resource, "/api", 4) == 0) {
        return handle_api_request(server_ptr, resource, packet_ptr);
    }
    return send_spa_shell_response(server_ptr);
}
