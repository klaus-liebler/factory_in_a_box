// ============================================================================
// Routen-Implementierung fuer den HTTPS/WebSocket-Server (s. webserver.hpp/
// http_websocket_server.hpp). Bewusst kein JSON auf C++-Seite (weder Parser noch Encoder) --
// /api/registers liefert alle Register voll binaer (2 Byte Little-Endian je Register, s.
// register_binary_total_length()), /api/system liefert ein kompaktes festes Binaer-Struct (s.
// fill_system_info()), das u.a. auch das Ergebnis des einmaligen I2C-Geraete-Scans beim Boot
// enthaelt (s. PerformBootI2cScans(), aufgerufen aus Io::Setup()), /api/write-holding ist ein
// GET mit Query-Parametern statt POST+JSON-Body. Unbekannte /api/*-Pfade bekommen automatisch
// 404 von Http::WebServer (s. dortige DispatchRoute()), jeder andere Pfad die SPA-Shell.
// ============================================================================
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#include "webserver.hpp"
#include "app.hh"
#include "modbus_register_model.hh"
#include "assets.h"
#include "lan8742.h"
#include "task_monitor.hpp"
#include "generated/ws_protocol.hh"
#include "setup_and_loops/roarm_mission_gpio.hh"

// Aus sysmem.c -- gleiche Deklaration wie in io.cpp (FREE_HEAP_KIB-Register) genutzt, hier fuer
// den free_heap_bytes-Wert in /api/system.
extern "C" size_t GetFreeHeapBytes(void);

// Fuer PerformBootI2cScans() (s. weiter unten) -- dieselben drei Busse, die
// setup_and_loops/tof_color.hh und setup_and_loops/power.hh bereits nutzen (main.c
// MX_I2C{1,2,4}_Init()).
extern "C" I2C_HandleTypeDef hi2c1;
extern "C" I2C_HandleTypeDef hi2c2;
extern "C" I2C_HandleTypeDef hi2c4;

// Quelle fuer Response::SendStreamed(): der eincompilierte Brotli-Blob, context = Cursor (Byte-
// Offset), der zwischen Aufrufen weiterlaeuft.
static void write_from_flash_blob(void *context, char *dest, size_t want) {
    size_t *cursor = (size_t *)context;
    memcpy(dest, _binary_index_html_br_start + *cursor, want);
    *cursor += want;
}

// Quelle fuer Response::SendStreamed(): alle Holding- (Index 0..HOLDING_REGISTER_MAX_INDEX)
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

// Quelle fuer Response::SendStreamed(): ein einmalig (bei GET /api/system) zusammengestelltes,
// festes Binaer-Struct mit Systeminformationen -- klein genug, um komplett in einen lokalen
// Stack-Puffer geschrieben und in einem einzigen Chunk gesendet zu werden. Enthaelt bewusst NUR
// echte Laufzeitwerte -- Firmware-Version, Board-Name, Hostname, Chip-UID und MAC-Adressen sind
// Compile-Zeit-Konstanten (s. Core/Inc/constants.hh bzw. Core/generated/device_ids.hh) und werden
// stattdessen einmalig beim Web-Build nach web/generated/build-info.ts eincompiliert (s.
// builder/Phases/ReadGitStatus.cs) -- keine unnoetige Wire-Uebertragung bei jedem
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
    // LOG_INFO_ML/LOG_ML/LOG_ML_END (log.h): one locked block for the whole scan (up to ~256ms,
    // see the HAL_Delay(2) comment below) so another thread's log line can't land in the middle.
    // The log_debug() call below is deliberately left as a plain call, not LOG_ML(): an ML block
    // has only one level for its whole duration (set once at LOG_INFO_ML time), so promoting it
    // would make it always print instead of staying independently filterable at LOG_DEBUG.
    LOG_INFO_ML("I2C%u: Boot-Scan gestartet (State=%d, ErrorCode=0x%08lX)",
              bus_number, (int)HAL_I2C_GetState(hi2c), (unsigned long)hi2c->ErrorCode);

    memset(out_bitfield, 0, 16);
    uint16_t found_count = 0;
    for (uint16_t addr = 0; addr < 128; addr++) {
        HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 2);
        if (status == HAL_OK) {
            out_bitfield[addr / 8] |= (uint8_t)(1u << (addr % 8));
            found_count++;
            LOG_ML("I2C%u: Geraet gefunden bei Adresse 0x%02X", bus_number, (unsigned int)addr);
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

    LOG_ML("I2C%u: Boot-Scan fertig -- %u Geraet(e) gefunden", bus_number, (unsigned int)found_count);
    LOG_ML_END();
}

void PerformBootI2cScans() {
    App &app = App::Instance();
    scan_i2c_bus(1, &hi2c1, app.i2c1_scan);
    scan_i2c_bus(2, &hi2c2, app.i2c2_scan);
    scan_i2c_bus(4, &hi2c4, app.i2c4_scan);
}

// Sucht "key=value" in einer "&"-getrennten Query-String (s. /api/write-holding) und liefert den
// Wert als vorzeichenlose Ganzzahl. Ersetzt das bisherige nx_web_http_server_query_get()
// (dessen HTTP-Server-spezifische API mit dem neuen Http::WebServer entfaellt) durch eine
// schlanke eigene Suche direkt auf dem bereits '\0'-terminierten Request::query-String.
static bool find_query_uint(const char *query, const char *key, unsigned int *out_value) {
    if (!query) return false;
    size_t key_len = strlen(key);
    const char *p = query;
    while (*p) {
        const char *eq = strchr(p, '=');
        if (!eq) return false;
        size_t name_len = (size_t)(eq - p);
        const char *amp = strchr(eq, '&');
        size_t value_len = amp ? (size_t)(amp - eq - 1) : strlen(eq + 1);

        if (name_len == key_len && strncmp(p, key, key_len) == 0) {
            char buf[24] = {0};
            if (value_len >= sizeof(buf)) value_len = sizeof(buf) - 1;
            memcpy(buf, eq + 1, value_len);
            buf[value_len] = '\0';
            *out_value = (unsigned int)strtoul(buf, nullptr, 10);
            return true;
        }

        p = amp ? amp + 1 : eq + 1 + value_len;
    }
    return false;
}

static void handle_registers(void *, const Http::Request &, Http::Response &resp) {
    RegisterBinaryState state;
    state.register_model = App::Instance().register_model;
    resp.SendStreamed(200, "application/octet-stream", nullptr,
                       register_binary_total_length(), write_register_binary_chunk, &state);
}

static void handle_system(void *, const Http::Request &, Http::Response &resp) {
    SystemInfoState state;
    fill_system_info(state);
    resp.SendStreamed(200, "application/octet-stream", nullptr,
                       sizeof(state.buffer), write_system_info_chunk, &state);
}

static void handle_write_holding(void *, const Http::Request &req, Http::Response &resp) {
    unsigned int address = 0, value = 0;
    const char *response_text = "ERROR";

    if (find_query_uint(req.query, "address", &address) &&
        find_query_uint(req.query, "value", &value) &&
        address <= ModbusRegisters::HOLDING_REGISTER_MAX_INDEX && value <= UINT16_MAX) {
        App::Instance().register_model->SetHoldingRegister((uint16_t)address, (uint16_t)value);
        response_text = "OK";
    }

    resp.SendFixed(200, "OK", "text/plain", nullptr, (const uint8_t *)response_text, strlen(response_text));
}

// Liefert fuer "/" UND jeden Client-seitigen Router-Pfad (s. web/src/shell/router.ts -- History-
// API-Routing, z.B. "/system"/"/power", kein Hash-Routing) dieselbe Single-File-UI aus. Ein
// direkter Aufruf oder Reload einer solchen URL landet damit trotzdem in der App, die anhand von
// window.location.pathname selbst entscheidet, was sie anzeigt (klassischer SPA-Fallback).
// Registriert als Http::WebServer-Default-Handler, greift also fuer jeden Pfad, der nicht mit
// dem API-Praefix beginnt und keiner expliziten Route entspricht.
static void handle_spa_shell(void *, const Http::Request &, Http::Response &resp) {
    size_t cursor = 0;
    size_t html_br_len = (size_t)(_binary_index_html_br_end - _binary_index_html_br_start);
    resp.SendStreamed(200, "text/html", "Content-Encoding: br\r\n", html_br_len, write_from_flash_blob, &cursor);
}

// WebSocket-Endpunkt "/ws" -- Anwendungsnachrichten (s. best_binary_buffers_schema/*.cs) werden
// hier anhand des 4-Byte-Kopfes (namespaceId/messageTypeId) ausgewertet, symmetrisch zum
// client-seitigen Dispatch in web/src/ws-client.ts::handleMessage(). Zwei Namespaces mit
// eingehenden (Client->Firmware) Nachrichten sind verdrahtet: "roarm" (Teach-Modus/Jogging/
// Mission-Verwaltung, geroutet an App::Instance().io->RoArm(), s. setup_and_loops/roarm.hh) und
// "tasks" (tasks.TaskManagerRequest, s. task_monitor.cpp). "system" hat nur ausgehende Nachrichten
// (LogMessage, s. ws_log_bridge.cpp) und wird hier nie empfangen. Alles andere faellt weiterhin
// auf das reine Transport-Echo zurueck.
static void handle_ws_open(void *, Http::WebSocketConnection &) {
    log_info("WebSocket: Verbindung geoeffnet");
}

namespace {

// Scratch-Puffer fuers (De-)Serialisieren -- getrennt, da z.B. bei GetMissionRequest die
// dekodierten Mission::Payload-Zeiger (name/stepsData) noch in mission_load_scratch liegen,
// waehrend die ausgehende GetMissionResponse gleichzeitig in ws_response_scratch aufgebaut wird.
// NICHT mit RoArmSetupAndLoop::missionLoadBuffer_ zusammengelegt -- s. dortiger Kommentar (echtes
// Cross-Thread-Race zwischen StartMission()/io_thread und diesem Webserver-Thread).
//
// Alle drei waren vormals grosszuegig geraten (4096/4300/4096 Byte); jetzt exakt anhand der von
// BestBinaryBuffers generierten constexpr <Message>_MAX_SIZE-Konstanten dimensioniert (s.
// best_binary_buffers_schema/*.cs [BinaryMaxEncodedByteLength]/[BinaryMaxItemCount]-Bounds).
uint8_t g_mission_load_scratch[WsProtocol::roarm::Mission::Mission_MAX_SIZE];
// Groesste ueber diesen Puffer kodierte Nachricht ist ListMissionsResponse (1160 B) -- alle
// anderen hier kodierten Antworten (StartTeachModeResponse/StopTeachModeResponse/GetMissionResponse/
// SaveMissionResponse/DeleteMissionResponse/GetMissionGpioListResponse) sind kleiner. Bei einer
// neuen, noch groesseren Nachricht auf diesem Pfad hier den Maximalwert nachziehen.
uint8_t g_ws_response_scratch[WsProtocol::roarm::ListMissionsResponse::ListMissionsResponse_MAX_SIZE];
// Haelt nur die Roh-Element-Bytes VOR dem finalen Encode() (ListMissionsResponse.missions/
// GetMissionGpioListResponse.names) -- kann nie groesser werden als die sie umschliessende
// Nachricht, daher derselbe (sichere Ober-)Grenzwert wie oben.
uint8_t g_ws_build_scratch[WsProtocol::roarm::ListMissionsResponse::ListMissionsResponse_MAX_SIZE];

RoArmSetupAndLoop *GetRoArm() {
    App &app = App::Instance();
    return app.io ? &app.io->RoArm() : nullptr;
}

void HandleStartTeachMode(Http::WebSocketConnection &conn, const uint8_t *data, size_t len) {
    WsProtocol::roarm::StartTeachModeRequest::Payload req{};
    if (!WsProtocol::roarm::StartTeachModeRequest::Decode(data, len, req)) return;
    RoArmSetupAndLoop *roarm = GetRoArm();

    WsProtocol::roarm::StartTeachModeResponse::Payload resp{};
    resp.requestId = req.requestId;
    resp.success = roarm && roarm->StartTeachMode();
    size_t n = WsProtocol::roarm::StartTeachModeResponse::Encode(resp, g_ws_response_scratch, sizeof(g_ws_response_scratch));
    if (n > 0) conn.SendBinary(g_ws_response_scratch, n);
}

void HandleStopTeachMode(Http::WebSocketConnection &conn, const uint8_t *data, size_t len) {
    WsProtocol::roarm::StopTeachModeRequest::Payload req{};
    if (!WsProtocol::roarm::StopTeachModeRequest::Decode(data, len, req)) return;
    RoArmSetupAndLoop *roarm = GetRoArm();
    if (roarm) roarm->StopTeachMode();

    WsProtocol::roarm::StopTeachModeResponse::Payload resp{};
    resp.requestId = req.requestId;
    resp.success = roarm != nullptr;
    size_t n = WsProtocol::roarm::StopTeachModeResponse::Encode(resp, g_ws_response_scratch, sizeof(g_ws_response_scratch));
    if (n > 0) conn.SendBinary(g_ws_response_scratch, n);
}

// JointJogTarget/CartesianJogTarget sind Events (kein Request/Response-Umlauf, s. Schema-
// Kommentar) -- absichtlich keine Antwort, auch bei ungueltigem/abgelehntem Ziel (RoArmSetupAndLoop
// ignoriert ein Ziel dann einfach, s. dortige Guards).
void HandleJointJogTarget(const uint8_t *data, size_t len) {
    WsProtocol::roarm::JointJogTarget::Payload msg{};
    if (!WsProtocol::roarm::JointJogTarget::Decode(data, len, msg)) return;
    RoArmSetupAndLoop *roarm = GetRoArm();
    if (!roarm) return;
    std::array<int16_t, RoArmKinematics::kJointCount> centiDeg{};
    for (int i = 0; i < RoArmKinematics::kJointCount; i++) centiDeg[i] = msg.jointAnglesCentiDeg[i];
    roarm->SetJointJogTargetCentiDeg(centiDeg);
}

void HandleCartesianJogTarget(const uint8_t *data, size_t len) {
    WsProtocol::roarm::CartesianJogTarget::Payload msg{};
    if (!WsProtocol::roarm::CartesianJogTarget::Decode(data, len, msg)) return;
    RoArmSetupAndLoop *roarm = GetRoArm();
    if (!roarm) return;
    RoArmKinematics::CartesianPose pose;
    pose.xMm = msg.xMm;
    pose.yMm = msg.yMm;
    pose.zMm = msg.zMm;
    pose.pitchRad = RoArmKinematics::CentiDegToRad(msg.pitchCentiDeg);
    pose.rollRad = RoArmKinematics::CentiDegToRad(msg.rollCentiDeg);
    pose.gripperRad = RoArmKinematics::CentiDegToRad(msg.gripperCentiDeg);
    roarm->SetCartesianJogTarget(pose);
}

void HandleListMissions(Http::WebSocketConnection &conn, const uint8_t *data, size_t len) {
    WsProtocol::roarm::ListMissionsRequest::Payload req{};
    if (!WsProtocol::roarm::ListMissionsRequest::Decode(data, len, req)) return;
    RoArmSetupAndLoop *roarm = GetRoArm();

    RoArmMissionStore::MissionSummary list[32];
    size_t count = 0;
    if (roarm) roarm->ListMissions(list, 32, count);

    size_t buildPos = 0;
    for (size_t i = 0; i < count; i++) {
        WsProtocol::roarm::MissionSummary::Payload item{};
        item.missionIndex = list[i].index;
        item.name = list[i].name;
        size_t next = WsProtocol::roarm::AppendListMissionsResponseMissionsMissionSummaryElement(item, g_ws_build_scratch, buildPos,
                                                                                                   sizeof(g_ws_build_scratch));
        if (next == 0) break; // Puffer voll -- Rest der Liste stillschweigend abschneiden
        buildPos = next;
    }

    WsProtocol::roarm::ListMissionsResponse::Payload resp{};
    resp.requestId = req.requestId;
    resp.missionsData = g_ws_build_scratch;
    resp.missionsCount = count;
    resp.missionsDataSize = buildPos;
    size_t n = WsProtocol::roarm::ListMissionsResponse::Encode(resp, g_ws_response_scratch, sizeof(g_ws_response_scratch));
    if (n > 0) conn.SendBinary(g_ws_response_scratch, n);
}

void HandleGetMission(Http::WebSocketConnection &conn, const uint8_t *data, size_t len) {
    WsProtocol::roarm::GetMissionRequest::Payload req{};
    if (!WsProtocol::roarm::GetMissionRequest::Decode(data, len, req)) return;
    RoArmSetupAndLoop *roarm = GetRoArm();

    WsProtocol::roarm::GetMissionResponse::Payload resp{};
    resp.requestId = req.requestId;
    resp.missionIndex = req.missionIndex;
    resp.found = false;
    resp.name = "";
    resp.stepsData = nullptr;
    resp.stepsCount = 0;
    resp.stepsDataSize = 0;

    size_t rawSize = 0;
    WsProtocol::roarm::Mission::Payload mission{};
    if (roarm && roarm->LoadMissionRaw(req.missionIndex, g_mission_load_scratch, sizeof(g_mission_load_scratch), rawSize) &&
        WsProtocol::roarm::Mission::Decode(g_mission_load_scratch, rawSize, mission)) {
        // Das Wire-Format der Mission-Schritte (Datei-Inhalt) und von GetMissionResponse.steps
        // sind byteidentisch ([classId:u16][Klassenfelder]*) -- direkte Weitergabe ohne
        // Decode+Reencode-Umweg.
        resp.found = true;
        resp.name = mission.name;
        resp.stepsData = mission.stepsData;
        resp.stepsCount = mission.stepsCount;
        resp.stepsDataSize = mission.stepsDataSize;
    }

    size_t n = WsProtocol::roarm::GetMissionResponse::Encode(resp, g_ws_response_scratch, sizeof(g_ws_response_scratch));
    if (n > 0) conn.SendBinary(g_ws_response_scratch, n);
}

void HandleSaveMission(Http::WebSocketConnection &conn, const uint8_t *data, size_t len) {
    WsProtocol::roarm::SaveMissionRequest::Payload req{};
    if (!WsProtocol::roarm::SaveMissionRequest::Decode(data, len, req)) return;
    RoArmSetupAndLoop *roarm = GetRoArm();

    WsProtocol::roarm::SaveMissionResponse::Payload resp{};
    resp.requestId = req.requestId;
    // Dieselbe Byteidentitaet wie bei GetMissionResponse: req.stepsData ist bereits exakt das
    // Format, das roarm_mission_store als Dateiinhalt erwartet (s. RoArmSetupAndLoop::SaveMission()).
    if (roarm && roarm->SaveMission(req.missionIndex, req.name, req.stepsData, req.stepsDataSize, req.stepsCount)) {
        resp.success = true;
        resp.errorCode = 0;
    } else {
        resp.success = false;
        resp.errorCode = -1;
    }
    size_t n = WsProtocol::roarm::SaveMissionResponse::Encode(resp, g_ws_response_scratch, sizeof(g_ws_response_scratch));
    if (n > 0) conn.SendBinary(g_ws_response_scratch, n);
}

void HandleDeleteMission(Http::WebSocketConnection &conn, const uint8_t *data, size_t len) {
    WsProtocol::roarm::DeleteMissionRequest::Payload req{};
    if (!WsProtocol::roarm::DeleteMissionRequest::Decode(data, len, req)) return;
    RoArmSetupAndLoop *roarm = GetRoArm();

    WsProtocol::roarm::DeleteMissionResponse::Payload resp{};
    resp.requestId = req.requestId;
    resp.success = roarm && roarm->DeleteMission(req.missionIndex);
    size_t n = WsProtocol::roarm::DeleteMissionResponse::Encode(resp, g_ws_response_scratch, sizeof(g_ws_response_scratch));
    if (n > 0) conn.SendBinary(g_ws_response_scratch, n);
}

void HandleGetMissionGpioList(Http::WebSocketConnection &conn, const uint8_t *data, size_t len) {
    WsProtocol::roarm::GetMissionGpioListRequest::Payload req{};
    if (!WsProtocol::roarm::GetMissionGpioListRequest::Decode(data, len, req)) return;

    size_t buildPos = 0;
    for (const auto &def : RoArmMissionGpio::kGpios) {
        size_t nameLen = strlen(def.name);
        if (buildPos + nameLen + 1 > sizeof(g_ws_build_scratch)) break;
        memcpy(g_ws_build_scratch + buildPos, def.name, nameLen);
        buildPos += nameLen;
        g_ws_build_scratch[buildPos++] = 0;
    }

    WsProtocol::roarm::GetMissionGpioListResponse::Payload resp{};
    resp.requestId = req.requestId;
    resp.namesData = g_ws_build_scratch;
    resp.namesCount = RoArmMissionGpio::kCount;
    resp.namesDataSize = buildPos;
    size_t n = WsProtocol::roarm::GetMissionGpioListResponse::Encode(resp, g_ws_response_scratch, sizeof(g_ws_response_scratch));
    if (n > 0) conn.SendBinary(g_ws_response_scratch, n);
}

} // namespace

static void handle_ws_message(void *, Http::WebSocketConnection &conn, bool is_binary,
                               const uint8_t *data, size_t len) {
    if (is_binary && len >= 4) {
        uint16_t namespace_id = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
        uint16_t type_id = (uint16_t)data[2] | ((uint16_t)data[3] << 8);

        if (namespace_id == WsProtocol::roarm::NAMESPACE_ID) {
            switch (type_id) {
            case WsProtocol::roarm::StartTeachModeRequest::TYPE_ID: HandleStartTeachMode(conn, data, len); break;
            case WsProtocol::roarm::StopTeachModeRequest::TYPE_ID: HandleStopTeachMode(conn, data, len); break;
            case WsProtocol::roarm::JointJogTarget::TYPE_ID: HandleJointJogTarget(data, len); break;
            case WsProtocol::roarm::CartesianJogTarget::TYPE_ID: HandleCartesianJogTarget(data, len); break;
            case WsProtocol::roarm::ListMissionsRequest::TYPE_ID: HandleListMissions(conn, data, len); break;
            case WsProtocol::roarm::GetMissionRequest::TYPE_ID: HandleGetMission(conn, data, len); break;
            case WsProtocol::roarm::SaveMissionRequest::TYPE_ID: HandleSaveMission(conn, data, len); break;
            case WsProtocol::roarm::DeleteMissionRequest::TYPE_ID: HandleDeleteMission(conn, data, len); break;
            case WsProtocol::roarm::GetMissionGpioListRequest::TYPE_ID: HandleGetMissionGpioList(conn, data, len); break;
            default: break; // unbekannte/nur ausgehende Nachrichtentypen -- stillschweigend ignorieren
            }
            return;
        }

        if (namespace_id == WsProtocol::tasks::NAMESPACE_ID &&
            type_id == WsProtocol::tasks::TaskManagerRequest::TYPE_ID) {
            WsProtocol::tasks::TaskManagerRequest::Payload request{};
            if (WsProtocol::tasks::TaskManagerRequest::Decode(data, len, request)) {
                HandleTaskManagerRequest(conn, request.requestId);
            }
            return;
        }
    }

    if (is_binary) {
        conn.SendBinary(data, len);
    } else {
        conn.SendText((const char *)data, len);
    }
}

static void handle_ws_close(void *, Http::WebSocketConnection &) {
    log_info("WebSocket: Verbindung geschlossen");
}

// ~10Hz (100ms) waehrend Teach-Modus aktiv -- s. Deklaration in webserver.hpp fuer die
// Begruendung, warum das hier statt in roarm.hh lebt. Broadcast() ist von jedem Thread aufrufbar
// (s. Http::WebServer::Broadcast()-Kommentar), Aufruf erfolgt aus dem io_thread heraus
// (Io::processMembers()).
void RoArmBroadcastPoseFeedbackIfDue(uint32_t now) {
    static uint32_t last_broadcast_ms = 0;
    App &app = App::Instance();
    if (!app.io) return;
    RoArmSetupAndLoop &roarm = app.io->RoArm();
    if (!roarm.IsTeachModeActive()) return;
    if (now - last_broadcast_ms < 100) return;
    last_broadcast_ms = now;

    WsProtocol::roarm::PoseFeedback::Payload payload = roarm.GetPoseFeedback();
    uint8_t buffer[64];
    size_t n = WsProtocol::roarm::PoseFeedback::Encode(payload, buffer, sizeof(buffer));
    if (n > 0) app.https_server.Broadcast(buffer, n);
}

void webserver_register_routes(Http::WebServer &server) {
    server.RegisterRoute(Http::Method::GET, "/api/registers", true, handle_registers, nullptr);
    server.RegisterRoute(Http::Method::GET, "/api/system", true, handle_system, nullptr);
    server.RegisterRoute(Http::Method::GET, "/api/write-holding", true, handle_write_holding, nullptr);
    server.SetDefaultHandler(handle_spa_shell, nullptr);

    server.SetWebSocketPath("/ws");
    server.SetWebSocketHandlers(handle_ws_open, handle_ws_message, handle_ws_close, nullptr);
}
