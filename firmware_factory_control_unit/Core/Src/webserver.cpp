// ============================================================================
// Modbus-Register-Weboberflaeche -- liefert die unter web/ gebaute, minifizierte und Brotli-
// komprimierte Single-File-UI (per objcopy ins Flash einkompiliert, siehe assets/index.html.br
// und CMakeLists.txt BINARY_ASSETS) fuer JEDEN Pfad AUSSER "/api/*" aus (SPA-Fallback fuer den
// History-API-Router der UI, s. web/src/shell/router.ts -- "/system"/"/power" muessen serverseitig
// dieselbe index.html.br liefern wie "/"), sowie schlanke Endpunkte zum Lesen/Schreiben aller
// Register und zum Abfragen von System-/I2C-Bus-Informationen. Bewusst kein JSON auf C++-Seite
// (weder Parser noch Encoder) -- /api/registers liefert alle Register voll binaer (2 Byte
// Little-Endian je Register, s. register_binary_total_length()), /api/system liefert ein
// kompaktes festes Binaer-Struct (s. write_system_info_chunk()), /api/i2c liefert eine
// Geraete-Discovery ueber alle drei I2C-Busse als Bitfeld (s. perform_i2c_scan()),
// /api/write-holding ist ein GET mit Query-Parametern statt POST+JSON-Body (erspart
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

// Fuer /api/i2c (s. perform_i2c_scan() weiter unten) -- dieselben drei Busse, die
// setup_and_loops/tof_color.hh und setup_and_loops/power.hh bereits nutzen (main.c
// MX_I2C{1,2,4}_Init()).
extern "C" I2C_HandleTypeDef hi2c1;
extern "C" I2C_HandleTypeDef hi2c2;
extern "C" I2C_HandleTypeDef hi2c4;
// Schuetzt hi2c1/hi2c2/hi2c4 vor gleichzeitigem Zugriff aus Io::Loop() (periodisches
// Sensor-Polling, s. io.cpp) waehrend eines /api/i2c-Scans, s. Kommentar bei der Definition in
// app.cc.
extern TX_MUTEX i2c_bus_mutex;
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

// LAN8742-PHY-Diagnose fuer /api/system (s. SystemInfoState-Layout unten): passive Register-Reads
// (Link/Speed/Duplex/ENERGYON/Symbolfehler/Auto-MDIX/Polaritaet) PLUS eine aktiv ausgeloeste
// TDR-Kabeldiagnose (Kurzschluss/Unterbrechung/Angepasst + grobe Fehlerstellen-/Kabellaenge).
//
// ETH_PHY_ADDRESS = 0: nicht per CubeMX generiert (dieses Projekt nutzt keinen ST-BSP-ethernetif.c-
// Layer, s. Datei-Kommentar oben) -- Standardadresse fuer LAN8742 auf den meisten STM32-Nucleo-144-
// Boards (Hardware-Strap). Bislang las kein Code-Pfad in diesem Projekt ueberhaupt MDIO-Register
// (Ethernet lief rein MAC-seitig ohne PHY-Statusabfrage) -- falls diese Adresse fuer die reale
// Factory-Control-Unit-Platine nicht stimmt, liefert HAL_ETH_ReadPHYRegister() lediglich einen
// Timeout (kein Hardware-Risiko), sichtbar an read_ok == false.
constexpr uint32_t ETH_PHY_ADDRESS = 0;

struct PhyDiagnostics {
    bool read_ok = false;
    bool link_up = false;
    bool autonego_done = false;
    bool energy_detected = false;
    bool auto_mdix_enabled = false;
    bool polarity_reversed = false;
    uint8_t speed_duplex_code = 0xFF; // 0=10 HD, 1=10 FD, 2=100 HD, 3=100 FD, 0xFF=unbekannt
    uint16_t symbol_error_count = 0;
    bool tdr_available = false;       // TDR-Schreib-/Lesezugriff erfolgreich (unabhaengig von read_ok)
    bool tdr_status = false;          // TCSR.TDR_CH_STATUS -- laut Datenblatt "Messung gueltig"
    uint8_t cable_type_code = 0;      // TCSR.TDR_CH_CABLE_TYPE: 0=default/unklar,1=shorted,2=open,3=match
    uint8_t tdr_length_raw = 0;       // TCSR.TDR_CH_LENGTH, roh (0-255) -- s. Kommentar bei read_tdr_diagnostics()
    uint8_t cable_length_class = 0;   // CLR.CABLE_LENGTH, 4-Bit-Klasse (0-15), nur bei bestehendem Link aussagekraeftig
};

// AKTIVE Messung: schreibt TCSR.TDR_ENABLE, wartet kurz auf die Pulsreflexions-Messung und liest
// Kabeltyp/Status/Laenge zurueck. TDR_CH_LENGTH ist laut LAN8742-Registerbeschreibung ein roher,
// unkalibrierter 8-Bit-Wert (keine direkte Meter-Umrechnungskonstante dokumentiert) -- hier bewusst
// als relativer Rohwert durchgereicht statt einer erfundenen Meterangabe. CLR.CABLE_LENGTH ist eine
// zweite, groebere 4-Bit-Klassifizierung, die laut Datenblatt nur bei bestehendem Link (nicht bei
// TDR) aussagekraeftig ist -- beide Werte werden deshalb parallel geliefert statt nur einer.
// HAL_Delay(2): empirisch gewaehlte kurze Wartezeit (analog zum TMC2209-Inter-Transaction-Delay,
// s. Projekt-Historie) -- falls Messwerte in der Praxis noch "stale" wirken, hier zuerst ansetzen.
static void read_tdr_diagnostics(PhyDiagnostics &d) {
    if (HAL_ETH_WritePHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_TCSR, LAN8742_TCSR_TDR_ENABLE) != HAL_OK) {
        return;
    }
    HAL_Delay(2);

    uint32_t tcsr = 0;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_TCSR, &tcsr) != HAL_OK) {
        return;
    }
    d.tdr_available = true;
    d.tdr_status = (tcsr & LAN8742_TCSR_TDR_CH_STATUS) != 0;
    d.cable_type_code = (uint8_t)((tcsr & LAN8742_TCSR_TDR_CH_CABLE_TYPE) >> 9);
    d.tdr_length_raw = (uint8_t)(tcsr & LAN8742_TCSR_TDR_CH_LENGTH);

    uint32_t clr = 0;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_CLR, &clr) == HAL_OK) {
        d.cable_length_class = (uint8_t)((clr & LAN8742_CLR_CABLE_LENGTH) >> 12);
    }
}

// SECR (Symbol Error Counter) loescht sich laut LAN8742-Datenblatt beim Lesen selbst -- der hier
// gelesene Wert ist deshalb der Zaehlerstand SEIT DEM LETZTEN GET /api/system, nicht seit
// Systemstart.
static PhyDiagnostics read_phy_diagnostics() {
    PhyDiagnostics d;
    uint32_t bsr = 0, physcsr = 0, mcsr = 0, secr = 0, scsir = 0;

    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_BSR, &bsr) != HAL_OK) return d;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_PHYSCSR, &physcsr) != HAL_OK) return d;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_MCSR, &mcsr) != HAL_OK) return d;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_SECR, &secr) != HAL_OK) return d;
    if (HAL_ETH_ReadPHYRegister(&heth, ETH_PHY_ADDRESS, LAN8742_SCSIR, &scsir) != HAL_OK) return d;

    d.read_ok = true;
    d.link_up = (bsr & LAN8742_BSR_LINK_STATUS) != 0;
    d.autonego_done = (physcsr & LAN8742_PHYSCSR_AUTONEGO_DONE) != 0;
    d.energy_detected = (mcsr & LAN8742_MCSR_ENERGYON) != 0;
    d.auto_mdix_enabled = (scsir & LAN8742_SCSIR_AUTO_MDIX_ENABLE) != 0;
    d.polarity_reversed = (scsir & LAN8742_SCSIR_XPOLALITY) != 0;
    d.symbol_error_count = (uint16_t)secr;

    switch (physcsr & LAN8742_PHYSCSR_HCDSPEEDMASK) {
        case LAN8742_PHYSCSR_10BT_HD:   d.speed_duplex_code = 0; break;
        case LAN8742_PHYSCSR_10BT_FD:   d.speed_duplex_code = 1; break;
        case LAN8742_PHYSCSR_100BTX_HD: d.speed_duplex_code = 2; break;
        case LAN8742_PHYSCSR_100BTX_FD: d.speed_duplex_code = 3; break;
        default: d.speed_duplex_code = 0xFF; break;
    }

    read_tdr_diagnostics(d);
    return d;
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
//       17      1  phy_flags               (uint8 Bitfeld, s. PhyDiagnostics/read_phy_diagnostics():
//                                            Bit0 link_up, Bit1 autonego_done, Bit2 energy_detected,
//                                            Bit3 auto_mdix_enabled, Bit4 polarity_reversed,
//                                            Bit5 read_ok, Bit6 tdr_available, Bit7 tdr_status --
//                                            Bit0..4 nur aussagekraeftig wenn Bit5 gesetzt,
//                                            phy_cable_type_code/phy_tdr_length_raw nur wenn Bit6)
//       18      1  phy_speed_duplex_code   (uint8, s. PhyDiagnostics::speed_duplex_code)
//       19      2  phy_symbol_error_count  (uint16, s. read_phy_diagnostics()-Kommentar zu SECR)
//       21      1  phy_cable_type_code     (uint8, s. PhyDiagnostics::cable_type_code)
//       22      1  phy_tdr_length_raw      (uint8, s. read_tdr_diagnostics()-Kommentar)
//       23      1  phy_cable_length_class  (uint8, 0-15, s. PhyDiagnostics::cable_length_class)
//       24          Gesamtlaenge
struct SystemInfoState {
    uint8_t buffer[24];
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

    PhyDiagnostics phy = read_phy_diagnostics();
    uint8_t phy_flags = 0;
    if (phy.link_up) phy_flags |= 0x01;
    if (phy.autonego_done) phy_flags |= 0x02;
    if (phy.energy_detected) phy_flags |= 0x04;
    if (phy.auto_mdix_enabled) phy_flags |= 0x08;
    if (phy.polarity_reversed) phy_flags |= 0x10;
    if (phy.read_ok) phy_flags |= 0x20;
    if (phy.tdr_available) phy_flags |= 0x40;
    if (phy.tdr_status) phy_flags |= 0x80;
    p[17] = phy_flags;
    p[18] = phy.speed_duplex_code;
    put_u16(p + 19, phy.symbol_error_count);
    p[21] = phy.cable_type_code;
    p[22] = phy.tdr_length_raw;
    p[23] = phy.cable_length_class;

    st.pos = 0;
}

static void write_system_info_chunk(void *context, char *dest, size_t want) {
    SystemInfoState *st = (SystemInfoState *)context;
    memcpy(dest, st->buffer + st->pos, want);
    st->pos += want;
}

// I2C-Geraete-Discovery fuer /api/i2c: scannt alle 128 7-Bit-Adressen je Bus per
// HAL_I2C_IsDeviceReady() und liefert das Ergebnis als kompaktes Bitfeld (16 Byte = 128 Bit pro
// Bus, Bit N = Adresse N, LSB zuerst) statt eines 128-Byte-Arrays -- die Web-App kennt die
// Adress-Reihenfolge bereits (s. web/src/i2c-device-names.ts) und muss nur ja/nein pro Adresse
// wissen.
struct I2cScanState {
    uint8_t buffer[48]; // 3 Busse x 16 Byte
    size_t pos = 0;
};

// Trials=1/Timeout=2ms halten einen einzelnen Adressversuch kurz (128 Adressen * 3 Busse binnen
// eines Wimpernschlags) -- ein Scan ist trotzdem eine bewusste, seltene Nutzeraktion ("Scan
// starten" im Web-UI), kein Kandidat fuer periodisches Polling.
//
// HAL_I2C_DeInit()+HAL_I2C_Init() (Reclaim) NACH jedem Bus-Scan ist kein Sicherheitstheater:
// ina226.cpp dokumentiert ausfuehrlich, dass ein NACK auf I2C4 (z.B. TOF3, auf dieser
// Platinen-Revision nicht bestueckt) den GESAMTEN Bus fuer alle folgenden Transaktionen lahmlegt,
// obwohl HAL_I2C_GetState() weiterhin READY meldet -- ein 128-Adressen-Scan produziert garantiert
// etliche NACKs. Ohne diesen Reclaim wuerde ein einziger /api/i2c-Aufruf das laufende
// INA226/TOF-Sensor-Polling in Io::Loop() dauerhaft zerschiessen, bis zum naechsten Reset.
static void scan_i2c_bus(I2C_HandleTypeDef *hi2c, uint8_t out_bitfield[16]) {
    memset(out_bitfield, 0, 16);
    for (uint16_t addr = 0; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 1, 2) == HAL_OK) {
            out_bitfield[addr / 8] |= (uint8_t)(1u << (addr % 8));
        }
    }
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
}

// Haelt i2c_bus_mutex fuer die GESAMTE Scandauer (alle drei Busse) statt bus-/adressweise --
// einfacher korrekt als feingranulares Locking, fuer eine seltene On-Demand-Aktion unkritisch.
static void perform_i2c_scan(uint8_t out[48]) {
    tx_mutex_get(&i2c_bus_mutex, TX_WAIT_FOREVER);
    scan_i2c_bus(&hi2c1, out + 0);
    scan_i2c_bus(&hi2c2, out + 16);
    scan_i2c_bus(&hi2c4, out + 32);
    tx_mutex_put(&i2c_bus_mutex);
}

static void write_i2c_scan_chunk(void *context, char *dest, size_t want) {
    I2cScanState *st = (I2cScanState *)context;
    memcpy(dest, st->buffer + st->pos, want);
    st->pos += want;
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

    if (strcmp(resource, "/api/i2c") == 0) {
        I2cScanState state;
        perform_i2c_scan(state.buffer);
        UINT status = send_streamed_response(server_ptr, "application/octet-stream", NX_NULL,
                                             sizeof(state.buffer), write_i2c_scan_chunk, &state);
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
