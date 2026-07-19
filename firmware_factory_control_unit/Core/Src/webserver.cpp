// ============================================================================
// Modbus-Register-Weboberflaeche -- liefert die unter web/ gebaute, minifizierte und Brotli-
// komprimierte Single-File-UI (per objcopy ins Flash einkompiliert, siehe assets/index.html.br
// und CMakeLists.txt BINARY_ASSETS) unter "/" aus, sowie schlanke Endpunkte zum Lesen/Schreiben
// aller Register und zum Abfragen von Systeminformationen. Bewusst kein JSON auf C++-Seite
// (weder Parser noch Encoder) -- /api/registers liefert alle Register voll binaer (2 Byte
// Little-Endian je Register, s. register_binary_total_length()), /api/system liefert ein
// kompaktes festes Binaer-Struct (s. write_system_info_chunk()), /api/write-holding ist ein GET
// mit Query-Parametern statt POST+JSON-Body (erspart nx_web_http_server_content_get()+Parser
// fuer diese Demo-UI).
// ============================================================================
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "webserver.hpp"
#include "app.hh"
#include "modbus_register_model.hh"
#include "generated/device_hostname.hh"
#include "assets.h"

// Aus sysmem.c -- gleiche Deklaration wie in io.cpp (FREE_HEAP_KIB-Register) genutzt, hier fuer
// den free_heap_bytes-Wert in /api/system.
extern "C" size_t GetFreeHeapBytes(void);
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

// Quelle fuer send_streamed_response(): ein einmalig (bei GET /api/system) zusammengestelltes,
// festes Binaer-Struct mit Systeminformationen -- klein genug, um komplett in einen lokalen
// Stack-Puffer geschrieben und in einem einzigen Chunk gesendet zu werden. Layout (alle
// Mehrbyte-Felder Little-Endian, s. RegisterBinaryState-Kommentar oben), von system-info-app.ts
// per DataView an exakt denselben Offsets wieder ausgelesen:
//   Offset  Bytes  Feld
//        0      2  fw_version_major        (uint16)
//        2      2  fw_version_minor        (uint16)
//        4      2  fw_version_patch        (uint16)
//        6      4  uptime_seconds          (uint32)
//       10      4  free_heap_bytes         (uint32)
//       14      4  ip_address              (4x uint8, MSB-Oktett zuerst, wie IP_ADDR_FMT_ARGS)
//       18      4  net_mask                (4x uint8, MSB-Oktett zuerst)
//       22      6  mac_address             (6x uint8)
//       28     12  chip_uid                (3x uint32)
//       40      1  reset_cause_code        (uint8, s. App::ResetCauseCode())
//       41     32  hostname                (ASCII, nullgepolstert)
//       73     48  board_name              (ASCII, nullgepolstert)
//      121          Gesamtlaenge
struct SystemInfoState {
    uint8_t buffer[121];
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

static void put_padded_string(uint8_t *dest, size_t field_len, const char *s, size_t s_len) {
    memset(dest, 0, field_len);
    if (s_len > field_len) s_len = field_len;
    memcpy(dest, s, s_len);
}

static void fill_system_info(SystemInfoState &st) {
    App &app = App::Instance();
    uint8_t *p = st.buffer;

    put_u16(p + 0, FW_VERSION_MAJOR);
    put_u16(p + 2, FW_VERSION_MINOR);
    put_u16(p + 4, FW_VERSION_PATCH);
    put_u32(p + 6, (uint32_t)(tx_time_get() / TX_TIMER_TICKS_PER_SECOND));
    put_u32(p + 10, (uint32_t)GetFreeHeapBytes());
    put_ip_octets(p + 14, app.ip_address);
    put_ip_octets(p + 18, app.net_mask);
    memcpy(p + 22, heth.Init.MACAddr, 6);
    put_u32(p + 28, app.chip_uid[0]);
    put_u32(p + 32, app.chip_uid[1]);
    put_u32(p + 36, app.chip_uid[2]);
    p[40] = app.ResetCauseCode();
    put_padded_string(p + 41, 32, DEVICE_HOSTNAME, strlen(DEVICE_HOSTNAME));
    put_padded_string(p + 73, 48, BOARD_NAME.data(), BOARD_NAME.length());

    st.pos = 0;
}

static void write_system_info_chunk(void *context, char *dest, size_t want) {
    SystemInfoState *st = (SystemInfoState *)context;
    memcpy(dest, st->buffer + st->pos, want);
    st->pos += want;
}

UINT webserver_request_callback(NX_WEB_HTTP_SERVER *server_ptr, UINT request_type,
                                 CHAR *resource, NX_PACKET *packet_ptr) {
    (void)request_type;

    if (strcmp(resource, "/") == 0) {
        size_t cursor = 0;
        size_t html_br_len = (size_t)(_binary_index_html_br_end - _binary_index_html_br_start);
        UINT status = send_streamed_response(server_ptr, "text/html", "Content-Encoding: br\r\n",
                                             html_br_len, write_from_flash_blob, &cursor);
        return (status == NX_SUCCESS) ? NX_WEB_HTTP_CALLBACK_COMPLETED : NX_NOT_SUCCESSFUL;
    }

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
            App::Instance().register_model->SetHoldingRegister((uint16_t)address, (uint16_t)value);
            sprintf(response_data, "OK");
        } else {
            sprintf(response_data, "ERROR");
        }
    } else {
        return NX_SUCCESS;
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
