#pragma once

#include "nx_api.h"
#include "tx_api.h"
#include <cstring>
#include <cstdint>
#include <cstddef>
#include <vector>
#include <utility>
#include "modbus_commons.hh"
#include "modbus_register_model.hh"
#include "common_macros.hh"

constexpr uint16_t MODBUS_TCP_PORT = 502;
constexpr uint16_t MODBUS_PROTOCOL_ID = 0;
constexpr uint8_t MAX_MODBUS_CLIENTS = 5;

// Modbus TCP MBAP Header (7 bytes) + PDU (bis 252 bytes = 259 max)
constexpr uint16_t MODBUS_MAX_ADU_SIZE = 260;

// MBAP Header struktur (Modbus Application Header Protocol)
struct MbapHeader {
    uint16_t transaction_id;
    uint16_t protocol_id;
    uint16_t length;
    uint8_t  unit_id;

    void serialize(uint8_t* buffer) const {
        buffer[0] = (transaction_id >> 8) & 0xFF;
        buffer[1] = transaction_id & 0xFF;
        buffer[2] = (protocol_id >> 8) & 0xFF;
        buffer[3] = protocol_id & 0xFF;
        buffer[4] = (length >> 8) & 0xFF;
        buffer[5] = length & 0xFF;
        buffer[6] = unit_id;
    }

    static MbapHeader deserialize(const uint8_t* buffer) {
        MbapHeader hdr;
        hdr.transaction_id = ((uint16_t)buffer[0] << 8) | buffer[1];
        hdr.protocol_id = ((uint16_t)buffer[2] << 8) | buffer[3];
        hdr.length = ((uint16_t)buffer[4] << 8) | buffer[5];
        hdr.unit_id = buffer[6];
        return hdr;
    }
};

// Modbus TCP Server als Header-Only Klasse
class ModbusTcpServer {
public:
    // callback_before_read wird vor dem Zusammenstellen einer FC03/FC04-Antwort
    // aufgerufen, callback_after_write nach einer FC06/FC16-Schreiboperation --
    // jeweils mit (function_code, start_addr, count). Optional (nullptr = kein Callback).
    ModbusTcpServer(NX_IP* ip_instance, NX_PACKET_POOL* pool, Modbus::IModbusRegisterModel& registers,
                    void (*callback_after_write)(uint8_t, uint16_t, size_t) = nullptr,
                    void (*callback_before_read)(uint8_t, uint16_t, size_t) = nullptr)
        : m_ip_instance(ip_instance), m_packet_pool(pool), m_registers(registers),
          m_callback_after_write(callback_after_write), m_callback_before_read(callback_before_read),
          m_transaction_id(0), m_is_running(true) {
    }

    // Initialisierung
    void initialize() {
    
        // TCP Socket erstellen
        XASSERT(nx_tcp_socket_create(
            m_ip_instance,
            &m_server_socket,
            const_cast<CHAR *>("Modbus Server Socket"),
            NX_IP_NORMAL,
            NX_FRAGMENT_OKAY,
            NX_IP_TIME_TO_LIVE,
            512,  // window size
            NX_NULL,  // urgent data callback
            NX_NULL   // disconnect callback
        ), "Failed to create TCP socket in ModbusTcpServer::initialize()");

        // Socket Binden auf Port 502
        XASSERT(nx_tcp_server_socket_listen(
            m_ip_instance,
            MODBUS_TCP_PORT,
            &m_server_socket,
            MAX_MODBUS_CLIENTS,
            NX_NULL
        ), "Failed to listen on TCP socket in ModbusTcpServer::initialize()");
    }

    // Main Server-Loop (wird in ThreadX-Thread aufgerufen)
    void run() {
        UINT status;

        while (m_is_running) {
            // Neuen Client akzeptieren
            status = nx_tcp_server_socket_accept(&m_server_socket, 100);

            if (status != NX_SUCCESS) {
                tx_thread_sleep(10);  // Kurze Pause, dann retry
                continue;
            }

            // Client-Verbindung im m_server_socket ist jetzt aktiv
            // Anfragen empfangen und verarbeiten
            process_client_requests(&m_server_socket);

            // Verbindung schliessen, ERST danach darf relisten() aufgerufen werden --
            // relisten() erwartet einen bereits geschlossenen Socket (sonst blockiert
            // sie bzw. liefert NX_NOT_CLOSED) und versetzt ihn zurueck in den
            // Listen-Zustand fuer den naechsten Client. unaccept() dazwischen setzt
            // den Socket in den Zustand direkt nach dem Erstellen zurueck -- ohne das
            // schlaegt relisten() fehl bzw. der Server nimmt keine weiteren Clients
            // mehr an (Referenzmuster: Middlewares/ST/netxduo/addons/web/nx_tcpserver.c).
            nx_tcp_socket_disconnect(&m_server_socket, NX_WAIT_FOREVER);
            nx_tcp_server_socket_unaccept(&m_server_socket);

            status = nx_tcp_server_socket_relisten(
                m_ip_instance,
                MODBUS_TCP_PORT,
                &m_server_socket
            );

            if (status != NX_SUCCESS) {
                continue;
            }
        }
    }


private:
    void process_client_requests(NX_TCP_SOCKET* socket) {
        NX_PACKET *packet_ptr;
        UINT status;
        uint8_t request_buffer[MODBUS_MAX_ADU_SIZE];
        uint8_t response_buffer[MODBUS_MAX_ADU_SIZE];
        uint32_t bytes_received;

        while (m_is_running) {
            // Warte auf Daten vom Client (mit Timeout)
            status = nx_tcp_socket_receive(
                socket,
                &packet_ptr,
                NX_IP_PERIODIC_RATE  // 100ms timeout
            );

            if (status != NX_SUCCESS) {
                break;  // Client disconnected oder timeout
            }

            // Paket-Daten in Buffer kopieren
            bytes_received = 0;
            NX_PACKET *current = packet_ptr;

            while (current && bytes_received < MODBUS_MAX_ADU_SIZE) {
                uint32_t fragment_len = current->nx_packet_length;
                if (bytes_received + fragment_len > MODBUS_MAX_ADU_SIZE) {
                    fragment_len = MODBUS_MAX_ADU_SIZE - bytes_received;
                }

                std::memcpy(
                    request_buffer + bytes_received,
                    current->nx_packet_prepend_ptr,
                    fragment_len
                );

                bytes_received += fragment_len;
                current = current->nx_packet_next;
            }

            // Paket freigeben
            nx_packet_release(packet_ptr);

            // Modbus-Request verarbeiten
            if (bytes_received >= 8) {  // Mindestens MBAP-Header + FC
                uint32_t response_len = process_modbus_request(
                    request_buffer,
                    bytes_received,
                    response_buffer
                );

                if (response_len > 0) {
                    // Response senden
                    send_response(socket, response_buffer, response_len);
                }
            }
        }
    }

    uint32_t process_modbus_request(
        const uint8_t* request,
        uint32_t request_len,
        uint8_t* response
    ) {
        if (request_len < 8) return 0;

        // MBAP Header parsen
        MbapHeader mbap = MbapHeader::deserialize(request);
        uint8_t function_code = request[7];

        // Response-MBAP kopieren (Transaction ID etc. beibehalten)
        mbap.serialize(response);

        uint32_t pdu_len = 0;

        // Funktion 03: Read Holding Registers
        if (function_code == 0x03 && request_len >= 12) {
            uint16_t start_addr = ((uint16_t)request[8] << 8) | request[9];
            uint16_t quantity = ((uint16_t)request[10] << 8) | request[11];

            if (quantity > 125 || start_addr + quantity > ModbusRegisters::HOLDING_REGISTER_MAX_INDEX + 1) {
                return build_exception_response(response, function_code, 0x02);
            }

            if (m_callback_before_read)
                m_callback_before_read(function_code, start_addr, quantity);

            response[7] = function_code;
            response[8] = quantity * 2;  // Byte count

            uint32_t offset = 9;

            for (uint16_t i = 0; i < quantity; i++) {
                uint16_t value = m_registers.GetHoldingRegister(start_addr + i);
                response[offset++] = (value >> 8) & 0xFF;
                response[offset++] = value & 0xFF;
            }

            pdu_len = 3 + quantity * 2;
        }

        // Funktion 04: Read Input Registers
        else if (function_code == 0x04 && request_len >= 12) {
            uint16_t start_addr = ((uint16_t)request[8] << 8) | request[9];
            uint16_t quantity = ((uint16_t)request[10] << 8) | request[11];

            if (quantity > 125 || start_addr + quantity > ModbusRegisters::INPUT_REGISTER_MAX_INDEX + 1) {
                return build_exception_response(response, function_code, 0x02);
            }

            if (m_callback_before_read)
                m_callback_before_read(function_code, start_addr, quantity);

            response[7] = function_code;
            response[8] = quantity * 2;

            uint32_t offset = 9;
            for (uint16_t i = 0; i < quantity; i++) {
                uint16_t value = m_registers.GetInputRegister(start_addr + i);
                response[offset++] = (value >> 8) & 0xFF;
                response[offset++] = value & 0xFF;
            }
            pdu_len = 3 + quantity * 2;
        }

        // Funktion 06: Write Single Register
        else if (function_code == 0x06 && request_len >= 12) {
            uint16_t addr = ((uint16_t)request[8] << 8) | request[9];
            uint16_t value = ((uint16_t)request[10] << 8) | request[11];

            if (addr > ModbusRegisters::HOLDING_REGISTER_MAX_INDEX) {
                return build_exception_response(response, function_code, 0x02);
            }

            m_registers.SetHoldingRegister(addr, value);

            if (m_callback_after_write)
                m_callback_after_write(function_code, addr, 1);

            // Echo request back
            std::memcpy(response + 7, request + 7, 6);
            pdu_len = 6;
        }

        // Funktion 16: Write Multiple Registers
        else if (function_code == 0x10 && request_len >= 13) {
            uint16_t start_addr = ((uint16_t)request[8] << 8) | request[9];
            uint16_t quantity = ((uint16_t)request[10] << 8) | request[11];
            uint8_t byte_count = request[12];

            if (quantity > 123 || byte_count != quantity * 2 ||
                request_len < 13u + byte_count ||
                start_addr + quantity > ModbusRegisters::HOLDING_REGISTER_MAX_INDEX + 1) {
                return build_exception_response(response, function_code, 0x02);
            }

            // Schreibe Register -- jeder einzelne SetRegister()-Aufruf ist bereits ueber den
            // internen Mutex des Register-Modells (ModbusRegisterModel::Lock/Unlock)
            // synchronisiert, eine zusaetzliche Sperre ueber die ganze Schleife ist nicht noetig.
            const uint8_t* values = request + 13;
            for (uint16_t i = 0; i < quantity; i++) {
                uint16_t value = ((uint16_t)values[i*2] << 8) | values[i*2 + 1];
                m_registers.SetHoldingRegister(start_addr + i, value);
            }

            if (m_callback_after_write)
                m_callback_after_write(function_code, start_addr, quantity);

            response[7] = function_code;
            response[8] = (start_addr >> 8) & 0xFF;
            response[9] = start_addr & 0xFF;
            response[10] = (quantity >> 8) & 0xFF;
            response[11] = quantity & 0xFF;
            pdu_len = 5;
        }

        // Unbekannte Funktion
        else {
            return build_exception_response(response, function_code, 0x01);
        }

        // MBAP-Header aktualisieren: Length = PDU + Unit ID (1 byte)
        uint16_t length = pdu_len + 1;
        response[4] = (length >> 8) & 0xFF;
        response[5] = length & 0xFF;

        return 7 + pdu_len;  // MBAP (7) + PDU
    }

    uint32_t build_exception_response(uint8_t* response, uint8_t function_code, uint8_t exception_code) {
        // Function Code im Response mit MSB=1 setzen (Exception-Indikator)
        response[7] = function_code | 0x80;
        response[8] = exception_code;

        // MBAP Length = 2 (FC + Exception)
        response[4] = 0x00;
        response[5] = 0x02;

        return 9;  // MBAP (7) + FC + Exception
    }

    UINT send_response(NX_TCP_SOCKET* socket, const uint8_t* data, uint32_t len) {
        NX_PACKET *packet_ptr;
        UINT status;

        status = nx_packet_allocate(
            m_packet_pool,
            &packet_ptr,
            NX_TCP_PACKET,
            NX_WAIT_FOREVER
        );

        if (status != NX_SUCCESS) {
            return status;
        }

        status = nx_packet_data_append(
            packet_ptr,
            (VOID*)data,
            len,
            m_packet_pool,
            NX_WAIT_FOREVER
        );

        if (status != NX_SUCCESS) {
            nx_packet_release(packet_ptr);
            return status;
        }

        status = nx_tcp_socket_send(socket, packet_ptr, NX_WAIT_FOREVER);

        if (status != NX_SUCCESS) {
            nx_packet_release(packet_ptr);
        }

        return status;
    }

private:
    NX_IP* m_ip_instance;
    NX_PACKET_POOL* m_packet_pool;
    NX_TCP_SOCKET m_server_socket;
    Modbus::IModbusRegisterModel& m_registers;
    void (*m_callback_after_write)(uint8_t, uint16_t, size_t);
    void (*m_callback_before_read)(uint8_t, uint16_t, size_t);
    uint16_t m_transaction_id;
    bool m_is_running;
};
