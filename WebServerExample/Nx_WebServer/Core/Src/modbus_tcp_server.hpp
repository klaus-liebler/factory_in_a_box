#pragma once

#include "nx_api.h"
#include "tx_api.h"
#include <cstring>
#include <cstdint>

constexpr uint16_t MODBUS_TCP_PORT = 502;
constexpr uint16_t MODBUS_PROTOCOL_ID = 0;
constexpr uint8_t MAX_MODBUS_CLIENTS = 5;
constexpr uint16_t MODBUS_MAX_REGISTERS = 1024;
constexpr uint16_t MODBUS_MAX_COILS = 2048;

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
    // Modbus-Registerspeicher
    struct Registers {
        uint16_t holding_registers[MODBUS_MAX_REGISTERS];
        uint16_t input_registers[MODBUS_MAX_REGISTERS];
        uint8_t  coils[MODBUS_MAX_COILS / 8];
        uint8_t  discrete_inputs[MODBUS_MAX_COILS / 8];

        Registers() {
            std::memset(holding_registers, 0, sizeof(holding_registers));
            std::memset(input_registers, 0, sizeof(input_registers));
            std::memset(coils, 0, sizeof(coils));
            std::memset(discrete_inputs, 0, sizeof(discrete_inputs));
        }

        // Hilfsfunktionen für Bit-Zugriff
        bool get_coil(uint16_t addr) const {
            if (addr >= MODBUS_MAX_COILS) return false;
            return (coils[addr / 8] >> (addr % 8)) & 1;
        }

        void set_coil(uint16_t addr, bool value) {
            if (addr >= MODBUS_MAX_COILS) return;
            if (value)
                coils[addr / 8] |= (1 << (addr % 8));
            else
                coils[addr / 8] &= ~(1 << (addr % 8));
        }

        bool get_discrete_input(uint16_t addr) const {
            if (addr >= MODBUS_MAX_COILS) return false;
            return (discrete_inputs[addr / 8] >> (addr % 8)) & 1;
        }

        void set_discrete_input(uint16_t addr, bool value) {
            if (addr >= MODBUS_MAX_COILS) return;
            if (value)
                discrete_inputs[addr / 8] |= (1 << (addr % 8));
            else
                discrete_inputs[addr / 8] &= ~(1 << (addr % 8));
        }
    };

public:
    ModbusTcpServer(NX_IP* ip_instance, NX_PACKET_POOL* pool)
        : m_ip_instance(ip_instance), m_packet_pool(pool),
          m_transaction_id(0), m_is_running(false) {
    }

    // Initialisierung
    UINT initialize() {
        UINT status;

        // Mutex zum Schutz von m_registers gegen parallelen Zugriff aus
        // dem Modbus-Thread (dieser Klasse) und den I/O-Threads (Sensoren/Aktuatoren).
        status = tx_mutex_create(&m_register_mutex, const_cast<CHAR *>("Modbus Register Mutex"), TX_INHERIT);
        if (status != TX_SUCCESS) {
            return status;
        }

        // TCP Socket erstellen
        status = nx_tcp_socket_create(
            m_ip_instance,
            &m_server_socket,
            const_cast<CHAR *>("Modbus Server Socket"),
            NX_IP_NORMAL,
            NX_FRAGMENT_OKAY,
            NX_IP_TIME_TO_LIVE,
            512,  // window size
            NX_NULL,  // urgent data callback
            NX_NULL   // disconnect callback
        );

        if (status != NX_SUCCESS) {
            return status;
        }

        // Socket Binden auf Port 502
        status = nx_tcp_server_socket_listen(
            m_ip_instance,
            MODBUS_TCP_PORT,
            &m_server_socket,
            MAX_MODBUS_CLIENTS,
            NX_NULL
        );

        if (status != NX_SUCCESS) {
            nx_tcp_socket_delete(&m_server_socket);
            return status;
        }

        m_is_running = true;
        return NX_SUCCESS;
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

            // Client-Socket vorbereiten
            status = nx_tcp_server_socket_relisten(
                m_ip_instance,
                MODBUS_TCP_PORT,
                &m_server_socket
            );

            if (status != NX_SUCCESS) {
                continue;
            }

            // Client-Verbindung im m_server_socket ist jetzt aktiv
            // Anfragen empfangen und verarbeiten
            process_client_requests(&m_server_socket);

            // Verbindung schließen
            nx_tcp_socket_disconnect(&m_server_socket, NX_WAIT_FOREVER);
        }
    }

    // Registerzugriff -- Thread-Safe via m_register_mutex, da sowohl der
    // Modbus-Thread (process_modbus_request) als auch I/O-Threads (Sensoren/Aktuatoren)
    // dieselben Arrays anfassen.
    uint16_t read_holding_register(uint16_t addr) const {
        uint16_t value = 0;
        if (addr < MODBUS_MAX_REGISTERS) {
            tx_mutex_get(const_cast<TX_MUTEX*>(&m_register_mutex), TX_WAIT_FOREVER);
            value = m_registers.holding_registers[addr];
            tx_mutex_put(const_cast<TX_MUTEX*>(&m_register_mutex));
        }
        return value;
    }

    void write_holding_register(uint16_t addr, uint16_t value) {
        if (addr < MODBUS_MAX_REGISTERS) {
            tx_mutex_get(&m_register_mutex, TX_WAIT_FOREVER);
            m_registers.holding_registers[addr] = value;
            tx_mutex_put(&m_register_mutex);
        }
    }

    uint16_t read_input_register(uint16_t addr) const {
        uint16_t value = 0;
        if (addr < MODBUS_MAX_REGISTERS) {
            tx_mutex_get(const_cast<TX_MUTEX*>(&m_register_mutex), TX_WAIT_FOREVER);
            value = m_registers.input_registers[addr];
            tx_mutex_put(const_cast<TX_MUTEX*>(&m_register_mutex));
        }
        return value;
    }

    void write_input_register(uint16_t addr, uint16_t value) {
        if (addr < MODBUS_MAX_REGISTERS) {
            tx_mutex_get(&m_register_mutex, TX_WAIT_FOREVER);
            m_registers.input_registers[addr] = value;
            tx_mutex_put(&m_register_mutex);
        }
    }

    bool read_coil(uint16_t addr) const {
        return m_registers.get_coil(addr);
    }

    void write_coil(uint16_t addr, bool value) {
        m_registers.set_coil(addr, value);
    }

    bool read_discrete_input(uint16_t addr) const {
        return m_registers.get_discrete_input(addr);
    }

    void write_discrete_input(uint16_t addr, bool value) {
        m_registers.set_discrete_input(addr, value);
    }

    void stop() {
        m_is_running = false;
        nx_tcp_socket_disconnect(&m_server_socket, NX_WAIT_FOREVER);
        nx_tcp_socket_delete(&m_server_socket);
        tx_mutex_delete(&m_register_mutex);
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

            if (quantity > 125 || start_addr + quantity > MODBUS_MAX_REGISTERS) {
                return build_exception_response(response, function_code, 0x02);
            }

            response[7] = function_code;
            response[8] = quantity * 2;  // Byte count

            uint32_t offset = 9;
            tx_mutex_get(&m_register_mutex, TX_WAIT_FOREVER);
            for (uint16_t i = 0; i < quantity; i++) {
                uint16_t value = m_registers.holding_registers[start_addr + i];
                response[offset++] = (value >> 8) & 0xFF;
                response[offset++] = value & 0xFF;
            }
            tx_mutex_put(&m_register_mutex);

            pdu_len = 3 + quantity * 2;
        }

        // Funktion 04: Read Input Registers
        else if (function_code == 0x04 && request_len >= 12) {
            uint16_t start_addr = ((uint16_t)request[8] << 8) | request[9];
            uint16_t quantity = ((uint16_t)request[10] << 8) | request[11];

            if (quantity > 125 || start_addr + quantity > MODBUS_MAX_REGISTERS) {
                return build_exception_response(response, function_code, 0x02);
            }

            response[7] = function_code;
            response[8] = quantity * 2;

            uint32_t offset = 9;
            tx_mutex_get(&m_register_mutex, TX_WAIT_FOREVER);
            for (uint16_t i = 0; i < quantity; i++) {
                uint16_t value = m_registers.input_registers[start_addr + i];
                response[offset++] = (value >> 8) & 0xFF;
                response[offset++] = value & 0xFF;
            }
            tx_mutex_put(&m_register_mutex);

            pdu_len = 3 + quantity * 2;
        }

        // Funktion 06: Write Single Register
        else if (function_code == 0x06 && request_len >= 12) {
            uint16_t addr = ((uint16_t)request[8] << 8) | request[9];
            uint16_t value = ((uint16_t)request[10] << 8) | request[11];

            if (addr >= MODBUS_MAX_REGISTERS) {
                return build_exception_response(response, function_code, 0x02);
            }

            tx_mutex_get(&m_register_mutex, TX_WAIT_FOREVER);
            m_registers.holding_registers[addr] = value;
            tx_mutex_put(&m_register_mutex);

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
                start_addr + quantity > MODBUS_MAX_REGISTERS) {
                return build_exception_response(response, function_code, 0x02);
            }

            // Schreibe Register
            const uint8_t* values = request + 13;
            tx_mutex_get(&m_register_mutex, TX_WAIT_FOREVER);
            for (uint16_t i = 0; i < quantity; i++) {
                uint16_t value = ((uint16_t)values[i*2] << 8) | values[i*2 + 1];
                m_registers.holding_registers[start_addr + i] = value;
            }
            tx_mutex_put(&m_register_mutex);

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
    Registers m_registers;
    TX_MUTEX m_register_mutex;
    uint16_t m_transaction_id;
    bool m_is_running;
};
