# ModbusTcpServer — Header-Only C++ Klasse

Eine einfache, sichere Modbus-TCP-Server-Implementierung für Azure RTOS (ThreadX/NetX Duo), als Single-Header-Datei.

## Features

- **Header-Only** — Keine separaten `.cpp`-Dateien, alles in `modbus_tcp_server.hpp`
- **ThreadX/NetX Duo Integration** — Nutzt `NX_TCP_SOCKET` und NetX-Packet-Pools direkt
- **Registerspeicher wird injiziert** — Holding- und Input-Register liegen als rohe `uint16_t*` + Anzahl beim Aufrufer (z.B. `AppState`), nicht in der Server-Klasse. Kein Heap noetig, passt zum ThreadX-Byte-Pool-Muster des Projekts.
- **Optionale Callbacks** — `callback_before_read` (vor FC03/FC04) und `callback_after_write` (nach FC06/FC16), jeweils `(function_code, start_addr, count)`
- **Unterstützte Funktionscodes**:
  - **03** — Read Holding Registers (bis 125 auf einmal)
  - **04** — Read Input Registers (bis 125 auf einmal)
  - **06** — Write Single Register
  - **16** — Write Multiple Registers (bis 123 auf einmal)
- **Exception-Handling** — Rückgabe von Exception-Codes (0x01=Illegal Function, 0x02=Illegal Address)
- **MBAP-Header-Verarbeitung** — Korrekte Transaction-ID und Protocol-ID-Verwaltung
- **Client-Loop** — Akzeptiert mehrere sequenzielle Clients (TCP-Verbindungen)

## Verwendung

### 1. Header includieren (in `app_netxduo.c` oder `.cpp`):

```c
#include "modbus_tcp_server.hpp"
```

### 2. Registerspeicher anlegen und Instanz erstellen:

```c
// Registerspeicher lebt beim Aufrufer (z.B. statisch oder als Member von AppState)
static uint16_t holding_registers[MODBUS_MAX_REGISTERS] = {0};
static uint16_t input_registers[MODBUS_MAX_REGISTERS] = {0};

ModbusTcpServer::RegisterModel registers{
    holding_registers, MODBUS_MAX_REGISTERS,
    input_registers, MODBUS_MAX_REGISTERS
};

// Optional: Callback vor Reads / nach Writes registrieren (sonst nullptr)
void on_write(uint8_t function_code, uint16_t start_addr, size_t count) {
    printf("FC%u write @%u, count=%zu\n", function_code, start_addr, count);
}

ModbusTcpServer server(&NetXDuoEthIpInstance, &WebServerPool, registers, on_write);

// Initialisieren (bindet auf Port 502)
if (server.initialize() != NX_SUCCESS) {
    printf("Modbus Init failed\n");
    return;
}
```

### 3. In einem ThreadX-Thread ausführen:

```c
void my_modbus_thread_entry(ULONG arg) {
    ModbusTcpServer* server = (ModbusTcpServer*)arg;
    server->run();  // Blockierend: akzeptiert Clients forever
}

// Thread erstellen und starten
tx_thread_create(&thread_handle, "Modbus", my_modbus_thread_entry, 
                 (ULONG)&server, stack_ptr, stack_size,
                 PRIORITY, PRIORITY, 0, TX_AUTO_START);
```

### 4. Auf Register zugreifen (aus anderen Threads/Callbacks):

```c
// Lesen
uint16_t value = server.read_holding_register(100);

// Schreiben
server.write_holding_register(100, 1234);
server.write_holding_register(101, 5678);
```

### 5. Server stoppen:

```c
server.stop();
tx_thread_suspend(&modbus_thread_handle);
```

## Interne Struktur

```
ModbusTcpServer
├── RegisterModel (vom Aufrufer injiziert, keine eigene Storage)
│   ├── holding_registers  (uint16_t*, holding_register_count)
│   └── input_registers    (uint16_t*, input_register_count)
│
├── NX_TCP_SOCKET m_server_socket  (Server-Listen-Socket)
├── NX_IP* m_ip_instance           (NetX IP-Instanz)
├── NX_PACKET_POOL* m_packet_pool  (Für Paket-Allokation)
├── m_callback_before_read         (optional, vor FC03/FC04)
├── m_callback_after_write         (optional, nach FC06/FC16)
│
└── Methoden
    ├── initialize()                (Socket binden auf Port 502)
    ├── run()                       (Main-Loop, akzeptiert Clients)
    ├── stop()                      (Socket cleanup)
    ├── read_holding_register(addr) (uint16_t)
    ├── write_holding_register(addr, value)
    ├── read_input_register(addr)   (uint16_t)
    ├── write_input_register(addr, value)
    └── ...
    [private]
    ├── process_client_requests()   (Client-Kommunikation)
    ├── process_modbus_request()    (PDU-Parsing & Response-Building)
    ├── build_exception_response()  (Exception-Response)
    └── send_response()             (Paket-Versand)
```

## Sicherheit & Threadsicherheit

- **Einfacher Schutz**: Das Design setzt darauf, dass Registerzugriff aus kontrolliertem Kontext erfolgt (z.B. Applikations-Threads, HTTP-Callbacks)
- **Modbus-Thread läuft separat**: Die `run()`-Schleife blockiert in TCP-Accept/Receive, daher sind Datenzugriffe praktisch serialisiert
- **Keine Mutex/Semaphoren**: Für einfache Anwendungen mit 1–2 Client-Connections nicht nötig; bei Bedarf könnten TX_SEMAPHORE um Register-Zugriffe gewickelt werden

## Erweiterung

Um weitere Funktionscodes hinzuzufügen (z.B. FC 01=Read Coils, FC 05=Write Single Coil, FC 15=Write Multiple Coils):

1. `process_modbus_request()` in der `else if`-Kette erweitern
2. Entsprechende Register lesen/schreiben
3. Response gemäß Modbus-Spezifikation bauen und zurück

## Beispiel: Modbus-Client-Anfrage (mit `mbpoll` oder ähnlich)

```bash
# TCP-Verbindung zu localhost:502
# Read Holding Registers 0-9
# REQUEST:  00 01 00 00 00 06 01 03 00 00 00 0A
#           ^^^^^^^^^^  MBAP Header (Transaction=1, Protocol=0, Length=6, Unit=1)
#                       ^^ Function Code 3
#                          ^^^^^^ Start Addr=0, Quantity=10
#
# RESPONSE: 00 01 00 00 00 15 01 03 14 [20 bytes of register data]
```

## Limitations

- **Keine Coil/Discrete-Input-Funktionen (FC 01, 02, 05, 15)** im Grundgerüst — können einfach hinzugefügt werden
- **Einzelne TCP-Verbindung gleichzeitig** — `nx_tcp_server_socket_accept()` blockiert auf eine neue Verbindung; sequenzielle Clients OK, gleichzeitige brauchen Event-Loop-Refactor
- **Keine Authentifizierung** — Modbus TCP selbst hat keine; würde bei Bedarf in den Socket-Accept vor `process_client_requests()` hinzugefügt

## Testing

Mit `mbpoll` (Open-Source Modbus-Client):

```bash
mbpoll -t u 127.0.0.1:502 -a 1 -c 10 0
# Liest 10 Holding Registers von Adresse 0 aus Unit 1 (localhost:502)

mbpoll -t u 127.0.0.1:502 -a 1 0 1234 5678
# Schreibt zwei Register (Funktionscode 16)
```

Oder in C mit libmodbus, Python mit `pymodbus`, etc.
