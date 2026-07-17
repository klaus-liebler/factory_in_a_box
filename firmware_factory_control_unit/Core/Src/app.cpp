// ============================================================================
// ThreadX Application Wiring -- erzeugt alle Pools/Threads (tx_application_define(), laeuft
// vor dem Scheduler) und orchestriert danach den Boot-Ablauf (app_main_thread_entry(), der
// einzige Thread mit TX_AUTO_START). Nur drei ThreadX-Threads insgesamt:
//   - app_main_thread: einmaliger Boot-Orchestrator (FileX/TLS/HTTP/DHCP/Modbus-Setup, dann
//     Start der beiden anderen Threads)
//   - modbus_server_thread: blockierender TCP-Accept-Loop (ModbusTcpServer::run())
//   - io_thread: EIN gemeinsamer Thread fuer alle Sensoren/Aktoren (s. io_thread.cpp fuer die
//     Begruendung, warum das zusammengefasst werden konnte)
// ============================================================================
#include <cstdint>
#include <vector>
#include <utility>
#include "tx_api.h"

#include "app_state.hpp"
#include "net_setup.hpp"
#include "io_thread.hpp"
#include "greeting.hpp"

#include "modbus_register_map.hpp"
#include "diagnostics.hpp"
#include "log.h"

AppState g_app_state = {};

// ============================================================================
// Configuration Constants
// ============================================================================

constexpr uint32_t TX_APP_MEM_POOL_SIZE = 10 * 1024;
constexpr uint32_t FX_APP_MEM_POOL_SIZE = 10 * 1024;
// Die Summe aller tx_byte_allocate(&nx_app_byte_pool, ...)-Aufrufe unten (v.a. NetXDuo-
// Paketpool mit ~80 KB, HTTP-Server-Stack 16 KB fuer RSA/ECDHE-Handshakes, plus die Stacks
// von app_main/modbus_server/io_thread) liegt bei ca. 130 KB vor Byte-Pool-Verwaltungsoverhead
// -- deutliche Reserve statt auf Kante genau genug.
constexpr uint32_t NX_APP_MEM_POOL_SIZE = 192 * 1024;

constexpr uint32_t DEFAULT_MEMORY_SIZE = 1024;
constexpr UINT DEFAULT_PRIORITY = 5;

// ============================================================================
// Modbus Server Thread
// ============================================================================
static void modbus_server_thread_entry(ULONG thread_input) {
    // thread_input ist hier immer gesetzt: tx_thread_create() bekommt
    // (ULONG)g_app_state.modbus_server direkt beim Erzeugen dieses Threads uebergeben,
    // unmittelbar nachdem der Pointer per new ModbusTcpServer(...) zugewiesen wurde.
    ModbusTcpServer* server = (ModbusTcpServer*)thread_input;
    log_info("Modbus TCP Server started on port 502");
    server->run();  // Infinite loop, akzeptiert Clients
    log_info("Modbus TCP Server stopped");
}

// ============================================================================
// Main App Thread -- einziger Thread mit TX_AUTO_START. Fuehrt alle
// Initialisierungen durch, die einen laufenden Thread-Kontext brauchen
// (FileX/HTTP/DHCP/Modbus), und startet danach io_thread + modbus_server_thread
// (mit TX_DONT_START angelegt) explizit per tx_thread_resume().
// ============================================================================
static void app_main_thread_entry(ULONG arg) {
    (void)arg;

    net_setup_start();

    // Modbus-Server-Initialisierung braucht wie der HTTP-Server-Start einen laufenden
    // Thread-Kontext (nx_tcp_server_socket_listen() lehnt Aufrufe aus tx_application_define()
    // sonst mit NX_CALLER_ERROR ab).
    XASSERT(g_app_state.modbus_server->initialize(), "Modbus TCP Server initialization failed");
    Diagnostics::write_startup_registers(*g_app_state.modbus_server);

    // io_thread/modbus_server_thread sind mit TX_DONT_START angelegt und werden erst hier,
    // ganz am Ende der Initialisierung, gestartet -- so haengt die Boot-Reihenfolge nicht von
    // Thread-Prioritaeten ab, sondern ist explizit festgelegt.
    XASSERT(tx_thread_resume(&g_app_state.modbus_server_thread), "Modbus Server Thread resume failed");
    XASSERT(tx_thread_resume(&g_app_state.io_thread), "IO Thread resume failed");
}

// ============================================================================
// ThreadX Application Entry Point
// ============================================================================
extern "C" void malloc_lock_init(void);
extern "C" void log_setup_init(void);

extern "C" void tx_application_define(void *first_unused_memory) {
    (void)first_unused_memory;

    // Muss vor dem ersten log_*()-Aufruf laufen (s. log_setup.cpp).
    log_setup_init();
    malloc_lock_init();

    greeting();
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

    net_setup_create(&nx_app_byte_pool);

    void *ptr = 0;
    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, NX_APP_THREAD_STACK_SIZE, TX_NO_WAIT), "App Main Thread stack allocate failed");
    tx_thread_create(&g_app_state.app_main_thread, NX_CHAR_LITERAL("App Main Thread"),
                     app_main_thread_entry, 0,
                     (VOID *)ptr, NX_APP_THREAD_STACK_SIZE,
                     NX_APP_THREAD_PRIORITY, NX_APP_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    // --- Modbus TCP Server Setup ---
    // Server-Instanz und Registerspeicher (std::vector) kommen aus dem newlib-Heap statt aus
    // dem nx_app_byte_pool -- sicher seit malloc_lock_init() (siehe Core/Src/malloc_lock.c)
    // oben aufgerufen wurde.
    ModbusTcpServer::RegisterModel modbus_registers{
        std::vector<uint16_t>(ModbusRegisters::HOLDING_REGISTER_MAX_INDEX + 1, 0),
        std::vector<uint16_t>(ModbusRegisters::INPUT_REGISTER_MAX_INDEX + 1, 0)
    };
    g_app_state.modbus_server = new ModbusTcpServer(&g_app_state.ip_instance, &g_app_state.packet_pool, std::move(modbus_registers));

    // initialize() ruft u.a. nx_tcp_server_socket_listen() auf, das NetX nur aus einem
    // laufenden Thread heraus erlaubt -- hier in tx_application_define() laeuft noch kein
    // Thread. Der eigentliche initialize()-Aufruf passiert daher in app_main_thread_entry().
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

    // io_setup() (registerlevel-Init aller Subsysteme: ADC-Start, PWM-Modus-Fixups, TMC2209-
    // UART-Init, I2C-Sensor-Init, FDCAN-Filter, WS2812/LED-Init, USB-PD-Start) laeuft NICHT
    // hier, sondern schon in main() vor tx_kernel_enter() (s. main.c, USER CODE 2, sowie die
    // Begruendung in io_thread.hpp) -- mehrere der darin aufgerufenen *_setup()-Funktionen
    // brauchen funktionierende HAL-Timeouts (HAL_Delay()/HAL_I2C_*/HAL_UART_* mit endlichem
    // Timeout), die waehrend tx_application_define() nicht funktionieren (Interrupts gesperrt,
    // solange tx_kernel_enter() laeuft).
    XASSERT(tx_byte_allocate(&nx_app_byte_pool, (VOID **)&ptr, IO_THREAD_STACK_SIZE, TX_NO_WAIT), "IO Thread stack allocate failed");
    XASSERT(tx_thread_create(&g_app_state.io_thread, NX_CHAR_LITERAL("IO Thread"),
                     io_thread_entry, 0,
                     (VOID *)ptr, IO_THREAD_STACK_SIZE,
                     IO_THREAD_PRIORITY, IO_THREAD_PRIORITY,
                     TX_NO_TIME_SLICE, TX_DONT_START), "IO thread create failed");

    log_info("Application initialization complete");
}
