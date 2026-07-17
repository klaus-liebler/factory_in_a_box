// Bindet log.h (stm32_libs/common_stm32) an die Debug-UART (USART3, DEBUG_TX/RX) an.
//
// Ohne diese Datei ist __io_putchar() im ganzen Projekt nirgends definiert (nur als
// __attribute__((weak)) extern in syscalls.c deklariert) -- ein nicht aufgeloestes weak-Symbol
// bindet beim Linken an Adresse 0, jeder log_*()-Aufruf (der erste passiert bereits ganz am
// Anfang von tx_application_define()) haette also sofort einen HardFault ausgeloest.
//
// Zusaetzlich: log_set_lock() macht log_log() threadsicher. Mit inzwischen zehn ThreadX-Threads,
// die alle gelegentlich loggen (led/link/io/usb_pd/scale/stepper/tof_color/can/modbus_server/
// app_main), wuerden sich unsynchronisierte log_log()-Aufrufe (mehrere fprintf(stdout, ...) pro
// Zeile) sonst gegenseitig ueberschreiben/verschachteln.
#include "main.h"
#include "log.h"
#include "tx_api.h"

extern "C" UART_HandleTypeDef huart3;

static TX_MUTEX s_log_mutex;

static void log_lock(bool lock) {
    if (lock) {
        tx_mutex_get(&s_log_mutex, TX_WAIT_FOREVER);
    } else {
        tx_mutex_put(&s_log_mutex);
    }
}

// Muss vor dem ersten log_*()-Aufruf laufen -- wird als erstes in tx_application_define()
// aufgerufen (vor malloc_lock_init()).
extern "C" void log_setup_init(void) {
    tx_mutex_create(&s_log_mutex, const_cast<CHAR *>("Log Mutex"), TX_NO_INHERIT);
    log_set_lock(log_lock);
}

// Blockierend, ein Byte pro HAL_UART_Transmit-Aufruf -- fuer Log-Zeilen (gelegentlich, nicht auf
// dem heissen Pfad) ausreichend; DMA/IT-basiertes Senden waere nur bei deutlich hoeherem
// Log-Volumen noetig.
extern "C" int __io_putchar(int ch) {
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
    return ch;
}
