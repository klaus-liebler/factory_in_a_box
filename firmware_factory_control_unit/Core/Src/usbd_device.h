#pragma once
// USB-Composite-Device auf Basis von TinyUSB (USB_DRD_FS-Peripherie, Full-Speed-only, 8
// Hardware-Endpunkte). Bewusst "usbd_" statt "usb_" praefixiert, um Verwechslungen mit
// usb_pd_control.* (USB Power Delivery ueber UCPD1 -- eine komplett andere Peripherie) zu
// vermeiden.
//
// Reines C (nicht .cc/.hh): TinyUSBs ThreadX-OSAL-Anbindung (tinyusb/src/osal/osal_threadx.h)
// enthaelt Inline-Funktionsdefinitionen mit in C++ nicht zulaessigen impliziten
// void*->CHAR*-Konvertierungen (ueber TX_NULL bzw. tx_semaphore_create()) -- gueltiges C, aber
// ein harter Compilerfehler, sobald dieselbe Header-Kette in einer C++-Uebersetzungseinheit
// eingebunden wird. Deshalb liegt jeglicher Code, der tusb.h einbindet, hier in usbd_device.c
// (reines C); C++-Aufrufer (app.cc) sehen nur diese extern-"C"-Schnittstelle.
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialisiert TinyUSB (tusb_init()) und merkt sich die Board-eindeutige Chip-UID fuer den
// USB-Seriennummer-Stringdeskriptor -- dieselbe UID, die auch schon fuer CHIP_ID_*
// Modbus-Register und den Hostnamen verwendet wird (s. App::chip_uid). Muss aus einem
// laufenden ThreadX-Thread heraus aufgerufen werden, NICHT vor tx_kernel_enter() (die
// ThreadX-OSAL-Anbindung von TinyUSB braucht den laufenden Scheduler).
void usbd_device_setup(uint32_t chip_uid0, uint32_t chip_uid1, uint32_t chip_uid2);

// Endlosschleife (kehrt nie zurueck): ruft tud_task() sowie die USB_VSENSE-Ueberwachung, die
// den Stack per tud_connect()/tud_disconnect() an die tatsaechliche 5V-Praesenz am USB-Port
// koppelt (PC0/USB_VSENSE, 10k/10k-Spannungsteiler). tusb_init()/tud_task() muessen laut
// TinyUSB-Doku im selben Thread-Kontext laufen -- usbd_device_setup() und usbd_device_loop()
// werden deshalb beide aus App::UsbdDeviceThread() aufgerufen, niemals einzeln.
_Noreturn void usbd_device_loop(void);

// Diagnose (Enumerations-Haenger nach echtem Host-Anschluss, s. Debugging-Sitzung): Anzahl der
// bisherigen USB_DRD_FS_IRQHandler()-Aufrufe seit Boot. Wird vom Heartbeat-Thread mitgeloggt --
// waechst der Zaehler unkontrolliert, waehrend sonst nichts mehr vorangeht, deutet das auf einen
// Interrupt-Storm (ISR feuert immer wieder neu, ohne dass Threads noch drankommen) hin; bleibt
// selbst der Heartbeat aus, eher auf ein echtes Haengenbleiben INNERHALB der ISR. In der ISR
// selbst wird bewusst nicht geloggt (log_info()/HAL_UART_Transmit sind aus Interrupt-Kontext
// nicht sicher aufrufbar) -- nur ein einfacher Zaehler-Increment.
uint32_t usbd_device_get_isr_count(void);

#ifdef __cplusplus
}
#endif
