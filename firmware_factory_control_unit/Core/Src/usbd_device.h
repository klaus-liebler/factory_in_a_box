#pragma once
// USB-Composite-Device auf Basis von USBX (Eclipse ThreadX) statt TinyUSB -- USB_DRD_FS-
// Peripherie, Full-Speed-only, 8 Hardware-Endpunkte. Bewusst "usbd_" statt "usb_" praefixiert
// (s. usb_pd_control.* fuer USB Power Delivery, eine komplett andere Peripherie).
//
// Reines C (nicht .cc/.hh): wie schon bei der TinyUSB-Fassung bindet diese Datei USBX-Header
// (ux_api.h) ein, die selbst wieder ThreadX/NetX-Duo-Header einbinden -- unproblematisch in
// reinem C, aber der Stil (extern-"C"-Schnittstelle nach aussen) bleibt hier bewusst erhalten,
// damit C++-Aufrufer (app.cc) weiterhin nur diese schmale Schnittstelle sehen.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialisiert USBX (ux_system_initialize()/ux_device_stack_initialize()/Klassen-Registrierung)
// und merkt sich die Board-eindeutige Chip-UID fuer den USB-Seriennummer-Stringdeskriptor --
// dieselbe UID, die auch schon fuer CHIP_ID_* Modbus-Register und den Hostnamen verwendet wird
// (s. App::chip_uid). Muss aus einem laufenden ThreadX-Thread heraus aufgerufen werden, NICHT vor
// tx_kernel_enter() (USBX braucht wie TinyUSB den laufenden Scheduler).
// ecm_mac: 6 Bytes fuer den MAC-Adress-String-Deskriptor der virtuellen CDC-ECM-NIC -- von
// App::ComputeNcmMac() berechnet, dieselben Bytes gehen auch an usb_cdc_ecm_driver_init() (s.
// dort). Anders als bei TinyUSB startet USBX den eigentlichen USB-Controller (HAL_PCD_Start())
// noch NICHT hier, sondern erst in usbd_device_loop(), sobald USB_VSENSE tatsaechlich 5V zeigt
// (s. dort) -- ux_device_stack_initialize()/Klassen-Registrierung/ux_dcd_stm32_initialize()
// koennen dagegen bereits unabhaengig vom Kabel-Status passieren.
void usbd_device_setup(uint32_t chip_uid0, uint32_t chip_uid1, uint32_t chip_uid2, uint8_t const ecm_mac[6]);

// Endlosschleife (kehrt nie zurueck): koppelt den USB-Controller per HAL_PCD_Start()/_Stop() an
// die tatsaechliche 5V-Praesenz am USB-Port (PC0/USB_VSENSE, 10k/10k-Spannungsteiler) -- analog
// zu tud_connect()/tud_disconnect() bei der TinyUSB-Fassung. Anders als dort gibt es aber KEINE
// tud_task()-Polling-Schleife mehr: USBX' Geraeteklassen (CDC-ACM x2, CDC-ECM) bedienen sich
// jeweils ueber eigene, von der Klasse selbst erzeugte ThreadX-Threads (bulkin_thread/
// bulkout_thread/interrupt_thread) komplett selbststaendig, angetrieben vom USB_DRD_FS-Interrupt
// (s. USB_DRD_FS_IRQHandler unten). Diese Schleife hier dient nur noch der VSENSE-Ueberwachung
// und dem periodischen Aufruf von poll_hook (fuer ModbusRtuServer::Poll(), das -- anders als die
// USBX-Klassen -- weiterhin aktiv aus der Anwendung heraus angestossen werden muss, s.
// usbd_cdc_modbus_read()).
//
// poll_hook (optional, NULL erlaubt): wird in jedem Schleifendurchlauf aufgerufen (fester,
// kurzer Sleep zwischen den Durchlaeufen statt TinyUSBs adaptivem tud_task_ext()-Timeout -- ohne
// eigene tud_task()-Polling-Schleife entfaellt der urspruengliche Grund fuer die Adaptivitaet,
// s. Kommentar-Historie der TinyUSB-Fassung).
_Noreturn void usbd_device_loop(void (*poll_hook)(void));

// Diagnose (Enumerations-Haenger nach echtem Host-Anschluss, s. Debugging-Sitzung der TinyUSB-
// Fassung). Anzahl der bisherigen USB_DRD_FS_IRQHandler()-Aufrufe seit Boot.
uint32_t usbd_device_get_isr_count(void);

// Duenner Wrapper um die erste CDC-ACM-Instanz ("FactoryControl Debug") -- von syscalls.c's
// _write()-Spiegelung der Log-Ausgabe genutzt (s. dort). Nichtblockierend/best-effort wie
// TinyUSBs tud_cdc_n_write(0, ...): liefert 0 (verwirft die Ausgabe), solange kein Host
// verbunden ist oder eine vorherige Uebertragung noch nicht abgeschlossen wurde.
uint32_t usbd_debug_cdc_write(uint8_t const* buffer, uint32_t len);

// --- Duenne C-Wrapper um die zweite CDC-ACM-Instanz ("FactoryControl Modbus") -- fuer
// ModbusRtuServer (C++, stm32_libs/modbus/modbus_rtu_server.hpp), die dieselben vier
// Funktionssignaturen wie bei der TinyUSB-Fassung erwartet (bewusst UNVERAENDERT, s.
// modbus_rtu_server.hpp) und daher weiterhin nichtblockierend/pollend arbeitet.
//
// usbd_cdc_modbus_available()/_read(): USBX' ux_device_class_cdc_acm_read() blockiert (wartet
// bis zu UX_NON_CONTROL_TRANSFER_TIMEOUT auf tatsaechliche OUT-Daten vom Host) -- fuer den hier
// benoetigten nichtblockierenden Poll-Kontext ungeeignet. Ein dedizierter Empfangs-Thread (s.
// usbd_device.c) ruft ux_device_class_cdc_acm_read() deshalb selbst blockierend in einer Schleife
// auf und puffert das Ergebnis in einem Ringpuffer -- diese beiden Funktionen lesen NUR aus
// diesem Ringpuffer, nie direkt aus USBX.
uint32_t usbd_cdc_modbus_available(void);
uint32_t usbd_cdc_modbus_read(uint8_t* buffer, uint32_t bufsize);
// usbd_cdc_modbus_write(): nichtblockierend ueber ux_device_class_cdc_acm_write_with_callback()
// (Fire-and-forget, wie TinyUSBs tud_cdc_n_write() -- Abschluss wird intern von der Klasse ueber
// deren eigenen bulkin_thread erledigt). Liefert 0, wenn eine vorherige Uebertragung noch
// nicht abgeschlossen ist (analog zu TinyUSBs vollem TX-Ringpuffer) -- ModbusRtuServer prueft den
// Rueckgabewert aktuell nicht, verhaelt sich dann wie bei einem verlorenen Frame (Modbus-Master
// wiederholt selbst nach Timeout).
uint32_t usbd_cdc_modbus_write(const uint8_t* buffer, uint32_t len);
// write_flush(): bei TinyUSB noetig (eigener Ringpuffer, der explizit geleert werden musste),
// bei USBX' write_with_callback()-Pfad ueberfluessig (jeder write() stoesst die Uebertragung
// bereits selbst an) -- bleibt als No-Op-Stub erhalten, damit modbus_rtu_server.hpp UNVERAENDERT
// bleiben kann (s. Klassenkommentar oben).
void usbd_cdc_modbus_write_flush(void);

#ifdef __cplusplus
}
#endif
