#pragma once
// USB-Composite-Device auf Basis von USBX (Eclipse ThreadX) statt TinyUSB -- USB_DRD_FS-
// Peripherie, Full-Speed-only, 8 Hardware-Endpunkte. Bewusst "usbd_" statt "usb_" praefixiert
// (s. usb_pd_control.* fuer USB Power Delivery, eine komplett andere Peripherie).
//
// Reines C (nicht .cc/.hh): wie schon bei der TinyUSB-Fassung bindet diese Datei USBX-Header
// (ux_api.h) ein, die selbst wieder ThreadX/NetX-Duo-Header einbinden -- unproblematisch in
// reinem C, aber der Stil (extern-"C"-Schnittstelle nach aussen) bleibt hier bewusst erhalten,
// damit C++-Aufrufer (app.cc) weiterhin nur diese schmale Schnittstelle sehen.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialisiert USBX (ux_system_initialize()/ux_device_stack_initialize()/Klassen-Registrierung).
// serial_string/net_mac/net_mac_string kommen bereits fix-und-fertig aus Core/generated/
// device_ids.hh (DEVICE_USB_SERIAL_STRING/DEVICE_USB_NCM_MAC/DEVICE_USB_NCM_MAC_STRING, s. dortiger
// Klassenkommentar) -- pro Board eincompilierte Konstanten statt hier zur Laufzeit aus der
// Chip-UID zusammengesetzt (frueher per snprintf(), s. Commit-Historie). net_mac_string MUSS
// laut CDC1.2 Ethernet Networking Functional Descriptor (5.2.3.4) aus genau 12 Grossbuchstaben-
// Hexziffern ohne Trennzeichen bestehen und dieselben 6 Byte wie net_mac repraesentieren --
// device_ids.hh generiert beide zusammen, daher hier bewusst als zwei getrennte Parameter statt
// net_mac_string selbst aus net_mac herzuleiten (keine doppelte Formatierungslogik). Muss aus
// einem laufenden ThreadX-Thread heraus aufgerufen werden, NICHT vor tx_kernel_enter() (USBX
// braucht wie TinyUSB den laufenden Scheduler). Anders als bei TinyUSB startet USBX den
// eigentlichen USB-Controller (HAL_PCD_Start()) noch NICHT hier, sondern erst in
// usbd_device_loop(), sobald USB_VSENSE tatsaechlich 5V zeigt (s. dort) --
// ux_device_stack_initialize()/Klassen-Registrierung/ux_dcd_stm32_initialize() koennen dagegen
// bereits unabhaengig vom Kabel-Status passieren.
void usbd_device_setup(char const* serial_string, uint8_t const net_mac[6], char const* net_mac_string);

// Endlosschleife (kehrt nie zurueck): koppelt den USB-Controller per HAL_PCD_Start()/_Stop() an
// die tatsaechliche 5V-Praesenz am USB-Port (PC0/USB_VSENSE, 10k/10k-Spannungsteiler) -- analog
// zu tud_connect()/tud_disconnect() bei der TinyUSB-Fassung. Anders als dort gibt es aber KEINE
// tud_task()-Polling-Schleife mehr: USBX' Geraeteklassen (CDC-ACM x2, CDC-NCM) bedienen sich
// jeweils ueber eigene, von der Klasse selbst erzeugte ThreadX-Threads (bulkin_thread/
// bulkout_thread/interrupt_thread) komplett selbststaendig, angetrieben vom USB_DRD_FS-Interrupt
// (s. USB_DRD_FS_IRQHandler unten). Diese Schleife hier dient nur noch der VSENSE-Ueberwachung --
// ModbusRtuServer laeuft in seinem eigenen Thread (App::ModbusRtuThread()) und ruft dort direkt
// ux_device_class_cdc_acm_read()/_write() auf (s. usbd_cdc_modbus_instance() unten), ohne von
// hier aus angestossen zu werden (kein poll_hook mehr noetig).
_Noreturn void usbd_device_loop(void);

// Duenner Wrapper um die erste CDC-ACM-Instanz ("FactoryControl Debug") -- von syscalls.c's
// _write()-Spiegelung der Log-Ausgabe genutzt (s. dort). Nichtblockierend/best-effort wie
// TinyUSBs tud_cdc_n_write(0, ...): liefert 0 (verwirft die Ausgabe), solange kein Host
// verbunden ist oder eine vorherige Uebertragung noch nicht abgeschlossen wurde.
uint32_t usbd_debug_cdc_write(uint8_t const* buffer, uint32_t len);

// Opaker Zeiger auf die zweite CDC-ACM-Instanz ("FactoryControl Modbus", von
// cdc_modbus_activate()/_deactivate() in usbd_device.c gesetzt) -- NUR als Vorwaertsdeklaration
// (kein #include von ux_device_class_cdc_acm.h hier), damit diese Datei weiterhin ohne die vollen
// USBX-Header auskommt. ModbusRtuServer (C++, stm32_libs/modbus/modbus_rtu_server.hpp) bindet
// diesen einen Header selbst ein (dessen extern-"C"-Bloecke machen das C++-safe) und ruft
// ux_device_class_cdc_acm_read()/_write() direkt blockierend in ihrem eigenen Thread
// (App::ModbusRtuThread()) auf -- kein Zwischen-Wrapper, kein Ringpuffer/Mailbox/Zwischen-Thread
// mehr noetig (Modbus-RTU ist ohnehin strikt Anfrage/Antwort, Lesen und Schreiben passieren im
// Aufrufer nie gleichzeitig).
typedef struct UX_SLAVE_CLASS_CDC_ACM_STRUCT UX_SLAVE_CLASS_CDC_ACM;
UX_SLAVE_CLASS_CDC_ACM* usbd_cdc_modbus_instance(void);

// Vom Heartbeat-Thread (App::HeartbeatThread(), app.cc) periodisch aufzurufen, NICHT aus ISR-
// Kontext: liefert true und fuellt out_message, falls sich der USB-Geraetezustand seit dem
// letzten Aufruf geaendert hat (Attach/Detach/Suspend/Resume/...), sonst false. Ersetzt ein
// frueheres direktes log_info() aus dem USB-ISR heraus, das das Log-Mutex umging (ISR-Kontext
// erlaubt kein tx_mutex_get(), s. Kommentar bei usbx_device_state_change() in usbd_device.c).
bool usbd_device_poll_state_change(char *out_message, size_t out_message_size);

#ifdef __cplusplus
}
#endif
