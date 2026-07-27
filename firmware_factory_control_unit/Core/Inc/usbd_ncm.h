#pragma once
// Eigene, projektinterne USBX-Geraeteklasse fuer CDC-NCM (Network Control Model, USB-IF-
// Standard, CDC1.2 + [USBNCM1.0]) -- ersetzt das gescheiterte RNDIS-Interface (Windows'
// Legacy-RNDIS-Treiber crashte den Host, s. Commit-Historie/Debugging-Sitzung). Anders als
// CDC-ACM/RNDIS/CDC-ECM (alle von ST/Microsoft als fertige USBX-Geraeteklasse in libs/ST/usbx
// mitgeliefert) gibt es dort KEINE NCM-Klasse -- diese Datei ist deshalb eine komplette
// Neuimplementierung, KEIN Vendor-Code, daher bewusst in Core/ statt libs/ST/usbx/ abgelegt.
//
// Wire-Format-Referenz (NUR das Byteformat auf dem USB-Draht, s. usbd_ncm.c): TinyUSBs
// ncm_device.c (stm32_libs/tinyusb_port/tinyusb/src/class/net/), mit dem dieses Projekt VOR der
// USBX-Migration bereits hardwareverifiziert per DHCP+HTTPS ueber USB-NCM lief.
//
// NetX-Duo-Anbindung -- bewusst NICHT nach TinyUSB-Vorbild (eigener "Treiber" mit eigener
// Warteschlange, per App-Poll-Hook angestossen), sondern exakt nach dem Vorbild von USBX' EIGENEN
// Netzwerk-Geraeteklassen (CDC-ECM/RNDIS, s. deren activate.c/write.c/bulkin_thread.c/change.c in
// libs/ST/usbx/common/usbx_device_classes/src/ux_device_class_cdc_ecm_*.c, per "git show
// 15ecbe5:...cdc_ecm_*.c" einsehbar -- physisch aus dem Arbeitsverzeichnis entfernt, aber weiter
// im Git-Verlauf vorhanden) -- also ueber USBX' eigenen generischen Netzwerk-Bruecken-Treiber
// (libs/ST/usbx/common/usbx_network/{inc,src}/ux_network_driver.*, _ux_network_driver_activate/
// _deactivate/_link_up/_link_down/_packet_received): kein eigener Treiber, kein App-Poll-Hook,
// kein separates TX_QUEUE-Konstrukt -- NX_PACKETs werden ueber ihr eigenes
// nx_packet_queue_next-Feld intrusiv verkettet (wie ECMs xmit_queue) und der Versand laeuft rein
// ereignisgetrieben (Semaphore weckt ncm_bulkin_thread, statt per Timer/Poll-Zyklus abgefragt zu
// werden).
//
// USB-Alternate-Setting: das Daten-Interface hat wie bei CDC-ECM ZWEI Auspraegungen (Alt=0: 0
// Endpunkte/inaktiv, Alt=1: 2 Bulk-Endpunkte/aktiv). WICHTIG (per Studium von
// ux_device_stack_alternate_setting_set.c verifiziert, s. dortiger Aufruf von
// class_ptr->ux_slave_class_entry_function() mit UX_SLAVE_CLASS_COMMAND_CHANGE): USBX' Kern ruft
// bei SET_INTERFACE **NICHT** erneut ACTIVATE/DEACTIVATE auf, sondern einen eigenen
// UX_SLAVE_CLASS_COMMAND_CHANGE-Befehl (s. ncm_change() in usbd_ncm.c) -- eine fruehere Fassung
// dieser Datei nahm faelschlich an, ACTIVATE wuerde erneut aufgerufen, wodurch die
// Bulk-Endpunkte beim Wechsel auf Alt=1 NIE gefunden worden waeren (Datenverkehr haette also nie
// funktioniert). CDC-ECMs change.c/activate.c dienten als Vorlage fuer die korrekte Aufteilung.

#include <stdint.h>

#include "ux_api.h"
#include "ux_device_stack.h"

#ifdef __cplusplus
extern "C" {
#endif

// Registriert die Klasse bei USBX (ux_device_stack_class_register()) -- muss NACH
// ux_device_stack_initialize() und VOR ux_dcd_stm32_initialize()/HAL_NVIC_EnableIRQ() laufen,
// exakt wie die CDC-ACM-Registrierungen in usbd_device.c. itf_control ist die
// Control-Interface-Nummer (das Daten-Interface liegt per Konvention -- s. TUD_CDC_NCM_DESCRIPTOR
// -- auf itf_control+1). net_mac: 6 Bytes lokale MAC-Adresse der virtuellen NIC -- an
// _ux_network_driver_activate() weitergereicht (physical_address_msw/lsw), exakt wie CDC-ECMs
// ux_slave_class_cdc_ecm_local_node_id.
UINT usbd_ncm_class_register(ULONG itf_control, uint8_t const net_mac[6]);

#ifdef __cplusplus
}
#endif
