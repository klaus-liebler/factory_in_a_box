#pragma once
// Glue-Treiber zwischen TinyUSBs CDC-NCM-Geraeteklasse (virtuelle Netzwerkkarte ueber USB) und
// NetX Duo -- vom NX_IP_DRIVER-Kommandomuster her analog zu
// libs/ST/netxduo/common/drivers/ethernet/nx_stm32_eth_driver.c, aber die "Hardware" ist hier
// kein echter Ethernet-Controller, sondern TinyUSBs tud_network_*()-API (s. usb_ncm_driver.c).
//
// Reines C wie usbd_device.c/.h (nicht .cc/.hh): usb_ncm_driver.c bindet tusb.h ein, das ueber
// TinyUSBs ThreadX-OSAL-Anbindung in C++-Uebersetzungseinheiten nicht kompiliert (s.
// Klassenkommentar in usbd_device.h). nx_api.h selbst ist reines C und daher hier in diesem, von
// C++-Aufrufern (net_setup.cpp) inkludierten Header unproblematisch.
#include <stdint.h>
#include "nx_api.h"

#ifdef __cplusplus
extern "C" {
#endif

// Einmalig aus net_setup_create() aufzurufen, BEVOR nx_ip_interface_attach() mit
// nx_usb_ncm_driver() als Treiberfunktion aufgerufen wird -- hinterlegt IP-Instanz/Paket-Pool
// sowie die MAC-Adresse der virtuellen NIC (dieselben 6 Bytes, die auch usbd_device_setup() fuer
// den MAC-Adress-String-Deskriptor bekommt, s. App::ComputeNcmMac() in app.hh/.cc). Erzeugt dabei
// auch die interne TX_QUEUE fuer ausgehende Pakete -- Rueckgabewert pruefen (ThreadX-Objekterzeugung
// kann wie ueberall im Projekt per XASSERT abgesichert werden).
UINT usb_ncm_driver_init(NX_IP *ip_ptr, NX_PACKET_POOL *pool_ptr, const uint8_t mac[6]);

// Die eigentliche NetX-Duo-Treiberfunktion, an nx_ip_interface_attach() zu uebergeben.
void nx_usb_ncm_driver(NX_IP_DRIVER *driver_req_ptr);

// Muss regelmaessig aus dem usbd_device_thread heraus aufgerufen werden (s.
// App::UsbdDevicePollHook(), analog zu ModbusRtuServer::Poll()) -- sendet von der NetX-IP-Thread-
// Seite zwischengepufferte ausgehende Pakete, sobald TinyUSB dafuer bereit ist.
// tud_network_xmit()/tud_network_can_xmit() duerfen NUR aus demselben Thread-Kontext wie
// tud_task() aufgerufen werden (TinyUSBs Geraete-Stack ist nicht thread-safe) -- der eigentliche
// NX_LINK_PACKET_SEND-Aufruf laeuft dagegen im NetX-IP-Thread, deshalb dieser Umweg ueber eine
// TX_QUEUE statt direktem Aufruf aus dem Treiber heraus.
void usb_ncm_driver_poll(void);

#ifdef __cplusplus
}
#endif
