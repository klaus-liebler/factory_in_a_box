#include "usb_ncm_driver.h"

#include <string.h>

#include "tusb.h"
#include "nx_link.h"
#include "tx_api.h"
#include "log.h"

// --- Zustand -----------------------------------------------------------------------------
// ip_ptr/pool_ptr/mac werden einmalig von usb_ncm_driver_init() gesetzt (vor
// nx_ip_interface_attach()) und danach nur noch gelesen. interface_index wird erst spaeter, beim
// tatsaechlichen NX_LINK_INTERFACE_ATTACH-Aufruf, aus driver_req_ptr->nx_ip_driver_interface
// ermittelt (nx_ip_interface_attach() sucht sich den ersten freien Interface-Slot selbst --
// nicht von aussen vorgebbar), analog zu nx_stm32_eth_driver.c's nx_driver_information.
static NX_IP *g_ip_ptr;
static NX_PACKET_POOL *g_pool_ptr;
static uint8_t g_mac[6];
static UINT g_interface_index;
static NX_INTERFACE *g_interface_ptr;

// 14 Byte: 6 (Ziel-MAC) + 6 (Quell-MAC) + 2 (EtherType) -- Groesse eines klassischen (unge-
// tagten) Ethernet-Headers, wie ihn NCM-Datagramme Byte-fuer-Byte tragen (kein eigenes
// Rahmenformat, s. CDC-NCM-Spezifikation 3.2 "Datagram encapsulation").
#define NCM_ETHERNET_HEADER_SIZE 14

enum {
    NCM_DRIVER_STATE_NOT_INITIALIZED = 0,
    NCM_DRIVER_STATE_INITIALIZED,
    NCM_DRIVER_STATE_LINK_ENABLED,
};
static volatile UCHAR g_driver_state;

// Einzelplatz-"Mailbox" fuer ein ausgehendes Paket -- s. usb_ncm_driver.h fuer die Begruendung,
// warum kein direkter tud_network_xmit()-Aufruf aus dem Treiber (NetX-IP-Thread-Kontext) heraus
// erfolgen darf. TX_DISABLE/TX_RESTORE schuetzen den Zeiger vor gleichzeitigem Zugriff aus
// ncm_driver_packet_send() (NetX-IP-Thread) und usb_ncm_driver_poll() (usbd_device_thread).
static NX_PACKET *volatile g_tx_pending;

void usb_ncm_driver_init(NX_IP *ip_ptr, NX_PACKET_POOL *pool_ptr, const uint8_t mac[6]) {
    g_ip_ptr = ip_ptr;
    g_pool_ptr = pool_ptr;
    memcpy(g_mac, mac, 6);
}

static void ncm_driver_interface_attach(NX_IP_DRIVER *driver_req_ptr) {
    driver_req_ptr->nx_ip_driver_status = NX_SUCCESS;
}

// Schreibt einen 14-Byte-Ethernet-Header (Ziel-MAC, Quell-MAC, EtherType) an data. msw/lsw-Paare
// folgen derselben Aufteilung wie NX_INTERFACE::nx_interface_physical_address_msw/lsw (msw = die
// ersten 2, lsw = die letzten 4 Byte der 6-Byte-MAC-Adresse).
static void write_ethernet_header(uint8_t *data, ULONG dest_msw, ULONG dest_lsw,
                                   ULONG src_msw, ULONG src_lsw, UINT ethertype) {
    data[0] = (uint8_t)(dest_msw >> 8);
    data[1] = (uint8_t)(dest_msw);
    data[2] = (uint8_t)(dest_lsw >> 24);
    data[3] = (uint8_t)(dest_lsw >> 16);
    data[4] = (uint8_t)(dest_lsw >> 8);
    data[5] = (uint8_t)(dest_lsw);
    data[6] = (uint8_t)(src_msw >> 8);
    data[7] = (uint8_t)(src_msw);
    data[8] = (uint8_t)(src_lsw >> 24);
    data[9] = (uint8_t)(src_lsw >> 16);
    data[10] = (uint8_t)(src_lsw >> 8);
    data[11] = (uint8_t)(src_lsw);
    data[12] = (uint8_t)(ethertype >> 8);
    data[13] = (uint8_t)(ethertype);
}

static void ncm_driver_initialize(NX_IP_DRIVER *driver_req_ptr) {
    NX_INTERFACE *interface_ptr = driver_req_ptr->nx_ip_driver_interface;
    g_interface_ptr = interface_ptr;
    g_interface_index = interface_ptr->nx_interface_index;

    interface_ptr->nx_interface_ip_mtu_size = CFG_TUD_NET_MTU - NX_LINK_ETHERNET_HEADER_SIZE;
    interface_ptr->nx_interface_physical_address_msw = ((ULONG)g_mac[0] << 8) | g_mac[1];
    interface_ptr->nx_interface_physical_address_lsw =
        ((ULONG)g_mac[2] << 24) | ((ULONG)g_mac[3] << 16) | ((ULONG)g_mac[4] << 8) | g_mac[5];
    interface_ptr->nx_interface_address_mapping_needed = NX_TRUE;

    g_driver_state = NCM_DRIVER_STATE_INITIALIZED;
    driver_req_ptr->nx_ip_driver_status = NX_SUCCESS;
}

// Kein echtes Kabel/Hardware-Enable noetig -- die virtuelle NIC ist "immer bereit", sobald der
// Host das NCM-Interface aktiviert (tud_network_recv_cb()/tud_network_xmit_cb() greifen erst
// dann tatsaechlich, unabhaengig von diesem Enable/Disable-Zustand). Wird bereits waehrend des
// IP-Thread-Starts aufgerufen (nx_ip_interface_attach() aus net_setup_create(), also lange bevor
// usbd_device_thread ueberhaupt laeuft) -- unkritisch, s. usb_ncm_driver.h.
static void ncm_driver_enable(NX_IP_DRIVER *driver_req_ptr) {
    if (g_driver_state < NCM_DRIVER_STATE_INITIALIZED) {
        driver_req_ptr->nx_ip_driver_status = NX_NOT_SUCCESSFUL;
        return;
    }
    g_driver_state = NCM_DRIVER_STATE_LINK_ENABLED;
    driver_req_ptr->nx_ip_driver_ptr->nx_ip_driver_link_up = NX_TRUE;
    driver_req_ptr->nx_ip_driver_status = NX_SUCCESS;
}

static void ncm_driver_disable(NX_IP_DRIVER *driver_req_ptr) {
    g_driver_state = NCM_DRIVER_STATE_INITIALIZED;
    driver_req_ptr->nx_ip_driver_ptr->nx_ip_driver_link_up = NX_FALSE;
    driver_req_ptr->nx_ip_driver_status = NX_SUCCESS;
}

static void ncm_driver_packet_send(NX_IP_DRIVER *driver_req_ptr) {
    NX_PACKET *packet_ptr = driver_req_ptr->nx_ip_driver_packet;

    if (g_driver_state != NCM_DRIVER_STATE_LINK_ENABLED) {
        nx_packet_transmit_release(packet_ptr);
        driver_req_ptr->nx_ip_driver_status = NX_NOT_SUCCESSFUL;
        return;
    }

    UINT packet_type;
    switch (driver_req_ptr->nx_ip_driver_command) {
        case NX_LINK_ARP_SEND:
        case NX_LINK_ARP_RESPONSE_SEND:
            packet_type = NX_LINK_ETHERNET_ARP;
            break;
        case NX_LINK_RARP_SEND:
            packet_type = NX_LINK_ETHERNET_RARP;
            break;
        default:
            packet_type = (packet_ptr->nx_packet_ip_version == 4) ? NX_LINK_ETHERNET_IP : NX_LINK_ETHERNET_IPV6;
            break;
    }

    // Ethernet-Header von Hand voranstellen (statt nx_link_ethernet_header_add() aus nx_link.c
    // zu nutzen -- dessen komplette Implementierung ist in dieser NetX-Duo-Version hinter
    // #ifdef NX_ENABLE_VLAN verborgen, s. Commit-Historie/Debugging dieser Aenderung; das ganze
    // VLAN-Subsystem nur fuer diese eine Hilfsfunktion zu aktivieren waere unverhaeltnismaessig).
    // Quelladresse bewusst NICHT ueber das legacy nx_ip_arp_physical_address_msw/lsw-Makro (s.
    // nx_api.h: fest auf nx_ip_interface[0] verdrahtet!) -- das waere bei einem zweiten, NICHT
    // Interface-0-Link wie diesem hier schlicht falsch. g_interface_ptr zeigt auf das tatsaechlich
    // von nx_ip_interface_attach() zugewiesene Interface (s. ncm_driver_initialize()).
    packet_ptr->nx_packet_prepend_ptr -= NCM_ETHERNET_HEADER_SIZE;
    packet_ptr->nx_packet_length += NCM_ETHERNET_HEADER_SIZE;
    write_ethernet_header(packet_ptr->nx_packet_prepend_ptr,
                           driver_req_ptr->nx_ip_driver_physical_address_msw,
                           driver_req_ptr->nx_ip_driver_physical_address_lsw,
                           g_interface_ptr->nx_interface_physical_address_msw,
                           g_interface_ptr->nx_interface_physical_address_lsw,
                           packet_type);

    // Bereits ein Paket unterwegs (usb_ncm_driver_poll() noch nicht dazu gekommen, es zu
    // senden) -- verwerfen statt den NetX-IP-Thread hier blockierend warten zu lassen. TCP
    // wiederholt verlorene Segmente ohnehin selbst; fuer UDP (DHCP/mDNS) ist das dieselbe
    // Best-Effort-Semantik wie bei jeder echten Netzwerkkarte unter Last.
    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    bool dropped = (g_tx_pending != NULL);
    if (!dropped) {
        g_tx_pending = packet_ptr;
    }
    TX_RESTORE

    if (dropped) {
        nx_packet_transmit_release(packet_ptr);
        driver_req_ptr->nx_ip_driver_status = NX_NOT_SUCCESSFUL;
        return;
    }

    driver_req_ptr->nx_ip_driver_status = NX_SUCCESS;
}

void nx_usb_ncm_driver(NX_IP_DRIVER *driver_req_ptr) {
    driver_req_ptr->nx_ip_driver_status = NX_SUCCESS;

    switch (driver_req_ptr->nx_ip_driver_command) {
        case NX_LINK_INTERFACE_ATTACH:
            ncm_driver_interface_attach(driver_req_ptr);
            break;
        case NX_LINK_INITIALIZE:
            ncm_driver_initialize(driver_req_ptr);
            break;
        case NX_LINK_ENABLE:
            ncm_driver_enable(driver_req_ptr);
            break;
        case NX_LINK_DISABLE:
            ncm_driver_disable(driver_req_ptr);
            break;
        case NX_LINK_ARP_SEND:
        case NX_LINK_ARP_RESPONSE_SEND:
        case NX_LINK_PACKET_BROADCAST:
        case NX_LINK_RARP_SEND:
        case NX_LINK_PACKET_SEND:
            ncm_driver_packet_send(driver_req_ptr);
            break;
        case NX_LINK_MULTICAST_JOIN:
        case NX_LINK_MULTICAST_LEAVE:
            // Kein echter Hardware-Multicast-Filter -- der virtuelle Link empfaengt ohnehin
            // "promiscuous" alles, was der Host per NCM sendet (s. tud_network_recv_cb()).
            break;
        case NX_LINK_GET_STATUS:
            *(driver_req_ptr->nx_ip_driver_return_ptr) = (g_driver_state == NCM_DRIVER_STATE_LINK_ENABLED) ? NX_TRUE : NX_FALSE;
            break;
        default:
            driver_req_ptr->nx_ip_driver_status = NX_UNHANDLED_COMMAND;
            break;
    }
}

// --- TinyUSB-Netzwerk-Callbacks (laufen aus tud_task()-Kontext, also im usbd_device_thread) ----

// Vom NCM-Treiber pro empfangenem Datagramm aufgerufen. Kopiert sofort in ein eigenes NX_PACKET
// (der Quellpuffer src bleibt nur bis zur Rueckkehr gueltig), entfernt den Ethernet-Header von
// Hand (aus demselben Grund wie beim Senden kein nx_link_ethernet_packet_received() aus
// nx_link.c -- s. Kommentar in ncm_driver_packet_send()) und reicht das Paket ueber
// _nx_ip_packet_deferred_receive()/_nx_arp_packet_deferred_receive()/
// _nx_rarp_packet_deferred_receive() weiter (nx_api.h: "for exclusive use by NetX I/O drivers",
// exakt dieselben Funktionen, die auch nx_stm32_eth_driver.c verwendet). Diese Funktionen sind
// fuer den Aufruf aus einem beliebigen Thread-Kontext ausgelegt (sie queuen das Paket nur fuer
// den eigentlichen NetX-IP-Thread), muessen also NICHT selbst schon im IP-Thread laufen.
bool tud_network_recv_cb(const uint8_t *src, uint16_t size) {
    if (g_driver_state != NCM_DRIVER_STATE_LINK_ENABLED || size < NCM_ETHERNET_HEADER_SIZE) {
        tud_network_recv_renew();
        return true;
    }

    NX_PACKET *packet_ptr;
    if (nx_packet_allocate(g_pool_ptr, &packet_ptr, NX_RECEIVE_PACKET, NX_NO_WAIT) != NX_SUCCESS) {
        // Pool momentan erschoepft -- Datagramm verwerfen statt zu blockieren (wuerde
        // usbd_device_thread und damit auch die anderen USB-Funktionen mit anhalten).
        tud_network_recv_renew();
        return true;
    }

    if (nx_packet_data_append(packet_ptr, (void *)src, size, g_pool_ptr, NX_NO_WAIT) != NX_SUCCESS) {
        nx_packet_release(packet_ptr);
        tud_network_recv_renew();
        return true;
    }

    packet_ptr->nx_packet_ip_interface = g_interface_ptr;

    UINT ethertype = ((UINT)src[12] << 8) | src[13];
    packet_ptr->nx_packet_prepend_ptr += NCM_ETHERNET_HEADER_SIZE;
    packet_ptr->nx_packet_length -= NCM_ETHERNET_HEADER_SIZE;

    if (ethertype == NX_LINK_ETHERNET_IP || ethertype == NX_LINK_ETHERNET_IPV6) {
        _nx_ip_packet_deferred_receive(g_ip_ptr, packet_ptr);
    } else if (ethertype == NX_LINK_ETHERNET_ARP) {
        _nx_arp_packet_deferred_receive(g_ip_ptr, packet_ptr);
    } else if (ethertype == NX_LINK_ETHERNET_RARP) {
        _nx_rarp_packet_deferred_receive(g_ip_ptr, packet_ptr);
    } else {
        // Unbekannter EtherType -- verwerfen (analog nx_stm32_eth_driver.c).
        nx_packet_release(packet_ptr);
    }

    tud_network_recv_renew();
    return true;
}

// Vom NCM-Treiber synchron aus tud_network_xmit() heraus aufgerufen (s. usb_ncm_driver_poll()) --
// kopiert den kompletten Ethernet-Frame (Header bereits in ncm_driver_packet_send() von Hand
// ergaenzt) aus dem NX_PACKET (ref) in den vom NCM-Treiber bereitgestellten NTB-Puffer dst.
uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg) {
    (void)arg;
    NX_PACKET *packet_ptr = (NX_PACKET *)ref;
    ULONG bytes_copied = 0;
    nx_packet_data_extract_offset(packet_ptr, 0, dst, packet_ptr->nx_packet_length, &bytes_copied);
    return (uint16_t)bytes_copied;
}

void usb_ncm_driver_poll(void) {
    if (!tud_ready()) {
        return;
    }

    TX_INTERRUPT_SAVE_AREA
    TX_DISABLE
    NX_PACKET *packet_ptr = g_tx_pending;
    TX_RESTORE

    if (packet_ptr == NULL) {
        return;
    }
    if (!tud_network_can_xmit(packet_ptr->nx_packet_length)) {
        return;  // naechster Poll versucht es erneut
    }

    // Synchroner Aufruf: tud_network_xmit() ruft intern sofort tud_network_xmit_cb() auf, die
    // Bytes sind danach bereits kopiert -- das NX_PACKET kann sofort freigegeben werden.
    tud_network_xmit(packet_ptr, 0);

    TX_DISABLE
    g_tx_pending = NULL;
    TX_RESTORE

    nx_packet_transmit_release(packet_ptr);
}
