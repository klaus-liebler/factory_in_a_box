// Eigene USBX-Geraeteklasse fuer CDC-NCM -- s. Klassenkommentar in usbd_ncm.h fuer den
// vollstaendigen Hintergrund (warum kein Vendor-Code, Referenzquelle, warum die NetX-Duo-
// Anbindung ueber USBX' eigenen generischen ux_network_driver statt eines eigenen Treibers
// laeuft). Reines C wie usbd_device.c (dieselbe Begruendung: USBX/ThreadX/NetX-Duo-Header sind C,
// keine Notwendigkeit fuer C++ hier).

#include "usbd_ncm.h"

#include <string.h>

#include "ux_api.h"
#include "ux_device_stack.h"
#include "ux_network_driver.h"
#include "tx_api.h"
#include "nx_api.h"
#include "log.h"

// --- CDC-NCM Wire-Format (CDC1.2 + [USBNCM1.0]) -------------------------------------------
// Byteweise identisch zu TinyUSBs ncm.h (stm32_libs/tinyusb_port/tinyusb/src/class/net/ncm.h)
// -- das ist der Teil, der tatsaechlich ueber USB auf den Draht geht und mit dem Windows-
// Host-Treiber interoperieren muss, deshalb bewusst 1:1 uebernommen statt neu entworfen.
// __attribute__((packed)): GCC-Aequivalent zu TinyUSBs TU_ATTR_PACKED (kein Padding zwischen
// den Feldern -- diese Structs werden direkt auf/von den Draht gemappt).
#define NCM_NTH16_SIGNATURE 0x484D434Eu
#define NCM_NDP16_SIGNATURE_NCM0 0x304D434Eu

typedef struct __attribute__((packed)) {
    uint32_t dwSignature;
    uint16_t wHeaderLength;
    uint16_t wSequence;
    uint16_t wBlockLength;
    uint16_t wNdpIndex;
} ncm_nth16_t;

typedef struct __attribute__((packed)) {
    uint16_t wDatagramIndex;
    uint16_t wDatagramLength;
} ncm_ndp16_datagram_t;

typedef struct __attribute__((packed)) {
    uint32_t dwSignature;
    uint16_t wLength;
    uint16_t wNextNdpIndex;
} ncm_ndp16_t;

// Bis zu 8 Datagramme je gesendetem NTB (+1 Nullterminator-Eintrag) -- mehrere kurz
// hintereinander eintreffende NX_PACKETs (z.B. DHCP+ARP) werden von ncm_bulkin_thread in EINEM
// NTB gebuendelt, s. dort.
#define NCM_XMIT_MAX_DATAGRAMS 8u

// GET_NTB_PARAMETERS-Antwortstruktur (CDC-NCM 1.0 Tabelle 6-3).
typedef struct __attribute__((packed)) {
    uint16_t wLength;
    uint16_t bmNtbFormatsSupported;
    uint32_t dwNtbInMaxSize;
    uint16_t wNdbInDivisor;
    uint16_t wNdbInPayloadRemainder;
    uint16_t wNdbInAlignment;
    uint16_t wReserved;
    uint32_t dwNtbOutMaxSize;
    uint16_t wNdbOutDivisor;
    uint16_t wNdbOutPayloadRemainder;
    uint16_t wNdbOutAlignment;
    uint16_t wNtbOutMaxDatagrams;
} ncm_ntb_parameters_t;

// CDC-Benachrichtigung (Notification-Endpunkt) -- 8-Byte-Header (identisch zum Setup-Paket-
// Layout, s. UX_SETUP_*-Offsets) + optionale Nutzdaten (hier: 8 Byte fuer
// CONNECTION_SPEED_CHANGE, 0 Byte fuer NETWORK_CONNECTION).
typedef struct __attribute__((packed)) {
    uint8_t bmRequestType;
    uint8_t bNotification;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
    uint32_t data[2];  // nur von CONNECTION_SPEED_CHANGE genutzt (downlink/uplink bps)
} ncm_notify_t;

#define NCM_NOTIF_NETWORK_CONNECTION 0x00u
#define NCM_NOTIF_CONNECTION_SPEED_CHANGE 0x2Au
#define NCM_NOTIF_REQUEST_TYPE 0xA1u  // Device-to-Host, Class, Interface

// CDC-NCM-Klassen-Requests (CDC-NCM 1.0 Tabelle 6-2) -- nur die tatsaechlich behandelten hier
// aufgefuehrt (bmCapabilities=0 im Deskriptor, s. usbd_device.c: der Host fragt die restigen
// optionalen (CRC_MODE, NET_ADDRESS, ...) mangels beworbener Capability gar nicht erst ab).
#define NCM_GET_NTB_PARAMETERS 0x80u
#define NCM_SET_ETHERNET_PACKET_FILTER 0x43u
#define NCM_GET_NTB_INPUT_SIZE 0x85u
#define NCM_SET_NTB_INPUT_SIZE 0x86u

// --- Groessen/Zeitkonstanten ----------------------------------------------------------------
// 2048 Byte: CDC-NCM 1.0 Tabelle 6-4 Mindestgroesse -- reicht fuer dieses Interface (Modbus-
// Webkonfiguration/HTTPS-Steuerverkehr, kein Hochdurchsatz-Streaming), s. historischer
// TinyUSB-Kommentar in tusb_config.h (per Git-Historie wiederhergestellt).
#define NCM_NTB_MAX_SIZE 2048u
#define NCM_EP_SIZE 64u
// 14 Byte: 6 (Ziel-MAC) + 6 (Quell-MAC) + 2 (EtherType) -- Groesse des Ethernet-Headers, den
// _ux_network_driver_entry() (ux_network_driver.c, NX_LINK_PACKET_SEND) VOR jedem ausgehenden
// NX_PACKET voranstellt bzw. den ncm_bulkout_thread selbst vor jedem eingehenden Datagramm
// ergaenzen muss, bevor es an _ux_network_driver_packet_received() geht -- identische Konvention
// zu USBX' eigener CDC-ECM-Klasse (UX_DEVICE_CLASS_CDC_ECM_ETHERNET_SIZE).
#define NCM_ETHERNET_HEADER_SIZE 14u
// Gemeinsame Stackgroesse fuer alle drei Klassen-Threads (Notif/Bulk-IN/Bulk-OUT) -- der
// Notif-Thread lief frueher mit knapperen 512 Byte, was bei -O0 (unoptimierter Debug-Build) in
// dessen tiefster Aufrufkette (ncm_notify_send() -> _ux_device_stack_transfer_request() -> ... ->
// ThreadX' _tx_thread_system_suspend(), 8 Ebenen) zum Stapelueberlauf fuehrte: der Ueberlauf lief
// nach UNTEN (ThreadX-Stacks wachsen absteigend) direkt in notif_thread's EIGENEN
// TX_THREAD-Kontrollblock hinein (steht in usbd_ncm_t VOR notif_thread_stack) und zerstoerte
// dessen interne Ready-Liste -- Symptom war ein HardFault mitten in _tx_thread_system_suspend
// beim allerersten Aufwachen dieses Threads (SET_INTERFACE zu Alt=1).
#define NCM_THREAD_STACK_SIZE 1024u
#define NCM_THREAD_PRIORITY 6u  // wie USBD_DEVICE_THREAD_PRIORITY in app.hh/app.cc

// --- Klasseninstanz (genau EINE, statisch -- kein generisches Mehrfachinstanz-Design wie bei
// den ST-eigenen USBX-Klassen noetig, dieses Geraet hat nur ein einziges NCM-Interface). --------
typedef struct {
    UX_SLAVE_ENDPOINT *ep_notif;
    UX_SLAVE_ENDPOINT *ep_in;
    UX_SLAVE_ENDPOINT *ep_out;
    ULONG itf_control;
    ULONG itf_data;
    volatile UINT data_alt_active;  // 0 = Alt-Setting 0 (inaktiv), 1 = Alt-Setting 1 (Bulk-EPs aktiv, Link oben)
    UCHAR mac[6];

    // NetX-Duo-Anbindung ueber USBX' generischen ux_network_driver (s. Klassenkommentar in
    // usbd_ncm.h) -- network_handle wird von _ux_network_driver_activate() in ncm_activate()
    // vergeben, packet_pool erst spaeter lazy aus dessen ux_network_device_ip_instance
    // aufgeloest (exakt das Muster aus CDC-ECMs bulkout_thread.c: zum Zeitpunkt von
    // ncm_activate() ist die NX_IP-Instanz zwar am Handle vermerkt, aber ncm_bulkout_thread
    // greift ohnehin erst beim ersten tatsaechlichen Empfang darauf zu).
    VOID *network_handle;
    NX_PACKET_POOL *packet_pool;

    // Sendepfad: intrusiv verkettete NX_PACKET-Warteschlange (ueber deren eigenes
    // nx_packet_queue_next-Feld, exakt wie UX_SLAVE_CLASS_CDC_ECM_STRUCTs xmit_queue) --
    // ncm_network_write() (als write_function bei _ux_network_driver_activate() registriert,
    // synchron aus dem NetX-IP-Thread heraus aufgerufen sobald ein Paket zu senden ist) haengt
    // Pakete hinten an und weckt ncm_bulkin_thread per Semaphore; kein separates TX_QUEUE/
    // App-Poll-Hook-Konstrukt mehr noetig (anders als eine fruehere TinyUSB-Bruecken-Fassung
    // dieser Datei).
    TX_MUTEX xmit_mutex;
    NX_PACKET *xmit_queue_head;
    NX_PACKET *xmit_queue_tail;
    TX_SEMAPHORE xmit_ready_sem;
    uint8_t xmit_buf[NCM_NTB_MAX_SIZE] __attribute__((aligned(4)));
    uint16_t xmit_sequence;

    // Empfangspfad: EIN Puffer, ausschliesslich vom ncm_bulkout_thread beschrieben; die daraus
    // extrahierten Datagramme werden je in ein frisches NX_PACKET kopiert und SYNCHRON (noch im
    // selben Thread-Kontext) per _ux_network_driver_packet_received() an NetX Duo uebergeben,
    // bevor der Puffer fuer die naechste Empfangstransaktion wiederverwendet wird.
    uint8_t recv_buf[NCM_NTB_MAX_SIZE] __attribute__((aligned(4)));

    TX_THREAD notif_thread;
    UCHAR notif_thread_stack[NCM_THREAD_STACK_SIZE];
    TX_SEMAPHORE notif_kick_sem;  // "Alt-Setting 1 wurde aktiv, Benachrichtigungen faellig"

    TX_THREAD bulkin_thread;
    UCHAR bulkin_thread_stack[NCM_THREAD_STACK_SIZE];

    TX_THREAD bulkout_thread;
    UCHAR bulkout_thread_stack[NCM_THREAD_STACK_SIZE];
} usbd_ncm_t;

static usbd_ncm_t g_ncm;

// bInterfaceClass-Bytes laut unserem eigenen Deskriptor (s. usbd_device.c) -- Communications
// (0x02) fuer das Control-Interface, CDC-Data (0x0A) fuer das Daten-Interface. Dieselben
// numerischen Werte wie bei CDC-ACM/CDC-ECM (Standard-USB-Klassencodes, kein NCM-Spezifikum).
#define NCM_CLASS_COMMUNICATION_CONTROL 0x02u
#define NCM_CLASS_COMMUNICATION_DATA 0x0Au

//----------------------------------------------------------------------------------------------
// --- Benachrichtigungen (Interrupt-IN-Endpunkt) -----------------------------------------------
//----------------------------------------------------------------------------------------------

// Sendet EINE Benachrichtigung blockierend (USBX' _ux_device_stack_transfer_request() ist
// synchron -- anders als TinyUSBs usbd_edpt_xfer(), das nur den Transfer ANSTOESST und den
// Abschluss per netd_xfer_cb()-Callback zurueckmeldet). Deshalb genuegt hier ein einfacher,
// sequentieller Ablauf (SPEED_CHANGE, dann NETWORK_CONNECTION) ohne Zustandsautomat.
static void ncm_notify_send(uint8_t bNotification, uint16_t wValue, uint16_t wLength, uint32_t const *data) {
    static ncm_notify_t notif;  // static: bleibt bis zum Abschluss von _ux_device_stack_transfer_request() gueltig
    UX_SLAVE_TRANSFER *transfer_request;

    if (g_ncm.ep_notif == UX_NULL) {
        return;
    }

    memset(&notif, 0, sizeof(notif));
    notif.bmRequestType = NCM_NOTIF_REQUEST_TYPE;
    notif.bNotification = bNotification;
    notif.wValue = wValue;
    notif.wIndex = (uint16_t)g_ncm.itf_control;
    notif.wLength = wLength;
    if (data != UX_NULL) {
        memcpy(notif.data, data, wLength);
    }

    transfer_request = &g_ncm.ep_notif->ux_slave_endpoint_transfer_request;
    transfer_request->ux_slave_transfer_request_data_pointer = (UCHAR *)&notif;
    _ux_device_stack_transfer_request(transfer_request, (ULONG)(8u + wLength), (ULONG)(8u + wLength));
}

static void ncm_notif_thread_entry(ULONG arg) {
    (void)arg;
    for (;;) {
        tx_semaphore_get(&g_ncm.notif_kick_sem, TX_WAIT_FOREVER);

        // NETWORK_CONNECTION_SPEED_CHANGE zuerst, dann NETWORK_CONNECTION -- Reihenfolge laut
        // CDC1.2 8.6.1/8.6.2. Full-Speed-only (USB_DRD_FS): 12 Mbit/s in beide Richtungen.
        uint32_t const speed[2] = {12000000u, 12000000u};
        ncm_notify_send(NCM_NOTIF_CONNECTION_SPEED_CHANGE, 0, 8u, speed);
        ncm_notify_send(NCM_NOTIF_NETWORK_CONNECTION, g_ncm.data_alt_active ? 1u : 0u, 0, UX_NULL);
    }
}

//----------------------------------------------------------------------------------------------
// --- Sendepfad (NetX Duo -> Host, Bulk-IN) ------------------------------------------------------
//----------------------------------------------------------------------------------------------

// Als write_function bei _ux_network_driver_activate() registriert (s. ncm_activate()) --
// _ux_network_driver_entry() (ux_network_driver.c) ruft diese Funktion SYNCHRON aus dem
// NX_LINK_PACKET_SEND-Zweig heraus auf, NACHDEM es bereits selbst den vollstaendigen
// Ethernet-Header vor packet_ptr gesetzt hat (prepend_ptr -= 14, length += 14) -- und nur dann,
// wenn der Link-Status am generischen Treiber bereits als "up" hinterlegt ist (s. dortige
// Pruefung), ein Aufruf bei inaktivem Link kommt hier also gar nicht erst an. Analog zu
// _ux_device_class_cdc_ecm_write(): haengt das Paket nur in die eigene Warteschlange ein
// (intrusiv ueber packet_ptr->nx_packet_queue_next) und weckt ncm_bulkin_thread -- der
// eigentliche NTB-Aufbau/Versand passiert dort, NICHT hier (dieser Aufruf darf den NetX-IP-Thread
// nicht blockieren).
static UINT ncm_network_write(VOID *ncm_class, NX_PACKET *packet_ptr) {
    (void)ncm_class;

    tx_mutex_get(&g_ncm.xmit_mutex, TX_WAIT_FOREVER);
    packet_ptr->nx_packet_queue_next = NX_NULL;
    if (g_ncm.xmit_queue_head == NX_NULL) {
        g_ncm.xmit_queue_head = packet_ptr;
    } else {
        g_ncm.xmit_queue_tail->nx_packet_queue_next = packet_ptr;
    }
    g_ncm.xmit_queue_tail = packet_ptr;
    tx_mutex_put(&g_ncm.xmit_mutex);

    tx_semaphore_put(&g_ncm.xmit_ready_sem);
    return UX_SUCCESS;
}

// Baut aus bis zu NCM_XMIT_MAX_DATAGRAMS wartenden NX_PACKETs EIN NTB (NTH16+NDP16-Header +
// Datagramme) und sendet es blockierend -- analog zu CDC-ECMs bulkin_thread.c, nur mit NCMs
// NTB-Buendelung mehrerer Datagramme statt eines Pakets pro Bulk-Transfer.
static void ncm_bulkin_thread_entry(ULONG arg) {
    (void)arg;
    UX_SLAVE_DEVICE *device = &_ux_system_slave->ux_system_slave_device;

    UINT const header_len = (UINT)(sizeof(ncm_nth16_t) + sizeof(ncm_ndp16_t) +
                                    (NCM_XMIT_MAX_DATAGRAMS + 1u) * sizeof(ncm_ndp16_datagram_t));

    for (;;) {
        tx_semaphore_get(&g_ncm.xmit_ready_sem, TX_WAIT_FOREVER);

        for (;;) {
            ncm_nth16_t *nth = (ncm_nth16_t *)(void *)g_ncm.xmit_buf;
            ncm_ndp16_t *ndp = (ncm_ndp16_t *)(void *)(g_ncm.xmit_buf + sizeof(ncm_nth16_t));
            ncm_ndp16_datagram_t *table =
                (ncm_ndp16_datagram_t *)(void *)(g_ncm.xmit_buf + sizeof(ncm_nth16_t) + sizeof(ncm_ndp16_t));
            UINT block_length = header_len;
            UINT datagram_count = 0;

            memset(table, 0, (NCM_XMIT_MAX_DATAGRAMS + 1u) * sizeof(ncm_ndp16_datagram_t));

            while (datagram_count < NCM_XMIT_MAX_DATAGRAMS) {
                NX_PACKET *packet_ptr;
                ULONG copied;
                UINT align_offset;

                tx_mutex_get(&g_ncm.xmit_mutex, TX_WAIT_FOREVER);
                packet_ptr = g_ncm.xmit_queue_head;
                if (packet_ptr != NX_NULL &&
                    block_length + packet_ptr->nx_packet_length + 3u <= NCM_NTB_MAX_SIZE) {
                    g_ncm.xmit_queue_head = packet_ptr->nx_packet_queue_next;
                    if (g_ncm.xmit_queue_head == NX_NULL) {
                        g_ncm.xmit_queue_tail = NX_NULL;
                    }
                } else {
                    packet_ptr = NX_NULL;
                }
                tx_mutex_put(&g_ncm.xmit_mutex);

                if (packet_ptr == NX_NULL) {
                    break;
                }

                nx_packet_data_extract_offset(packet_ptr, 0, g_ncm.xmit_buf + block_length,
                                               packet_ptr->nx_packet_length, &copied);
                table[datagram_count].wDatagramIndex = (uint16_t)block_length;
                table[datagram_count].wDatagramLength = (uint16_t)copied;
                datagram_count++;

                align_offset = (4u - ((UINT)copied & 3u)) & 3u;
                block_length += (UINT)copied + align_offset;

                // Undo den Ethernet-Header-Vorschub, den _ux_network_driver_entry() beim
                // Einreihen dieses Pakets vorgenommen hatte -- exakt CDC-ECMs bulkin_thread.c-
                // Konvention, bevor das Paket an NetX zurueckgegeben wird.
                packet_ptr->nx_packet_prepend_ptr += NCM_ETHERNET_HEADER_SIZE;
                packet_ptr->nx_packet_length -= NCM_ETHERNET_HEADER_SIZE;
                nx_packet_transmit_release(packet_ptr);
            }

            if (datagram_count == 0) {
                break;  // Warteschlange leer (oder naechstes Paket passt nicht mehr) -- fertig fuer diesen Weck-Zyklus
            }

            nth->dwSignature = NCM_NTH16_SIGNATURE;
            nth->wHeaderLength = (uint16_t)sizeof(ncm_nth16_t);
            nth->wSequence = g_ncm.xmit_sequence++;
            nth->wBlockLength = (uint16_t)block_length;
            nth->wNdpIndex = (uint16_t)sizeof(ncm_nth16_t);

            ndp->dwSignature = NCM_NDP16_SIGNATURE_NCM0;
            ndp->wLength = (uint16_t)(sizeof(ncm_ndp16_t) + (NCM_XMIT_MAX_DATAGRAMS + 1u) * sizeof(ncm_ndp16_datagram_t));
            ndp->wNextNdpIndex = 0;

            if (device->ux_slave_device_state != UX_DEVICE_CONFIGURED || g_ncm.data_alt_active == 0 || g_ncm.ep_in == UX_NULL) {
                continue;  // Verbindung waehrenddessen verloren -- NTB verwerfen, naechsten Batch pruefen
            }

            UX_SLAVE_TRANSFER *transfer_request = &g_ncm.ep_in->ux_slave_endpoint_transfer_request;
            transfer_request->ux_slave_transfer_request_data_pointer = g_ncm.xmit_buf;
            _ux_device_stack_transfer_request(transfer_request, block_length, block_length);

            // Zero-Length-Packet, falls die NTB-Laenge ein exaktes Vielfaches der Bulk-
            // Paketgroesse ist -- sonst interpretiert der Host das letzte volle Paket als "es
            // folgt noch mehr" und wartet auf weitere Daten (Standard-USB-Bulk-Terminierung).
            if ((block_length % NCM_EP_SIZE) == 0) {
                transfer_request->ux_slave_transfer_request_data_pointer = g_ncm.xmit_buf;
                _ux_device_stack_transfer_request(transfer_request, 0, 0);
            }
        }
    }
}

//----------------------------------------------------------------------------------------------
// --- Empfangspfad (Host -> NetX Duo, Bulk-OUT) ---------------------------------------------------
//----------------------------------------------------------------------------------------------

// Prueft ein empfangenes NTB auf Plausibilitaet (Signaturen/Laengenfelder innerhalb des
// tatsaechlich empfangenen Puffers) und uebergibt jedes enthaltene Datagramm einzeln an NetX Duo
// -- dieselben Pruefungen wie TinyUSBs recv_validate_datagram(), nur ohne dessen
// wNextNdpIndex!=0-Sonderfallbehandlung (dieses Projekt erzeugt selbst nie mehr als eine NDP je
// NTB, und der einzige Host-Gegenpart, der mehrere NDPs je NTB senden koennte, waere ein zweites
// NCM-faehiges Betriebssystem -- Windows tut das laut Spezifikation nicht).
static UINT ncm_validate_and_dispatch(UINT actual_length) {
    if (g_ncm.packet_pool == UX_NULL) {
        return UX_ERROR;  // noch nicht aufgeloest (s. ncm_bulkout_thread_entry) -- NTB verwerfen
    }
    if (actual_length < sizeof(ncm_nth16_t) + sizeof(ncm_ndp16_t) + 2u * sizeof(ncm_ndp16_datagram_t)) {
        return UX_ERROR;
    }

    ncm_nth16_t const *nth = (ncm_nth16_t const *)(void const *)g_ncm.recv_buf;
    if (nth->dwSignature != NCM_NTH16_SIGNATURE || nth->wHeaderLength != sizeof(ncm_nth16_t)) {
        return UX_ERROR;
    }
    if (nth->wBlockLength > actual_length || nth->wBlockLength > NCM_NTB_MAX_SIZE) {
        return UX_ERROR;
    }
    if (nth->wNdpIndex < sizeof(ncm_nth16_t) ||
        (ULONG)nth->wNdpIndex + sizeof(ncm_ndp16_t) + 2u * sizeof(ncm_ndp16_datagram_t) > actual_length) {
        return UX_ERROR;
    }

    ncm_ndp16_t const *ndp = (ncm_ndp16_t const *)(void const *)(g_ncm.recv_buf + nth->wNdpIndex);
    if (ndp->dwSignature != NCM_NDP16_SIGNATURE_NCM0) {
        return UX_ERROR;
    }
    if ((ULONG)nth->wNdpIndex + ndp->wLength > actual_length || ndp->wLength < sizeof(ncm_ndp16_t) + 2u * sizeof(ncm_ndp16_datagram_t)) {
        return UX_ERROR;
    }

    ncm_ndp16_datagram_t const *table =
        (ncm_ndp16_datagram_t const *)(void const *)(g_ncm.recv_buf + nth->wNdpIndex + sizeof(ncm_ndp16_t));
    UINT max_entries = (UINT)((ndp->wLength - sizeof(ncm_ndp16_t)) / sizeof(ncm_ndp16_datagram_t));

    for (UINT i = 0; i < max_entries; i++) {
        NX_PACKET *packet_ptr;
        UINT datagram_length = table[i].wDatagramLength;

        if (table[i].wDatagramIndex == 0 || datagram_length == 0) {
            break;  // Nullterminator erreicht
        }
        if ((ULONG)table[i].wDatagramIndex + datagram_length > actual_length) {
            return UX_ERROR;
        }
        if (datagram_length < NCM_ETHERNET_HEADER_SIZE) {
            continue;
        }

        // Frisches NX_PACKET je Datagramm -- CDC-ECMs bulkout_thread.c-Konvention: 2 Fuellbytes
        // vor dem Ethernet-Header reservieren (NX_PACKET_ALIGNMENT sorgt fuer eine 4-Byte-
        // ausgerichtete prepend_ptr-Startposition, der 14-Byte-Ethernet-Header ist aber kein
        // Vielfaches von 4 -- ohne dieses Polster laege der IP-Header nach dem Header-Abstreifen
        // auf einem falschen Offset, was zu stillem Verwerfen wegen falscher Pruefsummen fuehrt).
        if (nx_packet_allocate(g_ncm.packet_pool, &packet_ptr, NX_RECEIVE_PACKET, NX_NO_WAIT) != NX_SUCCESS) {
            continue;
        }
        packet_ptr->nx_packet_prepend_ptr += 2;
        packet_ptr->nx_packet_append_ptr += 2;

        if (nx_packet_data_append(packet_ptr, g_ncm.recv_buf + table[i].wDatagramIndex, datagram_length,
                                   g_ncm.packet_pool, NX_NO_WAIT) != NX_SUCCESS) {
            nx_packet_release(packet_ptr);
            continue;
        }

        // Direkt an NetX Duo weiterreichen -- SYNCHRON, noch in diesem Thread-Kontext.
        _ux_network_driver_packet_received(g_ncm.network_handle, packet_ptr);
    }

    return UX_SUCCESS;
}

static void ncm_bulkout_thread_entry(ULONG arg) {
    (void)arg;
    UX_SLAVE_DEVICE *device = &_ux_system_slave->ux_system_slave_device;

    for (;;) {
        while (device->ux_slave_device_state == UX_DEVICE_CONFIGURED) {
            // packet_pool wird erst hier lazy aufgeloest (nicht schon in ncm_activate()): zum
            // Zeitpunkt der Control-Interface-Aktivierung koennte network_handle zwar schon
            // stehen, aber dieses Muster (statt einer Reihenfolge-Annahme) folgt exakt CDC-ECMs
            // bulkout_thread.c.
            if (g_ncm.packet_pool == UX_NULL && g_ncm.network_handle != UX_NULL) {
                USB_NETWORK_DEVICE_TYPE *nx_device = (USB_NETWORK_DEVICE_TYPE *)g_ncm.network_handle;
                if (nx_device->ux_network_device_ip_instance != UX_NULL) {
                    g_ncm.packet_pool = nx_device->ux_network_device_ip_instance->nx_ip_default_packet_pool;
                }
            }

            if (g_ncm.data_alt_active == 0 || g_ncm.ep_out == UX_NULL) {
                tx_thread_sleep(10);
                continue;
            }

            UX_SLAVE_TRANSFER *transfer_request = &g_ncm.ep_out->ux_slave_endpoint_transfer_request;
            transfer_request->ux_slave_transfer_request_data_pointer = g_ncm.recv_buf;
            if (_ux_device_stack_transfer_request(transfer_request, NCM_NTB_MAX_SIZE, NCM_NTB_MAX_SIZE) == UX_SUCCESS) {
                ncm_validate_and_dispatch(transfer_request->ux_slave_transfer_request_actual_length);
            }
            // Fehlerfall (Timeout/Bus-Reset/ungueltiges NTB): Puffer wird beim naechsten
            // Schleifendurchlauf einfach erneut fuer eine neue Empfangstransaktion verwendet --
            // kein Sonderbehandlungscode noetig (analog TinyUSBs "verify failed: ignore NTB").
        }

        // Nicht (mehr) konfiguriert -- kurz schlafen statt die Schleife leerzudrehen.
        tx_thread_sleep(10);
    }
}

//----------------------------------------------------------------------------------------------
// --- Steuertransfers (Klassen-spezifische NCM-Requests) -----------------------------------------
//----------------------------------------------------------------------------------------------

static UINT ncm_control_request(UX_SLAVE_CLASS_COMMAND *command) {
    (void)command;
    // Anders als bei einigen Feldern des UX_SLAVE_CLASS_COMMAND (z.B. ux_slave_class_command_
    // interface) gibt es KEIN "ux_slave_class_command_transfer"-Feld -- der Transfer-Request
    // fuer den Control-Endpunkt haengt stattdessen am globalen Geraete-Objekt, exakt wie in
    // ux_device_class_cdc_ecm_control_request.c ("Get the pointer to the transfer request
    // associated with the control endpoint").
    UX_SLAVE_DEVICE *device = &_ux_system_slave->ux_system_slave_device;
    UX_SLAVE_TRANSFER *transfer_request = &device->ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;
    UCHAR *setup = transfer_request->ux_slave_transfer_request_setup;
    UINT bmRequestType = setup[0];
    UINT bRequest = setup[1];
    UINT wLength = (UINT)(setup[6] | (setup[7] << 8));

    // Nur Klassen-spezifische Requests an unser Control-Interface behandeln (bmRequestType
    // Bit 5-6 = 01 = Class) -- Standard-Requests (GET/SET_INTERFACE fuer den Alt-Setting-
    // Wechsel) werden bereits generisch von USBX' Kern erledigt, s. Klassenkommentar in
    // usbd_ncm.h.
    if ((bmRequestType & 0x60u) != 0x20u) {
        return UX_ERROR;
    }

    switch (bRequest) {
        case NCM_GET_NTB_PARAMETERS: {
            static ncm_ntb_parameters_t const params = {
                sizeof(ncm_ntb_parameters_t),
                0x01u,  // bmNtbFormatsSupported: nur NTB16
                NCM_NTB_MAX_SIZE,
                1u, 0u, 4u, 0u,
                NCM_NTB_MAX_SIZE,
                1u, 0u, 4u,
                NCM_XMIT_MAX_DATAGRAMS,
            };
            transfer_request->ux_slave_transfer_request_data_pointer = (UCHAR *)(void const *)&params;
            _ux_device_stack_transfer_request(transfer_request, sizeof(params), wLength);
            return UX_SUCCESS;
        }

        case NCM_SET_ETHERNET_PACKET_FILTER:
            // Kein eigener Multicast-/Promiscuous-Filter -- der virtuelle Link empfaengt ohnehin
            // alles, was der Host sendet. Reine Status-ACK-Quittierung.
            _ux_device_stack_transfer_request(transfer_request, 0, 0);
            return UX_SUCCESS;

        case NCM_GET_NTB_INPUT_SIZE: {
            static uint32_t const dw_ntb_in_max_size = NCM_NTB_MAX_SIZE;
            transfer_request->ux_slave_transfer_request_data_pointer = (UCHAR *)(void const *)&dw_ntb_in_max_size;
            _ux_device_stack_transfer_request(transfer_request, sizeof(dw_ntb_in_max_size), wLength);
            return UX_SUCCESS;
        }

        case NCM_SET_NTB_INPUT_SIZE:
            // bmCapabilities=0 im Deskriptor (kein NTB_INPUT_SIZE-Capability-Bit) -- der Host
            // sollte dies laut Spezifikation gar nicht erst senden. Falls doch: Datenphase
            // schlucken (Wert selbst bleibt fest bei NCM_NTB_MAX_SIZE) und mit STATUS-ACK
            // quittieren statt zu stallen -- manche Hosts fragen diesen Request unabhaengig von
            // der beworbenen Capability ab.
            _ux_device_stack_transfer_request(transfer_request, 0, wLength);
            return UX_SUCCESS;

        default:
            return UX_ERROR;
    }
}

//----------------------------------------------------------------------------------------------
// --- Interface-Aktivierung/-Deaktivierung/-Wechsel -------------------------------------------------
//----------------------------------------------------------------------------------------------

// Wird von USBX' Kern GENAU EINMAL PRO INTERFACE bei SET_CONFIGURATION aufgerufen (fuer das
// Control-Interface UND das Daten-Interface, letzteres dabei IMMER mit Alt-Setting 0 -- der
// USB-Spezifikation nach ist das der Default beim initialen Mounten). Der Wechsel auf Alt-Setting
// 1 passiert NICHT hier, sondern in ncm_change() (s. Klassenkommentar in usbd_ncm.h).
static UINT ncm_activate(UX_SLAVE_CLASS_COMMAND *command) {
    UX_SLAVE_INTERFACE *interface_ptr = (UX_SLAVE_INTERFACE *)command->ux_slave_class_command_interface;

    if (command->ux_slave_class_command_class == NCM_CLASS_COMMUNICATION_CONTROL) {
        UX_SLAVE_ENDPOINT *endpoint = interface_ptr->ux_slave_interface_first_endpoint;
        while (endpoint != UX_NULL) {
            if ((endpoint->ux_slave_endpoint_descriptor.bmAttributes & UX_MASK_ENDPOINT_TYPE) == UX_INTERRUPT_ENDPOINT) {
                g_ncm.ep_notif = endpoint;
            }
            endpoint = endpoint->ux_slave_endpoint_next_endpoint;
        }
        return UX_SUCCESS;
    }

    // Daten-Interface, Alt-Setting 0 (bNumEndpoints==0 laut Deskriptor) -- analog zu CDC-ECMs
    // activate.c: Endpunkte zuruecksetzen, physische MAC-Adresse an den generischen
    // ux_network_driver melden. Der eigentliche "Link ist da"-Uebergang (Bulk-Endpunkte finden,
    // _ux_network_driver_link_up()) passiert erst in ncm_change(), sobald der Host auf
    // Alt-Setting 1 umschaltet.
    g_ncm.ep_in = UX_NULL;
    g_ncm.ep_out = UX_NULL;
    g_ncm.data_alt_active = 0;

    ULONG physical_address_msw = ((ULONG)g_ncm.mac[0] << 8) | g_ncm.mac[1];
    ULONG physical_address_lsw = ((ULONG)g_ncm.mac[2] << 24) | ((ULONG)g_ncm.mac[3] << 16) |
                                  ((ULONG)g_ncm.mac[4] << 8) | g_ncm.mac[5];
    _ux_network_driver_activate((VOID *)&g_ncm, ncm_network_write, &g_ncm.network_handle,
                                 physical_address_msw, physical_address_lsw);

    return UX_SUCCESS;
}

// UX_SLAVE_CLASS_COMMAND_CHANGE -- vom USBX-Kern bei jedem SET_INTERFACE auf das Daten-Interface
// aufgerufen (s. ux_device_stack_alternate_setting_set.c), also der eigentliche Alt-Setting-0<->1-
// Uebergang. Struktur 1:1 CDC-ECMs change.c nachgebildet.
static UINT ncm_change(UX_SLAVE_CLASS_COMMAND *command) {
    UX_SLAVE_INTERFACE *interface_ptr = (UX_SLAVE_INTERFACE *)command->ux_slave_class_command_interface;

    if (interface_ptr->ux_slave_interface_descriptor.bAlternateSetting != 0) {
        // Alt-Setting 1 -- Bulk-Endpunkte suchen.
        UX_SLAVE_ENDPOINT *endpoint = interface_ptr->ux_slave_interface_first_endpoint;
        g_ncm.ep_in = UX_NULL;
        g_ncm.ep_out = UX_NULL;
        while (endpoint != UX_NULL) {
            if ((endpoint->ux_slave_endpoint_descriptor.bEndpointAddress & UX_ENDPOINT_DIRECTION) == UX_ENDPOINT_IN) {
                g_ncm.ep_in = endpoint;
            } else {
                g_ncm.ep_out = endpoint;
            }
            endpoint = endpoint->ux_slave_endpoint_next_endpoint;
        }

        if (g_ncm.ep_in == UX_NULL || g_ncm.ep_out == UX_NULL) {
            return UX_ERROR;
        }

        g_ncm.data_alt_active = 1;
        _ux_network_driver_link_up(g_ncm.network_handle);
        tx_semaphore_put(&g_ncm.notif_kick_sem);
        log_info("USBX: CDC-NCM Daten-Interface aktiv (Alt-Setting 1)");
    } else {
        // Ruecksprung auf Alt-Setting 0 -- Datenverkehr sperren.
        g_ncm.data_alt_active = 0;
        _ux_network_driver_link_down(g_ncm.network_handle);
        log_info("USBX: CDC-NCM Daten-Interface inaktiv (Alt-Setting 0)");
    }

    return UX_SUCCESS;
}

static UINT ncm_deactivate(UX_SLAVE_CLASS_COMMAND *command) {
    (void)command;
    g_ncm.data_alt_active = 0;
    g_ncm.ep_notif = UX_NULL;
    g_ncm.ep_in = UX_NULL;
    g_ncm.ep_out = UX_NULL;
    if (g_ncm.network_handle != UX_NULL) {
        _ux_network_driver_deactivate((VOID *)&g_ncm, g_ncm.network_handle);
        g_ncm.network_handle = UX_NULL;
    }
    g_ncm.packet_pool = UX_NULL;
    log_info("USBX: CDC-NCM deaktiviert");
    return UX_SUCCESS;
}

//----------------------------------------------------------------------------------------------
// --- Klassen-Einsprungpunkt (ux_device_stack_class_register()) ----------------------------------
//----------------------------------------------------------------------------------------------

static UINT ncm_entry(UX_SLAVE_CLASS_COMMAND *command) {
    switch (command->ux_slave_class_command_request) {
        case UX_SLAVE_CLASS_COMMAND_QUERY:
            if (command->ux_slave_class_command_class == NCM_CLASS_COMMUNICATION_CONTROL ||
                command->ux_slave_class_command_class == NCM_CLASS_COMMUNICATION_DATA) {
                return UX_SUCCESS;
            }
            return UX_NO_CLASS_MATCH;

        case UX_SLAVE_CLASS_COMMAND_ACTIVATE:
            return ncm_activate(command);

        case UX_SLAVE_CLASS_COMMAND_CHANGE:
            return ncm_change(command);

        case UX_SLAVE_CLASS_COMMAND_DEACTIVATE:
            return ncm_deactivate(command);

        case UX_SLAVE_CLASS_COMMAND_REQUEST:
            return ncm_control_request(command);

        default:
            return UX_SUCCESS;
    }
}

// _ux_system_slave_class_ncm_name/entry: anders als bei den ST-eigenen Klassen (deren Name als
// globales Symbol schon in ux_system.h/ux_device_stack_initialize.c deklariert+definiert ist)
// gibt es fuer diese komplett projekteigene Klasse kein vorbelegtes Namenssymbol -- daher hier
// selbst definiert.
static UCHAR const g_ncm_class_name[] = "ux_slave_class_ncm";

UINT usbd_ncm_class_register(ULONG itf_control, uint8_t const net_mac[6]) {
    memset(&g_ncm, 0, sizeof(g_ncm));
    g_ncm.itf_control = itf_control;
    g_ncm.itf_data = itf_control + 1u;
    memcpy(g_ncm.mac, net_mac, sizeof(g_ncm.mac));

    if (tx_mutex_create(&g_ncm.xmit_mutex, "NCM Xmit Queue", TX_NO_INHERIT) != TX_SUCCESS) {
        return UX_ERROR;
    }
    if (tx_semaphore_create(&g_ncm.xmit_ready_sem, "NCM Xmit Ready", 0) != TX_SUCCESS) {
        return UX_ERROR;
    }
    if (tx_semaphore_create(&g_ncm.notif_kick_sem, "NCM Notif Kick", 0) != TX_SUCCESS) {
        return UX_ERROR;
    }
    if (tx_thread_create(&g_ncm.notif_thread, "NCM Notif", ncm_notif_thread_entry, 0,
                          g_ncm.notif_thread_stack, NCM_THREAD_STACK_SIZE,
                          NCM_THREAD_PRIORITY, NCM_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
        return UX_ERROR;
    }
    if (tx_thread_create(&g_ncm.bulkin_thread, "NCM Bulk IN", ncm_bulkin_thread_entry, 0,
                          g_ncm.bulkin_thread_stack, NCM_THREAD_STACK_SIZE,
                          NCM_THREAD_PRIORITY, NCM_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
        return UX_ERROR;
    }
    if (tx_thread_create(&g_ncm.bulkout_thread, "NCM Bulk OUT", ncm_bulkout_thread_entry, 0,
                          g_ncm.bulkout_thread_stack, NCM_THREAD_STACK_SIZE,
                          NCM_THREAD_PRIORITY, NCM_THREAD_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS) {
        return UX_ERROR;
    }

    return ux_device_stack_class_register((UCHAR *)(void const *)g_ncm_class_name, ncm_entry, 1, itf_control, UX_NULL);
}
