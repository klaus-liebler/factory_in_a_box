#include "usbd_device.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ux_api.h"
#include "ux_device_stack.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_device_class_cdc_ecm.h"
#include "ux_network_driver.h"
#include "ux_dcd_stm32.h"
#include "main.h"
#include "log.h"
#include "tx_api.h"

// hpcd_USB_DRD_FS: von CubeMX-generiertem Code in main.c angelegt (PCD_HandleTypeDef
// hpcd_USB_DRD_FS;, s. dortiges MX_USB_PCD_Init()) -- main.h deklariert sie NICHT als extern
// (anders als z.B. huart3), daher hier lokal nachgeholt (identisches Vorgehen wie in ST's
// eigenem app_usbx_device.c-Referenzcode).
extern PCD_HandleTypeDef hpcd_USB_DRD_FS;

// Lokales Gegenstueck zu Core/Inc/common_macros.hh's XASSERT -- jene Datei ist eine .hh mit
// C++-spezifischem _C()-Makro (const_cast), daher aus dieser bewusst reinen C-Uebersetzungs-
// einheit (s. Klassenkommentar in usbd_device.h) nicht einbindbar. NX_SUCCESS/UX_SUCCESS sind
// beide 0 (durchgaengige ThreadX-Familien-Konvention), die Pruefung ist inhaltlich identisch.
#define UXASSERT(status_expr, message_on_fail) \
    do { \
        UINT assure_status_ = (status_expr); \
        if (assure_status_ != UX_SUCCESS) { \
            log_error("Error %s: %s:%d, status: 0x%x", message_on_fail, __FILE__, __LINE__, assure_status_); \
            Error_Handler(); \
        } \
    } while (0)

// --- USB VID/PID -------------------------------------------------------------------------
// Platzhalter (pid.codes-Wurzel-VID fuer nicht kommerziell vertriebene/private Projekte) --
// vor einer Weitergabe ausserhalb des eigenen Werks/Netzwerks durch eine eigene PID unter
// https://pid.codes ersetzen. Identische Werte wie in der TinyUSB-Fassung (kein Grund, den
// Host beim Umstieg auf USBX ein "neues" Geraet erkennen zu lassen).
#define USB_VID 0x1209u
#define USB_PID 0x0001u
#define USB_BCD_DEVICE 0x0100u  // Geraete-Revision 1.0

// --- String-Deskriptor-Indizes -------------------------------------------------------------
enum {
    STRIDX_LANGID = 0,
    STRIDX_MANUFACTURER,
    STRIDX_PRODUCT,
    STRIDX_SERIAL,
    STRIDX_CDC0_DEBUG,
    STRIDX_CDC1_MODBUS,
    STRIDX_ECM,
    STRIDX_ECM_MAC,
};

static char const* const STRING_MANUFACTURER = "Klaus Liebler";
static char const* const STRING_PRODUCT = "FactoryControl Unit";
static char const* const STRING_CDC0_DEBUG = "FactoryControl Debug";
static char const* const STRING_CDC1_MODBUS = "FactoryControl Modbus";
static char const* const STRING_ECM = "FactoryControl Network";

// Vom App-Chip-UID gebildet (s. usbd_device_setup()) -- 24 Hex-Ziffern (3x uint32_t) +
// Nullterminator.
static char g_serial_string[25] = "000000000000000000000000";

// MAC-Adress-String der virtuellen CDC-ECM-NIC -- CDC1.2 verlangt 12 GROSSGESCHRIEBENE
// Hex-Ziffern ohne Trennzeichen (Format "AABBCCDDEEFF"), identisch zum bisherigen NCM-Format.
// Einzige Stelle, an der die lokale MAC tatsaechlich auf dem USB-Draht landet (s.
// USBD_DESC_SUBTYPE_ETHERNET-Funktionsdeskriptor unten, iMACAddress-Feld) -- die davon
// unabhaengige "remote_node_id" (s. cdc_ecm_activate()) geht NIE ueber USB, sondern ist rein
// intern (s. dortiger Kommentar).
static char g_ecm_mac_string[13] = "000000000000";

// --- Interface-/Endpunkt-Layout ----------------------------------------------------------
// Unveraendert gegenueber der TinyUSB-Fassung: 2x CDC-ACM (Debug, Modbus) + 1x CDC-ECM (statt
// vormals CDC-NCM -- USBX kennt keine NCM-Geraeteklasse, s. Plan/Commit-Nachricht). Endpunkt-
// Nummern von Hand vergeben, damit alle drei Funktionen exakt in die 8 verfuegbaren
// USB_DRD_FS-Hardware-Endpunkte (EP0 + 7) passen -- EP7 bleibt frei (potenzielle spaetere
// MSC-Stufe, wie schon bei der TinyUSB-Fassung vorgesehen).
enum {
    ITF_CDC0_DEBUG_CONTROL = 0,
    ITF_CDC0_DEBUG_DATA,
    ITF_CDC1_MODBUS_CONTROL,
    ITF_CDC1_MODBUS_DATA,
    ITF_ECM_CONTROL,
    ITF_ECM_DATA,
    ITF_COUNT,
};

#define EP_CDC0_NOTIF 0x81u
#define EP_CDC0_OUT 0x02u
#define EP_CDC0_IN 0x82u
#define EP_CDC0_NOTIF_SIZE 8u
#define EP_CDC0_DATA_SIZE 64u

#define EP_CDC1_NOTIF 0x83u
#define EP_CDC1_OUT 0x04u
#define EP_CDC1_IN 0x84u
#define EP_CDC1_NOTIF_SIZE 8u
#define EP_CDC1_DATA_SIZE 64u

#define EP_ECM_NOTIF 0x85u
#define EP_ECM_OUT 0x06u
#define EP_ECM_IN 0x86u
// UX_DEVICE_CLASS_CDC_ECM_INTERRUPT_RESPONSE_LENGTH (ux_device_class_cdc_ecm.h): die CDC-ECM-
// Klasse verschickt NETWORK_CONNECTION-Benachrichtigungen ohne zusaetzliche Nutzdaten (nur der
// 8-Byte-Notification-Header) -- kleiner als das bisherige NCM-Notification-Endpunkt-Budget
// (16 Byte), spart aber nichts an physischen Endpunkten (weiterhin EP5).
#define EP_ECM_NOTIF_SIZE 8u
#define EP_ECM_DATA_SIZE 64u

// --- USB-Deskriptor-Framework (ein einziges zusammenhaengendes Byte-Array aus Geraete- +
// Konfigurationsdeskriptor, s. _ux_device_stack_initialize() -- anders als TinyUSBs getrennte
// tud_descriptor_device_cb()/tud_descriptor_configuration_cb()-Rueckrufe uebergibt USBX beides
// als EIN Rohdaten-Array). bDeviceClass=0xEF/bDeviceSubClass=0x02/bDeviceProtocol=0x01
// (Interface Association Descriptor) weiterhin noetig, damit Windows' usbccgp.sys die drei
// Funktionen anhand der IADs korrekt gruppiert statt nur die erste Interface-Beschreibung zu
// verwenden -- identisch zur TinyUSB-Fassung.
#define DEVICE_DESC_LEN 18u

#define IAD_LEN 8u
#define ITF_LEN 9u
#define CDC_HEADER_FD_LEN 5u
#define CDC_CALL_MGMT_FD_LEN 5u
#define CDC_ACM_FD_LEN 4u
#define CDC_UNION_FD_LEN 5u
#define EP_DESC_LEN 7u
#define CDC_ECM_FD_LEN 13u

// Ein CDC-ACM-Funktionsblock: IAD + Control-Interface (Header/CallMgmt/ACM/Union-FD +
// Notification-EP) + Data-Interface (2x Bulk-EP).
#define CDC_ACM_FUNCTION_LEN (IAD_LEN + ITF_LEN + CDC_HEADER_FD_LEN + CDC_CALL_MGMT_FD_LEN + \
                              CDC_ACM_FD_LEN + CDC_UNION_FD_LEN + EP_DESC_LEN + \
                              ITF_LEN + EP_DESC_LEN + EP_DESC_LEN)
// Ein CDC-ECM-Funktionsblock: IAD + Control-Interface (Header/Ethernet-FD/Union-FD +
// Notification-EP) + Data-Interface (2x Bulk-EP) -- kein Call-Management-FD (nur fuer ACM
// vorgeschrieben, s. CDC1.2 Tabelle 5-3/5-4).
#define CDC_ECM_FUNCTION_LEN (IAD_LEN + ITF_LEN + CDC_HEADER_FD_LEN + CDC_ECM_FD_LEN + \
                              CDC_UNION_FD_LEN + EP_DESC_LEN + \
                              ITF_LEN + EP_DESC_LEN + EP_DESC_LEN)

#define CONFIG_DESC_LEN 9u
#define CONFIG_TOTAL_LEN (CONFIG_DESC_LEN + 2u * CDC_ACM_FUNCTION_LEN + CDC_ECM_FUNCTION_LEN)
#define DEVICE_FRAMEWORK_LEN (DEVICE_DESC_LEN + CONFIG_TOTAL_LEN)

// CDC1.2-Deskriptor-Subtypen (bDescriptorSubtype im CS_INTERFACE-Deskriptor, bDescriptorType
// 0x24) -- dieselben numerischen Werte wie im TinyUSB-Stack intern verwendet, hier von Hand
// aufgefuehrt, da USBX (anders als TinyUSB) keine TUD_CDC_DESCRIPTOR()-aehnlichen Hilfsmakros
// mitbringt (s. Plan: ST's eigener Deskriptor-Baukasten in ux_device_descriptors.c ist
// Anwendungs-/Demo-Code, kein USBX-Middleware-Bestandteil, und fuer unser fest bekanntes
// 3-Funktionen-Composite unverhaeltnismaessig, s. Commit-Nachricht).
#define CS_INTERFACE 0x24u
#define CDC_FD_HEADER 0x00u
#define CDC_FD_CALL_MANAGEMENT 0x01u
#define CDC_FD_ACM 0x02u
#define CDC_FD_UNION 0x06u
#define CDC_FD_ETHERNET 0x0Fu

#define U16LE(v) (uint8_t)((v) & 0xFFu), (uint8_t)(((v) >> 8) & 0xFFu)

static uint8_t const g_device_framework[DEVICE_FRAMEWORK_LEN] = {
    // --- Geraetedeskriptor (18 Byte) ---
    DEVICE_DESC_LEN, 0x01,              // bLength, bDescriptorType=DEVICE
    U16LE(0x0200),                      // bcdUSB
    0xEFu, 0x02u, 0x01u,                // bDeviceClass/SubClass/Protocol (Misc/IAD)
    64u,                                 // bMaxPacketSize0
    U16LE(USB_VID), U16LE(USB_PID), U16LE(USB_BCD_DEVICE),
    STRIDX_MANUFACTURER, STRIDX_PRODUCT, STRIDX_SERIAL,
    1u,                                  // bNumConfigurations

    // --- Konfigurationsdeskriptor (9 Byte) ---
    CONFIG_DESC_LEN, 0x02,              // bLength, bDescriptorType=CONFIGURATION
    U16LE(CONFIG_TOTAL_LEN),
    ITF_COUNT,                           // bNumInterfaces
    1u, 0u,                              // bConfigurationValue, iConfiguration
    0xC0u,                               // bmAttributes (Self-Powered, Bit7 reserviert=1)
    50u,                                 // bMaxPower (100mA / 2mA-Einheiten)

    // ============ CDC-ACM 0 ("FactoryControl Debug") ============
    IAD_LEN, 0x0Bu, ITF_CDC0_DEBUG_CONTROL, 2u, 0x02u, 0x02u, 0x00u, STRIDX_CDC0_DEBUG,
    // Control-Interface
    ITF_LEN, 0x04, ITF_CDC0_DEBUG_CONTROL, 0u, 1u, 0x02u, 0x02u, 0x00u, STRIDX_CDC0_DEBUG,
    CDC_HEADER_FD_LEN, CS_INTERFACE, CDC_FD_HEADER, U16LE(0x0110),
    CDC_CALL_MGMT_FD_LEN, CS_INTERFACE, CDC_FD_CALL_MANAGEMENT, 0x00u, ITF_CDC0_DEBUG_DATA,
    CDC_ACM_FD_LEN, CS_INTERFACE, CDC_FD_ACM, 0x02u,
    CDC_UNION_FD_LEN, CS_INTERFACE, CDC_FD_UNION, ITF_CDC0_DEBUG_CONTROL, ITF_CDC0_DEBUG_DATA,
    EP_DESC_LEN, 0x05, EP_CDC0_NOTIF, 0x03u, U16LE(EP_CDC0_NOTIF_SIZE), 16u,
    // Data-Interface
    ITF_LEN, 0x04, ITF_CDC0_DEBUG_DATA, 0u, 2u, 0x0Au, 0x00u, 0x00u, 0u,
    EP_DESC_LEN, 0x05, EP_CDC0_OUT, 0x02u, U16LE(EP_CDC0_DATA_SIZE), 0u,
    EP_DESC_LEN, 0x05, EP_CDC0_IN, 0x02u, U16LE(EP_CDC0_DATA_SIZE), 0u,

    // ============ CDC-ACM 1 ("FactoryControl Modbus") ============
    IAD_LEN, 0x0Bu, ITF_CDC1_MODBUS_CONTROL, 2u, 0x02u, 0x02u, 0x00u, STRIDX_CDC1_MODBUS,
    ITF_LEN, 0x04, ITF_CDC1_MODBUS_CONTROL, 0u, 1u, 0x02u, 0x02u, 0x00u, STRIDX_CDC1_MODBUS,
    CDC_HEADER_FD_LEN, CS_INTERFACE, CDC_FD_HEADER, U16LE(0x0110),
    CDC_CALL_MGMT_FD_LEN, CS_INTERFACE, CDC_FD_CALL_MANAGEMENT, 0x00u, ITF_CDC1_MODBUS_DATA,
    CDC_ACM_FD_LEN, CS_INTERFACE, CDC_FD_ACM, 0x02u,
    CDC_UNION_FD_LEN, CS_INTERFACE, CDC_FD_UNION, ITF_CDC1_MODBUS_CONTROL, ITF_CDC1_MODBUS_DATA,
    EP_DESC_LEN, 0x05, EP_CDC1_NOTIF, 0x03u, U16LE(EP_CDC1_NOTIF_SIZE), 16u,
    ITF_LEN, 0x04, ITF_CDC1_MODBUS_DATA, 0u, 2u, 0x0Au, 0x00u, 0x00u, 0u,
    EP_DESC_LEN, 0x05, EP_CDC1_OUT, 0x02u, U16LE(EP_CDC1_DATA_SIZE), 0u,
    EP_DESC_LEN, 0x05, EP_CDC1_IN, 0x02u, U16LE(EP_CDC1_DATA_SIZE), 0u,

    // ============ CDC-ECM ("FactoryControl Network") ============
    IAD_LEN, 0x0Bu, ITF_ECM_CONTROL, 2u, 0x02u, 0x06u, 0x00u, STRIDX_ECM,
    ITF_LEN, 0x04, ITF_ECM_CONTROL, 0u, 1u, 0x02u, 0x06u, 0x00u, STRIDX_ECM,
    CDC_HEADER_FD_LEN, CS_INTERFACE, CDC_FD_HEADER, U16LE(0x0110),
    // Ethernet Networking Functional Descriptor (CDC1.2 Tabelle 5-4): iMACAddress,
    // bmEthernetStatistics (4 Byte, keine Statistiken unterstuetzt), wMaxSegmentSize
    // (1514 = Standard-Ethernet-MTU+Header), wNumberMCFilters (keine Multicast-Filter),
    // bNumberPowerFilters (kein Wake-on-LAN-Pattern-Filter).
    CDC_ECM_FD_LEN, CS_INTERFACE, CDC_FD_ETHERNET, STRIDX_ECM_MAC,
    0x00u, 0x00u, 0x00u, 0x00u, U16LE(1514u), U16LE(0u), 0x00u,
    CDC_UNION_FD_LEN, CS_INTERFACE, CDC_FD_UNION, ITF_ECM_CONTROL, ITF_ECM_DATA,
    EP_DESC_LEN, 0x05, EP_ECM_NOTIF, 0x03u, U16LE(EP_ECM_NOTIF_SIZE), 16u,
    ITF_LEN, 0x04, ITF_ECM_DATA, 0u, 2u, 0x0Au, 0x00u, 0x00u, 0u,
    EP_DESC_LEN, 0x05, EP_ECM_OUT, 0x02u, U16LE(EP_ECM_DATA_SIZE), 0u,
    EP_DESC_LEN, 0x05, EP_ECM_IN, 0x02u, U16LE(EP_ECM_DATA_SIZE), 0u,
};

// --- String-/Sprach-Framework -------------------------------------------------------------
// Format laut _ux_device_stack_descriptor_send()'s String-Framework-Parser (libs/ST/usbx/
// common/core/src/ux_device_stack_descriptor_send.c): je Eintrag 4 Header-Byte (LANGID
// Low/High, String-Index, ASCII-Laenge) gefolgt von den rohen ASCII-Zeichen selbst (USBX
// wandelt erst beim tatsaechlichen GET_DESCRIPTOR-Request nach UTF-16LE um) -- kein
// vorgefertigtes Makro dafuer in USBX, daher der kleine Hilfsbaustein append_string_entry()
// unten. Dynamisch befuellt (Seriennummer/MAC-Adresse stehen erst zur Laufzeit fest), deshalb
// hier nur Puffer + Laenge statt eines static-const-Arrays wie beim restlichen Framework.
#define USBD_LANGID_STRING 0x0409u  // en-US
#define STRING_FRAMEWORK_MAX_LEN 200u

static uint8_t g_string_framework[STRING_FRAMEWORK_MAX_LEN];
static ULONG g_string_framework_length;

static uint8_t g_language_id_framework[2] = {U16LE(USBD_LANGID_STRING)};

static void append_string_entry(uint8_t index, char const* str) {
    size_t len = strlen(str);
    // Kein XASSERT hier (reines C, kein log.h-XASSERT-Makro fuer diese Datei uebernommen) --
    // ein Ueberlauf waere ein Programmierfehler (zu lange String-Konstante oben), kein
    // Laufzeitfehler; ein stiller Abbruch der Eintragserstellung ist das sicherste Verhalten.
    if (g_string_framework_length + 4u + len > STRING_FRAMEWORK_MAX_LEN) {
        log_error("USB string framework: Puffer zu klein fuer Index %u", (unsigned)index);
        return;
    }
    uint8_t* p = &g_string_framework[g_string_framework_length];
    p[0] = (uint8_t)(USBD_LANGID_STRING & 0xFFu);
    p[1] = (uint8_t)(USBD_LANGID_STRING >> 8);
    p[2] = index;
    p[3] = (uint8_t)len;
    memcpy(p + 4, str, len);
    g_string_framework_length += 4u + (ULONG)len;
}

// --- CDC-ACM-Instanzen (Debug/Modbus) -----------------------------------------------------
// Von den jeweiligen Activate-Callbacks gesetzt (s. cdc_debug_activate()/cdc_modbus_activate()
// unten) -- NULL, solange der Host die Schnittstelle nicht konfiguriert hat (analog zu
// TinyUSBs tud_mounted()-Gate in syscalls.c bzw. tud_cdc_n_connected()).
static UX_SLAVE_CLASS_CDC_ACM* volatile g_cdc_debug_instance;
static UX_SLAVE_CLASS_CDC_ACM* volatile g_cdc_modbus_instance;

// --- Debug-CDC: nichtblockierendes Senden -------------------------------------------------
// UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_START (s. cdc_debug_activate()) aktiviert
// ux_device_class_cdc_acm_write_with_callback() -- echtes Fire-and-Forget wie TinyUSBs
// tud_cdc_n_write(), Abschluss laeuft ueber den klasseneigenen bulkin_thread. Debug-CDC liest
// NIE (nur Log-Spiegelung, s. syscalls.c), daher unproblematisch, dass
// TRANSMISSION_START/write_with_callback() den blockierenden ux_device_class_cdc_acm_read()-Pfad
// fuer diese Instanz sperrt (s. dortige Pruefung auf ux_slave_class_cdc_acm_transmission_status
// in ux_device_class_cdc_acm_read.c) -- fuer Modbus-CDC (braucht BEIDE Richtungen) unten deshalb
// bewusst ein anderer Ansatz.
#define CDC_DEBUG_TX_BUF_SIZE 256u
static uint8_t g_cdc_debug_tx_buf[CDC_DEBUG_TX_BUF_SIZE];
static volatile UINT g_cdc_debug_tx_busy;

static UINT cdc_debug_write_complete(UX_SLAVE_CLASS_CDC_ACM* cdc_acm, UINT status, ULONG length) {
    (void)cdc_acm;
    (void)status;
    (void)length;
    g_cdc_debug_tx_busy = UX_FALSE;
    return UX_SUCCESS;
}

static VOID cdc_debug_activate(VOID* instance) {
    g_cdc_debug_instance = (UX_SLAVE_CLASS_CDC_ACM*)instance;
    g_cdc_debug_tx_busy = UX_FALSE;
    UX_SLAVE_CLASS_CDC_ACM_CALLBACK_PARAMETER callback_parameter = {0};
    callback_parameter.ux_device_class_cdc_acm_parameter_write_callback = cdc_debug_write_complete;
    ux_device_class_cdc_acm_ioctl(g_cdc_debug_instance, UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_START,
                                   &callback_parameter);
}

static VOID cdc_debug_deactivate(VOID* instance) {
    (void)instance;
    g_cdc_debug_instance = NULL;
}

// Von syscalls.c's _write()-Spiegelung aufgerufen (s. usbd_device.h) -- best-effort/nicht
// blockierend: verwirft, solange eine vorherige Uebertragung noch nicht abgeschlossen ist
// (analog zu TinyUSBs vollem internen Ringpuffer) oder kein Host verbunden ist.
uint32_t usbd_debug_cdc_write(uint8_t const* buffer, uint32_t len) {
    if (g_cdc_debug_instance == NULL || g_cdc_debug_tx_busy) {
        return 0;
    }
    if (len > CDC_DEBUG_TX_BUF_SIZE) {
        len = CDC_DEBUG_TX_BUF_SIZE;
    }
    memcpy(g_cdc_debug_tx_buf, buffer, len);
    g_cdc_debug_tx_busy = UX_TRUE;
    if (ux_device_class_cdc_acm_write_with_callback(g_cdc_debug_instance, g_cdc_debug_tx_buf, len) != UX_SUCCESS) {
        g_cdc_debug_tx_busy = UX_FALSE;
        return 0;
    }
    return len;
}

// --- Modbus-CDC: RX (blockierender Lese-Thread + Ringpuffer) -----------------------------
// ux_device_class_cdc_acm_read() blockiert (wartet bis zu UX_NON_CONTROL_TRANSFER_TIMEOUT auf
// tatsaechliche OUT-Daten) -- fuer ModbusRtuServer::Poll()'s nichtblockierenden
// available()/read()-Vertrag (s. usbd_device.h) deshalb NICHT direkt nutzbar. Ein dedizierter
// Thread ruft die blockierende Funktion stattdessen selbst in einer Endlosschleife auf und
// puffert das Ergebnis in einem einfachen Single-Producer/Single-Consumer-Ringpuffer (Thread
// schreibt nur den Kopf-, Poll() nur den Schwanzindex -- auf Cortex-M sind 32-Bit-Zugriffe auf
// natuerlich ausgerichtete ULONGs atomar, ein Mutex ist daher nicht noetig, s. aehnliches Muster
// in usb_cdc_ecm_driver.c/vormals usb_ncm_driver.c).
#define MODBUS_RX_RING_SIZE 256u
static uint8_t g_modbus_rx_ring[MODBUS_RX_RING_SIZE];
static volatile uint32_t g_modbus_rx_head;
static volatile uint32_t g_modbus_rx_tail;

#define MODBUS_THREAD_STACK_SIZE 1024u
static TX_THREAD g_modbus_rx_thread;
static uint8_t g_modbus_rx_thread_stack[MODBUS_THREAD_STACK_SIZE];
// Priorisierung wie das umgebende usbd_device_thread (s. App::USBD_DEVICE_THREAD_PRIORITY in
// app.hh -- reaktionsschnell noetig fuer USB-Timing, aber unterhalb von Modbus-TCP) --
// hartkodiert statt aus C++ importiert, da diese Datei bewusst reines C bleibt (s.
// Klassenkommentar in usbd_device.h).
#define MODBUS_CDC_THREAD_PRIORITY 6u

static void modbus_rx_thread_entry(ULONG arg) {
    (void)arg;
    uint8_t chunk[64];
    for (;;) {
        UX_SLAVE_CLASS_CDC_ACM* instance = g_cdc_modbus_instance;
        if (instance == NULL) {
            // Kein Host verbunden -- kurz schlafen statt die Schleife leerzudrehen (analog zum
            // VSENSE-Polling in usbd_device_loop()).
            tx_thread_sleep(10);
            continue;
        }

        ULONG actual_length = 0;
        UINT status = ux_device_class_cdc_acm_read(instance, chunk, sizeof(chunk), &actual_length);
        if (status != UX_SUCCESS || actual_length == 0) {
            // Timeout, Deaktivierung waehrend des Wartens, o.ae. -- Schleife faengt sich von
            // selbst wieder (g_cdc_modbus_instance wird beim naechsten Durchlauf neu gelesen).
            continue;
        }

        for (ULONG i = 0; i < actual_length; i++) {
            uint32_t next_head = (g_modbus_rx_head + 1u) % MODBUS_RX_RING_SIZE;
            if (next_head == g_modbus_rx_tail) {
                // Ringpuffer voll -- Rest dieses Pakets verwerfen (wie TinyUSBs internes
                // RX-FIFO bei Ueberlauf; ModbusRtuServer erkennt einen unvollstaendigen Frame
                // ohnehin per CRC und verwirft ihn selbst).
                break;
            }
            g_modbus_rx_ring[g_modbus_rx_head] = chunk[i];
            g_modbus_rx_head = next_head;
        }
    }
}

uint32_t usbd_cdc_modbus_available(void) {
    uint32_t head = g_modbus_rx_head;
    uint32_t tail = g_modbus_rx_tail;
    return (head >= tail) ? (head - tail) : (MODBUS_RX_RING_SIZE - tail + head);
}

uint32_t usbd_cdc_modbus_read(uint8_t* buffer, uint32_t bufsize) {
    uint32_t n = 0;
    while (n < bufsize && g_modbus_rx_tail != g_modbus_rx_head) {
        buffer[n++] = g_modbus_rx_ring[g_modbus_rx_tail];
        g_modbus_rx_tail = (g_modbus_rx_tail + 1u) % MODBUS_RX_RING_SIZE;
    }
    return n;
}

// --- Modbus-CDC: TX (Mailbox + blockierender Schreib-Thread) -----------------------------
// write_with_callback() (s. Debug-CDC oben) scheidet fuer Modbus aus: UX_SLAVE_CLASS_CDC_ACM_
// IOCTL_TRANSMISSION_START (Voraussetzung fuer write_with_callback()) sperrt gleichzeitig den
// blockierenden Lesepfad komplett (s. Pruefung auf ux_slave_class_cdc_acm_transmission_status
// in ux_device_class_cdc_acm_read.c) -- Modbus braucht aber BEIDE Richtungen. Stattdessen: ein
// zweiter dedizierter Thread ruft die blockierende ux_device_class_cdc_acm_write() auf, eine
// Ein-Platz-"Mailbox" (kein tiefer Ringpuffer -- Modbus-RTU ist strikt Anfrage/Antwort, es ist
// nie mehr als eine Antwort gleichzeitig unterwegs) entkoppelt das vom aufrufenden
// ModbusRtuServer::Poll()-Kontext.
#define MODBUS_TX_BUF_SIZE 256u
static uint8_t g_modbus_tx_buf[MODBUS_TX_BUF_SIZE];
static volatile uint32_t g_modbus_tx_len;
static TX_SEMAPHORE g_modbus_tx_ready;
static TX_THREAD g_modbus_tx_thread;
static uint8_t g_modbus_tx_thread_stack[MODBUS_THREAD_STACK_SIZE];

static void modbus_tx_thread_entry(ULONG arg) {
    (void)arg;
    for (;;) {
        tx_semaphore_get(&g_modbus_tx_ready, TX_WAIT_FOREVER);
        UX_SLAVE_CLASS_CDC_ACM* instance = g_cdc_modbus_instance;
        if (instance != NULL) {
            ULONG actual_length = 0;
            // Blockierend (bis zu UX_NON_CONTROL_TRANSFER_TIMEOUT) -- unkritisch: laeuft in
            // einem eigenen Thread, nicht im gemeinsamen Poll-Kontext von
            // App::UsbdDevicePollHook().
            ux_device_class_cdc_acm_write(instance, g_modbus_tx_buf, g_modbus_tx_len, &actual_length);
        }
        g_modbus_tx_len = 0;
    }
}

static VOID cdc_modbus_activate(VOID* instance) {
    g_cdc_modbus_instance = (UX_SLAVE_CLASS_CDC_ACM*)instance;
}

static VOID cdc_modbus_deactivate(VOID* instance) {
    (void)instance;
    g_cdc_modbus_instance = NULL;
}

uint32_t usbd_cdc_modbus_write(const uint8_t* buffer, uint32_t len) {
    if (g_modbus_tx_len != 0) {
        // Vorherige Antwort noch nicht abgesendet -- sollte beim strikten Anfrage/Antwort-Takt
        // von Modbus-RTU nicht vorkommen; sicherheitshalber verwerfen statt zu ueberschreiben.
        return 0;
    }
    if (len > MODBUS_TX_BUF_SIZE) {
        len = MODBUS_TX_BUF_SIZE;
    }
    memcpy(g_modbus_tx_buf, buffer, len);
    g_modbus_tx_len = len;
    tx_semaphore_put(&g_modbus_tx_ready);
    return len;
}

void usbd_cdc_modbus_write_flush(void) {
    // No-Op (s. usbd_device.h) -- usbd_cdc_modbus_write() stoesst die Uebertragung bereits
    // selbst an (semaphorengesteuerter Schreib-Thread), ein separater Flush-Schritt entfaellt.
}

// --- CDC-ECM: lokale/entfernte MAC + Aktivierung ------------------------------------------
// Analog zu ST's ux_device_cdc_ecm.c (STM32H573I-DK-Referenzprojekt) -- lokale Node-ID geht
// als iMACAddress-Stringdeskriptor tatsaechlich auf den USB-Draht (s. STRIDX_ECM_MAC oben),
// die entfernte Node-ID ist rein interner USBX-Buchhaltungswert (nie uebertragen, die
// tatsaechliche MAC-Adresse des Hosts wird -- wie schon beim alten NCM-Treiber -- per ARP
// ermittelt) und wird hier nur der Vollstaendigkeit halber deterministisch aus derselben
// Chip-UID abgeleitet (letztes Byte um 1 versetzt, damit sie sich von der lokalen Adresse
// unterscheidet).
static UCHAR g_ecm_remote_node_id[UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH];

static VOID cdc_ecm_activate(VOID* instance) {
    (void)instance;
    log_info("USBX: CDC-ECM aktiviert (Host hat Netzwerk-Interface konfiguriert)");
}

static VOID cdc_ecm_deactivate(VOID* instance) {
    (void)instance;
    log_info("USBX: CDC-ECM deaktiviert");
}

// --- USBX-Systemspeicher -------------------------------------------------------------------
// ux_system_initialize() braucht einen zusammenhaengenden Speicherblock fuer alle internen
// Allokationen (Klasseninstanzen, Endpunkt-Puffer, ...) -- analog zu NX_APP_PACKET_POOL_SIZE
// bei NetX Duo, hier aber ein einfacher statischer Block statt ueber den byte_pool bezogen
// (usbd_device_setup() laeuft zwar bereits in einem ThreadX-Thread, aber vor app.cc's eigenem
// byte_pool-Zugriff eine zusaetzliche Kopplung einzufuehren lohnt sich fuer diese feste,
// einmalige Allokation nicht).
#define USBX_MEMORY_POOL_SIZE (32u * 1024u)
static UCHAR g_usbx_memory_pool[USBX_MEMORY_POOL_SIZE] __attribute__((aligned(4)));

static volatile uint32_t g_usb_isr_count = 0;

// USB_DRD_FS_IRQHandler() wird bewusst NICHT von CubeMX generiert (im .ioc kein NVIC-Haekchen
// fuer USB_DRD_FS_IRQn, s. main.c MX_USB_PCD_Init()) -- anders als bei der TinyUSB-Fassung
// NICHT, weil dieser Treiber den Interrupt exklusiv besaesse (USBX' ux_dcd_stm32-Treiber ist
// ganz im Gegenteil HAL_PCD-basiert und erwartet den STANDARD HAL_PCD_IRQHandler()-Ablauf,
// s. ux_dcd_stm32_callback.c's HAL_PCD_*Callback()-Ueberschreibungen), sondern damit die
// Freischaltung erst NACH ux_dcd_stm32_initialize()/Klassen-Registrierung passiert (s.
// usbd_device_setup() unten) -- vorher koennte ein Interrupt auf noch nicht initialisierten
// USBX-Zustand treffen.
void USB_DRD_FS_IRQHandler(void) {
    g_usb_isr_count++;
    HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
}

uint32_t usbd_device_get_isr_count(void) {
    return g_usb_isr_count;
}

// tud_mount_cb()/tud_umount_cb()-Aequivalent (s. TinyUSB-Fassung) -- ux_device_stack_initialize()
// letzter Parameter, aus dem Enumerationskontext heraus mit UX_DEVICE_ATTACHED/_ADDRESSED/
// _CONFIGURED/_SUSPENDED/_REMOVED (s. ux_api.h) aufgerufen. Reine Diagnose, kein
// Rueckgabewert-Handling noetig (USBX ignoriert ihn faktisch, s. Aufrufstelle in
// ux_device_stack_control_request_process.c/_transfer_all_request_abort.c).
static UINT usbx_device_state_change(ULONG state) {
    switch (state) {
        case UX_DEVICE_CONFIGURED:
            log_info("USBX: mounted (Host hat Enumeration abgeschlossen)");
            break;
        case UX_DEVICE_RESET:
            log_info("USBX: Bus-Reset");
            break;
        case UX_DEVICE_REMOVED:
            log_info("USBX: unmounted (Kabel getrennt)");
            break;
        case UX_DEVICE_SUSPENDED:
            log_info("USBX: suspend");
            break;
        default:
            break;
    }
    return UX_SUCCESS;
}

void usbd_device_setup(uint32_t chip_uid0, uint32_t chip_uid1, uint32_t chip_uid2, uint8_t const ecm_mac[6]) {
    snprintf(g_serial_string, sizeof(g_serial_string), "%08lX%08lX%08lX",
             (unsigned long)chip_uid0, (unsigned long)chip_uid1, (unsigned long)chip_uid2);
    snprintf(g_ecm_mac_string, sizeof(g_ecm_mac_string), "%02X%02X%02X%02X%02X%02X",
             ecm_mac[0], ecm_mac[1], ecm_mac[2], ecm_mac[3], ecm_mac[4], ecm_mac[5]);
    memcpy(g_ecm_remote_node_id, ecm_mac, UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH);
    g_ecm_remote_node_id[UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH - 1] ^= 0x01u;

    g_string_framework_length = 0;
    append_string_entry(STRIDX_MANUFACTURER, STRING_MANUFACTURER);
    append_string_entry(STRIDX_PRODUCT, STRING_PRODUCT);
    append_string_entry(STRIDX_SERIAL, g_serial_string);
    append_string_entry(STRIDX_CDC0_DEBUG, STRING_CDC0_DEBUG);
    append_string_entry(STRIDX_CDC1_MODBUS, STRING_CDC1_MODBUS);
    append_string_entry(STRIDX_ECM, STRING_ECM);
    append_string_entry(STRIDX_ECM_MAC, g_ecm_mac_string);

    HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 5, 0);

    UXASSERT(ux_system_initialize(g_usbx_memory_pool, USBX_MEMORY_POOL_SIZE, UX_NULL, 0),
            "USBX system initialize failed");

    // Muss vor ux_device_stack_class_register() der CDC-ECM-Instanz laufen (s. ST's
    // app_usbx_device.c MX_USBX_Device_Init()) -- registriert den generischen NetX-Duo-
    // Bruecken-Treiber (_ux_network_driver_entry, s. usb_cdc_ecm_driver.c), den die CDC-ECM-
    // Klasse intern verwendet.
    UXASSERT(ux_network_driver_init(), "USBX network driver init failed");

    UXASSERT(ux_device_stack_initialize((UCHAR*)UX_NULL, 0,           // kein High-Speed (USB_DRD_FS ist FS-only)
                                        (UCHAR*)g_device_framework, DEVICE_FRAMEWORK_LEN,
                                        g_string_framework, g_string_framework_length,
                                        g_language_id_framework, sizeof(g_language_id_framework),
                                        usbx_device_state_change),
            "USBX device stack initialize failed");

    static UX_SLAVE_CLASS_CDC_ACM_PARAMETER debug_parameter;
    debug_parameter.ux_slave_class_cdc_acm_instance_activate = cdc_debug_activate;
    debug_parameter.ux_slave_class_cdc_acm_instance_deactivate = cdc_debug_deactivate;
    UXASSERT(ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name, ux_device_class_cdc_acm_entry,
                                            1, ITF_CDC0_DEBUG_CONTROL, &debug_parameter),
            "USBX CDC-ACM Debug register failed");

    static UX_SLAVE_CLASS_CDC_ACM_PARAMETER modbus_parameter;
    modbus_parameter.ux_slave_class_cdc_acm_instance_activate = cdc_modbus_activate;
    modbus_parameter.ux_slave_class_cdc_acm_instance_deactivate = cdc_modbus_deactivate;
    UXASSERT(ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name, ux_device_class_cdc_acm_entry,
                                            1, ITF_CDC1_MODBUS_CONTROL, &modbus_parameter),
            "USBX CDC-ACM Modbus register failed");

    static UX_SLAVE_CLASS_CDC_ECM_PARAMETER ecm_parameter;
    ecm_parameter.ux_slave_class_cdc_ecm_instance_activate = cdc_ecm_activate;
    ecm_parameter.ux_slave_class_cdc_ecm_instance_deactivate = cdc_ecm_deactivate;
    memcpy(ecm_parameter.ux_slave_class_cdc_ecm_parameter_local_node_id, ecm_mac,
           UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH);
    memcpy(ecm_parameter.ux_slave_class_cdc_ecm_parameter_remote_node_id, g_ecm_remote_node_id,
           UX_DEVICE_CLASS_CDC_ECM_NODE_ID_LENGTH);
    UXASSERT(ux_device_stack_class_register(_ux_system_slave_class_cdc_ecm_name, ux_device_class_cdc_ecm_entry,
                                            1, ITF_ECM_CONTROL, &ecm_parameter),
            "USBX CDC-ECM register failed");

    // hpcd_USB_DRD_FS ist zu diesem Zeitpunkt bereits per HAL_PCD_Init() initialisiert (von
    // CubeMX-generiertem Code in main.c/MX_USB_PCD_Init(), laeuft unbedingt schon waehrend
    // main() VOR tx_kernel_enter() -- anders als bei der TinyUSB-Fassung, wo dieselbe HAL-Init
      // absichtlich NIE lief, s. main.c-Kommentar). ux_dcd_stm32_initialize() selbst fasst laut
    // eigenem Quelltext (ux_dcd_stm32_initialize.c) keine Register an, legt nur die interne
    // UX_DCD_STM32-Buchhaltungsstruktur an -- die Reihenfolge HAL_PCD_Init() (in main.c) VOR
    // ux_dcd_stm32_initialize() (hier) entspricht exakt ST's eigenem STM32H573I-DK-
    // Referenzprojekt (app_usbx_device.c MX_USBX_Device_Stack_Init()).
    UXASSERT(ux_dcd_stm32_initialize((ULONG)USB_DRD_FS, (ULONG)&hpcd_USB_DRD_FS),
            "USBX STM32 DCD initialize failed");

    HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);

    UXASSERT(tx_semaphore_create(&g_modbus_tx_ready, "Modbus CDC TX Ready", 0),
            "Modbus CDC TX semaphore create failed");
    UXASSERT(tx_thread_create(&g_modbus_rx_thread, "Modbus CDC RX", modbus_rx_thread_entry, 0,
                              g_modbus_rx_thread_stack, MODBUS_THREAD_STACK_SIZE,
                              MODBUS_CDC_THREAD_PRIORITY, MODBUS_CDC_THREAD_PRIORITY,
                              TX_NO_TIME_SLICE, TX_AUTO_START),
            "Modbus CDC RX thread create failed");
    UXASSERT(tx_thread_create(&g_modbus_tx_thread, "Modbus CDC TX", modbus_tx_thread_entry, 0,
                              g_modbus_tx_thread_stack, MODBUS_THREAD_STACK_SIZE,
                              MODBUS_CDC_THREAD_PRIORITY, MODBUS_CDC_THREAD_PRIORITY,
                              TX_NO_TIME_SLICE, TX_AUTO_START),
            "Modbus CDC TX thread create failed");

    log_info("USBX device stack initialized");
}

_Noreturn void usbd_device_loop(void (*poll_hook)(void)) {
    bool started = false;
    bool vsense_last = false;

    for (;;) {
        bool vsense_now = (HAL_GPIO_ReadPin(USB_VSENSE_GPIO_Port, USB_VSENSE_Pin) == GPIO_PIN_SET);
        if (vsense_now != vsense_last) {
            if (vsense_now) {
                // HAL_PCD_Start(): schaltet den Pull-Up an D+ zu (der Host beginnt zu
                // enumerieren) -- Aequivalent zu TinyUSBs tud_connect(). Nur einmalig
                // aufzurufen (kein erneuter Start bei jedem VSENSE-Flackern), daher das
                // zusaetzliche "started"-Flag statt blind bei jedem vsense_now==true
                // HAL_PCD_Start() erneut zu rufen.
                if (!started) {
                    HAL_PCD_Start(&hpcd_USB_DRD_FS);
                    started = true;
                }
                log_info("USB_VSENSE: 5V erkannt, USB-Stack verbunden");
            } else {
                HAL_PCD_Stop(&hpcd_USB_DRD_FS);
                started = false;
                log_info("USB_VSENSE: keine 5V mehr, USB-Stack getrennt");
            }
            vsense_last = vsense_now;
        }

        if (poll_hook) {
            poll_hook();
        }

        // Fester kurzer Sleep statt TinyUSBs adaptivem tud_task_ext()-Timeout: USBX' eigene
        // Klassen-Threads (bulkin/bulkout/interrupt, s. Klassenkommentar in usbd_device.h)
        // laufen bereits voll ereignisgetrieben unabhaengig von dieser Schleife -- die
        // urspruengliche Adaptivitaet (kurzes Timeout NUR bei ausstehendem TX-Paket, sonst
        // unbegrenzt) diente ausschliesslich dazu, TinyUSBs tud_task()-Polling nicht unnoetig
        // oft aufzuwecken, s. usb_ncm_driver.h-Historie -- dieser Grund entfaellt hier
        // komplett. 10 Ticks (~100ms) reichen fuer die VSENSE-Reaktionszeit.
        tx_thread_sleep(10);
    }
}
