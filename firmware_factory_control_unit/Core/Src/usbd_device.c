#include "usbd_device.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ux_api.h"
#include "ux_device_stack.h"
#include "ux_device_class_cdc_acm.h"
#include "ux_dcd_stm32.h"
#include "ux_network_driver.h"
#include "usbd_ncm.h"
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
    STRIDX_NCM,
    STRIDX_NCM_MAC,
};

static char const* const STRING_MANUFACTURER = "Klaus Liebler";
static char const* const STRING_PRODUCT = "FactoryControl Unit";
static char const* const STRING_CDC0_DEBUG = "FactoryControl Debug";
static char const* const STRING_CDC1_MODBUS = "FactoryControl Modbus";
static char const* const STRING_NCM = "FactoryControl Network";

// --- Interface-/Endpunkt-Layout ----------------------------------------------------------
// 1x CDC-NCM (Netzwerk) + 2x CDC-ACM (Debug, Modbus). NCM statt RNDIS: RNDIS lud zwar in
// Windows' eingebautem Treiber, brachte diesen Rechner aber mit einem Bluescreen
// (IRQL_NOT_LESS_OR_EQUAL) zum Absturz -- Ursache trotz ausfuehrlicher Codepruefung nicht
// gefunden, vermutlich eine bekannte Instabilitaet in Windows' Legacy-RNDIS-Treiber (s.
// Commit-Historie). NCM ist der USB-IF-STANDARD-Nachfolger, den dieses Projekt bereits VOR der
// USBX-Migration unter TinyUSB hardwareverifiziert (DHCP+HTTPS) einsetzte -- s. usbd_ncm.h/.c
// (komplette Neuimplementierung als USBX-Geraeteklasse, da USBX selbst keine NCM-Klasse
// mitbringt) und usb_ncm_driver.c (NetX-Duo-Bruecke, faktisch unveraendert aus der TinyUSB-Aera
// wiederhergestellt).
//
// Anders als RNDIS hat NCM KEINE "muss Interface 0/1 sein"-Eigenart (die betraf ausschliesslich
// Windows' RNDIS-Composite-Erkennung) -- die Reihenfolge hier ist rein kosmetisch beibehalten
// (NCM zuerst), keine funktionale Notwendigkeit. Alle drei Funktionen passen weiterhin exakt in
// die 8 verfuegbaren USB_DRD_FS-Hardware-Endpunkte (EP0 + 7) -- EP7 bleibt frei (potenzielle
// spaetere MSC-Stufe, wie schon bei der TinyUSB-Fassung vorgesehen).
enum {
    ITF_NCM_CONTROL = 0,
    ITF_NCM_DATA,
    ITF_CDC0_DEBUG_CONTROL,
    ITF_CDC0_DEBUG_DATA,
    ITF_CDC1_MODBUS_CONTROL,
    ITF_CDC1_MODBUS_DATA,
    ITF_COUNT,
};

#define EP_NCM_NOTIF 0x81u
#define EP_NCM_OUT 0x02u
#define EP_NCM_IN 0x82u
// Muss >= 8 (NETWORK_CONNECTION, ohne Nutzdaten) UND >= 16 (CONNECTION_SPEED_CHANGE, 8 Byte
// Header + 8 Byte Nutzdaten) sein, s. ncm_notify_t/ncm_notify_send() in usbd_ncm.c.
#define EP_NCM_NOTIF_SIZE 16u
#define EP_NCM_DATA_SIZE 64u

#define EP_CDC0_NOTIF 0x83u
#define EP_CDC0_OUT 0x04u
#define EP_CDC0_IN 0x84u
#define EP_CDC0_NOTIF_SIZE 8u
#define EP_CDC0_DATA_SIZE 64u

#define EP_CDC1_NOTIF 0x85u
#define EP_CDC1_OUT 0x06u
#define EP_CDC1_IN 0x86u
#define EP_CDC1_NOTIF_SIZE 8u
#define EP_CDC1_DATA_SIZE 64u

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
#define CDC_ETH_NET_FD_LEN 13u
#define CDC_NCM_FD_LEN 6u
#define EP_DESC_LEN 7u

// Ein CDC-ACM-Funktionsblock (IAD + Control-Interface mit Header/CallMgmt/ACM/Union-FD +
// Notification-EP + Data-Interface mit 2x Bulk-EP) -- byteidentischer Aufbau fuer beide
// CDC-ACM-Funktionen (Debug/Modbus).
#define CDC_FUNCTION_LEN (IAD_LEN + ITF_LEN + CDC_HEADER_FD_LEN + CDC_CALL_MGMT_FD_LEN + \
                          CDC_ACM_FD_LEN + CDC_UNION_FD_LEN + EP_DESC_LEN + \
                          ITF_LEN + EP_DESC_LEN + EP_DESC_LEN)

// Der CDC-NCM-Funktionsblock (IAD + Control-Interface mit Header/Union/EthernetNetworking/NCM-FD
// + Notification-EP + Daten-Interface Alt=0 [0 EP] + Daten-Interface Alt=1 [2 Bulk-EP]) --
// byteidentisch zu TinyUSBs TUD_CDC_NCM_DESCRIPTOR()-Makro-Expansion (device/usbd.h,
// TUD_CDC_NCM_DESC_LEN = 8+9+5+5+13+6+7+9+9+7+7 = 85, hier per _Static_assert unten
// gegengeprueft), s. usbd_ncm.h fuer den Gesamtzusammenhang.
#define NCM_FUNCTION_LEN (IAD_LEN + ITF_LEN + CDC_HEADER_FD_LEN + CDC_UNION_FD_LEN + \
                          CDC_ETH_NET_FD_LEN + CDC_NCM_FD_LEN + EP_DESC_LEN + \
                          ITF_LEN + ITF_LEN + EP_DESC_LEN + EP_DESC_LEN)
_Static_assert(NCM_FUNCTION_LEN == 85u, "NCM_FUNCTION_LEN weicht von TinyUSBs TUD_CDC_NCM_DESC_LEN ab");

#define CONFIG_DESC_LEN 9u
#define CONFIG_TOTAL_LEN (CONFIG_DESC_LEN + NCM_FUNCTION_LEN + 2u * CDC_FUNCTION_LEN)
#define DEVICE_FRAMEWORK_LEN (DEVICE_DESC_LEN + CONFIG_TOTAL_LEN)

// CDC1.2/[USBNCM1.0]-Deskriptor-Subtypen (bDescriptorSubtype im CS_INTERFACE-Deskriptor,
// bDescriptorType 0x24) -- dieselben numerischen Werte wie im TinyUSB-Stack intern verwendet,
// hier von Hand aufgefuehrt, da USBX (anders als TinyUSB) keine TUD_CDC_DESCRIPTOR()-aehnlichen
// Hilfsmakros mitbringt (s. Plan: ST's eigener Deskriptor-Baukasten in ux_device_descriptors.c
// ist Anwendungs-/Demo-Code, kein USBX-Middleware-Bestandteil, und fuer unser fest bekanntes
// 3-Funktionen-Composite unverhaeltnismaessig, s. Commit-Nachricht).
#define CS_INTERFACE 0x24u
#define CDC_FD_HEADER 0x00u
#define CDC_FD_CALL_MANAGEMENT 0x01u
#define CDC_FD_ACM 0x02u
#define CDC_FD_UNION 0x06u
#define CDC_FD_ETHERNET_NETWORKING 0x0Fu
#define CDC_FD_NCM 0x1Au

// CDC-NCM 1.0 Kapitel 5.1: bInterfaceSubClass=0x0D (Network Control Model) statt 0x02 (ACM);
// Daten-Interface-bInterfaceProtocol=0x01 (Network Transfer Block), statt 0x00 bei CDC-ACM/
// RNDIS-Daten-Interfaces.
#define CDC_SUBCLASS_NCM 0x0Du
#define NCM_DATA_PROTOCOL_NTB 0x01u

// CDC-NCM 1.0 Tabelle 5-2 bmNetworkCapabilities: keine der optionalen Capability-Bits gesetzt
// (kein SET_ETHERNET_PACKET_FILTER-Statistik-Support, kein NET_ADDRESS, ...) -- passend zu
// ncm_control_request()s reiner ACK-Quittierung dieser optionalen Requests, s. usbd_ncm.c.
#define NCM_NETWORK_CAPS_NONE 0x00u

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

    // ============ CDC-NCM ("FactoryControl Network") ============
    // bFunctionClass/SubClass/Protocol = 0x02/0x0D/0x00 (Communications/Network Control Model) --
    // der USB-IF-Standardweg fuer eine virtuelle Netzwerkkarte, byteidentisch zu TinyUSBs
    // TUD_CDC_NCM_DESCRIPTOR()-Expansion (s. Kommentar bei NCM_FUNCTION_LEN oben).
    IAD_LEN, 0x0Bu, ITF_NCM_CONTROL, 2u, 0x02u, CDC_SUBCLASS_NCM, 0x00u, STRIDX_NCM,
    // Control-Interface -- Alt=0, 1 Endpunkt (Notification).
    ITF_LEN, 0x04, ITF_NCM_CONTROL, 0u, 1u, 0x02u, CDC_SUBCLASS_NCM, 0x00u, STRIDX_NCM,
    CDC_HEADER_FD_LEN, CS_INTERFACE, CDC_FD_HEADER, U16LE(0x0110),
    CDC_UNION_FD_LEN, CS_INTERFACE, CDC_FD_UNION, ITF_NCM_CONTROL, ITF_NCM_DATA,
    // Ethernet Networking FD (CDC1.2 5.2.3.4) -- iMACAddress verweist auf den 12-Hex-Ziffern-
    // MAC-Stringdeskriptor (s. DEVICE_USB_NCM_MAC_STRING/usbd_device_setup()), bmEthernetStatistics=0
    // (keine der optionalen Statistik-Faehigkeiten beworben), wMaxSegmentSize=NCM_MTU (14 Byte
    // Ethernet-Header + 1500 Byte IP-MTU, s. gleichlautende Konstante in usb_ncm_driver.c),
    // wNumberMCFilters=0, bNumberPowerFilters=0.
    CDC_ETH_NET_FD_LEN, CS_INTERFACE, CDC_FD_ETHERNET_NETWORKING, STRIDX_NCM_MAC,
        0x00u, 0x00u, 0x00u, 0x00u, U16LE(1514u), U16LE(0u), 0x00u,
    // NCM FD (CDC-NCM 1.0 Tabelle 5-2) -- bcdNcmVersion=1.00, bmNetworkCapabilities=0 (s.
    // NCM_NETWORK_CAPS_NONE oben).
    CDC_NCM_FD_LEN, CS_INTERFACE, CDC_FD_NCM, U16LE(0x0100), NCM_NETWORK_CAPS_NONE,
    EP_DESC_LEN, 0x05, EP_NCM_NOTIF, 0x03u, U16LE(EP_NCM_NOTIF_SIZE), 8u,
    // Daten-Interface Alt=0 -- 0 Endpunkte (idle), bis der Host per SET_INTERFACE auf Alt=1
    // umschaltet. bInterfaceClass=0x0A (CDC Data), bInterfaceProtocol=0x01 (Network Transfer
    // Block, s. NCM_DATA_PROTOCOL_NTB) statt 0x00 bei CDC-ACM-Daten-Interfaces.
    ITF_LEN, 0x04, ITF_NCM_DATA, 0u, 0u, 0x0Au, 0x00u, NCM_DATA_PROTOCOL_NTB, 0u,
    // Daten-Interface Alt=1 -- 2 Bulk-Endpunkte (aktiv).
    ITF_LEN, 0x04, ITF_NCM_DATA, 1u, 2u, 0x0Au, 0x00u, NCM_DATA_PROTOCOL_NTB, 0u,
    EP_DESC_LEN, 0x05, EP_NCM_IN, 0x02u, U16LE(EP_NCM_DATA_SIZE), 0u,
    EP_DESC_LEN, 0x05, EP_NCM_OUT, 0x02u, U16LE(EP_NCM_DATA_SIZE), 0u,

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
};
_Static_assert(sizeof(g_device_framework) == DEVICE_FRAMEWORK_LEN,
               "g_device_framework literal byte count does not match the hand-computed "
               "DEVICE_FRAMEWORK_LEN -- a mismatch here silently zero-pads/truncates the "
               "descriptor (C array-initializer rules: fewer initializers than the declared "
               "size is NOT a compile error) and corrupts every descriptor after the first "
               "short one, which breaks enumeration on the host without any build-time signal.");

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

// --- Modbus-CDC: nur noch Instanz-Verwaltung ------------------------------------------------
// ModbusRtuServer::Run() (stm32_libs/modbus/modbus_rtu_server.hpp) ruft ux_device_class_cdc_acm_
// read()/_write() SELBST direkt blockierend auf (bindet dafuer den vollen USBX-CDC-ACM-Header
// ein) -- diese Datei muss dafuer nur noch die Instanz herausgeben, kein Zwischen-Wrapper mehr.
static VOID cdc_modbus_activate(VOID* instance) {
    g_cdc_modbus_instance = (UX_SLAVE_CLASS_CDC_ACM*)instance;
}

static VOID cdc_modbus_deactivate(VOID* instance) {
    (void)instance;
    g_cdc_modbus_instance = NULL;
}

UX_SLAVE_CLASS_CDC_ACM* usbd_cdc_modbus_instance(void) {
    return g_cdc_modbus_instance;
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

// USB_DRD_FS_IRQHandler() wird bewusst NICHT von CubeMX generiert (im .ioc kein NVIC-Haekchen
// fuer USB_DRD_FS_IRQn, s. main.c MX_USB_PCD_Init()) -- anders als bei der TinyUSB-Fassung
// NICHT, weil dieser Treiber den Interrupt exklusiv besaesse (USBX' ux_dcd_stm32-Treiber ist
// ganz im Gegenteil HAL_PCD-basiert und erwartet den STANDARD HAL_PCD_IRQHandler()-Ablauf,
// s. ux_dcd_stm32_callback.c's HAL_PCD_*Callback()-Ueberschreibungen), sondern damit die
// Freischaltung erst NACH ux_dcd_stm32_initialize()/Klassen-Registrierung passiert (s.
// usbd_device_setup() unten) -- vorher koennte ein Interrupt auf noch nicht initialisierten
// USBX-Zustand treffen.
void USB_DRD_FS_IRQHandler(void) {
    HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
}

// tud_mount_cb()/tud_umount_cb()-Aequivalent (s. TinyUSB-Fassung).
//
// KORRIGIERT (urspruenglicher Kommentar war falsch/nie hardwaregetestet, s. Commit-Historie):
// In diesem vendorten USBX-Stand (libs/ST/usbx) wird diese Callback KEINESFALLS mit
// UX_DEVICE_CONFIGURED/_ADDRESSED/_RESET aufgerufen -- grep ueber den gesamten Baum zeigt genau
// EINEN Aufrufer mit einem generischen ux_api.h-UX_DEVICE_*-Wert
// (ux_device_stack_disconnect.c: nur UX_DEVICE_REMOVED, u.a. bei JEDEM Bus-Reset ausgeloest,
// nicht nur bei echtem Kabelabzug -- s. HAL_PCD_ResetCallback in ux_dcd_stm32_callback.c). Der
// tatsaechliche "Enumeration abgeschlossen"-Indikator sind stattdessen die klassen-eigenen
// Activate-Callbacks (cdc_debug_activate()/cdc_modbus_activate()/ncm_activate() in usbd_ncm.c), vom
// USBX-Klassen-Framework beim SET_CONFIGURATION/SET_INTERFACE aufgerufen -- NICHT dieser
// generischen Callback. Die restlichen tatsaechlich aufgerufenen Werte kommen aus
// ux_dcd_stm32_callback.c/ux_dcd_stm32_initialize_complete.c (STM32-DCD-spezifische
// UX_DCD_STM32_*-Codes ausserhalb des ux_api.h-Wertebereichs, s. ux_dcd_stm32.h) plus
// UX_DEVICE_ATTACHED (ebenfalls bei jedem Bus-Reset, direkt nach dem UX_DEVICE_REMOVED oben).
// UX_DCD_STM32_SOF_RECEIVED bewusst nicht geloggt (feuert alle 1ms, reine Log-Flut).
static UINT usbx_device_state_change(ULONG state) {
    switch (state) {
        case UX_DEVICE_ATTACHED:
            log_info("USBX: attached (PHY/Bus-Reset abgeschlossen, EP0 offen)");
            break;
        case UX_DEVICE_REMOVED:
            log_info("USBX: disconnect-Event (Bus-Reset ODER echter Kabelabzug)");
            break;
        case UX_DCD_STM32_DEVICE_CONNECTED:
            log_info("USBX: DCD connected (VBUS/D+ Pullup erkannt)");
            break;
        case UX_DCD_STM32_DEVICE_DISCONNECTED:
            log_info("USBX: DCD disconnected");
            break;
        case UX_DCD_STM32_DEVICE_SUSPENDED:
            log_info("USBX: DCD suspend");
            break;
        case UX_DCD_STM32_DEVICE_RESUMED:
            log_info("USBX: DCD resume");
            break;
        default:
            break;
    }
    return UX_SUCCESS;
}

void usbd_device_setup(char const* serial_string, uint8_t const net_mac[6], char const* net_mac_string) {
    g_string_framework_length = 0;
    append_string_entry(STRIDX_MANUFACTURER, STRING_MANUFACTURER);
    append_string_entry(STRIDX_PRODUCT, STRING_PRODUCT);
    append_string_entry(STRIDX_SERIAL, serial_string);
    append_string_entry(STRIDX_CDC0_DEBUG, STRING_CDC0_DEBUG);
    append_string_entry(STRIDX_CDC1_MODBUS, STRING_CDC1_MODBUS);
    append_string_entry(STRIDX_NCM, STRING_NCM);
    append_string_entry(STRIDX_NCM_MAC, net_mac_string);

    HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 5, 0);

    UXASSERT(ux_system_initialize(g_usbx_memory_pool, USBX_MEMORY_POOL_SIZE, UX_NULL, 0),
            "USBX system initialize failed");

    // Initialisiert USBX' generischen, klassenunabhaengigen NetX-Duo-Bruecken-Treiber
    // (_ux_network_driver_entry/_activate/_deactivate/_link_up/_down/_packet_received, s.
    // libs/ST/usbx/common/usbx_network), an den sich unsere eigene usbd_ncm.c-Klasse anbindet (s.
    // dortiger Klassenkommentar). NUR DEFENSIV/idempotent hier nochmal aufgerufen (interne
    // Guard-Variable verhindert ein zweites memset()) -- der WIRKLICH entscheidende Aufruf
    // passiert bereits VOR net_setup.cpp's nx_ip_interface_attach() in net_setup_create() (s.
    // dortiger ausfuehrlicher Kommentar): ux_network_driver_init() memset()t seine gesamte interne
    // usb_network_devices[]-Tabelle, und muss deshalb VOR dem allerersten NX_LINK_INTERFACE_ATTACH
    // laufen, das diese Tabelle befuellt -- nx_ip_interface_attach() laeuft aber schon in
    // tx_application_define() (VOR Scheduler-Start), waehrend usbd_device_setup() hier erst
    // spaeter aus dem UsbdDeviceThread heraus laeuft. Ein alleiniger Aufruf HIER (wie es eine
    // fruehere Fassung dieser Datei tat) haette ATTACHs Eintrag nachtraeglich wieder auf 0
    // zurueckgesetzt -- Symptom war ein fuer immer NULL bleibender ip_instance-Zeiger (Paket-Pool
    // liess sich nie aufloesen, DHCP kam trotz augenscheinlich funktionierendem NTB-Empfang nie
    // durch), s. Debugging-Sitzung.
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

    // Eigene NCM-Klasse (s. usbd_ncm.h/.c) -- anders als bei CDC-ACM gibt es hier keinen
    // ST-eigenen PARAMETER-Typ zu befuellen; die Registrierung braucht nur die Control-Interface-
    // Nummer (das Daten-Interface liegt per Konvention auf itf_control+1) und die lokale
    // MAC-Adresse (an _ux_network_driver_activate() weitergereicht, s. usbd_ncm.c).
    UXASSERT(usbd_ncm_class_register(ITF_NCM_CONTROL, net_mac), "USBX CDC-NCM register failed");

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

    // USB_DRD_FS (kein OTG, sondern der aeltere register-/PMA-basierte "USB Device"-Kern, wie er
    // auf STM32H563 statt eines OTG-Controllers verbaut ist) braucht fuer JEDEN Endpunkt eine
    // explizite Packet-Memory-Adresse -- HAL_PCD_EP_Open() (aufgerufen aus
    // ux_dcd_stm32_endpoint_create.c) setzt ep->pmaadress selbst NICHT, das ist Sache der
    // Applikation (klassisches CubeMX-USB-Device-Beispiel: usbd_conf.c::USBD_LL_Init() macht das
    // ueber HAL_PCDEx_PMAConfig() je Endpunkt). ST's eigenes USBX-Referenzbeispiel
    // (STM32H573I-DK, s. Kommentar oben) hat dieses Problem nicht, weil das ein OTG-HS-Board ist
    // -- OTG-Controller adressieren ueber FIFOs, nicht ueber PMA, brauchen diesen Schritt also
    // gar nicht. Ohne diesen Block bleiben alle ep->pmaadress-Felder bei ihrem
    // Zero-Initialisierungswert 0 stehen: JEDER Endpunkt (EP0 und alle sechs Klassen-Endpunkte)
    // teilt sich dieselbe physische PMA-Adresse und ueberschreibt sich gegenseitig -- das war der
    // Auslöser fuer die "USB-Geraet hat ungueltigen Geraetedeskriptor zurueckgegeben"-Meldung von
    // Windows: ein anderer Endpunkt hat der EP0-SETUP-Paketpufferstelle noch waehrend der
    // Verarbeitung Daten hinterhergeschrieben (s. Debug-Log: wLength wurde als 33800 statt der
    // tatsaechlich angefragten 8/18 Byte gelesen).
    //
    // NACHTRAG (zweiter Fund im selben Bereich, gleicher Debug-Log-Symptom): der erste
    // PMA-Anlauf (pma_offset ab 0) uebersah, dass USB_DRD_PMA_BUFF (stm32h563xx.h) -- die
    // Buffer-Deskriptor-Tabelle (TXBD/RXBD-Adress+Laengen-Paare, die die Hardware fuer JEDEN
    // Endpunktkanal braucht) -- auf DIESELBE Basisadresse USB_DRD_PMAADDR gelegt ist wie der
    // Nutzdatenbereich selbst, nicht getrennt davon. Diese Tabelle belegt fest
    // UX_DCD_STM32_MAX_ED (8, s. ux_stm32_config.h) Eintraege a 8 Byte (sizeof
    // USB_DRD_PMABuffDescTypeDef: TXBD+RXBD je uint32_t) = 64 Byte am Anfang der PMA -- mit
    // pma_offset bei 0 beginnend ueberschrieb EP0s eigener Datenpuffer also die
    // Deskriptortabelle aller Endpunkte (inklusive seiner eigenen), was exakt zum beobachteten,
    // *deterministisch reproduzierbaren* falschen wLength (immer derselbe Wert 33800 bei jedem
    // Boot) passt: RAM-Startzustand nach Reset ist deterministisch, keine echte
    // Endpunkt-Kollision noetig, um denselben falschen Wert jedes Mal zu erzeugen.
    //
    // Reihenfolge/Adressen: nach den reservierten 64 Byte zuerst EP0 (OUT dann IN, je
    // bMaxPacketSize0), danach je Klasse Notification-IN, Bulk-OUT, Bulk-IN -- durchlaufend auf
    // 4 Byte aufgerundet (USB_ReadPMA/USB_WritePMA kopieren wortweise, s. stm32h5xx_ll_usb.c),
    // alle Adressen und Groessen muessen ausserdem <= USB_DRD_PMA_SIZE (2048 Byte, s.
    // stm32h563xx.h) bleiben.
    {
        uint16_t pma_offset = (uint16_t)(UX_DCD_STM32_MAX_ED * sizeof(USB_DRD_PMABuffDescTypeDef));
#define PMA_ALLOC(ep_addr, size)                                                         \
        do {                                                                             \
            HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, (ep_addr), PCD_SNG_BUF, pma_offset);   \
            pma_offset = (uint16_t)((pma_offset + (size) + 3u) & ~3u);                   \
        } while (0)

        PMA_ALLOC(0x00u, 64u);              // EP0 OUT
        PMA_ALLOC(0x80u, 64u);              // EP0 IN
        PMA_ALLOC(EP_NCM_NOTIF, EP_NCM_NOTIF_SIZE);
        PMA_ALLOC(EP_NCM_OUT, EP_NCM_DATA_SIZE);
        PMA_ALLOC(EP_NCM_IN, EP_NCM_DATA_SIZE);
        PMA_ALLOC(EP_CDC0_NOTIF, EP_CDC0_NOTIF_SIZE);
        PMA_ALLOC(EP_CDC0_OUT, EP_CDC0_DATA_SIZE);
        PMA_ALLOC(EP_CDC0_IN, EP_CDC0_DATA_SIZE);
        PMA_ALLOC(EP_CDC1_NOTIF, EP_CDC1_NOTIF_SIZE);
        PMA_ALLOC(EP_CDC1_OUT, EP_CDC1_DATA_SIZE);
        PMA_ALLOC(EP_CDC1_IN, EP_CDC1_DATA_SIZE);
#undef PMA_ALLOC
        if (pma_offset > 2048u) {
            log_error("USB PMA layout overflow: %u > 2048 Byte", (unsigned)pma_offset);
            Error_Handler();
        }
    }

    HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);

    log_info("USBX device stack initialized");
}

_Noreturn void usbd_device_loop(void) {
    bool started = false;
    bool vsense_last = false;

    for (;;) {
#ifdef BOARD_NUCLEO_H563ZI
        // Test-Rig: USB_VSENSE (PC0, 10k/10k-Spannungsteiler an VBUS) ist board-spezifische
        // Zusatzbeschaltung der echten Platine, auf dem Nucleo unbeschaltet/floatend -- dort
        // gilt VBUS als immer da (das Nucleo erkennt VBUS an seinem USB-User-Connector CN13
        // bereits selbst ueber VBUS_SENSE/PA4, s. UM3115 Abschnitt 11.6.2; die PCD/USBX-Seite
        // braucht dafuer keine zusaetzliche Software-Gate-Logik). HAL_PCD_Start() laeuft dadurch
        // wie unten schon vorgesehen genau einmal beim ersten Schleifendurchlauf.
        bool vsense_now = true;
#else
        bool vsense_now = (HAL_GPIO_ReadPin(USB_VSENSE_GPIO_Port, USB_VSENSE_Pin) == GPIO_PIN_SET);
#endif
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
