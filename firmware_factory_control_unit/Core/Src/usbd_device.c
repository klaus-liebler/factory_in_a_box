#include "usbd_device.h"

#include <stdio.h>
#include <string.h>

#include "tusb.h"
#include "main.h"
#include "log.h"
#include "tx_api.h"

// --- USB VID/PID -------------------------------------------------------------------------
// Platzhalter (pid.codes-Wurzel-VID fuer nicht kommerziell vertriebene/private Projekte) --
// vor einer Weitergabe ausserhalb des eigenen Werks/Netzwerks durch eine eigene PID unter
// https://pid.codes ersetzen.
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

// Vom App-Chip-UID gebildet (s. usbd_device_setup()) -- 24 Hex-Ziffern (3x uint32_t) +
// Nullterminator.
static char g_serial_string[25] = "000000000000000000000000";

// MAC-Adress-String der virtuellen NCM-NIC -- CDC-NCM-Spezifikation verlangt 12 GROSSGESCHRIEBENE
// Hex-Ziffern ohne Trennzeichen (Format "AABBCCDDEEFF"). Inhalt kommt von usbd_device_setup()
// (dieselben 6 Bytes wie die Quelladresse im Ethernet-Header, s. App::ComputeNcmMac()).
static char g_ncm_mac_string[13] = "000000000000";

// --- Device-Deskriptor -----------------------------------------------------------------
// bDeviceClass=0xEF/bDeviceSubClass=0x02/bDeviceProtocol=0x01 (Interface Association Descriptor)
// ist bei Composite-Geraeten mit mehreren CDC-Funktionen erforderlich, damit Windows'
// USB-Common-Class-Generator-Treiber (usbccgp.sys) die Funktionen korrekt anhand der IADs
// gruppiert statt nur die erste Interface-Deskription zu verwenden.
static tusb_desc_device_t const g_device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = USB_BCD_DEVICE,
    .iManufacturer = STRIDX_MANUFACTURER,
    .iProduct = STRIDX_PRODUCT,
    .iSerialNumber = STRIDX_SERIAL,
    .bNumConfigurations = 1,
};

// --- Konfigurations-Deskriptor -----------------------------------------------------------
// Stufe 3 (s. Implementierungsplan): Debug-CDC + Modbus-CDC + CDC-NCM. MSC kommt in Stufe 4 dazu
// -- Endpunkt-Nummern werden dabei von Hand vergeben (s. Kommentar unten), damit am Ende alle
// vier Funktionen exakt in die 8 verfuegbaren Hardware-Endpunkte (EP0 + 7) passen.
enum {
    ITF_CDC0_DEBUG_CONTROL = 0,
    ITF_CDC0_DEBUG_DATA,
    ITF_CDC1_MODBUS_CONTROL,
    ITF_CDC1_MODBUS_DATA,
    ITF_NCM_CONTROL,
    ITF_NCM_DATA,
    ITF_COUNT,
};

// Endpunkt-Adressen: CDC0 belegt EP1 (Notification, IN-only) + EP2 (Bulk-Datenpaar, IN+OUT auf
// derselben Nummer -- der USB_DRD_FS-Block kann pro Endpunkt-Nummer unabhaengige IN/OUT-Puffer
// fuehren, s. Endpoint-Budget-Analyse). CDC1 (Modbus) setzt mit EP3/EP4 fort, NCM mit EP5/EP6.
// MSC (Stufe 4) setzt mit EP7 fort -- damit sind alle 8 Hardware-Endpunkte (inkl. EP0) belegt.
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

#define EP_NCM_NOTIF 0x85u
#define EP_NCM_OUT 0x06u
#define EP_NCM_IN 0x86u
#define EP_NCM_NOTIF_SIZE 16u
#define EP_NCM_DATA_SIZE 64u
// CDC-NCM 1.0 Tabelle 6-4: Mindestgroesse 2048 -- muss zum tusb_config.h-Wert passen (dort per
// _Static_assert unten geprueft, s. weiter unten).
#define NCM_MAX_SEGMENT_SIZE CFG_TUD_NCM_IN_NTB_MAX_SIZE

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 2 * TUD_CDC_DESC_LEN + TUD_CDC_NCM_DESC_LEN)

static uint8_t const g_config_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),
    TUD_CDC_DESCRIPTOR(ITF_CDC0_DEBUG_CONTROL, STRIDX_CDC0_DEBUG,
                       EP_CDC0_NOTIF, EP_CDC0_NOTIF_SIZE, EP_CDC0_OUT, EP_CDC0_IN, EP_CDC0_DATA_SIZE),
    TUD_CDC_DESCRIPTOR(ITF_CDC1_MODBUS_CONTROL, STRIDX_CDC1_MODBUS,
                       EP_CDC1_NOTIF, EP_CDC1_NOTIF_SIZE, EP_CDC1_OUT, EP_CDC1_IN, EP_CDC1_DATA_SIZE),
    TUD_CDC_NCM_DESCRIPTOR(ITF_NCM_CONTROL, STRIDX_NCM, STRIDX_NCM_MAC,
                           EP_NCM_NOTIF, EP_NCM_NOTIF_SIZE, EP_NCM_OUT, EP_NCM_IN, EP_NCM_DATA_SIZE,
                           NCM_MAX_SEGMENT_SIZE, 8, NCM_NETWORK_CAPS_NONE),
};

_Static_assert(sizeof(g_config_descriptor) == CONFIG_TOTAL_LEN, "Config descriptor length mismatch");

// UTF-16LE-Zwischenpuffer fuer tud_descriptor_string_cb() -- TinyUSB verlangt den Rueckgabewert
// bis zum naechsten Aufruf gueltig, ein einzelner statischer Puffer genuegt (Aufrufe erfolgen
// synchron/nicht ueberlappend aus dem Enumerationsprozess heraus).
static uint16_t g_string_desc_buf[32];

// USB_DRD_FS_IRQHandler() wird bewusst NICHT von CubeMX generiert (im .ioc kein NVIC-Haekchen
// fuer USB_DRD_FS_IRQn, s. main.c MX_USB_PCD_Init()/USER CODE USB_Init 2) -- TinyUSBs
// dcd_stm32_fsdev-Treiber besitzt die Peripherie exklusiv und regelt Freischaltung/Sperrung des
// Interrupts komplett selbst (fsdev_int_enable()/_disable(), aufgerufen aus tusb_init()/
// tud_deinit() ueber dcd_int_enable()/dcd_int_disable()); ein von CubeMX zusaetzlich generierter
// HAL_NVIC_EnableIRQ()-Aufruf in HAL_PCD_MspInit() koennte den Interrupt schon vor tusb_init()
// feuern lassen und TinyUSBs noch nicht existierenden internen Zustand (Queue/Mutex) korrumpieren.
// Diese Funktion hier ist der vollstaendige, einzige Handler -- als starke Definition ueberschreibt
// sie automatisch den schwachen Default in startup_stm32h563xx.s, ganz ohne CubeMX-Beteiligung.
static volatile uint32_t g_usb_isr_count = 0;

void USB_DRD_FS_IRQHandler(void) {
    g_usb_isr_count++;
    tud_int_handler(0);
}

uint32_t usbd_device_get_isr_count(void) {
    return g_usb_isr_count;
}

// --- CDC-Wrapper fuer ModbusRtuServer (s. usbd_device.h) -- ITF_CDC1_MODBUS_DATA ist die zweite
// TUD_CDC_DESCRIPTOR()-Instanz oben (g_config_descriptor), TinyUSB zaehlt CDC-Instanzen in der
// Reihenfolge ihres Auftretens im Konfigurations-Deskriptor: Index 0 = Debug, Index 1 = Modbus.
#define CDC_ITF_MODBUS 1u

uint32_t usbd_cdc_modbus_available(void) {
    return tud_cdc_n_available(CDC_ITF_MODBUS);
}

uint32_t usbd_cdc_modbus_read(uint8_t* buffer, uint32_t bufsize) {
    return tud_cdc_n_read(CDC_ITF_MODBUS, buffer, bufsize);
}

uint32_t usbd_cdc_modbus_write(const uint8_t* buffer, uint32_t len) {
    return tud_cdc_n_write(CDC_ITF_MODBUS, buffer, len);
}

void usbd_cdc_modbus_write_flush(void) {
    tud_cdc_n_write_flush(CDC_ITF_MODBUS);
}

// --- TinyUSB-Lifecycle-Callbacks (laufen aus tud_task()-Kontext, also im usbd_device_thread --
// unproblematisch zu loggen, im Gegensatz zur ISR selbst). Reine Diagnose fuer den
// Enumerations-Haenger nach echtem Host-Anschluss (s. Debugging-Sitzung): zeigt, ob/wann der
// Host die Enumeration tatsaechlich abschliesst (SET_CONFIGURATION -> tud_mount_cb()).
void tud_mount_cb(void) {
    log_info("TinyUSB: mounted (Host hat Enumeration abgeschlossen)");
}

void tud_umount_cb(void) {
    log_info("TinyUSB: unmounted (Bus-Reset oder Kabel getrennt)");
}

void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    log_info("TinyUSB: suspend");
}

void tud_resume_cb(void) {
    log_info("TinyUSB: resume");
}

void usbd_device_setup(uint32_t chip_uid0, uint32_t chip_uid1, uint32_t chip_uid2, uint8_t const ncm_mac[6]) {
    snprintf(g_serial_string, sizeof(g_serial_string), "%08lX%08lX%08lX",
             (unsigned long)chip_uid0, (unsigned long)chip_uid1, (unsigned long)chip_uid2);

    snprintf(g_ncm_mac_string, sizeof(g_ncm_mac_string), "%02X%02X%02X%02X%02X%02X",
             ncm_mac[0], ncm_mac[1], ncm_mac[2], ncm_mac[3], ncm_mac[4], ncm_mac[5]);

    // Prioritaet einmalig setzen, bevor der Interrupt ueberhaupt zum ersten Mal freigeschaltet
    // werden kann (per tusb_init() unten oder spaeter in usbd_device_loop()) -- TinyUSBs
    // fsdev_int_enable() setzt selbst keine Prioritaet, nur NVIC_EnableIRQ(). Ohne diese Zeile
    // bliebe der Reset-Default (Prioritaet 0 = hoechste) stehen und wuerde ThreadX-kritische
    // Interrupts gleicher/niedrigerer Prioritaet (z.B. GPDMA/SPI2 bei 5, s. .ioc) verdraengen.
    HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 5, 0);

    // USB_VSENSE (PC0, 10k/10k-Spannungsteiler an den 5V des USB-Ports, s. .ioc): tusb_init()
    // erst aufrufen, wenn tatsaechlich ein Kabel steckt -- vermeidet einen unbenutzten
    // PCD/BTABLE-Init ohne angeschlossenen Host. Ist beim Boot noch kein Kabel gesteckt, holt
    // usbd_device_loop() das per Polling nach (s. dort).
    if (HAL_GPIO_ReadPin(USB_VSENSE_GPIO_Port, USB_VSENSE_Pin) == GPIO_PIN_SET) {
        // Rueckgabewert bewusst pruefen (frueher hier ignoriert): tusb_init() kann intern per
        // TU_ASSERT() scheitern (z.B. ABI-Mismatch zwischen tinyusb_port und dem Rest der
        // Firmware, s. CMakeLists.txt-Kommentar zu TX_INCLUDE_USER_DEFINE_FILE) und dabei still
        // "false" zurueckgeben, OHNE dass ohne angeschlossenen Debugger irgendetwas sichtbar
        // wird (TU_BREAKPOINT() haelt nur an, wenn ein Debugger aktiv ist) -- der Stack blieb
        // dann unbemerkt dauerhaft nicht funktionsfaehig, "USB device stack initialized"
        // suggerierte aber faelschlich Erfolg.
        if (!tusb_init(0, NULL)) {
            log_error("USB device stack: tusb_init() fehlgeschlagen (VBUS present at boot)");
        } else {
            log_info("USB device stack initialized (VBUS present at boot)");
        }
    } else {
        log_info("USB device stack: kein Kabel/keine 5V an PC0/USB_VSENSE beim Boot -- Init folgt bei Anschluss");
    }
}

_Noreturn void usbd_device_loop(void (*poll_hook)(void), uint32_t (*get_task_timeout_ms)(void)) {
    bool vsense_last = (HAL_GPIO_ReadPin(USB_VSENSE_GPIO_Port, USB_VSENSE_Pin) == GPIO_PIN_SET);

    while (1) {
        bool vsense_now = (HAL_GPIO_ReadPin(USB_VSENSE_GPIO_Port, USB_VSENSE_Pin) == GPIO_PIN_SET);
        if (vsense_now != vsense_last) {
            if (vsense_now) {
                if (!tud_inited()) {
                    if (!tusb_init(0, NULL)) {
                        log_error("USB_VSENSE: 5V erkannt, tusb_init() fehlgeschlagen");
                    } else {
                        log_info("USB_VSENSE: 5V erkannt, USB-Stack initialisiert");
                    }
                } else {
                    tud_connect();
                    log_info("USB_VSENSE: 5V erkannt, USB-Stack verbunden");
                }
            } else {
                tud_disconnect();
                log_info("USB_VSENSE: keine 5V mehr, USB-Stack getrennt");
            }
            vsense_last = vsense_now;
        }

        if (tud_inited()) {
            // tud_task() ist nur tud_task_ext(UINT32_MAX, false) -- blockiert unbegrenzt, bis ein
            // ECHTES USB-Transfer-Ereignis eintrifft (kein SOF-Callback aktiv, also kein
            // periodischer Rueckkehrpunkt von Haus aus). get_task_timeout_ms() (s. usbd_device.h)
            // liefert nur dann ein kurzes Timeout, wenn tatsaechlich ein Antwortpaket wartet --
            // sonst UINT32_MAX, damit dieser Thread (Prioritaet 6, hoeher als io_thread mit 8) im
            // USB-Leerlauf nicht unnoetig oft dazwischenfunkt.
            uint32_t timeout_ms = get_task_timeout_ms ? get_task_timeout_ms() : UINT32_MAX;
            tud_task_ext(timeout_ms, false);
            if (poll_hook) {
                poll_hook();
            }
        } else {
            // BUG-Fix (Debugging-Sitzung): ohne dieses Sleep dreht diese Schleife bei fehlendem
            // Kabel (tud_inited()==false, tud_task() wird uebersprungen) ungebremst -- eine
            // Busy-Loop mit Prioritaet 6, hoeher als io_thread (8) und heartbeat_thread (12).
            // Dadurch verhungerten beide vollstaendig, komplett unabhaengig vom eigentlichen
            // USB-Interrupt-Verdacht (der sich als Sackgasse herausstellte). 10 Ticks (~100ms
            // bei TX_TIMER_TICKS_PER_SECOND=100) sind fuer die VSENSE-Reaktionszeit voellig
            // ausreichend und geben io_thread/heartbeat_thread reichlich Gelegenheit zu laufen.
            tx_thread_sleep(10);
        }
    }
}

// --- TinyUSB-Deskriptor-Callbacks (vom Stack aus dem Enumerationskontext aufgerufen) ---------

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&g_device_descriptor;
}

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;  // nur eine Konfiguration
    return g_config_descriptor;
}

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    if (index == STRIDX_LANGID) {
        g_string_desc_buf[0] = (uint16_t)((TUSB_DESC_STRING << 8) | 4);
        g_string_desc_buf[1] = 0x0409;  // en-US -- Windows zeigt iInterface-Strings
                                          // unabhaengig vom gewaehlten LangID korrekt an.
        return g_string_desc_buf;
    }

    char const* str;
    switch (index) {
        case STRIDX_MANUFACTURER: str = STRING_MANUFACTURER; break;
        case STRIDX_PRODUCT:      str = STRING_PRODUCT; break;
        case STRIDX_SERIAL:       str = g_serial_string; break;
        case STRIDX_CDC0_DEBUG:   str = STRING_CDC0_DEBUG; break;
        case STRIDX_CDC1_MODBUS:  str = STRING_CDC1_MODBUS; break;
        case STRIDX_NCM:          str = STRING_NCM; break;
        case STRIDX_NCM_MAC:      str = g_ncm_mac_string; break;
        default: return NULL;
    }

    size_t len = strlen(str);
    if (len > (sizeof(g_string_desc_buf) / 2) - 1) {
        len = (sizeof(g_string_desc_buf) / 2) - 1;
    }
    for (size_t i = 0; i < len; i++) {
        g_string_desc_buf[1 + i] = (uint16_t)str[i];  // ASCII -> UTF-16LE (keine Umlaute in den Strings)
    }
    g_string_desc_buf[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * (len + 1)));
    return g_string_desc_buf;
}
