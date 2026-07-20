#pragma once
// TinyUSB-Konfiguration fuer das FactoryControl-Composite-Device (STM32H563, USB_DRD_FS,
// Full-Speed-only, 8 Hardware-Endpunkte). CFG_TUSB_MCU/CFG_TUSB_OS werden bereits als PUBLIC
// compile definition von stm32_libs/tinyusb_port/CMakeLists.txt gesetzt.
//
// Stufenweiser Ausbau (s. Implementierungsplan): aktuell nur CDC "FactoryControl Debug"
// aktiviert. CFG_TUD_CDC/CFG_TUD_MSC/CFG_TUD_NCM werden mit den weiteren Stufen erhoeht bzw.
// aktiviert -- absichtlich nicht vorab alle vier Funktionen auf einmal freigeschaltet, um jede
// Stufe einzeln bauen/testen zu koennen.

#define CFG_TUSB_RHPORT0_MODE      (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_ENABLED            1
#define CFG_TUD_MAX_SPEED          OPT_MODE_FULL_SPEED
#define CFG_TUD_ENDPOINT0_SIZE     64

// --- Device-Klassen ---
// Stufe 1: nur die Debug-CDC-Schnittstelle. Modbus-CDC/NCM/MSC folgen in ihren jeweiligen
// Implementierungsstufen (dann CFG_TUD_CDC=2, CFG_TUD_NCM=1, CFG_TUD_MSC=1).
#define CFG_TUD_CDC                1
#define CFG_TUD_MSC                0
#define CFG_TUD_NCM                0
#define CFG_TUD_ECM_RNDIS          0

// CDC FIFO-Groessen (Full-Speed-Bulk-Endpunktgroesse 64 Byte, s. Endpoint-Budget-Analyse) --
// Standardwerte reichen, hier nur explizit dokumentiert.
#define CFG_TUD_CDC_RX_BUFSIZE     256
#define CFG_TUD_CDC_TX_BUFSIZE     256
#define CFG_TUD_CDC_EP_BUFSIZE     64

// Kein malloc/free im Stack -- TinyUSB kommt hier ganz ohne dynamische Allokation aus.
