#pragma once
// TinyUSB-Konfiguration fuer das FactoryControl-Composite-Device (STM32H563, USB_DRD_FS,
// Full-Speed-only, 8 Hardware-Endpunkte). CFG_TUSB_MCU/CFG_TUSB_OS werden bereits als PUBLIC
// compile definition von stm32_libs/tinyusb_port/CMakeLists.txt gesetzt.
//
// Stufenweiser Ausbau (s. Implementierungsplan): aktuell CDC "FactoryControl Debug" + CDC
// "FactoryControl Modbus" (Stufe 2). NCM/MSC folgen in ihren jeweiligen Implementierungsstufen
// (dann CFG_TUD_NCM=1, CFG_TUD_MSC=1) -- absichtlich nicht vorab alle vier Funktionen auf einmal
// freigeschaltet, um jede Stufe einzeln bauen/testen zu koennen.

#define CFG_TUSB_RHPORT0_MODE      (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_ENABLED            1
#define CFG_TUD_MAX_SPEED          OPT_MODE_FULL_SPEED
#define CFG_TUD_ENDPOINT0_SIZE     64

// --- Device-Klassen ---
// Stufe 2: zwei CDC-Schnittstellen (Debug + Modbus-RTU). NCM/MSC folgen in ihren jeweiligen
// Implementierungsstufen (dann CFG_TUD_NCM=1, CFG_TUD_MSC=1).
#define CFG_TUD_CDC                2
#define CFG_TUD_MSC                0
#define CFG_TUD_NCM                0
#define CFG_TUD_ECM_RNDIS          0

// CDC FIFO-Groessen (Full-Speed-Bulk-Endpunktgroesse 64 Byte, s. Endpoint-Budget-Analyse) --
// Standardwerte reichen, hier nur explizit dokumentiert.
#define CFG_TUD_CDC_RX_BUFSIZE     256
#define CFG_TUD_CDC_TX_BUFSIZE     256
#define CFG_TUD_CDC_EP_BUFSIZE     64

// Kein malloc/free im Stack -- TinyUSB kommt hier ganz ohne dynamische Allokation aus.
