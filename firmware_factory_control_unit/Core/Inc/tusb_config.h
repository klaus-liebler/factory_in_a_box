#pragma once
// TinyUSB-Konfiguration fuer das FactoryControl-Composite-Device (STM32H563, USB_DRD_FS,
// Full-Speed-only, 8 Hardware-Endpunkte). CFG_TUSB_MCU/CFG_TUSB_OS werden bereits als PUBLIC
// compile definition von stm32_libs/tinyusb_port/CMakeLists.txt gesetzt.
//
// Stufenweiser Ausbau (s. Implementierungsplan): aktuell CDC "FactoryControl Debug" + CDC
// "FactoryControl Modbus" (Stufe 2) + CDC-NCM "FactoryControl Network" (Stufe 3). MSC folgt in
// Stufe 4 (dann CFG_TUD_MSC=1) -- absichtlich nicht vorab alle vier Funktionen auf einmal
// freigeschaltet, um jede Stufe einzeln bauen/testen zu koennen.

#define CFG_TUSB_RHPORT0_MODE      (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_ENABLED            1
#define CFG_TUD_MAX_SPEED          OPT_MODE_FULL_SPEED
#define CFG_TUD_ENDPOINT0_SIZE     64

// --- Device-Klassen ---
// Stufe 3: zwei CDC-Schnittstellen (Debug + Modbus-RTU) + CDC-NCM. MSC folgt in Stufe 4 (dann
// CFG_TUD_MSC=1).
#define CFG_TUD_CDC                2
#define CFG_TUD_MSC                0
#define CFG_TUD_NCM                1
#define CFG_TUD_ECM_RNDIS          0

// CDC FIFO-Groessen (Full-Speed-Bulk-Endpunktgroesse 64 Byte, s. Endpoint-Budget-Analyse) --
// Standardwerte reichen, hier nur explizit dokumentiert.
#define CFG_TUD_CDC_RX_BUFSIZE     256
#define CFG_TUD_CDC_TX_BUFSIZE     256
#define CFG_TUD_CDC_EP_BUFSIZE     64

// --- CDC-NCM ---
// CDC-NCM 1.0 Tabelle 6-4 verlangt mindestens 2048 Byte NTB-Groesse -- dieses virtuelle Interface
// dient nur der Konfig-Weboberflaeche (kein Bulk-Datentransfer), daher bewusst am Minimum
// belassen statt der TinyUSB-Default-3200-Byte-Puffer (spart RAM, s. ncm.h).
#define CFG_TUD_NCM_IN_NTB_MAX_SIZE   2048
#define CFG_TUD_NCM_OUT_NTB_MAX_SIZE  2048
#define CFG_TUD_NCM_IN_NTB_N          1
#define CFG_TUD_NCM_OUT_NTB_N         1

// Kein malloc/free im Stack -- TinyUSB kommt hier ganz ohne dynamische Allokation aus.
