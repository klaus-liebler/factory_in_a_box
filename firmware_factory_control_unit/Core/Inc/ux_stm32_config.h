#pragma once
// USBX-STM32-DCD-Konfiguration (libs/ST/usbx/common/usbx_stm32_device_controllers/ux_dcd_stm32.h)
// -- eingebunden von ux_dcd_stm32.h selbst (#include "ux_stm32_config.h"), analog zu tusb_config.h
// bei TinyUSB. Bewusst minimal: nur die beiden Werte, die ux_dcd_stm32.h nicht selbst schon
// sinnvoll vorbelegt (s. dortige #ifndef-Fallbacks).

#include "stm32h5xx_hal.h"

// UX_DCD_STM32_MAX_ED: Anzahl der vom DCD-Treiber verwalteten physischen Endpunkte -- muss zur
// tatsaechlichen Hardware passen (USB_DRD_FS: 8 Endpunkte inkl. EP0, s. main.c
// hpcd_USB_DRD_FS.Init.dev_endpoints=8). ux_dcd_stm32.h faellt sonst auf 4 zurueck, was bei
// unserem Composite-Geraet (EP0 + bis zu 6 weitere, s. usbd_device.c) zu kurz griffe.
#define UX_DCD_STM32_MAX_ED 8
