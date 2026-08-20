#pragma once
// littlefs-Blockgeraet-Anbindung fuer den reservierten 256K-Flash-Bereich am Ende des Flashs
// (s. STM32H563xx_FLASH.ld, _roarm_mission_flash_start) -- implementiert die vier von littlefs
// verlangten Callbacks (read/prog/erase/sync) direkt ueber HAL_FLASH_Program/HAL_FLASHEx_Erase.
// Sektorgroesse (8K) und Quad-Word-Programmiergranularitaet (16 Byte) kommen von der
// STM32H5-Flash-Hardware (stm32h563xx.h/stm32h5xx_hal_flash.h), nicht frei waehlbar.
//
// TrustZone: dieses Board laeuft mit TZEN=0 (deaktiviert, s. Kommentar in Core/Src/app.cc zu
// STM32H573I-DK/Nx_WebServer) -- Erase nutzt daher die nicht-"_NS"-Konstanten
// (FLASH_TYPEERASE_SECTORS statt FLASH_TYPEERASE_SECTORS_NS). Falls TrustZone auf der realen
// Platine doch aktiv sein sollte, muss das beim Bring-up angepasst werden (HAL_FLASHEx_Erase
// schlaegt sonst mit HAL_ERROR fehl, kein stiller Fehlzustand).
#include "lfs.h"

namespace RoArmMissionFlashBd {

// Muss vor lfs_mount()/lfs_format() genau einmal aufgerufen werden (fuellt 'config' mit den
// Callback-Zeigern und Geometrie-Werten). 'config' muss danach fuer die gesamte Lebensdauer des
// gemounteten Dateisystems bestehen bleiben (lfs_t haelt nur einen Zeiger darauf).
void InitConfig(lfs_config& config);

} // namespace RoArmMissionFlashBd
