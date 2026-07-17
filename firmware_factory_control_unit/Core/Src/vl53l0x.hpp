#pragma once
// Minimaler VL53L0X-ToF-Treiber (I2C, Einzelmessung/Polling). Nutzt bewusst NUR den oeffentlich
// dokumentierten Kern-Registersatz (Identifikation, SYSRANGE_START, RESULT_*,
// SYSTEM_INTERRUPT_CLEAR) OHNE die von ST empfohlene volle Kalibrierungs-/Tuning-Sequenz
// (SPAD-Enable, VHV/Phase-Kalibrierung, ~50 "Magic"-Tuning-Register aus der offiziellen API) --
// die Werkskalibrierung im NVM des Chips wird als ausreichend fuer eine grobe
// Abstandsmessung angenommen. Fuer produktionsreife Genauigkeit muesste die volle ST-API-
// Init-Sequenz nachgezogen werden.
//
// Adressierung: Standardadresse 0x29 (7-bit), sofern nicht per address-Parameter override
// -- wird fuer TOF1 gebraucht, siehe tof_color_thread.cpp fuer die Begruendung
// (Adresskonflikt mit dem Farbsensor auf demselben I2C-Bus, kein XSHUT-Pin verdrahtet).
#include <cstdint>
#include "main.h"

namespace vl53l0x {

class VL53L0X {
public:
    explicit VL53L0X(I2C_HandleTypeDef *hi2c, uint8_t address_7bit = 0x29)
        : hi2c_(hi2c), addr8_((uint8_t)(address_7bit << 1)) {}

    // Prueft IDENTIFICATION_MODEL_ID (0xC0, muss 0xEE liefern). Bei Erfolg ist der Sensor auf
    // dem Bus vorhanden und antwortet unter der konfigurierten Adresse.
    bool Probe();

    // Stoesst eine Einzelmessung an, wartet (blockierend, mit Timeout) auf das Ergebnis und
    // liefert die Distanz in mm. Rueckgabe false bei Timeout/I2C-Fehler.
    bool ReadRangeMm(uint16_t *out_mm, uint32_t timeout_ms = 50);

private:
    I2C_HandleTypeDef *hi2c_;
    uint8_t addr8_;

    bool writeReg8(uint8_t reg, uint8_t value);
    bool readReg8(uint8_t reg, uint8_t *value);
    bool readReg16(uint8_t reg, uint16_t *value);
};

} // namespace vl53l0x
