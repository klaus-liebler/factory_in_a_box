#pragma once
// Minimaler TCS34725-Farbsensor-Treiber (I2C, feste Standardadresse 0x29 -- der Chip hat
// keinen Adress-Pin, daher immer 0x29, anders als beim VL53L0X).
#include <cstdint>
#include "main.h"

namespace tcs34725 {

struct RawColor {
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
};

class TCS34725 {
public:
    explicit TCS34725(I2C_HandleTypeDef *hi2c) : hi2c_(hi2c) {}

    // Prueft die ID (0x12, muss 0x44 fuer TCS34725 liefern) und aktiviert Power+ADC
    // (ENABLE = PON|AEN) mit einer festen Integrationszeit (~101ms, ATIME=0xC0) und 4x Gain.
    bool Init();

    // Liest die vier 16-bit-Kanaele (Clear/Red/Green/Blue). Rueckgabe false bei I2C-Fehler.
    bool Read(RawColor *out);

private:
    I2C_HandleTypeDef *hi2c_;
    static constexpr uint8_t ADDR8 = 0x29 << 1;

    bool writeReg8(uint8_t reg, uint8_t value);
    bool readReg8(uint8_t reg, uint8_t *value);
    bool readReg16(uint8_t reg, uint16_t *value);
};

} // namespace tcs34725
