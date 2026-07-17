#include "tcs34725.hpp"

namespace tcs34725 {

namespace reg {
constexpr uint8_t COMMAND_BIT = 0x80;
constexpr uint8_t ENABLE = 0x00;
constexpr uint8_t ATIME = 0x01;
constexpr uint8_t CONTROL = 0x0F;
constexpr uint8_t ID = 0x12;
constexpr uint8_t CDATAL = 0x14;
constexpr uint8_t RDATAL = 0x16;
constexpr uint8_t GDATAL = 0x18;
constexpr uint8_t BDATAL = 0x1A;

constexpr uint8_t ENABLE_PON = 0x01;
constexpr uint8_t ENABLE_AEN = 0x02;
constexpr uint8_t GAIN_4X = 0x01;
constexpr uint8_t ID_EXPECTED = 0x44;
} // namespace reg

constexpr uint32_t I2C_TIMEOUT_MS = 20;

bool TCS34725::writeReg8(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(hi2c_, ADDR8, reg::COMMAND_BIT | reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

bool TCS34725::readReg8(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(hi2c_, ADDR8, reg::COMMAND_BIT | reg, I2C_MEMADD_SIZE_8BIT, value, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

bool TCS34725::readReg16(uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(hi2c_, ADDR8, reg::COMMAND_BIT | reg, I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    // TCS34725: Low-Byte zuerst (Little-Endian ueber I2C).
    *value = ((uint16_t)buf[1] << 8) | buf[0];
    return true;
}

bool TCS34725::Init() {
    uint8_t id = 0;
    if (!readReg8(reg::ID, &id) || id != reg::ID_EXPECTED) {
        return false;
    }
    if (!writeReg8(reg::ATIME, 0xC0)) { // ~101ms Integrationszeit
        return false;
    }
    if (!writeReg8(reg::CONTROL, reg::GAIN_4X)) {
        return false;
    }
    if (!writeReg8(reg::ENABLE, reg::ENABLE_PON)) {
        return false;
    }
    HAL_Delay(3); // PON muss vor AEN mindestens 2.4ms stehen (Datenblatt)
    if (!writeReg8(reg::ENABLE, reg::ENABLE_PON | reg::ENABLE_AEN)) {
        return false;
    }
    return true;
}

bool TCS34725::Read(RawColor *out) {
    return readReg16(reg::CDATAL, &out->clear) &&
           readReg16(reg::RDATAL, &out->red) &&
           readReg16(reg::GDATAL, &out->green) &&
           readReg16(reg::BDATAL, &out->blue);
}

} // namespace tcs34725
