#include "vl53l0x.hpp"

namespace vl53l0x {

namespace reg {
constexpr uint8_t SYSRANGE_START = 0x00;
constexpr uint8_t SYSTEM_INTERRUPT_CLEAR = 0x0B;
constexpr uint8_t RESULT_INTERRUPT_STATUS = 0x13;
constexpr uint8_t RESULT_RANGE_STATUS = 0x14;
constexpr uint8_t RESULT_RANGE_MM = 0x1E; // RESULT_RANGE_STATUS + 10, 16-bit big-endian
constexpr uint8_t IDENTIFICATION_MODEL_ID = 0xC0;
constexpr uint8_t IDENTIFICATION_MODEL_ID_EXPECTED = 0xEE;
} // namespace reg

constexpr uint32_t I2C_TIMEOUT_MS = 20;

bool VL53L0X::writeReg8(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(hi2c_, addr8_, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

bool VL53L0X::readReg8(uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(hi2c_, addr8_, reg, I2C_MEMADD_SIZE_8BIT, value, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

bool VL53L0X::readReg16(uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(hi2c_, addr8_, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

bool VL53L0X::Probe() {
    uint8_t model_id = 0;
    if (!readReg8(reg::IDENTIFICATION_MODEL_ID, &model_id)) {
        return false;
    }
    return model_id == reg::IDENTIFICATION_MODEL_ID_EXPECTED;
}

bool VL53L0X::ReadRangeMm(uint16_t *out_mm, uint32_t timeout_ms) {
    if (!writeReg8(reg::SYSRANGE_START, 0x01)) {
        return false;
    }

    uint32_t elapsed = 0;
    uint8_t interrupt_status = 0;
    while (elapsed < timeout_ms) {
        if (!readReg8(reg::RESULT_INTERRUPT_STATUS, &interrupt_status)) {
            return false;
        }
        if ((interrupt_status & 0x07) != 0) {
            break;
        }
        HAL_Delay(1);
        elapsed++;
    }
    if ((interrupt_status & 0x07) == 0) {
        return false; // Timeout
    }

    uint16_t range_mm = 0;
    bool ok = readReg16(reg::RESULT_RANGE_MM, &range_mm);
    writeReg8(reg::SYSTEM_INTERRUPT_CLEAR, 0x01);

    if (!ok) {
        return false;
    }
    *out_mm = range_mm;
    return true;
}

} // namespace vl53l0x
