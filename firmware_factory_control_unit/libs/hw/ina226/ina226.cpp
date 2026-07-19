#include "ina226.hpp"
#include "log.h"

namespace ina226 {

namespace reg {
constexpr uint8_t CONFIG = 0x00;
constexpr uint8_t SHUNT_VOLTAGE = 0x01;
constexpr uint8_t BUS_VOLTAGE = 0x02;
constexpr uint8_t POWER = 0x03;
constexpr uint8_t CURRENT = 0x04;
constexpr uint8_t CALIBRATION = 0x05;
constexpr uint8_t MANUFACTURER_ID = 0xFE;
constexpr uint8_t DIE_ID = 0xFF;

constexpr uint16_t CONFIG_RESET = 0x8000;
// Reserved[14:12]=100b | AVG[11:9]=001b (4 Samples) | VBUSCT[8:6]=100b (1.1ms, Default) |
// VSHCT[5:3]=100b (1.1ms, Default) | MODE[2:0]=111b (Shunt+Bus, kontinuierlich).
constexpr uint16_t CONFIG_VALUE = 0x4327;

constexpr double SHUNT_VOLTAGE_LSB_UV = 2.5;
constexpr double BUS_VOLTAGE_LSB_MV = 1.25;
constexpr double CAL_CONSTANT = 0.00512;
constexpr uint16_t MANUFACTURER_ID_EXPECTED = 0x5449;
} // namespace reg

constexpr uint32_t I2C_TIMEOUT_MS = 20;

// TOF3 (VL53L0X am selben I2C4-Bus, s. Core/Src/setup_and_loops/tof_color.hh) ist auf dieser
// Platinen-Revision nicht bestueckt -- dessen fehlgeschlagener Probe()-Zugriff waehrend
// Io::Setup() (NACK auf eine nicht existierende Adresse) legt den I2C4-Bus fuer ALLE folgenden
// Transaktionen lahm, auch fuer dieses INA226, das mit dem eigentlichen TOF3-Fehler nichts zu
// tun hat -- bestaetigt per Test (tof_color_.Setup() vor power_.Setup() gezogen: INA226s eigenes
// Probe()/Init() schlug daraufhin ebenfalls sofort fehl). WICHTIG: HAL_I2C_GetState() meldet in
// diesem Zustand weiterhin READY -- ein Check-and-recover darauf (frueherer Versuch) griff nie.
// Der Fix ist daher ein BEDINGUNGSLOSER DeInit()+Init() einmalig beim Start von INA226::Probe()
// (dem allerersten Registerzugriff ueberhaupt), bevor irgendein Register angefasst wird. Io::Setup()
// ruft tof_color_.Setup() bewusst VOR power_.Setup() auf, damit dieser Reclaim den TOF3-Schaden
// sicher ueberschreibt und der Bus fuer die restliche Laufzeit sauber bleibt -- TOF3 fasst I2C4
// nach Setup() nie wieder an.
bool INA226::ReclaimI2CBus() {
    HAL_I2C_DeInit(hi2c_);
    return HAL_I2C_Init(hi2c_) == HAL_OK;
}

bool INA226::writeReg16(uint8_t reg, uint16_t value) {
    uint8_t buf[2] = {static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
    HAL_StatusTypeDef res = HAL_I2C_Mem_Write(hi2c_, addr8_, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT_MS);
    if (res != HAL_OK) {
        log_warn("INA226: writeReg16(0x%02X, 0x%04X) fehlgeschlagen - HAL-Status=%d I2C-ErrorCode=0x%08lX",
                  (unsigned int)reg, (unsigned int)value, (int)res, (unsigned long)hi2c_->ErrorCode);
        return false;
    }
    return true;
}

bool INA226::readReg16(uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    HAL_StatusTypeDef res = HAL_I2C_Mem_Read(hi2c_, addr8_, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT_MS);
    if (res != HAL_OK) {
        log_warn("INA226: readReg16(0x%02X) fehlgeschlagen - HAL-Status=%d I2C-ErrorCode=0x%08lX",
                  (unsigned int)reg, (int)res, (unsigned long)hi2c_->ErrorCode);
        return false;
    }
    // INA226: MSB zuerst (Big-Endian ueber I2C), anders als der TCS34725.
    *value = (static_cast<uint16_t>(buf[0]) << 8) | buf[1];
    return true;
}

bool INA226::Probe() {
    if (!ReclaimI2CBus()) {
        log_warn("INA226: I2C4-Reclaim (DeInit+Init) vor Probe() fehlgeschlagen");
        return false;
    }
    uint16_t id = 0;
    return readReg16(reg::MANUFACTURER_ID, &id) && id == reg::MANUFACTURER_ID_EXPECTED;
}

bool INA226::ReadDiagnosticIds(uint16_t *manufacturer_id, uint16_t *die_id) {
    // Nicht kurzschliessend -- beide Register unabhaengig voneinander versuchen (gleiches
    // Diagnose-Muster wie Read()).
    bool ok_mfg = readReg16(reg::MANUFACTURER_ID, manufacturer_id);
    bool ok_die = readReg16(reg::DIE_ID, die_id);
    return ok_mfg && ok_die;
}

bool INA226::Init() {
    // Current_LSB so gewaehlt, dass max_expected_current_ma_ nahe an den vollen 15 Bit des
    // Strom-Registers ausnutzt (Datenblatt-Empfehlung: Current_LSB = Max_Expected_Current / 2^15).
    current_lsb_ma_ = static_cast<double>(max_expected_current_ma_) / 32768.0;
    power_lsb_mw_ = 25.0 * current_lsb_ma_;

    const double current_lsb_a = current_lsb_ma_ / 1000.0;
    const double shunt_ohm = static_cast<double>(shunt_milliohm_) / 1000.0;
    const uint16_t cal = static_cast<uint16_t>(reg::CAL_CONSTANT / (current_lsb_a * shunt_ohm) + 0.5);

    if (!writeReg16(reg::CONFIG, reg::CONFIG_RESET)) {
        return false;
    }
    HAL_Delay(1); // kurze Erholzeit nach Reset (Datenblatt: max. 1 Wandlungszyklus)
    if (!(writeReg16(reg::CONFIG, reg::CONFIG_VALUE) &&
          writeReg16(reg::CALIBRATION, cal))) {
        return false;
    }

    // Kalibrierung zurücklesen und loggen -- der CURRENT-/POWER-Register-ALU des Chips liefert
    // ohne (oder mit versehentlich 0 geschriebener) Kalibrierung IMMER 0 zurueck, auch wenn
    // Shunt-/Busspannung genuin korrekt gemessen werden. Diese Zeile macht sichtbar, ob genau
    // das der Grund fuer eine dauerhaft 0 mA anzeigende Strommessung ist.
    uint16_t cal_readback = 0xFFFF;
    readReg16(reg::CALIBRATION, &cal_readback);
    log_info("INA226: Init OK, current_lsb=%ld.%03ld mA/bit, CAL geschrieben=%u zurueckgelesen=%u",
             (long)current_lsb_ma_, (long)((current_lsb_ma_ - (long)current_lsb_ma_) * 1000),
             (unsigned int)cal, (unsigned int)cal_readback);
    return true;
}

bool INA226::Read(Measurement *out) {
    // Bewusst NICHT kurzschliessend (kein "||"-Verkettung) -- fuer die Diagnose, ob ein
    // NACK/Timeout auf ein einzelnes Register beschraenkt ist oder alle vier gleichermassen
    // betrifft, sollen ALLE vier Register versucht werden, auch wenn eines schon fehlschlaegt.
    uint16_t raw_bus = 0, raw_shunt = 0, raw_current = 0, raw_power = 0;
    bool ok_bus = readReg16(reg::BUS_VOLTAGE, &raw_bus);
    bool ok_shunt = readReg16(reg::SHUNT_VOLTAGE, &raw_shunt);
    bool ok_current = readReg16(reg::CURRENT, &raw_current);
    bool ok_power = readReg16(reg::POWER, &raw_power);
    if (!ok_bus || !ok_shunt || !ok_current || !ok_power) {
        return false;
    }
    out->bus_voltage_mv = static_cast<int32_t>(raw_bus * reg::BUS_VOLTAGE_LSB_MV);
    out->shunt_voltage_uv = static_cast<int32_t>(static_cast<int16_t>(raw_shunt) * reg::SHUNT_VOLTAGE_LSB_UV);
    out->current_ma = static_cast<int32_t>(static_cast<int16_t>(raw_current) * current_lsb_ma_);
    out->power_mw = static_cast<uint32_t>(raw_power * power_lsb_mw_);
    log_debug("INA226: raw bus=0x%04X shunt=0x%04X current=0x%04X power=0x%04X -> %ld mV, %ld uV, %ld mA, %lu mW",
              (unsigned int)raw_bus, (unsigned int)raw_shunt, (unsigned int)raw_current, (unsigned int)raw_power,
              (long)out->bus_voltage_mv, (long)out->shunt_voltage_uv, (long)out->current_ma, (unsigned long)out->power_mw);
    return true;
}

} // namespace ina226
