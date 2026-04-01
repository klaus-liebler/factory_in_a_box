#pragma once

#include <cstdint>
#include "stm32g4xx_hal.h"
#include "gpio.hh"
#include "tmc2209_reg.hpp"

namespace tmc2209 {
/*
17HS4401S:
    - Step angle: 1.8 degrees (200 steps/rev)
    - max current: 1.7A
    - PhaseVoltage: 2.6V
    - Resistance: 1.5 Ohm
    - Inductance: 2.8 mH
    - Holding torque: 43 Ncm

*/

enum class MicroStepResolution { RES256=0b0000, RES128=0b0001, RES64=0b0010, RES32=0b0011, RES16=0b0100, RES8=0b0101, RES4=0b0110, RES2=0b0111, RES_FULL_STEP=0b1000 };


struct TuningResults{
    uint32_t pwm_grad;
    uint32_t pwm_ofs;
    uint32_t pwm_scale;
    uint32_t pwm_auto;
};

class TMC2209 {
public:
    // address: 0..15 (MSB used for read flag internally)
    // Optional EN pin: pass Pin::NO_PIN to disable GPIO control
    TMC2209(UART_HandleTypeDef* huart, uint8_t address, gpio::Pin enPin = gpio::Pin::NO_PIN);

    bool InitForNormalSpeedAndUartBasedOperation(bool usePotentiometerForCurrentScaling = false, bool disable_read = false,  MicroStepResolution resolution = MicroStepResolution::RES256);
    bool ShaftTurnReversed(bool reversed);
    void PrintPrettyFullSystemState();
    bool PerformStealthChopAutoTuningForQuietOperation();

   

    // Generate steps at specified speed in full steps per second. A common stepper motor with 200 steps/rev
    // will rotate at with 2 Rotations per second when set to value "400"
    bool GenerateSteps(int fullStepsPerSecond);

    // Convenience APIs
    void enable();
    void disable();

    // IHOLD_IRUN fields: ihold/run in 0..31, iholddelay 0..15
    bool setIHoldIRun(uint8_t ihold, uint8_t irun, uint8_t iholddelay);

private:
    UART_HandleTypeDef* huart_;
    uint8_t addr_; // 0..15
    gpio::Pin en_;
    REG_FIELD::GCONF gconf;
    REG_FIELD::CHOPCONF chopconf;

    bool fetchImportantRegistersForLocalMirroring();
    bool clearGStat(const char* error_msg);
    bool writeRegister(RegIdx reg, uint32_t value, uint32_t timeoutMs = 200);
    bool readRegister(RegIdx reg, uint32_t& value, uint32_t timeoutMs = 200, bool checkStartByte = true);

    bool checkedWriteRegister(RegIdx reg, uint32_t value, const char* error_msg);                                   
    bool checkedReadRegister(RegIdx reg, uint32_t& value, const char* error_msg);

    static uint8_t crc8(uint8_t* datagram, size_t datagram_size_with_crc);
};

} // namespace tmc2209
