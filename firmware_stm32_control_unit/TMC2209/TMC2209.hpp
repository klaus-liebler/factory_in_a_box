#pragma once

#include <cstdint>
#include "stm32g4xx_hal.h"
#include "gpio.hh"
#include "tmc2209_reg.hpp"

namespace tmc2209 {


class TMC2209 {
public:
    // address: 0..15 (MSB used for read flag internally)
    // Optional EN pin: pass Pin::NO_PIN to disable GPIO control
    TMC2209(UART_HandleTypeDef* huart, uint8_t address, gpio::Pin enPin = gpio::Pin::NO_PIN);

    bool writeRegister(Reg reg, uint32_t value, uint32_t timeoutMs = 200);
    bool readRegister(Reg reg, uint32_t& value, uint32_t timeoutMs = 200);

    // Convenience APIs
    void enable();
    void disable();

    // IHOLD_IRUN fields: ihold/run in 0..31, iholddelay 0..15
    bool setIHoldIRun(uint8_t ihold, uint8_t irun, uint8_t iholddelay);

private:
    UART_HandleTypeDef* huart_;
    uint8_t addr_; // 0..15
    gpio::Pin en_;

    static uint8_t crc8(uint8_t* datagram, size_t datagram_size_with_crc);
};

} // namespace tmc2209
