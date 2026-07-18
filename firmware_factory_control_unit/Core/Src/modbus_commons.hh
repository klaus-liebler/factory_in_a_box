#pragma once
#include <cstdint>

namespace Modbus {

class IModbusRegisterModel {
public:
    virtual ~IModbusRegisterModel() = default;
    virtual uint16_t GetInputRegister(uint16_t addr) const = 0;
    virtual uint16_t GetHoldingRegister(uint16_t addr) const = 0;
    virtual void SetInputRegister(uint16_t addr, uint16_t value) = 0;
    virtual void SetHoldingRegister(uint16_t addr, uint16_t value) = 0;
};
} // namespace Modbus