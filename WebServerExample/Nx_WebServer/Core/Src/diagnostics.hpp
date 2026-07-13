#pragma once

#include "stm32h5xx_hal.h"
#include "modbus_register_map.hpp"
#include "modbus_tcp_server.hpp"

// Diagnostik-Bloecke des Register-Maps: Chip-ID/Version werden einmalig beim
// Start befuellt, HealthState wird zyklisch aus dem I/O-Thread aktualisiert.

namespace Diagnostics {

constexpr uint16_t FW_VERSION_MAJOR = 0;
constexpr uint16_t FW_VERSION_MINOR = 1;
constexpr uint16_t FW_VERSION_PATCH = 0;

inline void write_startup_registers(ModbusTcpServer& server) {
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();

    server.write_input_register(ModbusRegisters::Input::CHIP_ID_W0_HI, (uint16_t)(uid0 >> 16));
    server.write_input_register(ModbusRegisters::Input::CHIP_ID_W0_LO, (uint16_t)(uid0 & 0xFFFF));
    server.write_input_register(ModbusRegisters::Input::CHIP_ID_W1_HI, (uint16_t)(uid1 >> 16));
    server.write_input_register(ModbusRegisters::Input::CHIP_ID_W1_LO, (uint16_t)(uid1 & 0xFFFF));
    server.write_input_register(ModbusRegisters::Input::CHIP_ID_W2_HI, (uint16_t)(uid2 >> 16));
    server.write_input_register(ModbusRegisters::Input::CHIP_ID_W2_LO, (uint16_t)(uid2 & 0xFFFF));

    server.write_input_register(ModbusRegisters::Input::FW_VERSION_MAJOR, FW_VERSION_MAJOR);
    server.write_input_register(ModbusRegisters::Input::FW_VERSION_MINOR, FW_VERSION_MINOR);
    server.write_input_register(ModbusRegisters::Input::FW_VERSION_PATCH, FW_VERSION_PATCH);
}

// Wird zyklisch aus dem I/O-Thread aufgerufen. eth_link_up kommt aus der
// NetXDuo-Link-Status-Abfrage (siehe app.cpp / link_thread_entry).
inline void update_health_state(ModbusTcpServer& server, bool eth_link_up) {
    uint16_t health = (1u << ModbusRegisters::HealthBit::OK);
    if (!eth_link_up) {
        health |= (1u << ModbusRegisters::HealthBit::ETH_LINK_DOWN);
        health &= ~(1u << ModbusRegisters::HealthBit::OK);
    }
    server.write_input_register(ModbusRegisters::Input::HEALTH_STATE, health);
}

} // namespace Diagnostics
