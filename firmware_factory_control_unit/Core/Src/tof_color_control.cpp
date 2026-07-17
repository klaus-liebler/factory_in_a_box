// ============================================================================
// ToF/Color -- 3x VL53L0X (TOF1..3, je auf eigenem I2C-Bus: I2C1/I2C2/I2C4) + 1x TCS34725
// (Farbsensor, I2C1).
//
// ANNAHME (auf Nutzerwunsch so umgesetzt, mangels Schaltplan-Eintrag fuer diese Sensoren und
// ohne verdrahteten XSHUT-Pin, um TOF1 zur Laufzeit umzuadressieren): TOF1 und der Farbsensor
// haengen beide an I2C1 und benutzen werksseitig beide Adresse 0x29 -- das wuerde ohne
// Aenderung kollidieren. Es wird angenommen, dass das TOF1-Breakout-Modul ab Werk/per
// Loetbruecke bereits auf eine andere Adresse (hier: 0x30) eingestellt ist. Antwortet unter
// dieser Adresse nichts, bleibt TOF1_STATUS dauerhaft auf 0 (nicht erkannt) -- das ist dann
// ein Hinweis, dass diese Annahme fuer die tatsaechlich verbaute Sensor-Charge nicht stimmt
// und stattdessen doch ein XSHUT-Pin (oder ein I2C-Multiplexer) noetig waere.
// ============================================================================
#include "tof_color_control.hpp"
#include "app_state.hpp"
#include "modbus_register_map.hpp"
#include "log.h"

#include "vl53l0x.hpp"
#include "tcs34725.hpp"

extern "C" I2C_HandleTypeDef hi2c1;
extern "C" I2C_HandleTypeDef hi2c2;
extern "C" I2C_HandleTypeDef hi2c4;

constexpr uint8_t TOF1_ASSUMED_ALT_ADDRESS = 0x30;

static vl53l0x::VL53L0X tof1(&hi2c1, TOF1_ASSUMED_ALT_ADDRESS);
static vl53l0x::VL53L0X tof2(&hi2c2);
static vl53l0x::VL53L0X tof3(&hi2c4);
static tcs34725::TCS34725 color_sensor(&hi2c1);

static bool s_tof1_present = false;
static bool s_tof2_present = false;
static bool s_tof3_present = false;
static bool s_color_present = false;

void tof_color_setup() {
    s_tof1_present = tof1.Probe();
    if (!s_tof1_present) {
        log_warn("TOF1 (VL53L0X @0x%02X on I2C1) not detected", TOF1_ASSUMED_ALT_ADDRESS);
    }
    s_tof2_present = tof2.Probe();
    if (!s_tof2_present) {
        log_warn("TOF2 (VL53L0X on I2C2) not detected");
    }
    s_tof3_present = tof3.Probe();
    if (!s_tof3_present) {
        log_warn("TOF3 (VL53L0X on I2C4) not detected");
    }
    s_color_present = color_sensor.Init();
    if (!s_color_present) {
        log_warn("Color sensor (TCS34725 on I2C1) not detected/init failed");
    }
}

void tof_color_update() {
    ModbusTcpServer& server = *g_app_state.modbus_server;

    uint16_t mm;
    if (s_tof1_present && tof1.ReadRangeMm(&mm)) {
        server.write_input_register(ModbusRegisters::Input::TOF1_DISTANCE_MM, mm);
        server.write_input_register(ModbusRegisters::Input::TOF1_STATUS, 1);
    } else {
        server.write_input_register(ModbusRegisters::Input::TOF1_STATUS, 0);
    }

    if (s_tof2_present && tof2.ReadRangeMm(&mm)) {
        server.write_input_register(ModbusRegisters::Input::TOF2_DISTANCE_MM, mm);
        server.write_input_register(ModbusRegisters::Input::TOF2_STATUS, 1);
    } else {
        server.write_input_register(ModbusRegisters::Input::TOF2_STATUS, 0);
    }

    if (s_tof3_present && tof3.ReadRangeMm(&mm)) {
        server.write_input_register(ModbusRegisters::Input::TOF3_DISTANCE_MM, mm);
        server.write_input_register(ModbusRegisters::Input::TOF3_STATUS, 1);
    } else {
        server.write_input_register(ModbusRegisters::Input::TOF3_STATUS, 0);
    }

    if (s_color_present) {
        tcs34725::RawColor color;
        if (color_sensor.Read(&color)) {
            server.write_input_register(ModbusRegisters::Input::COLOR_CLEAR, color.clear);
            server.write_input_register(ModbusRegisters::Input::COLOR_RED, color.red);
            server.write_input_register(ModbusRegisters::Input::COLOR_GREEN, color.green);
            server.write_input_register(ModbusRegisters::Input::COLOR_BLUE, color.blue);
        }
    }
}
