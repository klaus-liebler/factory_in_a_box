#pragma once
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
#include "interfaces.hh"
#include "modbus_register_model.hh"
#include "log.h"

#include "vl53l0x.hpp"
#include "tcs34725.hpp"

extern "C" I2C_HandleTypeDef hi2c1;
extern "C" I2C_HandleTypeDef hi2c2;
extern "C" I2C_HandleTypeDef hi2c4;

class TofColorSetupAndLoop : public ISetupAndLoop {
    private:
    static constexpr uint8_t TOF1_ASSUMED_ALT_ADDRESS = 0x30;

    Modbus::IModbusRegisterModel& register_model;
    vl53l0x::VL53L0X tof1{&hi2c1, TOF1_ASSUMED_ALT_ADDRESS};
    vl53l0x::VL53L0X tof2{&hi2c2};
    vl53l0x::VL53L0X tof3{&hi2c4};
    tcs34725::TCS34725 color_sensor{&hi2c1};

    bool tof1_present_ = false;
    bool tof2_present_ = false;
    bool tof3_present_ = false;
    bool color_present_ = false;

    public:
    TofColorSetupAndLoop(Modbus::IModbusRegisterModel& model) : register_model(model) {}

    void Setup() override {
        tof1_present_ = tof1.Probe();
        if (!tof1_present_) {
            log_warn("TOF1 (VL53L0X @0x%02X on I2C1) not detected", TOF1_ASSUMED_ALT_ADDRESS);
        }
        tof2_present_ = tof2.Probe();
        if (!tof2_present_) {
            log_warn("TOF2 (VL53L0X on I2C2) not detected");
        }
        tof3_present_ = tof3.Probe();
        if (!tof3_present_) {
            log_warn("TOF3 (VL53L0X on I2C4) not detected");
        }
        color_present_ = color_sensor.Init();
        if (!color_present_) {
            log_warn("Color sensor (TCS34725 on I2C1) not detected/init failed");
        }
    }

    void Loop(uint32_t now) override {
        (void)now;
        uint16_t mm;
        if (tof1_present_ && tof1.ReadRangeMm(&mm)) {
            register_model.SetInputRegister(ModbusRegisters::Input::TOF1_DISTANCE_MM, mm);
            register_model.SetInputRegister(ModbusRegisters::Input::TOF1_STATUS, 1);
        } else {
            register_model.SetInputRegister(ModbusRegisters::Input::TOF1_STATUS, 0);
        }

        if (tof2_present_ && tof2.ReadRangeMm(&mm)) {
            register_model.SetInputRegister(ModbusRegisters::Input::TOF2_DISTANCE_MM, mm);
            register_model.SetInputRegister(ModbusRegisters::Input::TOF2_STATUS, 1);
        } else {
            register_model.SetInputRegister(ModbusRegisters::Input::TOF2_STATUS, 0);
        }

        if (tof3_present_ && tof3.ReadRangeMm(&mm)) {
            register_model.SetInputRegister(ModbusRegisters::Input::TOF3_DISTANCE_MM, mm);
            register_model.SetInputRegister(ModbusRegisters::Input::TOF3_STATUS, 1);
        } else {
            register_model.SetInputRegister(ModbusRegisters::Input::TOF3_STATUS, 0);
        }

        if (color_present_) {
            tcs34725::RawColor color;
            if (color_sensor.Read(&color)) {
                register_model.SetInputRegister(ModbusRegisters::Input::COLOR_CLEAR, color.clear);
                register_model.SetInputRegister(ModbusRegisters::Input::COLOR_RED, color.red);
                register_model.SetInputRegister(ModbusRegisters::Input::COLOR_GREEN, color.green);
                register_model.SetInputRegister(ModbusRegisters::Input::COLOR_BLUE, color.blue);
            }
        }
    }
};
