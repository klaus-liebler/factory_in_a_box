#pragma once
// ============================================================================
// Power -- INA226 Strom-/Spannungsmesser an I2C4, 2 mOhm Shunt, A0/A1 auf GND (Adresse 0x40).
// Kein Alerting (Mask/Enable, Alert-Limit, ALERT-Pin PD4) -- bewusst entfernt (s. History:
// Verdacht, dass diese Schreibvorgaenge die Mess-Register-Reads (0x01-0x04) dauerhaft NACKen
// liessen).
//
// WICHTIG fuer Io::Setup()-Reihenfolge (s. io.cpp): PowerSetupAndLoop::Setup() MUSS NACH
// TofColorSetupAndLoop::Setup() laufen. TOF3 (ebenfalls an I2C4, nicht bestueckt) legt den Bus
// per NACK lahm; INA226::Probe() erzwingt deshalb einen I2C4-Reclaim (DeInit+Init) als
// allerersten Schritt -- das hilft aber nichts, wenn TOF3 danach noch auf denselben Bus zugreift.
// ============================================================================
#include "interfaces.hh"
#include "modbus_register_model.hh"
#include "log.h"

#include "ina226.hpp"

extern "C" I2C_HandleTypeDef hi2c4;

class PowerSetupAndLoop : public ISetupAndLoop {
    private:
    static constexpr double SHUNT_MILLIOHM = 10.0; // 10 mOhm Shunt in Zukunft!
    static constexpr uint32_t MAX_EXPECTED_CURRENT_MA = 8000;

    Modbus::IModbusRegisterModel& register_model;
    ina226::INA226 power_monitor{&hi2c4, SHUNT_MILLIOHM, MAX_EXPECTED_CURRENT_MA};
    bool present_ = false;
    bool logged_first_reading_ = false;
    uint32_t consecutive_read_failures_ = 0;

    public:
    PowerSetupAndLoop(Modbus::IModbusRegisterModel& model) : register_model(model) {}

    void Setup() override {
        present_ = power_monitor.Probe() && power_monitor.Init();
        if (!present_) {
            log_warn("INA226 (I2C4, addr 0x40) not detected/init failed");
        }
    }

    void Loop(uint32_t now) override {
        (void)now;


        // Fehlercode-Konvention (s. register-map.json): 0 = ok, ungleich 0 = Fehler.
        register_model.SetInputRegister(ModbusRegisters::Input::PWR_STATUS, present_ ? 0 : 1);

        if (!present_) {
            return;
        }
        ina226::Measurement m;
        if (power_monitor.Read(&m)) {
            consecutive_read_failures_ = 0;
            if (!logged_first_reading_) {
                logged_first_reading_ = true;
                log_info("INA226: erste Messung -- bus=%ld mV, shunt=%ld uV, current=%ld mA, power=%lu mW",
                          (long)m.bus_voltage_mv, (long)m.shunt_voltage_uv, (long)m.current_ma,
                          (unsigned long)m.power_mw);
            }
            register_model.SetInputRegister(ModbusRegisters::Input::PWR_BUS_VOLTAGE_MV, (uint16_t)m.bus_voltage_mv);
            register_model.SetInputRegister(ModbusRegisters::Input::PWR_SHUNT_VOLTAGE_UV, (uint16_t)m.shunt_voltage_uv);
            register_model.SetInputRegister(ModbusRegisters::Input::PWR_CURRENT_MA, (uint16_t)m.current_ma);
            register_model.SetInputRegister(ModbusRegisters::Input::PWR_POWER_MW, (uint16_t)m.power_mw);
        } else {
            // readReg16() in ina226.cpp loggt bereits den Grund (HAL-Status/I2C-ErrorCode) pro
            // fehlgeschlagenem Register -- hier nur ein Zaehler, um zu sehen, ob es sich um
            // einen einmaligen Ausrutscher oder ein dauerhaftes Problem handelt.
            consecutive_read_failures_++;
            if (consecutive_read_failures_ <= 3 || (consecutive_read_failures_ % 20) == 0) {
                log_warn("INA226: Read() fehlgeschlagen (%lu mal in Folge)", (unsigned long)consecutive_read_failures_);
                // Diagnose (auf Nutzeranfrage): sind Manufacturer-ID/Die-ID (0xFE/0xFF, in
                // Probe() schon einmal erfolgreich gelesen) im Fehlerfall noch lesbar? Falls ja,
                // ist der Bus/die Adresse generell intakt und das Problem auf die
                // Mess-Register (0x01-0x04) beschraenkt. Falls nein, ist der Bus zwischenzeitlich
                // umfassender gestoert.
                uint16_t manufacturer_id = 0, die_id = 0;
                bool ids_ok = power_monitor.ReadDiagnosticIds(&manufacturer_id, &die_id);
                log_warn("INA226: Diagnose -- Manufacturer-ID/Die-ID %s (0x%04X / 0x%04X)",
                         ids_ok ? "lesbar" : "NICHT lesbar", (unsigned int)manufacturer_id, (unsigned int)die_id);
            }
            // Laufzeit-Gegenstueck zum Setup()-Reclaim in Probe(): ein Read()-Fehlschlag zur
            // Laufzeit (z.B. Timeout durch kurzzeitige ETH-DMA-Interrupt-Verdraengung des
            // io_threads, aehnlich dem HX711-Fall) kann den I2C4-Peripherie-Zustand ebenso
            // durcheinanderbringen wie TOF3s NACK beim Boot -- vor dem naechsten Zyklus
            // bedingungslos zuruecksetzen, statt dauerhaft haengen zu bleiben.
            power_monitor.ReclaimI2CBus();
        }
    }
};
