#pragma once
// ============================================================================
// Scale -- Waegezelle A (HX711 ueber SPI2, per Dummy-Bytes getakteter DOUT-Bitbang, siehe
// stm32_libs/hx711/hx711.hh) an PC2/PC3 (HX711_SPI2_MISO/MOSI).
//
// Waegezelle B (SCALE_B_* im Register-Map) ist auf dieser Platinen-Revision nicht bestueckt
// ("Reserve/Erweiterung" laut register-map.json) -- dafuer wird hier nichts angesteuert, die
// Register bleiben auf 0.
//
// HX711::loop() nutzt bewusst einen blockierenden HAL_SPI_TransmitReceive() statt
// HAL_SPI_TransmitReceive_DMA() -- die DMA-Variante blieb auf diesem Board zuverlaessig in
// HAL_SPI_STATE_BUSY_TX_RX haengen (s. Kommentar in hx711.hh, loop()), daher keine
// CubeMX-DMA-Verlinkung fuer SPI2 mehr noetig.
// ============================================================================
#include "interfaces.hh"
#include "modbus_register_model.hh"
#include "hx711.hh"

extern "C" SPI_HandleTypeDef hspi2;

class ScaleSetupAndLoop : public ISetupAndLoop {
    private:
    Modbus::IModbusRegisterModel& register_model;
    HX711<8> scale_a{&hspi2, gpio::Pin::PC02, Channel_And_Gain::CH_A_GAIN_128};

    public:
    ScaleSetupAndLoop(Modbus::IModbusRegisterModel& model) : register_model(model) {}

    void Setup() override {
        scale_a.Setup();
    }

    void Loop(uint32_t now) override {
        (void)now;
        scale_a.loop();

        int32_t raw_value;
        if (scale_a.getAverageRawValue(&raw_value)) {
            // Noch unkalibriert (keine Referenzgewichte vermessen) -- Rohwert direkt in die
            // WEIGHT-Register geschrieben, bis Kalibrierpunkte feststehen (siehe
            // HX711::getAverageCalibratedWeight() fuer die Umrechnung, sobald verfuegbar).
            register_model.SetInputRegister(ModbusRegisters::Input::SCALE_A_WEIGHT_HI, (uint16_t)((uint32_t)raw_value >> 16));
            register_model.SetInputRegister(ModbusRegisters::Input::SCALE_A_WEIGHT_LO, (uint16_t)((uint32_t)raw_value & 0xFFFF));
            register_model.SetInputRegister(ModbusRegisters::Input::SCALE_A_STATUS, 1);
        } else {
            register_model.SetInputRegister(ModbusRegisters::Input::SCALE_A_STATUS, 0);
        }
    }
};
