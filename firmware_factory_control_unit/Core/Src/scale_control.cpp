// ============================================================================
// Scale -- Waegezelle A (HX711 ueber SPI2, per Dummy-Bytes getakteter DOUT-Bitbang, siehe
// stm32_libs/hx711/hx711.hh) an PC2/PC3 (HX711_SPI2_MISO/MOSI).
//
// Waegezelle B (SCALE_B_* im Register-Map) ist auf dieser Platinen-Revision nicht bestueckt
// ("Reserve/Erweiterung" laut register-map.json) -- dafuer wird hier nichts angesteuert, die
// Register bleiben auf 0.
//
// HAL_SPI_TransmitReceive_DMA() (siehe HX711::loop()) braucht eine per CubeMX auf SPI2 TX/RX
// verlinkte GPDMA-Instanz.
// ============================================================================
#include "scale_control.hpp"
#include "app_state.hpp"
#include "modbus_register_map.hpp"
#include "log.h"

#include "hx711.hh"

extern "C" SPI_HandleTypeDef hspi2;

static HX711<8> scale_a(&hspi2, gpio::Pin::PC02, Channel_And_Gain::CH_A_GAIN_128);

void scale_setup() {
    scale_a.Setup();
}

void scale_update() {
    ModbusTcpServer& server = *g_app_state.modbus_server;

    scale_a.loop();

    int32_t raw_value;
    if (scale_a.getAverageRawValue(&raw_value)) {
        // Noch unkalibriert (keine Referenzgewichte vermessen) -- Rohwert direkt in die
        // WEIGHT-Register geschrieben, bis Kalibrierpunkte feststehen (siehe
        // HX711::getAverageCalibratedWeight() fuer die Umrechnung, sobald verfuegbar).
        server.write_input_register(ModbusRegisters::Input::SCALE_A_WEIGHT_HI, (uint16_t)((uint32_t)raw_value >> 16));
        server.write_input_register(ModbusRegisters::Input::SCALE_A_WEIGHT_LO, (uint16_t)((uint32_t)raw_value & 0xFFFF));
        server.write_input_register(ModbusRegisters::Input::SCALE_A_STATUS, 1);
    } else {
        server.write_input_register(ModbusRegisters::Input::SCALE_A_STATUS, 0);
    }
}
