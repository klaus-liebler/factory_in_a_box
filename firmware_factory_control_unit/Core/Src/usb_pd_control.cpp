// ============================================================================
// USB-PD -- fuer's Erste nur Ausgabe der vom Netzteil gemeldeten Capabilities auf der
// Konsole, keine aktive Spannungsanforderung (der Sink fordert selbstaendig 5V an, siehe
// PDSink::onSourceCapabilities()).
// ============================================================================
#include "usb_pd_control.hpp"
#include "app_state.hpp"
#include "modbus_register_map.hpp"
#include "PDSink.h"
#include "log.h"
#include <memory>

static void usb_pd_event_callback(PDSinkEventType eventType) {
    switch (eventType) {
        case PDSinkEventType::sourceCapabilitiesChanged:
            {
                std::unique_ptr<char[]> buf = PowerSink.printCapabilitiesToBuf(25);
                log_info("%s", buf.get());
            }
            break;
        case PDSinkEventType::voltageChanged:
            log_info("USB-PD: active supply now %d mV / %d mA", PowerSink.activeVoltage, PowerSink.activeCurrent);
            // g_app_state.modbus_server ist hier immer gesetzt: der Callback feuert nur aus
            // PowerSink.Loop() (siehe usb_pd_update()), das erst aus dem io_thread heraus
            // aufgerufen wird, lange nachdem modbus_server in tx_application_define()
            // zugewiesen wurde.
            g_app_state.modbus_server->write_input_register(ModbusRegisters::Input::PWR_PD_VOLTAGE_MV, (uint16_t)PowerSink.activeVoltage);
            g_app_state.modbus_server->write_input_register(ModbusRegisters::Input::PWR_PD_CURRENT_MA, (uint16_t)PowerSink.activeCurrent);
            g_app_state.modbus_server->write_input_register(ModbusRegisters::Input::PWR_PD_STATUS, PowerSink.isConnected() ? 1 : 0);
            break;
        case PDSinkEventType::powerRejected:
            log_warn("USB-PD: power request rejected by source");
            break;
    }
}

void usb_pd_setup() {
    // Anders als auf dem STM32H573I-DK sitzt hier kein TCPP0203-Port-Protection-IC
    // zwischen UCPD1 und der USB-C-Buchse, das vor PowerSink.start() erst per I2C
    // geweckt werden muesste -- UCPD1 liegt direkt an CC1/CC2.

    // PowerSink.start() enables the scheduler timer (TIM7) and initializes the UCPD1
    // PHY (clocks/GPIO/DMA/NVIC) -- plain register-level setup, unlike the NetX calls
    // it has no running-thread requirement, so it's safe to call from tx_application_define().
    PowerSink.start(usb_pd_event_callback);
}

void usb_pd_update() {
    PowerSink.Loop();
}
