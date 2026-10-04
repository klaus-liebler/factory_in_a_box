#include "usb_pd_control.hpp"
#include "modbus_register_model.hh"
#include "PDSink.h"
#include "log.h"
#include "main.h"
#include <memory>

void USBPDControl::UpdatePdStatusRegister() {
    uint16_t status;
    if (!PowerSink.isConnected()) {
        status = 2; // keine PD-Quelle verbunden
    } else if (target_voltage_mv_ > 0 && PowerSink.activeVoltage < target_voltage_mv_) {
        status = 1; // verbunden, aber Zielspannung nicht erreicht/gehalten
    } else {
        status = 0; // verbunden und Zielspannung aktiv
    }
    register_model->SetInputRegister(ModbusRegisters::Input::PWR_PD_STATUS, status);
}

void USBPDControl::HandleUsbPDEvent(PDSinkEventType eventType) {
    switch (eventType) {
        case PDSinkEventType::sourceCapabilitiesChanged:
            {
                std::unique_ptr<char[]> buf = PowerSink.printCapabilitiesToBuf(25);
                log_info("%s", buf.get());
            }
            break;
        case PDSinkEventType::voltageChanged:
            log_info("USB-PD: active supply now %d mV / %d mA", PowerSink.activeVoltage, PowerSink.activeCurrent);
            // register_model existiert bereits beim Start() (wird in
            // App::InitIdentityAndRegisterModel() angelegt, s. app.cc) -- Events waehrend App::WaitForSupplyVoltage() koennen also
            // schon Register schreiben.
            register_model->SetInputRegister(ModbusRegisters::Input::PWR_PD_VOLTAGE_MV, (uint16_t)PowerSink.activeVoltage);
            register_model->SetInputRegister(ModbusRegisters::Input::PWR_PD_CURRENT_MA, (uint16_t)PowerSink.activeCurrent);
            UpdatePdStatusRegister();
            break;
        case PDSinkEventType::powerRejected:
            log_warn("USB-PD: power request rejected by source");
            UpdatePdStatusRegister();
            break;
    }
}


void USBPDControl::Start(int target_voltage_mv) {
    target_voltage_mv_ = target_voltage_mv;

    // Zielspannung VOR dem Start hinterlegen (PDSink::desiredVoltage, Default 5000): ohne Quelle
    // gibt requestPower() nur notConnected zurueck, merkt sich den Wert aber -- sobald
    // Source_Capabilities eintreffen, fordert PDSink::onSourceCapabilities() direkt diese
    // Spannung an statt 5V. Vor start() aufgerufen, damit keine bereits per ISR eintreffenden
    // Capabilities mit dem alten 5V-Default beantwortet werden koennen.
    PowerSink.requestPower(target_voltage_mv);

    // PowerSink.start() aktiviert den Scheduler-Timer (TIM7) und initialisiert den UCPD1-PHY
    // (Takte/GPIO/DMA/NVIC); "this" ist der IUsbPdEventHandler. Nicht blockierend -- ob die
    // Zielspannung tatsaechlich anliegt, prueft der Aufrufer (App::WaitForSupplyVoltage(), s.
    // app.cc) per INA226 und ruft dabei regelmaessig Loop() auf, damit die Events zugestellt
    // werden.
    PowerSink.start(this);
    log_info("USB-PD: gestartet, fordere %d mV an", target_voltage_mv);
    UpdatePdStatusRegister();
}

void USBPDControl::Loop() {
    PowerSink.Loop();
}
