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
            // register_model existiert bereits beim allerersten EarlySetup()-Aufruf (wird in
            // App::SetupBeforeThreadX() vor der USBPDControl-Konstruktion angelegt, s. app.cc)
            // -- auch die blockierende 20V-Anforderung dort kann also schon Register schreiben.
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


// Rein elektrische CC-Anschlusserkennung (PowerController.ccPin != 0, s. PDSink::reset())
// laesst PowerSink.isConnected() sofort false bleiben, wenn am USB-C-Port ueberhaupt nichts
// angeschlossen ist -- kein Warten auf einen PD-Protokoll-Timeout in diesem Fall. Ist dagegen
// etwas angeschlossen, das (noch) nicht antwortet, deckt dieses Fenster die interne Retry-/
// Hard-Reset-Kaskade ab (PDSink.cpp: 100ms initiale Anfrage + bis zu 3x 200ms Retry vor einem
// selbstausgeloesten Hard Reset, der die Kaskade neu beginnt).
constexpr uint32_t PD_SOURCE_DETECT_TIMEOUT_MS = 2000;

void USBPDControl::EarlySetup(int target_voltage_mv) {
    target_voltage_mv_ = target_voltage_mv;

    // PowerSink.start() enables the scheduler timer (TIM7) and initializes the UCPD1
    // PHY (clocks/GPIO/DMA/NVIC) -- plain register-level setup, unlike the NetX calls
    // it has no running-thread requirement, so it's safe to call from tx_application_define().
    // "this" is the IUsbPdEventHandler: USBPDControl implements HandleUsbPDEvent() itself.
    PowerSink.start(this);

    // Ganz frueh (vor allen anderen io_setup()-Schritten, s. io_thread.cpp) pruefen, ob ein
    // USB-PD-Netzteil die Versorgung uebernimmt: ohne aktive Anforderung liefert ein PD-
    // Netzteil per Spec nur die 5V-Default-Spannung -- an einer 20V-Verbraucherlast (Stepper-
    // Endstufen etc.) waere das kein harmloser Normalzustand, sondern eine Unterspannung.
    uint32_t detect_start = HAL_GetTick();
    while (!PowerSink.isConnected() && (HAL_GetTick() - detect_start) < PD_SOURCE_DETECT_TIMEOUT_MS) {
        PowerSink.Loop();
        HAL_Delay(1);
    }

    if (!PowerSink.isConnected()) {
        log_info("USB-PD: no source detected within %lums - assuming separate/conventional power supply",
                 (unsigned long)PD_SOURCE_DETECT_TIMEOUT_MS);
        UpdatePdStatusRegister();
        return;
    }

    log_info("USB-PD: source detected, requesting %d mV...", target_voltage_mv);
    PowerSink.requestPower(target_voltage_mv);
    // Absichtlich unbegrenztes Warten: sobald feststeht, dass ein PD-Netzteil die Versorgung
    // uebernimmt, darf io_setup() (und alles danach, insbesondere stepper_setup()) sich erst
    // fortsetzen, wenn PD_TARGET_VOLTAGE_MV tatsaechlich anliegt -- kein Fallback-Timeout hier,
    // sonst liefe die restliche Initialisierung ggf. auf einer Unterspannung weiter.
    while (PowerSink.activeVoltage < target_voltage_mv) {
        PowerSink.Loop();
        HAL_Delay(1);
    }
    log_info("USB-PD: %d mV confirmed active.", PowerSink.activeVoltage);
    UpdatePdStatusRegister();
}

void USBPDControl::Loop() {
    PowerSink.Loop();
}
