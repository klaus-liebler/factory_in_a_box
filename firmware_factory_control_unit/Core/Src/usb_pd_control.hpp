#pragma once
#include "modbus_commons.hh"
#include "IUsbPdEventHandler.h"

class USBPDControl:public IUsbPdEventHandler {
    private:
    Modbus::IModbusRegisterModel* register_model;
    public:
    USBPDControl(Modbus::IModbusRegisterModel* register_model) : register_model(register_model) {}
    

    // Initialisiert PowerSink (UCPD1-PHY, Scheduler-Timer) und registriert den Event-Callback.
    // Reiner Registerlevel-Setup ohne Thread-Kontext-Anforderung, daher überall aufrufbar (auch vor tx_kernel_enter() in io_setup(), s. io_thread.cpp). Der
    void EarlySetup(int );

    // Liefert Events (Capabilities-Aenderung, Spannungswechsel etc.) an den intern registrierten
    // Callback aus -- muss regelmaessig aus Thread-Kontext aufgerufen werden (PowerSink haelt
    // intern eine eigene, ISR-getriebene Zustandsmaschine fuer das eigentliche PD-Protokolltiming;
    // Loop() dient nur der Zustellung an den App-Callback ausserhalb des IRQ-Kontexts, s.
    // usb_pd_control.cpp). Aufruf einmal pro io_thread-Zyklus (s. io_thread.cpp).
    void Loop();
    void HandleUsbPDEvent(PDSinkEventType eventType) override;
};
