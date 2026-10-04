#pragma once
#include "modbus_commons.hh"
#include "IUsbPdEventHandler.h"

class USBPDControl:public IUsbPdEventHandler {
    private:
    Modbus::IModbusRegisterModel* register_model;
    // Von Start() gemerkt, um in UpdatePdStatusRegister() zu erkennen, ob eine spaeter
    // (Event-getrieben) aktive Spannung tatsaechlich der urspruenglich angeforderten entspricht
    // (PWR_PD_STATUS-Fehlercode 1 = verbunden, aber Zielspannung nicht erreicht/gehalten).
    int target_voltage_mv_ = 0;

    public:
    // Fehlercode-Konvention (s. register-map.json PWR_PD_STATUS): 0 = ok/Zielspannung aktiv,
    // 1 = verbunden aber Zielspannung nicht erreicht, 2 = keine PD-Quelle verbunden. Oeffentlich,
    // damit der Aufrufer den Status auch dann setzen kann, wenn PD gar nicht gestartet wird
    // (Versorgung ueber Hohlstecker, s. App::WaitForSupplyVoltage()).
    void UpdatePdStatusRegister();

    USBPDControl(Modbus::IModbusRegisterModel* register_model) : register_model(register_model) {}
    

    // Initialisiert PowerSink (UCPD1-PHY, Scheduler-Timer TIM7), registriert den Event-Callback
    // und hinterlegt target_voltage_mv als anzufordernde Spannung. Kehrt sofort zurueck -- die
    // Aushandlung laeuft ISR-getrieben weiter. Nur aus Thread-Kontext nach dem ThreadX-Start
    // aufrufen: der Scheduler leitet seine Zeitbasis aus tx_time_get() ab (s. TaskScheduler.cpp).
    void Start(int target_voltage_mv);

    // Liefert Events (Capabilities-Aenderung, Spannungswechsel etc.) an den intern registrierten
    // Callback aus -- muss regelmaessig aus Thread-Kontext aufgerufen werden (PowerSink haelt
    // intern eine eigene, ISR-getriebene Zustandsmaschine fuer das eigentliche PD-Protokolltiming;
    // Loop() dient nur der Zustellung an den App-Callback ausserhalb des IRQ-Kontexts, s.
    // usb_pd_control.cpp). Aufruf waehrend App::WaitForSupplyVoltage() und danach einmal pro
    // IO-Thread-Zyklus (s. io.cpp). Ohne vorheriges Start() wirkungslos.
    void Loop();
    void HandleUsbPDEvent(PDSinkEventType eventType) override;
};
