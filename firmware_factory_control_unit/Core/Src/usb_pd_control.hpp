#pragma once
// Initialisiert PowerSink (UCPD1-PHY, Scheduler-Timer) und registriert den Event-Callback.
// Reiner Registerlevel-Setup ohne Thread-Kontext-Anforderung, daher aus tx_application_define()
// aufrufbar (siehe app.cpp).
void usb_pd_setup();

// Liefert Events (Capabilities-Aenderung, Spannungswechsel etc.) an den intern registrierten
// Callback aus -- muss regelmaessig aus Thread-Kontext aufgerufen werden (PowerSink haelt
// intern eine eigene, ISR-getriebene Zustandsmaschine fuer das eigentliche PD-Protokolltiming;
// Loop() dient nur der Zustellung an den App-Callback ausserhalb des IRQ-Kontexts, s.
// usb_pd_control.cpp). Aufruf einmal pro io_thread-Zyklus (s. io_thread.cpp).
void usb_pd_update();
