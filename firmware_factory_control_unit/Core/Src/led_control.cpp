// ============================================================================
// INFOLED -- blinkt dauerhaft als Heartbeat (sichtbares Lebenszeichen: io_thread laeuft noch,
// haengt nirgends fest). Kein Webserver-Endpunkt mehr dafuer (/LedOn /LedOff wurden entfernt) --
// nichts schaltet das Muster mehr um, daher genuegt ein einmaliges AnimatePixel() in init()
// statt der vorherigen Aenderungserkennung in update(). Nutzt single_led::M (aktiv=LOW,
// Template-Default) statt direkter HAL_GPIO_*-Aufrufe.
// ============================================================================
#include "led_control.hpp"
#include "main.h"
#include "single_led.hh"
#include "tx_api.h"

// gpio::Pin::PE15 entspricht main.h's INFOLED_Pin/INFOLED_GPIO_Port (PE15).
static single_led::M<false> info_led(gpio::Pin::PE15);
// timeOn/timeOff in ThreadX-Ticks (10ms/Tick, s. tx_initialize_low_level.S) -- 50/50 ergibt
// eine 1s-Blinkperiode.
static single_led::BlinkPattern heartbeat_pattern(50, 50);

void led_control_init() {
    uint32_t now = tx_time_get();
    info_led.Begin(now);
    info_led.AnimatePixel(now, &heartbeat_pattern);
}

void led_control_update() {
    info_led.Loop(tx_time_get());
}
