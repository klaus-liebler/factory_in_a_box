#pragma once
// Eigene, vom Register-Map unabhaengige GPIO-Liste fuer Mission-Schritte (roarm.GpioStep,
// gpioId indiziert hier hinein) -- siehe Plan Abschnitt 7. Bewusst NICHT an die Modbus-
// Registerkarte gekoppelt: ein Missions-Schritt soll direkt einen physischen Pin schalten,
// unabhaengig davon, ob/wie dieser Pin sonst irgendwo als Register auftaucht.
//
// Die vier Eintraege sind Platzhalter (port=nullptr) -- welche physischen Pins hierfuer frei
// sind, stand beim Schreiben dieser Datei noch nicht fest. Dies ist bewusst der EINZIGE Ort, an
// dem spaeter echte Pins eingetragen werden muessen; alles andere (Schema, Web-UI, Mission-
// Player in roarm.hh) ist bereits fertig damit verdrahtet und braucht dann keine Aenderung.
#include "main.h"

namespace RoArmMissionGpio {

struct Def {
    const char* name;
    GPIO_TypeDef* port;
    uint16_t pin;
};

inline constexpr int kCount = 4;
inline constexpr Def kGpios[kCount] = {
    {"Mission GPIO 1", nullptr, 0},
    {"Mission GPIO 2", nullptr, 0},
    {"Mission GPIO 3", nullptr, 0},
    {"Mission GPIO 4", nullptr, 0},
};

// gpioId aus einem GpioStep -> HAL-Pin setzen. Solange fuer den betreffenden Eintrag noch kein
// Port eingetragen ist (s. Kommentar oben), wird der Schritt sicher ignoriert (kein Hardfault
// durch NULL-Zugriff) statt eines stillschweigend falschen Verhaltens.
inline bool Set(uint8_t gpioId, bool state) {
    if (gpioId >= kCount) return false;
    const Def& def = kGpios[gpioId];
    if (def.port == nullptr) return false;
    HAL_GPIO_WritePin(def.port, def.pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return true;
}

} // namespace RoArmMissionGpio
