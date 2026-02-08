/**
 * @file Usage_Example.cpp
 * @brief Einfaches Verwendungsbeispiel für die generierte Lookup-Tabelle
 */

#include "SigmoidAcceleration.hpp"
#include "sigmoid_lookup_table_acceleration_100_1000_5.hpp"

// Globale Instanz
static SigmoidAccelerationProfile accel_profile(
    6538461,   // Timer-Frequenz (170 MHz / 26)
    100,       // Start-Geschwindigkeit
    1000,      // Ziel-Geschwindigkeit
    10         // Shift-Bits
);

void setup() {
    // Setze die Lookup-Tabelle
    accel_profile.setLookupTable(acceleration_100_1000_5.data());
    
    // Reset beim Start
    accel_profile.reset();
}

void step_interrupt_handler() {
    // Rufe diese Funktion bei jedem Step auf
    uint16_t arr_value = accel_profile.getNextARR();
    
    // Schreibe in das Timer-Register
    // TIM1->ARR = arr_value;
    
    // Optional: Prüfe ob fertig
    if (accel_profile.isComplete()) {
        // Beschleunigung abgeschlossen
    }
}
