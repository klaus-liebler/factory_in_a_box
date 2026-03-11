#pragma once

#include <cstdint>
#include <array>
#include <cmath>
#include <cstddef>

/**
 * @file SigmoidAcceleration.hpp
 * @brief Sigmoid-basierte Beschleunigung für Schrittmotoren mit ARR-Lookup-Tabelle
 * 
 * Diese Header-Datei implementiert die Sigmoid-Beschleunigung für STM32-Timer-basierte
 * Motorsteuerung. Die Lookup-Tabelle wird vom Python-Skript generiert und muss in
 * SigmoidLookupTables.hpp definiert sein.
 */

/// Struktur für ein Segment der Lookup-Tabelle
struct AccelerationSegment {
    uint16_t steps;      ///< Anzahl der Schritte in diesem Segment
    int32_t arr_slope;   ///< Änderung des ARR-Werts pro Schritt (geshiftet)
};

/**
 * @class SigmoidAccelerationProfile
 * @brief Verwaltet einen Beschleunigungsprofil mit Sigmoid-Funktion
 * 
 * Diese Klasse stellt eine einfache Schnittstelle zur Verwendung der vorberechneten
 * Lookup-Tabelle bereit. Sie kann direkt in der STM32-Firmware verwendet werden.
 */
class SigmoidAccelerationProfile {
public:
    /// Konstruktor mit Beschleunigungsparametern
    template <size_t N>
    constexpr SigmoidAccelerationProfile(
        uint32_t timer_freq_hz,           ///< Timer-Frequenz nach Prescaler in Hz
        uint16_t prescaler,               ///< Prescaler-Wert (PSC Register)
        uint16_t low_speed_arr,             ///< Low-Speed ARR-Wert (Startgeschwindigkeit)
        uint16_t low_speed_steps_per_s, ///< Startgeschwindigkeit in steps/s
        uint16_t high_speed_steps_per_s,///< Zielgeschwindigkeit in steps/s
        uint8_t shift_bits,               ///< Anzahl der Bit-Verschiebungen für ARR-Werte
        uint32_t total_steps,             ///< Gesamtanzahl Schritte (Beschleunigen+Abbremsen)
        const std::array<AccelerationSegment, N>& table ///< Lookup-Tabelle
    ) : timer_frequency_hz(timer_freq_hz),
        low_speed_steps_per_s(low_speed_steps_per_s),
        high_speed_steps_per_s(high_speed_steps_per_s),
        shift_bits(shift_bits),
        prescaler_value(prescaler),
        low_speed_arr(low_speed_arr),
        total_steps(total_steps),
        num_segments(static_cast<uint8_t>(N)),
        lookup_table(table.data())
    {}
    

    
    
    /// Gibt die Anzahl der Segmente zurück
    uint8_t getNumSegments() const {
        return num_segments;
    }

    /// Liefert ein Segment aus der Lookup-Tabelle
    const AccelerationSegment& getSegment(uint8_t index) const {
        return lookup_table[index];
    }
    
    /// Konvertiert Speed in ARR-Wert (ohne Bit-Verschiebung)
    uint16_t speed_to_arr(uint16_t speed_steps_per_s) const {
        if (speed_steps_per_s <= 0) {
            return 65535;
        }
        uint32_t arr_value = (timer_frequency_hz / speed_steps_per_s) - 1;
        if (arr_value > 65535) arr_value = 65535;
        if (arr_value < 1) arr_value = 1;
        return static_cast<uint16_t>(arr_value);
    }
    
    /// Konvertiert ARR-Wert zu Speed
    uint16_t arr_to_speed(uint16_t arr) const {
        if (arr <= 0) {
            return 0;
        }
        return static_cast<uint16_t>(timer_frequency_hz / (arr + 1));
    }
    
    /// Gibt die Timer-Frequenz zurück
    uint32_t getTimerFrequency() const {
        return timer_frequency_hz;
    }
    
    /// Gibt die Startgeschwindigkeit zurück
    uint16_t getLowSpeedStepsPerSecond() const {
        return low_speed_steps_per_s;
    }
    
    /// Gibt die Zielgeschwindigkeit zurück
    uint16_t getHighSpeedStepsPerSecond() const {
        return high_speed_steps_per_s;
    }
    
    /// Gibt den Prescaler-Wert zurück
    uint16_t getPrescaler() const {
        return prescaler_value;
    }
    
    /// Gibt den initialen ARR-Wert zurück
    uint16_t getLowSpeedARR16() const {
        return low_speed_arr;
    }

    /// Gibt die Gesamtanzahl der Schritte zurück (Beschleunigen+Abbremsen)
    uint32_t getTotalSteps() const {
        return total_steps;
    }

    uint8_t getShiftBits() const {
        return shift_bits;
    }

private:
    const uint32_t timer_frequency_hz;
    const uint16_t low_speed_steps_per_s;
    const uint16_t high_speed_steps_per_s;
    const uint8_t shift_bits;
    const uint16_t prescaler_value;
    const uint16_t low_speed_arr;
    const uint32_t total_steps;
    const uint8_t num_segments;
    const AccelerationSegment* lookup_table;
};

/**
 * @brief Prescaler-Berechnung für STM32-Timer
 * 
 * Berechnet den Prescaler-Wert, damit ARR nicht größer als 65535 ist.
 * 
 * @param cpu_freq CPU-Frequenz in Hz
 * @param arr_max Maximaler ARR-Wert (normalerweise 65535 für 16-Bit Timer)
 * @param desired_freq Gewünschte Timer-Frequenz in Hz nach dem Prescaler
 * @return Prescaler-Wert (PSC Register, von 0 bis 65535)
 * 
 * @note Formel: PSC = ceil(cpu_freq / (arr_max * desired_freq)) - 1
 */
constexpr uint16_t calculatePrescaler(
    uint32_t cpu_freq,
    uint16_t arr_max,
    uint32_t desired_freq
) {
    // Berechne den Prescaler: ceil(cpu_freq / (arr_max * desired_freq)) - 1
    uint32_t divisor = (uint32_t)arr_max * desired_freq;
    uint32_t prescaler = (cpu_freq + divisor - 1) / divisor - 1;
    
    if (prescaler > 65535) prescaler = 65535;
    
    return static_cast<uint16_t>(prescaler);
}

/**
 * @brief Alternative Prescaler-Berechnung: gegeben ist die minimale Geschwindigkeit
 * 
 * Berechnet den Prescaler so, dass bei der minimalen Geschwindigkeit ARR <= 65535
 * 
 * @param cpu_freq CPU-Frequenz in Hz
 * @param min_speed_steps_per_s Minimale Geschwindigkeit in steps/s
 * @return Prescaler-Wert (PSC Register)
 */
constexpr uint16_t calculatePrescalerFromMinSpeed(
    uint32_t cpu_freq,
    uint16_t min_speed_steps_per_s
) {
    // ARR = cpu_freq / ((PSC+1) * speed) - 1
    // ARR <= 65535 => cpu_freq / ((PSC+1) * speed) - 1 <= 65535
    // => cpu_freq / ((PSC+1) * speed) <= 65536
    // => cpu_freq <= 65536 * (PSC+1) * speed
    // => (PSC+1) >= cpu_freq / (65536 * speed)
    // => PSC >= cpu_freq / (65536 * speed) - 1
    
    uint32_t psc_plus_1 = (cpu_freq + (65536 * min_speed_steps_per_s) - 1) / (65536 * min_speed_steps_per_s);
    uint16_t prescaler = (psc_plus_1 > 1) ? psc_plus_1 - 1 : 0;
    
    if (prescaler > 65535) prescaler = 65535;
    
    return prescaler;
}
