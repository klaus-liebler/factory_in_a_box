#pragma once

#include <cstdint>
#include <array>
#include <cmath>

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
    constexpr SigmoidAccelerationProfile(
        uint32_t timer_freq_hz,           ///< Timer-Frequenz nach Prescaler in Hz
        uint16_t prescaler,               ///< Prescaler-Wert (PSC Register)
        uint16_t initial_arr,             ///< Initialer ARR-Wert (Startgeschwindigkeit)
        uint16_t start_speed_steps_per_s, ///< Startgeschwindigkeit in steps/s
        uint16_t target_speed_steps_per_s,///< Zielgeschwindigkeit in steps/s
        uint8_t shift_bits,               ///< Anzahl der Bit-Verschiebungen für ARR-Werte
        uint8_t num_segments = 0          ///< Anzahl der Segmente (wird beim Initialisieren gesetzt)
    ) : timer_frequency_hz(timer_freq_hz),
        start_speed(start_speed_steps_per_s),
        target_speed(target_speed_steps_per_s),
        shift_bits(shift_bits),
        prescaler_value(prescaler),
        initial_arr_value(initial_arr),
        num_segments(num_segments),
        current_segment_idx(0),
        current_step_in_segment(0),
        current_arr(initial_arr << shift_bits),
        current_arr_slope(0),
        is_acceleration_complete(false)
    {}
    
    /// Reset des Beschleunigungsprofils
    void reset() {
        current_segment_idx = 0;
        current_step_in_segment = 0;
        is_acceleration_complete = false;
        // Verwende den initialisierten ARR-Wert
        current_arr = initial_arr_value << shift_bits;
    }
    
    /// Liefert den nächsten ARR-Wert für einen Schritt
    /// @return ARR-Wert für den Timer (0-65535), oder 65535 wenn fertig
    uint16_t getNextARR() {
        if (is_acceleration_complete) {
            return speed_to_arr(target_speed);
        }
        
        if (current_segment_idx >= num_segments) {
            is_acceleration_complete = true;
            return speed_to_arr(target_speed);
        }
        
        const AccelerationSegment& seg = lookup_table[current_segment_idx];
        
        // Wenn wir das Segment abgeschlossen haben, zum nächsten gehen
        if (current_step_in_segment >= seg.steps) {
            current_segment_idx++;
            current_step_in_segment = 0;
            
            if (current_segment_idx >= num_segments) {
                is_acceleration_complete = true;
                return speed_to_arr(target_speed);
            }
            const AccelerationSegment& next_seg = lookup_table[current_segment_idx];
            current_arr_slope = next_seg.arr_slope;
        }
        
        // Berechne den nächsten ARR-Wert
        uint16_t result = current_arr >> shift_bits;
        
        // Aktualisiere für nächsten Aufruf
        current_arr += current_arr_slope;
        current_step_in_segment++;
        
        return result;
    }
    
    /// Gibt zurück, ob die Beschleunigung abgeschlossen ist
    bool isComplete() const {
        return is_acceleration_complete;
    }
    
    /// Gibt die aktuelle Segmentnummer zurück (0-basiert)
    uint8_t getCurrentSegment() const {
        return current_segment_idx;
    }
    
    /// Gibt die Anzahl der Segmente zurück
    uint8_t getNumSegments() const {
        return num_segments;
    }
    
    /// Setzt den Zeiger auf eine externe Lookup-Tabelle
    void setLookupTable(const AccelerationSegment* table) {
        lookup_table = table;
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
    uint16_t getStartSpeed() const {
        return start_speed;
    }
    
    /// Gibt die Zielgeschwindigkeit zurück
    uint16_t getTargetSpeed() const {
        return target_speed;
    }
    
    /// Gibt den Prescaler-Wert zurück
    uint16_t getPrescaler() const {
        return prescaler_value;
    }
    
    /// Gibt den initialen ARR-Wert zurück
    uint16_t getInitialARR() const {
        return initial_arr_value;
    }

private:
    const uint32_t timer_frequency_hz;
    const uint16_t start_speed;
    const uint16_t target_speed;
    const uint8_t shift_bits;
    const uint16_t prescaler_value;
    const uint16_t initial_arr_value;
    uint8_t num_segments;
    
    // Laufzeit-Variablen
    const AccelerationSegment* lookup_table = nullptr;
    uint8_t current_segment_idx;
    uint16_t current_step_in_segment;
    uint32_t current_arr;
    int16_t current_arr_slope;
    bool is_acceleration_complete;
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
