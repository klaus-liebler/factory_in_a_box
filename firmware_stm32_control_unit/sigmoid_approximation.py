import sys
import numpy as np
import matplotlib.pyplot as plt


class SigmoidAcceleration:
    
    
    def __init__(self, timer_freq_after_prescaler_hz:int, t_total_time_seconds:int, start_speed:int, target_speed:int, max_intermediate_error, shift_bits):
        self.timer_frequency_after_prescaler_hz = timer_freq_after_prescaler_hz
        self.t_total_time_seconds = t_total_time_seconds
        self.t_total_ticks = t_total_time_seconds * timer_freq_after_prescaler_hz
        self.start_speed = start_speed
        self.target_speed = target_speed
        self.max_intermediate_error = max_intermediate_error
        self.shift_bits = shift_bits
        self.lookup_table = []
        
    def speed_to_arr(self, speed_steps_per_second:float) -> int:
        """Berechnet ARR-Wert aus Geschwindigkeit in steps/s"""
        if speed_steps_per_second <= 0:
            return 2**16 - 1
        arr_value = (self.timer_frequency_after_prescaler_hz / speed_steps_per_second) - 1
        return max(1, min(int(arr_value), 2**16 - 1))
    
    def arr_to_speed(self, arr:int) -> float:
        """Berechnet Geschwindigkeit aus ARR-Wert"""
        if arr <= 0:
            return 0
        return self.timer_frequency_after_prescaler_hz / (arr + 1)
    
    def sigmoid(self, x:float) -> float:
        """Standard-Sigmoid-Funktion von 0 bis 1"""
        return 1 / (1 + np.exp(-x))
    
    def sigmoid_speed(self, t_ticks:int, steepness:int=12) -> float:
        """Berechnet Geschwindigkeit zum Zeitpunkt t mit Sigmoid-Funktion"""
        # Normalisierte Zeit von 0 bis 1
        x = t_ticks / self.t_total_ticks
        #weitere Normalisierung von x auf -steepness/2 bis +steepness/2
        x= x * steepness - steepness/2
        y_min=self.sigmoid(-steepness/2)
        y_max=self.sigmoid(steepness/2)
        y = self.sigmoid(x)
        # Normalisieren von y auf [0, 1]
        y = (y - y_min) / (y_max - y_min)
        # Interpolieren zwischen Start- und Zielgeschwindigkeit
        return (self.start_speed + (self.target_speed - self.start_speed) * y)
    
    def arr(self, t_ticks:int) -> int:
        """Berechnet ARR-Wert zum Zeitpunkt t"""
        speed = self.sigmoid_speed(t_ticks)
        return self.speed_to_arr(speed)
    
    def endpoint_of_segment(self, start_time:int, start_arr_shifted:int, arr_delta:int, sections:int) :
        """Berechnet den End-ARR-Wert eines Segments"""
        current_arr_shifted=start_arr_shifted
        current_time=start_time
        for _ in range(sections):
            # Nutze UNGESHIFTETE Wert für Zeit-Berechnung (Hardware liest ungeshiftete ARR)
            current_time += current_arr_shifted >> self.shift_bits
            current_arr_shifted+=arr_delta
        return current_time, current_arr_shifted
    
    def max_unshifted_error_of_intermediate_points(self, start_time, start_arr_shifted, arr_delta, sections):
        """Berechnet die maximale Abweichung (ungeshiftet!) und den Schritt mit der maximalen Abweichung"""
        max_deviation = 0
        max_deviation_step = 0
        segment_arr_shifted = start_arr_shifted
        current_time = start_time
        

        for step in range(1, sections + 1):
            # Nutze UNGESHIFTETE Wert für Zeit-Berechnung (Hardware liest ungeshiftete ARR)
            current_time += segment_arr_shifted >> self.shift_bits
            segment_arr_shifted += arr_delta
            expected_arr = self.arr(current_time)
            error = abs((segment_arr_shifted >> self.shift_bits) - expected_arr)
            point_type = "Endpunkt" if step == sections else f"Zwischenpunkt {step}"
            print(f"    {point_type}: t={current_time}, segment_arr={segment_arr_shifted >> self.shift_bits}, expected_arr={expected_arr}, abweichung={error}")
            if error > max_deviation:
                max_deviation = error
                max_deviation_step = step
        return max_deviation, max_deviation_step
    
    def find_and_append_next_segment(self, start_time, start_arr_shifted, previous_slope):
        """Findet das längste Segment, das innerhalb der Grenzen liegt"""
        best_segment_section = None
        best_segment_arr_slope = None
        best_segment_t_end = None
        best_segment_y_end = None
        best_segment_max_error = 0
        best_segment_max_error_step = 0
        initial_slope = previous_slope  # Erste Schätzung der Steigung basierend auf vorheriger Steigung   
        
        for sections in range(1, 100):  # Maximal 100 Schritte pro Segment
            arr_slope = initial_slope
            t, y_seg = self.endpoint_of_segment(start_time, start_arr_shifted, arr_slope, sections)
            y_target = self.arr(t)<<self.shift_bits  # Geshifteter Zielwert
            
            # Passe arr_delta, also die Steigung an. Die Steigung muss immer -1 oder kleiner sein
            # wenn y_seg < y (ARR zu klein): arr_delta positiver machen (ARR erhöhen)
            # wenn y_seg > y (ARR zu groß): arr_delta negativer machen (ARR senken)

            if(y_seg < y_target):
                while y_seg < y_target:
                    arr_slope += 1
                    t, y_seg = self.endpoint_of_segment(start_time, start_arr_shifted, arr_slope, sections)
            elif y_seg > y_target:
                while y_seg > y_target:
                    arr_slope -= 1
                    t, y_seg = self.endpoint_of_segment(start_time, start_arr_shifted, arr_slope, sections)
            
            print(f"  Prüfe Segment ab t= {start_time}  start_arr = {start_arr_shifted}({start_arr_shifted >> self.shift_bits}) mit {sections} Schritten und arr_delta={arr_slope} ({arr_slope >> self.shift_bits}): t_end={t}, y_end_seg={y_seg}({y_seg >> self.shift_bits}), y_end_exakt={y_target}({y_target >> self.shift_bits})")
            if sections == 1 :
                #erstmal abspeichern
                best_segment_section = sections
                best_segment_arr_slope = arr_slope
                best_segment_t_end = t
                best_segment_y_end = y_seg
                best_segment_max_error = 0
                best_segment_max_error_step = 0
                if(t>self.t_total_ticks):
                    break
                else:
                    continue

            # Mehr als eine Sektion -> Prüfe Abweichung
            max_unshifted_error, max_error_step = self.max_unshifted_error_of_intermediate_points(start_time, start_arr_shifted, arr_slope, sections)
            
            # Akzeptiere, sofern Abweichung unterhalb Limit
            if  max_unshifted_error <= self.max_intermediate_error:
                print(f"  Maximale Zwischenabweichung: {max_unshifted_error} (bei Schritt {max_error_step}) -> Gültiges Segment mit {sections} Schritten gefunden.")
                #Gültiges Segment gefunden -->abspeichern und dann weiter probieren
                best_segment_section = sections
                best_segment_arr_slope = arr_slope
                best_segment_t_end = t
                best_segment_y_end = y_seg
                best_segment_max_error = max_unshifted_error
                best_segment_max_error_step = max_error_step
            else:
                # Ansonsten vorheriges bestes Segment verwenden und abbrechen
                print(f"  Maximale Zwischenabweichung: {max_unshifted_error} (bei Schritt {max_error_step}) -> Überschreitet das Limit von {self.max_intermediate_error}. --> Verwende vorheriges Segment.")
                print("-----------------------------------------------------")
                print(f"Gespeichertes Segment Nummer {len(self.lookup_table)}: sections={best_segment_section}, arr_delta={best_segment_arr_slope}, t_end={best_segment_t_end}, y_end={best_segment_y_end}")
                print("-----------------------------------------------------")
                print()
                break  # Überschreitet die maximale Abweichung

            if t > self.t_total_ticks:
                print(f"  Ende der Beschleunigung erreicht.")
                print("-----------------------------------------------------")
                print(f"Gespeichertes Segment Nummer {len(self.lookup_table)}: sections={best_segment_section}, arr_delta={best_segment_arr_slope}, t_end={best_segment_t_end}, y_end={best_segment_y_end}")
                print("-----------------------------------------------------")
                print()
                break  # Ende der Beschleunigung erreicht
        
        
        self.lookup_table.append({
            'steps': best_segment_section,
            'arr_slope': best_segment_arr_slope,
            'end_time': best_segment_t_end,
            'y_end': best_segment_y_end,  # Geshifteter Wert, keine Verschiebung nötig
            'max_deviation': best_segment_max_error,
            'max_deviation_step': best_segment_max_error_step
        })
        return best_segment_t_end, best_segment_y_end
        
    
    def generate_lookup_table(self):
        """
        Präzise Generierung der Lookup-Tabelle durch Simulation der tatsächlichen Schrittzeiten
        und Anpassung der Segmente basierend auf der maximalen Abweichung.
        """
        segment_start_time = 0
        last_slope = 0  # Start mit einer Steigung von 0 (konstanter ARR) für das erste Segment
        segment_start_arr = self.speed_to_arr(self.start_speed)<<self.shift_bits  # Geshifteter Start-ARR-Wert

        while segment_start_time < self.t_total_ticks:
            segment_start_time, segment_start_arr = self.find_and_append_next_segment(segment_start_time, segment_start_arr, last_slope)
            last_slope = self.lookup_table[-1]['arr_slope']  # Aktualisiere die Steigung für das nächste Segment
        return self.lookup_table

    def plot_details(self, lookup_table, ):
        """Plottet den gewünschten ARR-Verlauf vs. den angenäherten Verlauf aus der Lookup-Tabelle"""
        if len(lookup_table) == 0:
            return
        
        # Berechne den angenäherten ARR-Verlauf aus der Lookup-Tabelle
        approx_times = []
        approx_arr_values = []
        approx_arr_slopes = []  # NEU: Speichere arr_slope für jede Setzung
        segment_times = []
        current_time = 0
        
        current_arr_shifted = self.speed_to_arr(self.start_speed) << self.shift_bits  # Geshifteter Start-ARR-Wert

        # Erstelle das Diagramm
        fig, ax = plt.subplots(figsize=(14, 8))
        
        for i, segment in enumerate(lookup_table):
            arr_slope = segment['arr_slope']
            steps = segment['steps']
            
            for step in range(steps):
                current_time += current_arr_shifted >> self.shift_bits
                current_arr_shifted += arr_slope
                approx_times.append(current_time)
                approx_arr_values.append(current_arr_shifted >> self.shift_bits)  
                approx_arr_slopes.append(arr_slope >> self.shift_bits)  # NEU: Speichere arr_slope
            segment_times.append(current_time / self.timer_frequency_after_prescaler_hz * 1000)
            # Markiere die Segmentübergänge
            ax.axvline(x=segment_times[-1], color='gray', linestyle=':', alpha=0.5, linewidth=0.8)
        
        # Berechne den exakten ARR-Verlauf (Sigmoid)
        num_points = 1000
        exact_times = np.linspace(0, self.t_total_ticks, num_points)
        exact_arr_values = [self.arr(t) for t in exact_times]
        
      
        
        # Plot 1: Exakte ARR-Kurve (Sigmoid)
        ax.plot(exact_times / self.timer_frequency_after_prescaler_hz * 1000, exact_arr_values,
                'b-', linewidth=2.5, label='Gewünschter ARR-Verlauf (Sigmoid)', zorder=3)
        
        # Plot 2: Angenäherter ARR-Verlauf (Lookup-Tabelle als Punkte)
        ax.scatter(np.array(approx_times) / self.timer_frequency_after_prescaler_hz * 1000, 
                   approx_arr_values,
                   color='r', s=20, label='Angenäherter ARR-Verlauf (Lookup-Tabelle)', 
                   alpha=0.8, zorder=2)
        
        
        # Berechne die Abweichung
        max_deviation = 0
        total_deviation = 0
        if len(approx_times) > 0:
            for app_time, app_arr in zip(approx_times, approx_arr_values):
                # Finde den nächsten exakten Punkt
                idx = np.argmin(np.abs(exact_times - app_time))
                exact_arr_at_time = exact_arr_values[idx]
                deviation = abs(app_arr - exact_arr_at_time)
                max_deviation = max(max_deviation, deviation)
                total_deviation += deviation
            
            avg_deviation = total_deviation / len(approx_times)
        
        ax.set_xlabel('Zeit (ms)', fontsize=12)
        ax.set_ylabel('ARR-Wert', fontsize=12)
        ax.set_title(f'ARR-Verlauf: Gewuenscht vs. Angenaehert ({self.start_speed} -> {self.target_speed} steps/s)', 
                     fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=11)
        
        plt.tight_layout()
        plt.savefig('arr_comparison.png', dpi=150, bbox_inches='tight')
        
        # Tabellenausgabe
        print(f"\n{'='*110}")
        print(f"LOOKUP-TABELLE ÜBERSICHT:")
        print(f"{'='*110}")
        print(f"{'Seg':<4} {'Steps':<8} {'D.ARR':<8} {'End-Time [ms]':<15} {'End-ARR':<10} {'Max Abw.':<10} {'Schritt':<8}")
        print(f"{'-'*110}")
        
        current_time = 0
        current_arr_shifted = self.speed_to_arr(self.start_speed) << self.shift_bits  # Geshifteter Start-ARR-Wert
        for i, segment in enumerate(lookup_table):
            steps = segment['steps']
            arr_slope = segment['arr_slope']
            max_dev = segment.get('max_deviation', 'N/A')
            max_dev_step = segment.get('max_deviation_step', 'N/A')
            
            # FIX: Reihenfolge konsistent mit endpoint_of_segment
            for _ in range(steps):
                # Nutze UNGESHIFTETE Wert für Zeit-Berechnung
                current_time += current_arr_shifted >> self.shift_bits
                current_arr_shifted += arr_slope
            
            print(f"{i+1:<4} {steps:<8} {arr_slope:<8} {current_time / self.timer_frequency_after_prescaler_hz * 1000:<15.2f} {current_arr_shifted >> self.shift_bits:<10} {max_dev:<10} {max_dev_step:<8}")
        
        print(f"{'='*110}")
        
        # Ausgabe der ersten 20 ARR-Wert-Setzungen
        print(f"\n{'='*120}")
        print(f"ERSTE 20 ARR-WERT-SETZUNGEN:")
        print(f"{'='*120}")
        print(f"{'Nr.':<4} {'Zeit [Ticks]':<15} {'Berechnung':<45} {'Ideal ARR':<10} {'Abweichung':<12}")
        print(f"{'-'*120}")
        
        num_to_show = min(20, len(approx_times))
        prev_arr = self.speed_to_arr(self.start_speed) << self.shift_bits
        
        for i in range(num_to_show):
            time_ticks = approx_times[i]
            arr_val = approx_arr_values[i]
            arr_slope = approx_arr_slopes[i]
            # Berechnung anzeigen: prev_arr + arr_slope = arr_val
            calculation = f"{prev_arr} + {arr_slope:+d} = {arr_val}"
            
            # Finde den nächsten exakten Punkt
            idx = np.argmin(np.abs(exact_times - approx_times[i]))
            ideal_arr = exact_arr_values[idx]
            deviation = abs(arr_val - ideal_arr)
            print(f"{i+1:<4} {time_ticks:<15.0f} {calculation:<45} {ideal_arr:<10.1f} {deviation:<12.1f}")
            
            prev_arr = arr_val
        
        print(f"{'='*120}")
        
        # Gesamtstatistik auf der Konsole ausgeben
        print(f"\n{'='*80}")
        print(f"GESAMTSTATISTIK:")
        print(f"{'='*80}")
        print(f"Anzahl Segmente: {len(lookup_table)}")
        print(f"Anzahl Schritte: {sum(s['steps'] for s in lookup_table)}")
        print(f"Max. Abweichung: {max_deviation:.1f} ARR-Einheiten")
        print(f"Durchschn. Abweichung: {avg_deviation:.1f} ARR-Einheiten")
        print(f"{'='*80}")
        
        # Zeige das Diagramm am Ende
        plt.show()

def main():
    # Parameter
    cpu_freq = 170e6
    low_speed = 100  # in steps/s
    high_speed = 1000  # in steps/s
    assert low_speed < high_speed, "low_speed muss kleiner als high_speed sein"
    prescaler = int(((cpu_freq+(65536 * low_speed)) // (65536 * low_speed)))-1
    timer_freq = cpu_freq / (prescaler + 1)  # Timer-Frequenz in Hz
    
    print(f"Berechneter Prescaler: {prescaler} (muss manuell im Code angepasst werden)")
    print(f"Timer-Frequenz nach Prescaler: {timer_freq:.2f} Hz")
    print(f"Minimale Frequenz: {timer_freq / 65535:.2f} steps/s")
    print("ARR-Wert bei low_speed:", SigmoidAcceleration(timer_freq, 1, low_speed, high_speed, 0, 10).speed_to_arr(low_speed))
    print("ARR-Wert bei high_speed:", SigmoidAcceleration(timer_freq, 1, low_speed, high_speed, 0, 10).speed_to_arr(high_speed))
    assert prescaler >= 0, "Prescaler muss positiv sein. Bitte low_speed oder cpu_freq anpassen."
    assert prescaler < 65536, "Prescaler zu groß für 16-Bit Timer. Bitte low_speed oder cpu_freq anpassen."
    assert timer_freq / low_speed < 65536,"Die gewünschte Frequenz bei low_speed führt zu einem ARR-Wert über 65535. Bitte low_speed oder cpu_freq anpassen."
    
    accel_time_s=0.5  # Beschleunigungszeit in Sekunden
   
    shift_bits = 10  # Anzahl der Bits zum Verschieben (x4096)
    max_deviation =30


    print(f"\n{'#'*80}")
    print(f"TESTFALL: {low_speed} -> {high_speed} steps/s in {accel_time_s} Sekunden")
    print(f"{'#'*80}")
        
    # Generator initialisieren
    accel = SigmoidAcceleration(timer_freq, accel_time_s, low_speed, high_speed, max_deviation, shift_bits)
    
    
    # Präzise Lookup-Tabelle generieren mit geshiftetem start_arr
    lookup_table = accel.generate_lookup_table()
    print(f"Generierte Lookup-Tabelle mit {len(lookup_table)} Einträgen.")
    
    # Details plotten mit geshiftetem start_arr
    accel.plot_details(lookup_table, )
    
    # Generiere C++-Header-Datei
    generate_cpp_header(lookup_table, low_speed, high_speed, accel_time_s, timer_freq, shift_bits, prescaler)
        

def generate_cpp_header(lookup_table, low_speed, high_speed, accel_time_s, timer_freq, shift_bits, prescaler):
    """
    Generiert eine C++-Header-Datei mit der Lookup-Tabelle als constexpr std::array
    """
    import datetime
    
    # Generiere einen beschreibenden Namen für das Profil
    profile_name = f"acceleration_{int(low_speed)}_{int(high_speed)}_{int(accel_time_s*10)}"
    
    # Generiere den Header-Content
    header_content = f"""// Auto-generated by sigmoid_approximation.py on {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
// DO NOT EDIT MANUALLY

#pragma once

#include "SigmoidAcceleration.hpp"
#include <array>

/**
 * @file sigmoid_lookup_table_{profile_name}.hpp
 * @brief Auto-generated Acceleration Profile
 * 
 * Profile Parameters:
 * - Speed Range: {int(low_speed)} -> {int(high_speed)} steps/s
 * - Acceleration Time: {accel_time_s} seconds
 * - CPU Frequency: 170.0 MHz
 * - Prescaler: {prescaler} (Timer Frequency: {timer_freq:.2f} Hz)
 * - Shift Bits: {shift_bits}
 * - Segments: {len(lookup_table)}
 */

// Lookup table with {len(lookup_table)} segments
constexpr std::array<AccelerationSegment, {len(lookup_table)}> {profile_name} = {{{{
"""
    
    # Füge jeden Segment hinzu
    for i, segment in enumerate(lookup_table):
        steps = segment['steps']
        arr_slope = segment['arr_slope']
        
        header_content += f"""    {{{steps}, {arr_slope}}}{'' if i == len(lookup_table) - 1 else ','}
"""
    
    # Berechne den initialen ARR-Wert
    initial_arr = SigmoidAcceleration(timer_freq, 1, low_speed, high_speed, 0, 10).speed_to_arr(low_speed)
    
    header_content += f"""
}}}};

// Profile instance for easy access
constexpr SigmoidAccelerationProfile {profile_name}_profile(
    {int(timer_freq)},  // timer_freq_hz
    {prescaler},        // prescaler
    {initial_arr},      // initial_arr
    {int(low_speed)},   // start_speed_steps_per_s
    {int(high_speed)},  // target_speed_steps_per_s
    {shift_bits},       // shift_bits
    {len(lookup_table)} // num_segments
);
"""
    
    # Speichere die Header-Datei
    filename = f"sigmoid_lookup_table_{profile_name}.hpp"
    with open(filename, 'w') as f:
        f.write(header_content)
    
    print(f"\n{'='*80}")
    print(f"C++-HEADER-DATEI GENERIERT:")
    print(f"{'='*80}")
    print(f"Dateiname: {filename}")
    print(f"Variablenname: {profile_name}")
    print(f"Segmente: {len(lookup_table)}")
    print(f"{'='*80}\n")

if __name__ == "__main__":
    main()