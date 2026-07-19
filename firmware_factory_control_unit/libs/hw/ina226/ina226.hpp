#pragma once
// INA226 Strom-/Spannungs-/Leistungsmesser (I2C). Adresse wird ueber A0/A1 bestimmt -- auf
// diesem Board liegen beide auf GND, das ergibt die Standardadresse 0x40 (s. Datenblatt Tabelle
// "A1/A0 Address Pin Connections").
#include <cstdint>
#include "main.h"

namespace ina226 {

struct Measurement {
    int32_t bus_voltage_mv;
    int32_t shunt_voltage_uv;
    int32_t current_ma;
    uint32_t power_mw;
};

class INA226 {
public:
    // hi2c: I2C-Handle (hier hi2c4).
    // shunt_milliohm: EFFEKTIVER Shunt-Widerstand in Milliohm, wie ihn der INA226 zwischen
    // seinen IN+/IN--Pins tatsaechlich sieht -- nicht zwingend der Nennwert des Bauteils! Bei so
    // kleinen Werten (hier: 2 mOhm Nennwert, Vishay WSLP1206) schlaegt selbst bei sorgfaeltiger
    // Kelvin-Anbindung zusaetzlicher Pad-/Via-/Leiterbahnwiderstand zwischen den echten
    // Kelvin-Abgriffen und dem Widerstandskoerper prozentual stark durch. Deshalb double statt
    // eines ganzzahligen mOhm-Werts -- erlaubt eine empirische Fein-Kalibrierung per
    // Referenzmessung (Multimeter/Netzteil-Anzeige vs. Firmware-Wert), s. Aufrufer in power.hh.
    // max_expected_current_ma: bestimmt Aufloesung/Kalibrierung (Current_LSB = max/32768, s.
    // Datenblatt-Formel CAL = 0.00512 / (Current_LSB * Rshunt)).
    // address7: 7-bit-I2C-Adresse (0x40 bei A0=A1=GND).
    INA226(I2C_HandleTypeDef *hi2c, double shunt_milliohm, uint32_t max_expected_current_ma,
           uint8_t address7 = 0x40)
        : hi2c_(hi2c), shunt_milliohm_(shunt_milliohm), max_expected_current_ma_(max_expected_current_ma),
          addr8_(static_cast<uint8_t>(address7 << 1)) {}

    // Prueft die Manufacturer-ID (0xFE, muss 0x5449 = "TI" liefern) -- ohne Seiteneffekte auf
    // die Konfiguration, damit ein fehlender Sensor sicher erkannt werden kann.
    bool Probe();

    // Setzt Config (Averaging=4, Conversion-Times=Default 1.1ms, kontinuierlich Shunt+Bus) sowie
    // Kalibrierung. Kein Alerting (Mask/Enable, Alert-Limit) -- bewusst entfernt (s. History:
    // Verdacht, dass diese Schreibvorgaenge die anschliessenden Mess-Register-Reads (0x01-0x04)
    // dauerhaft NACKen liessen).
    bool Init();

    // Liest Bus-/Shunt-Spannung, Strom und Leistung. Rueckgabe false bei I2C-Fehler.
    bool Read(Measurement *out);

    // Diagnose-Hilfsfunktion (auf Nutzeranfrage): liest Manufacturer-ID (0xFE, bereits in
    // Probe() erfolgreich gelesen) und Die-ID (0xFF) erneut -- falls Read() fehlschlaegt, zeigt
    // dies, ob der I2C-Bus/die Adresse generell noch funktioniert (diese beiden Register lesbar)
    // oder ob das Problem breiter geworden ist (auch diese NACKen jetzt).
    bool ReadDiagnosticIds(uint16_t *manufacturer_id, uint16_t *die_id);

    // Erzwingt einen sauberen I2C4-Peripherie-Zustand (DeInit()+Init()) -- automatisch als
    // allererster Schritt in Probe() (s. .cpp: TOF3-NACK beim Setup() legt den Bus lahm), aber
    // auch OEFFENTLICH aufrufbar, um den Bus nach einem Laufzeit-Fehlschlag (z.B. Timeout durch
    // ETH-DMA-Interrupt-Verdraengung, aehnlich dem HX711-Fall) vor dem naechsten Versuch
    // zurueckzuerobern -- s. Aufrufer in power.hh.
    bool ReclaimI2CBus();

private:
    I2C_HandleTypeDef *hi2c_;
    double shunt_milliohm_;
    uint32_t max_expected_current_ma_;
    uint8_t addr8_;
    double current_lsb_ma_ = 0;
    double power_lsb_mw_ = 0;

    bool writeReg16(uint8_t reg, uint16_t value);
    bool readReg16(uint8_t reg, uint16_t *value);
};

} // namespace ina226
