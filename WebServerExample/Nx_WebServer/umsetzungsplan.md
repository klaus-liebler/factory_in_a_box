# Modbus-Register-Map für Sensoren/Aktuatoren der Control-Unit

## Context

`Nx_WebServer` läuft aktuell auf einem **Eval-Board (STM32H573IIKx)** und dient dazu, die grundlegende Funktionalität des Modbus-TCP-Servers (Verbindungsaufbau, FC03/04/06/16) zu verifizieren. Die eigentliche Ziel-Hardware ist die **Control-Unit-Platine** (`firmware_control_unit_ethercat`, **STM32H563VITx**), auf der 26 reale Sensoren/Aktuatoren (GPIO-direkt und bus-basiert: I2C, SPI, UART, PWM/Timer, FDCAN) angebunden sind. Diese sollen künftig ausschließlich über **Input- und Holding-Register** angesprochen werden (keine Coils/Discrete Inputs – deckt sich mit dem bereits implementierten Funktionsumfang, der ohnehin nur FC03/04/06/16 kennt).

Die Pin-/Peripherie-Zuordnung wurde direkt aus `firmware_control_unit_ethercat` (main.c/main.h/msp.c) verifiziert – das ist die verbindliche Quelle, nicht Annahmen meinerseits. Gleiche STM32H5-Familie wie das Eval-Board, aber anderes Package (LQFP100 vs. UFBGA176) – Peripherie-IP und AF-Belegung sind innerhalb der Familie identisch, einzelne Pins können auf dem Eval-Board aber schlicht nicht herausgeführt sein.

## Bereits getroffene Entscheidungen (mit dir abgestimmt)

1. **Bool-Werte** (Lichtschranken, Ventile, Stepper-Enable): je **1 eigenes Register** (0/1), kein Bitpacking.
2. **Adressraum**: blockweise pro Funktionsgruppe mit Reserve für spätere Erweiterung.
3. **TMC2209-UART-Konfiguration** (Mikroschritte, Strom): fest im Code beim Boot gesetzt, **nicht** über Modbus einstellbar.
4. **Stepper-Modell**: voller Funktionsumfang – Zielposition, Geschwindigkeit/Beschleunigung (Holding) + Istposition, Status (Input).
5. **Treiber-Reuse**: TMC2209- und HX711-Treiber aus `firmware_stm32_control_unit` werden portiert statt neu geschrieben.
6. **Vorgehen**: stufenweise Implementierung statt Big-Bang.
7. **PC8-Konflikt geklärt**: Auf der echten Platine ist PC8 = `LIGHTBARRIER3`, es gibt dort **keine SD-Karte/FileX**. Der SDMMC1/FileX-Code in `Nx_WebServer` ist Eval-Board-spezifisch (dient dem Website-Hosting-Test) und hat mit der Ziel-Platine nichts zu tun. **Auf dem Eval-Board bleibt PC8 daher bei SDMMC1** (Website-Hosting nicht antasten) – Lichtschranke 3 wird auf dem Eval-Board vorerst **nicht verdrahtet/getestet**, der Registerplatz wird aber reserviert und funktioniert auf der echten Platine.
8. **WS2812**: pro Kanal genau ein Holding-Register = Index in eine fest im Code hinterlegte Farbmuster-Tabelle (kein Streaming einzelner LED-Farben über Modbus). Konkrete LED-Anzahl pro Kanal folgt erst bei der WS2812-Implementierung (Stage 3) als Firmware-Konstante.

### Zusätzliche technische Entscheidung (aus der Recherche abgeleitet, nicht separat rückgefragt, da direkte Konsequenz aus Punkt 4)

**Stepper-Ansteuerung läuft über STEP/DIR-Timer-Pulse (TIM2_CH1/PA15 + TIM3_CH1/PB4, DIR PD6/PD7), nicht über die VACTUAL-UART-Velocity-Steuerung des portierten TMC2209-Treibers.** Begründung: Der TMC2209 hat kein auslesbares Positionsregister; „volle Istposition + Zielposition" (Punkt 4) ist nur möglich, wenn die MCU selbst die Schritte per Timer erzeugt und mitzählt. Die UART-Verbindung (UART5, PB5/PB6) wird dann nur noch für Boot-Konfiguration (Mikroschritte, Strom) und Status-Polling (StallGuard, Temperatur) genutzt – die im Treiber vorhandene `GenerateSteps()`/VACTUAL-Funktion wird nicht verwendet.

## Register-Map

Alle Adressen sind Register-Offsets (0-basiert) innerhalb der bestehenden `holding_registers[1024]` / `input_registers[1024]` Arrays. 32-Bit-Werte belegen zwei Register (High-Word zuerst, Big-Endian, wie in `MbapHeader::serialize` bereits Konvention).

### Input-Register (FC04, read-only)

| Adresse | Block | Inhalt |
|---|---|---|
| 0 | **Diagnostik** (Basis 0, Reserve bis 49) | HealthState (Bitfeld) |
| 1–6 | | Chip-ID, 3×32-Bit (`HAL_GetUIDw0/1/2`, `UID_BASE`) |
| 7–9 | | FW-Version Major/Minor/Patch |
| 50–79 | **CAN-Bus-Statistik** | 50–51 TX-Frames (32-Bit), 52–53 RX-Frames, 54–55 Error-Count, 56 Last-Error-Code, 57 Bus-State |
| 80–109 | **Ethernet-Statistik** (LAN8742) | 80 Link-Status, 81 Speed, 82 Duplex, 83–84 TX-Frames, 85–86 RX-Frames, 87–88 Error-Count, 89 Last-Error-Code |
| 110–139 | **Stromversorgung** (INA226 @ I2C_4 + USB-PD) | 110 Bus-Voltage (mV), 111 Shunt-Voltage (µV, signed), 112 Current (mA, signed), 113 Power (mW), 114 PD-Voltage (mV), 115 PD-Current (mA), 116 PD-Status |
| 140–159 | **ToF-Sensoren** | 140/141 TOF1 (PC15 IRQ) Distanz+Status, 142/143 TOF2 (PH1 IRQ), 144/145 TOF3 (PA3 IRQ) |
| 160–179 | **Farbsensor TCS34725** (I2C_1, IRQ PH0) | 160 Clear, 161 Red, 162 Green, 163 Blue |
| 180–199 | **Lichtschranken** | 180 LS1 (PC6), 181 LS2 (PC7), 182 LS3 (PC8, s.o. auf Eval-Board nicht verdrahtet) |
| 200–219 | **Drucksensor** (ADC1 CH10, PA4) | 200 Druck (skaliert oder Rohwert – Skalierung TBD) |
| 220–239 | **Wägezelle HX711** (SPI2, PC2/PC3) | 220–221 Gewicht Kanal A (32-Bit signed), 222 Status A, 223–224 Kanal B Rohwert (Reserve/Erweiterung), 225 Status B |
| 240–269 | **Stepper-Status** | 240–241 Stepper1 Istposition (32-Bit signed), 242 Stepper1 Status (Bit0 Enabled, Bit1 Moving, Bit2 Error/StallGuard, Bit3 InPosition), 243–244 Stepper2 Istposition, 245 Stepper2 Status |

### Holding-Register (FC03/06/16, read/write)

| Adresse | Block | Inhalt |
|---|---|---|
| 0–19 | reserviert | (künftige Konfiguration, z.B. Watchdog) |
| 20–39 | **Pneumatikventile** | 20 Ventil1 (PD10), 21 Ventil2 (PD5), 22 Ventil3 (PD11) |
| 40–59 | **Motor-PWM** | 40 Förderband (TIM4_CH3/PD14), 0–1000 ‰ Duty; 41 Kompressor (TIM4_CH4/PD15), 0–1000 ‰ |
| 60–99 | **Stepper-Steuerung** | 60 Global-Enable (PB7), 61–62 Stepper1 Zielposition (32-Bit signed), 63 Geschwindigkeit, 64 Beschleunigung, 65–66 Stepper2 Zielposition, 67 Geschwindigkeit, 68 Beschleunigung |
| 100–109 | **WS2812** | 100 Kanal1 Farbmuster-Index (PE5/TIM15_CH1), 101 Kanal2 Farbmuster-Index (PE6/TIM15_CH2) |

Alle Adresskonstanten kommen in eine neue Datei **`Core/Src/modbus_register_map.hpp`** (benannte `constexpr uint16_t`, gruppiert wie oben), damit Firmware-Code und diese Dokumentation nie auseinanderlaufen.

## Architektur-Änderungen

1. **Thread-Sicherheit (Voraussetzung für alles Weitere):** `modbus_tcp_server.hpp` hat aktuell **keinerlei Locking** um `m_registers` – weder in den öffentlichen Accessor-Methoden noch in `process_modbus_request` (FC03/04/06/16 greifen direkt auf die Arrays zu). Da künftig I/O-Threads dieselben Arrays parallel beschreiben/lesen, wird ein `TX_MUTEX m_register_mutex` Member ergänzt (erzeugt in `initialize()`), das in den vier Accessor-Methoden **und** in den FC03/04/06/16-Blöcken von `process_modbus_request` per `tx_mutex_get`/`tx_mutex_put` genutzt wird.

2. **Neuer periodischer I/O-Thread** (`io_thread`, ThreadX, Priorität niedriger als `modbus_server_thread` (5), z.B. 11 wie `link_thread`): liest zyklisch (~50–100 ms) digitale Eingänge/ADC und schreibt Input-Register; liest Holding-Register für Ventile/Motor-PWM und setzt GPIO/PWM. Analog zum bestehenden `link_thread`-Muster in `app.cpp`.

3. **Diagnostik-Modul** (`Core/Src/diagnostics.hpp`): einmalige Chip-ID/Version-Befüllung beim Start, HealthState-Aggregation zyklisch aus dem I/O-Thread.

4. **Treiber-Ports** (aus `firmware_stm32_control_unit`):
   - `TMC2209.hpp/.cpp` + `tmc2209_reg.hpp` → UART5-Register-I/O für Boot-Konfig/Status (nicht für Motion, s.o.). `gpio.hh`-Abhängigkeit wird **nicht** mitportiert (nur für STM32G4 gebaut) – stattdessen direkte `HAL_GPIO_*`-Aufrufe passend zum bestehenden Stil in `app.cpp`.
   - `hx711.hh` (SPI+DMA-Variante, passt zu SPI2 MISO/MOSI PC2/PC3) → Kanal A Wägezelle; Kanal B vorerst nur Rohwert-Passthrough.

5. **Neue Peripherie-Initialisierung** (aktuell in `Nx_WebServer` nicht vorhanden, muss neu geschrieben werden, orientiert an den in `firmware_control_unit_ethercat/Core/Src/main.c` verifizierten Settings): I2C1/I2C2/I2C4, ADC1 (CH10/PA4), GPIO für PC6/PC7/PD10/PD5/PD11/PB7/PD6/PD7, TIM2/TIM3 (Step-Pulse), TIM4 (Motor-PWM), TIM15+DMA (WS2812), UART5 (115200 8N1), SPI2 (HX711), FDCAN1.

## Gestuftes Vorgehen

**Stage 1 (jetzt umzusetzen):**
- `modbus_register_map.hpp` mit **allen** Adresskonstanten (auch für später kommende Blöcke – Adressraum steht von Anfang an fest).
- Mutex-Absicherung in `modbus_tcp_server.hpp`.
- Diagnostik: HealthState (Basisversion), Chip-ID (echt, sofort testbar), Version (Compile-Time-Konstante).
- Ethernet-Link-Status (echt nutzbar, da ETH auf dem Eval-Board bereits aktiv ist für den Webserver).
- Digitale I/O ohne Pin-Konflikt: Lichtschranke 1/2 (PC6/PC7), Ventile 1–3 (PD10/PD5/PD11), Drucksensor (ADC1/PA4). Lichtschranke 3 (PC8) bleibt Register-seitig reserviert, aber auf dem Eval-Board unverdrahtet (s. Punkt 7 oben).
- Neuer `io_thread` inkl. Mutex-Nutzung.

**Stage 2 (Folge-Schritt, eigene Planungsrunde):** I2C-Sensoren – 3× VL53L0X, TCS34725, INA226 (I2C1/I2C2/I2C4-Init + minimale Register-Level-Treiber, da keine vorhandenen Treiber im Repo).

**Stage 3 (Folge-Schritt):** Motion – Motor-PWM (TIM4), Stepper (TIM2/TIM3 Step/Dir + portierter TMC2209 für UART-Konfig/Status), WS2812 (TIM15+DMA, Farbmuster-Tabelle inkl. konkreter LED-Anzahl pro Kanal).

**Stage 4 (Folge-Schritt):** CAN-Statistik (FDCAN1-Init neu, da im Eval-Board-Projekt noch nicht vorhanden), HX711 (SPI2+DMA-Port), finale HealthState-Aggregation über alle Subsysteme.

## Offene Punkte / Caveats (nicht blockierend für Stage 1)

- **Drucksensor-Skalierung:** rohe ADC-Counts oder physikalische Einheit (mbar) – Kennlinie/Sensortyp noch nicht spezifiziert, Stage 1 liefert zunächst Rohwert.
- **Ethernet-Frame-Zähler** (TX/RX/Error, Adressen 83–89): welche NetXDuo/Treiber-API dafür konkret genutzt wird (IP-Interface-Zähler vs. `nx_stm32_eth_driver`-interne Statistik) klärt sich beim Implementieren; Link-Status/Speed/Duplex (80–82) sind in jedem Fall verfügbar.
- **UART5-Modus:** Board nutzt Standard-TX/RX (zwei Pins, PB5/PB6), nicht Single-Wire-Halfduplex – vermutlich per Widerstand extern zusammengeführt; wird beim TMC2209-Port (Stage 3) verifiziert.

## Verifikation

- Stage 1 lässt sich auf dem vorhandenen Eval-Board bauen und über den bereits funktionierenden Modbus-TCP-Server testen: FC04 auf Adresse 0–9 (HealthState/ChipID/Version), 80–82 (Ethernet-Link), 180–181 (Lichtschranken), 200 (Druck-Rohwert); FC03/FC06/FC16 auf 20–22 (Ventile) zum Schalten testen (LED/Multimeter am jeweiligen Pin, falls am Eval-Board herausgeführt – sonst Verifikation der reinen Registerlogik per Modbus-Client, z.B. `mbpoll`).
- Build via bestehendem CMake-Preset (`cube-cmake --build .../build/Debug`), wie in den vorherigen Fixes verwendet.
