# Firmware STM32 Control Unit

Kurzer Startpunkt fuer Doku und Onboarding. Bitte nach Bedarf erweitern.

## Projektuebersicht
- Ziel: Firmware fuer STM32G431-basierte Steuerungseinheit.
- Tooling: CMake Presets, ARM-GCC Toolchain, STM32Cube HAL.
- Hauptquellen: Core/ (Anwendung), Drivers/ (HAL/CMSIS), cmake/ (Toolchain-Konfiguration).

## Voraussetzungen
- ARM GCC Toolchain (gcc-arm-none-eabi) im PATH.
- CMake 3.21+ und Ninja oder Make.
- Python optional fuer Skripte/Generatoren.
- STM32CubeProgrammer zum Flashen.

## Schnellstart (Build)
1. Build-Verzeichnis anlegen: `mkdir build && cd build`
2. Preset konfigurieren (Beispiel): `cmake .. --preset default`
3. Bauen: `cmake --build --preset default`

> Hinweis: Passen Sie Preset-Namen ggf. an `CMakePresets.json` an.

## Flashen
- Mit STM32CubeProgrammer oder `st-flash` auf das Ziel schreiben.
- Ziel-Binary nach Build: `build/Core/app.elf` (pfad ggf. pruefen/anpassen).
- Beispiel `st-flash --reset write build/Core/app.bin 0x08000000` (Adresse ggf. anpassen).

## Projektstruktur
- Core/Inc: Oeffentliche Header der Anwendung
- Core/Src: Anwendungscode (C/C++)
- common_stm32/: Gemeinsame STM32-Hilfsfunktionen (GPIO, Register-Access, etc.)
- TMC2209/: Stepper-Motor-Treiber-Bibliothek (nutzt common_stm32)
- Drivers/: STM32 HAL und CMSIS
- cmake/: Toolchain-Dateien und CubeMX-Integration
- firmware_stm32_control_unit.ioc: CubeMX Projektdatei

## Konfiguration
- HAL/Clock/Pin-Setup in firmware_stm32_control_unit.ioc pflegen und generieren.
- Toolchain-Einstellungen in cmake/gcc-arm-none-eabi.cmake
- Linkerskript: STM32G431XX_FLASH.ld

## C++-Support
- Aktiv: Projekt baut C und C++ (C++17).
- C++-Dateien liegen z.B. in `Core/Src/app.cc`; Header in `Core/Inc`.
- Flags: ohne Exceptions/RTTI (`-fno-exceptions -fno-rtti`) per Toolchain gesetzt.
- Weitere C++-Quellen: in `CMakeLists.txt` unter `target_sources()` ergänzen.

## TMC2209-Bibliothek
- Ort: `TMC2209/` (statische Lib `TMC2209`).
- Einbindung: bereits in `CMakeLists.txt` verlinkt (`target_link_libraries(... TMC2209)`).
- GPIO-Steuerung: nutzt `common_stm32::Gpio` (siehe unten).
- **Type-Safe Register Access**: Alle Register-Parameter nutzen `enum class Reg`

Funktionstest bei Startup:
- `app_setup()` scannt alle 4 möglichen IDs (0–3) an `huart3`
- Versucht, **GSTAT** und **IOIN** Register zu lesen
- Gibt Erfolg/Fehler über UART2 aus
- Keine Identifikations-Nummer nötig; TMC2209 identifiziert sich durch gültigen CRC bei Register-Lesezugriff

Beispielcode:
```c++
#include "TMC2209.hpp"
extern UART_HandleTypeDef huart3;

void motor_init() {
    // Erstelle Driver mit UART3, Adresse 0x00, und EN-Pin (PB12)
    tmc2209::TMC2209 driver(&huart3, 0x00, Pin::PB12);
    driver.enable();
    
    // Setze Hold/Run Strom (je 0..31) und Hold-Delay (0..15)
    driver.setIHoldIRun(8, 16, 4);
    
    // Schreibe/lese Register (Type-safe mit enum class Reg)
    driver.writeRegister(tmc2209::Reg::CHOPCONF, 0x10001c44);
    uint32_t status = 0;
    driver.readRegister(tmc2209::Reg::DRV_STATUS, status);
}
```

## common_stm32 Utilities
- Ort: `common_stm32/` (Header-only Library).
- Inhalt: GPIO-Funktionen, Register-Bit-Manipulation.
- Kernklasse: `Gpio` (siehe `gpioG4.hh`).

Beispiele:
```c++
#include "gpioG4.hh"

// Pin als Ausgang konfigurieren
Gpio::ConfigureGPIOOutput(Pin::PB12, false); // low = enabled

// Pin setzen/lesen
Gpio::Set(Pin::PB12, true);
bool state = Gpio::Get(Pin::PB12);
Gpio::Toggle(Pin::PA00);
```

## Coding-Guidelines
- Sprache: C/C++ (C++17 falls genutzt).
- Nutze const, constexpr wo moeglich; keine dynamische Allokation im Laufzeitpfad.
- Interrupt-Handler in Core/Src/stm32g4xx_it.c halten.
- Kurze Kommentare nur bei nicht offensichtlicher Logik.

## Tests und Verifikation
- Unit-Tests: noch offen (TODO: Rahmen festlegen).
- HW-Tests: Smoke-Test-Checkliste dokumentieren (TODO).

## Fehlersuche
- Serielles Logging konfigurieren (TODO: Port/Baudrate definieren).
- SWD aktiv lassen; nutze gdb/pyOCD/openocd nach Bedarf.

## Release-Notizen
- Fuehre Changelog pro Release (TODO: Pfad definieren, z.B. CHANGELOG.md).

## Offene Punkte
- Flash/Reset-Workflow finalisieren.
- CI-Pipeline definieren (Build + Format + Static Analysis?).
- Dokumentation fuer Pinout und Schnittstellen ergaenzen.
