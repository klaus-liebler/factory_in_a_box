#pragma once
// ============================================================================
// CPU-Temperatur -- STM32H563 interner DTS (Digital Temperature Sensor, RM0481 Kapitel 29),
// NICHT der klassische ADC-basierte Temperatursensor anderer STM32-Familien (der existiert auf
// diesem Chip zwar formal als ADC_CHANNEL_TEMPSENSOR, ist laut Datenblatt aber nicht der
// eigentliche Sensor -- DTS ist die vorgesehene Messmethode).
//
// DTS wandelt die Temperatur in ein Rechtecksignal um, dessen Frequenz proportional zur
// Temperatur ist, und zaehlt PCLK1-Zyklen waehrend mehrerer Perioden dieses Signals
// (REFCLK_SEL=0, PCLK-Modus -- der LSE-Modus wuerde einen in diesem Projekt nicht konfigurierten
// 32.768kHz-Quarz voraussetzen). Waehrend der (automatischen) Kalibrierung vor jeder Messung
// muss der interne Zaehler-Takt unter 1MHz liegen (RM0481 29.3.6) -- HSREF_CLK_DIV teilt PCLK1
// dafuer herunter (max. Teiler 127). Bei PCLK1=80MHz (s. main.c SystemClock_Config(), APB1-
// Teiler=2) ergibt der maximale Teiler ~630kHz, also innerhalb der Vorgabe.
//
// T0VALR1/RAMPVALR sind werksseitig JE CHIP kalibrierte Register (nicht nur Datenblatt-
// Typwerte) -- die Formel (RM0481 29.3.7, PCLK-Modus, aus den (klar beschriebenen, nicht nur
// aus der z.T. unklaren Formel-Textdarstellung selbst abgeleiteten) Registereinheiten
// hergeleitet):
//   FM(T)[Hz]  = PCLK1 * TS1_SMP_TIME / TS1_MFREQ   (TS1_MFREQ PCLK-Zyklen laufen waehrend
//                TS1_SMP_TIME FM(T)-Perioden ab, s. 29.3.7 "counting of REF_CLK cycles ...
//                during one or several FM(T) cycles")
//   FMT0[Hz]   = TS1_FMT0 * 100                     (TS1_FMT0 ist in 0.1kHz-Einheiten)
//   T[C]       = T0 + (FM(T) - FMT0) / TS1_RAMP_COEFF   (TS1_RAMP_COEFF direkt in Hz/C)
//
// Kein ST-HAL-Treiber fuer DTS in diesem Paket vorhanden -- direkter Registerzugriff ueber die
// CMSIS-Definitionen (DTS_TypeDef, DTS_CFGR1_*, ...).
// ============================================================================
#include "interfaces.hh"
#include "modbus_register_model.hh"
#include "log.h"
#include "main.h"
#include "hw_config_assert.hh"

class CpuTempSetupAndLoop : public ISetupAndLoop {
    private:
    Modbus::IModbusRegisterModel& register_model;

    // TS1_SMP_TIME-Registerwert (nicht 0, s. RM0481 Tabelle 298: 0 und 1 bedeuten beide "1
    // Zyklus" -- 7 ergibt 7 Zyklen, mehr Praezision als der Default (1 Zyklus) bei noch kurzer
    // Messzeit).
    static constexpr uint32_t SMP_TIME_REG_VALUE = 7;
    static constexpr uint32_t DTS_READY_TIMEOUT_MS = 10;
    // Maximaler HSREF_CLK_DIV-Teiler laut RM0481 ist 127 (7-Bit-Register) -- oberhalb von
    // 127MHz PCLK1 koennte der Kalibrierungstakt selbst mit dem groesstmoeglichen Teiler nicht
    // mehr unter die von RM0481 29.3.6 geforderte 1MHz-Grenze gebracht werden (s.
    // Klassenkommentar oben, aktuell PCLK1=80MHz, also unkritisch).
    static constexpr uint32_t MAX_PCLK1_FOR_DTS_CALIBRATION_HZ = 127000000u;

    // Teiler, um PCLK1 waehrend der Kalibrierung unter 1MHz zu bringen (Registerwert = Teiler
    // direkt, 0 und 1 bedeuten beide "kein Teiler" -- s. RM0481 HSREF_CLK_DIV-Beschreibung).
    static uint32_t ComputeHsrefClkDiv() {
        uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
        uint32_t divider = (pclk1_hz + 999999u) / 1000000u; // aufrunden auf <1MHz
        if (divider < 1u) {
            divider = 1u;
        }
        if (divider > 127u) {
            divider = 127u; // Registerbreite 7 Bit -- mehr geht nicht (s. Klassenkommentar)
        }
        return divider;
    }

    static bool WaitReady(uint32_t timeout_ms) {
        uint32_t start = HAL_GetTick();
        while ((DTS->SR & DTS_SR_TS1_RDY) == 0) {
            if ((HAL_GetTick() - start) > timeout_ms) {
                return false;
            }
        }
        return true;
    }

    public:
    CpuTempSetupAndLoop(Modbus::IModbusRegisterModel& model) : register_model(model) {}

    void Setup() override {
        HW_CONFIG_ASSERT(HAL_RCC_GetPCLK1Freq() < MAX_PCLK1_FOR_DTS_CALIBRATION_HZ,
                          "DTS: PCLK1 zu hoch fuer den maximalen HSREF_CLK_DIV-Teiler (127) -- "
                          "Kalibrierungstakt waere >=1MHz (RM0481 29.3.6)");

        __HAL_RCC_DTS_CLK_ENABLE();

        const uint32_t hsref_div = ComputeHsrefClkDiv();
        // Volle Zuweisung statt OR: stellt sicher, dass TS1_INTRIG_SEL=0 (Software-Trigger,
        // kein Hardware-Trigger), REFCLK_SEL=0 (PCLK) und Q_MEAS_OPT=0 (volle Kalibrierung --
        // Quick-Measure ist laut RM0481 nur im LSE-Modus zulaessig) definiert gesetzt sind.
        DTS->CFGR1 = (hsref_div << DTS_CFGR1_HSREF_CLK_DIV_Pos) |
                     (SMP_TIME_REG_VALUE << DTS_CFGR1_TS1_SMP_TIME_Pos);
        DTS->CFGR1 |= DTS_CFGR1_TS1_EN;

        if (!WaitReady(DTS_READY_TIMEOUT_MS)) {
            log_warn("DTS (CPU-Temperatursensor): TS1_RDY nach Enable nicht gesetzt -- CPU_TEMPERATURE_C bleibt unveraendert");
        }
    }

    void Loop(uint32_t now) override {
        (void)now;
        if ((DTS->SR & DTS_SR_TS1_RDY) == 0) {
            // Vorherige Messung/Kalibrierung noch nicht fertig (oder Sensor kam in Setup() nie
            // bereit) -- kein Fehlerzustand, einfach naechsten Zyklus erneut versuchen.
            return;
        }

        DTS->CFGR1 |= DTS_CFGR1_TS1_START;
        if (!WaitReady(DTS_READY_TIMEOUT_MS)) {
            log_warn("DTS (CPU-Temperatursensor): Messung nach Start nicht abgeschlossen (Timeout)");
            return;
        }

        const uint32_t mfreq = (DTS->DR & DTS_DR_TS1_MFREQ_Msk) >> DTS_DR_TS1_MFREQ_Pos;
        if (mfreq == 0) {
            return; // Division durch 0 vermeiden -- sollte bei TS1_RDY=1 nicht vorkommen
        }

        const uint32_t smp_time_reg = (DTS->CFGR1 & DTS_CFGR1_TS1_SMP_TIME_Msk) >> DTS_CFGR1_TS1_SMP_TIME_Pos;
        const uint32_t smp_time = (smp_time_reg == 0) ? 1u : smp_time_reg; // s. RM0481 Tabelle 298
        const uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();

        const double fmt_hz = (double)pclk1_hz * (double)smp_time / (double)mfreq;

        const uint32_t fmt0_raw = (DTS->T0VALR1 & DTS_T0VALR1_TS1_FMT0_Msk) >> DTS_T0VALR1_TS1_FMT0_Pos;
        const uint32_t t0_code = (DTS->T0VALR1 & DTS_T0VALR1_TS1_T0_Msk) >> DTS_T0VALR1_TS1_T0_Pos;
        const double fmt0_hz = (double)fmt0_raw * 100.0; // TS1_FMT0 in 0.1kHz-Einheiten
        const double ramp_coeff = (double)((DTS->RAMPVALR & DTS_RAMPVALR_TS1_RAMP_COEFF_Msk) >> DTS_RAMPVALR_TS1_RAMP_COEFF_Pos); // Hz/C
        const double t0_c = (t0_code == 1) ? 130.0 : 30.0; // RM0481 TS1_T0: 00=30C, 01=130C

        if (ramp_coeff == 0.0) {
            return; // unprogrammierte/leere Kalibrierregister -- Division durch 0 vermeiden
        }
        const double temp_c = t0_c + (fmt_hz - fmt0_hz) / ramp_coeff;

        register_model.SetInputRegister(ModbusRegisters::Input::CPU_TEMPERATURE_C, (uint16_t)(int16_t)temp_c);
    }
};
