#pragma once
// ============================================================================
// Stepper -- zwei Achsen (STEPPER1/STEPPER2), je ein TMC2209-Treiber-IC am gemeinsamen
// UART5-Bus (STEPPER_RX/TX, Knotenadressen per MS1/MS2-Bruecken auf dem Board: Stepper1 =
// 0b10, Stepper2 = 0b00) fuer Konfiguration (Mikroschritt-Aufloesung, Strom), plus je ein
// SigmoidStepper fuer die eigentliche STEP/DIR-Pulserzeugung mit S-Kurven-Beschleunigung
// (siehe stm32_libs/sigmoid_stepper/sigmoid_stepper.hh).
//
// SigmoidStepper braucht einen eigenen Hardware-Timer pro gleichzeitig aktiver Achse (der
// Timer erzeugt die Step-Pulse per Update-Interrupt, unabhaengig vom io_thread-Zyklus) --
// STEPPER1 nutzt TIM17, STEPPER2 TIM16 (beide bereits per CubeMX als Basic-Timer
// konfiguriert, aber ohne NVIC-Interrupt-Freigabe; die wird hier manuell nachgeholt, s.
// SetupEarly()). Die TIM16/TIM17-IRQ-Handler sind freie extern-"C"-Funktionen (von HAL/dem
// Vektortable so vorgegeben) und erreichen die Motion-Objekte ueber einen statischen
// Selbstzeiger (instance_) -- es gibt ohnehin nur eine StepperSetupAndLoop-Instanz (angelegt
// einmalig in App::SetupBeforeThreadX(), s. app.cc).
//
// SetupEarly() (TMC2209-UART-Init, IHOLD_IRUN, Timer-Init, NVIC-Freigabe) laeuft bewusst NICHT
// aus dem IO-Thread, sondern schon in App::SetupBeforeThreadX() -- also bare-metal vor
// tx_kernel_enter(), noch ohne ThreadX-Scheduler/-Tick. Die interruptgetriebenen UART-Reads in
// tmc2209.cpp (HAL_UART_Transmit_IT/Receive_IT + Busy-Wait auf gState/RxState) liefen aus dem
// ThreadX-IO-Thread heraus unzuverlaessig (erster Read ok, alle folgenden liefen in TX/RX-
// Timeouts) -- vor tx_kernel_enter() nicht. Analog zu USBPDControl::EarlySetup(), s. app.cc.
// Deshalb konstruiert App dieses Objekt selbst (nicht Io) und reicht es Io nur per Referenz
// durch (s. io.hpp) -- Setup() (die ISetupAndLoop-Schnittstelle, aus dem IO-Thread aufgerufen)
// ist hier bewusst ein No-Op.
//
// profile_100_1000_5_160mhz wurde per stm32_libs/sigmoid_stepper/sigmoid_approximation.py
// --cpu-freq 160e6 fuer den tatsaechlichen TIM16/TIM17-Takt dieses Boards erzeugt (APB2 x2
// wegen Prescaler != 1, siehe .ioc: APB2TimFreq_Value=160000000) -- die im Profil
// "gelabelten" 100..1000 Schritte/s entsprechen damit den tatsaechlich erreichten
// Geschwindigkeiten.
//
// STEPPER1_SPEED/STEPPER1_ACCEL bzw. STEPPER2_SPEED/STEPPER2_ACCEL (Holding-Register) werden
// aktuell NICHT ausgewertet: profile_100_1000_5_160mhz ist ein fest vorberechnetes Profil
// ohne Laufzeit-Parametrisierung von Zielgeschwindigkeit/Beschleunigung. Nur Zielposition
// (STEPPER1_TARGET_*) und globale Freigabe (STEPPER_ENABLE) sind wirksam.
//
// Homing per Modbus: STEPPERn_START_HOMING (Holding) -- 1 schreiben stoppt die normale
// Positionsregelung dieser Achse und startet sensorloses Homing per StallGuard
// (TMC2209::StartHomingNonBlocking()/TickHoming(), s. stm32_libs/tmc2209). Nicht blockierend:
// TickHoming() wird einmal pro Loop()-Zyklus aufgerufen, damit der gemeinsame IO-Thread
// waehrenddessen weiter Ventile/PWM/andere Sensoren bedienen kann (anders als die blockierende
// HomeSensorless(), die den kompletten Thread fuer die Homing-Dauer angehalten haette). Nach
// erfolgreichem Stall schreibt die Firmware 0 zurueck (SigmoidStepper::SetCurrentPosition(0)),
// bei Timeout/Fehler einen Code >1. Beide Achsen teilen sich ein einziges Enable (STEPPER_EN,
// s. Loop()) -- waehrend eine der beiden Achsen homt, wird dieses gemeinsame Enable zusaetzlich
// zum STEPPER_ENABLE-Register erzwungen (sonst koennte der Treiber disabled sein und gar nicht
// fahren).
// ============================================================================
#include "interfaces.hh"
#include "modbus_register_model.hh"
#include "log.h"
#include "main.h"

#include "tmc2209.hpp"
#include "sigmoid_stepper.hh"
#include "sigmoid_acceleration_profile_100_1000_5_160mhz.hh"

extern "C" UART_HandleTypeDef huart5;

class StepperSetupAndLoop : public ISetupAndLoop {
    public:
    // Selbstzeiger fuer TIM16_IRQHandler/TIM17_IRQHandler (freie extern-"C"-Funktionen, s.
    // unten) -- oeffentlich, da von dort aus erreichbar sein muss.
    static StepperSetupAndLoop* instance_;

    private:
    Modbus::IModbusRegisterModel& register_model;

    // Knotenadressen per MS1/MS2-Bruecken auf dem Board: Stepper1 = 0b10 (2), Stepper2 = 0b00 (0).
    tmc2209::TMC2209 tmc_stepper1{&huart5, 0b10, gpio::Pin::NO_PIN};
    tmc2209::TMC2209 tmc_stepper2{&huart5, 0b00, gpio::Pin::NO_PIN};

    SigmoidStepper<gpio::Peripheral::TIM17_> stepper1_motion{gpio::Pin::PA15, gpio::Pin::PD06, &profile_100_1000_5_160mhz};
    SigmoidStepper<gpio::Peripheral::TIM16_> stepper2_motion{gpio::Pin::PB04, gpio::Pin::PD07, &profile_100_1000_5_160mhz};

    bool homing1_active_ = false;
    bool homing2_active_ = false;

    // Gemeinsame Homing-Ablaufsteuerung fuer eine Achse -- Motion ist
    // SigmoidStepper<TIM17_>/SigmoidStepper<TIM16_> (unterschiedliche Templates je Achse, daher
    // hier als Funktionstemplate statt zweier fast identischer Methoden).
    template <typename Motion>
    static void ProcessHoming(bool& homing_active, tmc2209::TMC2209& tmc, Motion& motion,
                              Modbus::IModbusRegisterModel& register_model, uint16_t homing_register,
                              int homing_speed_full_steps_per_second) {
        if (!homing_active) {
            if (register_model.GetHoldingRegister(homing_register) == 1) {
                homing_active = true;
                tmc.StartHomingNonBlocking(homing_speed_full_steps_per_second);
            }
            return;
        }

        switch (tmc.TickHoming(HAL_GetTick())) {
            case tmc2209::TMC2209::HomingResult::InProgress:
                return;
            case tmc2209::TMC2209::HomingResult::Success:
                motion.SetCurrentPosition(0);
                register_model.SetHoldingRegister(homing_register, 0);
                homing_active = false;
                return;
            case tmc2209::TMC2209::HomingResult::Timeout:
                register_model.SetHoldingRegister(homing_register, 2);
                homing_active = false;
                return;
            case tmc2209::TMC2209::HomingResult::Error:
                register_model.SetHoldingRegister(homing_register, 3);
                homing_active = false;
                return;
        }
    }

    static uint16_t compute_status(bool enabled, const SigmoidStepper<gpio::Peripheral::TIM17_>& s, int32_t target) {
        uint16_t status = 0;
        if (enabled) status |= (1u << 0);
        if (s.IsMotionActive()) status |= (1u << 1);
        // Bit2 (Error/StallGuard) nicht verdrahtet -- kein DIAG-Pin/GSTAT-Polling implementiert.
        if (!s.IsMotionActive() && s.GetCurrentPosition() == target) status |= (1u << 3);
        return status;
    }
    static uint16_t compute_status(bool enabled, const SigmoidStepper<gpio::Peripheral::TIM16_>& s, int32_t target) {
        uint16_t status = 0;
        if (enabled) status |= (1u << 0);
        if (s.IsMotionActive()) status |= (1u << 1);
        if (!s.IsMotionActive() && s.GetCurrentPosition() == target) status |= (1u << 3);
        return status;
    }

    public:
    StepperSetupAndLoop(Modbus::IModbusRegisterModel& model) : register_model(model) {
        instance_ = this;
    }

    void HandleTim17UpdateInterrupt() { stepper1_motion.Handle_update_interrupt_(); }
    void HandleTim16UpdateInterrupt() { stepper2_motion.Handle_update_interrupt_(); }

    // Laeuft bewusst frueh (App::SetupBeforeThreadX(), vor tx_kernel_enter()), nicht als
    // ISetupAndLoop::Setup() aus dem IO-Thread -- s. Klassenkommentar oben.
    void SetupEarly() {
        // Reihenfolge zu Diagnosezwecken umgedreht (stepper2 zuerst): stepper2 hatte vor dem
        // Umbau auf setup_and_loops funktioniert, seitdem schlagen seine UART-Reads konsistent
        // fehl. Falls das an einer Bus-/Timing-Abhaengigkeit von der Initialisierungsreihenfolge
        // liegt (nicht an der Hardware), sollte sich das hier zeigen.
        if (!tmc_stepper2.InitForNormalSpeedAndUartBasedOperation(false, false, tmc2209::MicroStepResolution::RES16)) {
            log_warn("TMC2209 stepper2: UART init failed - STEP/DIR pulses will still be generated, "
                     "but the driver may be running with unconfirmed microstep/current settings");
        }
        if (!tmc_stepper1.InitForNormalSpeedAndUartBasedOperation(false, false, tmc2209::MicroStepResolution::RES16)) {
            log_warn("TMC2209 stepper1: UART init failed - STEP/DIR pulses will still be generated, "
                     "but the driver may be running with unconfirmed microstep/current settings");
        }

        // Standstill-Strom absenken (IHOLD < IRUN): ohne diesen Aufruf blieb der Motor auf dem
        // OTP-Default stehen -- praktisch Volllaststrom auch im Stillstand, daher die spuerbare
        // Erwaermung. Werte wie bereits in PerformStealthChopAutoTuningForQuietOperation() als
        // "leiser/kuehler Betrieb"-Preset dieses Projekts verwendet: IRUN=12/31 fuer die Fahrt,
        // IHOLD=4/31 im Stillstand, IHOLDDELAY=6 (sanftes Herunterfahren nach TPOWERDOWN). Bei
        // spuerbar zu wenig Drehmoment IRUN erhoehen -- die exakt passenden Werte haengen vom
        // tatsaechlichen Motorstrom (RSENSE auf der Platine) ab und wurden hier nicht messtechnisch
        // kalibriert.
        tmc_stepper2.setIHoldIRun(/*ihold=*/4, /*irun=*/12, /*iholddelay=*/6);
        tmc_stepper1.setIHoldIRun(/*ihold=*/4, /*irun=*/12, /*iholddelay=*/6);

        stepper1_motion.Init();
        stepper2_motion.Init();

        NVIC_SetPriority(TIM17_IRQn, 5);
        NVIC_EnableIRQ(TIM17_IRQn);
        NVIC_SetPriority(TIM16_IRQn, 5);
        NVIC_EnableIRQ(TIM16_IRQn);
    }

    // No-Op: die eigentliche Initialisierung laeuft bereits in SetupEarly() (vor
    // tx_kernel_enter(), s. Klassenkommentar oben) -- dennoch als leere Override noetig, um das
    // ISetupAndLoop-Interface zu erfuellen (Io ruft Setup()/Loop() einheitlich fuer alle
    // Mitglieder auf).
    void Setup() override {}

    void Loop(uint32_t now) override {
        (void)now;

        // Sensorloses Homing per StallGuard hat Vorrang vor der normalen Positionsregelung der
        // jeweiligen Achse -- Vorzeichen der Geschwindigkeit bestimmt die Fahrtrichtung zum
        // Anschlag, ggf. an die tatsaechliche Mechanik anpassen (hier wie beim frueheren
        // Boot-Test willkuerlich negativ gewaehlt, da unbekannt, auf welcher Seite der Anschlag
        // liegt).
        ProcessHoming(homing1_active_, tmc_stepper1, stepper1_motion, register_model,
                      ModbusRegisters::Holding::STEPPER1_START_HOMING, /*homing_speed=*/-200);
        ProcessHoming(homing2_active_, tmc_stepper2, stepper2_motion, register_model,
                      ModbusRegisters::Holding::STEPPER2_START_HOMING, /*homing_speed=*/-200);

        // Waehrend eine Achse homt, muss das gemeinsame Enable an sein, unabhaengig vom
        // STEPPER_ENABLE-Register -- sonst kann der Treiber gar nicht fahren.
        bool enabled = register_model.GetHoldingRegister(ModbusRegisters::Holding::STEPPER_ENABLE) != 0
                       || homing1_active_ || homing2_active_;
        // TMC2209 ENN ist Low-aktiv (siehe tmc2209.cpp: "EN low = enabled") -- direkt an STEPPER_EN
        // verdrahtet (kein invertierender Pegelwandler im Schaltplan), also RESET=enabled/SET=disabled.
        HAL_GPIO_WritePin(STEPPER_EN_GPIO_Port, STEPPER_EN_Pin, enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
        if (enabled) {
            tmc_stepper1.enable();
            tmc_stepper2.enable();
        } else {
            tmc_stepper1.disable();
            tmc_stepper2.disable();
        }

        int32_t target1 = ((int32_t)register_model.GetHoldingRegister(ModbusRegisters::Holding::STEPPER1_TARGET_HI) << 16) |
                           register_model.GetHoldingRegister(ModbusRegisters::Holding::STEPPER1_TARGET_LO);
        int32_t target2 = ((int32_t)register_model.GetHoldingRegister(ModbusRegisters::Holding::STEPPER2_TARGET_HI) << 16) |
                           register_model.GetHoldingRegister(ModbusRegisters::Holding::STEPPER2_TARGET_LO);

        // Waehrend eine Achse homt, wird ihre normale Zielpositionsfahrt ausgesetzt (die
        // Homing-Bewegung laeuft chip-intern ueber VACTUAL, s. TMC2209::GenerateSteps() --
        // GotoPosition() wuerde sonst gleichzeitig ueber STEP/DIR gegensteuern wollen).
        if (enabled && !homing1_active_) {
            stepper1_motion.GotoPosition(target1);
        }
        if (enabled && !homing2_active_) {
            stepper2_motion.GotoPosition(target2);
        }

        int32_t pos1 = stepper1_motion.GetCurrentPosition();
        int32_t pos2 = stepper2_motion.GetCurrentPosition();
        register_model.SetInputRegister(ModbusRegisters::Input::STEPPER1_POSITION_HI, (uint16_t)((uint32_t)pos1 >> 16));
        register_model.SetInputRegister(ModbusRegisters::Input::STEPPER1_POSITION_LO, (uint16_t)((uint32_t)pos1 & 0xFFFF));
        register_model.SetInputRegister(ModbusRegisters::Input::STEPPER2_POSITION_HI, (uint16_t)((uint32_t)pos2 >> 16));
        register_model.SetInputRegister(ModbusRegisters::Input::STEPPER2_POSITION_LO, (uint16_t)((uint32_t)pos2 & 0xFFFF));

        register_model.SetInputRegister(ModbusRegisters::Input::STEPPER1_STATUS, compute_status(enabled, stepper1_motion, target1));
        register_model.SetInputRegister(ModbusRegisters::Input::STEPPER2_STATUS, compute_status(enabled, stepper2_motion, target2));
    }
};

inline StepperSetupAndLoop* StepperSetupAndLoop::instance_ = nullptr;

extern "C" inline void TIM17_IRQHandler(void) {
    if (TIM17->SR & TIM_SR_UIF) {
        TIM17->SR = (uint16_t)~TIM_SR_UIF;
        if (StepperSetupAndLoop::instance_) {
            StepperSetupAndLoop::instance_->HandleTim17UpdateInterrupt();
        }
    }
}

extern "C" inline void TIM16_IRQHandler(void) {
    if (TIM16->SR & TIM_SR_UIF) {
        TIM16->SR = (uint16_t)~TIM_SR_UIF;
        if (StepperSetupAndLoop::instance_) {
            StepperSetupAndLoop::instance_->HandleTim16UpdateInterrupt();
        }
    }
}
