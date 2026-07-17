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
// stepper_setup()).
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
// ============================================================================
#include "stepper_control.hpp"
#include "app_state.hpp"
#include "modbus_register_map.hpp"
#include "log.h"
#include "main.h"

#include "tmc2209.hpp"
#include "sigmoid_stepper.hh"
#include "sigmoid_acceleration_profile_100_1000_5_160mhz.hh"

extern "C" UART_HandleTypeDef huart5;

// Knotenadressen per MS1/MS2-Bruecken auf dem Board: Stepper1 = 0b10 (2), Stepper2 = 0b00 (0).
static tmc2209::TMC2209 tmc_stepper1(&huart5, 0b10, gpio::Pin::NO_PIN);
static tmc2209::TMC2209 tmc_stepper2(&huart5, 0b00, gpio::Pin::NO_PIN);

static SigmoidStepper<gpio::Peripheral::TIM17_> stepper1_motion(gpio::Pin::PA15, gpio::Pin::PD06, &profile_100_1000_5_160mhz);
static SigmoidStepper<gpio::Peripheral::TIM16_> stepper2_motion(gpio::Pin::PB04, gpio::Pin::PD07, &profile_100_1000_5_160mhz);

extern "C" void TIM17_IRQHandler(void) {
    if (TIM17->SR & TIM_SR_UIF) {
        TIM17->SR = (uint16_t)~TIM_SR_UIF;
        stepper1_motion.Handle_update_interrupt_();
    }
}

extern "C" void TIM16_IRQHandler(void) {
    if (TIM16->SR & TIM_SR_UIF) {
        TIM16->SR = (uint16_t)~TIM_SR_UIF;
        stepper2_motion.Handle_update_interrupt_();
    }
}

void stepper_setup() {
    if (!tmc_stepper1.InitForNormalSpeedAndUartBasedOperation()) {
        log_warn("TMC2209 stepper1: UART init failed - STEP/DIR pulses will still be generated, "
                 "but the driver may be running with unconfirmed microstep/current settings");
    }
    if (!tmc_stepper2.InitForNormalSpeedAndUartBasedOperation()) {
        log_warn("TMC2209 stepper2: UART init failed - STEP/DIR pulses will still be generated, "
                 "but the driver may be running with unconfirmed microstep/current settings");
    }

    stepper1_motion.Init();
    stepper2_motion.Init();

    NVIC_SetPriority(TIM17_IRQn, 5);
    NVIC_EnableIRQ(TIM17_IRQn);
    NVIC_SetPriority(TIM16_IRQn, 5);
    NVIC_EnableIRQ(TIM16_IRQn);
}

static uint16_t compute_status(bool enabled, const SigmoidStepper<gpio::Peripheral::TIM17_> &s, int32_t target) {
    uint16_t status = 0;
    if (enabled) status |= (1u << 0);
    if (s.IsMotionActive()) status |= (1u << 1);
    // Bit2 (Error/StallGuard) nicht verdrahtet -- kein DIAG-Pin/GSTAT-Polling implementiert.
    if (!s.IsMotionActive() && s.GetCurrentPosition() == target) status |= (1u << 3);
    return status;
}
static uint16_t compute_status(bool enabled, const SigmoidStepper<gpio::Peripheral::TIM16_> &s, int32_t target) {
    uint16_t status = 0;
    if (enabled) status |= (1u << 0);
    if (s.IsMotionActive()) status |= (1u << 1);
    if (!s.IsMotionActive() && s.GetCurrentPosition() == target) status |= (1u << 3);
    return status;
}

void stepper_update() {
    ModbusTcpServer& server = *g_app_state.modbus_server;

    bool enabled = server.read_holding_register(ModbusRegisters::Holding::STEPPER_ENABLE) != 0;
    HAL_GPIO_WritePin(STEPPER_EN_GPIO_Port, STEPPER_EN_Pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (enabled) {
        tmc_stepper1.enable();
        tmc_stepper2.enable();
    } else {
        tmc_stepper1.disable();
        tmc_stepper2.disable();
    }

    int32_t target1 = ((int32_t)server.read_holding_register(ModbusRegisters::Holding::STEPPER1_TARGET_HI) << 16) |
                       server.read_holding_register(ModbusRegisters::Holding::STEPPER1_TARGET_LO);
    int32_t target2 = ((int32_t)server.read_holding_register(ModbusRegisters::Holding::STEPPER2_TARGET_HI) << 16) |
                       server.read_holding_register(ModbusRegisters::Holding::STEPPER2_TARGET_LO);

    if (enabled) {
        stepper1_motion.GotoPosition(target1);
        stepper2_motion.GotoPosition(target2);
    }

    int32_t pos1 = stepper1_motion.GetCurrentPosition();
    int32_t pos2 = stepper2_motion.GetCurrentPosition();
    server.write_input_register(ModbusRegisters::Input::STEPPER1_POSITION_HI, (uint16_t)((uint32_t)pos1 >> 16));
    server.write_input_register(ModbusRegisters::Input::STEPPER1_POSITION_LO, (uint16_t)((uint32_t)pos1 & 0xFFFF));
    server.write_input_register(ModbusRegisters::Input::STEPPER2_POSITION_HI, (uint16_t)((uint32_t)pos2 >> 16));
    server.write_input_register(ModbusRegisters::Input::STEPPER2_POSITION_LO, (uint16_t)((uint32_t)pos2 & 0xFFFF));

    server.write_input_register(ModbusRegisters::Input::STEPPER1_STATUS, compute_status(enabled, stepper1_motion, target1));
    server.write_input_register(ModbusRegisters::Input::STEPPER2_STATUS, compute_status(enabled, stepper2_motion, target2));
}
