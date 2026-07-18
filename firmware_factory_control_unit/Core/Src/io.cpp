// ============================================================================
// IO Thread -- EIN gemeinsamer Thread fuer alle Sensoren/Aktoren (digitale Eingaenge,
// Drucksensor, Ventile, Foerderband/Kompressor-PWM, INFOLED, Ethernet-Link, USB-PD, Waegezelle,
// Stepper-Register, ToF/Farbsensor, CAN, WS2812), Zyklus ca. alle 50ms (IO_THREAD_SLEEP_TICKS).
//
// Vorher waren das zehn einzelne ThreadX-Threads (led/link/io/usb_pd/scale/stepper/tof_color/
// can/ws2812 + modbus_server); bis auf modbus_server (blockierender TCP-Accept-Loop, ein
// grundsaetzlich anderes Ausfuehrungsmodell) und app_main (einmaliger Boot-Orchestrator) sind
// die alle hier zusammengefasst -- keins der zusammengefassten Subsysteme blockiert nennenswert
// lange (HX711/FDCAN/WS2812 sind nicht-blockierendes Register-/DMA-Polling, ToF/Farbsensor
// nutzen kurze I2C-Timeouts, USB-PD's PowerSink.Loop() liefert nur Events aus einer eigenen,
// ISR-getriebenen Zustandsmaschine aus -- s. usb_pd_control.cpp) und teilt sich damit denselben
// Stack/Thread-Kontext ohne Nachteile bei deutlich weniger ThreadX-Verwaltungsoverhead
// (Kontextwechsel, je ein eigener Stack).
//
// Die STEP/DIR-Pulserzeugung der Stepper selbst laeuft weiterhin unabhaengig per
// Timer-Update-Interrupt (TIM16/TIM17), nicht in diesem Thread -- siehe setup_and_loops/stepper.hh.
// ============================================================================
#include "io.hpp"
#include "main.h"

extern "C" ADC_HandleTypeDef hadc1;
extern "C" TIM_HandleTypeDef htim4;

// Holding-Register sind 0..1000 Promille Duty -- CCR = (ARR+1) * permille / 1000.
static void set_pwm_duty_permille(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t permille) {
    if (permille > 1000) {
        permille = 1000;
    }
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = ((uint64_t)(arr + 1) * permille) / 1000;
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}

void Io::readInputs(uint32_t now) {
    (void)now;
    // Lichtschranken (NPN, idle-high -> aktiv = LOW).
    bool ls1_active = (HAL_GPIO_ReadPin(LIGHTBARRIER1_GPIO_Port, LIGHTBARRIER1_Pin) == GPIO_PIN_RESET);
    bool ls2_active = (HAL_GPIO_ReadPin(LIGHTBARRIER2_GPIO_Port, LIGHTBARRIER2_Pin) == GPIO_PIN_RESET);
    bool ls3_active = (HAL_GPIO_ReadPin(LIGHTBARRIER3_GPIO_Port, LIGHTBARRIER3_Pin) == GPIO_PIN_RESET);
    register_model.SetInputRegister(ModbusRegisters::Input::LIGHTBARRIER1, ls1_active ? 1 : 0);
    register_model.SetInputRegister(ModbusRegisters::Input::LIGHTBARRIER2, ls2_active ? 1 : 0);
    register_model.SetInputRegister(ModbusRegisters::Input::LIGHTBARRIER3, ls3_active ? 1 : 0);

    // Analoger Drucksensor -- Rohwert, physikalische Skalierung noch offen.
    // ADC1 laeuft im Continuous-Conversion-Modus (einmalig per HAL_ADC_Start() in main.c
    // gestartet, siehe dort) -- hier also kein Start/Poll/Stop je Durchlauf mehr, einfach den
    // zuletzt gewandelten Wert abholen.
    uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    register_model.SetInputRegister(ModbusRegisters::Input::PRESSURE_RAW, raw);
}

void Io::processMembers(uint32_t now) {
    info_led.Loop(now);
    can_setup_and_loop.Loop(now);
    eth_link.Loop(now);
    usb_pd_control.Loop();
    scale_.Loop(now);
    stepper_.Loop(now);
    tof_color_.Loop(now);
    ws2812_.Loop(now);
}

void Io::updateOutputs(uint32_t now) {
    // Pneumatikventile aus Holding-Registern setzen
    bool valve1 = register_model.GetHoldingRegister(ModbusRegisters::Holding::VALVE1) != 0;
    bool valve2 = register_model.GetHoldingRegister(ModbusRegisters::Holding::VALVE2) != 0;
    bool valve3 = register_model.GetHoldingRegister(ModbusRegisters::Holding::VALVE3) != 0;
    HAL_GPIO_WritePin(VALVE1_GPIO_Port, VALVE1_Pin, valve1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VALVE2_GPIO_Port, VALVE2_Pin, valve2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VALVE3_GPIO_Port, VALVE3_Pin, valve3 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Foerderband/Kompressor -- Holding-Register in 0..1000 Promille Duty.
    uint16_t conveyor_permille = register_model.GetHoldingRegister(ModbusRegisters::Holding::CONVEYOR_PWM);
    uint16_t compressor_permille = register_model.GetHoldingRegister(ModbusRegisters::Holding::COMPRESSOR_PWM);
    set_pwm_duty_permille(&htim4, TIM_CHANNEL_3, conveyor_permille);
    set_pwm_duty_permille(&htim4, TIM_CHANNEL_4, compressor_permille);

    // HealthState-Aggregation -- ETH_LINK_STATUS/SPEED/DUPLEX werden von eth_link.Loop() selbst
    // edge-getriggert geschrieben, hier nur der aktuelle Status fuer die Health-Bit-Berechnung.
    ULONG actual_status;
    bool eth_link_up = (nx_ip_interface_status_check(&ip_instance, 0,
                                                      NX_IP_LINK_ENABLED, &actual_status, 10) == NX_SUCCESS);
    uint16_t health = (1u << ModbusRegisters::Input::HEALTH_BIT_OK);
    if (!eth_link_up) {
        health |= (1u << ModbusRegisters::Input::HEALTH_BIT_ETH_LINK_DOWN);
        health &= ~(1u << ModbusRegisters::Input::HEALTH_BIT_OK);
    }
    register_model.SetInputRegister(ModbusRegisters::Input::HEALTH_STATE, health);
    register_model.SetInputRegister(ModbusRegisters::Input::TIMER_TICK, (uint16_t)(now & 0xFFFFu));
}

void Io::Setup() {
    uint32_t now = tx_time_get();
    info_led.Begin(now);
    info_led.AnimatePixel(now, &heartbeat_pattern_);
    eth_link.Setup();
    can_setup_and_loop.Setup();
    scale_.Setup();
    stepper_.Setup();
    tof_color_.Setup();
    ws2812_.Setup();
}

void Io::Loop() {
    while (true) {
        uint32_t now = tx_time_get();
        readInputs(now);
        processMembers(now);
        updateOutputs(now);

        tx_thread_sleep(IO_THREAD_SLEEP_TICKS);
    }
}
