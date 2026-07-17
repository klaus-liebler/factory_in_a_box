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
// Timer-Update-Interrupt (TIM16/TIM17), nicht in diesem Thread -- siehe stepper_control.cpp.
// ============================================================================
#include "io_thread.hpp"
#include "app_state.hpp"
#include "modbus_register_map.hpp"
#include "diagnostics.hpp"
#include "main.h"

#include "led_control.hpp"
#include "link_control.hpp"
#include "usb_pd_control.hpp"
#include "scale_control.hpp"
#include "stepper_control.hpp"
#include "tof_color_control.hpp"
#include "can_control.hpp"
#include "ws2812_control.hpp"

extern "C" ADC_HandleTypeDef hadc1;
extern "C" TIM_HandleTypeDef htim4;

// MX_TIM4_Init() (main.c, CubeMX-generiert) konfiguriert CH3/CH4 im reinen Output-Compare-
// "Timing"-Modus (kein Ausgang, siehe sConfigOC.OCMode=TIM_OCMODE_TIMING dort) -- fuer Motor-PWM
// muss das auf echten PWM-Modus 1 umgestellt werden. Passiert hier statt in main.c, weil eine
// CubeMX-Neuerzeugung main.c's generierten Teil sonst wieder auf TIMING zuruecksetzen wuerde;
// sauberer waere es, in CubeMX unter TIM4 CH3/CH4 direkt "PWM Generation" statt "Output Compare
// No Output" zu waehlen (dann wuerde MX_TIM4_Init() das schon richtig generieren) -- bis dahin
// tut es dieser Fixup bei jedem Boot.
static void configure_pwm_channel(TIM_HandleTypeDef *htim, uint32_t channel) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
    HAL_TIM_PWM_Start(htim, channel);
}

// Holding-Register sind 0..1000 Promille Duty -- CCR = (ARR+1) * permille / 1000.
static void set_pwm_duty_permille(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t permille) {
    if (permille > 1000) {
        permille = 1000;
    }
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = ((uint64_t)(arr + 1) * permille) / 1000;
    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}

extern "C" void io_setup() {
    // ADC1 laeuft im Continuous-Conversion-Modus (ContinuousConvMode=ENABLE in main.c) --
    // einmalig hier gestartet, laeuft danach im Hintergrund frei weiter; digital_io_update()
    // liest nur noch per HAL_ADC_GetValue(), kein Start/Stop/Poll je Durchlauf mehr.
    HAL_ADC_Start(&hadc1);

    configure_pwm_channel(&htim4, TIM_CHANNEL_3); // CONVEYOR_PWM, PD14
    configure_pwm_channel(&htim4, TIM_CHANNEL_4); // COMPRESSOR_PWM, PD15

    led_control_init();
    usb_pd_setup();
    scale_setup();
    stepper_setup();
    tof_color_setup();
    can_setup();
    ws2812_setup();
}

static void digital_io_update() {
    // g_app_state.modbus_server ist hier immer gesetzt: io_thread wird erst ganz am Ende von
    // app_main_thread_entry per tx_thread_resume() gestartet, lange nachdem modbus_server in
    // tx_application_define() zugewiesen wurde.
    ModbusTcpServer& server = *g_app_state.modbus_server;

    // Lichtschranken (NPN, idle-high -> aktiv = LOW).
    bool ls1_active = (HAL_GPIO_ReadPin(LIGHTBARRIER1_GPIO_Port, LIGHTBARRIER1_Pin) == GPIO_PIN_RESET);
    bool ls2_active = (HAL_GPIO_ReadPin(LIGHTBARRIER2_GPIO_Port, LIGHTBARRIER2_Pin) == GPIO_PIN_RESET);
    bool ls3_active = (HAL_GPIO_ReadPin(LIGHTBARRIER3_GPIO_Port, LIGHTBARRIER3_Pin) == GPIO_PIN_RESET);
    server.write_input_register(ModbusRegisters::Input::LIGHTBARRIER1, ls1_active ? 1 : 0);
    server.write_input_register(ModbusRegisters::Input::LIGHTBARRIER2, ls2_active ? 1 : 0);
    server.write_input_register(ModbusRegisters::Input::LIGHTBARRIER3, ls3_active ? 1 : 0);

    // Analoger Drucksensor -- Rohwert, physikalische Skalierung noch offen.
    // ADC1 laeuft im Continuous-Conversion-Modus (einmalig per HAL_ADC_Start()
    // in main.c gestartet, siehe dort) -- hier also kein Start/Poll/Stop je
    // Durchlauf mehr, einfach den zuletzt gewandelten Wert abholen.
    uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    server.write_input_register(ModbusRegisters::Input::PRESSURE_RAW, raw);

    // Pneumatikventile aus Holding-Registern setzen
    bool valve1 = server.read_holding_register(ModbusRegisters::Holding::VALVE1) != 0;
    bool valve2 = server.read_holding_register(ModbusRegisters::Holding::VALVE2) != 0;
    bool valve3 = server.read_holding_register(ModbusRegisters::Holding::VALVE3) != 0;
    HAL_GPIO_WritePin(VALVE1_GPIO_Port, VALVE1_Pin, valve1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VALVE2_GPIO_Port, VALVE2_Pin, valve2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(VALVE3_GPIO_Port, VALVE3_Pin, valve3 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Foerderband/Kompressor -- Holding-Register in 0..1000 Promille Duty.
    uint16_t conveyor_permille = server.read_holding_register(ModbusRegisters::Holding::CONVEYOR_PWM);
    uint16_t compressor_permille = server.read_holding_register(ModbusRegisters::Holding::COMPRESSOR_PWM);
    set_pwm_duty_permille(&htim4, TIM_CHANNEL_3, conveyor_permille);
    set_pwm_duty_permille(&htim4, TIM_CHANNEL_4, compressor_permille);

    // HealthState-Aggregation -- ETH_LINK_STATUS/SPEED/DUPLEX selbst werden von
    // link_status_update() edge-getriggert geschrieben, hier nur der aktuelle Status fuer die
    // Health-Bit-Berechnung.
    ULONG actual_status;
    bool eth_link_up = (nx_ip_interface_status_check(&g_app_state.ip_instance, 0,
                                                      NX_IP_LINK_ENABLED, &actual_status, 10) == NX_SUCCESS);
    Diagnostics::update_health_state(server, eth_link_up);
    Diagnostics::update_timer_tick(server, tx_time_get());
}

void io_thread_entry(ULONG arg) {
    (void)arg;

    while (1) {
        digital_io_update();
        led_control_update();
        link_status_update();
        usb_pd_update();
        scale_update();
        stepper_update();
        tof_color_update();
        can_update();
        ws2812_update();

        tx_thread_sleep(IO_THREAD_SLEEP_TICKS);
    }
}
