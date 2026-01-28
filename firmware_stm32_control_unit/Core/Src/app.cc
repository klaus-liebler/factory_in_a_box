#include "log.h"
#include "main.h"
#include <cmath>
#include <cstring>
#include <cstdio>
#include "TMC2209.hpp"
#include "gpio.hh"
#include "single_led.hh"
#include "log.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"
#include "sigmoid_stepper.hh"

namespace pins {
    constexpr gpio::Pin TMC2209_EN = gpio::Pin::PA08;
    constexpr gpio::Pin TMC2209_DIR = gpio::Pin::PB05;
    constexpr gpio::Pin STEPPER1_STEP = gpio::Pin::PB00;
    constexpr gpio::Pin STEPPER2_STEP = gpio::Pin::PB01;
    constexpr gpio::Pin LED_PIN = gpio::Pin::PB12;
};

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

// Motor controller instance
static tmc2209::TMC2209* g_motor = nullptr;

// RampStepper global instance with pins
static SigmoidStepper g_ramp_stepper(pins::STEPPER1_STEP, pins::TMC2209_DIR);



// Private variables
static uint32_t last_tick = 0;




/**
 * UART RX complete callback - called by HAL
 * This needs to be in the main.c or called from there
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        // Process the received byte
        //uint8_t rx_byte = cmd_buffer[0];
        //process_uart_data(rx_byte);
        
        // Re-enable receive for next byte
        //HAL_UART_Receive_IT(&huart2, cmd_buffer, 1);
    }
}

extern "C" void TIM1_TRG_COM_TIM17_IRQHandler(void) {
    if (TIM17->SR & TIM_SR_UIF) {
        TIM17->SR &= ~TIM_SR_UIF;
        g_ramp_stepper.handle_update_interrupt_();
    }
}

single_led::M<false> led(pins::LED_PIN);
single_led::BlinkPattern blink_pattern(200, 800);

extern "C" void app_setup(void) {
    printf("\r\n\r\n\r\n");
    log_info("FacoryInABox Controller Application Starting...");
    
    // Initialize GPIO
    gpio::Gpio::ConfigureGPIOOutput(pins::LED_PIN, false);
    gpio::Gpio::ConfigureGPIOOutput(pins::TMC2209_EN, true); // Enable TMC2209 driver
    gpio::Gpio::ConfigureGPIOOutput(pins::TMC2209_DIR, false); // Set initial direction for motor
    gpio::Gpio::ConfigureGPIOOutput(pins::STEPPER1_STEP, false); // Initialize step pin low
    gpio::Gpio::ConfigureGPIOOutput(pins::STEPPER2_STEP, false); // Initialize step pin low

    // Initialize ramp stepper subsystem
    g_ramp_stepper.init();


    
    // Create motor controller instance
    g_motor = new tmc2209::TMC2209(&huart3, 0, pins::TMC2209_EN);
    
    // Initialize motor
    if (g_motor->initForNormalSpeedAndUartBasedOperation(false)) {
        log_info("Motor initialized successfully");
    } else {
        log_error("Failed to initialize motor");
    }
    g_motor->printPrettyFullSystemState();
    g_motor->performStealthChopAutoTuningForQuietOperation();
    
    
    led.AnimatePixel(HAL_GetTick(), &blink_pattern);
    
    last_tick = HAL_GetTick();
}

// Loop-Funktion: im Hauptloop aufgerufen
extern "C" void app_loop(void) {
    uint32_t current_tick = HAL_GetTick();
    led.Loop(current_tick);
    HAL_Delay(20);
}
