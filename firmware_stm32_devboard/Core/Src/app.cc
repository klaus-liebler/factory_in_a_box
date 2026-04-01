
#include "gpio.hh"
#include "log.h"
#include "main.h"
#include "single_led.hh"
#include "stm32g431xx.h"
#include "stm32g4xx_hal.h"

#include "stm32g4xx_hal_gpio.h"
#include "tmc2209.hpp"
#include <string_view>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "sigmoid_acceleration_profile_100_1000_5.hh"
#include "sigmoid_stepper.hh"

namespace pinsWeActCore {
constexpr std::string_view BOARD_NAME = "WeActCore Board v1.0";
constexpr gpio::Pin RESET = gpio::Pin::PG10;
constexpr gpio::Pin USB_VSENSE = gpio::Pin::PF01;


constexpr gpio::Pin STEPPER_TX = gpio::Pin::PB10;
constexpr gpio::Pin STEPPER_RX = gpio::Pin::PB11;
constexpr gpio::Pin STEPPER_EN = gpio::Pin::PB02;

constexpr gpio::Pin STEPPER1_DIR = gpio::Pin::PB00;
constexpr gpio::Pin STEPPER1_STEP = gpio::Pin::PB01;
constexpr gpio::Pin LED_PIN = gpio::Pin::PA08;
constexpr bool LED_ON_LEVEL = true; // LED is active high
constexpr int VOLTAGE_FROM_USBC = 5000; // default voltage if powered from USB-C

}; // namespace pinsWeActCore

// Namespace alias to allow easy switching between pin versions
namespace pins = pinsWeActCore;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi3;

// Motor controller instance
// RampStepper global instance with pins
tmc2209::TMC2209 g_motor(&huart3, 0, pins::STEPPER_EN);
SigmoidStepper g_ramp_stepper(pins::STEPPER1_STEP, pins::STEPPER1_DIR,
                              &profile_100_1000_5);

/**
 * UART RX complete callback - called by HAL
 * This needs to be in the main.c or called from there
 */
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    // Process the received byte
    // uint8_t rx_byte = cmd_buffer[0];
    // process_uart_data(rx_byte);

    // Re-enable receive for next byte
    // HAL_UART_Receive_IT(&huart2, cmd_buffer, 1);
  }
}

extern "C" void TIM1_TRG_COM_TIM17_IRQHandler(void) {
  if (TIM17->SR & TIM_SR_UIF) {
    TIM17->SR &= ~TIM_SR_UIF;
    g_ramp_stepper.HandleUpdateInterrupt_();
  }
}

single_led::M<pins::LED_ON_LEVEL> led(pins::LED_PIN);
single_led::BlinkPattern blink_pattern(200, 800);


extern "C" void app_setup(void) {

  // Initialize GPIO
  gpio::Gpio::ConfigureGPIOOutput(pins::LED_PIN, false);
  gpio::Gpio::ConfigureGPIOOutput(pins::TMC2209_EN,
                                  true); // Disable TMC2209 driver
  HAL_Delay(50); // Short delay to ensure driver is disabled before changing EN pin state
  gpio::Gpio::ConfigureGPIOOutput(pins::STEPPER1_DIR,
                                  false); // Set initial direction for motor
  gpio::Gpio::ConfigureGPIOOutput(pins::STEPPER1_STEP,
                                  false); // Initialize step pin low


  // Initialize ramp stepper subsystem
  g_ramp_stepper.Init();
  // Init USB PD sink
  PowerSink.start(handleEvent);
  led.AnimatePixel(HAL_GetTick(), &blink_pattern);
  while(PowerSink.activeVoltage < 15000) {
    PowerSink.loop();
    led.Loop(current_tick);
    HAL_Delay(1);
  }
    
  log_info("USB-PD target voltage reached.");
  if (!g_motor.InitForNormalSpeedAndUartBasedOperation()) {
    log_error("Failed to initialize motor after USB-PD voltage became available");
  }
  log_info("Motor initialized successfully");
  g_motor.PrintPrettyFullSystemState();
  log_info("Starting motor at %d full steps/s for bring-up", kMotorStartupSpeedFullStepsPerSecond);
  if (!g_motor.GenerateSteps(400)) {
    log_error("Failed to start motor after initialization");
  }
}

uint32_t lastDebugOutput = 0;
// Loop-Funktion: im Hauptloop aufgerufen
extern "C" void app_loop(void) {
  uint32_t current_tick = HAL_GetTick();
  led.Loop(current_tick);
  if (current_tick - lastDebugOutput > 1000) {
    log_info("app_loop is running. Current Stepper Position: %ld. Tick: %lu",
             g_ramp_stepper.GetCurrentPosition(), current_tick);
    lastDebugOutput = current_tick;
  }
  HAL_Delay(10);
}
